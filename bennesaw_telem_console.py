'''
@Author: Sophia Smith, Bo Han Zhu
@Date: 4/23/2022
@Description: HyTech live telemetry console
'''


import PySimpleGUI as sg
import sys
import time
import threading
from os import path
from enum import Enum
from datetime import datetime
import paho.mqtt.client as mqtt
import itertools
import binascii
import struct
import sys
import argparse
import serial
import glob
from parser_api import *
from decimal import Decimal
from embed_plot_test import *
matplotlib.use('TkAgg')
__file__ = sys.path[0]

parser = argparse.ArgumentParser(description="Telemetry console")
parser.add_argument('--mode', '-m', action='store', default='0',
                    required=False, help="Mode for parser to run in")
parser.add_argument('--font', '-f', action='store',
                    default='Comic Sans', required=False, help="Font to use")
parser.add_argument('--title_size', '-ts', action='store',
                    default='14', required=False, help="Title font")
parser.add_argument('--body_size', '-bs', action='store',
                    default='10', required=False, help='Body font size')

args = parser.parse_args()


class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def fastreadline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

# Connection type definitions


class ConnectionType(Enum):
    SERVER = 0
    TEENSY = 1
    TEST_CSV = 2
    UNKNOWN = 3


# AWS/MQTT Connection Definitions
MQTT_SERVER = "ec2-3-134-2-166.us-east-2.compute.amazonaws.com"
MQTT_PORT = 1883
MQTT_TOPIC = 'hytech_car/telemetry'

# Set this to whatever the script is running
# @TODO: Make a screen at the beginning to let the user choose the type

CONNECTION = {'0': ConnectionType.SERVER.value,
              '1': ConnectionType.TEENSY.value,
              '2': ConnectionType.TEST_CSV.value,
              '3': ConnectionType.UNKNOWN.value}[args.mode]

DICT = {
    "DASHBOARD": {
        "SSOK_ABOVE_THRESHOLD": " ",
        "SHUTDOWN_H_ABOVE_THRESHOLD": " "
    },
    "BATTERY_MANAGEMENT_SYSTEM": {
        "Pack_Open_Voltage": " ",
        "Pack_Inst_Voltage": " ",
        "Pack_Summed_Voltage": " ",
        "Average_Temperature": " ",
        "Low_Temperature": " ",
        "High_Temperature": " ",
        "Pack_Current": " ",
        "Pack_DCL": " ",
        "Pack_CCL": " "

    },
    "ENERGY_METER": {
        "VOLTAGE": " ",
        "CURRENT": " ",
        "OVERPOWER": " ",
        "OVERVOLTAGE": " ",
        "LOGGING": " ",
        "VOLTAGE_GAIN": " ",
        "CURRENT_GAIN": " ",
    },
    "RACEGRADE_IMU": {
        "LAT_ACCEL": " ",
        "LONG_ACCEL": " ",
        "VERT_ACCEL": " ",
        "YAW": " ",
        "PITCH": " ",
        "ROLL": " "
    },
    "RMS_INVERTER": {
        "OUTPUT_POWER": " ",
        "D3_Power_On_Timer": " ",
        "Inverter_Enable": " ",
        "D1_Motor_Angle_Electrical": " ",
        "D3_Electrical_Output_Frequency": " ",
        "D1_Commanded_Torque": " ",
        "D2_Torque_Feedback": " ",
        "D1_DC_Bus_Voltage": " ",
        "D2_Output_Voltage": " ",
        "D3_VAB_Vd_Voltage": " ",
        "D4_VBC_Vq_Voltage": " ",
        "D4_DC_Bus_Current": " ",
        "D1_Phase_A_Current": " ",
        "D2_Phase_B_Current": " ",
        "D3_Phase_C_Current": " ",
        "D3_Motor_Temperature": " ",
        "D4_Gate_Driver_Board": " ",
        "D1_Module_A": " ",
        "D2_Module_B": " ",
        "D3_Module_C": " ",
        "D4_Torque_Shudder": " ",
        "D2_Inverter_State": " ",
        "D1_VSM_State": " ",
        "D4_Inverter_Discharge_State": " ",
        "D5_Inverter_Command_Mode": " ",
        "D7_Direction_Command": " ",
        "D1_Post_Fault_Lo": " ",
        "D2_Post_Fault_Hi": " ",
        "D3_Run_Fault_Lo": " ",
        "D4_Run_Fault_Hi": " ",
    },
    "MAIN_ECU": {
        "GLV_BATTERY_VOLTAGE": " ",
        "ECU_CURRENT": " ",
        "COOLING_CURRENT": " ",
        "TEMPERATURE": " ",
        "IMD_OK_HIGH": " ",
        "BMS_OK_HIGH": " ",
        "BSPD_OK_HIGH": " ",
        "SOFTWARE_OK_HIGH": " ",
        "INVERTER_POWERED": " ",
        "TORQUE_MODE": " ",
        "MAX_TORQUE": " ",
        "Torque_Command": " ",
        "APPS1": " ",
        "APPS2": " ",
        "NO_ACCEL_IMPLAUSIBILITY": " ",
        "BSE1": " ",
        "STEERING": " ",
        "BRAKE_PEDAL_ACTIVE": " ",
        "NO_BRAKE_IMPLAUSIBILITY": " ",
        "LAUNCH_CTRL_ACTIVE": " ",
        # tests from here
        "SHUTDOWN_STATES": " ",
        "PEDAL_STATES": " ",
        "ECU_STATES": " ",
        "MAX_TORQUE": " ",
        "TORQUE_MODE": " ",
        "DISTANCE_TRAVELLED": " ",

    },
    "SENSOR_ACQUISITION_BOARD": {
        "COOLING_LOOP_FLUID_TEMP": " ",
        "AMB_AIR_TEMP": " ",
        "FL_shockpot": " ",
        "FR_shockpot": " ",
        "RL_shockpot": " ",
        "RR_shockpot": " ",
    },
    "WHEEL_SPEED_SENSORS": {
        "D2_Motor_Speed": " ",
        "RPM_FL": " ",
        "RPM_FR": " "
    },
}

# Variables to keep track of inverter current and power for inverter power calculation
inverter_voltage = 0.0
inverter_current = 0.0
inverter_power = 0.0
ALPHA = 0.95  # for filtering

'''
@brief: Helper function to search for keys in a nested dictionary
@reference: https://stackoverflow.com/questions/49662970/how-can-i-search-for-specific-keys-in-this-nested-dictionary-in-python
@param[in]: k - the key to search
@param[in]: d - the (nested) dictionary
@param[out]: true if found, false if not
'''


def recursive_lookup(k, d):
    if k in d:
        return d[k]
    for v in d.values():
        if isinstance(v, dict):
            a = recursive_lookup(k, v)
            if a is not None:
                return a
    return None


'''
@brief: Helper function to calculate inverter power if inverter voltage or current is updated
@param[in]: name - name of the parsed label
@param[in]: data - the parsed data; if used, it will be a floating-type value
@param[in]: window - the PySimpleGUI window object
'''


def handle_inverter_power(name, data, window):
    # Specially handle Inverter output power since it is not a CAN label and needs calculation
    if name == "Pack_Current" or name == "D1_DC_Bus_Voltage":
        if name == "D1_DC_Bus_Voltage":
            global inverter_voltage

            inverter_voltage = int(Decimal(data))
        else:
            global inverter_current
            inverter_current = int(Decimal(data))

        # Power = voltage * current
        # Apply filtering constant so changes are not so volatile
        global inverter_power
        inverter_power = round(
            ALPHA * inverter_power + (1.0 - ALPHA) * inverter_current * inverter_voltage, 2)

        window.write_event_value(
            "-Update Data-", ["OUTPUT_POWER", "OUTPUT POWER: " + str(inverter_power) + " W"])


def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def user_prompt(prompt, options):
    layoutt = [[sg.Text(prompt)],     # Part 2 - The Layout
               [sg.Text(options)],
               [sg.Input()],
               [sg.Button('Ok')]]
    # Create the window
    # Part 3 - Window Defintion
    windoww = sg.Window('KS5e Parser', layoutt)

    # Display and interact with the Window

    # Part 4 - Event loop or Window.read call
    event, values = windoww.read()
    # Finish up by removing from the screen
    windoww.close()
    return values[0]


def read_from_teensy_thread(window, comport):
    db = get_dbc_files()
    dbc_ids = print_all_the_shit_in_dbc_file(db)
    unknown_ids = []
    ser = serial.Serial()
    ser.port = comport  # Arduino serial port
    ser.baudrate = 256000
    ser.timeout = .1  # specify timeout when using readline()
    ser.open()
    rl = ReadLine(ser)
    if ser.is_open == True:
        print("\nAll right, serial port now open. Configuration:\n")
        print(ser, "\n")  # print serial parameters
    else:
        sys.exit("Opening serial failed")
    window.write_event_value("-Connection Success-", "good job!")
    while True:
        while (ser.in_waiting > 0):
            # print(ser.in_waiting)

            line = ser.readline()
            # print(line)
            line = line.replace(b'\n', b'')
            raw_id = (line.split(b',')[0]).decode()
            raw_message = (line.split(b',')[1]).decode()
            length = 8
            # raw_message = raw_message[:(int(length) * 2)] # Strip trailing end of line/file characters that may cause bad parsing
            # raw_message = raw_message.zfill(16) # Sometimes messages come truncated if 0s on the left. Append 0s so field-width is 16.
            table = parse_message(raw_id, raw_message,
                                  db, dbc_ids, unknown_ids)
            # print(table)
            if table != "INVALID_ID" and table != "UNPARSEABLE":
                for i in range(len(table[1])):
                    name = table[1][i]
                    data = table[2][i]
                    units = table[3][i]
                    if name == "MCU_STATE":
                        window.write_event_value(
                            "-MCU State Change-", [data.replace("_", " ")])
                    elif recursive_lookup(name, DICT):
                        #print("found a thing\n\tname: " + name + "\n\tdata: "+data + "\n\tunits: " + units)
                        window.write_event_value(
                            "-Update Data-", [name, name.replace("_", " ") + ": " + str(data) + " " + units])
                        handle_inverter_power(name, data, window)


'''
@brief: Thread to read raw CSV line, parse it, and send event to GUI if match 
        Sends event to close GUI upon CSV read completion
        Requires a raw data CSV in the current directory with the name raw_data.csv
@param[in]: window - the PySimpleGUI window object
'''


def read_from_csv_thread(window):
    db = get_dbc_files()
    dbc_ids = print_all_the_shit_in_dbc_file(db)
    unknown_ids = []
    infile = open("raw_data.csv", "r")
    line_count = 1  # bypass first header line
    raw_data_lines = infile.readlines()
    window.write_event_value("-Test Connection Success-", "good job!")

    while line_count < len(raw_data_lines):
        print("Linecount: " + str(line_count))
        raw_id = raw_data_lines[line_count].split(",")[1]
        length = raw_data_lines[line_count].split(",")[2]
        raw_message = raw_data_lines[line_count].split(",")[3]
        # Strip trailing end of line/file characters that may cause bad parsing
        raw_message = raw_message[:(int(length) * 2)]
        # Sometimes messages come truncated if 0s on the left. Append 0s so field-width is 16.
        raw_message = raw_message.zfill(16)
        table = parse_message(raw_id, raw_message, db, dbc_ids, unknown_ids)
        # print(table)
        if table != "INVALID_ID" and table != "UNPARSEABLE":
            for i in range(len(table[1])):
                name = table[1][i]
                data = table[2][i]
                units = table[3][i]
                if name == "MCU_STATE":
                    window.write_event_value(
                        "-MCU State Change-", [data.replace("_", " ")])
                elif recursive_lookup(name, DICT):
                    #print("found a thing\n\tname: " + name + "\n\tdata: "+data + "\n\tunits: " + units)
                    window.write_event_value(
                        "-Update Data-", [name, name.replace("_", " ") + ": " + str(data) + " " + units])
                    handle_inverter_power(name, data, window)

        line_count += 1

    window.write_event_value("-Read CSV Done-", "No data for you left")


'''
@brief: Thread to connect to MQTT broker on AWS EC2 instance.
        Parses incoming messages and packages them as an event to the GUI if match.
@param[in]: window - the PySimpleGUI window object
'''

'''
@brief: Helper function to get multi-columns rows for detailed messages
@return: dictionary, an list of combined elements 
'''


def get_bms_detailed_messages():
    ic_list = ['IC_' + str(x) for x in range(8)]
    cell_list_odd = ['CELL_' + str(x) for x in range(9)]
    cell_list_even = ['CELL_' + str(x) for x in range(12)]
    ic_cells = []

    for i in range(8):
        # Even ICs have 12 cells
        if i % 2 == 0:
            ic_cells += [ic_list[i] + '_' + cell for cell in cell_list_even]
        # Odd ICs have 9 cells
        else:
            ic_cells += [ic_list[i] + '_' + cell for cell in cell_list_odd]

    result = dict.fromkeys(ic_cells, ' ')
    dictionary = {"BATTERY_MANAGEMENT_SYSTEM_DETAILED_VOLTAGES": result}

    ic_list2 = ['IC_' + str(x) for x in range(8)]
    temperature_list = ['THERM_' + str(x) for x in range(4)]
    ic_temperature = [ic + '_' +
                      therm for ic in ic_list2 for therm in temperature_list]
    ic_temperature = ic_temperature + \
        ["IC_0_HUMIDITY", "IC_2_HUMIDITY", "IC_4_HUMIDITY", "IC_6_HUMIDITY"]
    ic_temperature = ic_temperature + \
        ["IC_1_TEMPERATURE", "IC_3_TEMPERATURE",
            "IC_5_TEMPERATURE", "IC_7_TEMPERATURE"]
    result2 = dict.fromkeys(ic_temperature, ' ')
    dictionary2 = {"BATTERY_MANAGEMENT_SYSTEM_DETAILED_TEMPERATURES": result2}
    return dictionary, dictionary2


fig = matplotlib.figure.Figure(figsize=(5, 4), dpi=100)
t = np.arange(0, 3, .01)
fig.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t))

'''
@brief: The main function to spawn the PySimpleGUI and handle events
'''


def main():
    sg.change_look_and_feel("Black")
    title_font = (args.font, int(args.title_size))
    text_font = (args.font, int(args.body_size))

    # Subtitle text declarations
    inverter = [[sg.Text("RMS INVERTER", pad=(
        0, 2), font=title_font, text_color="light blue")]]
    dashboard = [
        [sg.Text("DASHBOARD", pad=(0, 2), font=title_font, text_color="light blue")]]
    bms = [[sg.Text("BMS OVERVIEW", pad=(0, 2),
                    font=title_font, text_color="light blue")]]
    main_ecu = [
        [sg.Text("MAIN ECU", pad=(0, 2), font=title_font, text_color="light blue")]]
    wheel_speed_sensors = [[sg.Text("WHEEL SPEED SENSORS", pad=(
        0, 2), font=title_font, text_color="light blue")]]
    sab = [[sg.Text("SENSOR ACQUISITION BOARD", pad=(0, 2),
                    font=title_font, text_color="light blue")]]
    imu = [[sg.Text("RACEGRADE IMU", pad=(0, 2),
                    font=title_font, text_color="light blue")]]
    em = [[sg.Text("ENERGY METER", pad=(0, 2),
                   font=title_font, text_color="light blue")]]
    bms_detailed_voltages = [[sg.Text("BMS DETAILED VOLTAGES", size=(
        33, 1), pad=(0, 2), font=title_font, text_color="light blue")]]
    bms_detailed_temps = [[sg.Text("BMS DETAILED TEMPERATURES", pad=(
        0, 2), font=title_font, text_color="light blue")]]

    bms_voltages = [[]]
    bms_temperatures = [[]]

    DICT1, DICT2 = get_bms_detailed_messages()
    DICT.update(DICT1)
    DICT.update(DICT2)
    row_count_temperatures = 0
    row_count_voltages = 0

    # Data text arrangements and manipulations
    for label, value in DICT["RMS_INVERTER"].items():
        inverter.append([sg.Text(label.replace("_", " ") + ": " + value,
                        justification="left", size=(40, 1), pad=(0, 0), font=text_font, key=label)])
    for label, value in DICT["BATTERY_MANAGEMENT_SYSTEM"].items():
        bms.append([sg.Text(label.replace("_", " ") + ": " + value,
                   justification="left", size=(35, 1), pad=(0, 0), font=text_font, key=label)])
    for label, value in DICT["MAIN_ECU"].items():
        main_ecu.append([sg.Text(label.replace("_", " ") + ": " + value,
                        justification="left", size=(35, 1), pad=(0, 0), font=text_font, key=label)])
    for label, value in DICT["DASHBOARD"].items():
        dashboard.append([sg.Text(label.replace("_", " ") + ": " + value,
                         justification="left", size=(35, 1), pad=(0, 0), font=text_font, key=label)])
    for label, value in DICT["WHEEL_SPEED_SENSORS"].items():
        wheel_speed_sensors.append([sg.Text(label.replace(
            "_", " ") + ": " + value, justification="left", size=(35, 1), pad=(0, 0), font=text_font, key=label)])
    for label, value in DICT["SENSOR_ACQUISITION_BOARD"].items():
        sab.append([sg.Text(label.replace("_", " ") + ": " + value,
                   justification="left", size=(35, 1), pad=(0, 0), font=text_font, key=label)])
    for label, value in DICT["RACEGRADE_IMU"].items():
        imu.append([sg.Text(label.replace("_", " ") + ": " + value,
                   justification="left", size=(35, 1), pad=(0, 0), font=text_font, key=label)])
    for label, value in DICT["ENERGY_METER"].items():
        em.append([sg.Text(label.replace("_", " ") + ": " + value,
                  justification="left", size=(35, 1), pad=(0, 0), font=text_font, key=label)])
    for label, value in DICT["BATTERY_MANAGEMENT_SYSTEM_DETAILED_VOLTAGES"].items():
        if row_count_voltages % 9 == 8 and int(label[3]) == 3 or int(label[3]) == 7:
            # No padding for ICs 3 and 7 last cells since text will take care of it in the next column over
            bms_voltages.append([sg.Text(label.replace("_", " ") + ": " + value,
                                justification="left", size=(23, 1), pad=(0, 0), font=text_font, key=label)])
            row_count_voltages = 0
        elif row_count_voltages % 9 == 8 and int(label[3]) % 2 == 1:
            bms_voltages.append([sg.Text(label.replace("_", " ") + ": " + value, justification="left",
                                size=(23, 1), pad=((0, 0), (0, 10)), font=text_font, key=label)])
            row_count_voltages = 0
        elif row_count_voltages % 12 == 11 and int(label[3]) % 2 == 0:
            bms_voltages.append([sg.Text(label.replace("_", " ") + ": " + value, justification="left",
                                size=(23, 1), pad=((0, 0), (0, 10)), font=text_font, key=label)])
            row_count_voltages = 0
        else:
            bms_voltages.append([sg.Text(label.replace("_", " ") + ": " + value,
                                justification="left", size=(23, 1), pad=(0, 0), font=text_font, key=label)])
            row_count_voltages = row_count_voltages + 1
    for label, value in DICT["BATTERY_MANAGEMENT_SYSTEM_DETAILED_TEMPERATURES"].items():
        if row_count_temperatures >= 36:
            bms_temperatures.append([sg.Text(label.replace(
                "_", " ") + ": " + value, justification="left", size=(25, 1), pad=(0, 0), font=text_font, key=label)])
        elif row_count_temperatures % 4 == 3:
            bms_temperatures.append([sg.Text(label.replace(
                "_", " ") + ": " + value, justification="left", size=(23, 1), pad=((0, 0), (0, 10)), font=text_font, key=label)])
        else:
            bms_temperatures.append([sg.Text(label.replace(
                "_", " ") + ": " + value, justification="left", size=(23, 1), pad=(0, 0), font=text_font, key=label)])
        row_count_temperatures = row_count_temperatures + 1

    # We ran out of room so ICs 3 and 7 will be on column with BMS detailed temps
    left_voltages_first_column = bms_voltages[:34]
    right_voltages_first_column = bms_voltages[43:76]
    left_voltages_second_column = bms_voltages[34:43]
    right_voltages_second_column = bms_voltages[76:]

    first_half_therm = bms_temperatures[:17]
    second_half_therm = bms_temperatures[17:33]
    therm_humidities = bms_temperatures[33:37]
    therm_temperatures = bms_temperatures[37:]

    voltages = [[sg.Column(left_voltages_first_column, pad=(0, 0), vertical_alignment='t'), sg.Column(
        right_voltages_first_column, pad=(0, 0), vertical_alignment='t')]]
    voltages_second_column = [[sg.Column(left_voltages_second_column, pad=(0, 0), vertical_alignment='t'), sg.Column(
        right_voltages_second_column, pad=(0, 0), vertical_alignment='t')]]
    temperatures = [[sg.Column(first_half_therm + therm_humidities, pad=(0, 0), vertical_alignment='t'),
                     sg.Column(second_half_therm + therm_temperatures, pad=(0, 0), vertical_alignment='t')]]

    # Header texts and columns
    connection_text = [[sg.Text("CONSOLE STATUS: NOT CONNECTED", justification="left", pad=(
        (5, 0), 12), text_color='red', font=title_font, key="-Connection Text-")]]
    divider_text_1 = [[sg.Text(" | ", pad=(5, 12), font=title_font)]]
    vehicle_status_text = [[sg.Text("VEHICLE STATUS: NOT RECEIVED", justification="left", pad=(
        (0, 0), 12), font=title_font, key="-Vehicle Status Text-")]]
    divider_text_2 = [[sg.Text(" | ", pad=(5, 12), font=title_font)]]
    last_update_text = [[sg.Text("LAST UPDATE: NOT RECEIVED", justification="left", pad=(
        (0, 5), 12), font=title_font, key="-Last Update Text-")]]

    status_header_column1 = sg.Column(
        connection_text, pad=(0, 0), vertical_alignment='t')
    status_header_column2 = sg.Column(
        divider_text_1, pad=(0, 0), vertical_alignment='t')
    status_header_column3 = sg.Column(
        vehicle_status_text, pad=(0, 0), vertical_alignment='t')
    status_header_column4 = sg.Column(
        divider_text_2, pad=(0, 0), vertical_alignment='t')
    status_header_column5 = sg.Column(
        last_update_text, pad=(0, 0), vertical_alignment='t')

    # Data colummns
    column1 = sg.Column(dashboard + [[sg.Text(" ", size=(35, 1), pad=(0, 0), font=text_font)]] + bms + [[sg.Text(" ", size=(35, 1), pad=(
        0, 0), font=text_font)]] + em + [[sg.Text(" ", size=(35, 1), pad=(0, 0), font=text_font)]] + imu, vertical_alignment='t')
    column2 = sg.Column(main_ecu + [[sg.Text(" ", size=(35, 1), pad=(0, 0), font=text_font)]] + sab + [
                        [sg.Text(" ", size=(35, 1), pad=(0, 0), font=text_font)]] + wheel_speed_sensors, vertical_alignment='t')
    column3 = sg.Column(inverter, vertical_alignment='t')
    column4 = sg.Column(bms_detailed_voltages +
                        voltages, vertical_alignment='t')
    column5 = sg.Column(voltages_second_column + [[sg.Text(" ", size=(35, 1), pad=(
        0, 0), font=text_font)]] + bms_detailed_temps + temperatures, vertical_alignment='t')

    # Finalize layout
    layout = [[status_header_column1, status_header_column2, status_header_column3, status_header_column4,
               status_header_column5], [column1, column2, column3, sg.Canvas(key='-CANVAS-')]]

    window = sg.Window("KSU Motorsports Live Telemetry Console",
                       resizable=True).Layout(layout).Finalize()
    fig_canvas_agg = draw_figure(window['-CANVAS-'].TKCanvas, fig)

    # window.Maximize()
    CONNECTION = int(user_prompt("Enter Connection Type",
                     "SERVER=0, TEENSY=1, TEST_CSV=2"))
    # Choose messaging thread based on connection type
    if CONNECTION == ConnectionType.SERVER.value:
        sys.exit("Invalid connection source selection. Terminating script")
    elif CONNECTION == ConnectionType.TEENSY.value:
        comport = user_prompt("Enter Teensy COM port", serial_ports())
        thread = threading.Thread(target=read_from_teensy_thread, args=[
                                  window, comport], daemon=True)
    elif CONNECTION == ConnectionType.TEST_CSV.value:
        thread = threading.Thread(
            target=read_from_csv_thread, args=[window], daemon=True)
    else:
        sys.exit("Invalid connection source selection. Terminating script")
    #thread = threading.Thread(target=read_from_csv_thread, args=[window], daemon=True)

    thread.start()

    # Event Loop
    while True:
        event, values = window.read()

        if event in (sg.WIN_CLOSED, "Quit"):
            break
        elif event == "-Read CSV Done-":
            thread.join(timeout=0)
            break
        elif event == "-Test Connection Success-":
            window["-Connection Text-"].update(
                "CONSOLE STATUS: TESTING", text_color="yellow")
        elif event == "-Connection Success-":
            window["-Connection Text-"].update(
                "CONSOLE STATUS: CONNECTED", text_color="green")
        elif event == "-MCU State Change-":
            received_status = values["-MCU State Change-"][0]
            status_color = ""

            if received_status == "STARTUP":
                status_color = "cyan"
            elif received_status == "TRACTIVE SYSTEM NOT ACTIVE":
                status_color = "light grey"
            elif received_status == "TRACTIVE SYSTEM ACTIVE":
                status_color = "orange"
            elif received_status == "ENABLING INVERTER":
                status_color = "yellow"
            elif received_status == "WAITING READY TO DRIVE SOUND":
                status_color = "green yellow"
            elif received_status == "READY TO DRIVE":
                status_color = "green"
            elif received_status == "UNRECOGNIZED STATE":
                status_color = "red"
            else:
                # Should not get here since parser will output UNRECOGNIZED STATE if invalid; here just as a failsafe
                status_color = "red"

            window["-Vehicle Status Text-"].update(
                "VEHICLE STATUS: " + received_status, text_color=status_color)
            window.refresh()
        elif event == "-Update Data-":
            window[values["-Update Data-"][0]
                   ].update(values["-Update Data-"][1])
            window["-Last Update Text-"].update(
                "LAST UPDATE: " + datetime.now().strftime('%H:%M:%S.%f')[:-5])
            window.refresh()

    window.close()


############################
# Entry point to application
############################
main()
