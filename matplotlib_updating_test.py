from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import PySimpleGUI as sg
import matplotlib, time, threading
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np


def fig_maker(window): # this should be called as a thread, then time.sleep() here would not freeze the GUI
    plt.scatter(np.random.rand(1,10),np.random.rand(1,10))
    window.write_event_value('-THREAD-', 'done.')
    time.sleep(1)
    return plt.gcf()


def draw_figure(canvas, figure, loc=(0, 0)):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg


def delete_fig_agg(fig_agg):
    fig_agg.get_tk_widget().forget()
    plt.close('all')


if __name__ == '__main__':
    # define the window layout
    layout = [[sg.Button('update'), sg.Button('Stop', key="-STOP-"), sg.Button('Exit', key="-EXIT-")],
              [sg.Radio('Keep looping', "RADIO1", default=True, size=(12,3),key="-LOOP-"),sg.Radio('Stop looping', "RADIO1", size=(12,3), key='-NOLOOP-')],
              [sg.Text('Plot test', font='Any 18')],             
              [sg.Canvas(size=(500,500), key='canvas')]]

    # create the form and show it without the plot
    window = sg.Window('Demo Application - Embedding Matplotlib In PySimpleGUI',
                       layout, finalize=True)

    fig_agg = None
    while True:
        event, values = window.read()
        if event is None:  # if user closes window
            break
        
        if event == "update":
            if fig_agg is not None:
                    delete_fig_agg(fig_agg)
            fig = fig_maker(window)
            fig_agg = draw_figure(window['canvas'].TKCanvas, fig)

        if event == "-THREAD-":
            print('Acquisition: ', values[event])
            time.sleep(1)
            if values['-LOOP-'] == True:
                if fig_agg is not None:
                    delete_fig_agg(fig_agg)
                fig = fig_maker(window)
                fig_agg = draw_figure(window['canvas'].TKCanvas, fig)
                window.Refresh()
        
        if event == "-STOP-":
            window['-NOLOOP-'].update(True)
        
        if event == "-EXIT-":
            break
            
    
    window.close()  