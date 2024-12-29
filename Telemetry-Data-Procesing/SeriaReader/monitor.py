import csv
import queue
import re
import threading
from time import perf_counter
import PySimpleGUI as sg
import serial_comm as my_serial

class Application:

    def __init__(self, *args, **kwargs):
        super(Application, self).__init__(*args, **kwargs)
        baud_rate = 115200
        gui_queue = queue.Queue()
        serial_connector = my_serial.SerialObj(baud_rate)

        headerFont = ('Helvetica', 16)
        middleFont = ('Helvetica', 14)
        contextFont = ('Helvetica', 12)
        smallFont = ('Helvetica', 10)
        sg.theme('DarkBlue')

        layout = [
            [sg.Text('GET ACCELEROMETER GYROSCOPE\nSAMPLING DATA VIA SERIAL', font=headerFont)],
            [sg.Text('Select your serial port', font=contextFont),
             sg.Button('Serial Port Reload', size=(20, 1), font=smallFont)],
            [sg.Listbox(values=[x[0] for x in my_serial.SerialObj.get_ports()],
                        size=(40, 6), key='_SERIAL_PORT_LIST_', font=contextFont, enable_events=True)],
            [sg.Text('', key='_SERIAL_PORT_CONFIRM_', size=(40, 1), font=middleFont)],
            [sg.Text('Baud Rate: {} bps'.format(baud_rate), size=(40, 1), font=middleFont)],
            [sg.HorizontalSeparator()],
            [sg.Text('Enter CSV File Name:', font=contextFont)],
            [sg.InputText(default_text='.csv', key='_FILE_NAME_', font=contextFont)],
            [sg.Text('Serial Comm Status', font=contextFont, pad=((6, 0), (20, 0)))],
            [sg.Text('', key='_OUTPUT_', size=(40, 2), font=middleFont)],
            [sg.Button('Start', key='_ACT_BUTTON_', font=middleFont, size=(40, 1), pad=((0, 0), (0, 0)))],
            [sg.Button('Exit', font=middleFont, size=(40, 1), pad=((0, 0), (20, 0)))],
            [sg.Text('ThatProject - Version: 0.1', justification='right', size=(50, 1), font=smallFont)]
        ]

        self.window = sg.Window('Simple Serial Application', layout, size=(320, 500), keep_on_top=True)

        while True:
            event, values = self.window.Read(timeout=100)

            if event is None or event == 'Exit':
                break

            if event == 'Serial Port Reload':
                self.get_ports()

            if event == '_SERIAL_PORT_LIST_':
                self.window['_SERIAL_PORT_CONFIRM_'].update(value=self.window['_SERIAL_PORT_LIST_'].get()[0])

            if event == '_ACT_BUTTON_':
                if self.window[event].get_text() == 'Start':
                    if len(self.window['_SERIAL_PORT_LIST_'].get()) == 0:
                        self.popup_dialog('Serial Port is not selected yet!', 'Serial Port', contextFont)
                    else:
                        file_name = values['_FILE_NAME_']
                        if not file_name.endswith('.csv'):
                            file_name += '.csv'

                        self.stop_thread_trigger = False
                        self.thread_serial = threading.Thread(target=self.start_serial_comm,
                                                              args=(serial_connector,
                                                                    self.window['_SERIAL_PORT_LIST_'].get()[0],
                                                                    gui_queue, lambda: self.stop_thread_trigger, file_name),
                                                              daemon=True)
                        self.thread_serial.start()
                        self.window['_ACT_BUTTON_'].update('Stop')
                else:
                    self.stop_thread_trigger = True
                    self.thread_serial.join()
                    self.window['_ACT_BUTTON_'].update('Start')

            try:
                message = gui_queue.get_nowait()
            except queue.Empty:
                message = None
            if message is not None:
                self.window['_OUTPUT_'].Update(message)
                if 'Done' in message:
                    self.window['_ACT_BUTTON_'].update('Start')
                    self.popup_dialog(message, 'Success', contextFont)

        self.window.Close()

    def popup_dialog(self, contents, title, font):
        sg.Popup(contents, title=title, keep_on_top=True, font=font)

    def get_ports(self):
        self.window['_SERIAL_PORT_LIST_'].Update(values=[x[0] for x in my_serial.SerialObj.get_ports()])

    def start_serial_comm(self, serial_connector, serialport, gui_queue, stop_thread_trigger, file_name):
        serial_connector.connect(serialport)
        if serial_connector.is_connect():
            gui_queue.put('Serial Connected!!')
            start_time = perf_counter()

            with open(file_name, 'a', newline='') as f:
                writer = csv.writer(f, delimiter=",", quoting=csv.QUOTE_NONE, escapechar=' ')
                n = 0
                while not stop_thread_trigger():
                    try:
                        data = serial_connector.get_data()
                        if data is not None:
                            if n == 0:
                                gui_queue.put(' - Data Transmitting ::: Wait! ')
                                start_time = perf_counter()

                            n += 1
                            writer.writerow([n, re.sub(r"\s+", "", data.decode('utf-8'))])  # Guarda solo el primer valor sin espacios
                            if n % 100 == 0:
                                gui_queue.put('Saving to CSV File: {} samples logged'.format(n))

                    except OSError as error:
                        print(error)
                    except UnicodeDecodeError as error:
                        print(error)

            time_taken = perf_counter() - start_time
            sampling_rate = n / time_taken if time_taken > 0 else 0
            gui_queue.put(f'Sampling Rate: {int(sampling_rate)} hz ::: Done!')
            serial_connector.disconnect()
        return

if __name__ == '__main__':
    Application()
