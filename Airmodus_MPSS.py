#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
# Airmodus MPSS GUI
@author: Airmodus (support.airmodus@envea.global)
         
"""

# import other required packages
import pyqtgraph as pg
import numpy as np
import datetime as dt
import time
import serial
import serial.tools.list_ports
import os
import datetime
import sys
# Import qt spesific packages and timer
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree
from pyqtgraph.Qt import QtGui
from PyQt5.QtGui import QPixmap
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import (
    QMainWindow, QSplitter, QApplication, QTabWidget, QGridLayout,
    QLabel,QAction, QWidget, QVBoxLayout, QLineEdit, QPushButton,
    QSpinBox, QDoubleSpinBox, QTextEdit, QSizePolicy, QFileDialog
)
import json

# Softare version
major_ver = 0
minor_ver = 1
patch_ver = 3

# mkPen for curve
global AMPen
AMPen = pg.mkPen(color='#77B800', width = 4)
AMPen_single = pg.mkPen(color='#77B800', width = 2)

PPen = pg.mkPen(color='#77B800', width = 2)
TPen = pg.mkPen(color='#277f8e', width = 2) 
RHPen = pg.mkPen(color='#46327e', width = 2)

Pen2 = pg.mkPen(color='#4ac16d', width = 2)
Pen3 = pg.mkPen(color='#277f8e', width = 1.5)
Pen4 = pg.mkPen(color='#46327e', width = 1)


# Assign file path to a variable
basedir = os.path.dirname(__file__)

# Status lights widget shows measurement and saving status
# displayed under parameter tree
class StatusLights(QSplitter):
    def __init__(self, *args, **kwargs):
        super().__init__()
        font = self.font() # get current global font
        font.setPointSize(20) # set font size

        # create OK light widget
        self.error_light = QLabel(objectName="label")
        self.error_light.setFont(font) # apply font to label
        self.error_light.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter) # align text in center
        self.error_light.setAutoFillBackground(True) # automatically fill the background with color
        self.addWidget(self.error_light) # add widget to splitter

        # create saving light widget
        self.saving_light = QLabel(objectName="label")
        self.saving_light.setFont(font) # apply font to label
        self.saving_light.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.saving_light.setAutoFillBackground(True)
        self.addWidget(self.saving_light)

        # set relative sizes of widgets in splitter
        self.setSizes([100, 100])

    # set the color and text of ok light according to error flag, 1 = errors, 0 = no errors
    def set_error_light(self, flag):
        if flag == 1:
            self.error_light.setStyleSheet("QLabel { background-color : red }")
            self.error_light.setText("Error")
        else:
            self.error_light.setStyleSheet("QLabel { background-color : green }")
            self.error_light.setText("OK")

    # set the color and text of saving light, 1 = saving, 0 = saving off
    def set_saving_light(self, flag):
        if flag == 1:
            self.saving_light.setStyleSheet("QLabel { background-color : green }")
            self.saving_light.setText("Saving")
        else:
            self.saving_light.setStyleSheet("QLabel { background-color : red }")
            self.saving_light.setText("Saving off")

# Class for automatically calculating scan properties based on input values
class Size_scan_settings(pTypes.GroupParameter):
    def __init__(self, **opts):
        opts['type'] = 'bool'
        opts['value'] = True
        pTypes.GroupParameter.__init__(self, **opts)
           
        # input basic parameter and connect them
        self.addChild({'name': 'Scan type', 'type': 'str', 'value': 'Stepping (DMPS)', 'readonly': True})
        self.addChild({'name': 'Time between scans (s)', 'type': 'float', 'value': 120, 'suffix': 's', 'tip':'Modifies the measuring time on each size bin', 'readonly': True})
        self.addChild({'name': 'N size bins (#)', 'type': 'int', 'value': 12,'tip':'Modifies the measuring time on each size bin'})
        self.addChild({'name': 'Wait time between sizes(s)', 'type': 'float', 'value': 7, 'limits': (0, 300), 'suffix': 's', 'tip':'Default: 5 s. Time to stabilize between size change. Modifies the scan length.'})
        self.addChild({'name': 'Wait time at scan end(s)', 'type': 'float', 'value': 5, 'limits': (0, 300), 'suffix': 's', 'tip':'Default: 5 s. Time to stabilize between size change. Modifies the scan length.'})
        self.addChild({'name': 'Wait time at scan start(s)', 'type': 'float', 'value': 5, 'limits': (0, 300), 'suffix': 's', 'tip':'Default: 5 s. Time to stabilize between size change. Modifies the scan length.'})
        self.addChild({'name': 'Measuring time (s)', 'type': 'float', 'value': 3, 'limits': (1, 300), 'suffix': 's','tip':'Default: 5. Time to sample the selected size. Modifies the scan length.'})        
        
        self.scan_length = self.param('Time between scans (s)')
        self.n_bins = self.param('N size bins (#)')
        self.wait_t = self.param('Wait time between sizes(s)')
        self.wait_t_end = self.param('Wait time at scan end(s)')
        self.wait_t_start = self.param('Wait time at scan start(s)')
        self.meas_t = self.param('Measuring time (s)')

        # implement the size scan parameters        
        self.addChild({'name': 'Min size (nm)', 'type': 'int', 'value': 5, 'limits': (3, 1000),'tip': "Minimum is 3 nm"})
        self.addChild({'name': 'Max size (nm)', 'type': 'int', 'value': 250, 'limits': (3, 1000), 'tip': "Maximum is 1000 nm"})
        self.min_size = self.param('Min size (nm)')
        self.max_size = self.param('Max size (nm)')
        self.addChild({'name': 'Log spacing', 'type': 'bool', 'value': True})
        self.addChild({'name': 'Down scan', 'type': 'bool', 'value': False, 'tip': "By default scanning up"})

        self.dp_list = np.linspace(0,0,1)
        self.addChild({'name': 'Size list', 'type': 'text', 'value': str(list(np.round(self.dp_list,1)))[1:-1]}),#, 'readonly': True}),
        self.size_list = self.param('Size list')

        # Connect the signal emitted, when value is changed to a corresponding function to be triggered        
        self.min_size.sigValueChanged.connect(self.recalculte_dp_list)
        self.max_size.sigValueChanged.connect(self.recalculte_dp_list)
        self.scan_length.sigValueChanged.connect(self.scan_lengthChanged)
        self.n_bins.sigValueChanged.connect(self.scanChanged)
        self.n_bins.sigValueChanged.connect(self.recalculte_dp_list)
        self.wait_t.sigValueChanged.connect(self.scanChanged)
        self.wait_t_start.sigValueChanged.connect(self.scanChanged)
        self.wait_t_end.sigValueChanged.connect(self.scanChanged)
        self.meas_t.sigValueChanged.connect(self.scanChanged)
        self.size_list.sigValueChanged.connect(self.size_listChanged)
        self.child('Log spacing').sigValueChanged.connect(self.recalculte_dp_list)
        self.child('Down scan').sigValueChanged.connect(self.recalculte_dp_list)

        self.recalculte_dp_list()

    # Function to calculate new values for the interdependent variables, and to trigger them if changed
    def scan_lengthChanged(self):
        # Substract the wait time at the start and beginning of each round
        meas_time = int(np.round(((self.scan_length.value()-self.wait_t_total) - self.wait_t.value()*self.n_bins.value())/self.n_bins.value()))
        # add block signals, and error messages
        self.meas_t.setValue(meas_time)

    def scanChanged(self):
        self.wait_t_total = self.wait_t_start + self.wait_t_end
        meas_time = int(np.round((self.meas_t.value() + self.wait_t.value())*self.n_bins.value()))
        # includes wait time at the end and beginning of each round
        self.scan_length.setValue(meas_time + self.wait_t_total)
        
    def scan_paramChanged(self):
        if self.scan_param.value() == True:
            self.scan_fixed.setValue(False,blockSignal=self.scan_fixedChanged)
            self.n_bins.setReadonly(False)
            self.size_list.setReadonly(True)
        else:
            self.scan_fixed.setValue(True,blockSignal=self.scan_fixedChanged)
            self.n_bins.setReadonly(True)
            self.size_list.setReadonly(False)
    
    def scan_fixedChanged(self):
        if self.scan_fixed.value() == True:
            self.scan_param.setValue(False,blockSignal=self.scan_paramChanged)
            self.n_bins.setReadonly(True)
            self.size_list.setReadonly(False)
        else:
            self.scan_param.setValue(True,blockSignal=self.scan_paramChanged)
            self.n_bins.setReadonly(False)
            self.size_list.setReadonly(True)
    
    def size_listChanged(self):       
        size_list_vals = np.array(list(map(float,self.size_list.value().split(','))))
        list_len = len(size_list_vals)
        self.n_bins.setValue(list_len,blockSignal=self.recalculte_dp_list)
        self.dp_list = size_list_vals

    def recalculte_dp_list(self):
        if self.child('Log spacing').value():
            self.dp_list = np.logspace(np.log10(self.min_size.value()),np.log10(self.max_size.value()),self.n_bins.value())
        else:
            self.dp_list = np.linspace(self.min_size.value(),self.max_size.value(),self.n_bins.value())

        if self.child('Down scan').value():
            self.dp_list = np.flip(self.dp_list)

        self.size_list.setValue(str(list(np.round(self.dp_list,1)))[1:-1],blockSignal=self.size_listChanged)


# define a serial connection class 
class dmps_device_serial_connection():
    def __init__(self):
        # Define the class to have attributes of serial_port, time_out,port_in_use
        self.serial_port = "NaN"
        self.timeout = 0.1
        # Stores the comport name in use currently
        self.port_in_use = "NaN"
        self.port_list = self.list_com_ports()
        self.connection = []
        
    def set_port(self,serial_port):
            # drop whitespaces
        serial_port = serial_port.strip()
        self.serial_port = serial_port

    def set_timeout(self,timeout):
        self.timeout = timeout
            
    def list_com_ports(self):
        com_ports = serial.tools.list_ports.comports()
        com_ports_text = ""
        com_port_list = []
        for port, desc, hwid in sorted(com_ports):
            com_ports_text += ("{} : {}\n".format(port, desc))
            com_port_list.append(port)
        return com_port_list
        
    # Create the serial connection
    def connect(self):
        
        # Try to close with the port that was last used
        # (needed if the port has been changed)
        # if fails (i.e. port has not been open) continue normally
        try:
            self.connection = serial.Serial(
            port=self.port_in_use,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=self.timeout,
            rtscts=True)
            self.connection.close()
        except:
            pass
        
        self.connection = serial.Serial(
        port=self.serial_port,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=self.timeout,
        rtscts=True)
        self.port_in_use = self.serial_port

    # Close the serial port
    def close(self):
        # if connection exist, it is closed
        for port in self.port_list:
            try:
                # Try to close with the port that was last used
                # (needed if the port has been changed)
                self.connection = serial.Serial(
                port=port,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.timeout,
                rtscts=True)
                self.connection.close()
                self.connection.__del__()
                
            except:
                # Try to close
                self.connection.close()
                self.connection.__del__()
            else:
                print("Failed to close port")
                pass
        
    # Define CPC control functions
    def send_and_read(self,message):
        try:
            # add line termination and convert message to bytes                           
            message = str(message)+'\r\n'
            message = bytes(message, 'utf-8')
            
            # clear buffer to drop any junk in serial buffer
            self.connection.reset_input_buffer()
            self.connection.reset_output_buffer()

            # read evertyhing in com port, again for removing junk. Could be removed?
            self.connection.read_all()
            
            # send and read message
            self.connection.write(message)
            return  self.connection.read_until(b'\r')
        except:
            pass
    
    def send_message(self,message):                                  
        message = str(message)+'\r\n'
        message = bytes(message, 'utf-8')
        self.connection.reset_input_buffer()
        self.connection.reset_output_buffer()
        self.connection.write(message)
    
    def send_clean_message(self,message):                                  
        message = str(message)+'\r\n'
        message = bytes(message, 'utf-8')
        self.connection.write(message)
    
    def read_line(self):                                  
        return  self.connection.read_until(b'\r')
    
    def read_line_DMA(self):                                  
        return  self.connection.read_until(expected=b'\r\n', size=128)

    def read_all(self):                                  
        return  self.connection.read_all()
    
    def get_arduino_data(self):
        message = 'READ_ALL\r\n'
        message = bytes(message, 'utf-8')
        self.connection.write(message)
        # A small wait time needed for the answer - not optimal structure
        time.sleep(0.1)
        arduino_answer = self.connection.read_until(b'\r\n')
        arduino_answer= arduino_answer.decode()[:-1].split(",")[:-1]
        tries = 0
        
        # Loop for waiting the arduino answer e.g. for startup
        while not arduino_answer:
            self.connection.write(message)
            arduino_answer = self.connection.read_until(b'\r\n')
            print(arduino_answer)
            arduino_answer= arduino_answer.decode()[:-1].split(",")[:-1]
            tries = tries +1 
            if tries > 20:
                print("Arduino not answering")
                break
            time.sleep(0.01)
        return arduino_answer
    
    def to_dict(self):
        return {
            'serial_port': self.serial_port,
            'timeout': self.timeout,
            'port_in_use': self.port_in_use,
            'port_list': self.port_list
        }

    def from_dict(self, data):
        self.serial_port = data.get('serial_port', "NaN")
        self.timeout = data.get('timeout', 0.1)
        self.port_in_use = data.get('port_in_use', "NaN")
        self.port_list = data.get('port_list', [])
   

# ScalableGroup for creating a menu where to set up new COM devices
class ScalableGroup(pTypes.GroupParameter):
    def __init__(self, **opts):
        pTypes.GroupParameter.__init__(self, **opts)
        self.n_devices = 1 # 0 = DMA, 1 = ref cpc -> others will start from 2
            
    def addNew(self):
        self.n_devices = self.n_devices+1
        # New types of devices should be added inthe "Device type" list and given unique id number
        self.addChild({'name': "Device (ID %d)" % (self.n_devices), 'type': 'group', 'children': [
                dict(name="Device name (ID %d)" % (self.n_devices), type='str', value="serial device", removable=True, renamable=True),
                dict(name="COM port (ID %d)" % (self.n_devices), type='str', value=0, removable=True),
                dict(name="Baud rate (ID %d)" % (self.n_devices), type='int', value=115200, removable=True, renamable=True),             
                dict(name = "Connection (ID %d)" % (self.n_devices),value = dmps_device_serial_connection(), visible=False),
                {'name': 'Device type', 'type': 'list', 'values': {"DMA": 1, "Airmodus CPC": 2, "Airmodus CPC (traditional)": 3, "Other": 4, "Ref CPC": 5}, 'value': 2},
                dict(name = "Connected", type='bool', value=False,readonly = True),
                dict(name = "DevID", type='int', value=self.n_devices,readonly = True,visible = False),
                ]})

# Createa a dictionary for the main parameter tree, in which the names, types and default values are set
params = [
     # Serial controls
    {'name': 'Before starting', 'type': 'group', 'children': [
    {'name': 'Save settings', 'type': 'action'},
    {'name': 'Load settings', 'type': 'action'},
    {'name': 'Resume on startup', 'type': 'bool', 'value': False, 'tip': "Option to resume the last settings on startup"},
        ScalableGroup(name="Device settings", children=[
            
            {'name': 'DMA connection', 'type': 'group', 'children': [
                {'name': 'Device name (ID 0)', 'type': 'str', 'value': 'DMA controls'},
                {'name': 'COM port (ID 0)', 'type': 'list', 'values': {}, 'value': ''},
                {'name': 'Baud rate (ID 0)', 'type': 'int', 'value':115200, 'visible': False},
                dict(name = "Connection (ID 0)",value = dmps_device_serial_connection(), visible=False),
                {'name': 'Device type', 'type': 'int','value': 1,'readonly':True, 'visible':False},
                dict(name = "Connected", type='bool', value=False,readonly = True),
                dict(name = "DevID", type='int', value=0,readonly = True,visible = False),
            ]},
            {'name': 'Ref CPC connection', 'type': 'group', 'children': [
                {'name': 'Device name (ID 1)', 'type': 'str', 'value': 'Ref CPC'},
                {'name': 'COM port (ID 1)', 'type': 'list', 'values': {}, 'value': ''},
                {'name': 'Baud rate (ID 1)', 'type': 'int', 'value':115200, 'visible': False},
                dict(name = "Connection (ID 1)",value = dmps_device_serial_connection(), visible=False),
                {'name': 'Device type', 'type': 'int', 'value': 5,'readonly':True, 'visible':False},
                dict(name = "Connected", type='bool', value=False,readonly = True),
                dict(name = "DevID", type='int', value=1,readonly = True,visible = False),
            ]},
             
            ]),
            {'name': 'Re-scan for devices', 'type': 'action'},
           {'name': 'Data settings', 'type': 'group', 'children': [
            {'name': 'File path', 'type': 'str', 'value': "C:/Data/", 'tip': "Default path for saving data. Change if needed"},
            {'name': 'File name', 'type': 'str', 'value': "", 'tip': "Datafile format: YYYYMMDD_HHMM_filename.csv"},
            {'name': 'Save data', 'type': 'bool', 'value': False, 'tip': "Data saving off by default"},
        ]},
             ]},
    
    {'name': 'Measurement status', 'type': 'group', 'children': [

        Size_scan_settings(name='Size scan settings'),

        {'name': 'DMA controls', 'type': 'group', 'children': [
            {'name': 'Select gas', 'type': 'list', 'limits': {"Air": "Air", "Nitrogen": "Nitrogen", "Argon": "Argon"}},
            
            {'name': 'DMA sheath flow', 'type': 'float', 'value': 10.0, 'suffix': 'lpm'},
            {'name': 'Sheath air on', 'type': 'bool', 'value': False, 'tip': "By default starts with blower off"},
            {'name': 'HV on', 'type': 'bool', 'value': False, 'tip': "By default starts with HV off"},
            {'name': 'Manual DMA voltage', 'type': 'float', 'value': 0,'decimals': 4, 'suffix': 'V','siPrefix': False},
            {'name': 'Apply manual voltage', 'type': 'action'},
            {'name': 'Manual Dp', 'type': 'float', 'value': 0, 'suffix': 'nm', 'decimals': 4,'siPrefix': False},
            {'name': 'Apply manual Dp', 'type': 'action'},
            {'name': 'Scanning on', 'type': 'bool', 'value': False, 'tip': "By default scanning is off"},
        ]},

        {'name': 'Current values', 'type': 'group', 'children': [
            {'name': 'Ref CPC concentration', 'type': 'float', 'value': 0, 'suffix': '#/cc', 'readonly': True, 'decimals': 2,'siPrefix': False},
            {'name': 'Ref CPC status', 'type': 'str', 'value': "OK", 'readonly': True},
            {'name': 'Sheath flow', 'type': 'float', 'value': "0", 'readonly': True, 'suffix': 'lpm', 'decimals': 3,'siPrefix': False},
            {'name': 'Sheath T', 'type': 'float', 'value': "0", 'readonly': True, 'suffix': '\u00b0'+'C', 'decimals': 3,'siPrefix': False},
            {'name': 'Sheath P', 'type': 'float', 'value': "0", 'readonly': True, 'suffix': 'kPa', 'decimals': 3,'siPrefix': False},
            {'name': 'Sheath RH', 'type': 'float', 'value': "0", 'readonly': True, 'suffix': '%', 'decimals': 3,'siPrefix': False},
            {'name': 'Current Dp', 'type': 'float', 'value': "0", 'readonly': True, 'suffix': 'nm', 'decimals': 3,'siPrefix': False},
            {'name': 'DMA set V', 'type': 'float', 'value': "0", 'readonly': True, 'suffix': 'V', 'decimals': 3,'siPrefix': False},
            {'name': 'Monitor V', 'type': 'float', 'value': "0", 'readonly': True, 'suffix': 'V','decimals': 3,'siPrefix': False,'visible':False},
        
        ]},
        {'name': 'DMA properties', 'type': 'group', 'children': [
            {'name': 'R1', 'type': 'float', 'value': 0.025, 'suffix': 'm', 'decimals': 3},
            {'name': 'R2', 'type': 'float', 'value': 0.033, 'suffix': 'm', 'decimals': 3},
            {'name': 'L', 'type': 'float', 'value': 0.109, 'suffix': 'm', 'decimals': 3},
            {'name': 'Voltage offset', 'type': 'float', 'value': -4.4084, 'decimals': 6},
            {'name': 'Voltage slope', 'type': 'float', 'value': 2.1991, 'decimals': 6},
            {'name': 'Sheath offset', 'type': 'float', 'value': 0, 'decimals': 6},
            {'name': 'Sheath slope', 'type': 'float', 'value': 0.9948, 'decimals': 6},
            {'name': 'Save parameters', 'type': 'action', 'tip': "Set the calibration values to the DMA"},
            
        ]},
    ]},
   
]   

## Create tree of Parameter objects
p = Parameter.create(name='params', type='group', children=params)

# MAIN PROGRAM 
class MainWindow(QMainWindow):
    def __init__(self,params=p,n_points = 200,parent=None):
        super().__init__() # super init function must be called when subclassing a Qt class
        self.setWindowTitle("Airmodus MPSS " + str(major_ver)+'.'+str(minor_ver)+'.'+str(patch_ver)+'.') # set window title
        self.timer = QTimer() # create timer object
        self.params = params # predefined paramete tree
        self.len_points = n_points # number of points shown on plots (from parameter tree in the future?)
        self.config_file_path = "" # path to the configuration file
        
        # load CSS style and apply it to the main window
        with open(os.path.join(basedir, "style.css"), "r") as f:
            self.style = f.read()
        self.setStyleSheet(self.style)
        
        # Helper counters
        self.counter = 0
        self.size_counter = 0

        # Set the time base
        self.time_step = 0.1 # time step in seconds

        # Init sheath and voltage slopes and offsets
        self.calibration_sheath_slope_air = 1
        self.calibration_sheath_offset_air = 0
        self.calibration_sheath_slope_n2 = 1
        self.calibration_sheath_offset_n2 = 0
        self.calibration_sheath_slope_ar = 1
        self.calibration_sheath_offset_ar = 0
        self.calibration_voltage_slope = 2
        self.calibration_voltage_offset = 0

        # Initilize gas A,B,C values (Cunningham slip correction factors)
        self.gas_parameters_A = 1.165
        self.gas_parameters_B = 0.483
        self.gas_parameters_C = 0.997
         
        # init seriacl connection dictionary
        self.com_dict = {}

        # signal if the filaname has changed or the data saving is turned on/off
        self.params.child('Before starting').child('Data settings').child('Save data').sigValueChanged.connect(self.filename_check)
        self.params.child('Before starting').child('Data settings').child('Save data').sigValueChanged.connect(self.saving_light_state)
        self.params.child('Before starting').child('Data settings').child('Save data').sigValueChanged.connect(self.save_params_check)
        self.params.child('Before starting').child('Data settings').child('Save data').sigValueChanged.connect(self.save_ini)
        self.params.child('Before starting').child('Data settings').child('File name').sigValueChanged.connect(self.filename_check)
        self.params.child('Before starting').child('Data settings').child('File path').sigValueChanged.connect(self.filename_check)
        self.params.child('Before starting').child('Device settings').child('Ref CPC connection').child('Connected').sigValueChanged.connect(self.get_ref_cpc_data)
        self.params.child('Before starting').child('Device settings').child('DMA connection').child('Connected').sigValueChanged.connect(self.get_calibration)
        
        # DMA parameters
        self.syst_stable_time = self.params.child('Measurement status').child('Size scan settings').child('Wait time between sizes(s)').value()
        self.syst_meas_time = self.params.child('Measurement status').child('Size scan settings').child('Measuring time (s)').value()
        self.min_dp = self.params.child('Measurement status').child('Size scan settings').child('Min size (nm)').value()
        self.max_dp = self.params.child('Measurement status').child('Size scan settings').child('Max size (nm)').value()
        self.n_size_bins = self.params.child('Measurement status').child('Size scan settings').n_bins.value()
        self.dp_list = self.params.child('Measurement status').child('Size scan settings').dp_list

        # Init gas parameters
        self.load_gas_parameters()

        self.scan_dp_history = []
        self.scan_history = []
        self.x = np.linspace(0,20,21)

        # Create timer for the measurements       
        self.timer = QTimer()
        self.timer.setTimerType(0) # 0 = precise, 1 coarse, 2 very coarse
        self.tim = dt.datetime.now()

        self.dp_ind = 0 
        self.waited_time = 0
        self.measured_time = 0
        
        self.scan_Q = []
        self.scan_RH = []
        self.scan_T = []
        self.scan_P = []
        
        # These probably need to be read/imported from somewhere, will become a long list eventually
        self.message0 = ":SYST:VER"
        self.message1 = ":SYST:PRNT"
        self.message2 = ":SYST:POUT"
        self.message3 = ":MEAS:TEMP"
        self.message4 = ":MEAS:PRES"
        self.message5 = ":MEAS:OPC"
        self.message6 = ":MEAS:ALL"
        self.message7 = ":SYST:PALL"
        self.message6header = ["CPC concentration",
                               "Counter pulses",
                               "Counter duration",
                               "Counter 2 pulses",
                               "Avg. time",
                               "Saturator temp.",
                               "Optics temp.",
                               "Condenser temp.",
                               "Cabin temp.",
                               "Inlet pres.",
                               "Critical orifice pres.",
                               "Nozzle pres.",
                               "Cabin pres.",
                               "Water removal pres.",
                               "Liquid level status",
                               "System status"]
            
        self.latest_data = {} # Dictionary used to store the latest values
        self.plot_data = {} # Dictionary used to store the plotted values
        self.curve_dict = {} # Dictionary for the various CPC/EM plots
        self.con_val = 0 # Connection value, 0 not connected 1 = connected
        
        # Init plot data for environmental parameters
        self.plot_data['RH'] = np.nan*np.zeros(self.len_points)
        self.plot_data['T'] = np.nan*np.zeros(self.len_points)
        self.plot_data['P'] = np.nan*np.zeros(self.len_points)       

        # Signal for changing DMA flow (blower voltage / blower voltage)
        self.params.child('Before starting').child('Re-scan for devices').sigActivated.connect(self.set_port_list)
        self.params.child('Before starting').child('Save settings').sigActivated.connect(self.save_configuration)
        self.params.child('Before starting').child('Load settings').sigActivated.connect(self.manual_load_configuration)
        self.params.child('Before starting').child('Resume on startup').sigValueChanged.connect(self.save_ini)

        # If measurement related parameters are changed, signal approriate functions
        self.params.child('Measurement status').child('DMA controls').child('Select gas').sigValueChanged.connect(self.load_gas_parameters)
        self.params.child('Measurement status').child('DMA controls').child('DMA sheath flow').sigValueChanged.connect(self.changeFlow)
        self.params.child('Measurement status').child('DMA controls').child('Sheath air on').sigValueChanged.connect(self.changeFlow)
        self.params.child('Measurement status').child('DMA controls').child('Apply manual voltage').sigActivated.connect(self.applyDmaVoltage)
        self.params.child('Measurement status').child('DMA controls').child('Apply manual Dp').sigActivated.connect(self.applyDp)
        self.params.child('Measurement status').child('DMA controls').child('Scanning on').sigValueChanged.connect(self.startScan)
        self.params.child('Measurement status').child('DMA properties').child('Sheath slope').sigValueChanged.connect(self.changeFlow)

        # If the DMA properties are changed, the calibration flag is set to True to pass the setting to the MPSS box
        self.params.child('Measurement status').child('DMA properties').child('Save parameters').sigActivated.connect(self.set_calibration_flag)

    def save_params_check(self):
        # save the parameter automatically, when turning the save data on
        if self.params.child('Before starting').child('Data settings').child('Save data').value():
            self.save_configuration_automatics()
        
    def save_configuration_automatics(self):
        # Automatically generate and timestamp a name and save the configuration
        tim = dt.datetime.now()
        timeStampStr = str(tim.strftime("%Y%m%d%H%M%S"))
        configName = 'measurement_parameters'
        self.config_file_path = os.path.join(self.filePath, f"{timeStampStr}{configName}.json")
        self.collect_and_dump_config(self.config_file_path)

    def save_configuration(self):
        # Ask the user for the file path to save the configuration
        file_dialog = QFileDialog(self)
        self.config_file_path, _ = file_dialog.getSaveFileName(self, 'Save Configuration', '', 'JSON Files (*.json)')
        self.collect_and_dump_config(self.config_file_path)

    def collect_and_dump_config(self,output_file):
        # Get the parameter tree values
        parameter_values = self.save_parameters_recursive(self.params)
        # Handle non-serializable objects (e.g., dmps_device_serial_connection)
        parameter_values = self.handle_non_serializable(parameter_values)
        # Save the configuration to the JSON file
        with open(output_file, 'w') as file:
            json.dump(parameter_values, file)
    
    def collect_and_dump_resume_config(self):
        # Get the parameter tree values
        parameter_values = self.save_parameters_recursive(self.params)
        # Handle non-serializable objects (e.g., dmps_device_serial_connection)
        parameter_values = self.handle_non_serializable(parameter_values)
        # Save the configuration to the JSON file
        with open(os.path.join(basedir, 'resume_config.json'), 'w') as file:
            json.dump(parameter_values, file)
                
    def handle_non_serializable(self, obj):
        if isinstance(obj, dmps_device_serial_connection):
            # Handle the non-serializable object (e.g., replace it with a default value)
            return "DefaultReplacementValue"
        elif isinstance(obj, dict):
            # Recursively handle non-serializable objects in nested dictionaries
            return {key: self.handle_non_serializable(value) for key, value in obj.items()}
        elif isinstance(obj, list):
            # Recursively handle non-serializable objects in nested lists
            return [self.handle_non_serializable(item) for item in obj]
        else:
            # Return the object as is (already serializable)
            return obj
    
    def save_parameters_recursive(self, parameters):
        result = {}
        for param in parameters:
            if param.hasChildren():
                result[param.name()] = self.save_parameters_recursive(param.children())
            else:
                # Check if the parameter value is an instance of dmps_device_serial_connection
                if isinstance(param.value(), dmps_device_serial_connection):
                    # Use the to_dict method for serialization
                    result[param.name()] = param.value().to_dict()
                else:
                    result[param.name()] = param.value()
        return result

    def load_ini(self):
        # load the configuration file "config.ini" from the basedir
        try:
            # Replace with os path join
            with open(os.path.join(basedir, 'config.ini'),'r') as f:
                config = f.read()
                json_path = config.split(';')[0]
                resume_measurements = config.split(';')[1]
                # If json path is empty
                if not json_path:
                    json_path = os.path.join(basedir, 'resume_config.json')
                self.config_file_path = json_path
                resume_measurements = int(resume_measurements)
                if resume_measurements:
                    self.load_configuration(json_path)
        except:
            # If the file does not exist, raise an exception saying that the file does not exist
            print("No ini file found")

    def save_ini(self):
        resume_measurements = 0
        if self.params.child('Before starting').child('Resume on startup').value():
            resume_measurements = 1

        # Always resume from the last configuration
        self.config_file_path = os.path.join(basedir, 'resume_config.json')
        with open(os.path.join(basedir, 'config.ini'),'w') as f:
            f.write(self.config_file_path)
            f.write(';')
            f.write(str(resume_measurements))

        self.collect_and_dump_resume_config()

    def manual_load_configuration(self):
        # Ask the user for the file path to load the configuration
        file_dialog = QFileDialog(self)
        file_path, _ = file_dialog.getOpenFileName(self, 'Load Configuration', '', 'JSON Files (*.json)')
        self.load_configuration(file_path)

    def load_gas_parameters(self):
        # Load the gas parameters for the selected gas, based on DMA properties selection
        if self.params.child('Measurement status').child('DMA controls').child('Select gas').value() == "Air":
            self.gas_viscosity = 1.85e-5 # kg/m/s
            self.gas_mfp = 6.73e-8 # 67.3 nm
            self.gas_voltage_limit = 9000
            self.gas_parameters_A = 1.165
            self.gas_parameters_B = 0.483
            self.gas_parameters_C = 0.997

            self.params.child('Measurement status').child('DMA properties').child('Sheath offset').setValue(self.calibration_sheath_offset_air)
            self.params.child('Measurement status').child('DMA properties').child('Sheath slope').setValue(self.calibration_sheath_slope_air)

        elif self.params.child('Measurement status').child('DMA controls').child('Select gas').value() == "Nitrogen":
            self.gas_viscosity = 1.78e-5
            self.gas_mfp = 6.55e-8 # 67.3 nm
            self.gas_voltage_limit = 9000
            # Nitrogen gas parameters - same as air
            self.gas_parameters_A = 1.165
            self.gas_parameters_B = 0.483
            self.gas_parameters_C = 0.997

            self.params.child('Measurement status').child('DMA properties').child('Sheath offset').setValue(self.calibration_sheath_offset_n2)
            self.params.child('Measurement status').child('DMA properties').child('Sheath slope').setValue(self.calibration_sheath_slope_n2)

        elif self.params.child('Measurement status').child('DMA controls').child('Select gas').value() == "Argon":
            self.gas_viscosity = 2.23e-5
            self.gas_mfp = 7.97e-8
            self.gas_voltage_limit = 4750
            # Argon slip correction factor parameters from Rader, 1990 "Momentum slip correction factors for small particles in nine common gases"
            self.gas_parameters_A = 1.227
            self.gas_parameters_B = 0.420
            self.gas_parameters_C = 1.0

            self.params.child('Measurement status').child('DMA properties').child('Sheath offset').setValue(self.calibration_sheath_offset_ar)
            self.params.child('Measurement status').child('DMA properties').child('Sheath slope').setValue(self.calibration_sheath_slope_ar)
        
    def load_configuration(self,file_path=None):

        if file_path:
            # Load the configuration from the JSON file
            with open(file_path, 'r') as file:
                parameter_values = json.load(file)

            # Set the loaded parameter values to the parameter tree
            self.load_parameters_recursive(self.params, parameter_values)
            
    def load_parameters_recursive(self, parameters, values):
        for param in parameters:
            if param.hasChildren():
                self.load_parameters_recursive(param.children(), values.get(param.name(), {}))
            else:
                # Check if the parameter value is a dictionary (indicating a complex object)
                if isinstance(values.get(param.name()), dict):
                    # Recreate the dmps_device_serial_connection object from the dictionary
                    try:
                        connection_values = values.get(param.name())
    
                        connection_name = connection_values[list(connection_values.keys())[0]]
                        param.children()[0].setValue(connection_name)
    
                        connection_port = connection_values[list(connection_values.keys())[1]]
                        param.children()[3].value().set_port(connection_port)
                        param.children()[1].setValue(connection_port)
                        param.children()[3].value().port_in_use = connection_port
                        
                        devtype = connection_values[list(connection_values.keys())[2]]
                        param.children()[2].setValue(devtype)
    
                        devtype = connection_values[list(connection_values.keys())[4]]
                        param.children()[4].setValue(devtype)
    
                        devid = connection_values[list(connection_values.keys())[6]]
                        param.children()[6].setValue(devid)
    
                        param.children()[3].value().connect
                    except:
                        pass
                else:
                    # Set the parameter value as usual
                    param.setValue(values.get(param.name(), param.value()))

    def calculate_DMA_V(self,Dp = None):
        # Dp in nm, Q in lpm, T1 in celcius, P in Pa
        # Flow value is taken from flow setpoint
        if Dp is None:
            Dp = self.dp_list[self.dp_ind]
        # Let  use the setpoin to avoid noise of measurments, instead of current values for the DMA flow
        Q = self.params.child('Measurement status').child('DMA controls').child('DMA sheath flow').value() 
        T1 = self.params.child('Measurement status').child('Current values').child('Sheath T').value()
        P = self.params.child('Measurement status').child('Current values').child('Sheath P').value()
        R1 = self.params.child('Measurement status').child('DMA properties').child('R1').value() # m
        R2 = self.params.child('Measurement status').child('DMA properties').child('R2').value() # m
        L = self.params.child('Measurement status').child('DMA properties').child('L').value() # m

        mu0 = self.gas_viscosity # kg/m/s
        mean_free_path_0 = self.gas_mfp # nm

        P = P*1000 # kPa to Pa
        Q_s = Q/(1000*60)
        T = T1+273.15
        T0 = 296.15
        P0 = 101300 # Pa
        S = 11.04
        mu = mu0 *(T/T0)**(3/2)*((T0+S)/(T+S))
        mean_free_path = mean_free_path_0*(T/T0)**2*(P0/P)*((T0+S)/(T+S))       
        element = 1.602177e-19 # elementary charge
        Dp = Dp*1e-9 # from nm to m
        n = 1# number of charges
        Kn=2*mean_free_path/Dp
        
        A = self.gas_parameters_A
        B = self.gas_parameters_B
        C = self.gas_parameters_C

        if self.params.child('Measurement status').child('DMA controls').child('Select gas').value() == "Argon":
            C = 1 + A*Kn

        Cc=1+(Kn)*(A+B*np.exp(-(C)/(Kn)))
        Z=(n*element*Cc)/(3*np.pi*mu*Dp)
        return Q_s*np.log(R2/R1)/(2*np.pi*L*Z)
    
    def calculate_DMA_Dp(self,voltage):
        # Dp in nm
        try:
            V = voltage
            T1 = self.params.child('Measurement status').child('Current values').child('Sheath T').value()
            P1 = self.params.child('Measurement status').child('Current values').child('Sheath P').value()
            Q_s_lpm = self.params.child('Measurement status').child('DMA controls').child('DMA sheath flow').value() 
            R1 = self.params.child('Measurement status').child('DMA properties').child('R1').value() # m
            R2 = self.params.child('Measurement status').child('DMA properties').child('R2').value() # m
            L = self.params.child('Measurement status').child('DMA properties').child('L').value() # m

            P = P1*1000 # kPa to Pa
            T = T1+273.15
            Q_s = Q_s_lpm/1000/60
          
            mu0 = self.gas_viscosity # kg/m/s
            mean_free_path_0 = self.gas_mfp # nm

            T0 = 296.15
            P0 = 101300 # Pa
            S = 11.04
            mu = mu0 *(T/T0)**(3/2)*((T0+S)/(T+S))
            mean_free_path = mean_free_path_0*(T/T0)**2*(P0/P)*((T0+S)/(T+S))           
            element = 1.602177e-19 # elementary charge

            Z = Q_s*np.log(R2/R1)/(2*np.pi*L*V)
            Dp = np.logspace(np.log10(0.1),np.log10(1000),10000)*1e-9
            #Calculating the Knudsen number, slip correction factor and the resulting electrical mobility for all 100 000 values:
            Kn=2*mean_free_path/Dp

            A = self.gas_parameters_A
            B = self.gas_parameters_B
            C = self.gas_parameters_C

            if self.params.child('Measurement status').child('DMA controls').child('Select gas').value() == "Argon":
                C = 1 + A*Kn

            Cc=1+(Kn)*(A+B*np.exp(-(C)/(Kn)))
            Zz=(1*element*Cc)/(3*np.pi*mu*Dp)
            #Compare the thus calculated values, to the required mean mobility based on the DMA parameters, and choose the value closest to the theoretical:
            ind = np.argmin(np.abs(Zz-Z))
            Dpout = Dp[ind]
        except:
            Dpout = -1
        return Dpout
    
    # Initialize the size scan
    def startScan(self):
        # Recheck the wait and meas time
        self.dp_list = self.params.child('Measurement status').child('Size scan settings').dp_list
        self.syst_stable_time = self.params.child('Measurement status').child('Size scan settings').child('Wait time between sizes(s)').value()
        self.syst_meas_time = self.params.child('Measurement status').child('Size scan settings').child('Measuring time (s)').value()
        # Get wait_t_end and wait_t_start from parameter tree
        self.wait_t_end = self.params.child('Measurement status').child('Size scan settings').child('Wait time at scan end(s)').value()    
        self.wait_t_start = self.params.child('Measurement status').child('Size scan settings').child('Wait time at scan start(s)').value()
        self.wait_total = self.wait_t_end + self.wait_t_start
        self.waited_time_up = 0

        # Turn first voltage to 0 and init counters to 0
        self.init_size_dist_data()
        self.clear_image()
        self.setDMAtoZero()
        self.waited_time = 0
        self.measured_time = 0
        self.dp_ind = 0

        # If scanning is on (True)
        if self.params.child('Measurement status').child('DMA controls').child('Scanning on').value():
            self.set_Dp()
            self.voltage_stepping()

    def scan_check(self):
        if self.params.child('Measurement status').child('DMA controls').child('Scanning on').value():
            self.voltage_stepping()

    def update_image(self):
        # Update the surface plot        
        # Scale the y-axis to log scale if needed
        if self.params.child('Measurement status').child('Size scan settings').child('Log spacing').value():
            y = np.log10(np.logspace(np.log10(np.nanmin(self.dp_list)),np.log10(np.nanmax(self.dp_list)),len(self.dp_list)+1))
            self.p2.setYRange(np.log10(np.nanmin(self.dp_list)),np.log10(np.nanmax(self.dp_list)))
        else:
            y = np.linspace(np.nanmin(self.dp_list),np.nanmax(self.dp_list),len(self.dp_list)+1)
            self.p2.setYRange(np.nanmin(self.dp_list),np.nanmax(self.dp_list))

        # Set the surface plot data
        X, Y = np.meshgrid(self.x, y)
        
        # Adjust the y-axis ticks to the size range        
        if self.params.child('Measurement status').child('Size scan settings').child('Log spacing').value():
            ticks = [[(np.log10(i), str(np.round(i,2))) for i in np.logspace(np.log10(np.nanmin(self.dp_list)),np.log10(np.nanmax(self.dp_list)),6)]]
        else:
            ticks = [[(i, str(np.round(i,2))) for i in np.linspace(np.nanmin(self.dp_list),np.nanmax(self.dp_list),6)]]
        self.p2.getAxis('left').setTicks(ticks)
        return [X.T, Y.T]

    def clear_image(self):
        self.scan_history = np.zeros([20,len(self.dp_list)])
        [X,Y] = self.update_image()
        self.p2.clear()
        self.img = pg.PColorMeshItem(X, Y ,self.scan_history)
        self.p2.addItem(self.img)

    def init_size_dist_data(self):                    
        # Init and create a storage for the size distribution data                    
        self.plot_data['Size dist conc'] = np.nan*np.zeros(len(self.dp_list))
        self.plot_data['Dp_list'] = self.dp_list

    # Send command to set DMA voltage
    def set_voltage(self):

        connectionStatus = self.params.child('Before starting').child('Device settings').child('Ref CPC connection').child('Connected').value()
        # IF CPC connected
        if connectionStatus == True:

            voltage = self.set_voltage_value

            # Ensure that message goes throug, if calbration yields negative values
            if voltage < 0:
                voltage = 0

            # Limit the voltage to 9kV to prevent arcing
            if voltage > self.gas_voltage_limit:
                voltage = self.gas_voltage_limit

            # Rename for correct printout
            voltage_aimed = voltage

            # Calibration for the spellman HV source
            voltage_offset = self.params.child('Measurement status').child('DMA properties').child('Voltage offset').value() # m
            voltage_slope = self.params.child('Measurement status').child('DMA properties').child('Voltage slope').value() # m
            voltage = (voltage-voltage_offset)/voltage_slope

            # convert to V from kV
            voltage = voltage/1000


            #dev = self.params.child('Before starting').child('Device settings').child('Ref CPC connection').children()[3].value()
            message = ":SET:VPOINT " + str(voltage)
            self.refConnection.send_clean_message(message)



            self.params.child('Measurement status').child('Current values').child('DMA set V').setValue(voltage_aimed)
            # If the HV is set on and flow is set, then calculate the current Dp, else set to 0
            current_dp = np.nan
            if self.params.child('Measurement status').child('DMA controls').child('Sheath air on').value():
                # if the HV is set on, then calculate the current Dp
                if self.params.child('Measurement status').child('DMA controls').child('HV on').value():
                    # Check if the voltage is set
                    if voltage_aimed > 0:
                        current_dp = self.calculate_DMA_Dp(voltage_aimed)*1e9

            self.params.child('Measurement status').child('Current values').child('Current Dp').setValue(current_dp)

    def setDMAtoZero(self):
        self.set_voltage_flag = 1
        self.set_voltage_value = 0

    # Send command to set DMA voltage to corresponding Dp value
    def set_Dp(self):
        voltage = self.calculate_DMA_V()
        self.set_voltage_flag = 1
        self.set_voltage_value = voltage

    def store_scan_conditions(self):
        self.scan_Q.append(self.latest_data[0][3])
        self.scan_RH.append(self.latest_data[0][0])
        self.scan_T.append(self.latest_data[0][1])
        self.scan_P.append(self.latest_data[0][2])

        round_time = (len(self.dp_list) * (self.syst_stable_time + self.syst_meas_time + self.wait_total) )
        if len(self.scan_Q) > round_time:
            del self.scan_Q[0]
            del self.scan_RH[0]      
            del self.scan_T[0]
            del self.scan_P[0]

    def store_point_conc(self):
        # Calculate the average of last n seconds (n = self.syst_meas_time)
        latest_mean_conc = np.nanmean(self.plot_data[1][-int(self.syst_meas_time):])
        self.plot_data['Size dist conc'][self.dp_ind] = latest_mean_conc

    def store_scan_data(self):
        # for plotting the previous size distribution on top of ongoing
        self.scan_last = self.plot_data['Size dist conc']
        self.scan_dp_last = self.plot_data['Dp_list']

        try:
            self.scan_history = np.delete(self.scan_history, 0, 0)
            self.scan_history = np.vstack([self.scan_history,self.scan_last])
        except:
            self.scan_history = np.zeros([20,len(self.dp_list)])
            print("Initializing size data")            
        
        [X,Y] = self.update_image()

        # Check if down scan is checked
        if self.params.child('Measurement status').child('Size scan settings').child('Down scan').value():
            self.img.setData(X, Y,np.flip(self.scan_history,1))
        else:
            self.img.setData(X, Y,self.scan_history)

        
        # Assuming your x values represent indices in time (replace this with your actual time data)
        scan_length_in_s = self.params.child('Measurement status').child('Size scan settings').child('Time between scans (s)').value()
        time_indices = np.linspace(0, 20*scan_length_in_s, 21)
        # Convert time indices to datetime objects
        time_values = [datetime.datetime.now() - datetime.timedelta(seconds=int(i)) for i in np.flip(time_indices)]
        # Convert datetime objects to "HH:MM" format
        time_labels = [i.strftime("%H:%M") for i in time_values]
        # Create a list of tuples for x-axis ticks
        x_ticks = [(i, label) for i, label in zip(self.x, time_labels)]
        # Set the x-axis ticks
        self.p2.getAxis('bottom').setTicks([x_ticks])

        # Zero the current concentration line after scan is over
        self.plot_data['Size dist conc'] = np.nan*np.zeros(len(self.dp_list))

    # perform with start scan        
    def voltage_stepping(self):
        
        self.store_scan_conditions()

        # Add wait time to the start of the scan
        if self.dp_ind == 0 and self.waited_time == 0:
            self.waited_time = - self.wait_t_start

        # If the Dp list changes, a trigger to resest this will be needed
        # if wait time on going just add to waited time
        if self.waited_time < self.syst_stable_time:
            self.waited_time += self.time_step
        # else start measuring
        else:
            # if measurement time not full, then just let time run
            if self.measured_time < self.syst_meas_time:
                self.measured_time += self.time_step           
            else:

                # if last size then set dp to first size:
                if self.dp_ind == len(self.dp_list)-1:

                    if self.waited_time_up == 0:
                        # if measruement time full, then change size
                        self.store_point_conc()

                    # Check if we've waited at the end of the scan for the agreed time
                    if self.waited_time_up < self.wait_t_end:
                        # Add wait time to the end of the scan
                        self.waited_time_up += self.time_step
                    else: # if waited time is up, then save the data
                        self.store_scan_data()
                        if self.params.child('Before starting').child('Data settings').child('Save data').value():
                            self.write_scan_data()
                        self.dp_ind = 0
                        self.waited_time_up = 0
                        self.waited_time = 0
                        self.measured_time = 0

                        # Zero the scan status flag 
                        self.scan_status_flag = 0

                # else set to next size, and zero measurement time
                else: 
                    # if measruement time full, then change size
                    self.store_point_conc()
                    self.dp_ind += 1
                    self.waited_time = 0
                    self.measured_time = 0

                # after one size zero the wait and meas time time
                self.set_Dp()

    def write_scan_data(self):
        
        write_headers = self.header_check(self.filename_scan)

        if write_headers:
            with open(self.filename_scan, 'a') as out_file:
                # Write headers if they don't exist
                out_file.write('DOY_start,DOY_end,YEAR_star,YEAR_end,T_internal,P_internal,Q_sample,Q_sheath,RH_sample,RH_sheath,#_size_bins,')
                # Join the strings in the list with a comma
                size_bins = ['Size_bin_' + str(i+1) for i in range(len(self.dp_list))]
                size_bins_str = ','.join(size_bins)
                size_bins_str = size_bins_str.replace('\n', '').replace('\r', '')
                out_file.write(size_bins_str+',')
                conc_bins = ['Conc_bin_' + str(i+1) for i in range(len(self.dp_list))]
                conc_bins_str = ','.join(conc_bins)
                conc_bins_str = conc_bins_str.replace('\n', '').replace('\r', '')
                out_file.write(conc_bins_str+',QUALITY_FLAG')
                out_file.write("\r")
        
        # Write the scan data to the file
        with open(self.filename_scan, 'a') as out_file:
            # Get time at the end of scan
            timenow = dt.datetime.now().timetuple()

            # Columns in corresponding order:
            day_of_year_start = str(self.day_of_year_start)
            day_of_year_end = str(timenow.tm_yday+(timenow.tm_hour + timenow.tm_min/60 + timenow.tm_sec/3600)/24)
            start_year = str(self.start_year)
            end_year = str(timenow.tm_year)
            internal_T = str(np.nanmean(self.scan_T))
            internal_P = str(np.nanmean(self.scan_P)*10) # Conversion to hPa
            internal_Q_sample = self.internal_Q_sample
            internal_Q_sheath = str(np.nanmean(self.scan_Q))
            internal_RH_sample = 'NaN' # Sensor missing
            internal_RH_sheath = str(np.nanmean(self.scan_RH))
            self.n_size_bins = self.params.child('Measurement status').child('Size scan settings').n_bins.value()
            n_size_bins = str(self.n_size_bins)

            size_vals = self.dp_list
            conc_vals = self.scan_history[-1,:]
            # If using down scan then reverse the size bins
            if self.params.child('Measurement status').child('Size scan settings').child('Down scan').value():
                size_vals = np.flip(size_vals)
                conc_vals = np.flip(conc_vals)

            # size bins
            size_data = np.array2string(size_vals,separator=",", formatter={'float_kind':lambda x: "%.3f" % x})[1:-1]
            size_data = size_data.replace('\n', '').replace('\r', '')

            # concentrations
            write_data = np.array2string(conc_vals,separator=",", formatter={'float_kind':lambda x: "%.3f" % x})[1:-1]
            write_data = write_data.replace('\n', '').replace('\r', '')

            # Quality flag (0 if ok, 999 if something missing)
            if self.scan_status_flag:
                quality_flag = '999'
            else:
                quality_flag = '0'

            # Create a list of values
            values = [day_of_year_start, 
                      day_of_year_end, 
                      start_year,
                      end_year,
                      internal_T,
                      internal_P,
                      internal_Q_sample,
                      internal_Q_sheath,
                      internal_RH_sample,
                      internal_RH_sheath,
                      n_size_bins,
                      size_data,
                      write_data,
                      quality_flag]

            # Combine the values into a single string with commas
            result = ', '.join(values)
            out_file.write(result+"\r")

        # Send command to change blower value
    def setBlowerValue(self,blowerValue):
        connection = self.params.child('Before starting').child('Device settings').child('DMA connection').children()[3].value()
        flow_offset = self.params.child('Measurement status').child('DMA properties').child('Sheath offset').value() # m
        flow_slope = self.params.child('Measurement status').child('DMA properties').child('Sheath slope').value() # m
        blowerValue = (blowerValue+flow_offset)*flow_slope
        message = "SET_FLOW "+str(blowerValue)
        connection.send_message(message)        

    def setBlowerOn(self):
        # Sheath air on, then
        if self.params.child('Measurement status').child('DMA controls').child('Sheath air on').value():
            return 1
        else:
            return 0

    def status_monitor(self):
        flag = 0
        # Check the status of the DMA blower, hv and blowersetpoint
        if not self.params.child('Measurement status').child('DMA controls').child('Sheath air on').value():
            flag = 1
        if not self.params.child('Measurement status').child('DMA controls').child('HV on').value():
            flag = 1

        # Check if the sheath flow is within 5% of the set value
        sheath = self.params.child('Measurement status').child('Current values').child('Sheath flow').value()
        sheath_set = self.params.child('Measurement status').child('DMA controls').child('DMA sheath flow').value()
        if sheath > sheath_set*1.05 or sheath < sheath_set*0.95:
            flag = 1

        # Check the status of the instruments
        for dev in self.params.child('Before starting').child('Device settings').children():
            # if device is connected
            if dev.child('Connected').value():
                if dev.child('Device type').value() == 5:
                    # Last value of latest data should be the status
                    if not self.latest_data[dev.child('DevID').value()][-1] == '0x0000':
                        flag = 1
                if dev.child('Device type').value() == 1: # DMA
                    if self.latest_data[dev.child('DevID').value()][-1] == 1:
                        flag = 1
                if dev.child('Device type').value() == 2: # other CPC
                    if not self.latest_data[dev.child('DevID').value()][-1] == '0x0000':
                        flag = 1
        if flag:
            self.status_lights.set_error_light(1)
        else:
            self.status_lights.set_error_light(0)
        
        # Store the information if the status has been at red during the scan
        if flag:
            self.scan_status_flag = 1

    def changeFlow(self):
        connectionStatus = self.params.child('Before starting').child('Device settings').child('DMA connection').child('Connected').value()
        # IF DMA connected
        if connectionStatus == True:
            # Get blower voltage values
            blowerValue = self.params.child('Measurement status').child('DMA controls').child('DMA sheath flow').value()
            # Check that the value is reasonable
            try:
                blowerValue = float(blowerValue)
                if (blowerValue >= 0) | (blowerValue < 119):
                    # Here send command
                    self.setBlowerValue(blowerValue)
                else:
                    print("Blower value out of range")
            except:
                print("Incorrect blower value given")
        else:
            print("No blower/DMA connected")             

    # Send command to change blower value
    def setHVOn(self):
        # Sheath air on, then
        if self.params.child('Measurement status').child('DMA controls').child('HV on').value():
            return 1
        else:
            return 0
        
    # Send command to change HV value
    def applyDmaVoltage(self):
        # Connection 
        self.params.child('Measurement status').child('DMA controls').child('Scanning on').setValue(False)
        voltage = self.params.child('Measurement status').child('DMA controls').child('Manual DMA voltage').value()
        self.set_voltage_flag = 1
        self.set_voltage_value = voltage

    # Send command to change HV value based on Dp value
    def applyDp(self):
        self.params.child('Measurement status').child('DMA controls').child('Scanning on').setValue(False)
        Dp = self.params.child('Measurement status').child('DMA controls').child('Manual Dp').value()
        # Convert Dp to voltage
        voltage = self.calculate_DMA_V(Dp)
        self.set_voltage_flag = 1
        self.set_voltage_value = voltage

    # Check the validity of the filename when data saving turned on
    # if name is not valid, does not allow saving, and prints an error
    # if name is valid only prints that a new file is set, when filename has changed
    def filename_check(self):
        verified_filename = 0
        if self.params.child('Before starting').child('Data settings').child('Save data').value():
            try:
                self.fileIdStr = self.params.child('Before starting').child('Data settings').child('File name').value()
                self.filePath = self.params.child('Before starting').child('Data settings').child('File path').value()
                self.file_start_tim = dt.datetime.now()
                timeStampStr = str(self.file_start_tim.strftime("%Y%m%d%H%M"))
                # Assuming self.filePath already has the directory path
                self.filename = os.path.join(self.filePath, f"{timeStampStr}{self.fileIdStr}.csv")
                self.filename_scan = os.path.join(self.filePath, f"{timeStampStr}{self.fileIdStr}_scan.csv")
                
                f= open(self.filename,"w+")
                f.close()

                f= open(self.filename_scan,"w+")
                f.close()
                verified_filename = 1
                
            except:
                print("Error in file name - rename to save data")
                self.params.child('Before starting').child('Data settings').child('Save data').setValue(False)#,blockSignal=self.connection_test)
            else:
                print('New file set')
        else:
            pass

    def connection_test(self):
        # Go through each of the devices set in the parameter tree
        n_devices = len(self.params.child('Before starting').child('Device settings').children())
        for n in range(n_devices):
            con_val = False # not connected
            
            try:
                # if connection value is not true
                if self.params.child('Before starting').child('Device settings').children()[n].children()[5].value() != True:
                    port = str(self.params.child('Before starting').child('Device settings').children()[n].children()[1].value())
                    self.params.child('Before starting').child('Device settings').children()[n].children()[3].value().set_port(port)
                    self.params.child('Before starting').child('Device settings').children()[n].children()[3].value().connect()
                    
                    # Check that if the DMA is connected stop timer, wait for arduino to reboot and restart timer
                    dev_type = self.params.child('Before starting').child('Device settings').children()[n].children()[4].value()
                    if dev_type == 1:
                        time.sleep(1)
        
                if self.params.child('Before starting').child('Device settings').children()[n].children()[3].value().connection.is_open == True:
                    con_val = True # is connected
            
                    self.params.child('Before starting').child('Device settings').children()[n].children()[5].setValue(con_val)
                
            except:
                self.params.child('Before starting').child('Device settings').children()[n].children()[5].setValue(con_val)


    def closeAllPorts(self):
        n_devices = len(self.params.child('Before starting').child('Device settings').children())
        for n in range(n_devices):
            try:
                self.params.child('Before starting').child('Device settings').children()[n].children()[3].value().close() 
            except:
                pass

            
    def readIndata(self):
        for dev in self.params.child('Before starting').child('Device settings').children():
            # if device is connected
            if dev.child('Connected').value():
                if dev.child('Device type').value() == 1: # DMA
                    try:
                        ser = dev.children()[3].value()
                        if not hasattr(ser, '_dma_buffer'):
                            ser._dma_buffer = b''

                        ser._dma_buffer += ser.connection.read(ser.connection.in_waiting or 1)

                        while b'\r' in ser._dma_buffer:
                            line, _, ser._dma_buffer = ser._dma_buffer.partition(b'\r')
                            line = line.decode(errors='ignore').strip()
                            if not line:
                                continue
                            try:
                                temp = list(map(float, line.split(',')[:-1]))
                                flow_offset = self.params.child('Measurement status').child('DMA properties').child('Sheath offset').value()
                                flow_slope = self.params.child('Measurement status').child('DMA properties').child('Sheath slope').value()
                                temp[3] = (temp[3] - flow_offset) / flow_slope
                                temp[2] = temp[2] / 1000.0
                                # Add the DMA set V rounded to 2 decimal places
                                temp[7] = round(self.params.child('Measurement status').child('Current values').child('DMA set V').value(),2)
                                self.latest_data[dev.child('DevID').value()] = temp
                                break  # stop after successful parse
                            except Exception as e:
                                print(f"Failed to parse DMA line: {e} | Line: {line}")

                    except Exception as e:
                        print(f"Failed to read DMA data: {e}")

                elif dev.child('Device type').value() == 5:  # Ref CPC
                    try:
                        ser = dev.children()[3].value()
                        if not hasattr(ser, '_cpc_buffer'):
                            ser._cpc_buffer = b''

                        ser._cpc_buffer += ser.connection.read(ser.connection.in_waiting or 1)

                        while b'\r' in ser._cpc_buffer:
                            line, _, ser._cpc_buffer = ser._cpc_buffer.partition(b'\r')
                            line = line.decode(errors='ignore').strip()
                            if not line.startswith(":MEAS:ALL"):
                                print(f"Unexpected start: {repr(line)}")
                                continue

                            raw_data = line[len(":MEAS:ALL"):].strip().strip(',').split(',')

                            if len(raw_data) != 16:
                                print(f"Serial read failed: Expected 16 values, got {len(raw_data)}")
                                continue

                            temp = list(map(float, raw_data[:-1])) + [raw_data[-1]]
                            status = raw_data[-1]

                            if status == '0x0000':
                                self.params.child('Measurement status').child('Current values').child('Ref CPC status').setValue('OK')
                            else:
                                self.params.child('Measurement status').child('Current values').child('Ref CPC status').setValue('Not OK')

                            self.latest_data[dev.child('DevID').value()] = temp
                            break

                    except Exception as e:
                        print(f"Serial read failed: {e}")
                                                
                if dev.child('Device type').value() == 2: # Other CPC
                    try:
                        temp = dev.children()[3].value().read_all().decode().split(" ")[1][:-1]                        
                        temp = temp.split(",")
                        status = temp[-1]
                        temp = list(map(float,temp[:-1]))
                        temp.append(status)
                        if len(temp) == 16:
                            self.latest_data[dev.child('DevID').value()] = temp
                    except:
                        raise ValueError('CPC reading failed - wrong number of values gotten from :MEAS:ALL')

    # Function for getting parameters for the MPSS box    
    def get_calibration(self):
        if not self.params.child('Before starting').child('Device settings').child('DMA connection').child('Connected').value():
            return
        
        connection = self.params.child('Before starting').child('Device settings').child('DMA connection').children()[3].value()
        connection.read_all()
        connection.send_message("GET_CAL")

        # .readline() respects the set timeout
        response = connection.read_line().decode().strip()
        if not response:
            raise TimeoutError("No response received from device (timeout)")

        try:
            parts = response.split(":")[1].split(",")
            floats = [float(p.strip()) for p in parts]
            self.calibration_values = floats

            self.calibration_sheath_slope_air = floats[0]
            self.calibration_sheath_offset_air = floats[1]
            self.calibration_sheath_slope_n2 = floats[2]
            self.calibration_sheath_offset_n2 = floats[3]
            self.calibration_sheath_slope_ar = floats[4]
            self.calibration_sheath_offset_ar = floats[5]
            self.calibration_voltage_slope = floats[6]
            self.calibration_voltage_offset = floats[7]

            # Check what gas is in use
            if self.params.child('Measurement status').child('DMA controls').child('Select gas').value() == "Air":
                self.params.child('Measurement status').child('DMA properties').child('Sheath slope').setValue(self.calibration_sheath_slope_air)
                self.params.child('Measurement status').child('DMA properties').child('Sheath offset').setValue(self.calibration_sheath_offset_air)
            elif self.params.child('Measurement status').child('DMA controls').child('Select gas').value() == "Nitrogen":
                self.params.child('Measurement status').child('DMA properties').child('Sheath slope').setValue(self.calibration_sheath_slope_n2)
                self.params.child('Measurement status').child('DMA properties').child('Sheath offset').setValue(self.calibration_sheath_offset_n2)
            elif self.params.child('Measurement status').child('DMA controls').child('Select gas').value() == "Argon":
                self.params.child('Measurement status').child('DMA properties').child('Sheath slope').setValue(self.calibration_sheath_slope_ar)
                self.params.child('Measurement status').child('DMA properties').child('Sheath offset').setValue(self.calibration_sheath_offset_ar)

            self.params.child('Measurement status').child('DMA properties').child('Voltage slope').setValue(self.calibration_voltage_slope)
            self.params.child('Measurement status').child('DMA properties').child('Voltage offset').setValue(self.calibration_voltage_offset)

            self.set_calibration_flag = 0
        except Exception as e:
            raise ValueError(f"Invalid response format: '{response}'") from e

    def set_calibration_flag(self):
        # Set the calibration flag to 1
        self.set_calibration_flag = 1

    # Function for setting parameters for the MPSS box    
    def set_calibration(self):
         
        connectionStatus = self.params.child('Before starting').child('Device settings').child('DMA connection').child('Connected').value()
        # IF DMA connected
        if connectionStatus == True:
            connection = self.params.child('Before starting').child('Device settings').child('DMA connection').children()[3].value()

            sheat_slope = self.params.child('Measurement status').child('DMA properties').child('Sheath slope').value() # m
            sheat_offset = self.params.child('Measurement status').child('DMA properties').child('Sheath offset').value() # m

            # Check which gas is in use
            if self.params.child('Measurement status').child('DMA controls').child('Select gas').value() == "Air":
                message = "SET_SHEATH_CAL_AIR "+str(sheat_slope)+' '+str(sheat_offset)
                self.calibration_sheath_offset_air = sheat_offset
                self.calibration_sheath_slope_air = sheat_slope
            elif self.params.child('Measurement status').child('DMA controls').child('Select gas').value() == "Nitrogen":
                message = "SET_SHEATH_CAL_N2 "+str(sheat_slope)+' '+str(sheat_offset)
                self.calibration_sheath_offset_n2 = sheat_offset
                self.calibration_sheath_slope_n2 = sheat_slope
            elif self.params.child('Measurement status').child('DMA controls').child('Select gas').value() == "Argon":
                message = "SET_SHEATH_CAL_AR "+str(sheat_slope)+' '+str(sheat_offset)
                self.calibration_sheath_offset_ar = sheat_offset
                self.calibration_sheath_slope_ar = sheat_slope
            connection.send_message(message)

            voltage_slope = self.params.child('Measurement status').child('DMA properties').child('Voltage slope').value() # m
            voltage_offset = self.params.child('Measurement status').child('DMA properties').child('Voltage offset').value() # m
            message = "SET_VOLTAGE_CAL "+str(voltage_slope)+' '+str(voltage_offset)
            connection.send_message(message)

            # Set the calibration flag to 0 after setting
            self.set_calibration_flag = 0

    def main_loop(self):

        # Send commands to retrievi device data
        self.get_dev_data()
        QTimer.singleShot(75, self.readIndata)
        
        # Capture timestamps for the start of the measurement
        if self.dp_ind == 0 and self.waited_time == 0:
            # store start time as a fractional day
            timenow = dt.datetime.now().timetuple()
            self.day_of_year_start = timenow.tm_yday+(timenow.tm_hour + timenow.tm_min/60 + timenow.tm_sec/3600)/24
            self.start_year = timenow.tm_year
        
        # Set and get settings
        if self.set_calibration_flag == 1:
            self.set_calibration()

            
        # Roll the data, append the latest values and update the stored self.plot_data
        self.update1()
        
        # Updates the plot data and variables in the parameter tree with the latest updated data
        self.update_figures_and_menus()

        # Writes the 1 Hz data using self.latest_values
        self.write_data()

        # Check if the scanning tab is on
        # if is it triggers voltage stepping, which triggers store scan variables
        self.scan_check()

        # Monitor for controlling the status light
        self.status_monitor()
       
    # Send read command to serial devices and wait for response
    def get_dev_data(self):
        # Loop through devices
        for dev in self.params.child('Before starting').child('Device settings').children():
            # if device is connected
            if dev.child('Connected').value():
                if dev.child('Device type').value() == 1: # DMA
                    # Check blower and HV status and control accordingly
                    message = "READ_ALL"+','+str(self.setBlowerOn())+','+str(self.setHVOn())
                    dev.children()[3].value().send_message(message)
                elif dev.child('Device type').value() == 2: # CPC
                    dev.children()[3].value().send_message(self.message6)
                elif dev.child('Device type').value() == 5: # Ref CPC
                    self.set_voltage()
                elif dev.child('Device type').value() == 3: # Template for other devices
                    pass
                else:
                    print("No devices connected")
        
        
    def print_time(self):
        print(time.time())

    # Update plot and store
    # Store the CPC data for both plotting and the save data (print to file)
    def update1(self):
        # If data saving is on
        # Update the plot data dictionary
        # Check the connected devices
        for dev in self.params.child('Before starting').child('Device settings').children():
            try:
                var_name = dev.child('DevID').value()
                var_id = var_name
                # if device is connected
                if dev.child('Connected').value():
                    if dev.child('Device type').value() == 1: # DMA
                        self.dev_update_plot_data(dev,3,var_name,var_id)
                        self.dev_update_plot_data(dev,0,'RH',var_id)
                        self.dev_update_plot_data(dev,1,'T',var_id)
                        self.dev_update_plot_data(dev,2,'P',var_id)
                    
                    elif dev.child('Device type').value() == 5: # Ref CPC
                        self.dev_update_plot_data(dev,0,var_name,var_name)

                    elif dev.child('Device type').value() == 2: # Other CPC
                        self.dev_update_plot_data(dev,0,var_name,var_name)
            except:
                # Silence the error if the device is not connected
                raise ValueError('Instrument not connected')                       
                        
                             
    def dev_update_plot_data(self,dev,var_number,var_name,var_id):
        try:
            # check if variable exists in plot_data dict
            if var_name in self.plot_data:
                # move the data and append last:
                self.plot_data[var_name][:-1] = self.plot_data[var_name][1:]  # shift data in the array one sample left  (see also: np.roll)
                self.plot_data[var_name][-1] = self.latest_data[var_id][var_number] # 3 = flow in DMA, 0 in CPC
            else:  
                # if not then create a variable:
                self.plot_data[var_name] = np.nan*np.zeros([self.len_points]) 
                # plot the variable in window 1
                self.plot_data[var_name][-1] = self.latest_data[var_id][var_number] # 3 = flow in DMA, 0 in CPC
        except:
            print('Failed to update plot data')
            
    def saving_light_state(self):
        # Check the status of the save data button
        if self.params.child('Before starting').child('Data settings').child('Save data').value():
            self.status_lights.set_saving_light(1)
        else:
            self.status_lights.set_saving_light(0)

        
    # Update of CPC concentration both on menu and on plot
    def update_figures_and_menus(self):

        for dev in self.params.child('Before starting').child('Device settings').children():
            # if device is connected
            if dev.child('Connected').value():

                if dev.child('Device type').value() == 1: # DMA
                # Update values
                    try:
                        # Check appropriate answer from DMA based on expected length (8 numbers)
                        if len(self.latest_data[dev.child('DevID').value()]) == 9:
                            temp = self.latest_data[dev.child('DevID').value()]
                            # DMA Voltage
                            voltage_offset = self.params.child('Measurement status').child('DMA properties').child('Voltage offset').value() # m
                            voltage_slope = self.params.child('Measurement status').child('DMA properties').child('Voltage slope').value() # m
                            mon_voltage = ((temp[7]/10)+voltage_offset)/voltage_slope*100
                            self.params.child('Measurement status').child('Current values').child('Monitor V').setValue(mon_voltage)
                            # DMA Pressure
                            self.params.child('Measurement status').child('Current values').child('Sheath P').setValue(temp[2])
                            # DMA Flow
                            self.params.child('Measurement status').child('Current values').child('Sheath flow').setValue(temp[3])
                            # Sheat temp
                            self.params.child('Measurement status').child('Current values').child('Sheath T').setValue(temp[1])
                            # Sheat RH
                            self.params.child('Measurement status').child('Current values').child('Sheath RH').setValue(temp[0])
                            # Current dp - check if scanning on
                            if self.params.child('Measurement status').child('DMA controls').child('Scanning on').value():
                                self.params.child('Measurement status').child('Current values').child('Current Dp').setValue(self.dp_list[self.dp_ind])                    

                        if dev.child('DevID').value() not in self.curve_dict:
                            name = dev.children()[0].value()
                            # plot to p3 window
                            self.curve_dict[dev.child('DevID').value()] = self.p3.plot(self.plot_data[dev.child('DevID').value()],pen = AMPen_single)
                            self.legendp3.addItem(self.curve_dict[dev.child('DevID').value()],name = name)
                            # TODO: Change pen settings
                            self.curve_dict['RH'] = self.p4.plot(self.plot_data['RH'],pen = RHPen)
                            self.legendp4.addItem(self.curve_dict['RH'],name = 'RH (%)')
                            self.curve_dict['T'] = self.p4.plot(self.plot_data['T'],pen = TPen)
                            self.legendp4.addItem(self.curve_dict['T'],name = 'T (C)')
                            self.curve_dict['P'] = self.p4.plot(self.plot_data['P'],pen = PPen)
                            self.legendp4.addItem(self.curve_dict['P'],name = 'P (kPa)')
                        else:
                            self.curve_dict[dev.child('DevID').value()].setData(self.plot_data[dev.child('DevID').value()])
                            self.curve_dict['RH'].setData(self.plot_data['RH'])
                            self.curve_dict['T'].setData(self.plot_data['T'])
                            self.curve_dict['P'].setData(self.plot_data['P'])
       
                        # if curves name in legend does not match the device name
                        # remove the legend, and rewrite it to match
                        name = dev.children()[0].value()
                        if self.curve_dict[0].name() !=  name:    
                            self.legendp3.removeItem(self.curve_dict[dev.child('DevID').value()])
                            self.legendp3.addItem(self.curve_dict[dev.child('DevID').value()],name = name)
    
                    except:
                        print("Failed to read DMA values")
                
                if (dev.child('Device type').value() == 5) & (dev['Connected']): # Ref CPC
                
                    # Update concentration reading to measurement status window
                    temp = self.latest_data[dev.child('DevID').value()][0]
                    self.params.child('Measurement status').child('Current values').child('Ref CPC concentration').setValue(temp)
                    
                    # set data i.e. update plot (now plot is not present, as connection has been established after start timer)
                    if dev.child('DevID').value() not in self.curve_dict:
                        name = dev.children()[0].value()
                        self.curve_dict[dev.child('DevID').value()] = self.p1.plot(self.plot_data[dev.child('DevID').value()],pen = AMPen_single)                        
                        #self.curve_dict[dev.child('DevID').value()] = self.p1.plot(self.plot_data[dev.child('DevID').value()],pen = dev.child('DevID').value())
                        self.legendp1.addItem(self.curve_dict[dev.child('DevID').value()],name = name)
                    else:
                        self.curve_dict[dev.child('DevID').value()].setData(self.plot_data[dev.child('DevID').value()])
                    
                    # if curves name in legend does not match the device name
                    # remove the legend, and rewrite it to match
                    name = dev.children()[0].value()
                    if self.curve_dict[1].name() !=  name:    
                        self.legendp1.removeItem(self.curve_dict[dev.child('DevID').value()])
                        self.legendp1.addItem(self.curve_dict[dev.child('DevID').value()],name = name)
                    
                    # set data i.e. update plot (now plot is not present, as connection has been established after start timer)
                    if 'Size dist. - Current' not in self.curve_dict:
                        # Check that your not trying to plot nan values (will result in an error)
                        if not np.all(np.isnan(self.plot_data['Size dist conc'])):
                            name = 'Size dist. - Current'
                            #self.plot_data['Size dist conc'] = np.zeros(len(self.dp_list))
                            self.curve_dict[name] = self.p5.plot(self.plot_data['Dp_list'],self.plot_data['Size dist conc'],pen = AMPen)
                            self.legendp5.addItem(self.curve_dict[name],name = name)
                    else:
                        if not np.all(np.isnan(self.plot_data['Size dist conc'])):
                            name = 'Size dist. - Current'                        
                            self.curve_dict[name].setData(self.plot_data['Dp_list'],self.plot_data['Size dist conc'])

                    if 'Size dist. - 3' not in self.curve_dict:
                        # Check that your not trying to plot nan values (will result in an error)
                        if not np.all(np.isnan(self.scan_history[-3,])):
                            name = 'Size dist. - 3'
                            self.curve_dict[name] = self.p5.plot(self.plot_data['Dp_list'],self.scan_history[-3,],pen = Pen4)
                            self.legendp5.addItem(self.curve_dict[name],name = name)
                    else:
                        if not np.all(np.isnan(self.scan_history[-3,])):
                            name = 'Size dist. - 3'                        
                            self.curve_dict[name].setData(self.plot_data['Dp_list'],self.scan_history[-3,])
                    
                    
                    if 'Size dist. - 2' not in self.curve_dict:
                        # Check that your not trying to plot nan values (will result in an error)
                        if not np.all(np.isnan(self.scan_history[-2,])):
                            name = 'Size dist. - 2'
                            self.curve_dict[name] = self.p5.plot(self.plot_data['Dp_list'],self.scan_history[-2,],pen = Pen3)
                            self.legendp5.addItem(self.curve_dict[name],name = name)
                    else:
                        if not np.all(np.isnan(self.scan_history[-2,])):
                            name = 'Size dist. - 2'                        
                            self.curve_dict[name].setData(self.plot_data['Dp_list'],self.scan_history[-2,])


                    if 'Size dist. - 1' not in self.curve_dict:
                        # Check that your not trying to plot nan values (will result in an error)
                        if not np.all(np.isnan(self.scan_history[-1,])):
                            name = 'Size dist. - 1'
                            self.curve_dict[name] = self.p5.plot(self.plot_data['Dp_list'],self.scan_history[-1,],pen = Pen2)
                            self.legendp5.addItem(self.curve_dict[name],name = name)
                    else:
                        if not np.all(np.isnan(self.scan_history[-1,])):
                            name = 'Size dist. - 1'                        
                            self.curve_dict[name].setData(self.plot_data['Dp_list'],self.scan_history[-1,])
                       
                elif (dev.child('Device type').value() == 2) & (dev['Connected']): # Other CPC               
                    # Update concentration reading to measurement status window
                    # set data i.e. update plot (now plot is not present, as connection has been established after start timer)
                    if dev.child('DevID').value() not in self.curve_dict:
                        name = dev.children()[0].value()                        
                        self.curve_dict[dev.child('DevID').value()] = self.p1.plot(self.plot_data[dev.child('DevID').value()],pen = dev.child('DevID').value())
                        self.legendp1.addItem(self.curve_dict[dev.child('DevID').value()],name = name)
                    else:
                        self.curve_dict[dev.child('DevID').value()].setData(self.plot_data[dev.child('DevID').value()])
                    
                    # if curves name in legend does not match the device name
                    # remove the legend, and rewrite it to match
                    name = dev.children()[0].value()
                    if self.curve_dict[1].name() !=  name:    
                        self.legendp1.removeItem(self.curve_dict[dev.child('DevID').value()])
                        self.legendp1.addItem(self.curve_dict[dev.child('DevID').value()],name = name)

    def get_ref_cpc_data(self):
        # find the connection of the ref CPC
        for dev in self.params.child('Before starting').child('Device settings').children():
            # if device is connected
            if dev.child('Connected').value():
                if dev.child('Device type').value() == 5:
                    # send a command :SYST:POUT to get the data
                    dev.children()[3].value().send_message(self.message7)
            time.sleep(1)
            # read the data from the connection
            self.refConnection = self.params.child('Before starting').child('Device settings').child('Ref CPC connection').children()[3].value()
            
            try:
                temp = dev.children()[3].value().read_all().decode()
                temp = temp.split(" ")[1][:-1]
                temp = temp.split(",")
                # check the correct length of returned data
                if len(temp) == 28:
                    self.internal_Q_sample = temp[24]
            except:
                print("Failed to read Ref CPC data")                  
                
    def header_check(self,filename):
        # Check the type and length of header
        write_headers = 0
        with open(filename, 'r') as out_file:
            out_file.seek(0) 
            header_row1 = out_file.readline()
            header_len = len(header_row1)
            # At the moment only check is a header exists
            if header_len == 0:
                write_headers = 1
        return write_headers    
    
    def write_data(self):
        
        # Data for 1 Hz logging
        # If saving is on open file
        if self.params.child('Before starting').child('Data settings').child('Save data').value():

            current_time = dt.datetime.now().timetuple()
                       
            if self.file_start_tim.timetuple().tm_yday != current_time.tm_yday:
                self.filename_check()
                                
            write_headers = self.header_check(self.filename)
            
            with open(self.filename, 'a') as out_file:
            #with open(filename, 'a') as out_file:
            
                # Write headers if they don't exist
                if write_headers == 1:
                    out_file.write('YYYY,MM,DD,hh,mm,ss,ms')
                    for dev in self.params.child('Before starting').child('Device settings').children():
                        # if device is connected
                        if dev.child('Connected').value():
                            if dev.child('Device type').value() == 1: # DMA
                                write_data = ',Sheath RH(%),Sheath Temp(C),Sheath Pressure(Pa),Sheath Flow(lpm),Sheath Flow Setpoint(lpm),Sheath air on,HV on,DMA setpoint V, DMA controller errors,'
                                out_file.write(write_data)                        
                            elif dev.child('Device type').value() == 5: # Ref CPC
                                write_data = 'Device name,Concentration(#/cc),Pulses (N),Dead time (ms),Pulses C2 (N),Avg time (s),Sat temp (C),Optics temp (C),Condenser temp (C),Inlet temp (C),Inlet pres. (kPa),Critikal pres. (kPa),Nozzle pres. (kPa),Ambient pres. (kPa),Wrem pres. (kPa),Liquid level, CPC status,'
                                out_file.write(write_data)         
                            elif dev.child('Device type').value() == 2: # New rall
                                write_data = 'Device name,Concentration(#/cc),Pulses (N),Dead time (ms),Pulses C2 (N),Avg time (s),Sat temp (C),Optics temp (C),Condenser temp (C),Inlet temp (C),Inlet pres. (kPa),Critikal pres. (kPa),Nozzle pres. (kPa),Ambient pres. (kPa),Wrem pres. (kPa),Liquid level, CPC status,'
                                out_file.write(write_data)         
                    out_file.write("\r")

                # Write the actual data
                # Add timestamp
                tim = dt.datetime.now()
                # Add fractional seconds to timestamp
                timeStampStr = str(tim.strftime("%Y,%m,%d,%H,%M,%S"))+','+str(tim.microsecond//1000)
#                timeStampStr = str(tim.strftime("%Y,%m,%d,%H,%M,%S"))
                out_file.write(timeStampStr+',')
                
                # Check that the number of data matches the headers
                # If not, then new file and appropriate headers
                # Add data from each device
                for dev in self.params.child('Before starting').child('Device settings').children():
                    # if device is connected
                    if dev.child('Connected').value():
                        if dev.child('Device type').value() == 1: # DMA
                            # Convert data to string                            
                            write_data = ','.join(str(vals) for vals in self.latest_data[dev.child('DevID').value()])
                            out_file.write(write_data+',')
                        elif dev.child('Device type').value() == 5: # Ref CPC
                            # Convert data to string
                            out_file.write(str(dev.children()[0].value())+',')
                            write_data = ','.join(str(vals) for vals in self.latest_data[dev.child('DevID').value()])
                            out_file.write(write_data+',')
                        elif dev.child('Device type').value() == 2: # Other CPC
                            # Convert data to string
                            out_file.write(str(dev.children()[0].value())+',')
                            write_data = ','.join(str(vals) for vals in self.latest_data[dev.child('DevID').value()])
                            out_file.write(write_data+',')
                out_file.write("\r")
  

    # Start the timer or scheduler    
    def initGUI(self):
        
        # Create and set central widget
        self.main_splitter = QSplitter()
        self.setCentralWidget(self.main_splitter)

        # Create logo label
        self.logo = QLabel(alignment=Qt.AlignCenter)
        
        # Get the directory of the current script
        script_dir = os.path.dirname(os.path.realpath(__file__))
        
        # Define the relative path to your image
        rel_path = "res/Airmodus_white.png"
        
        # Join the script directory with the relative path to get the absolute path
        abs_file_path = os.path.join(script_dir, rel_path)
        
        self.pixmap = QPixmap(abs_file_path)  # Replace with the actual path
        self.logo.setPixmap(self.pixmap.scaled(400, 100, Qt.KeepAspectRatio, Qt.SmoothTransformation))

        ## Create two ParameterTree widgets, both accessing the same data
        # One for plots, and the other for the controls
        self.t = ParameterTree(showHeader=False)
        self.t.setParameters(self.params, showTop=False)
        
        # Create status lights widget 
        self.status_lights = StatusLights()
        self.status_lights.set_error_light(1) # update error status light according to error_status flag
        self.status_lights.set_saving_light(0) # update saving status light according to saving_status flag

        # Create left side vertical splitter
        left_splitter = QSplitter(Qt.Vertical)
        left_splitter.addWidget(self.logo)
        left_splitter.addWidget(self.t)
        left_splitter.addWidget(self.status_lights)
        left_splitter.setSizes([100, 800, 100])

        self.main_splitter.addWidget(left_splitter)
        
        self.init_size_dist_data()
        self.scan_history = np.zeros([20,len(self.dp_list)])
        self.set_voltage_flag = 0
        self.set_voltage_value = 0

        # Two larger plots on the left
        left_plot_splitter = QSplitter(Qt.Vertical)
        self.p5 = pg.PlotWidget(title="Raw data - no inversion applied")
        self.p5.getAxis('left').enableAutoSIPrefix(False)
        self.p5.getAxis('bottom').enableAutoSIPrefix(False)

        self.p2 = pg.PlotWidget()
        self.p2.getAxis('left').enableAutoSIPrefix(False)
        left_plot_splitter.addWidget(self.p5)
        left_plot_splitter.addWidget(self.p2)
        
        # Set labels, ranges, and other properties for each plot
        # Create splitter for the three smaller plots on the right
        right_plot_splitter = QSplitter(Qt.Vertical)
        self.p1 = pg.PlotWidget()
        self.p1.getAxis('left').enableAutoSIPrefix(False)
        self.p3 = pg.PlotWidget()
        self.p3.getAxis('left').enableAutoSIPrefix(False)
        self.p4 = pg.PlotWidget()
        self.p4.getAxis('left').enableAutoSIPrefix(False)
        right_plot_splitter.addWidget(self.p1)
        right_plot_splitter.addWidget(self.p3)
        right_plot_splitter.addWidget(self.p4)

        # Add the two splitters to the main splitter
        self.main_splitter.addWidget(left_plot_splitter)
        self.main_splitter.addWidget(right_plot_splitter)

        self.main_splitter.setSizes([300, 700,300]) # set relative sizes of widgets
        self.resize(1400, 800)

        self.p5.setLabel('bottom', "Mobility diameter", units='nm')
        self.p5.setLabel('left', "Concentration", units='#/cm^3',)
        self.p5.setLogMode(x=True, y=False)
        self.p5.showGrid(x=True, y=True)
        self.legendp5 = pg.LegendItem(offset=(100, 10),labelTextSize = '10pt')
        self.legendp5.setParentItem(self.p5.graphicsItem())
               
        self.p2.setLabel('bottom', "Scan time")
        self.p2.setLabel('left', "Mobility diameter", units='nm')
        Z = np.zeros([20,len(self.dp_list)])
        [X,Y] = self.update_image()
        self.img = pg.PColorMeshItem(X, Y ,Z)
        self.p2.addItem(self.img)

        self.p1.setLabel('left', "Concentration", units='#/cm^3')
        self.p1.setLabel('bottom', "Time", units='s')
        self.p1.showGrid(x=True, y=True)        
        self.legendp1 = pg.LegendItem(offset=(100, 10),labelTextSize = '10pt')
        self.legendp1.setParentItem(self.p1.graphicsItem())

        self.p3.setLabel('bottom', "Time", units='s')
        self.p3.setLabel('left', "Sheath flow", units='lpm')
        self.p3.showGrid(x=True, y=True)
        self.legendp3 = pg.LegendItem(offset=(100, 10),labelTextSize = '10pt')
        self.legendp3.setParentItem(self.p3.graphicsItem())

        self.p4.setLabel('bottom', "Time", units='s')
        self.p4.setLabel('left', "Sheath conditions")
        self.p4.showGrid(x=True, y=True)
        self.legendp4 = pg.LegendItem(offset=(100, 10),labelTextSize = '10pt')
        self.legendp4.setParentItem(self.p4.graphicsItem())
        
        self.com_port_changed()

        # Init plot_data for plotted values
        for dev in self.params.child('Before starting').child('Device settings').children():
            # if device is connected
            if dev.child('Connected').value():
                if dev.child('Device type').value() == 5: # CPC
                    try:
                        name = dev.children()[0].value()
                        self.curve_dict[dev.child('DevID').value()] = self.p1.plot(self.plot_data[dev.child('DevID').value()])
                        self.legendp1.addItem(self.curve_dict[dev.child('DevID').value()],name = name)
                    except:
                        pass
                if dev.child('Device type').value() == 2: # CPC
                    try:
                        name = dev.children()[0].value()
                        self.curve_dict[dev.child('DevID').value()] = self.p1.plot(self.plot_data[dev.child('DevID').value()])
                        self.legendp1.addItem(self.curve_dict[dev.child('DevID').value()],name = name)
                    except:
                        pass
                # 
                if dev.child('Device type').value() == 1: # DMA
                    # Plot for blower
                    try:
                        name = dev.children()[0].value()
                        self.curve_dict[dev.child('DevID').value()] = self.p3.plot(self.plot_data[dev.child('DevID').value()])
                        self.legendp3.addItem(self.curve_dict[dev.child('DevID').value()],name = name)
                        
                        self.curve_dict['RH'] = self.p3.plot(self.plot_data['RH'])
                        self.legendp4.addItem(self.curve_dict['RH'],name = 'RH (%)')
                    except:
                        print("Figure init failed")
                        pass

                    # Plot for size distribution
                    try:
                        name = 'Size dist. - Current'#'Mobility size distribution'
                        self.curve_dict['Size dist. - Current'] = self.p5.plot(self.plot_data['Dp_list'],self.plot_data['Size dist conc'])
                        self.legendp5.addItem(self.curve_dict['Size dist. - Current'],name = name)
                    except:
                        print("Figure init failed")
                        pass

        # Start the timer and list the functions the timer triggers
        self.timer.timeout.connect(self.main_loop)

        # Load the initial settings file
        self.load_ini()
        
        # show graphic windows        
        self.main_splitter.show()

    def startTimer(self):
        self.timer.start(int(self.time_step*1000))  # Convert seconds to milliseconds

    def com_port_changed(self):
        for dev in self.params.child('Before starting').child('Device settings').children():
            try:
                dev.children()[1].sigValueChanged.connect(self.connection_test)
            except:
                pass

    # Function to stop the timer - note not the same as closing the program        
    def endTimer(self):
        self.timer.stop()
        
    def set_port_list(self):
        self.scan_ports()
        for devs in self.params.child('Before starting').child('Device settings').children():
            try:
                available_comports = self.com_dict
                devs.children()[1].setLimits(available_comports)
            except:
                pass

    def scan_ports(self):
        # Function to scan ports and list them in the parameter tree
        # List all the ports
        self.closeAllPorts()
        self.com_dict = {'Select device':'NaN'}
        ports = list(serial.tools.list_ports.comports())
        # send and read the command '*IDN?' to all the ports
        ser_list = np.zeros(len(ports)).tolist()

        # Arduino boots up every time you create the connection, hence this
        # complex rutine: list ports->ask id from all->wait->read all    
        for iterator,p in enumerate(ports):
            try:
                ser_list[iterator] = serial.Serial(p[0], 115200, timeout=0.1)
                time.sleep(1)
                ser_list[iterator].write(b'*IDN?\r')
            except:
                pass
        time.sleep(1)
        for iterator,p in enumerate(ports):
            try:                
                # answer = ser_list[iterator].read_all().decode()
                answer = str(ser_list[iterator].read_all())
                ser_list[iterator].close()
                print(answer)
                if '*IDN' in answer:
                    dev_id = answer.split('*IDN ')[1].split('\\')[0]
                    # Set ID as the device name and port as the value
                    self.com_dict[dev_id] = p[0]

            except:
                print('no fail')
                pass


def main():
    app = QApplication([])
    window = MainWindow()
    window.show()
    window.initGUI()
    window.set_port_list()
    window.startTimer()
    #app.exec()
    sys.exit(app.exec())
    return window

if __name__ == '__main__':
    foo = main()
