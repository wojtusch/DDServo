#!/usr/bin/env python

import os, serial, sys, platform

from PyQt4 import uic
from PyQt4.QtGui import QApplication, QMainWindow, QIcon
from PyQt4.QtCore import SIGNAL

# Load main window
scriptPath = os.path.dirname(os.path.realpath( __file__ ))
(MainWindowClass, MainWindowBaseClass) = uic.loadUiType(os.path.join(scriptPath, 'DDFlashTool.ui'))

class DDFlashTool(QMainWindow, MainWindowClass):
    serialPorts = {
        'Linux': ['/dev/ttyUSB%d' % i for i in range(10)],
        'Darwin': ['/dev/tty.usbserial'] + ['/dev/tty.usbserial%d' % i for i in range(10)],
    }

    def __init__(self, parent = None):
        QMainWindow.__init__(self)
        self.setupUi(self)
	self.setWindowIcon(QIcon('DDFlashTool.png'))
	self.setWindowIcon(QIcon(os.path.join(scriptPath, 'DDFlashTool.png')))
        
        # Set variables
        self.filePath = os.path.join(scriptPath, 'DD28Firmware.bin')
        self.clearAll = 0
        
        # Set UI components
        self.progressWriteFirmware.hide()
        self.progressWriteFirmware.setValue(0)
        self.labelWriteFirmware.hide()
        self.lineFilePath.setText(self.filePath)
        self.comboSerialPort.clear()
        for port in self.scanSerialPorts():
            self.comboSerialPort.addItem(port[0], port[1])
        
        baudrateList = [9600, 19200, 57600, 115200, 200000, 250000, 400000, 500000, 1000000, 2000000]
        self.comboComBaudrate.clear()
        for baudrate in baudrateList:
          self.comboComBaudrate.addItem(str(baudrate), str(baudrate))
        self.comboComBaudrate.setCurrentIndex(8)
        
        baudrateList = [1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 56000, 57600, 115200]
        self.comboProgBaudrate.clear()
        for baudrate in baudrateList: 
          self.comboProgBaudrate.addItem(str(baudrate), str(baudrate))
        self.comboProgBaudrate.setCurrentIndex(9)
        
        # Connect signals
        self.connect(self.checkClearAll, SIGNAL('toggled(bool)'), self.toggleClearAllCheckbox)
        self.connect(self.lineServoId, SIGNAL('editingFinished()'), self.evaluateServoIdEditLine)
        self.connect(self.pushWriteFirmware, SIGNAL('clicked()'), self.writeFirmware)
    
    # Scans for available serial ports and returns a list of tuples with port number and name
    def scanSerialPorts(self):
        availableSerialPorts = []
        for port in self.serialPorts.get(platform.system(), range(20)):
            try:
                serialPort = serial.Serial(port)
                availableSerialPorts.append((serialPort.portstr, port))
                serialPort.close()
            except serial.SerialException:
                pass
        return availableSerialPorts
    
    # Evaluate the servo ID value
    def evaluateServoIdEditLine(self):
        try:
            if int(self.lineServoId.text()) > 254:
                self.lineServoId.setText('254')
        except:
            self.lineServoId.setText('254')
    
    # Toggle the state of the clear all flag according to the checkbox value
    def toggleClearAllCheckbox(self, checked):
        checkbox = self.sender()
        if checked:
            self.clearAll = 1
            checkbox.setStyleSheet("color:red;")
        else:
            self.clearAll = 0
            checkbox.setStyleSheet("color:black;")
    
    # Update status of serial thread
    def updateStatus(self, status, checksum):
        if status == 0:
            # Set UI components
            self.labelSerialPort.setEnabled(True)
            self.comboSerialPort.setEnabled(True)
            self.labelComBaudrate.setEnabled(True)
            self.comboComBaudrate.setEnabled(True)
            self.labelProgBaudrate.setEnabled(True)
            self.comboProgBaudrate.setEnabled(True)
            self.labelServoId.setEnabled(True)
            self.lineServoId.setEnabled(True)
            self.pushWriteFirmware.setEnabled(True)
            self.checkClearAll.setEnabled(True)
            self.checkClearAll.show()
            self.checkSendBootCommand.setEnabled(True)
            self.checkSendBootCommand.show()
            self.progressWriteFirmware.hide()
            self.progressWriteFirmware.setValue(0)
            self.labelWriteFirmware.hide()
            self.labelFilePath.show()
            self.lineFilePath.show()
        elif status == 1:
            # Serial port error
            self.labelStatus.setText("<font color='red'>Could not connect to serial port.</font>")
        elif status == 2:
            # Firmware file error
            self.labelStatus.setText("<font color='red'>Could not read firmware file.</font>")
        elif status == 3:
            # Firmware file status
            self.labelWriteFirmware.setText("Erasing flash memory of device.")
            self.labelStatus.setText("<font color='black'>Writing firmware file is in progress.</font>")
        elif status == 4:
            # Firmware file status
            self.labelWriteFirmware.setText("Writing firmware file to flash memory of device.")
            self.labelStatus.setText("<font color='black'>Writing firmware file is in progress.</font>")
        elif status == 5:
            # Firmware file and checksum status
            self.labelStatus.setText("<font color='green'>Firmware was written to device and checksum was verified with a value of " + str(checksum) + ".</font>")
        elif status == 6:
            # Checksum error
            self.labelStatus.setText("<font color='red'>Firmware was written to device, but checksum could not be verified. Checksum is supposed to have a value of " + str(checksum) + ".</font>")
        elif status == 7:
            # Checksum error
            self.labelStatus.setText("<font color='red'>Firmware was written to device, but checksum did not match. Checksum is supposed to have a value of " + str(checksum) + ".</font>")
        elif status == 8:
            # Firmware file and checksum status with broadcast
            self.labelStatus.setText("<font color='green'>Firmware was written to device, but checksum could not be verified due to broadcast operation. Checksum is supposed to have a value of " + str(checksum) + ".</font>")
    
    # Update progress visualization
    def updateProgress(self, maximumProgress, currentProgress):
        self.progressWriteFirmware.setMaximum(maximumProgress)
        self.progressWriteFirmware.setValue(currentProgress)
    
    # Writes the firmware to the connected device
    def writeFirmware(self):
        # Set UI components
        self.labelSerialPort.setEnabled(False)
        self.comboSerialPort.setEnabled(False)
        self.labelComBaudrate.setEnabled(False)
        self.comboComBaudrate.setEnabled(False)
        self.labelProgBaudrate.setEnabled(False)
        self.comboProgBaudrate.setEnabled(False)
        self.labelServoId.setEnabled(False)
        self.lineServoId.setEnabled(False)
        self.pushWriteFirmware.setEnabled(False)
        self.checkClearAll.setEnabled(False)
        self.checkClearAll.hide()
        self.checkSendBootCommand.setEnabled(False)
        self.checkSendBootCommand.hide()
        self.labelFilePath.hide()
        self.lineFilePath.hide()
        self.progressWriteFirmware.show()
        self.progressWriteFirmware.setValue(0)
        self.labelWriteFirmware.show()
        self.labelStatus.setText('')
        
        # Write firmware
        self.emit(SIGNAL('writeFirmware(QString, QString, QString, QString, QString, int, bool)'), str(self.comboSerialPort.currentText()), self.comboComBaudrate.currentText(), self.comboProgBaudrate.currentText(), str(self.lineServoId.text()), self.lineFilePath.text(), self.clearAll, self.checkSendBootCommand.isChecked())

import serial, struct, time, operator

from PyQt4.QtCore import SIGNAL, QThread

class DDSerialThread(QThread):
    
    def __init__(self, parent):
        QThread.__init__(self)
        
        # Set variables
        self.parent = parent
        self.dataAddress = 0x08000000 
        self.bootAddress = 0x08000000
        self.settingPages = 4
        self.flashSize = 0x0801FFFF - self.settingPages * 0x400 - self.dataAddress + 1
        self.broadcastId = 254
        self.connected = False
        self.readable = False
        self.bootloader = False
        self.dataString = ''
        self.checksumString = ''
        self.answerPacket = ''
        self.dataBlocks = []
        self.firmwareChecksum = 0
        self.answerChecksum = 0
        self.numberofBlocks = 0

    def __del__(self):
        if(self.connected):
            self.closeSerialPort()
    
    # Opens a serial port specified with port name and a defined baudrate
    def openSerialPort(self, serialPortName, serialBaudrate, programmingConnection):
        try:
            if programmingConnection:
                self.serialPort = serial.Serial(
                    port = serialPortName,
                    baudrate = serialBaudrate,
                    bytesize = 8,
                    parity = serial.PARITY_EVEN,
                    stopbits = 1,
                    xonxoff = 0,
                    rtscts = 0,
                    timeout = 0.5
                )
            else:
                self.serialPort = serial.Serial(
                    port = serialPortName,
                    baudrate = serialBaudrate,
                    bytesize = 8,
                    parity = serial.PARITY_NONE,
                    stopbits = 1,
                    xonxoff = 0,
                    rtscts = 0,
                    timeout = 0.5
                )
            self.connected = True
        except serial.serialutil.SerialException:
            self.connected = False
            self.emit(SIGNAL('updateStatus(int, int)'), 1, 0)
    
    # Closes a serial port
    def closeSerialPort(self):
        self.serialPort.close()
        self.connected = False
    
    # Reads a firmware file and calculates firmware checksum
    def readFirmware(self, filePath):
        try:
            self.dataString = open(filePath, 'rb').read()
            if (len(self.dataString) % 4) > 0:
                self.dataString += '\xFF' * (4 - (len(self.dataString) % 4))
            self.dataBlocks = self.splitInBlocks(self.dataString, 256)
            self.checksumString = self.dataString
            if len(self.checksumString) < self.flashSize:
                self.checksumString += '\xFF' * (self.flashSize - len(self.checksumString))
            self.firmwareChecksum = 0
            for i in self.checksumString:
                self.firmwareChecksum += ord(i)
                self.firmwareChecksum &= 0xFFFF
            self.firmwareChecksum = ~self.firmwareChecksum + 65536
            self.readable = True
        except IOError:
            self.readable = False
            self.emit(SIGNAL('updateStatus(int, int)'), 2, 0)
        
    # Calculates the packet checksum a string without start bytes and checksum
    def packetChecksum(self, string):
        sum = 0
        for i in string:
            sum += ord(i)
        return ~sum & 0xFF
    
    # Calculates the bootloader checksum for a string
    def bootloaderChecksum(self, string):
        return chr(reduce(operator.xor, [ord(i) for i in string]))
    
    # Splits a given string in seperate blocks with a specified block size
    def splitInBlocks(self, string, blockSize): 
        return [string[i : i + blockSize] for i in xrange(0, len(string), blockSize)]
    
    # Initiates writing process
    def writeFirmware(self, serialPortName, serialComBaudrate, serialProgBaudrate, servoId, filePath, clearAll, startBootloader):
        self.serialPortName = str(serialPortName)
        self.serialComBaudrate = str(serialComBaudrate)
        self.serialProgBaudrate = str(serialProgBaudrate)
        self.servoId = str(servoId)
        self.filePath = str(filePath)
        self.clearAll = clearAll
        self.startBootloader = startBootloader
        self.start()
    
    # Writes the firmware to the connected device
    def run(self):
        # Read firmware file
        self.readFirmware(self.filePath)
        # Start bootloader on device
        if self.startBootloader and self.readable:
            self.openSerialPort(self.serialPortName, self.serialComBaudrate, False)
            if self.connected:        
                # Bootloader command
                self.commandString = chr(int(self.servoId)) + '\x06\x00\x01\x02\x03\x04'
                self.serialPort.write('\xFF\xFF' + self.commandString + chr(self.packetChecksum(self.commandString)))
                time.sleep(0.5)
                self.bootloader = True
                self.closeSerialPort()
        else:
            self.bootloader = True
        
        # Write firmware to the connected device
        if self.bootloader and self.readable:
            self.openSerialPort(self.serialPortName, self.serialProgBaudrate, True)
            self.numberOfBlocks = len(self.dataBlocks)
            if self.connected and self.readable:
                # Auto baudrate detaction command
                self.serialPort.write('\x7F')
                time.sleep(0.1)
                # Erase command
                self.emit(SIGNAL('updateStatus(int, int)'), 3, 0)
                if self.clearAll:
                    self.serialPort.write('\x43\xBC')
                    self.serialPort.write('\xFF\x00')
                    for self.block in range(20):
                        time.sleep(0.05)
                        self.emit(SIGNAL('updateProgress(int, int)'), 20, (self.block + 1))
                else:
                    self.lengthString = chr(128 - self.settingPages - 1)
                    self.dataBlock = ''
                    for self.number in range(128 - self.settingPages):
                        self.dataBlock += chr(self.number)
                    self.dataChecksum = self.bootloaderChecksum(self.lengthString + self.dataBlock)
                    self.serialPort.write('\x43\xBC')
                    self.serialPort.write(self.lengthString + self.dataBlock + self.dataChecksum)
                    for self.block in range(128 - self.settingPages):
                        time.sleep(0.05)
                        self.emit(SIGNAL('updateProgress(int, int)'), (128 - self.settingPages), (self.block + 1))
                # Write command
                self.block = 1
                self.emit(SIGNAL('updateProgress(int, int)'), self.numberOfBlocks, 0)
                self.emit(SIGNAL('updateStatus(int, int)'), 4, 0)
                for self.dataBlock in self.dataBlocks:
                    self.addressString = struct.pack('>I', self.dataAddress)
                    self.addressChecksum = self.bootloaderChecksum(self.addressString)
                    self.dataAddress += len(self.dataBlock)
                    self.lengthString = chr(len(self.dataBlock) - 1)
                    self.dataChecksum = self.bootloaderChecksum(self.lengthString + self.dataBlock)
                    self.serialPort.write('\x31\xCE')
                    self.serialPort.write(self.addressString + self.addressChecksum)
                    self.serialPort.write(self.lengthString + self.dataBlock + self.dataChecksum)
		    time.sleep(0.06)
                    self.emit(SIGNAL('updateProgress(int, int)'), self.numberOfBlocks, self.block)
                    self.block += 1
                time.sleep(0.1)
                # Go command
                self.addressString = struct.pack('>I', self.bootAddress)
                self.addressChecksum = self.bootloaderChecksum(self.addressString)
                self.serialPort.write('\x21\xDE')
                self.serialPort.write(self.addressString + self.addressChecksum)
                self.closeSerialPort()
                time.sleep(1.0)
                # Verify checksum
                if int(self.servoId) == self.broadcastId:
                    self.emit(SIGNAL('updateStatus(int, int)'), 8, self.firmwareChecksum)
                else:
                    self.openSerialPort(self.serialPortName, self.serialComBaudrate, False)
                    if self.connected:        
                        if self.clearAll:
                            # Checksum command for resetted servo id
                            self.commandString = '\x00\x02\x1C'
                        else:
                            # Checksum command for defined servo id
                            self.commandString = chr(int(self.servoId)) + '\x02\x1C' 
                        self.serialPort.write('\xFF\xFF' + self.commandString + chr(self.packetChecksum(self.commandString)))
                        # Read answer packet
                        self.answerPacket = ''
                        self.answerByte = ''
                        while True:
                            self.answerByte = self.serialPort.read(1)
                            if self.answerByte == '':
                                # Timeout occured
                                self.emit(SIGNAL('updateStatus(int, int)'), 6, self.firmwareChecksum)
                                break

                            self.answerPacket += self.answerByte

                            # Answer packet has to start with two 0xFF
                            while len(self.answerPacket) >= 2 and (ord(self.answerPacket[0]) != 0xFF and ord(self.answerPacket[1]) != 0xFF):
                                self.answerPacket = self.answerPacket[1:]

                            # ID cannot be 0xFF
                            if len(self.answerPacket) == 3 and ord(self.answerPacket[2]) == 0xFF:
                                self.answerPacket = self.answerPacket[1:]

                            # Packet length cannot be smaller than 0x02
                            elif len(self.answerPacket) >= 4 and ord(self.answerPacket[3]) < 0x02:
                                self.answerPacket = self.answerPacket[1:]

                            # Answer packet is fully received
                            elif len(self.answerPacket) == 8 and len(self.answerPacket) == 0x04 + ord(self.answerPacket[3]):
                                if self.packetChecksum(self.answerPacket[2:-1]) == ord(self.answerPacket[-1]):
                                    self.answerChecksum = struct.unpack('<H', self.answerPacket[5:7])[0]
                                    if self.answerChecksum == self.firmwareChecksum:
                                        # Firmware checksum is correct
                                        self.emit(SIGNAL('updateStatus(int, int)'), 5, self.firmwareChecksum)
                                    else:
                                        # Firmware checksum is incorrect
                                        self.emit(SIGNAL('updateStatus(int, int)'), 7, self.firmwareChecksum)
                                else:
                                    # Packet checksum is incorrect
                                    self.emit(SIGNAL('updateStatus(int, int)'), 6, self.firmwareChecksum)
                                break
                    else:
                        # Could not connect to serial port
                        self.emit(SIGNAL('updateStatus(int, int)'), 6, self.firmwareChecksum)
                        return
                    self.closeSerialPort()
        
        # Set variables
        self.dataAddress = 0x08000000
        self.connected = False
        self.readable = False
        self.bootloader = False
        self.dataString = ''
        self.dataBlocks = []
        self.numberOfBlocks = 0
        
        self.emit(SIGNAL('updateStatus(int, int)'), 0, 0)
        
# Start application
application = QApplication(sys.argv)
mainWindow = DDFlashTool(parent = application)
mainWindow.show()

# Start serial thread
serialThread = DDSerialThread(application)

# Connect signals
application.connect(mainWindow, SIGNAL('writeFirmware(QString, QString, QString, QString, QString, int, bool)'), serialThread.writeFirmware)
application.connect(serialThread, SIGNAL('updateProgress(int, int)'), mainWindow.updateProgress)
application.connect(serialThread, SIGNAL('updateStatus(int, int)'), mainWindow.updateStatus)

# Start main thread
sys.exit(application.exec_())
