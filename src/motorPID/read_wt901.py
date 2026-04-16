# coding: UTF-8
"""
WT901C485 reader mengikuti struktur script contoh SDK WT901C485.py
"""
import datetime
import os
import platform
import sys
import threading
import time

# =====================================================
# PYTHON PATH -> folder chs lokal project
# =====================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS = os.path.abspath(os.path.join(BASE_DIR, "..", "Python-SDK-WT901C485", "chs"))
sys.path.insert(0, SDK_CHS)

import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver

welcome = """
Welcome to WT901C485 sample-style reader
"""

_writeF = None
_IsWriteF = False


def readConfig(device):
    """
    Read configuration example
    """
    vals = device.readReg(0x02, 3)
    if len(vals) > 0:
        print("Config 0x02.. :", vals)
    else:
        print("Config 0x02.. : no response")

    vals = device.readReg(0x23, 2)
    if len(vals) > 0:
        print("Config 0x23.. :", vals)
    else:
        print("Config 0x23.. : no response")


def setConfig(device):
    """
    Set configuration example
    """
    device.unlock()
    time.sleep(0.1)
    device.writeReg(0x03, 6)  # return rate
    time.sleep(0.1)
    device.writeReg(0x23, 0)  # installation direction
    time.sleep(0.1)
    device.writeReg(0x24, 0)  # algorithm
    time.sleep(0.1)
    device.save()


def accelerationCalibration(device):
    """
    Accelerometer calibration
    """
    device.AccelerationCalibration()
    print("Acceleration calibration done")


def filedCalibration(device):
    """
    Magnetic field calibration
    """
    device.BeginFiledCalibration()
    ans = input("Rotate slowly around X/Y/Z once. End calibration now (Y/N)? ").strip().lower()
    if ans == "y":
        device.EndFiledCalibration()
        print("Field calibration done")


def startRecord():
    """
    Start data recording
    """
    global _writeF, _IsWriteF
    filename = datetime.datetime.now().strftime("%Y%m%d%H%M%S") + ".txt"
    _writeF = open(filename, "w")
    _IsWriteF = True

    header = "Chiptime"
    header += "\tax(g)\tay(g)\taz(g)"
    header += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
    header += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
    header += "\tT(C)"
    header += "\tmagx\tmagy\tmagz"
    header += "\r\n"
    _writeF.write(header)
    print(f"Start recording data -> {filename}")


def endRecord():
    """
    End data recording
    """
    global _writeF, _IsWriteF
    _IsWriteF = False
    if _writeF is not None:
        _writeF.close()
        _writeF = None
    print("Stop recording data")


def onUpdate(dev):
    """
    Data update callback
    """
    chiptime = dev.getDeviceData("Chiptime")
    temperature = dev.getDeviceData("temperature")
    accx = dev.getDeviceData("accX")
    accy = dev.getDeviceData("accY")
    accz = dev.getDeviceData("accZ")
    gyrox = dev.getDeviceData("gyroX")
    gyroy = dev.getDeviceData("gyroY")
    gyroz = dev.getDeviceData("gyroZ")
    anglex = dev.getDeviceData("angleX")
    angley = dev.getDeviceData("angleY")
    anglez = dev.getDeviceData("angleZ")
    magx = dev.getDeviceData("magX")
    magy = dev.getDeviceData("magY")
    magz = dev.getDeviceData("magZ")

    print(
        "Chiptime:", chiptime,
        " Temp:", temperature,
        " Acc:", f"{accx},{accy},{accz}",
        " Gyro:", f"{gyrox},{gyroy},{gyroz}",
        " Angle:", f"{anglex},{angley},{anglez}",
        " Mag:", f"{magx},{magy},{magz}",
    )

    if _IsWriteF and _writeF is not None:
        row = " " + str(chiptime)
        row += "\t" + str(accx) + "\t" + str(accy) + "\t" + str(accz)
        row += "\t" + str(gyrox) + "\t" + str(gyroy) + "\t" + str(gyroz)
        row += "\t" + str(anglex) + "\t" + str(angley) + "\t" + str(anglez)
        row += "\t" + str(temperature)
        row += "\t" + str(magx) + "\t" + str(magy) + "\t" + str(magz)
        row += "\r\n"
        _writeF.write(row)


def loopReadThread(device):
    """
    Cyclic read data
    """
    while True:
        device.readReg(0x30, 41)


if __name__ == "__main__":
    print(welcome)

    device = deviceModel.DeviceModel(
        "WT901",
        Protocol485Resolver(),
        JY901SDataProcessor(),
        "51_0",
    )

    device.ADDR = 0x50
    if platform.system().lower() == "linux":
        device.serialConfig.portName = "/dev/ttyUSB0"
    else:
        device.serialConfig.portName = "/dev/tty.usbserial-1330"
    device.serialConfig.baud = 9600
    device.openDevice()

    readConfig(device)
    device.dataProcessor.onVarChanged.append(onUpdate)

    startRecord()
    t = threading.Thread(target=loopReadThread, args=(device,), daemon=True)
    t.start()

    try:
        print("Press Enter to stop...")
        input()
    finally:
        device.closeDevice()
        endRecord()
