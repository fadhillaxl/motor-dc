"""
wt901_scan.py - find active sensor address
Connect ONLY ONE sensor at a time
"""
import os, sys, time, platform

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_CHS  = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "Python-SDK-WT901C485", "chs"))
sys.path.insert(0, SDK_CHS)

import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver

PORT = "/dev/ttyUSB0"
CANDIDATES = [0x50, 0x01, 0x02, 0xFF]   # common defaults

def make_device(addr):
    try:
        dev = deviceModel.DeviceModel("WT901C485", Protocol485Resolver(), JY901SDataProcessor())
    except TypeError:
        dev = deviceModel.DeviceModel("WT901C485", Protocol485Resolver(), JY901SDataProcessor(), "AZ")
    dev.serialConfig.portName = PORT
    dev.serialConfig.baud = 9600
    dev.ADDR = addr
    return dev

print("Scanning... connect only ONE sensor")
dev = make_device(0x50)
dev.openDevice()
time.sleep(1)

for addr in CANDIDATES:
    dev.ADDR = addr
    try:
        if hasattr(dev, "readReg"):
            dev.readReg(0x30, 3)
        time.sleep(0.3)
    except:
        pass

    roll = None
    try:
        roll = dev.get("AngleX") if hasattr(dev, "get") else dev.getDeviceData("angleX")
    except:
        pass

    status = f"FOUND! roll={roll}" if roll is not None else "no response"
    print(f"  addr=0x{addr:02X} -> {status}")

dev.closeDevice()