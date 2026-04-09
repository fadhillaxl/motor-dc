# coding: UTF-8
import sys
import time
import threading
import platform

# =====================================================
# PYTHON PATH → HARUS KE FOLDER chs
# =====================================================
SDK_CHS = "/home/raspberrypi5/WitStandardModbus_WT901C485/Python/Python-SDK-WT901C485/chs"
sys.path.insert(0, SDK_CHS)

# =====================================================
# IMPORT SDK (SEKARANG AKAN BERHASIL)
# =====================================================
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver

# =====================================================
# CALLBACK DATA
# =====================================================
def onUpdate(device):
    roll  = device.getDeviceData("angleX")
    pitch = device.getDeviceData("angleY")
    yaw   = device.getDeviceData("angleZ")

    print(f"ROLL={roll:7.2f}  PITCH={pitch:7.2f}  YAW={yaw:7.2f}")

# =====================================================
# LOOP READ
# =====================================================
def loop_read(device):
    while True:
        device.readReg(0x30, 41)
        time.sleep(0.01)

# =====================================================
# MAIN
# =====================================================
print("WT901C485 running from motor-dc... Ctrl+C to stop")

device = deviceModel.DeviceModel(
    "WT901",
    Protocol485Resolver(),
    JY901SDataProcessor(),
    "51_0"
)

device.ADDR = 0x50

if platform.system().lower() == "linux":
    device.serialConfig.portName = "/dev/ttyUSB0"
else:
    device.serialConfig.portName = "COM82"

device.serialConfig.baud = 9600
device.openDevice()

device.dataProcessor.onVarChanged.append(onUpdate)

t = threading.Thread(target=loop_read, args=(device,), daemon=True)
t.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStop")
finally:
    device.closeDevice()
