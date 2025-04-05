# device_model.py
import time
import bleak
import asyncio

class DeviceModel:
    def __init__(self, deviceName, BLEDevice, callback_method):
        print("Initialize device model")
        self.deviceName = deviceName
        self.BLEDevice = BLEDevice
        self.client = None
        self.writer_characteristic = None
        self.isOpen = False
        self.callback_method = callback_method
        self.deviceData = {}
        self.TempBytes = []
        self._bg_task = None  # store reference to background task

    # -----------------------------
    #         Open Device
    # -----------------------------
    async def openDevice(self):
        print("Opening device......")
        # 1) Create client
        self.client = bleak.BleakClient(self.BLEDevice, timeout=15)
        # 2) Connect
        await self.client.connect()
        self.isOpen = True

        # 3) Find services/characteristics
        target_service_uuid = "0000ffe5-0000-1000-8000-00805f9a34fb"
        target_characteristic_uuid_read = "0000ffe4-0000-1000-8000-00805f9a34fb"
        target_characteristic_uuid_write = "0000ffe9-0000-1000-8000-00805f9a34fb"
        notify_characteristic = None

        print("Matching services......")
        for service in self.client.services:
            if service.uuid == target_service_uuid:
                print(f"Service: {service}")
                print("Matching characteristic......")
                for characteristic in service.characteristics:
                    if characteristic.uuid == target_characteristic_uuid_read:
                        notify_characteristic = characteristic
                    if characteristic.uuid == target_characteristic_uuid_write:
                        self.writer_characteristic = characteristic
                if notify_characteristic:
                    break

        # 4) If we found the writer char, do any setup needed
        if self.writer_characteristic:
            print("Reading magnetic field quaternions")
            # Replace blocking sleep with async sleep
            await asyncio.sleep(3)
            # Instead of an inline call, spawn a BG task
            self._bg_task = asyncio.create_task(self._sendDataLoop())

        # 5) If we found the notify char, subscribe
        if notify_characteristic:
            print(f"Characteristic: {notify_characteristic}")
            await self.client.start_notify(notify_characteristic.uuid, self.onDataReceived)
        else:
            print("No matching services or characteristic found")

        # 6) Return so main code can continue
        print("Finished openDevice, returning control.")
        # Now your main code sees 'await device.openDevice()' complete
        # -> It's free to do 'connected_device.set(True)' and show connected status

    # -----------------------------
    #         Close Device
    # -----------------------------
    async def closeDevice(self):
        self.isOpen = False
        print("The device is turned off")
        if self.client and self.client.is_connected:
            # Cancel BG tasks and stop notifications
            if self._bg_task:
                self._bg_task.cancel()
            # Stop notify on all
            for s in self.client.services:
                for c in s.characteristics:
                    try:
                        await self.client.stop_notify(c.uuid)
                    except Exception:
                        pass
            # Disconnect
            await self.client.disconnect()

    # -----------------------------
    #   Background read loop
    # -----------------------------
    async def _sendDataLoop(self):
        """Replaces your original sendDataTh while-loop with an async loop that won't block openDevice()."""
        while self.isOpen and self.client and self.client.is_connected:
            await self.readReg(0x3A)
            await asyncio.sleep(0.1)
            await self.readReg(0x51)
            await asyncio.sleep(0.1)

    # -----------------------------
    #      Notification Callback
    # -----------------------------
    def onDataReceived(self, sender, data):
        tempdata = bytes.fromhex(data.hex())
        for var in tempdata:
            self.TempBytes.append(var)
            if len(self.TempBytes) == 1 and self.TempBytes[0] != 0x55:
                del self.TempBytes[0]
                continue
            if len(self.TempBytes) == 2 and (self.TempBytes[1] not in [0x61, 0x71]):
                del self.TempBytes[0]
                continue
            if len(self.TempBytes) == 20:
                self.processData(self.TempBytes)
                self.TempBytes.clear()

    def processData(self, Bytes):
        # same code as your original
        if Bytes[1] == 0x61:
            Ax = self.getSignInt16(Bytes[3] << 8 | Bytes[2]) / 32768 * 16
            Ay = self.getSignInt16(Bytes[5] << 8 | Bytes[4]) / 32768 * 16
            Az = self.getSignInt16(Bytes[7] << 8 | Bytes[6]) / 32768 * 16
            Gx = self.getSignInt16(Bytes[9] << 8 | Bytes[8]) / 32768 * 2000
            Gy = self.getSignInt16(Bytes[11] << 8 | Bytes[10]) / 32768 * 2000
            Gz = self.getSignInt16(Bytes[13] << 8 | Bytes[12]) / 32768 * 2000
            AngX = self.getSignInt16(Bytes[15] << 8 | Bytes[14]) / 32768 * 180
            AngY = self.getSignInt16(Bytes[17] << 8 | Bytes[16]) / 32768 * 180
            AngZ = self.getSignInt16(Bytes[19] << 8 | Bytes[18]) / 32768 * 180
            self.set("AccX", round(Ax, 3))
            self.set("AccY", round(Ay, 3))
            self.set("AccZ", round(Az, 3))
            self.set("AsX", round(Gx, 3))
            self.set("AsY", round(Gy, 3))
            self.set("AsZ", round(Gz, 3))
            self.set("AngX", round(AngX, 3))
            self.set("AngY", round(AngY, 3))
            self.set("AngZ", round(AngZ, 3))
            self.callback_method(self)
        else:
            # read magnetics or quaternions
            if Bytes[2] == 0x3A:
                Hx = self.getSignInt16(Bytes[5] << 8 | Bytes[4]) / 120
                Hy = self.getSignInt16(Bytes[7] << 8 | Bytes[6]) / 120
                Hz = self.getSignInt16(Bytes[9] << 8 | Bytes[8]) / 120
                self.set("HX", round(Hx, 3))
                self.set("HY", round(Hy, 3))
                self.set("HZ", round(Hz, 3))
            elif Bytes[2] == 0x51:
                Q0 = self.getSignInt16(Bytes[5] << 8 | Bytes[4]) / 32768
                Q1 = self.getSignInt16(Bytes[7] << 8 | Bytes[6]) / 32768
                Q2 = self.getSignInt16(Bytes[9] << 8 | Bytes[8]) / 32768
                Q3 = self.getSignInt16(Bytes[11] << 8 | Bytes[10]) / 32768
                self.set("Q0", round(Q0, 5))
                self.set("Q1", round(Q1, 5))
                self.set("Q2", round(Q2, 5))
                self.set("Q3", round(Q3, 5))
            else:
                pass

    # -----------------------------
    #   Accessors for deviceData
    # -----------------------------
    def set(self, key, value):
        self.deviceData[key] = value

    def get(self, key):
        return self.deviceData.get(key, None)

    @staticmethod
    def getSignInt16(num):
        if num >= pow(2, 15):
            num -= pow(2, 16)
        return num

    # -----------------------------
    #   Send Data / Read/Write
    # -----------------------------
    async def sendData(self, data):
        try:
            if self.client and self.client.is_connected and self.writer_characteristic:
                await self.client.write_gatt_char(self.writer_characteristic.uuid, bytes(data))
        except Exception as ex:
            print(ex)

    async def readReg(self, regAddr):
        await self.sendData(self.get_readBytes(regAddr))

    async def writeReg(self, regAddr, sValue):
        # unlock
        await self.unlock()
        await asyncio.sleep(0.1)
        await self.sendData(self.get_writeBytes(regAddr, sValue))
        await asyncio.sleep(0.1)
        await self.save()

    @staticmethod
    def get_readBytes(regAddr):
        return [0xff, 0xaa, 0x27, regAddr, 0x00]

    @staticmethod
    def get_writeBytes(regAddr, rValue):
        tempBytes = [0xff, 0xaa, regAddr, (rValue & 0xff), (rValue >> 8)]
        return tempBytes

    async def unlock(self):
        cmd = self.get_writeBytes(0x69, 0xb588)
        await self.sendData(cmd)

    async def save(self):
        cmd = self.get_writeBytes(0x00, 0x0000)
        await self.sendData(cmd)
