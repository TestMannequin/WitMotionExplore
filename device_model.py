# device_model.py
import time
import asyncio
import bleak
import numpy as np
log_file_path = "imu_raw_packets.log"
def log_packet(packet):
    with open(log_file_path, "a") as f:
        f.write(" ".join(f"{byte:02X}" for byte in packet) + "\n")
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
        self._buffer = bytearray()  # use a bytearray to accumulate incoming bytes
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
        target_char_uuid_read = "0000ffe4-0000-1000-8000-00805f9a34fb"
        target_char_uuid_write = "0000ffe9-0000-1000-8000-00805f9a34fb"
        notify_characteristic = None

        print("Matching services......")
        for service in self.client.services:
            if service.uuid.lower() == target_service_uuid.lower():
                print(f"Service found: {service}")
                print("Matching characteristics......")
                for characteristic in service.characteristics:
                    if characteristic.uuid.lower() == target_char_uuid_read.lower():
                        notify_characteristic = characteristic
                    if characteristic.uuid.lower() == target_char_uuid_write.lower():
                        self.writer_characteristic = characteristic
                if notify_characteristic:
                    break

        # 4) If we found the writer char, do any additional setup
        if self.writer_characteristic:
            print("Writer characteristic found. Starting background tasks.")
            # Replace blocking sleep with async sleep; adjust delay as needed.
            await asyncio.sleep(3)
            # Spawn a background loop for polling registers
            self._bg_task = asyncio.create_task(self._sendDataLoop())

        # 5) If we found the notify char, subscribe to notifications
        if notify_characteristic:
            print(f"Subscribing to notification on: {notify_characteristic}")
            await self.client.start_notify(notify_characteristic.uuid, self.onDataReceived)
        else:
            print("No matching services or characteristic found for notifications.")

        print("Finished openDevice, returning control.")

    # -----------------------------
    #         Close Device
    # -----------------------------
    async def closeDevice(self):
        self.isOpen = False
        print("Closing device...")
        # Cancel background tasks
        if self._bg_task:
            self._bg_task.cancel()
            try:
                await self._bg_task
            except asyncio.CancelledError:
                pass
        # Stop notifications for all characteristics
        if self.client and self.client.is_connected:
            for service in self.client.services:
                for characteristic in service.characteristics:
                    try:
                        await self.client.stop_notify(characteristic.uuid)
                    except Exception:
                        pass
            # Disconnect
            await self.client.disconnect()
        print("Device closed.")

    # -----------------------------
    #   Background read loop
    # -----------------------------
    async def _sendDataLoop(self):
        """Background task that sends register read commands periodically."""
        while self.isOpen and self.client and self.client.is_connected:
            #await self.readReg(0x3A)  don't need magentomeer
            await self.readReg(0x51)
            await asyncio.sleep(0.1)  # 10ms = 100 Hz

    # -----------------------------
    #      Notification Callback
    # -----------------------------
    def onDataReceived(self, sender, data):
        # data is already a bytes object; just extend our buffer.
        self._buffer.extend(data)

        # Process packets whenever we have at least 20 bytes.
        while len(self._buffer) >= 20:
            # Look for packet header (0x55) at the beginning.
            if self._buffer[0] != 0x55:
                self._buffer.pop(0)
                continue
            # Check if the second byte indicates either 0x61 or 0x71.
            if self._buffer[1] not in [0x61, 0x71]:
                self._buffer.pop(0)
                continue
            # Assume fixed packet size of 20 bytes.
            packet = self._buffer[:20]
            # Remove processed packet from buffer.
            self._buffer = self._buffer[20:]
            self.processData(packet)




    # -----------------------------
    #     Process Received Data
    # -----------------------------
    def processData(self, packet):
        # For flag 0x61: acceleration, angular velocity, and angle data.
        if packet[1] == 0x61:
            log_packet(packet)
            # Use correct low-byte-first decoding:
            # Extract raw acceleration values (little-endian)
            ax_raw = self.getSignInt16(packet[3] << 8 | packet[2])
            ay_raw = self.getSignInt16(packet[5] << 8 | packet[4])
            az_raw = self.getSignInt16(packet[7] << 8 | packet[6])

            # Convert to acceleration in m/s²
            Ax = ax_raw / 32768 * 16 * 9.80665
            Ay = ay_raw / 32768 * 16 * 9.80665
            Az = az_raw / 32768 * 16 * 9.80665

            Gx = self.getSignInt16(packet[8] | (packet[9] << 8)) / 32768 * 2000
            Gy = self.getSignInt16(packet[10] | (packet[11] << 8)) / 32768 * 2000
            Gz = self.getSignInt16(packet[12] | (packet[13] << 8)) / 32768 * 2000
            AngX = self.getSignInt16(packet[14] | (packet[15] << 8)) / 32768 * 180
            AngY = self.getSignInt16(packet[16] | (packet[17] << 8)) / 32768 * 180
            AngZ = self.getSignInt16(packet[18] | (packet[19] << 8)) / 32768 * 180

            self.set("AccX", round(Ax, 3))
            self.set("AccY", round(Ay, 3))
            self.set("AccZ", round(Az, 3))
            self.set("AsX", round(Gx, 3))
            self.set("AsY", round(Gy, 3))
            self.set("AsZ", round(Gz, 3))
            self.set("AngX", round(AngX, 3))
            self.set("AngY", round(AngY, 3))
            self.set("AngZ", round(AngZ, 3))
            # Optionally, combine these with orientation data from quaternions if available.
            self.callback_method(self)
        else:
            # Process other packets (e.g., magnetic or quaternion data).
            if packet[2] == 0x3A:
                # Magnetic field output
                Hx = self.getSignInt16(packet[4] | (packet[5] << 8)) / 120
                Hy = self.getSignInt16(packet[6] | (packet[7] << 8)) / 120
                Hz = self.getSignInt16(packet[8] | (packet[9] << 8)) / 120
                self.set("HX", round(Hx, 3))
                self.set("HY", round(Hy, 3))
                self.set("HZ", round(Hz, 3))
            elif packet[2] == 0x51:
                # Quaternion output: format is 0x55, 0x71, 0x51, 0x00, then 8 bytes for Q0, Q1, Q2, Q3.
                Q0 = self.getSignInt16(packet[4] | (packet[5] << 8)) / 32768
                Q1 = self.getSignInt16(packet[6] | (packet[7] << 8)) / 32768
                Q2 = self.getSignInt16(packet[8] | (packet[9] << 8)) / 32768
                Q3 = self.getSignInt16(packet[10] | (packet[11] << 8)) / 32768
                self.set("Q0", round(Q0, 5))
                self.set("Q1", round(Q1, 5))
                self.set("Q2", round(Q2, 5))
                self.set("Q3", round(Q3, 5))
            else:
                # Unrecognized packet type; ignore or log.
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
        if num >= (1 << 15):
            num -= (1 << 16)
        return num

    # -----------------------------
    #   Send / Read / Write Commands
    # -----------------------------
    async def sendData(self, data):
        try:
            if self.client and self.client.is_connected and self.writer_characteristic:
                await self.client.write_gatt_char(self.writer_characteristic.uuid, bytes(data))
        except Exception as ex:
            print("SendData exception:", ex)

    async def readReg(self, regAddr):
        await self.sendData(self.get_readBytes(regAddr))

    async def writeReg(self, regAddr, sValue):
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
        return [0xff, 0xaa, regAddr, (rValue & 0xff), (rValue >> 8)]

    async def unlock(self):
        cmd = self.get_writeBytes(0x69, 0xb588)
        await self.sendData(cmd)

    async def save(self):
        cmd = self.get_writeBytes(0x00, 0x0000)
        await self.sendData(cmd)

    async def set_sampling_rate(self, hz_code=0x09):  # default to 100 Hz
        await self.unlock()
        await asyncio.sleep(0.1)
        await self.writeReg(0x03, hz_code)
        await asyncio.sleep(0.1)
        await self.save()