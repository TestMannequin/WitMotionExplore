import asyncio
import time
import bleak
import solara
from device_model import DeviceModel
import csv
from datetime import datetime
import os

# -----------------------------
# Constants
# -----------------------------
UI_INTERVAL = 0.2   # seconds between UI updates (5 Hz)
LOG_DIR     = "imu_logs"

# -----------------------------
# Reactive state
# -----------------------------
devices           = solara.reactive([])   # discovered BLEDevice objects
connected_devices = solara.reactive([])   # list of DeviceModel
accelerations     = solara.reactive([])   # list of (ax,ay,az)
angles            = solara.reactive([])   # list of (AngX,AngY,AngZ)
quaternions       = solara.reactive([])   # list of (q0,q1,q2,q3)
status            = solara.reactive("Idle")

# Recording state
recording         = solara.reactive(False)
recording_file    = solara.reactive(None)

# Throttle state
_last_ui_update = [0.0]

# Ensure log folder exists
os.makedirs(LOG_DIR, exist_ok=True)

# -----------------------------
# Data pump callback
# -----------------------------
def update_data(device: DeviceModel):
    now = time.time()

    # 1) Always fetch raw values
    ax = device.get("AccX") or 0.0
    ay = device.get("AccY") or 0.0
    az = device.get("AccZ") or 0.0
    AngX = device.get("AngX") or 0.0
    AngY = device.get("AngY") or 0.0
    AngZ = device.get("AngZ") or 0.0
    q0 = device.get("Q0") or 0.0
    q1 = device.get("Q1") or 0.0
    q2 = device.get("Q2") or 0.0
    q3 = device.get("Q3") or 0.0

    # 2) CSV logging at full rate
    if recording.value and recording_file.value:
        writer = csv.writer(recording_file.value)
        writer.writerow([
            device.BLEDevice.address,
            now,
            ax, ay, az,
            AngX, AngY, AngZ,
            q0, q1, q2, q3,
        ])
        recording_file.value.flush()

    # 3) Throttle UI updates
    if now - _last_ui_update[0] < UI_INTERVAL:
        return
    _last_ui_update[0] = now

    # 4) Figure out which index this device is
    addrs = [dm.BLEDevice.address for dm in connected_devices.value]
    try:
        idx = addrs.index(device.BLEDevice.address)
    except ValueError:
        return

    # 5) Update reactive lists
    accs = list(accelerations.value)
    accs[idx] = (round(ax,1), round(ay,1), round(az,1))
    accelerations.set(accs)

    angs = list(angles.value)
    angs[idx] = (round(AngX,1), round(AngY,1), round(AngZ,1))
    angles.set(angs)

    quats = list(quaternions.value)
    quats[idx] = (
        round(q0,3), round(q1,3),
        round(q2,3), round(q3,3),
    )
    quaternions.set(quats)


# -----------------------------
# BLE scan/connect logic
# -----------------------------
async def scan_devices():
    status.set("Scanning for devices…")
    try:
        found = await bleak.BleakScanner.discover(timeout=10)
        wt = [d for d in found if d.name and "WT" in d.name]
        devices.set(wt)
        status.set(f"Found {len(wt)} WT devices.")
    except Exception as e:
        status.set(f"Scan error: {e}")

async def connect_all():
    status.set("Connecting to devices…")
    models = []
    for d in devices.value:
        try:
            dm = DeviceModel("MyBle5.0", d, update_data)
            await dm.openDevice()
            await dm.set_sampling_rate(0x09)  # 100 Hz
            models.append(dm)
        except Exception as e:
            status.set(f"❌ {d.address} failed: {e}")
    if models:
        connected_devices.set(models)
        # initialize blank slots
        accelerations.set([(0.0,0.0,0.0) for _ in models])
        angles.set([(0.0,0.0,0.0)        for _ in models])
        quaternions.set([(0.0,0.0,0.0,0.0) for _ in models])
        status.set("✅ Connected to all devices.")

async def calibrate_acceleration(device: DeviceModel):
    try:
        await device.sendData([0xFF,0xAA,0x01,0x01])
        status.set(f"Calibrate sent to {device.BLEDevice.address}")
    except Exception as e:
        status.set(f"Calibrate error: {e}")


# -----------------------------
# Recording toggle
# -----------------------------
def toggle_recording():
    if not recording.value:
        fn = f"{LOG_DIR}/{datetime.now():%Y%m%d_%H%M%S}.csv"
        f = open(fn, "w", newline="")
        w = csv.writer(f)
        w.writerow([
            "address","timestamp",
            "AccX","AccY","AccZ",
            "AngX","AngY","AngZ",
            "Q0","Q1","Q2","Q3",
        ])
        recording_file.set(f)
        recording.set(True)
        status.set(f"Recording ➡️ {fn}")
    else:
        f = recording_file.value
        if f:
            f.close()
        recording_file.set(None)
        recording.set(False)
        status.set("Recording stopped.")


# -----------------------------
# UI Component
# -----------------------------
@solara.component
def Page():
    with solara.AppBar():
        solara.lab.ThemeToggle()

    # Sidebar: scan/connect + record
    with solara.Sidebar():
        with solara.Card("Device Setup"):
            solara.Markdown("## Bluetooth IMUs")
            solara.Button("Scan Devices", on_click=lambda: asyncio.create_task(scan_devices()))
            if devices.value:
                for d in devices.value:
                    solara.Markdown(f"- **{d.name}** ({d.address})")
                solara.Button("Connect All", on_click=lambda: asyncio.create_task(connect_all()))

        with solara.Card("Recording"):
            label = "Stop Recording" if recording.value else "Start Recording"
            solara.Button(label, on_click=toggle_recording)
            solara.Markdown(f"▶️ Recording: {recording.value}")

    # Main pane: live readings
    if connected_devices.value:
        for idx, (dm, acc, ang, quat) in enumerate(zip(
            connected_devices.value,
            accelerations.value,
            angles.value,
            quaternions.value,
        )):
            addr = dm.BLEDevice.address
            ax, ay, az = acc
            AngX, AngY, AngZ = ang
            q0, q1, q2, q3 = quat
            with solara.Card(f"Device {idx+1} ({addr})"):
                solara.Markdown(f"**Acc (g):** {ax:.1f}, {ay:.1f}, {az:.1f}")
                solara.Markdown(f"**Ang (°):** {AngX:.1f}, {AngY:.1f}, {AngZ:.1f}")
                solara.Markdown(f"**Quat:** {q0:.3f}, {q1:.3f}, {q2:.3f}, {q3:.3f}")
                solara.Button(
                    "Calibrate",
                    on_click=lambda d=dm: asyncio.create_task(calibrate_acceleration(d)),
                )
    else:
        solara.Info("Not Connected")

    solara.Markdown(f"### Status: {status.value}")
