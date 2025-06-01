import asyncio, time, bleak, solara
from collections import deque
from device_model import DeviceModel
import numpy as np

import plotly.graph_objs as go
from solara import FigurePlotly
import csv
from datetime import datetime

connected_device          = solara.reactive(False)
device_model              = solara.reactive(None)

acceleration              = solara.reactive((0.0, 0.0, 0.0))
quaternion                = solara.reactive((0.0, 0.0, 0.0, 0.0))

accel_history             = deque(maxlen=4000)          # ~40 s at 100 Hz
accel_history_reactive    = solara.reactive([])

plot_time_window          = solara.reactive(10.0)       # seconds

devices                   = solara.reactive([])
selected_device           = solara.reactive(None)
status                    = solara.reactive("Idle")

_last_plot_update = [0.0]

# Reactive state for recording
recording = solara.reactive(False)
recording_file = solara.reactive(None)

# ---------- data pump ---------- #
def update_data(device: DeviceModel):
    ax = device.get("AccX") or 0.0
    ay = device.get("AccY") or 0.0
    az = device.get("AccZ") or 0.0
    q0 = device.get("Q0")  or 0.0
    q1 = device.get("Q1")  or 0.0
    q2 = device.get("Q2")  or 0.0
    q3 = device.get("Q3")  or 0.0
    AngX = device.get("AngX")  or 0.0
    AngY = device.get("AngY")  or 0.0
    AngZ = device.get("AngZ")  or 0.0 
    AsX = device.get("AsX")  or 0.0
    AsY = device.get("AsY")  or 0.0
    AsZ = device.get("AsZ")  or 0.0 

    now = time.time()
    accel_history.append((now, ax, ay, az,AsX,AsY,AsZ, q0, q1, q2, q3))


    # Write to CSV if recording 
    if recording.value and recording_file.value:
        writer = csv.writer(recording_file.value)
        writer.writerow([now, ax, ay, az, AngX, AngY, AngZ, q0, q1, q2, q3])
        recording_file.value.flush()


    # push fresh values for UI every 0.1 s
    if now - _last_plot_update[0] > 0.1:
        acceleration.set((round(ax, 1), round(ay, 1), round(az, 1)))
        quaternion.set((round(q0, 3), round(q1, 3), round(q2, 3), round(q3, 3)))
        accel_history_reactive.set(list(accel_history))
        _last_plot_update[0] = now


async def scan_devices():
    status.set("Scanning for devices…")
    try:
        found = await bleak.BleakScanner.discover(timeout=10)
        wt_devices = [d for d in found if d.name and "WT" in d.name]
        devices.set(wt_devices)
        status.set(f"Found {len(wt_devices)} WT devices.")
    except Exception as e:
        status.set(f"Error during scanning: {e}")

async def connect():
    status.set("Attempting to connect…")
    if selected_device.value is None:
        status.set("No device selected.")
        return
    try:
        dm = DeviceModel("MyBle5.0", selected_device.value, update_data)
        await dm.openDevice()
        await dm.set_sampling_rate(0x09)  # set to 100 Hz
        device_model.set(dm)
        connected_device.set(True)
        
        status.set("Connected.")
    except Exception as e:
        status.set(f"Connection failed: {e}")

async def calibrate_acceleration():
    if device_model.value is None:
        status.set("Device not connected.")
        return
    try:
        await device_model.value.sendData([0xFF, 0xAA, 0x01, 0x01])
        status.set("Calibration command sent.")
    except Exception as e:
        status.set(f"Calibration failed: {e}")

# Record/Stop Button
def toggle_recording():
    if not recording.value:
        # Start recording
        filename = datetime.now().strftime("imu_logs/%Y%m%d_%H%M%S.csv")
        f = open(filename, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(["timestamp", "AccX", "AccY", "AccZ","AngX","AngY","AngZ","AsX", "AsY", "AsZ", "Q0", "Q1", "Q2", "Q3"])
        recording_file.set(f)
        recording.set(True)
        status.set(f"Recording to {filename}")
    else:
        # Stop recording
        f = recording_file.value
        if f:
            f.close()
        recording_file.set(None)
        recording.set(False)
        status.set("Recording stopped.")




@solara.component
def PlotData():
    history = accel_history_reactive.value
    win = plot_time_window.value

    if not history or len(history) < 2:
        solara.Markdown("Waiting for enough data to plot...")
        return

    try:
        # Unpack and window the data
        times, ax, ay, az, q0, q1, q2, q3 = map(np.array, zip(*history))
        times -= times[0]

        keep = times >= times[-1] - win
        times, ax, ay, az, q0, q1, q2, q3 = (arr[keep] for arr in (times, ax, ay, az, q0, q1, q2, q3))

        if len(times) < 2:
            solara.Markdown("Not enough data after filtering.")
            return

        # Create the plot
        fig = go.Figure()

        fig.add_trace(go.Scatter(x=times, y=ax, mode="lines", name="AccX"))
        fig.add_trace(go.Scatter(x=times, y=ay, mode="lines", name="AccY"))
        fig.add_trace(go.Scatter(x=times, y=az, mode="lines", name="AccZ"))

        for data, label, color in zip([q0, q1, q2, q3], ["Q0", "Q1", "Q2", "Q3"], ["gray", "blue", "green", "red"]):
            fig.add_trace(go.Scatter(x=times, y=data, name=label, yaxis="y2", line=dict(color=color, dash="dot")))

        fig.update_layout(
            xaxis=dict(title="Time (s)"),
            yaxis=dict(title="Acceleration (m/s²)"),
            yaxis2=dict(title="Quaternion", overlaying="y", side="right"),
            legend=dict(orientation="h"),
            margin=dict(t=30, b=30),
        )

        FigurePlotly(fig)

    except Exception as e:
        solara.Markdown(f"⚠️ **Plot error**: {str(e)}")

@solara.component
def Page():
    with solara.AppBar():
        solara.lab.ThemeToggle()
    with solara.Sidebar():
        with solara.Card("Device Setup") as main:        
            solara.Markdown("## Bluetooth Devices")
            solara.Button("Scan Devices", on_click=lambda: asyncio.create_task(scan_devices()))

            if devices.value:
                options = {f"{d.name} ({d.address})": d for d in devices.value}
                def on_select(label):
                    selected_device.set(options[label])
                    status.set(f"Selected: {label}")
                solara.Select(label="Select Device", values=list(options.keys()), on_value=on_select)

            if selected_device.value and not connected_device.value:
                solara.Button("Connect", on_click=lambda: asyncio.create_task(connect()))


    if connected_device.value:
        
        with solara.Card("Connected Device") as main:  
            solara.Markdown("## IMU Data")
            ax, ay, az = acceleration.value
            q0, q1, q2, q3 = quaternion.value
            solara.Markdown(f"**Acceleration (g):** {ax:.1f}, {ay:.1f}, {az:.1f}")
            solara.Markdown(f"**Quaternion:** {q0:.3f}, {q1:.3f}, {q2:.3f}, {q3:.3f}")

            solara.Button("Calibrate Accelerometer", on_click=lambda: asyncio.create_task(calibrate_acceleration()))
            solara.SliderFloat("Time Window (s)", value=plot_time_window, min=1, max=60, step=1)
            # Display appropriate button
            button_label = "Stop Recording" if recording.value else "Start Recording"
            solara.Button(button_label, on_click=toggle_recording)
            #PlotData()

    else:
        solara.Info("Not Connected")        


        solara.Markdown(f"### Status: {status.value}")
