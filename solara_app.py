import asyncio, time, bleak, solara
from collections import deque
from device_model import DeviceModel
import numpy as np

import plotly.graph_objs as go
from solara import FigurePlotly


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

# ---------- data pump ---------- #
def update_data(device: DeviceModel):
    ax = device.get("AccX") or 0.0
    ay = device.get("AccY") or 0.0
    az = device.get("AccZ") or 0.0
    q0 = device.get("Q0")  or 0.0
    q1 = device.get("Q1")  or 0.0
    q2 = device.get("Q2")  or 0.0
    q3 = device.get("Q3")  or 0.0

    now = time.time()
    accel_history.append((now, ax, ay, az, q0, q1, q2, q3))

    # push fresh values for UI every 0.1 s
    if now - _last_plot_update[0] > 0.1:
        acceleration.set((round(ax, 1), round(ay, 1), round(az, 1)))
        quaternion.set((round(q0, 3), round(q1, 3), round(q2, 3), round(q3, 3)))
        accel_history_reactive.set(list(accel_history))
        _last_plot_update[0] = now
# -------------------------------- #

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

# ---------- plotting component ---------- #
def _make_figure():
    fig = Figure(figsize=(10, 4))
    ax1 = fig.subplots()
    acc_lines = [
        ax1.plot([], [], label="AccX")[0],
        ax1.plot([], [], label="AccY", linestyle="--")[0],
        ax1.plot([], [], label="AccZ", linestyle=":")[0],
    ]
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Acceleration (m/s²)")
    ax1.grid(True)
    ax1.legend(loc="upper left")

    ax2 = ax1.twinx()
    quat_lines = [
        ax2.plot([], [], label="Q0", alpha=.4)[0],
        ax2.plot([], [], label="Q1", alpha=.4)[0],
        ax2.plot([], [], label="Q2", alpha=.4)[0],
        ax2.plot([], [], label="Q3", alpha=.4)[0],
    ]
    ax2.set_ylabel("Quaternion")
    ax2.legend(loc="upper right")
    return fig, ax1, ax2, acc_lines, quat_lines


@solara.component
def PlotData():
    history = accel_history_reactive.value
    win     = plot_time_window.value

    if not history:
        solara.Markdown("No data yet.")
        return

    # unpack history
    times, ax, ay, az, q0, q1, q2, q3 = map(np.array, zip(*history))
    times -= times[0]  # relative timestamps

    # limit time window
    keep = times >= times[-1] - win
    times, ax, ay, az, q0, q1, q2, q3 = (arr[keep] for arr in (times, ax, ay, az, q0, q1, q2, q3))

    fig = go.Figure()

    # Acceleration
    fig.add_trace(go.Scatter(x=times, y=ax, mode="lines", name="AccX"))
    fig.add_trace(go.Scatter(x=times, y=ay, mode="lines", name="AccY"))
    fig.add_trace(go.Scatter(x=times, y=az, mode="lines", name="AccZ"))

    # Quaternion (secondary Y axis)
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

@solara.component
def Page():
    solara.Title("WT901BLE5.0 Accelerometer Debug App")
    with solara.Column():
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
            solara.Markdown("## IMU Data")
            ax, ay, az = acceleration.value
            q0, q1, q2, q3 = quaternion.value
            solara.Markdown(f"**Acceleration (g):** {ax:.1f}, {ay:.1f}, {az:.1f}")
            solara.Markdown(f"**Quaternion:** {q0:.3f}, {q1:.3f}, {q2:.3f}, {q3:.3f}")

            solara.Button("Calibrate Accelerometer", on_click=lambda: asyncio.create_task(calibrate_acceleration()))
            solara.SliderFloat("Time Window (s)", value=plot_time_window, min=1, max=60, step=1)

            PlotData()

        solara.Markdown(f"### Status: {status.value}")
