import asyncio
import time
import solara
import bleak
from device_model import DeviceModel  # your updated device_model with non-blocking openDevice

connected_device = solara.reactive(False)
device_model = solara.reactive(None)
acceleration = solara.reactive((0.0, 0.0, 0.0))

# We'll store a running history of (timestamp, ax, ay, az)
accel_history = solara.reactive([])

devices = solara.reactive([])
selected_device = solara.reactive(None)
status = solara.reactive("Idle")


def update_data(device: DeviceModel):
    """Called whenever new IMU data arrives."""
    ax = device.get("AccX") or 0.0
    ay = device.get("AccY") or 0.0
    az = device.get("AccZ") or 0.0
    # Round to 1 decimal place for display
    ax_rounded = round(ax, 1)
    ay_rounded = round(ay, 1)
    az_rounded = round(az, 1)
    acceleration.set((ax_rounded, ay_rounded, az_rounded))

    # Append full-precision data to history for plotting (or also store the rounded data)
    t = time.time()
    # In order to trigger reactivity, make a copy or reassign
    new_data = accel_history.value.copy()
    new_data.append((t, ax, ay, az))
    accel_history.set(new_data)


async def scan_devices():
    """Scan for WT devices and populate the dropdown."""
    status.set("Scanning for devices...")
    try:
        found = await bleak.BleakScanner.discover(timeout=10.0)
        wt_devices = [d for d in found if d.name and "WT" in d.name]
        devices.set(wt_devices)
        status.set(f"Found {len(wt_devices)} WT devices.")
    except Exception as e:
        status.set(f"Error during scanning: {e}")


async def connect():
    """Connect to the selected device."""
    print("Connect button clicked!")
    status.set("Attempting to connect...")
    if selected_device.value is not None:
        try:
            print(f"Selected device: {selected_device.value}")
            dev_model = DeviceModel("MyBle5.0", selected_device.value, update_data)
            await dev_model.openDevice()  # Non-blocking open
            device_model.set(dev_model)
            connected_device.set(True)
            status.set("Connected to device.")
            print("Connection successful!")
        except Exception as e:
            status.set(f"Connection failed: {e}")
            print("Connection failed:", e)
    else:
        status.set("No device selected.")


async def calibrate_acceleration():
    """Send calibration command to the connected device."""
    status.set("Sending calibration command...")
    if device_model.value is not None:
        try:
            await device_model.value.sendData([0xFF, 0xAA, 0x01, 0x01])
            status.set("Calibration command sent.")
        except Exception as e:
            status.set(f"Calibration failed: {e}")
    else:
        status.set("Device not connected.")


@solara.component
def AccelerationPlot():
    """
    A simple time-series plot of X, Y, Z accelerations vs. time,
    using the data in accel_history.
    """
    import matplotlib.pyplot as plt

    data = accel_history.value
    if len(data) < 2:
        # Not enough data points to plot
        return solara.Markdown("*No acceleration data yet.*")

    # Separate the data
    times = [row[0] for row in data]
    xs = [row[1] for row in data]
    ys = [row[2] for row in data]
    zs = [row[3] for row in data]

    # Create figure
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    # Plot each axis
    ax.plot(times, xs, label="AccX")
    ax.plot(times, ys, label="AccY")
    ax.plot(times, zs, label="AccZ")
    ax.set_xlabel("Time (s, epoch)")
    ax.set_ylabel("Acceleration (±g)")
    ax.legend()
    ax.grid(True)

    # Return as a solara Figure
    return solara.FigureMatplotlib(fig)


@solara.component
def Page():
    solara.Title("WT901BLE5.0 Accelerometer App (with Time-Series)")

    with solara.Column():
        # --- Device scanning ---
        solara.Markdown("## Bluetooth Devices")
        solara.Button(
            "Scan Devices",
            on_click=lambda: asyncio.create_task(scan_devices())
        )

        # --- Device selection dropdown ---
        if devices.value:
            options = {f"{d.name} ({d.address})": d for d in devices.value}

            def on_select(label):
                selected_device.set(options[label])
                status.set(f"Selected device: {label}")

            solara.Select(
                label="Select Device",
                values=list(options.keys()),
                on_value=on_select,
            )

        # --- Connect button ---
        if selected_device.value and not connected_device.value:
            solara.Button(
                "Connect",
                on_click=lambda: asyncio.create_task(connect())
            )

        # --- Acceleration display & calibrate button ---
        if connected_device.value:
            solara.Markdown("## Acceleration (X, Y, Z)")
            ax, ay, az = acceleration.value
            solara.Text(f"{ax:.2f}, {ay:.2f}, {az:.2f} g")

            solara.Button(
                "Calibrate Accelerometer",
                on_click=lambda: asyncio.create_task(calibrate_acceleration())
            )

            solara.Markdown("### Real-time Acceleration Plot")
            #AccelerationPlot()

        # --- Status display ---
        solara.Markdown(f"### Status: {status.value}")
