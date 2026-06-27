# MotorBrake — ROS 2 over CAN bridge

This brings the STM32 `MotorBrake_Firmware` onto ROS 2 topics, using a CAN-to-USB
adapter on the PC. The slide asked for "micro-ROS over CAN"; we implement the same
behaviour with the more robust and adapter-agnostic pattern of **raw CAN frames on
the STM32 + a thin ROS 2 bridge node on the PC** (SocketCAN via python-can).

```
  ROS 2 topics                 PC bridge (this pkg)            CAN bus            STM32G474
 /brake_command  ──Bool──►  brake_bridge  ──CAN 0x130──►  [CAN-USB] ═══►  FDCAN1  ─► Relay
 /brake_status   ◄─BrakeStatus── brake_bridge ◄─CAN 0x131── [CAN-USB] ◄═══  FDCAN1 ◄─ INA240 + state
 /brake_estop    ◄─Bool── (raised by bridge if heartbeat lost > 100 ms)
 /servo_command  ──Float32MultiArray[start,stop]──►  brake_bridge  ──CAN 0x132──►  [CAN-USB] ═══►  FDCAN1  ─► Servo PWM (start/stop pos, saved to flash)
```

## Topics

| Topic             | Type                              | Direction   | Meaning |
|-------------------|-----------------------------------|-------------|---------|
| `/brake_command`  | `std_msgs/msg/Bool`               | PC → STM32  | `true` = Relay ON (engage brake), `false` = Relay OFF (release) |
| `/brake_status`   | `motorbrake_msgs/msg/BrakeStatus` | STM32 → PC  | `current_ma`, `relay_active`, `watchdog_status`, `servo_start_deg`/`servo_stop_deg` (angles the STM32 reports holding) |
| `/brake_estop`    | `std_msgs/msg/Bool`               | bridge → PC | `true` when the bridge has not seen a heartbeat for > 100 ms |
| `/servo_command`  | `std_msgs/msg/Float32MultiArray`  | PC → STM32  | `data: [start_deg, stop_deg]`, two servo angles (0.0–180.0°). `start` = brake OFF/released pos, `stop` = brake ON/engaged pos. Saved to STM32 flash |

## CAN protocol (classic CAN, 1 Mbps, 11-bit IDs)

| ID      | Dir        | DLC | Payload |
|---------|------------|-----|---------|
| `0x130` | PC → STM32 | 1   | `data[0]` = 1 (Relay ON) / 0 (Relay OFF) |
| `0x131` | STM32 → PC | 8   | `[0..3]` float32 `current_ma` LE, `[4]` `relay_active`, `[5]` `watchdog_status`, `[6..7]` uint16 seq LE |
| `0x132` | PC → STM32 | 8   | `[0..3]` float32 `start_deg` LE, `[4..7]` float32 `stop_deg` LE (both clamped 0–180°, saved to flash) |
| `0x133` | STM32 → PC | 8   | `[0..3]` float32 `start_deg` LE, `[4..7]` float32 `stop_deg` LE (angles the STM32 is holding) |

The STM32 sends `0x131` every ~20 ms (heartbeat). Bitrate, IDs and the 100 ms
fail-safe timeout are all editable: in firmware via the `#define`s near the top of
`Core/Src/main.c` / `Core/Src/fdcan.c`, and on the PC via node parameters (below).

## Hardware wiring

- `PA12` = FDCAN1_TX, `PA11` = FDCAN1_RX → CAN transceiver (TJA1050 / SN65HVD230 / MCP2562) → CANH/CANL.
- 120 Ω termination at **both** ends of the bus (STM32 end and the CAN-USB adapter end).
- INA240A2D (gain 50 V/V) across a 2 mΩ shunt, output → `PA0` (ADC1_IN1, `adc_buffer[0]`).
  Unidirectional wiring (REF → GND), so 0 A ≈ 0 V → `current_ma` reads only the positive direction.
  If you wired the INA240 output to a different ADC pin, change `CURRENT_ADC_INDEX` in `main.c`.

## 1) Flash the firmware

Build/flash `MotorBrake_Firmware` from STM32CubeIDE as usual (no `.ioc` change is
needed — the CAN logic lives in the `USER CODE` sections of `main.c`, `fdcan.c`,
`stm32g4xx_it.c`). On boot the board immediately starts emitting `0x131` heartbeats.

## 2) Bring up the CAN-USB interface on the PC

**SocketCAN-native adapters** (CANable/candleLight `gs_usb`, PEAK, Kvaser, …):

```bash
sudo ip link set can0 up type can bitrate 1000000
# verify:
candump can0        # from the can-utils package
```

**slcan adapters** (CANable in slcan firmware, USBtin, …):

```bash
sudo slcand -o -c -s8 /dev/ttyACM0 can0   # -s8 = 1 Mbps
sudo ip link set up can0
```

(`-s` codes: `s6`=500k, `s8`=1M. Match the firmware bitrate.)

## 3) Get ROS 2 — Option A (native) or Option B (Docker)

If your machine already runs a ROS 2-supported Ubuntu (22.04 Humble / 24.04 Jazzy),
use **Option A** below. If it does **not** (e.g. Ubuntu 26.04 + Python 3.14, where no
ROS 2 apt packages exist), use **Option B: Docker** — it runs ROS 2 Jazzy in a
container and shares the host's `can0` via `--network host`.

### Option B — Docker (recommended on Ubuntu 26.04 / no native ROS 2)

```bash
# 1. Install Docker (once)
sudo apt-get update && sudo apt-get install -y docker.io
sudo systemctl enable --now docker

# 2. Build the image (run from the ros2_bridge/ directory)
cd ros2_bridge
sudo docker build -t motorbrake-bridge -f docker/Dockerfile .

# 3. Bring up the CAN interface ON THE HOST (kernel-level, not in the container)
sudo ip link set can0 up type can bitrate 1000000

# 4. Run the bridge
sudo docker/run.sh
#   open a second shell in the running stack to publish/echo:
sudo docker/run.sh bash
#   then inside:  ros2 topic pub -r 10 --times 5 /brake_command std_msgs/msg/Bool "{data: true}"
#                 ros2 topic echo /brake_status
#   (publish repeatedly, NOT --once — see the warning in "5) Use it" below)
```

`docker/run.sh` uses `--network host --cap-add NET_ADMIN` so the container sees the
host's `can0` and so multiple `ros2` commands discover each other over DDS.
(Add your user to the `docker` group with `sudo usermod -aG docker $USER` + re-login
to drop the `sudo` prefix.)

Then skip to **section 5** to use it. The rest of section 3 / section 4 below is only
for **Option A (native install)**.

## 3A) Build & source the ROS 2 workspace (native)

```bash
# Dependencies
sudo apt install can-utils
pip install python-can            # or: sudo apt install python3-can

# Put these two packages in a workspace and build
mkdir -p ~/brake_ws/src
cp -r motorbrake_msgs motorbrake_bridge ~/brake_ws/src/
cd ~/brake_ws
colcon build
source install/setup.bash
```

## 4) Run the bridge

```bash
ros2 run motorbrake_bridge brake_bridge
# override defaults if needed:
ros2 run motorbrake_bridge brake_bridge --ros-args \
    -p can_channel:=can0 -p can_bitrate:=1000000 -p heartbeat_timeout:=0.1
```

## 5) Use it (commands from the slide)

> ⚠️ **Do not use `ros2 topic pub --once` for commands.** `--once` creates a
> publisher, sends a single message, and exits immediately — often *before* DDS
> discovery with the bridge's subscriber has finished (discovery can take 100 ms
> to several seconds, especially when the publisher and the bridge run in
> separate Docker containers). When that happens the message is dropped and
> **no CAN frame reaches the bus** (you will see nothing in `candump can0` and no
> `/brake_command -> Relay ...` line in the bridge log). This is why a command
> "works sometimes and not others". Publish the command a few times instead so it
> survives the discovery race — the firmware treats a repeated ON/OFF as
> idempotent, so this is safe:

```bash
# Engage the brake (Relay ON) — 5 messages at 10 Hz (~0.5 s), then exits
ros2 topic pub -r 10 --times 5 /brake_command std_msgs/msg/Bool "{data: true}"

# Release the brake (Relay OFF)
ros2 topic pub -r 10 --times 5 /brake_command std_msgs/msg/Bool "{data: false}"

# Watch the brake current / status coming back from the STM32
ros2 topic echo /brake_status

# Set start=0° (brake OFF/released) and stop=5° (brake ON/engaged); saved to flash
ros2 topic pub -r 10 --times 5 /servo_command std_msgs/msg/Float32MultiArray "{data: [0.0, 5.0]}"

# Larger throw: released at 10°, engaged at 90°
ros2 topic pub -r 10 --times 5 /servo_command std_msgs/msg/Float32MultiArray "{data: [10.0, 90.0]}"
```

For real/continuous control, prefer a long-lived publisher (rqt, a GUI button, or
a small node) that completes discovery once and stays up, rather than a fresh
`ros2 topic pub` process per command.

## Safety logic implemented

- **Heartbeat (20 ms):** STM32 streams `/brake_status` so the PC knows the brake controller is alive.
- **Fail-safe (100 ms):** if the bridge stops receiving heartbeats it logs an E-Stop and latches `true` on `/brake_estop` until the link recovers. Hook your higher-level controller to this topic to trigger the system E-Stop.
- **Current validation:** if the relay is commanded ON but `current_ma` stays below ~50 mA, the STM32 latches `watchdog_status = 1` (open-load: broken motor wire or failed motor). The hardware E-Stop on `PD2` also latches `watchdog_status = 1` and forces the relay OFF.

## Troubleshooting

- `Network is down` when starting the node → bring the interface up (step 2).
- No `/brake_status` → check 120 Ω termination, CANH/CANL not swapped, and that both ends use the **same bitrate**.
- `candump can0` shows nothing → wiring/transceiver/termination issue, before involving ROS.
- **A command "works sometimes, not others" / no `0x130` in `candump` when you publish** →
  almost always the `ros2 topic pub --once` discovery race (the one-shot publisher
  exits before DDS finds the bridge, so the message is dropped *before* it ever
  becomes a CAN frame). To confirm: when it fails, the bridge prints **no**
  `/brake_command -> Relay ...` line. Fix: publish repeatedly instead of `--once`,
  e.g. `ros2 topic pub -r 10 --times 5 /brake_command std_msgs/msg/Bool "{data: true}"`,
  or use a long-lived publisher. See the warning in "5) Use it". This is a
  host/ROS-side issue — the STM32 firmware and CAN bus are not involved.
