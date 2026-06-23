#!/usr/bin/env python3
"""ROS 2 <-> CAN bridge for the STM32 MotorBrake controller.

Topic  /brake_command (std_msgs/Bool)         -> CAN ID 0x130  (PC -> STM32)
        data:true  -> Relay ON  (engage brake)
        data:false -> Relay OFF (release brake)

CAN ID 0x131 (STM32 -> PC, ~20 ms heartbeat) -> /brake_status (motorbrake_msgs/BrakeStatus)
        [0..3] float32 current_ma  (little-endian)
        [4]    uint8   relay_active
        [5]    uint8   watchdog_status (0 = Normal, 1 = Triggered)
        [6..7] uint16  sequence counter (little-endian)

Topic  /servo_command (std_msgs/Float32)      -> CAN ID 0x132  (PC -> STM32)
        data: angle_deg (0.0–180.0°). This is the "brake engaged" servo
        position; the STM32 stores it in flash and only drives the servo to it
        while the relay is ON.

CAN ID 0x133 (STM32 -> PC, same 20 ms tick)  -> BrakeStatus.servo_angle_deg
        [0..3] float32 brake_angle_deg (little-endian). The angle the STM32 is
        actually holding: the flash-recalled value at boot, then the live
        (clamped) /servo_command value after each overwrite. This is the source
        of truth for servo_angle_deg — the bridge no longer echoes its own
        command.

Fail-safe: if no /brake_status heartbeat arrives for `heartbeat_timeout`
seconds (default 0.1 s = 100 ms), the bridge raises E-Stop and latches
True onto /brake_estop (std_msgs/Bool) until the link recovers.

The CAN link is opened with python-can. Use a SocketCAN device brought up at
the matching bitrate, e.g.:

    sudo ip link set can0 up type can bitrate 1000000
"""

import struct
import threading

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from motorbrake_msgs.msg import BrakeStatus


class BrakeBridge(Node):
    def __init__(self):
        super().__init__('brake_bridge')

        # ---- Parameters --------------------------------------------------
        self.declare_parameter('can_interface', 'socketcan')
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 1000000)
        self.declare_parameter('cmd_can_id', 0x130)
        self.declare_parameter('status_can_id', 0x131)
        self.declare_parameter('servo_cmd_can_id', 0x132)
        self.declare_parameter('servo_status_can_id', 0x133)
        self.declare_parameter('heartbeat_timeout', 0.1)  # seconds (100 ms)

        self.can_interface = self.get_parameter('can_interface').value
        self.can_channel = self.get_parameter('can_channel').value
        self.can_bitrate = int(self.get_parameter('can_bitrate').value)
        self.cmd_can_id = int(self.get_parameter('cmd_can_id').value)
        self.status_can_id = int(self.get_parameter('status_can_id').value)
        self.servo_cmd_can_id = int(self.get_parameter('servo_cmd_can_id').value)
        self.servo_status_can_id = int(self.get_parameter('servo_status_can_id').value)
        self.heartbeat_timeout = float(self.get_parameter('heartbeat_timeout').value)

        # ---- CAN bus -----------------------------------------------------
        try:
            self.bus = can.Bus(
                interface=self.can_interface,
                channel=self.can_channel,
                bitrate=self.can_bitrate,
            )
        except Exception as exc:  # noqa: BLE001 - surface any backend error clearly
            self.get_logger().fatal(
                f'Could not open CAN bus '
                f'({self.can_interface}:{self.can_channel}): {exc}')
            raise

        self.get_logger().info(
            f'CAN bus open on {self.can_interface}:{self.can_channel} '
            f'@ {self.can_bitrate} bps')

        # ---- ROS interfaces ---------------------------------------------
        self.status_pub = self.create_publisher(BrakeStatus, 'brake_status', 10)
        self.estop_pub = self.create_publisher(Bool, 'brake_estop', 10)

        self.cmd_sub = self.create_subscription(
            Bool, 'brake_command', self.on_brake_command, 10)
        self.servo_cmd_sub = self.create_subscription(
            Float32, 'servo_command', self.on_servo_command, 10)

        # ---- Heartbeat / fail-safe state --------------------------------
        self._last_status_time = None      # monotonic time of last RX, None until first frame
        self._estop_active = False
        self._servo_angle_deg = 0.0        # angle the STM32 reports holding (CAN 0x133)
        self._lock = threading.Lock()

        # Check the heartbeat at twice the timeout rate.
        self.create_timer(self.heartbeat_timeout / 2.0, self.check_heartbeat)

        # ---- Background CAN reader --------------------------------------
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    # ---------------------------------------------------------------------
    # PC -> STM32 : /brake_command -> CAN 0x130
    # ---------------------------------------------------------------------
    def on_brake_command(self, msg: Bool):
        data = bytes([1 if msg.data else 0])
        frame = can.Message(
            arbitration_id=self.cmd_can_id,
            data=data,
            is_extended_id=False,
        )
        try:
            self.bus.send(frame)
            self.get_logger().info(
                f'/brake_command -> Relay {"ON" if msg.data else "OFF"} '
                f'(CAN 0x{self.cmd_can_id:03X})')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to send brake command: {exc}')

    # ---------------------------------------------------------------------
    # PC -> STM32 : /servo_command -> CAN 0x132
    # ---------------------------------------------------------------------
    def on_servo_command(self, msg: Float32):
        data = struct.pack('<f', msg.data)
        frame = can.Message(
            arbitration_id=self.servo_cmd_can_id,
            data=data,
            is_extended_id=False,
        )
        try:
            self.bus.send(frame)
            self.get_logger().debug(
                f'/servo_command -> {msg.data:.1f}° (CAN 0x{self.servo_cmd_can_id:03X})')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to send servo command: {exc}')

    # ---------------------------------------------------------------------
    # STM32 -> PC : CAN 0x131 -> /brake_status
    # ---------------------------------------------------------------------
    def _rx_loop(self):
        while self._running:
            try:
                frame = self.bus.recv(timeout=0.2)
            except Exception as exc:  # noqa: BLE001
                if self._running:
                    self.get_logger().error(f'CAN receive error: {exc}')
                continue
            if frame is None:
                continue

            if frame.arbitration_id == self.status_can_id:
                self._handle_brake_status(frame)
            elif frame.arbitration_id == self.servo_status_can_id:
                self._handle_servo_status(frame)

    def _handle_brake_status(self, frame):
        if len(frame.data) < 6:
            self.get_logger().warn(
                f'Short brake_status frame ({len(frame.data)} bytes), ignored')
            return

        current_ma = struct.unpack('<f', bytes(frame.data[0:4]))[0]
        relay_active = bool(frame.data[4])
        watchdog_status = int(frame.data[5])

        status = BrakeStatus()
        status.current_ma = float(current_ma)
        status.relay_active = relay_active
        status.watchdog_status = watchdog_status
        status.servo_angle_deg = self._servo_angle_deg  # echo of last /servo_command
        self.status_pub.publish(status)

        with self._lock:
            self._last_status_time = self.get_clock().now()
            if self._estop_active:
                self._estop_active = False
                self._publish_estop(False)
                self.get_logger().info('Brake heartbeat recovered, E-Stop cleared')

        if watchdog_status != 0:
            self.get_logger().warn(
                'STM32 reports watchdog_status=1 (E-Stop / open-load fault)',
                throttle_duration_sec=1.0)

    def _handle_servo_status(self, frame):
        if len(frame.data) < 4:
            self.get_logger().warn(
                f'Short servo_status frame ({len(frame.data)} bytes), ignored')
            return
        # Held angle reported by the STM32: flash value at boot, live command after.
        # Read in the same _rx_loop thread as _handle_brake_status, so no lock needed.
        self._servo_angle_deg = struct.unpack('<f', bytes(frame.data[0:4]))[0]

    # ---------------------------------------------------------------------
    # Fail-safe : E-Stop when the heartbeat goes silent for > timeout
    # ---------------------------------------------------------------------
    def check_heartbeat(self):
        with self._lock:
            last = self._last_status_time
            if last is None:
                return  # no frame received yet; wait for the link to come up
            elapsed = (self.get_clock().now() - last).nanoseconds * 1e-9
            if elapsed > self.heartbeat_timeout and not self._estop_active:
                self._estop_active = True
                self._publish_estop(True)
                self.get_logger().error(
                    f'E-STOP: no brake heartbeat for {elapsed * 1000:.0f} ms '
                    f'(> {self.heartbeat_timeout * 1000:.0f} ms)')

    def _publish_estop(self, active: bool):
        self.estop_pub.publish(Bool(data=active))

    # ---------------------------------------------------------------------
    def destroy_node(self):
        self._running = False
        try:
            self.bus.shutdown()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BrakeBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
