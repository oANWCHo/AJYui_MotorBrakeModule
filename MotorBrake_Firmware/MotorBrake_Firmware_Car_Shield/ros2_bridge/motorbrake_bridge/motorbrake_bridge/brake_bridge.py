#!/usr/bin/env python3
"""ROS 2 <-> CAN bridge for the STM32 MotorBrake controller.

Topic  /brake_command (std_msgs/Bool)         -> CAN ID 0x130  (PC -> STM32)
        data:true  -> Relay ON  (engage brake)
        data:false -> Relay OFF (release brake)

CAN ID 0x131 (STM32 -> PC, ~20 ms heartbeat) -> /brake_status (motorbrake_msgs/BrakeStatus)
        [0..3] float32 current_ma  (little-endian)
        [4]    uint8   relay_active
        [5]    uint8   bit0 = watchdog_status (0 = Normal, 1 = Triggered),
                       bit1 = PC13 E_Stop live status
        [6..7] uint16  sequence counter (little-endian)

Topic  /servo_command (std_msgs/Float32)      -> CAN ID 0x132  (PC -> STM32)
        data: angle_deg (0.0–180.0°). This is the "brake engaged" servo
        position; the STM32 stores it in flash and only drives the servo to it
        while the relay is ON. The servo position is no longer reported back,
        so BrakeStatus.servo_angle_deg echoes the last commanded angle.

Closed-loop speed control (Curtis 1510). Speed on the wire is a signed
little-endian int16 in 0.01 m/s units (raw = round(m/s * 100), range
±327.67 m/s); the sign is direction (>0 forward, <0 backward, 0 = stop).

CAN ID 0x134 (STM32 -> PC, ~20 ms)            -> /brake_angle (std_msgs/Float32)
        data = current servo/brake angle in whole degrees (0..180).

Topic  /cmd_vel (geometry_msgs/Twist)         -> CAN ID 0x120  (PC -> STM32)
        linear.x -> target wheel speed (m/s), scaled int16.
Topic  /speed_enable (std_msgs/Bool)          -> CAN ID 0x121  (PC -> STM32)
        data:true  -> run the PID speed loop (mode relay ON)
        data:false -> release the Curtis outputs (input side)
CAN ID 0x122 (STM32 -> PC, ~20 ms)            -> /speed_status (motorbrake_msgs/SpeedStatus)
        [0..1] int16 measured_speed, [2..3] int16 target_speed (0.01 m/s/LSB),
        [4] status_flags, [5] fault_code, [6..7] uint16 sequence.
CAN ID 0x123 (STM32 -> PC, ~20 ms)            -> /speed_diagnostics (motorbrake_msgs/SpeedDiagnostics)
        [0] input_flags, [1] output_flags, [2..3] MCOR in mV, [4..5] MCOR out mV,
        [6..7] speed_sensor_hz.

Fail-safe: if no /brake_status heartbeat arrives for `heartbeat_timeout`
seconds (default 0.1 s = 100 ms), the bridge raises E-Stop and latches
True onto /brake_estop (std_msgs/Bool) until the link recovers.

The CAN link is opened with python-can. Use a SocketCAN device brought up at
the matching bitrate (the STM32 runs at 250 kbps), e.g.:

    sudo ip link set can0 up type can bitrate 250000
"""

import struct
import threading

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from motorbrake_msgs.msg import BrakeStatus, SpeedStatus, SpeedDiagnostics

# Speed wire scaling: signed int16, 0.01 m/s per LSB (matches the STM32 firmware).
SPEED_CMD_SCALE = 100.0
_INT16_MIN = -32768
_INT16_MAX = 32767


class BrakeBridge(Node):
    def __init__(self):
        super().__init__('brake_bridge')

        # ---- Parameters --------------------------------------------------
        self.declare_parameter('can_interface', 'socketcan')
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 250000)  # 250 kbps, matches firmware
        self.declare_parameter('cmd_can_id', 0x130)
        self.declare_parameter('status_can_id', 0x131)
        self.declare_parameter('servo_cmd_can_id', 0x132)
        self.declare_parameter('brake_angle_can_id', 0x134)
        self.declare_parameter('cmd_vel_can_id', 0x120)
        self.declare_parameter('speed_enable_can_id', 0x121)
        self.declare_parameter('speed_status_can_id', 0x122)
        self.declare_parameter('speed_diag_can_id', 0x123)
        self.declare_parameter('heartbeat_timeout', 0.1)  # seconds (100 ms)

        self.can_interface = self.get_parameter('can_interface').value
        self.can_channel = self.get_parameter('can_channel').value
        self.can_bitrate = int(self.get_parameter('can_bitrate').value)
        self.cmd_can_id = int(self.get_parameter('cmd_can_id').value)
        self.status_can_id = int(self.get_parameter('status_can_id').value)
        self.servo_cmd_can_id = int(self.get_parameter('servo_cmd_can_id').value)
        self.brake_angle_can_id = int(self.get_parameter('brake_angle_can_id').value)
        self.cmd_vel_can_id = int(self.get_parameter('cmd_vel_can_id').value)
        self.speed_enable_can_id = int(self.get_parameter('speed_enable_can_id').value)
        self.speed_status_can_id = int(self.get_parameter('speed_status_can_id').value)
        self.speed_diag_can_id = int(self.get_parameter('speed_diag_can_id').value)
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
        self.brake_angle_pub = self.create_publisher(Float32, 'brake_angle', 10)
        self.estop_pub = self.create_publisher(Bool, 'brake_estop', 10)
        self.speed_status_pub = self.create_publisher(SpeedStatus, 'speed_status', 10)
        self.speed_diag_pub = self.create_publisher(
            SpeedDiagnostics, 'speed_diagnostics', 10)

        self.cmd_sub = self.create_subscription(
            Bool, 'brake_command', self.on_brake_command, 10)
        self.servo_cmd_sub = self.create_subscription(
            Float32, 'servo_command', self.on_servo_command, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.speed_enable_sub = self.create_subscription(
            Bool, 'speed_enable', self.on_speed_enable, 10)

        # ---- Heartbeat / fail-safe state --------------------------------
        self._last_status_time = None      # monotonic time of last RX, None until first frame
        self._estop_active = False
        self._servo_angle_deg = 0.0        # last angle we commanded; echoed into BrakeStatus
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
            self._servo_angle_deg = float(msg.data)  # echoed into BrakeStatus
            self.get_logger().debug(
                f'/servo_command -> {msg.data:.1f}° (CAN 0x{self.servo_cmd_can_id:03X})')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to send servo command: {exc}')

    # ---------------------------------------------------------------------
    # PC -> STM32 : /cmd_vel.linear.x -> CAN 0x120 (scaled int16 m/s)
    # ---------------------------------------------------------------------
    def on_cmd_vel(self, msg: Twist):
        raw = int(round(msg.linear.x * SPEED_CMD_SCALE))
        raw = max(_INT16_MIN, min(_INT16_MAX, raw))   # clamp to int16
        frame = can.Message(
            arbitration_id=self.cmd_vel_can_id,
            data=struct.pack('<h', raw),
            is_extended_id=False,
        )
        try:
            self.bus.send(frame)
            self.get_logger().debug(
                f'/cmd_vel -> {msg.linear.x:.2f} m/s '
                f'(CAN 0x{self.cmd_vel_can_id:03X})')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to send cmd_vel: {exc}')

    # ---------------------------------------------------------------------
    # PC -> STM32 : /speed_enable -> CAN 0x121
    # ---------------------------------------------------------------------
    def on_speed_enable(self, msg: Bool):
        frame = can.Message(
            arbitration_id=self.speed_enable_can_id,
            data=bytes([1 if msg.data else 0]),
            is_extended_id=False,
        )
        try:
            self.bus.send(frame)
            self.get_logger().info(
                f'/speed_enable -> {"ON" if msg.data else "OFF"} '
                f'(CAN 0x{self.speed_enable_can_id:03X})')
        except can.CanError as exc:
            self.get_logger().error(f'Failed to send speed_enable: {exc}')

    # ---------------------------------------------------------------------
    # STM32 -> PC : CAN 0x131 -> /brake_status, CAN 0x122 -> /speed_status
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
            elif frame.arbitration_id == self.brake_angle_can_id:
                self._handle_brake_angle(frame)
            elif frame.arbitration_id == self.speed_status_can_id:
                self._handle_speed_status(frame)
            elif frame.arbitration_id == self.speed_diag_can_id:
                self._handle_speed_diagnostics(frame)

    def _handle_brake_status(self, frame):
        if len(frame.data) < 6:
            self.get_logger().warn(
                f'Short brake_status frame ({len(frame.data)} bytes), ignored')
            return

        current_ma = struct.unpack('<f', bytes(frame.data[0:4]))[0]
        relay_active = bool(frame.data[4])
        # byte5: bit0 = watchdog_status, bit1 = PC13 E_Stop live status.
        watchdog_status = int(frame.data[5] & 0x01)
        e_stop = bool(frame.data[5] & 0x02)

        status = BrakeStatus()
        status.current_ma = float(current_ma)
        status.relay_active = relay_active
        status.watchdog_status = watchdog_status
        status.e_stop = e_stop
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

    def _handle_brake_angle(self, frame):
        if len(frame.data) < 1:
            self.get_logger().warn(
                f'Short brake_angle frame ({len(frame.data)} bytes), ignored')
            return
        angle = float(frame.data[0])   # whole degrees, 0..180
        self.brake_angle_pub.publish(Float32(data=angle))

    def _handle_speed_status(self, frame):
        # 0x122, 8-byte: [0..1] measured int16, [2..3] target int16 (0.01 m/s/LSB),
        # [4] status_flags, [5] fault_code, [6..7] uint16 sequence (all little-endian).
        if len(frame.data) < 8:
            self.get_logger().warn(
                f'Short speed_status frame ({len(frame.data)} bytes), ignored')
            return
        measured, target, flags, fault_code, sequence = struct.unpack(
            '<hhBBH', bytes(frame.data[0:8]))

        msg = SpeedStatus()
        msg.measured_speed_mps = measured / SPEED_CMD_SCALE
        msg.target_speed_mps = target / SPEED_CMD_SCALE
        msg.controller_enabled = bool(flags & 0x01)
        msg.forward_cmd = bool(flags & 0x02)
        msg.reverse_cmd = bool(flags & 0x04)
        msg.pedal_output_active = bool(flags & 0x08)
        msg.speed_sensor_valid = bool(flags & 0x10)
        msg.timeout_active = bool(flags & 0x20)
        msg.e_stop = bool(flags & 0x40)
        msg.fault_active = bool(flags & 0x80)
        msg.fault_code = int(fault_code)
        msg.sequence = int(sequence)
        self.speed_status_pub.publish(msg)

    def _handle_speed_diagnostics(self, frame):
        # 0x123, 8-byte: [0] input_flags, [1] output_flags, [2..3] MCOR in mV,
        # [4..5] MCOR out mV, [6..7] speed_sensor_hz (all little-endian).
        if len(frame.data) < 8:
            self.get_logger().warn(
                f'Short speed_diagnostics frame ({len(frame.data)} bytes), ignored')
            return
        in_flags, out_flags, mcor_in_mv, mcor_out_mv, sensor_hz = struct.unpack(
            '<BBHHH', bytes(frame.data[0:8]))

        msg = SpeedDiagnostics()
        msg.fwd_input = bool(in_flags & 0x01)
        msg.rev_input = bool(in_flags & 0x02)
        msg.pedal_input = bool(in_flags & 0x04)
        msg.fwd_output = bool(out_flags & 0x01)
        msg.rev_output = bool(out_flags & 0x02)
        msg.pedal_output = bool(out_flags & 0x04)
        msg.mode_relay = bool(out_flags & 0x08)
        msg.mcor_input_v = mcor_in_mv / 1000.0
        msg.mcor_output_v = mcor_out_mv / 1000.0
        msg.speed_sensor_hz = float(sensor_hz)
        self.speed_diag_pub.publish(msg)

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
