#!/usr/bin/env python3

import time
import threading
import struct
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import Trigger, SetBool


from hr300_interfaces.srv import SetValues
from hr300_interfaces.action import MoveJoints


FRAME_STX = 0x02
FRAME_ETX = 0x03
FRAME_MAX_PAYLOAD = 256


def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(payload: bytes) -> bytes:
    length = len(payload)
    crc = crc8_maxim(payload)
    return (
        bytes([FRAME_STX])
        + struct.pack(">H", length)
        + payload
        + bytes([crc, FRAME_ETX])
    )


def parse_frame(buf: bytearray):
    while True:
        stx_pos = buf.find(FRAME_STX)
        if stx_pos == -1:
            return None, bytearray()

        if stx_pos > 0:
            buf = buf[stx_pos:]

        if len(buf) < 5:
            return None, buf

        length = struct.unpack(">H", buf[1:3])[0]

        if length == 0 or length > FRAME_MAX_PAYLOAD:
            buf = buf[1:]
            continue

        frame_end = 3 + length + 2
        if len(buf) < frame_end:
            return None, buf

        payload = bytes(buf[3:3 + length])
        crc_rx = buf[3 + length]
        etx = buf[3 + length + 1]

        if etx != FRAME_ETX:
            buf = buf[1:]
            continue

        if crc8_maxim(payload) != crc_rx:
            buf = buf[1:]
            continue

        return payload, buf[frame_end:]


class ArmDriver:
    SERIAL_COMMANDS = {
        "get_description": "HRGD",
        "get_status": "HRGS",
        "actual_q": "HRGC",
        "get_config_relative_degrees": "HRGJ",
        "get_target_config": "HRGT",
        "get_config_abs": "HRGA",
        "movep": "HRSC",
        "movel": "HRSE",
        "set_jump_XYZ": "HRSJ",
        "servoj": "HRSA",
        "get_fkin": "HRGO",
        "get_inversekin": "HRGI",
        "get_error": "HRGE",
        "get_zero_config": "HRGZ",
        "set_zero_config": "HRSZ",
        "disable_motors": "HRMD",
        "enable_motors": "HRME",
        "get_pid_gains": "HRGP",
        "set_pid_gains": "HRSP",
        "go_zero_config": "HRGH",
        "position_control_off": "HRSV",
        "pnevmo_on": "HRPI",
        "pnevmo_off": "HRPO",
        "conveyer_on": "HRCO",
        "conveyer_off": "HRCF",
        "laser_on": "HRLO",
        "laser_off": "HRLF",
        "get_laser_state": "HRLG",
        "base_linear_position": "HRLS",
        "base_linear_calibrate": "HRLC",
        "get_limits": "HRGL",
        "set_limits": "HRSL",
        "set_offsets": "HRSF",
        "get_offsets": "HRGF",
    }

    def __init__(self, port, baudrate, timeout=2.3):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.lock = threading.Lock()
        self.rx_buf = bytearray()
        self.read_timeout = 3.0
        print(f"Serial connection: {self.ser.port} @ {self.ser.baudrate} baud")

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def _pack_int32_payload(self, data=None, width=6) -> bytes:
        if data is None:
            data = []
        values = [int(v) for v in data]
        if len(values) > width:
            raise ValueError(f"Too many values: expected <= {width}, got {len(values)}")
        values.extend([0] * (width - len(values)))
        return b"".join(v.to_bytes(4, "little", signed=True) for v in values)

    def _send_frame(self, payload: bytes):
        frame = build_frame(payload)
        self.ser.write(frame)
        self.ser.flush()

    def _recv_frame(self):
        deadline = time.time() + self.read_timeout
        while time.time() < deadline:
            waiting = self.ser.in_waiting
            if waiting:
                self.rx_buf += self.ser.read(waiting)

            payload, self.rx_buf = parse_frame(self.rx_buf)
            if payload is not None:
                return payload

            time.sleep(0.005)

        return None

    def _recv_text(self):
        raw = self._recv_frame()
        if raw is None:
            return None
        return raw.decode("utf-8", errors="replace").strip()

    def request(self, method: str, code: str, data=None):
        with self.lock:
            if code not in self.SERIAL_COMMANDS:
                return False, f"Unknown command key: {code}"

            command = self.SERIAL_COMMANDS[code]

            try:
                if method == "get":
                    payload = command.encode("ascii")

                elif method == "tool":
                    if data is None:
                        payload = command.encode("ascii")
                    else:
                        payload = command.encode("ascii") + self._pack_int32_payload(data, width=6)

                elif method == "set":
                    payload = command.encode("ascii") + self._pack_int32_payload(data, width=6)

                elif method == "gcode":
                    payload = str(data if data else code).strip().encode("ascii")

                else:
                    return False, f"Unknown method: {method}"

                self._send_frame(payload)
                text = self._recv_text()

                if text is None:
                    return False, "No response"

                if method == "gcode":
                    return text.lower().startswith("ok"), text

                if ":" not in text:
                    return False, text

                resp_code, resp_payload = text.split(":", 1)

                if resp_code.strip() != command:
                    return False, f"Code mismatch: expected {command}, got {resp_code}"

                values = [x.strip() for x in resp_payload.split(",")]
                return True, values

            except Exception as e:
                return False, str(e)

    # ---- high-level API ----

    def enable_motors(self):
        return self.request("tool", "enable_motors")

    def disable_motors(self):
        return self.request("tool", "disable_motors")

    def go_home(self):
        return self.request("tool", "go_zero_config")

    def pnevmo_on(self):
        return self.request("tool", "pnevmo_on")

    def pnevmo_off(self):
        return self.request("tool", "pnevmo_off")

    def conveyer_on(self):
        return self.request("tool", "conveyer_on")

    def conveyer_off(self):
        return self.request("tool", "conveyer_off")

    def laser_on(self):
        return self.request("tool", "laser_on")

    def laser_off(self):
        return self.request("tool", "laser_off")

    def set_joints_deg(self, joints_deg):
        data = [int(round(float(v) * 1000.0)) for v in joints_deg]
        return self.request("set", "movep", data)

    def set_tcp_shift(self, values_mm_or_deg):
        data = [int(round(float(v) * 1000.0)) for v in values_mm_or_deg]
        return self.request("set", "set_jump_XYZ", data)

    def get_status_full(self):
        ok, payload = self.request("get", "get_status")
        if not ok:
            return None

        try:
            values = [int(x) for x in payload]
            status_code = values[0]
            joints_deg = [v / 1000.0 for v in values[1:]]
            return status_code, joints_deg
        except Exception:
            return None

    def get_joint_positions_from_status(self):
        status = self.get_status_full()
        if status is None:
            return None
        return status[1]

    def get_config_abs(self):
        ok, payload = self.request("get", "get_config_abs")
        if not ok:
            return None
        return [int(x) / 1000.0 for x in payload]

    def get_relative_config(self):
        ok, payload = self.request("get", "actual_q")
        if not ok:
            return None
        return [int(x) / 1000.0 for x in payload]

    def get_target_config(self):
        ok, payload = self.request("get", "get_target_config")
        if not ok:
            return None
        return [int(x) / 1000.0 for x in payload]

    def get_fkin(self):
        ok, payload = self.request("get", "get_fkin")
        if not ok:
            return None
        return [int(x) / 1000.0 for x in payload]

    def get_pid_gains(self):
        ok, payload = self.request("get", "get_pid_gains")
        if not ok:
            return None
        return [int(x) for x in payload]

    def set_pid_gains(self, values):
        return self.request("set", "set_pid_gains", [int(v) for v in values])

    def get_limits(self):
        ok, payload = self.request("get", "get_limits")
        if not ok:
            return None
        vals = [int(x) for x in payload]
        return {
            "max_speed": vals[0] / 1000.0,
            "max_accel": vals[1] / 1000.0,
            "tolerance": vals[2],
        }

    def set_limits(self, max_speed, max_accel, tolerance):
        data = [
            int(round(float(max_speed) * 1000.0)),
            int(round(float(max_accel) * 1000.0)),
            int(tolerance),
        ]
        return self.request("set", "set_limits", data)

    def get_offsets(self):
        ok, payload = self.request("get", "get_offsets")
        if not ok:
            return None
        return [int(x) / 1000.0 for x in payload[:3]]

    def set_offsets(self, x, y, z):
        data = [
            int(round(float(x) * 1000.0)),
            int(round(float(y) * 1000.0)),
            int(round(float(z) * 1000.0)),
        ]
        return self.request("set", "set_offsets", data)


class ArmROS2Driver(Node):
    def __init__(self):
        super().__init__("hr300_driver")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4"])

        port = self.get_parameter("port").value
        baudrate = int(self.get_parameter("baudrate").value)
        update_rate = float(self.get_parameter("update_rate").value)
        self.joint_names = list(self.get_parameter("joint_names").value)

        self.driver = ArmDriver(port, baudrate)
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)

        self.create_service(SetBool, "/arm/enable_motors", self.handle_enable_motors)
        self.create_service(SetBool, "/arm/enable_pnevmo", self.handle_enable_pnevmo)

        self.create_service(Trigger, "/arm/go_home", self.handle_go_home)
        self.create_service(Trigger, "/arm/conveyer_on", self.handle_conveyer_on)
        self.create_service(Trigger, "/arm/conveyer_off", self.handle_conveyer_off)
        self.create_service(Trigger, "/arm/laser_on", self.handle_laser_on)
        self.create_service(Trigger, "/arm/laser_off", self.handle_laser_off)

        self.create_service(SetValues, "/arm/set_joints", self.handle_set_joints)
        self.create_service(SetValues, "/arm/set_pid_gains", self.handle_set_pid_gains)
        self.create_service(SetValues, "/arm/set_tcp_shift", self.handle_set_tcp_shift)
        self.create_service(SetValues, "/arm/set_limits", self.handle_set_limits)
        self.create_service(SetValues, "/arm/set_offsets", self.handle_set_offsets)

        self.create_service(Trigger, "/arm/get_status", self.handle_get_status)
        self.create_service(Trigger, "/arm/get_absolute_angles", self.handle_get_absolute_angles)
        self.create_service(Trigger, "/arm/get_relative_angles", self.handle_get_relative_angles)
        self.create_service(Trigger, "/arm/get_target_angles", self.handle_get_target_angles)
        self.create_service(Trigger, "/arm/get_fkin", self.handle_get_fkin)
        self.create_service(Trigger, "/arm/get_pid_gains", self.handle_get_pid_gains)
        self.create_service(Trigger, "/arm/get_limits", self.handle_get_limits)
        self.create_service(Trigger, "/arm/get_offsets", self.handle_get_offsets)

        self.timer = self.create_timer(1.0 / max(1e-3, update_rate), self.publish_joint_state)

        self.get_logger().info("HR300 ROS2 driver initialized with new framed protocol")

    def _set_trigger_response(self, response, result):
        ok, payload = result
        response.success = bool(ok)
        response.message = ",".join(str(x) for x in payload) if isinstance(payload, list) else str(payload)
        return response

    def handle_enable_motors(self, request, response):
        result = self.driver.enable_motors() if request.data else self.driver.disable_motors()
        return self._set_trigger_response(response, result)

    def handle_enable_pnevmo(self, request, response):
        result = self.driver.pnevmo_on() if request.data else self.driver.pnevmo_off()
        return self._set_trigger_response(response, result)

    def handle_go_home(self, request, response):
        return self._set_trigger_response(response, self.driver.go_home())

    def handle_conveyer_on(self, request, response):
        return self._set_trigger_response(response, self.driver.conveyer_on())

    def handle_conveyer_off(self, request, response):
        return self._set_trigger_response(response, self.driver.conveyer_off())

    def handle_laser_on(self, request, response):
        return self._set_trigger_response(response, self.driver.laser_on())

    def handle_laser_off(self, request, response):
        return self._set_trigger_response(response, self.driver.laser_off())

    def handle_set_joints(self, request, response):
        ok, payload = self.driver.set_joints_deg(request.values)
        response.success = bool(ok)
        response.message = ",".join(str(x) for x in payload) if isinstance(payload, list) else str(payload)
        return response

    def handle_set_tcp_shift(self, request, response):
        ok, payload = self.driver.set_tcp_shift(request.values)
        response.success = bool(ok)
        response.message = ",".join(str(x) for x in payload) if isinstance(payload, list) else str(payload)
        return response

    def handle_set_pid_gains(self, request, response):
        ok, payload = self.driver.set_pid_gains(request.values)
        response.success = bool(ok)
        response.message = ",".join(str(x) for x in payload) if isinstance(payload, list) else str(payload)
        return response

    def handle_set_limits(self, request, response):
        if len(request.values) < 3:
            response.success = False
            response.message = "Expected values: [max_speed, max_accel, tolerance]"
            return response

        ok, payload = self.driver.set_limits(request.values[0], request.values[1], request.values[2])
        response.success = bool(ok)
        response.message = ",".join(str(x) for x in payload) if isinstance(payload, list) else str(payload)
        return response

    def handle_set_offsets(self, request, response):
        if len(request.values) < 3:
            response.success = False
            response.message = "Expected values: [x, y, z]"
            return response

        ok, payload = self.driver.set_offsets(request.values[0], request.values[1], request.values[2])
        response.success = bool(ok)
        response.message = ",".join(str(x) for x in payload) if isinstance(payload, list) else str(payload)
        return response

    def handle_get_status(self, request, response):
        status = self.driver.get_status_full()
        if status is None:
            response.success = False
            response.message = "Failed to get status"
        else:
            code, joints = status
            response.success = True
            response.message = f"status={code},joints=" + ",".join(f"{v:.3f}" for v in joints)
        return response

    def handle_get_absolute_angles(self, request, response):
        vals = self.driver.get_config_abs()
        response.success = vals is not None
        response.message = ",".join(f"{v:.3f}" for v in vals) if vals is not None else "Failed"
        return response

    def handle_get_relative_angles(self, request, response):
        vals = self.driver.get_relative_config()
        response.success = vals is not None
        response.message = ",".join(f"{v:.3f}" for v in vals) if vals is not None else "Failed"
        return response

    def handle_get_target_angles(self, request, response):
        vals = self.driver.get_target_config()
        response.success = vals is not None
        response.message = ",".join(f"{v:.3f}" for v in vals) if vals is not None else "Failed"
        return response

    def handle_get_fkin(self, request, response):
        vals = self.driver.get_fkin()
        response.success = vals is not None
        response.message = ",".join(f"{v:.3f}" for v in vals) if vals is not None else "Failed"
        return response

    def handle_get_pid_gains(self, request, response):
        vals = self.driver.get_pid_gains()
        response.success = vals is not None
        response.message = ",".join(str(v) for v in vals) if vals is not None else "Failed"
        return response

    def handle_get_limits(self, request, response):
        vals = self.driver.get_limits()
        response.success = vals is not None
        response.message = str(vals) if vals is not None else "Failed"
        return response

    def handle_get_offsets(self, request, response):
        vals = self.driver.get_offsets()
        response.success = vals is not None
        response.message = ",".join(f"{v:.3f}" for v in vals) if vals is not None else "Failed"
        return response

    def publish_joint_state(self):
        angles_deg = self.driver.get_joint_positions_from_status()
        if angles_deg is None:
            return

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        angles_deg = angles_deg[:len(self.joint_names)]
        if len(angles_deg) < len(self.joint_names):
            angles_deg += [0.0] * (len(self.joint_names) - len(angles_deg))

        # ROS JointState должен быть в радианах
        msg.position = [float(v) * 3.141592653589793 / 180.0 for v in angles_deg]
        self.publisher.publish(msg)

    def destroy_node(self):
        try:
            self.driver.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = ArmROS2Driver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()