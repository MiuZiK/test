#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wheeltec Serial Bridge V2 - ROS通信桥接节点
功能:
1. 实现Wheeltec Protocol V2与ROS的桥接
2. 发布 /cmd_vel → 转换为V2协议发送到STM32
3. 接收STM32状态数据 → 发布 /odom, /battery_status 等
4. 心跳监控与急停保护
5. 参数动态调节服务

Author: AI Assistant
Date: 2026-04-27
"""

import rospy
import serial
import struct
import threading
import time
import math
from collections import deque

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import Float32, Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# ==================== Protocol Constants ====================
PROTOCOL_HEAD = 0xAA
PROTOCOL_TAIL = 0x55
PROTOCOL_VERSION = 0x02
MAX_PAYLOAD_LEN = 20

# Command Codes (Host → STM32)
CMD_SET_VELOCITY   = 0x01
CMD_SET_PID        = 0x02
CMD_REQUEST_STATUS = 0x03
CMD_EMERGENCY_STOP = 0x04
CMD_RESET_ENCODER  = 0x05
CMD_SET_CAR_MODE   = 0x06
CMD_HEARTBEAT      = 0x0A

# Response Codes (STM32 → Host)
RESP_STATUS       = 0x81
RESP_ACK          = 0x82
RESP_NACK         = 0x83
RESP_ERROR        = 0x84
RESP_HEARTBEAT    = 0x8A


class CRC16CCITT:
    """CRC16-CCITT 校验"""

    TABLE = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    ]  # 简化版，实际使用完整表

    @staticmethod
    def calc(data):
        crc = 0xFFFF
        for byte in data:
            crc = ((crc << 8) ^ CRC16CCITT.TABLE[(crc >> 8) ^ byte]) & 0xFFFF
        return crc


class WheeltecProtocolV2:
    """Wheeltec Protocol V2 实现 (Python版)"""

    def __init__(self):
        self.tx_seq = 0
        self.rx_seq = 0
        self.lock = threading.Lock()
        self.last_heartbeat_time = 0
        self.heartbeat_ok = True
        self.heartbeat_miss_count = 0
        self.HEARTBEAT_TIMEOUT = 0.5  # seconds
        self.MAX_MISSED = 3

        # 回调
        self.on_status_callback = None
        self.on_error_callback = None

    def build_frame(self, cmd, payload=b''):
        """构建协议帧"""
        if len(payload) > MAX_PAYLOAD_LEN:
            payload = payload[:MAX_PAYLOAD_LEN]

        frame = bytearray()
        frame.append(PROTOCOL_HEAD)
        frame.append(cmd)
        frame.append(len(payload))
        frame.append(self.tx_seq & 0xFF)
        frame.extend(payload)

        # CRC16 on [cmd, len, seq, payload]
        crc_data = bytes(frame[1:4 + len(payload)])
        crc = CRC16CCITT.calc(crc_data)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)

        frame.append(PROTOCOL_TAIL)

        self.tx_seq = (self.tx_seq + 1) & 0xFF
        return bytes(frame)

    def parse_byte(self, byte):
        """解析接收字节，返回完整帧或None"""
        static_state = {
            'state': 'IDLE',
            'frame': bytearray(),
            'payload_idx': 0,
            'expected_len': 0,
            'crc_buf': bytearray()
        }

        # 使用实例变量存储解析状态
        if not hasattr(self, '_parse_state'):
            self._parse_state = {'state': 'IDLE', 'frame': bytearray(),
                                   'payload_idx': 0, 'expected_len': 0}

        s = self._parse_state

        if s['state'] == 'IDLE':
            if byte == PROTOCOL_HEAD:
                s['frame'] = bytearray([byte])
                s['state'] = 'HEAD'

        elif s['state'] == 'HEAD':
            s['frame'].append(byte)
            s['state'] = 'CMD'

        elif s['state'] == 'CMD':
            s['frame'].append(byte)
            if byte <= MAX_PAYLOAD_LEN:
                s['expected_len'] = byte
                s['state'] = 'LEN'
            else:
                s['state'] = 'IDLE'

        elif s['state'] == 'LEN':
            s['frame'].append(byte)
            s['rx_seq'] = byte
            s['payload_idx'] = 0
            if s['expected_len'] == 0:
                s['state'] = 'CRC_LOW'
            else:
                s['state'] = 'PAYLOAD'

        elif s['state'] == 'PAYLOAD':
            s['frame'].append(byte)
            s['payload_idx'] += 1
            if s['payload_idx'] >= s['expected_len']:
                s['state'] = 'CRC_LOW'

        elif s['state'] == 'CRC_LOW':
            s['frame'].append(byte)
            s['crc_low'] = byte
            s['state'] = 'CRC_HIGH'

        elif s['state'] == 'CRC_HIGH':
            s['frame'].append(byte)
            crc_recv = s['crc_low] | (byte << 8)
            s['state'] = 'TAIL'

        elif s['state'] == 'TAIL':
            if byte != PROTOCOL_TAIL:
                s['state'] = 'IDLE'
                return None

            s['frame'].append(byte)

            # 验证CRC
            data_for_crc = bytes(s['frame'][1:-3])  # cmd..payload
            crc_calc = CRC16CCITT.calc(data_for_crc)

            if crc_calc != crc_recv:
                if self.on_error_callback:
                    self.on_error_callback('CRC_FAIL')
                s['state'] = 'IDLE'
                return None

            # 帧有效！
            result = bytes(s['frame'])
            s['state'] = 'IDLE'
            return result

        return None

    def send_velocity(self, serial_port, vx, vy, vz):
        """发送速度命令"""
        payload = struct.pack('<fff', vx, vy, vz)
        frame = self.build_frame(CMD_SET_VELOCITY, payload)
        with self.lock:
            serial_port.write(frame)

    def send_emergency_stop(self, serial_port):
        """发送急停命令"""
        frame = self.build_frame(CMD_EMERGENCY_STOP)
        with self.lock:
            serial_port.write(frame)

    def send_heartbeat(self, serial_port):
        """发送心跳包"""
        frame = self.build_frame(CMD_HEARTBEAT)
        with self.lock:
            serial_port.write(frame)
        self.last_heartbeat_time = time.time()

    def request_status(self, serial_port):
        """请求状态数据"""
        frame = self.build_frame(CMD_REQUEST_STATUS)
        with self.lock:
            serial_port.write(frame)

    def check_heartbeat(self):
        """检查心跳状态"""
        if self.last_heartbeat_time > 0:
            elapsed = time.time() - self.last_heartbeat_time
            if elapsed > self.HEARTBEAT_TIMEOUT:
                self.heartbeat_miss_count += 1
                if self.heartbeat_miss_count >= self.MAX_MISSED:
                    self.heartbeat_ok = False
                    return False
        return True


class SerialBridgeNode:
    """ROS Serial Bridge 主节点"""

    def __init__(self):
        rospy.init_node('wheeltec_serial_bridge_v2', anonymous=False)

        # Parameters
        self.port_name = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud', 115200)
        self.pub_freq = rospy.get_param('~publish_rate', 50)
        self.heartbeat_interval = rospy.get_param('~heartbeat_interval', 0.1)
        self.timeout_threshold = rospy.get_param('~timeout', 0.5)
        self.wheel_base = rospy.get_param('~wheel_base', 0.315)
        self.wheel_perimeter = rospy.get_param('~wheel_perimeter', 0.32)

        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.battery_pub = rospy.Publisher('/battery', BatteryState, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.twist_pub = rospy.Publisher('/cmd_vel_sent', TwistStamped, queue_size=10)

        # Subscribers
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=5)

        # Services
        self.set_pid_srv = None  # TODO: Add service for PID tuning

        # State
        self.protocol = WheeltecProtocolV2()
        self.protocol.on_error_callback = self._on_protocol_error
        self.serial = None
        self.connected = False
        self.current_cmd = Twist()
        self.emergency_stop_flag = False
        self.recv_buffer = bytearray()

        # Thread
        self.serial_thread = None
        self.running = False

        # Odometry integration
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.last_odom_time = rospy.Time.now()

        # Statistics
        self.bytes_received = 0
        self.frames_received = 0
        self.frames_sent = 0
        self.crc_errors = 0

    def connect(self):
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=0.01,  # 10ms read timeout
                write_timeout=0.01
            )
            self.connected = True
            rospy.loginfo(f"[Bridge] Connected to {self.port_name} @ {self.baud_rate} baud")
            return True
        except Exception as e:
            rospy.logerr(f"[Bridge] Connection failed: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """断开连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False
        rospy.loginfo("[Bridge] Disconnected")

    def _on_protocol_error(self, error_type):
        """协议错误回调"""
        self.crc_errors += 1
        rospy.logwarn_throttle(1, f"[Bridge] Protocol error: {error_type} (total: {self.crc_errors})")

    def cmd_vel_callback(self, msg):
        """/cmd_vel 订阅回调"""
        if not self.connected or self.emergency_stop_flag:
            return

        self.current_cmd = msg
        try:
            self.protocol.send_velocity(
                self.serial,
                msg.linear.x,
                msg.linear.y,
                msg.angular.z
            )
            self.frames_sent += 1
        except Exception as e:
            rospy.logerr_throttle(1, f"[Bridge] Send error: {e}")

    def emergency_stop(self):
        """执行急停"""
        self.emergency_stop_flag = True
        if self.connected:
            self.protocol.send_emergency_stop(self.serial)
            stop_cmd = Twist()
            self.odom_pub.publish(stop_cmd)  # 发布零速度
        rospy.logwarn("[BRIDGE] EMERGENCY STOP triggered!")

    def resume_from_estop(self):
        """从急停恢复"""
        self.emergency_stop_flag = False
        rospy.loginfo("[BRIDGE] Resumed from emergency stop")

    def parse_received_data(self, data):
        """解析从STM32接收的数据"""
        for byte in data:
            frame = self.protocol.parse_byte(byte)
            if frame is not None:
                self._handle_received_frame(frame)
                self.frames_received += 1

    def _handle_received_frame(self, frame):
        """处理接收到的完整帧"""
        if len(frame) < 5:
            return

        cmd = frame[1]
        length = frame[2]
        seq = frame[3]
        payload = frame[4:4+length]

        if cmd == RESP_STATUS:
            self._process_status_payload(payload)
        elif cmd == RESP_ACK:
            pass  # ACK received
        elif cmd == RESP_NACK:
            rospy.logwarn(f"[Bridge] NACK received for cmd={seq}")
        elif cmd == RESP_ERROR:
            if len(payload) > 0:
                rospy.logerr(f"[Bridge] Error from STM32: code=0x{payload[0]:02X}")
        elif cmd == RESP_HEARTBEAT:
            self.protocol.last_heartbeat_time = time.time()
            self.protocol.heartbeat_ok = True
            self.protocol.heartbeat_miss_count = 0

    def _process_status_payload(self, payload):
        """处理状态数据载荷"""
        offset = 0

        # Motor speeds (4 * float = 16 bytes)
        motor_speeds = []
        if offset + 16 <= len(payload):
            motor_speeds = list(struct.unpack('<ffff', payload[offset:offset+16]))
            offset += 16

        # Encoder raw values (optional)
        encoders = []
        if offset + 16 <= len(payload):
            encoders = list(struct.unpack('<iiii', payload[offset:offset+16]))
            offset += 16

        # Euler angles (roll, pitch, yaw in radians)
        euler = (0.0, 0.0, 0.0)
        if offset + 12 <= len(payload):
            euler = struct.unpack('<fff', payload[offset:offset+12])
            offset += 12

        # Battery voltage
        voltage = 0.0
        if offset + 4 <= len(payload):
            voltage = struct.unpack('<f', payload[offset:offset+4])[0]
            offset += 4

        # Fault code
        fault_code = 0
        if offset < len(payload):
            fault_code = payload[offset]

        # Publish messages
        self._publish_odom(motor_speeds, euler)
        self._publish_battery(voltage)
        self._publish_joint_states(motor_speeds, encoders)
        self._publish_diagnostics(voltage, fault_code)

    def _publish_odom(self, motor_speeds, euler):
        """发布里程计消息"""
        if len(motor_speeds) >= 2:
            # Tank car model: linear velocity = average of two wheels
            v_left = motor_speeds[0]
            v_right = motor_speeds[1]
            self.vx = (v_left + v_right) / 2.0
            self.vth = (v_right - v_left) / self.wheel_base

            now = rospy.Time.now()
            dt = (now - self.last_odom_time).to_sec()
            if dt > 0 and dt < 1.0:  # Sanity check
                # Update position
                self.x += self.vx * math.cos(self.theta) * dt
                self.y += self.vx * math.sin(self.theta) * dt
                self.theta += self.vth * dt

                # Normalize angle
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            self.last_odom_time = now

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.angular.z = self.vth

            # Quaternion from yaw
            odom.pose.pose.orientation.z = math.sin(self.theta / 2)
            odom.pose.pose.orientation.w = math.cos(self.theta / 2)

            self.odom_pub.publish(odom)

    def _publish_battery(self, voltage):
        """发布电池状态"""
        bat = BatteryState()
        bat.header.stamp = rospy.Time.now()
        bat.voltage = voltage
        bat.percentage = max(0, min(100, (voltage - 10.0) / (12.6 - 10.0) * 100))
        bat.present = True
        self.battery_pub.publish(bat)

    def _publish_joint_states(self, speeds, encoders):
        """发布关节状态"""
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        if len(speeds) >= 2:
            js.velocity = speeds[:2]
        if len(encoders) >= 2:
            js.position = encoders[:2]
        self.joint_pub.publish(js)

    def _publish_diagnostics(self, voltage, fault_code):
        """发布诊断信息"""
        diag = DiagnosticArray()
        diag.header.stamp = rospy.Time.now()

        status = DiagnosticStatus()
        status.name = "Serial Bridge V2"
        status.level = DiagnosticStatus.OK
        status.message = "OK"

        status.values.append(dict(key="Voltage", value=f"{voltage:.2f}V"))
        status.values.append(dict(key="Fault Code", value=f"0x{fault_code:02X}"))
        status.values.append(dict(key="Frames RX", value=str(self.frames_received)))
        status.values.append(dict(key="Frames TX", value=str(self.frames_sent)))
        status.values.append(dict(key="CRC Errors", value=str(self.crc_errors)))
        status.values.append(dict(key="Heartbeat OK", value=str(self.protocol.heartbeat_ok)))

        if voltage < 10.5:
            status.level = DiagnosticStatus.WARN
            status.message = "Low battery"
        if fault_code != 0:
            status.level = DiagnosticStatus.ERROR
            status.message = f"Fault: 0x{fault_code:02X}"

        diag.status.append(status)
        self.diag_pub.publish(diag)

    def serial_read_thread(self):
        """串口读取线程"""
        while self.running and self.connected:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    if data:
                        self.bytes_received += len(data)
                        self.parse_received_data(data)
            except serial.SerialException as e:
                rospy.logerr(f"[Bridge] Read error: {e}")
                break
            except Exception as e:
                rospy.logerr(f"[Bridge] Unexpected error: {e}")

            time.sleep(0.001)  # 1ms sleep to reduce CPU usage

    def heartbeat_thread_func(self):
        """心跳发送线程"""
        rate = rospy.Rate(int(1.0 / self.heartbeat_interval))
        while self.running and not rospy.is_shutdown():
            if self.connected and not self.emergency_stop_flag:
                try:
                    self.protocol.send_heartbeat(self.serial)
                except Exception as e:
                    rospy.logwarn_throttle(1, f"[Bridge] Heartbeat send error: {e}")

            # Check heartbeat response timeout
            if not self.protocol.check_heartbeat():
                rospy.logerr("[Bridge] Heartbeat lost! Emergency stopping...")
                self.emergency_stop()

            rate.sleep()

    def run(self):
        """主运行循环"""
        if not self.connect():
            rospy.sleep(1)
            if not self.connect():
                rospy.logfatal("Cannot connect to serial port. Exiting.")
                return

        self.running = True

        # Start threads
        self.serial_thread = threading.Thread(target=self.serial_read_thread, daemon=True)
        self.serial_thread.start()

        hb_thread = threading.Thread(target=self.heartbeat_thread_func, daemon=True)
        hb_thread.start()

        rospy.loginfo("="*50)
        rospy.loginfo(" Wheeltec Serial Bridge V2 Started")
        rospy.loginfo(f" Port: {self.port_name} @ {self.baud_rate}")
        rospy.loginfo(f" Heartbeat interval: {self.heartbeat_interval}s")
        rospy.loginfo("="*50)

        # Main spin
        rate = rospy.Rate(self.pub_freq)
        while not rospy.is_shutdown():
            # Periodically request status
            if self.connected and int(rospy.get_time()) % 2 == 0:
                try:
                    self.protocol.request_status(self.serial)
                except:
                    pass

            rate.sleep()

        self.running = False
        self.disconnect()


def main():
    node = SerialBridgeNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.disconnect()


if __name__ == '__main__':
    main()
