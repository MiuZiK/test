#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 Aviateur Car Control - ROS Bridge Server (小车端)
 =================================================================================

功能：
  ──────
  1. 监听TCP连接（来自Aviateur地面站）
  2. 接收JSON格式的控制指令
  3. 转发到ROS话题 (/cmd_vel, /ui_xy 等)
  4. 订阅传感器数据并回传给地面站
  
使用方法：
  ────────
  # 在小车上运行（需要ROS环境）
  rosrun aviateur_car_control rosbridge_server.py
  
  # 或指定端口
  rosrun aviateur_car_control rosbridge_server.py --port 1955
  
依赖：
  ────
  - Python 3.6+
  - rospy
  - 标准库 (socket, json, threading)
  
Author: AI Assistant  
Date: 2026-05-06
"""

import socket
import json
import threading
import time
import struct
import argparse
import sys

try:
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[ERROR] ROS not found! This script must run on the robot with ROS installed.")
    print("        Install with: sudo apt install python3-rospy")
    sys.exit(1)


class ROSServerBridge:
    """
    ROS Bridge Server - 小车端
    
    作为TCP服务器，等待Aviateur地面站连接，
    将控制指令转发到ROS话题，并将传感器数据回传。
    """
    
    def __init__(self, host='0.0.0.0', port=1955):
        self.host = host
        self.port = port
        
        # Socket
        self.server_socket = None
        self.client_socket = None
        self.client_addr = None
        
        # 线程
        self.accept_thread = None
        self.recv_thread = None
        self.publish_thread = None
        
        # 运行标志
        self.running = False
        
        # ROS发布者
        self.cmd_vel_pub = None
        self.gimbal_pub = None
        
        # ROS订阅者缓存
        self.latest_odom = None
        self.latest_mode = "STANDBY"
        
        # 心跳
        self.last_ping_time = 0
        self.ping_timeout = 10.0  # 秒
        
        print(f"[Server] Initialized on {host}:{port}")
    
    def start(self):
        """启动服务器"""
        if self.running:
            return
        
        self.running = True
        
        # 初始化ROS节点
        rospy.init_node('aviateur_ros_bridge', anonymous=True)
        
        # 创建发布者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.gimbal_pub = rospy.Publisher('/ui_xy', String, queue_size=10)
        
        # 初始化Socket
        self._init_socket()
        
        # 启动接受连接线程
        self.accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.accept_thread.start()
        
        # 启动状态发布线程
        self.publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.publish_thread.start()
        
        rospy.loginfo(f"[Server] ✓ Listening for connections on {self.host}:{self.port}")
    
    def stop(self):
        """停止服务器"""
        self.running = False
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        rospy.loginfo("[Server] Stopped")
    
    def _init_socket(self):
        """初始化TCP socket"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)  # 只允许一个客户端连接
        self.server_socket.settimeout(1.0)  # 非阻塞accept
    
    def _accept_loop(self):
        """接受连接循环"""
        while self.running and not rospy.is_shutdown():
            try:
                client_sock, addr = self.server_socket.accept()
                
                if self.client_socket is not None:
                    # 已有连接，拒绝新连接
                    msg = json.dumps({"type": "error", "message": "Already connected by another client"})
                    client_sock.sendall(msg.encode())
                    client_sock.close()
                    continue
                
                self.client_socket = client_sock
                self.client_addr = addr
                
                rospy.loginfo(f"[Server] ✓ Client connected from {addr}")
                
                # 启动接收线程
                self.recv_thread = threading.Thread(target=self._recv_loop, args=(client_sock,), daemon=True)
                self.recv_thread.start()
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    rospy.logerr(f"[Server] Accept error: {e}")
    
    def _recv_loop(self, sock):
        """接收消息循环"""
        buffer = b""
        
        while self.running and not rospy.is_shutdown():
            try:
                # 接收长度前缀(4字节)
                len_data = self._recv_exact(sock, 4)
                if not len_data:
                    break
                
                msg_len = struct.unpack('!I', len_data)[0]
                
                # 接收消息体
                msg_data = self._recv_exact(sock, msg_len)
                if not msg_data:
                    break
                
                # 解析并处理
                try:
                    msg = json.loads(msg_data.decode('utf-8'))
                    self._handle_message(msg)
                    
                except json.JSONDecodeError as e:
                    rospy.logwarn(f"[Server] JSON decode error: {e}")
                    continue
                    
            except ConnectionResetError:
                break
            except Exception as e:
                if self.running:
                    rospy.logwarn(f"[Server] Recv error: {e}")
                break
        
        # 连接断开清理
        self.client_socket = None
        self.client_addr = None
        rospy.loginfo("[Server] Client disconnected")
    
    def _recv_exact(self, sock, n):
        """精确接收n字节数据"""
        data = b""
        while len(data) < n and self.running:
            try:
                chunk = sock.recv(n - len(data))
                if not chunk:
                    return None
                data += chunk
            except socket.timeout:
                continue
        return data if len(data) == n else None
    
    def _handle_message(self, msg):
        """处理接收到的消息"""
        msg_type = msg.get('type', '')
        data = msg.get('data', {})
        
        if msg_type == 'cmd_vel':
            self._handle_cmd_vel(data)
            
        elif msg_type == 'mode_change':
            self._handle_mode_change(data)
            
        elif msg_type == 'emergency_stop':
            self._handle_emergency_stop()
            
        elif msg_type == 'gimbal_cmd':
            self._handle_gimbal_cmd(data)
            
        elif msg_type == 'led_toggle':
            self._handle_led_toggle(data)
            
        elif msg_type == 'ping':
            self._handle_ping()
            
        else:
            rospy.logwarn(f"[Server] Unknown message type: {msg_type}")
    
    def _handle_cmd_vel(self, data):
        """处理速度指令"""
        twist = Twist()
        twist.linear.x = data.get('linear_x', 0.0)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = data.get('angular_z', 0.0)
        
        self.cmd_vel_pub.publish(twist)
    
    def _handle_mode_change(self, data):
        """处理模式切换"""
        mode = data.get('mode', 'STANDBY')
        self.latest_mode = mode
        rospy.loginfo(f"[CMD] Mode changed to: {mode}")
    
    def _handle_emergency_stop(self):
        """处理急停"""
        twist = Twist()  # 全零速度
        self.cmd_vel_pub.publish(twist)
        rospy.logwarn("[CMD] ⛑ EMERGENCY STOP!")
    
    def _handle_gimbal_cmd(self, data):
        """处理云台控制"""
        yaw = data.get('yaw', 0)
        pitch = data.get('pitch', 90)
        
        cmd_msg = String()
        cmd_msg.data = f"{yaw},{pitch}"
        self.gimbal_pub.publish(cmd_msg)
    
    def _handle_led_toggle(self, data):
        """处理LED开关"""
        on = data.get('on', False)
        rospy.loginfo(f"[CMD] LED {'ON' if on else 'OFF'}")
        # TODO: 实现实际的LED控制
    
    def _handle_ping(self):
        """处理心跳/保活"""
        self.last_ping_time = time.time()
        # 回复pong
        response = {"type": "pong", "timestamp": time.time()}
        self._send(response)
    
    def _publish_loop(self):
        """状态发布循环（向客户端推送传感器数据）"""
        rate = rospy.Rate(10)  # 10Hz
        
        while self.running and not rospy.is_shutdown():
            try:
                if self.client_socket and self._is_connected():
                    # 发送系统状态
                    status_msg = {
                        "type": "system_status",
                        "timestamp": time.time(),
                        "data": {
                            "mode": self.latest_mode,
                            "battery_voltage": self._get_battery_voltage(),
                            "uptime": time.time() - rospy.get_time(),
                            "connected": True,
                            "cpu_load": self._get_cpu_load(),
                            "temperature": self._get_temperature()
                        }
                    }
                    self._send(status_msg)
                    
                    # 发送里程计数据（如果有）
                    if self.latest_odom:
                        odom_msg = {
                            "type": "odometry",
                            "timestamp": time.time(),
                            "data": {
                                "position": {
                                    "x": round(self.latest_odom.pose.pose.position.x, 3),
                                    "y": round(self.latest_odom.pose.pose.position.y, 3)
                                },
                                "orientation": {
                                    "yaw": round(self._get_yaw_from_odom(), 4)
                                },
                                "linear_velocity": abs(self.latest_odom.twist.twist.linear.x),
                                "angular_velocity": self.latest_odom.twist.twist.angular.z,
                                "traveled_distance": getattr(self, '_traveled_distance', 0.0)
                            }
                        }
                        self._send(odom_msg)
                    
            except Exception as e:
                if self.running:
                    rospy.logdebug(f"[Server] Publish error: {e}")
            
            rate.sleep()
    
    def _send(self, msg_dict):
        """发送JSON消息给客户端"""
        if not self.client_socket:
            return
        
        try:
            serialized = json.dumps(msg_dict).encode('utf-8')
            length_prefix = struct.pack('!I', len(serialized))
            
            self.client_socket.sendall(length_prefix + serialized)
            
        except (BrokenPipeError, ConnectionResetError) as e:
            rospy.logwarn(f"[Server] Send failed: {e}")
            self.client_socket = None
    
    def _is_connected(self):
        """检查客户端是否仍连接"""
        if not self.client_socket:
            return False
        
        # 检查心跳超时
        if time.time() - self.last_ping_time > self.ping_timeout:
            return False
        
        return True
    
    def _get_battery_voltage(self):
        """获取电池电压（模拟/实际实现）"""
        # TODO: 从硬件读取或从其他话题获取
        return 12.0  # 默认值
    
    def _get_cpu_load(self):
        """获取CPU负载"""
        try:
            import psutil
            return psutil.cpu_percent()
        except:
            return 0
    
    def _get_temperature(self):
        """获取温度"""
        # TODO: 从传感器获取
        return 45.0  # 默认值
    
    def _get_yaw_from_odom(self):
        """从里程计提取航向角"""
        if not self.latest_odom:
            return 0.0
        
        import tf.transformations
        q = self.latest_odom.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw


def main():
    parser = argparse.ArgumentParser(description="Aviateur ROS Bridge Server (Robot Side)")
    parser.add_argument('--host', default='0.0.0.0', help='Bind address (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=1955, help='Listen port (default: 1955)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Enable verbose output')
    args = parser.parse_args()
    
    if args.verbose:
        rospy.set_param('log_level', rospy.DEBUG)
    
    server = ROSServerBridge(host=args.host, port=args.port)
    
    try:
        server.start()
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == '__main__':
    main()
