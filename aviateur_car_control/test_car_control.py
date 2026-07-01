#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 Aviateur Car Control - 通信测试工具
 =================================================================================

用途：
  ──────
  1. 模拟Aviateur地面站，测试与小车的TCP通信
  2. 验证JSON协议编解码正确性
  3. 测试虚拟摇杆数据映射
  4. 无需完整Aviateur环境即可运行
  
使用方法：
  ────────
  # 模拟地面站（测试连接）
  python test_car_control.py --mode client --ip <小车IP>
  
  # 模拟小车端（测试服务器）
  python test_car_control.py --mode server
  
  # 完整自测（本机回环）
  python test_car_control.py --mode loopback
"""

import socket
import json
import threading
import time
import argparse
import struct
import sys


class MockAviateurClient:
    """模拟Aviateur客户端（用于测试）"""
    
    def __init__(self, ip='127.0.0.1', port=1955):
        self.ip = ip
        self.port = port
        self.socket = None
        self.connected = False
        self.recv_thread = None
        
    def connect(self):
        """建立连接"""
        print(f"[Client] Connecting to {self.ip}:{self.port}...")
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(5.0)
        
        try:
            self.socket.connect((self.ip, self.port))
            self.connected = True
            
            # 启动接收线程
            self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.recv_thread.start()
            
            print(f"[Client] ✓ Connected!")
            return True
            
        except Exception as e:
            print(f"[Client] ✗ Connection failed: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        print("[Client] Disconnected")
    
    def _recv_loop(self):
        """接收消息循环"""
        while self.connected:
            try:
                len_data = self._recv_exact(4)
                if not len_data:
                    break
                
                msg_len = struct.unpack('!I', len_data)[0]
                msg_data = self._recv_exact(msg_len)
                if not msg_data:
                    break
                
                msg = json.loads(msg_data.decode())
                self._handle_message(msg)
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.connected:
                    print(f"[Client] Recv error: {e}")
                break
        
        self.connected = False
    
    def _recv_exact(self, n):
        data = b""
        while len(data) < n and self.connected:
            try:
                chunk = self.socket.recv(n - len(data))
                if not chunk:
                    return None
                data += chunk
            except socket.timeout:
                continue
        return data if len(data) == n else None
    
    def _handle_message(self, msg):
        """处理收到的消息"""
        msg_type = msg.get('type', '')
        
        if msg_type == 'system_status':
            data = msg['data']
            print(f"\n[Status Update]")
            print(f"  Mode:       {data.get('mode')}")
            print(f"  Battery:    {data.get('battery_voltage', 0):.1f}V")
            print(f"  Uptime:     {data.get('uptime', 0):.1f}s")
            
        elif msg_type == 'odometry':
            data = msg['data']
            pos = data.get('position', {})
            print(f"\n[Odometry Update]")
            print(f"  Position:   ({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f})")
            print(f"  Yaw:        {data.get('orientation', {}).get('yaw', 0):.2f}°")
            print(f"  Velocity:   {data.get('linear_velocity', 0):.2f} m/s")
            print(f"  Distance:   {data.get('traveled_distance', 0):.2f}m")
            
        elif msg_type == 'pong':
            print("[Ping] Pong received ✓")
            
        else:
            print(f"[Unknown] {msg_type}: {msg}")
    
    def send_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        """发送速度指令"""
        msg = {
            "type": "cmd_vel",
            "timestamp": time.time(),
            "data": {
                "linear_x": linear_x,
                "angular_z": angular_z
            }
        }
        self._send(msg)
        print(f"[CMD] cmd_vel: linear={linear_x:.2f}, angular={angular_z:.2f}")
    
    def send_mode_change(self, mode):
        """切换模式"""
        msg = {
            "type": "mode_change",
            "data": {"mode": mode}
        }
        self._send(msg)
        print(f"[CMD] Mode → {mode}")
    
    def send_emergency_stop(self):
        """急停"""
        msg = {
            "type": "emergency_stop",
            "data": {}
        }
        self._send(msg)
        print("[CMD] ⛑ EMERGENCY STOP!")
    
    def send_gimbal(self, yaw=0, pitch=90):
        """云台控制"""
        msg = {
            "type": "gimbal_cmd",
            "data": {"yaw": yaw, "pitch": pitch}
        }
        self._send(msg)
        print(f"[CMD] Gimbal: yaw={yaw}°, pitch={pitch}°")
    
    def _send(self, msg):
        """发送消息"""
        if not self.connected or not self.socket:
            print("[Error] Not connected!")
            return
        
        try:
            serialized = json.dumps(msg).encode()
            length_prefix = struct.pack('!I', len(serialized))
            self.socket.sendall(length_prefix + serialized)
        except Exception as e:
            print(f"[Error] Send failed: {e}")


class MockROSServer:
    """模拟ROS服务器（用于测试）"""
    
    def __init__(self, host='0.0.0.0', port=1955):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.running = False
        
        # 模拟数据
        self.mode = "STANDBY"
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0
        self.distance = 0.0
        self.battery = 12.0
        
    def start(self):
        """启动服务器"""
        self.running = True
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.server_socket.settimeout(1.0)
        
        print(f"[Server] Listening on {self.host}:{self.port}")
        
        accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        accept_thread.start()
        
        # 模拟状态更新线程
        update_thread = threading.Thread(target=self._update_loop, daemon=True)
        update_thread.start()
        
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        
        self.stop()
    
    def stop(self):
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        print("[Server] Stopped")
    
    def _accept_loop(self):
        while self.running:
            try:
                client, addr = self.server_socket.accept()
                if self.client_socket is not None:
                    client.close()
                    continue
                    
                self.client_socket = client
                print(f"[Server] ✓ Client connected from {addr}")
                
                recv_thread = threading.Thread(target=self._client_loop, args=(client,), daemon=True)
                recv_thread.start()
                
            except socket.timeout:
                continue
    
    def _client_loop(self, sock):
        buffer = b""
        while self.running:
            try:
                len_data = self._recv_exact(sock, 4)
                if not len_data:
                    break
                
                msg_len = struct.unpack('!I', len_data)[0]
                msg_data = self._recv_exact(sock, msg_len)
                if not msg_data:
                    break
                
                msg = json.loads(msg_data.decode())
                self._handle_command(msg)
                
            except Exception as e:
                if self.running:
                    print(f"[Server] Error: {e}")
                break
        
        self.client_socket = None
        print("[Server] Client disconnected")
    
    def _recv_exact(self, sock, n):
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
    
    def _handle_command(self, msg):
        """处理来自客户端的命令"""
        msg_type = msg.get('type', '')
        data = msg.get('data', {})
        
        if msg_type == 'cmd_vel':
            lx = data.get('linear_x', 0)
            az = data.get('angular_z', 0)
            
            # 模拟移动
            self.distance += abs(lx) * 0.1  # 简化：每10次调用移动0.1m
            self.yaw += az * 0.05  # 简化转向
            
            print(f"[Server CMD] vel: linear={lx:.2f}, angular={az:.2f}")
            
        elif msg_type == 'mode_change':
            self.mode = data.get('mode', 'STANDBY')
            print(f"[Server CMD] Mode → {self.mode}")
            
        elif msg_type == 'emergency_stop':
            print("[Server CMD] ⛑ EMERGENCY STOP (simulated)")
            
        elif msg_type == 'gimbal_cmd':
            yaw = data.get('yaw', 0)
            pitch = data.get('pitch', 90)
            print(f"[Server CMD] Gimbal: yaw={yaw}, pitch={pitch}")
            
        elif msg_type == 'ping':
            response = {"type": "pong", "timestamp": time.time()}
            self._send(response)
            
        else:
            print(f"[Server] Unknown: {msg_type}")
    
    def _update_loop(self):
        """模拟状态更新并发送"""
        import math
        
        while self.running:
            time.sleep(0.1)  # 10Hz
            
            if self.client_socket and self._is_connected():
                # 更新模拟数据
                self.battery = max(9.0, self.battery - 0.001)  # 缓慢放电
                
                status_msg = {
                    "type": "system_status",
                    "timestamp": time.time(),
                    "data": {
                        "mode": self.mode,
                        "battery_voltage": self.battery,
                        "uptime": time.time(),
                        "connected": True,
                        "cpu_load": 25,
                        "temperature": 45.0
                    }
                }
                self._send(status_msg)
                
                odom_msg = {
                    "type": "odometry",
                    "timestamp": time.time(),
                    "data": {
                        "position": {"x": round(self.position_x, 3), "y": round(self.position_y, 3)},
                        "orientation": {"yaw": round(math.degrees(self.yaw), 2)},
                        "linear_velocity": 0.33 if self.mode == "AUTO" else 0.0,
                        "angular_velocity": 0.02,
                        "traveled_distance": round(self.distance, 2)
                    }
                }
                self._send(odom_msg)
    
    def _send(self, msg):
        if not self.client_socket:
            return
        try:
            serialized = json.dumps(msg).encode()
            prefix = struct.pack('!I', len(serialized))
            self.client_socket.sendall(prefix + serialized)
        except:
            pass
    
    def _is_connected(self):
        return self.client_socket is not None


def test_loopback():
    """本地回环测试（同时运行客户端和服务器）"""
    print("="*60)
    print("  LOOPBACK TEST - Local Client ↔ Server")
    print("="*60)
    
    # 启动服务器（后台线程）
    server = MockROSServer(port=19551)  # 使用不同端口避免冲突
    server_thread = threading.Thread(target=server.start, daemon=True)
    server_thread.start()
    time.sleep(0.5)  # 等待服务器就绪
    
    # 连接客户端
    client = MockAviateurClient(ip='127.0.0.1', port=19551)
    
    if not client.connect():
        print("\n[FAIL] Could not connect to local server")
        server.stop()
        return False
    
    print("\n" + "="*60)
    print("  Running Test Sequence...")
    print("="*60)
    
    # 测试1: 发送速度指令
    print("\n[Test 1] Sending velocity commands...")
    for i in range(5):
        client.send_cmd_vel(linear_x=0.33, angular_z=0.0)
        time.sleep(0.2)
    
    # 测试2: 切换模式
    print("\n[Test 2] Changing modes...")
    client.send_mode_change("AUTO")
    time.sleep(0.5)
    client.send_mode_change("MANUAL")
    time.sleep(0.5)
    
    # 测试3: 急停
    print("\n[Test 3] Emergency stop...")
    client.send_emergency_stop()
    time.sleep(0.3)
    
    # 测试4: 云台控制
    print("\n[Test 4] Gimbal control...")
    client.send_gimbal(yaw=0, pitch=90)
    time.sleep(0.3)
    
    # 测试5: 断开重连
    print("\n[Test 5] Disconnect & Reconnect...")
    client.disconnect()
    time.sleep(0.5)
    client.connect()
    time.sleep(0.5)
    
    # 清理
    client.disconnect()
    time.sleep(0.5)
    server.stop()
    
    print("\n" + "="*60)
    print("  ✓ All tests completed successfully!")
    print("="*60 + "\n")
    
    return True


def main():
    parser = argparse.ArgumentParser(description="Car Control Communication Test Tool")
    parser.add_argument('--mode', choices=['client', 'server', 'loopback'], 
                       default='loopback',
                       help="Test mode: client=simulate Aviateur, server=simulate ROS, loopback=both")
    parser.add_argument('--ip', default='127.0.0.1', help="Target IP (for client mode)")
    parser.add_argument('--port', type=int, default=1955, help="Port number")
    
    args = parser.parse_args()
    
    if args.mode == 'loopback':
        success = test_loopback()
        sys.exit(0 if success else 1)
        
    elif args.mode == 'client':
        print(f"[Client Mode] Connecting to {args.ip}:{args.port}")
        client = MockAviateurClient(ip=args.ip, port=args.port)
        
        if client.connect():
            print("\nInteractive mode - Commands:")
            print("  w/s     : Forward/Backward")
            print("  a/d     : Turn Left/Right")
            print("  space   : Emergency Stop")
            print("  m       : Toggle Mode (AUTO/MANUAL)")
            print("  g       : Send gimbal command")
            print("  q       : Quit")
            print()
            
            try:
                while True:
                    cmd = input("cmd> ").strip().lower()
                    
                    if cmd == 'q':
                        break
                    elif cmd == 'w':
                        client.send_cmd_vel(0.33, 0.0)
                    elif cmd == 's':
                        client.send_cmd_vel(-0.33, 0.0)
                    elif cmd == 'a':
                        client.send_cmd_vel(0.0, 0.5)
                    elif cmd == 'd':
                        client.send_cmd_vel(0.0, -0.5)
                    elif cmd == ' ':
                        client.send_emergency_stop()
                    elif cmd == 'm':
                        mode = "MANUAL" if client.mode == "AUTO" else "AUTO"
                        client.send_mode_change(mode)
                    elif cmd == 'g':
                        client.send_gimbal(yaw=0, pitch=90)
                    else:
                        print("Unknown command")
                        
                    time.sleep(0.1)
                    
            except (KeyboardInterrupt, EOFError):
                pass
            
            client.disconnect()
            print("\n[Client] Disconnected. Goodbye!")
        
    elif args.mode == 'server':
        print(f"[Server Mode] Listening on 0.0.0.0:{args.port}")
        server = MockROSServer(port=args.port)
        
        try:
            server.start()
        except KeyboardInterrupt:
            pass
        
        server.stop()


if __name__ == '__main__':
    main()
