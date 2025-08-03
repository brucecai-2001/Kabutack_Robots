#!/usr/bin/env python3

from traceback import print_tb
import rclpy
import json
import zmq
import cv2
from go2_scripts.api.sport_api import SportAPI
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
from unitree_go.msg import SportModeState
from unitree_api.msg import Request

class Go2Node(Node):
    def __init__(self):
        super().__init__('kabutack_go2')
        
        ################ 服务器 ZMQ ################
        # 初始化ZMQ
        self._cmd_zmq_context = zmq.Context()
        self._cmd_zmq_socket = self._cmd_zmq_context.socket(zmq.PULL)
        self._cmd_zmq_socket.setsockopt(zmq.CONFLATE, 1)
        self._cmd_zmq_socket.connect("tcp://192.168.31.158:5555")

        self._image_zmq_context = zmq.Context()
        self._image_zmq_socket = self._image_zmq_context.socket(zmq.PUSH)
        self._image_zmq_socket.setsockopt(zmq.CONFLATE, 1)
        self._image_zmq_socket.bind("tcp://*:5556")  # 图像发布端口

        self._state_zmq_context = zmq.Context()
        self._state_zmq_socket = self._state_zmq_context.socket(zmq.PUSH)
        self._state_zmq_socket.setsockopt(zmq.CONFLATE, 1)
        self._state_zmq_socket.bind("tcp://*:5557") # 状态发布端口

        print("ZMQ servers initialized.")
        print("Waiting for commands...")
        
        ################# 订阅 状态,图像 ##################
        # 创建状态订阅者，订阅'sportmodestate'话题
        self.state_suber = self.create_subscription(
            SportModeState,
            'sportmodestate',
            self.state_callback,
            10)
        
        # 状态变量
        self.timestamp = 0  # 时间戳
        self.error_code = 0  # 错误码
        self.mode = 0  # 运动模式
        self.gait_type = 0  # 步态类型
        self.position = [0.0, 0.0, 0.0]  # 位置 [x, y, z]
        self.velocity = [0.0, 0.0, 0.0]  # 速度 [vx, vy, vz]
        
        # IMU状态
        self.quaternion = [0.0, 0.0, 0.0, 0.0]  # 四元数 [w, x, y, z]
        self.angular_velocity = [0.0, 0.0, 0.0]  # 角速度 [wx, wy, wz]
        self.rpy = [0.0, 0.0, 0.0]  # 欧拉角 [roll, pitch, yaw]
        
        # 足端状态
        self.foot_force = [[0.0, 0.0, 0.0] for _ in range(4)]  # 足端力
        self.foot_position = [[0.0, 0.0, 0.0] for _ in range(4)]  # 足端位置
        self.contact = [False] * 4  # 足端接触状态
        
        # 初始化坐标系变量
        self.t = -1  # 时间计数器，初始为-1
        self.px0 = 0.0  # 初始x位置
        self.py0 = 0.0  # 初始y位置
        self.yaw0 = 0.0  # 初始yaw角度

        print("State subscriber initialized.")

        # 图像
        self._bridge = CvBridge()
        self.raw_camera_suber = self.create_subscription(
                    Image, 
                    "raw", 
                    self.raw_image_callback,
                    10)

        print("Image subscriber initialized.")
        
        ################ 发布 动作 ##################
        # 创建请求发布者，发布到'/api/sport/request'话题
        self.req_puber = self.create_publisher(
            Request,
            '/api/sport/request',
            10)
        
         # 创建请求消息和运动客户端
        self.req = Request()
        self.sport_req_builder = SportAPI()

        print("Action publisher initialized.")
        
        # 线程控制变量
        self._running = True

    def stop_zmq_listener(self):
        """停止ZMQ监听器"""
        self._running = False
        
        if self._cmd_zmq_socket:
            self._cmd_zmq_socket.close()
        if self._cmd_zmq_context:
            self._cmd_zmq_context.term()
        
        if self._image_zmq_socket:
            self._image_zmq_socket.close()
        if self._image_zmq_context:
            self._image_zmq_context.term()

        if self._state_zmq_socket:
            self._state_zmq_socket.close()
        if self._state_zmq_context:
            self._state_zmq_context.term()
    
    def _zmq_listen_loop(self):
        """ZMQ监听循环，处理接收到的JSON移动指令"""
        while self._running:
            try:
                if self._cmd_zmq_socket:
                    json_msg = self._cmd_zmq_socket.recv_string(flags=zmq.NOBLOCK)
                    msg_data = json.loads(json_msg)
                    
                    # 解析移动指令参数
                    vx = float(msg_data.get('vx', 0.0))
                    vy = float(msg_data.get('vy', 0.0))
                    vyaw = float(msg_data.get('vyaw', 0.0))
                    
                    # 创建并发布移动请求
                    self.sport_req_builder.move(self.req, vx, vy, vyaw)
                    self.req_puber.publish(self.req)
            except zmq.Again:
                # 非阻塞模式下没有消息可接收
                pass
            except Exception as e:
                self.get_logger().error(f"处理ZMQ消息时出错: {e}")
    
    def state_callback(self, data):
        # 更新所有状态信息
        self.timestamp = data.stamp
        self.error_code = data.error_code
        self.mode = data.mode
        self.gait_type = data.gait_type
        
        # 更新位置和速度
        self.position = list(data.position)
        self.velocity = list(data.velocity)
        
        # 更新IMU状态
        self.quaternion = list(data.imu_state.quaternion)
        self.gyroscope = list(data.imu_state.gyroscope)
        self.accelerometer = list(data.imu_state.accelerometer)
        
        # 更新足端状态
        for i in range(4):
            self.foot_force[i] = data.foot_force[i]
            self.foot_position[i] = data.foot_position_body[i]
        
        # 当t<0时获取机器人当前位置作为初始坐标系
        if self.t < 0:
            self.px0 = data.position[0]
            self.py0 = data.position[1]
            self.yaw0 = data.imu_state.rpy[2]
            print(f"初始位置: x={self.px0:.2f}, y={self.py0:.2f}, yaw={self.yaw0:.2f}")
            print(f"当前模式: {self.mode}, 步态类型: {self.gait_type}")
            self.t = 0
        
        # 通过ZMQ发布状态数据 - 确保所有数据都是JSON可序列化的类型
        state_data = {
            'error_code': int(self.error_code),
            'mode': int(self.mode),
            'gait_type': int(self.gait_type),
            'position': [float(x) for x in self.position],
            'velocity': [float(x) for x in self.velocity],
            'quaternion': [float(x) for x in self.quaternion],
            'gyroscope': [float(x) for x in self.gyroscope],
            'accelerometer': [float(x) for x in self.accelerometer],
            'foot_force': [float(force) for force in self.foot_force],
            'foot_position': [float(pos) for pos in self.foot_position],
        }
        
        try:
            state_json = json.dumps(state_data)
            self._state_zmq_socket.send_string(state_json)
        except Exception as e:
            self.get_logger().error(f"发布状态数据时出错: {e}")
    
    def raw_image_callback(self, msg):
        """Convert ROS image to numpy array and publish via ZMQ"""
        frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 将图像编码为JPEG格式以减少传输数据量
        _, buffer = cv2.imencode('.jpg', frame)
        
        # 通过ZMQ发布图像数据
        self._image_zmq_socket.send(buffer.tobytes())

    def __del__(self):
        """析构函数，确保在对象销毁时停止ZMQ监听器"""
        self.stop_zmq_listener()

def main(args=None):
    """主函数，带有完善的异常处理和资源清理"""
    go2_node = None
    
    try:
        # 初始化rclpy
        rclpy.init(args=args)
        
        # 创建运动请求节点
        go2_node = Go2Node()
        
        # 启动ZMQ监听线程
        import threading
        zmq_thread = threading.Thread(target=go2_node._zmq_listen_loop, daemon=True)
        zmq_thread.start()
        
        print("Go2 Node started successfully. Press Ctrl+C to stop.")
        
        # 运行ROS2节点
        rclpy.spin(go2_node)
        
    except KeyboardInterrupt:
        print("\nReceived interrupt signal. Shutting down gracefully...")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 确保资源正确清理
        if go2_node is not None:
            try:
                go2_node.stop_zmq_listener()
                go2_node.destroy_node()
            except Exception as e:
                print(f"Error during cleanup: {e}")
        
        # 关闭rclpy
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error shutting down rclpy: {e}")
        
        print("Shutdown complete.")

    go2_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()