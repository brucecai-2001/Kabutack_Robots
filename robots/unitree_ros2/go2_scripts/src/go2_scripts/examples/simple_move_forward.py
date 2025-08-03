#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from go2_scripts.api.sport_api import SportAPI
from unitree_api.msg import Request

class SimpleMoveForward(Node):
    def __init__(self):
        super().__init__('simple_move_forward')
        
        # 创建请求发布者，发布到'/api/sport/request'话题
        self.req_puber = self.create_publisher(
            Request,
            '/api/sport/request',
            10)
        
        # 创建请求消息和运动客户端
        self.req = Request()
        self.sport_req_builder = SportAPI()
        
        self.get_logger().info('简单前进控制节点已启动')
        
        # 等待发布者连接
        time.sleep(1.0)
        
        # 执行前进动作
        self.execute_forward_movement()
    
    def execute_forward_movement(self):
        """执行前进1米的动作"""
        move_speed = 0.3  # 移动速度 m/s
        move_duration = 1.0 / move_speed  # 移动时间 = 距离 / 速度
        
        self.get_logger().info(f'开始前进，速度: {move_speed} m/s，持续时间: {move_duration:.1f} 秒')
        
        # 发送前进命令
        start_time = time.time()
        while time.time() - start_time < move_duration:
            # 向前移动：vx为正值表示前进，vy=0表示不侧移，vyaw=0表示不转向
            self.sport_req_builder.move(self.req, move_speed, 0.0, 0.0)
            self.req_puber.publish(self.req)
            time.sleep(0.1)  # 每100ms发送一次命令
        
        # 停止移动
        self.get_logger().info('停止移动')
        self.sport_req_builder.stop_move(self.req)
        self.req_puber.publish(self.req)
        
        self.get_logger().info('前进1米动作完成！')
        
        # 等待一段时间后关闭节点
        time.sleep(2.0)
        self.get_logger().info('任务完成，关闭节点')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        move_node = SimpleMoveForward()

    except KeyboardInterrupt:
        print('\n用户中断，停止程序')
    except Exception as e:
        print(f'程序出错: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()