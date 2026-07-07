import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import socket
import struct
import time

class Lite3ROS2Driver(Node):
    def __init__(self, robot_ip="192.168.1.120"):
        super().__init__('lite3_ros2_driver')
        
        # UDP 网络配置
        self.robot_ip = robot_ip
        self.port = 43893
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 速度缓存
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0
        
        # 步态指令字典
        self.GAIT_CMDS = {
            'slow': 0x21010300,    # 平地低速
            'medium': 0x21010307,  # 平地中速
            'fast': 0x21010303,    # 平地高速
            'stair': 0x21010407,   # 高踏步越障
            'crawl': 0x21010406    # 正常/匍匐切换
        }
        
        # 订阅 ROS2 话题
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, 'cmd_gait', self.cmd_gait_callback, 10) 
        
        # ROS2 定时器
        self.create_timer(0.2, self.send_heartbeat)        
        self.create_timer(0.04, self.send_velocity_loop)  
        # 启动序列 
        self.wakeup_robot()

    def send_simple(self, code, value=0):
        self.sock.sendto(struct.pack('<IiI', code, value, 0), (self.robot_ip, self.port))

    def send_complex(self, code, double_val):
        # <IIId 对应: 3个 uint32_t (指令码, 长度8, 类型1) + double (速度值)
        self.sock.sendto(struct.pack('<IIId', code, 8, 1, double_val), (self.robot_ip, self.port))

    def send_heartbeat(self):
        self.send_simple(0x21040001, 0)

    def send_velocity_loop(self):
        self.send_complex(0x0140, self.vel_x)  
        self.send_complex(0x0145, self.vel_y)   
        self.send_complex(0x0141, self.vel_yaw) 
    def cmd_vel_callback(self, msg):
        self.vel_x = max(-1.0, min(1.0, msg.linear.x))
        self.vel_y = max(-0.5, min(0.5, msg.linear.y))
        self.vel_yaw = max(-1.5, min(1.5, msg.angular.z))

    def cmd_gait_callback(self, msg):
        gait_name = msg.data.lower()
        if gait_name in self.GAIT_CMDS:
            self.get_logger().info(f"切换步态至: {gait_name} ...")
            self.send_simple(self.GAIT_CMDS[gait_name])
        else:
            self.get_logger().warning(f" 未知步态指令: '{gait_name}'。可选包括: slow, medium, fast, stair, crawl")

    def wakeup_robot(self):
        self.get_logger().info("正在回零初始化")
        self.send_simple(0x21010C05)
        time.sleep(3)
        self.get_logger().info("正在起立")
        self.send_simple(0x21010202)
        time.sleep(3)
        self.get_logger().info("切换至移动模式")
        self.send_simple(0x21010D06)
        time.sleep(0.5)
        self.get_logger().info("抢占控制权，切入自主模式")
        self.send_simple(0x21010C03)
        time.sleep(0.5)
        self.get_logger().info("初始化完成，正在监听 /cmd_vel ，/cmd_gait 话题")

    def shutdown_robot(self):
        self.get_logger().info("执行安全关机程序")
        self.vel_x = self.vel_y = self.vel_yaw = 0.0
        time.sleep(0.1) # 等待最后的停止速度发出去
        
        self.send_simple(0x21010C02) # 切手动
        time.sleep(0.5)
        self.send_simple(0x21010202) # 指令趴下
        time.sleep(2)
        self.sock.close()
        self.get_logger().info("已交还手柄控制权并趴下，节点安全退出。")

def main(args=None):
    rclpy.init(args=args)
    node = Lite3ROS2Driver("192.168.1.120") 
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 退出
        pass
    finally:
        node.shutdown_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()