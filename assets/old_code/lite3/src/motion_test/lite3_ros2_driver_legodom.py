import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Vector3, Quaternion
from nav_msgs.msg import Odometry 
from std_msgs.msg import String 
import socket
import struct
import time
import math

class Lite3ROS2Driver(Node):
    def __init__(self, robot_ip="192.168.1.120"):
        super().__init__('lite3_ros2_driver')
        
        # UDP 网络配置
        self.robot_ip = robot_ip
        self.port = 43893
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        try:
            self.sock.bind(('0.0.0.0', self.port)) 
            self.get_logger().info(f"UDP 接收端口 {self.port} 绑定成功")
        except Exception as e:
            self.get_logger().error(f"绑定端口失败: {e}")

        self.sock.setblocking(False) 
        
        # 速度缓存
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0
        
        # 话题发布者
        self.odom_pub = self.create_publisher(Odometry, 'leg_odom2', 10) 
        
        # 步态指令字典
        self.GAIT_CMDS = {
            'slow': 0x21010300, 'medium': 0x21010307, 'fast': 0x21010303,
            'stair': 0x21010407, 'crawl': 0x21010406
        }
        
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, 'cmd_gait', self.cmd_gait_callback, 10) 
        
        self.create_timer(0.2, self.send_heartbeat)        
        self.create_timer(0.04, self.send_velocity_loop)  
        self.create_timer(0.02, self.receive_data_loop) 
        
        self.wakeup_robot()

    def receive_data_loop(self):
        try:
            data, addr = self.sock.recvfrom(2048)
            if len(data) < 140: return

            code, p_size, cmd_type = struct.unpack('<IiI', data[:12])
            
            # 识别机器人状态上报包 
            if code == 0x0901 and cmd_type == 1:
                body = struct.unpack('<15d', data[20:140])
                
                pos_x, pos_y, pos_yaw = body[9], body[10], body[11]
                vel_x, vel_y, vel_yaw = body[12], body[13], body[14]
                
                # 发布ROS2里程计消息
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"

                # 填充位置
                odom.pose.pose.position.x = pos_x
                odom.pose.pose.position.y = pos_y
                odom.pose.pose.position.z = 0.0

                # 计算四元数 
                odom.pose.pose.orientation.x = 0.0
                odom.pose.pose.orientation.y = 0.0
                odom.pose.pose.orientation.z = math.sin(pos_yaw / 2.0)
                odom.pose.pose.orientation.w = math.cos(pos_yaw / 2.0)

                # 填充速度
                odom.twist.twist.linear.x = vel_x
                odom.twist.twist.linear.y = vel_y
                odom.twist.twist.angular.z = vel_yaw
                
                self.odom_pub.publish(odom)
                
        except (BlockingIOError, socket.error):
            pass 

    def send_simple(self, code, value=0):
        self.sock.sendto(struct.pack('<IiI', code, value, 0), (self.robot_ip, self.port))

    def send_complex(self, code, double_val):
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

    def _read_basic_state(self, per_recv_timeout=0.2):
        """尝试从下一个状态上报包读取 robot_basic_state，超时返回 None。"""
        self.sock.settimeout(per_recv_timeout)
        try:
            data, _ = self.sock.recvfrom(2048)
            if len(data) >= 16:
                code = struct.unpack('<I', data[:4])[0]
                cmd_type = struct.unpack('<I', data[8:12])[0]
                if code == 0x0901 and cmd_type == 1:
                    return struct.unpack('<i', data[12:16])[0]
        except (socket.timeout, BlockingIOError, socket.error):
            pass
        finally:
            self.sock.setblocking(False)
        return None

    def _wait_for_state(self, target_state, timeout=8.0):
        """阻塞等待 robot_basic_state 达到目标值，超时返回 False。"""
        self.get_logger().info(f"等待状态 {target_state}...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            state = self._read_basic_state(per_recv_timeout=0.2)
            if state == target_state:
                self.get_logger().info(f"已到达状态 {target_state}")
                return True
            if state is not None:
                self.get_logger().info(f"当前状态: {state}")
        self.get_logger().warning(f"等待状态 {target_state} 超时（{timeout}s）")
        return False

    def wakeup_robot(self):
        self.get_logger().info("发送回零指令")
        self.send_simple(0x21010C05)
        # 等待回零完成后机器人回到趴下状态（1），再发起立切换
        if not self._wait_for_state(1, timeout=10.0):
            self.get_logger().warning("回零后未检测到趴下状态，仍尝试起立")

        self.get_logger().info("发送起立指令")
        self.send_simple(0x21010202)
        # 等待进入力控站立状态（6）
        if not self._wait_for_state(6, timeout=8.0):
            self.get_logger().warning("起立超时，继续后续初始化")

        self.get_logger().info("切换至移动模式")
        self.send_simple(0x21010D06)
        time.sleep(0.5)
        self.get_logger().info("抢占控制权，切入自主模式")
        self.send_simple(0x21010C03)
        time.sleep(0.5)

    def shutdown_robot(self):
        self.get_logger().info("执行安全关机程序")
        self.vel_x = self.vel_y = self.vel_yaw = 0.0
        time.sleep(0.1)
        self.send_simple(0x21010C02)  # 切回手动模式
        time.sleep(0.5)

        # 读取当前状态，确认机器人不在趴下状态时才发切换指令，避免反向触发
        current_state = None
        deadline = time.time() + 2.0
        while time.time() < deadline:
            current_state = self._read_basic_state(per_recv_timeout=0.3)
            if current_state is not None:
                break

        self.get_logger().info(f"关机前状态: {current_state}")
        if current_state != 1:
            self.get_logger().info("发送趴下指令")
            self.send_simple(0x21010202)
            self._wait_for_state(1, timeout=5.0)
        else:
            self.get_logger().info("已处于趴下状态，跳过切换指令")

        self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    node = Lite3ROS2Driver("192.168.1.120") 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

