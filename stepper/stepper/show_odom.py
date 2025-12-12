import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class ShowOdom(Node):
    def __init__(self):
        super().__init__('show_odom_topic')
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription   # evitar advertencia de variable no usada
        self.get_logger().info('show_odom_topic inicializado')
        
    def cmd_vel_callback(self, msg: Twist):
        #self.get_logger().info(f"msg={msg}")
        v = msg.linear.x     # Velocidad lineal en eje X
        w = msg.angular.z    # Velocidad angular en eje Z
        self.reader.sendMOV(v, w)

    def odom_callback(self, odom: Odometry):
        fid = odom.header.frame_id
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        por = odom.pose.pose.orientation.w
        vl = odom.twist.twist.linear.x
        vw = odom.twist.twist.angular.z
        print(f"fid:{fid}, px:{px:.3f}, py:{py:.3f}, th:{por:.3f}, vl:{vl:.6f}, vw:{vw:.6f}")

#============================================================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ShowOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("¡¡ KeyboardInterrupt !!")
        pass
    node.destroy_node()
    rclpy.shutdown()

#============================================================================================================
if __name__ == '__main__':
    main()

