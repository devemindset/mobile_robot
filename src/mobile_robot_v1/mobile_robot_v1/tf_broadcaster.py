import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class OdomToTF(Node):
    def __init__(self):
        super().__init__("odometry_to_tf")
        self.subscription = self.create_subscription(
            Odometry,"/odom",self.handle_odometry,10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

    def handle_odometry(self,msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = msg.pose.pose.position.x 
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0 

        quat = tf_transformations.quaternion_from_euler(0,0, msg.pose.pose.orientation.z)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odom_to_tf = OdomToTF()
    rclpy.spin(odom_to_tf)
    odom_to_tf.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()