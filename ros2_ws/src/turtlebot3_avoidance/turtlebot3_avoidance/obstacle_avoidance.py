import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
	def __init__(self):
		super(). __init__('obstacle_avoidance')	
		self.subscription = self.create_subscription(LaserScan,'/scan',self.lidar_callback,10)
		self.publisher = self.create_publisher(Twist,'/cmd_vel',10)
		self.timer = self.create_timer(0.1,self.avoid_obstacle)
		self.lidar_data = None

	def lidar_callback(self,msg):
		self.lidar_data = msg.ranges
		self.get_logger().info("Get LiDAR Parameters!")

	def avoid_obstacle(self):
		if self.lidar_data is None:
			return
		front_distance = self.lidar_data[len(self.lidar_data)//2]
		cmd = Twist()
		if front_distance < 0.5:
			cmd.angular.z = 0.5
		else:
			cmd.linear.x = 0.2
		self.publisher.publish(cmd)
		
def main(args=None):
	rclpy.init(args = args)
	node = ObstacleAvoidance()
	rclpy.spin(node)
	node.destory_node()
	rcply.shutdown()
if __name__ == '__main__':
	main()
