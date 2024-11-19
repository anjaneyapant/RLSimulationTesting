import rclpy
from rclpy.node import Node
from geometry_msgs import Twist
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(1.0, self.control_loop)
        self.cmd = Twist()
        
        self.start_time = time.time()
        self.state = 'move_forward'
        
    def control_loop(self):
        # Switch between different states of the robot
        
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if self.state == 'move_forward':
            if elapsed_time < 5.0:
                self.move_forward()
            else:
                self.start_time = current_time
                self.state = 'rotate'
                
        elif self.state == 'rotate':
            if elapsed_time < 3.0:
                self.rotate()
            else:
                self.start_time = current_time
                self.state = 'stop'
                
        elif self.state == 'stop':
            self.stop()
            self.get_logger().info('Robot stopped.')
            
    def move_forward(self):
        
        self.cmd.linear.x = 0.2
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Moving forward...')
        
    def rotate(self):
        
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.5
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Rotating...')
        
        
    def stop(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        
def main(args=None):
    
    rclpy.init(args=args)
    
    robot_controller = RobotController()
    
    rclpy.spin(robot_controller)
    
    robot_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()