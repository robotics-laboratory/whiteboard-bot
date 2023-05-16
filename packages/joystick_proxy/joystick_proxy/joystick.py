import rclpy
from rclpy.node import Node
from yaml import load, Loader

from wbb_msgs.msg import Control
from sensor_msgs.msg import Joy


class Joystick(Node):
    def __init__(self):
        super(Joystick, self).__init__('joystick')
        self.sub = self.create_subscription(Joy, "/joy", self.handleJoystick, 10)
        self.pub = self.create_publisher(Control, "/movement", 10)

        # Path to the joy control map yaml file
        self.declare_parameter('control_map', None)
        self.declare_parameter('max_curvature', 2.0)
        self.declare_parameter('min_velocity', 0.1)

        self.max_curv = self.get_parameter('max_curvature').value
        self.min_velocity = self.get_parameter('min_velocity').value
        self.loadConfig()

    def loadConfig(self):
        with open(self.get_parameter('control_map').value, 'r') as f:
            self.joy_map = load(f, Loader=Loader)

    def handleJoystick(self, msg):
        curv = msg.axes[self.joy_map["curvature_axis"]] * self.max_curv
        vel = msg.axes[self.joy_map["velocity_axis"]]
        if abs(vel) < self.min_velocity:
            vel = 0.0  # stop

        #self.get_logger().info('curv: %f; vel: %f' % (curv, vel))

        control = Control()
        control.curvature = curv
        control.velocity = vel
        self.pub.publish(control)

def main():
    rclpy.init()
    joy = Joystick()
    rclpy.spin(joy)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
