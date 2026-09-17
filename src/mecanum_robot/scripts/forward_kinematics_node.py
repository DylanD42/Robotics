#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')

        # Robot geometry -- overridable from the launch file, defaults match
        # the real hardware / your HW2 URDF properties.
        self.declare_parameter('wheel_radius', 0.034)   # meters
        self.declare_parameter('lx', 0.097)             # half front-back axle spacing (m)
        self.declare_parameter('ly', 0.105)             # half track width (m)

        self.r  = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value

        # Subscriber: incoming wheel speeds, order = [FL, FR, RL, RR], rad/s
        self.wheel_sub = self.create_subscription(
            Float64MultiArray, '/wheel_speeds', self.wheel_speeds_callback, 10)

        # Publisher: resulting body velocity -> consumed by the planar_move
        # plugin in Gazebo (same topic teleop_twist_keyboard used in HW2).
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Forward Kinematics node started. Waiting on /wheel_speeds...')

    def wheel_speeds_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(f'Expected 4 wheel speeds, got {len(msg.data)}. Ignoring message.')
            return

        w_fl, w_fr, w_rl, w_rr = msg.data

        # ------------------------------------------------------------------
        # TODO(student): Implement the Mecanum forward kinematics equations
        # from Section 2 of the assignment. Given the four wheel speeds
        # above (rad/s) and self.r, self.lx, self.ly, compute:
        #
        #   vx - forward/backward body velocity (m/s)
        #   vy - lateral (strafe) body velocity  (m/s)
        #   wz - angular velocity about z         (rad/s)
        #
        # Derive it yourself from the roller-angle geometry -- don't just
        # copy the formula out of the assignment text without understanding
        # where each sign comes from. You will need this understanding to
        # sanity-check your results in Part C.
        # ------------------------------------------------------------------
        vx = ((self.r)/(4)) * ((w_fl)+(w_fr)+(w_rl)+(w_rr))
        vy = ((self.r)/(4)) * ((-w_fl)+(w_fr)+(w_rl)+(-w_rr))
        wz = ((self.r)/((4)*(self.lx + self.ly))) * ((-w_fl)+(w_fr)+(-w_rl)+(w_rr))
        # ------------------------------------------------------------------

        twist = Twist()
        twist.linear.x  = vx
        twist.linear.y  = vy
        twist.angular.z = wz
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
