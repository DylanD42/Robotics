#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class RobotControllerNode(Node):
    """
    Publishes [FL, FR, RL, RR] wheel speeds (rad/s) on a timer, cycling
    through named "schemes" so you can watch the Forward Kinematics node
    turn each one into body motion in Gazebo.

    Before running each scheme: predict vx, vy, wz on paper using the
    equations from Section 2, THEN run it and compare. Your write-up
    (Section 8) asks for this prediction-vs-actual comparison directly.
    """

    def __init__(self):
        super().__init__('robot_controller_node')

        self.wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_speeds', 10)

        self.speed = 5.0          # rad/s, magnitude used by all schemes below
        self.scheme_duration = 4.0  # seconds each scheme runs before advancing

        # ------------------------------------------------------------------
        # Each scheme is a tuple: (name, [FL, FR, RL, RR])
        # Given examples first, then TODO slots for you to fill in.
        # ------------------------------------------------------------------
        s = self.speed
        self.schemes = [
            ('forward',        [ s,  s,  s,  s]),
            ('reverse',        [-s, -s, -s, -s]),
            ('rotate_cw',      [ s, -s,  s, -s]),
	    ('strafe_left',  [-s,s,s,-s]),
	    ('strafe_right',   [s,-s,-s,s]),
	    ('rotate_ccw',   [-s,s,-s,s]),

            # ------------------------------------------------------------
            # TODO(student): add at least THREE more named schemes.
            # Ideas to try (pick at least three, or invent your own):
            #   - 'strafe_left'  / 'strafe_right'  (pure lateral motion)
            #   - 'diagonal_ne'  (forward + strafe combined, no rotation)
            #   - 'rotate_ccw'   (opposite sign of rotate_cw)
            #   - 'diff_drive_style' (constrain to look like a 2-wheel
            #      differential-drive robot -- what wheel-speed pattern
            #      does that require here?)
            # For each, predict vx/vy/wz by hand BEFORE you add it, using
            # the Section 2 equations -- that prediction is what you're
            # checking in your write-up.
            # ------------------------------------------------------------
        ]

        self.scheme_index = 0
        self.timer = self.create_timer(self.scheme_duration, self.advance_scheme)

        self.get_logger().info(f'Robot Controller started. Running scheme: {self.schemes[0][0]}')
        self.publish_current_scheme()

    def publish_current_scheme(self):
        name, speeds = self.schemes[self.scheme_index]
        msg = Float64MultiArray()
        msg.data = speeds
        self.wheel_pub.publish(msg)
        self.get_logger().info(f'Scheme: {name}  ->  wheel speeds {speeds}')

    def advance_scheme(self):
        self.scheme_index = (self.scheme_index + 1) % len(self.schemes)
        self.publish_current_scheme()


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
