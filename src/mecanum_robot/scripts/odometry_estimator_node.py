#!/usr/bin/env python3
import csv
import math
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class OdometryEstimatorNode(Node):
    """
    Independent dead-reckoning odometry estimator. Reads /wheel_speeds (the
    SAME topic your HW3 Forward Kinematics node reads) but never looks at
    Gazebo's ground-truth /odom. Injects noise, runs the FK equations,
    integrates pose over time, and publishes /odom_estimated.

    Compare this against ground-truth /odom (from planar_move) to see how
    the estimate drifts from reality.
    """

    def __init__(self):
        super().__init__('odometry_estimator_node')

        # --- Robot geometry (same as your HW3 FK node) ---------------------
        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('lx', 0.097)
        self.declare_parameter('ly', 0.105)
        self.r  = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value

        # --- Noise model parameters ----------------------------------------
        # Feel free to tune these, but keep an unmodified baseline run to
        # compare against per Section 6.
        self.declare_parameter('encoder_noise_std', 0.05)   # rad/s, gaussian std per wheel per sample
        self.declare_parameter('wheel_bias', [0.02, -0.015, 0.01, -0.02])  # rad/s, fixed per-wheel offset [FL,FR,RL,RR]
        self.declare_parameter('slip_noise_std', 0.08)       # extra gaussian std applied to the vy contribution only

        self.encoder_noise_std = self.get_parameter('encoder_noise_std').value
        self.wheel_bias = self.get_parameter('wheel_bias').value
        self.slip_noise_std = self.get_parameter('slip_noise_std').value

        # --- Pose state (dead-reckoned estimate, starts at origin) ---------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_wheel_speeds = [0.0, 0.0, 0.0, 0.0]

        self.dt = 0.02  # 50 Hz integration step

        # --- I/O -------------------------------------------------------------
        self.wheel_sub = self.create_subscription(
            Float64MultiArray, '/wheel_speeds', self.wheel_speeds_callback, 10)
        self.odom_est_pub = self.create_publisher(Odometry, '/odom_estimated', 10)
        self.timer = self.create_timer(self.dt, self.integrate_step)

        # --- CSV logging so you have data to actually answer Section 6 -----
        self.csv_file = open('odometry_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['t', 'est_x', 'est_y', 'est_theta'])
        self.t_elapsed = 0.0

        self.get_logger().info('Odometry Estimator node started.')

    def wheel_speeds_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 4:
            return
        self.last_wheel_speeds = list(msg.data)

    def apply_noise(self, clean_speeds):
        """
        clean_speeds: [FL, FR, RL, RR] as received from /wheel_speeds.
        Return a NEW list of 4 wheel speeds with noise injected.

        ------------------------------------------------------------------
        TODO(student): implement two of the three error sources described
        in Section 2.2 at the PER-WHEEL level:

          1. Random per-sample noise: add independent gaussian noise to
             each wheel speed, std = self.encoder_noise_std.
             (hint: random.gauss(0, self.encoder_noise_std))

          2. Systematic bias: add the fixed self.wheel_bias[i] offset to
             wheel i, every single timestep (not random - same value every
             call).

        Do NOT implement the slip factor here - that one is applied later,
        directly to vy, inside integrate_step's TODO block, since it's not
        a per-wheel effect the way the other two are.
        ------------------------------------------------------------------
        """
        
        for i in range clean_speeds:
        	clean_speeds[i] += random.gauss(0, self.encoder_noise_std)
        	clean_speeds[i] += wheel_bias[i]
        
        noisy_speeds = list(clean_speeds)  # TODO: replace with real logic
        return noisy_speeds

    def integrate_step(self):
        noisy_speeds = self.apply_noise(self.last_wheel_speeds)
        w_fl, w_fr, w_rl, w_rr = noisy_speeds

        # Reuse your HW3 forward kinematics equations here (body-frame
        # velocities from noisy wheel speeds).
        r, lx, ly = self.r, self.lx, self.ly
        vx = (r / 4.0) * (w_fl + w_fr + w_rl + w_rr)
        vy = (r / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
        wz = (r / (4.0 * (lx + ly))) * (-w_fl + w_fr - w_rl + w_rr)

        # ----------------------------------------------------------------
        # TODO(student):
        #   (a) Apply the slip-factor noise term (Section 2.2, #3) to vy
        #       ONLY, using self.slip_noise_std - this models the Mecanum
        #       roller's tendency to slip more under lateral load than
        #       longitudinal. (hint: vy += random.gauss(0, self.slip_noise_std))
        #
        #   (b) Rotate (vx, vy) from body frame into world frame using the
        #       current heading self.theta (Section 2.3 equations), then
        #       Euler-integrate self.x, self.y, self.theta forward by
        #       self.dt. Update self.x, self.y, self.theta in place.
        # ----------------------------------------------------------------
        vy += random.gauss(0, self.slip_noise_std)
        self.x = vx*(cos(theta)) - (vy(sin(theta)))
        self.y = vx(sin(theta))+vy(cos(theta))
        self.theta = wz
        
        self.x += (self.x)(self.dt)
        self.y += (self.y)(self.dt)
        self.theta += (self.theta)(self.dt)

        self.publish_odometry()
        self.log_pose()

    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom_estimated'
        msg.child_frame_id = 'base_link_estimated'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = self.yaw_to_quaternion(self.theta)
        self.odom_est_pub.publish(msg)

    def log_pose(self):
        self.t_elapsed += self.dt
        self.csv_writer.writerow([f'{self.t_elapsed:.3f}', f'{self.x:.5f}',
                                   f'{self.y:.5f}', f'{self.theta:.5f}'])

    @staticmethod
    def yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
