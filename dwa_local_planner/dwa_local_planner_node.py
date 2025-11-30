#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')

        # Parameters (tuned)
        self.declare_parameter('max_vel_x', 0.22)
        self.declare_parameter('min_vel_x', 0.02)
        self.declare_parameter('max_vel_theta', 2.84)
        self.declare_parameter('acc_lim_x', 0.8)
        self.declare_parameter('acc_lim_theta', 3.2)
        self.declare_parameter('predict_time', 2.0)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('v_samples', 11)
        self.declare_parameter('w_samples', 15)
        self.declare_parameter('robot_radius', 0.165)
        self.declare_parameter('obs_cost_gain', 0.3)
        self.declare_parameter('goal_cost_gain', 6.0)
        self.declare_parameter('vel_cost_gain', 0.05)
        self.declare_parameter('smooth_cost_gain', 0.2)

        self.max_vel_x = self.get_parameter('max_vel_x').value
        self.min_vel_x = self.get_parameter('min_vel_x').value
        self.max_vel_theta = self.get_parameter('max_vel_theta').value
        self.acc_lim_x = self.get_parameter('acc_lim_x').value
        self.acc_lim_theta = self.get_parameter('acc_lim_theta').value
        self.predict_time = self.get_parameter('predict_time').value
        self.dt = self.get_parameter('dt').value
        self.v_samples = int(self.get_parameter('v_samples').value)
        self.w_samples = int(self.get_parameter('w_samples').value)
        self.robot_radius = self.get_parameter('robot_radius').value
        self.obs_cost_gain = self.get_parameter('obs_cost_gain').value
        self.goal_cost_gain = self.get_parameter('goal_cost_gain').value
        self.vel_cost_gain = self.get_parameter('vel_cost_gain').value
        self.smooth_cost_gain = self.get_parameter('smooth_cost_gain').value

        # State
        self.odom = None
        self.scan = None
        self.goal = None
        self.current_vel = (0.0, 0.0)  # vx, w

        # Subscriptions & publishers
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # subscribe to both common RViz/clients topics for convenience
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        # Main timer
        self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info('DWA planner node started')

    # ---------- callbacks ----------
    def odom_cb(self, msg: Odometry):
        self.odom = msg
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.current_vel = (vx, wz)

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    def goal_cb(self, msg: PoseStamped):
        # store full PoseStamped
        self.goal = msg
        self.get_logger().info(f'Received new goal')

    # ---------- main loop ----------
    def timer_cb(self):
        if self.odom is None or self.scan is None or self.goal is None:
            return

        best = self.compute_dwa()
        if best is None:
            self.publish_stop()
            return

        cmd = Twist()
        cmd.linear.x = best['v']
        cmd.angular.z = best['w']
        self.cmd_pub.publish(cmd)
        self.get_logger().debug(f'Cmd published: v={cmd.linear.x:.3f}, w={cmd.angular.z:.3f}, cost={best["cost"]:.3f}')

    # ---------- DWA core ----------
    def compute_dwa(self):
        # robot pose
        rx = self.odom.pose.pose.position.x
        ry = self.odom.pose.pose.position.y
        ryaw = self.quat_to_yaw(self.odom.pose.pose.orientation)

        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y

        # if very close, stop
        goal_dist = math.hypot(gx - rx, gy - ry)
        if goal_dist < 0.15:
            self.get_logger().info('Goal reached. Stopping.')
            self.publish_stop()
            return None

        # obstacles: convert scan to global points
        obs = self.scan_to_points(self.scan, rx, ry, ryaw)

        # dynamic window
        vx, vw = self.current_vel
        v_min = max(self.min_vel_x, vx - self.acc_lim_x * self.dt)
        v_max = min(self.max_vel_x, vx + self.acc_lim_x * self.dt)
        w_min = max(-self.max_vel_theta, vw - self.acc_lim_theta * self.dt)
        w_max = min(self.max_vel_theta, vw + self.acc_lim_theta * self.dt)

        # sample velocities
        if self.v_samples > 1:
            v_list = [v_min + i * (v_max - v_min) / (self.v_samples - 1) for i in range(self.v_samples)]
        else:
            v_list = [(v_min + v_max) / 2.0]
        if self.w_samples > 1:
            w_list = [w_min + i * (w_max - w_min) / (self.w_samples - 1) for i in range(self.w_samples)]
        else:
            w_list = [(w_min + w_max) / 2.0]

        best = None
        markers = MarkerArray()
        marker_id = 0

        for v in v_list:
            # ignore tiny forward velocities to avoid rotation-only trivial picks
            if v < self.min_vel_x:
                continue
            for w in w_list:
                traj = self.predict_trajectory(rx, ry, ryaw, v, w)
                clearance = self.min_clearance(traj, obs)

                # relax clearance: require some safety margin
                if clearance < (self.robot_radius * 0.6):
                    continue

                # cost components
                endx, endy, endyaw = traj[-1]
                goal_cost = math.hypot(gx - endx, gy - endy)  # smaller is better
                vel_cost = (self.max_vel_x - v)  # prefer larger v
                smooth_cost = math.hypot(v - vx, w - vw)  # prefer smooth changes
                obs_cost = 1.0 / (clearance + 1e-6)  # inverse clearance

                total_cost = (self.goal_cost_gain * goal_cost +
                              self.obs_cost_gain * obs_cost +
                              self.vel_cost_gain * vel_cost +
                              self.smooth_cost_gain * smooth_cost)

                if best is None or total_cost < best['cost']:
                    best = {'v': v, 'w': w, 'cost': total_cost, 'traj': traj, 'clearance': clearance}

                # trajectory marker (thin)
                m = Marker()
                m.header.frame_id = 'odom'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'dwa_traj'
                m.id = marker_id
                marker_id += 1
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.01
                m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.3)
                m.points = [Point(x=px, y=py, z=0.0) for (px, py, pth) in traj]
                markers.markers.append(m)

        # publish markers and highlight best
        if best is not None:
            chosen = Marker()
            chosen.header.frame_id = 'odom'
            chosen.header.stamp = self.get_clock().now().to_msg()
            chosen.ns = 'dwa_selected'
            chosen.id = 9999
            chosen.type = Marker.LINE_STRIP
            chosen.action = Marker.ADD
            chosen.scale.x = 0.03
            chosen.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
            chosen.points = [Point(x=p[0], y=p[1], z=0.0) for p in best['traj']]
            markers.markers.append(chosen)

        try:
            self.marker_pub.publish(markers)
        except Exception:
            pass

        if best is None:
            self.get_logger().warn('No valid trajectory found, stopping robot.')
            self.publish_stop()
            return None

        # log chosen candidate for debugging
        self.get_logger().info(f"Chosen candidate: v={best['v']:.3f}, w={best['w']:.3f}, cost={best['cost']:.3f}, clearance={best['clearance']:.3f}")

        return best

    # ---------- helpers ----------
    def predict_trajectory(self, x, y, yaw, v, w):
        t = 0.0
        traj = []
        px = x
        py = y
        pth = yaw
        while t < self.predict_time:
            dt = self.dt
            if abs(w) < 1e-6:
                px += v * math.cos(pth) * dt
                py += v * math.sin(pth) * dt
            else:
                # circular arc integration
                px += v / w * (math.sin(pth + w * dt) - math.sin(pth))
                py += v / w * (-math.cos(pth + w * dt) + math.cos(pth))
                pth += w * dt
            traj.append((px, py, pth))
            t += dt
        return traj

    def scan_to_points(self, scan: LaserScan, rx, ry, yaw):
        points = []
        if scan is None:
            return points
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                ox = r * math.cos(angle)
                oy = r * math.sin(angle)
                gx = rx + ox * math.cos(yaw) - oy * math.sin(yaw)
                gy = ry + ox * math.sin(yaw) + oy * math.cos(yaw)
                points.append((gx, gy))
            angle += scan.angle_increment
        return points

    def min_clearance(self, traj, obs_points):
        min_d = float('inf')
        if not obs_points:
            return min_d
        for px, py, _ in traj:
            for ox, oy in obs_points:
                d = math.hypot(ox - px, oy - py) - self.robot_radius
                if d < min_d:
                    min_d = d
            if min_d <= 0.0:
                return 0.0
        return min_d

    def publish_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def quat_to_yaw(self, q):
        # quaternion -> yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
