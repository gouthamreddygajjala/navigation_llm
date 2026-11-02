#!/usr/bin/env python3
"""
Nav2 Web Server Node for TurtleBot3
Provides a modern HTTP/WebSocket bridge between ROS 2 / Nav2 and Web Frontends.
"""

import asyncio
import base64
import io
import json
import math
import os
import threading
import time
from typing import Set

import numpy as np
from PIL import Image

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)

from aiohttp import web
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    Twist
)
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Nav2WebServerNode(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('nav2_web_server_node')
        self.loop = loop
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        # Robot state
        self.robot_pose = {'x': -2.0, 'y': -0.5, 'yaw': 0.0}
        self.robot_speed = {'linear': 0.0, 'angular': 0.0}
        self.map_data = None
        self.map_metadata = None
        self.current_plan = []
        self.latest_scan_points = []
        self.last_scan_publish_time = 0.0

        # Nav2 Action state
        self.nav_status = 'IDLE'  # IDLE, PLANNING, NAVIGATING, SUCCEEDED, CANCELED, FAILED
        self.nav_feedback = {
            'distance_remaining': 0.0,
            'navigation_time': 0.0,
            'estimated_time_remaining': 0.0,
            'number_of_recoveries': 0
        }
        self.active_goal_handle = None
        self.active_goal_target = None

        # Connected WebSockets
        self.ws_clients: Set[web.WebSocketResponse] = set()

        # TF2 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS Profiles
        latched_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        sensor_qos = QoSProfile(
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, latched_qos,
            callback_group=self.callback_group
        )
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self.callback_group
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10,
            callback_group=self.callback_group
        )
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos,
            callback_group=self.callback_group
        )
        self.create_subscription(
            Path, '/plan', self.plan_callback, 10,
            callback_group=self.callback_group
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Nav2 Action Client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )

        # Periodic timer for TF lookup & state broadcast (15 Hz)
        self.create_timer(0.066, self.update_and_broadcast_state, callback_group=self.callback_group)

        self.get_logger().info(f'Nav2WebServerNode initialized. Local Web Server will run on http://{self.host}:{self.port}')

    def map_callback(self, msg: OccupancyGrid):
        try:
            width = msg.info.width
            height = msg.info.height
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            oyaw = quaternion_to_yaw(msg.info.origin.orientation)

            # Convert raw 1D int8 to 2D numpy array
            raw_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            # In ROS, row 0 is bottom. We flip upside-down so row 0 is top (standard image coordinate system)
            grid_flipped = np.flipud(raw_data)

            # Map values:
            # -1 -> unknown (128 gray)
            # 0  -> free (255 white)
            # 100 -> occupied (0 black)
            # 1-99 -> interpolated
            img_array = np.full((height, width), 128, dtype=np.uint8)
            img_array[grid_flipped == 0] = 255
            img_array[grid_flipped == 100] = 0
            
            # Grayscale values for partial occupancy
            mask_partial = (grid_flipped > 0) & (grid_flipped < 100)
            img_array[mask_partial] = (255 - (grid_flipped[mask_partial] * 2.55)).astype(np.uint8)

            # Encode as PNG image
            pil_img = Image.fromarray(img_array, mode='L')
            buf = io.BytesIO()
            pil_img.save(buf, format='PNG')
            png_b64 = base64.b64encode(buf.getvalue()).decode('utf-8')

            self.map_metadata = {
                'width': width,
                'height': height,
                'resolution': res,
                'origin_x': ox,
                'origin_y': oy,
                'origin_yaw': oyaw
            }
            self.map_data = png_b64

            self.get_logger().info(f'Received /map: {width}x{height} @ {res}m/cell, Origin: ({ox:.2f}, {oy:.2f})')

            # Broadcast new map to all connected clients
            payload = {
                'type': 'map',
                'metadata': self.map_metadata,
                'image': self.map_data
            }
            self.broadcast_json_threadsafe(payload)
        except Exception as e:
            self.get_logger().error(f'Error processing /map: {e}')

    def odom_callback(self, msg: Odometry):
        self.robot_speed['linear'] = float(msg.twist.twist.linear.x)
        self.robot_speed['angular'] = float(msg.twist.twist.angular.z)

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        # Backup robot pose if TF is not ready
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        # Only use if TF is failing
        if not hasattr(self, '_tf_available') or not self._tf_available:
            self.robot_pose['x'] = float(p.x)
            self.robot_pose['y'] = float(p.y)
            self.robot_pose['yaw'] = float(yaw)

    def scan_callback(self, msg: LaserScan):
        now = time.time()
        # Throttle scan broadcast to ~6 Hz to save bandwidth and UI CPU
        if now - self.last_scan_publish_time < 0.16:
            return
        self.last_scan_publish_time = now

        try:
            rx = self.robot_pose['x']
            ry = self.robot_pose['y']
            ryaw = self.robot_pose['yaw']

            ranges = msg.ranges
            angle = msg.angle_min
            step = 5  # Downsample 360 beams to ~72 beams
            pts = []
            for i in range(0, len(ranges), step):
                r = ranges[i]
                current_angle = angle + i * msg.angle_increment
                if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                    continue
                # Transform to map coordinate frame
                total_angle = ryaw + current_angle
                wx = rx + r * math.cos(total_angle)
                wy = ry + r * math.sin(total_angle)
                pts.append({'x': round(wx, 3), 'y': round(wy, 3)})

            self.latest_scan_points = pts
            self.broadcast_json_threadsafe({
                'type': 'scan',
                'points': pts
            })
        except Exception as e:
            self.get_logger().warn(f'Error in scan callback: {e}')

    def plan_callback(self, msg: Path):
        try:
            pts = []
            # Downsample path points
            step = max(1, len(msg.poses) // 80)
            for i in range(0, len(msg.poses), step):
                p = msg.poses[i].pose.position
                pts.append({'x': round(p.x, 3), 'y': round(p.y, 3)})
            if msg.poses and (len(msg.poses) - 1) % step != 0:
                p = msg.poses[-1].pose.position
                pts.append({'x': round(p.x, 3), 'y': round(p.y, 3)})
            
            self.current_plan = pts
            self.broadcast_json_threadsafe({
                'type': 'plan',
                'points': pts
            })
        except Exception as e:
            self.get_logger().warn(f'Error in plan callback: {e}')

    def update_and_broadcast_state(self):
        # Lookup transform from 'map' to 'base_footprint' (or 'base_link')
        tf_success = False
        for target_frame in ['base_footprint', 'base_link']:
            try:
                t = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())
                self.robot_pose['x'] = float(t.transform.translation.x)
                self.robot_pose['y'] = float(t.transform.translation.y)
                self.robot_pose['yaw'] = float(quaternion_to_yaw(t.transform.rotation))
                tf_success = True
                self._tf_available = True
                break
            except Exception:
                pass

        if not tf_success:
            self._tf_available = False

        # Broadcast state
        state_msg = {
            'type': 'robot_state',
            'pose': {
                'x': round(self.robot_pose['x'], 3),
                'y': round(self.robot_pose['y'], 3),
                'yaw': round(self.robot_pose['yaw'], 3),
                'yaw_deg': round(math.degrees(self.robot_pose['yaw']), 1)
            },
            'speed': {
                'linear': round(self.robot_speed['linear'], 2),
                'angular': round(self.robot_speed['angular'], 2)
            },
            'nav_status': self.nav_status,
            'nav_feedback': self.nav_feedback,
            'target': self.active_goal_target
        }
        self.broadcast_json_threadsafe(state_msg)

    # --- Nav2 Action & Control Commands ---

    def navigate_to(self, x: float, y: float, yaw: float):
        self.get_logger().info(f'Received navigation request to: ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)')
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 navigate_to_pose action server not available!')
            self.nav_status = 'SERVER_UNAVAILABLE'
            return False

        # If a goal is already running, cancel it first
        if self.active_goal_handle is not None and self.nav_status in ['PLANNING', 'NAVIGATING']:
            self.cancel_navigation()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = yaw_to_quaternion(yaw)

        self.active_goal_target = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
        self.nav_status = 'PLANNING'
        self.current_plan = []

        # Also publish to /goal_pose topic for compatibility
        self.goal_pose_pub.publish(goal_msg.pose)

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._action_feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2 server')
            self.nav_status = 'REJECTED'
            self.active_goal_handle = None
            return

        self.get_logger().info('Goal accepted by Nav2 server')
        self.active_goal_handle = goal_handle
        self.nav_status = 'NAVIGATING'

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _action_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.nav_status = 'NAVIGATING'
        self.nav_feedback = {
            'distance_remaining': round(float(fb.distance_remaining), 2),
            'navigation_time': round(float(fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9), 1),
            'estimated_time_remaining': round(float(fb.estimated_time_remaining.sec + fb.estimated_time_remaining.nanosec * 1e-9), 1),
            'number_of_recoveries': int(fb.number_of_recoveries)
        }

    def _goal_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal navigation succeeded!')
            self.nav_status = 'SUCCEEDED'
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal navigation was canceled.')
            self.nav_status = 'CANCELED'
        else:
            self.get_logger().warn(f'Goal failed with status code: {status}')
            self.nav_status = 'FAILED'

        self.active_goal_handle = None
        self.current_plan = []

    def cancel_navigation(self):
        self.get_logger().info('Canceling active navigation goal...')
        if self.active_goal_handle is not None:
            cancel_future = self.active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info('Goal cancel acknowledged'))
            self.active_goal_handle = None
        
        self.nav_status = 'CANCELED'
        self.current_plan = []
        self.emergency_stop()

    def set_initial_pose(self, x: float, y: float, yaw: float):
        self.get_logger().info(f'Setting initial AMCL pose: ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(yaw)

        # Small covariance for certainty
        cov = [0.0] * 36
        cov[0] = 0.25   # X variance
        cov[7] = 0.25   # Y variance
        cov[35] = 0.06  # Yaw variance (approx 15 deg std dev)
        msg.pose.covariance = cov

        self.initial_pose_pub.publish(msg)
        self.robot_pose['x'] = x
        self.robot_pose['y'] = y
        self.robot_pose['yaw'] = yaw

    def teleop_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(max(-0.26, min(0.26, linear)))
        msg.angular.z = float(max(-1.82, min(1.82, angular)))
        self.cmd_vel_pub.publish(msg)

    def emergency_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for _ in range(3):
            self.cmd_vel_pub.publish(msg)

    # --- Asyncio & WebSocket helpers ---

    def broadcast_json_threadsafe(self, data: dict):
        if not self.ws_clients:
            return
        json_str = json.dumps(data)
        self.loop.call_soon_threadsafe(
            asyncio.create_task,
            self._async_broadcast(json_str)
        )

    async def _async_broadcast(self, msg_str: str):
        dead_clients = []
        for ws in list(self.ws_clients):
            try:
                if not ws.closed:
                    await ws.send_str(msg_str)
                else:
                    dead_clients.append(ws)
            except Exception:
                dead_clients.append(ws)
        for dc in dead_clients:
            self.ws_clients.discard(dc)


def find_web_dir():
    # Try installed package share directory first
    try:
        pkg_dir = get_package_share_directory('turtlebot3_nav2_web')
        installed_web = os.path.realpath(os.path.join(pkg_dir, 'web'))
        if os.path.isdir(installed_web):
            return installed_web
    except PackageNotFoundError:
        pass

    # Fallback to current source directory
    src_web = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'web'))
    if os.path.isdir(src_web):
        return src_web

    # Fallback to local workspace relative path
    local_web = os.path.realpath('src/turtlebot3_nav2_web/web')
    if os.path.isdir(local_web):
        return local_web

    raise FileNotFoundError("Could not locate web UI directory!")


async def create_app(ros_node: Nav2WebServerNode):
    app = web.Application()
    web_dir = find_web_dir()
    ros_node.get_logger().info(f'Serving web UI from: {web_dir}')

    async def index_handler(request):
        return web.FileResponse(os.path.join(web_dir, 'index.html'))

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=20.0)
        await ws.prepare(request)
        ros_node.ws_clients.add(ws)
        ros_node.get_logger().info(f'Web client connected (Total: {len(ros_node.ws_clients)})')

        # Send initial map if already available
        if ros_node.map_data is not None and ros_node.map_metadata is not None:
            await ws.send_str(json.dumps({
                'type': 'map',
                'metadata': ros_node.map_metadata,
                'image': ros_node.map_data
            }))

        # Send initial robot state
        await ws.send_str(json.dumps({
            'type': 'robot_state',
            'pose': ros_node.robot_pose,
            'speed': ros_node.robot_speed,
            'nav_status': ros_node.nav_status,
            'nav_feedback': ros_node.nav_feedback,
            'target': ros_node.active_goal_target
        }))

        # Send active plan if exists
        if ros_node.current_plan:
            await ws.send_str(json.dumps({
                'type': 'plan',
                'points': ros_node.current_plan
            }))

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        action = data.get('action')
                        if action == 'navigate':
                            x = float(data['x'])
                            y = float(data['y'])
                            yaw = float(data.get('yaw', 0.0))
                            ros_node.navigate_to(x, y, yaw)
                        elif action == 'cancel':
                            ros_node.cancel_navigation()
                        elif action == 'initial_pose':
                            x = float(data['x'])
                            y = float(data['y'])
                            yaw = float(data.get('yaw', 0.0))
                            ros_node.set_initial_pose(x, y, yaw)
                        elif action == 'teleop':
                            lin = float(data.get('linear', 0.0))
                            ang = float(data.get('angular', 0.0))
                            ros_node.teleop_cmd(lin, ang)
                        elif action == 'stop':
                            ros_node.emergency_stop()
                        elif action == 'request_map':
                            if ros_node.map_data is not None:
                                await ws.send_str(json.dumps({
                                    'type': 'map',
                                    'metadata': ros_node.map_metadata,
                                    'image': ros_node.map_data
                                }))
                    except Exception as err:
                        ros_node.get_logger().error(f'Error processing ws message: {err}')
                elif msg.type == web.WSMsgType.ERROR:
                    ros_node.get_logger().warn(f'WS connection closed with exception: {ws.exception()}')
        finally:
            ros_node.ws_clients.discard(ws)
            ros_node.get_logger().info(f'Web client disconnected (Remaining: {len(ros_node.ws_clients)})')
        return ws

    # REST APIs
    async def api_status(request):
        return web.json_response({
            'pose': ros_node.robot_pose,
            'speed': ros_node.robot_speed,
            'nav_status': ros_node.nav_status,
            'nav_feedback': ros_node.nav_feedback,
            'target': ros_node.active_goal_target,
            'has_map': ros_node.map_data is not None
        })

    async def api_navigate(request):
        data = await request.json()
        x = float(data['x'])
        y = float(data['y'])
        yaw = float(data.get('yaw', 0.0))
        success = ros_node.navigate_to(x, y, yaw)
        return web.json_response({'status': 'ok' if success else 'failed'})

    async def api_cancel(request):
        ros_node.cancel_navigation()
        return web.json_response({'status': 'canceled'})

    async def api_teleop(request):
        data = await request.json()
        ros_node.teleop_cmd(float(data.get('linear', 0.0)), float(data.get('angular', 0.0)))
        return web.json_response({'status': 'ok'})

    app.router.add_get('/', index_handler)
    app.router.add_get('/ws', ws_handler)
    app.router.add_get('/api/status', api_status)
    app.router.add_post('/api/navigate', api_navigate)
    app.router.add_post('/api/cancel', api_cancel)
    app.router.add_post('/api/teleop', api_teleop)

    # Static routes with symlink following
    css_dir = os.path.realpath(os.path.join(web_dir, 'css'))
    js_dir = os.path.realpath(os.path.join(web_dir, 'js'))
    if os.path.isdir(css_dir):
        app.router.add_static('/css', css_dir, follow_symlinks=True)
    if os.path.isdir(js_dir):
        app.router.add_static('/js', js_dir, follow_symlinks=True)

    app.router.add_static('/', web_dir, follow_symlinks=True, show_index=True)

    return app


def main(args=None):
    rclpy.init(args=args)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    ros_node = Nav2WebServerNode(loop)

    # Run ROS 2 executor in background daemon thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ros_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Start aiohttp server in the main asyncio event loop
    app = loop.run_until_complete(create_app(ros_node))
    runner = web.AppRunner(app)
    loop.run_until_complete(runner.setup())
    site = web.TCPSite(runner, ros_node.host, ros_node.port)
    loop.run_until_complete(site.start())

    print(f"\n==================================================================")
    print(f"🚀 Nav2 Local Web Server is LIVE at: http://localhost:{ros_node.port}")
    print(f"==================================================================\n")

    try:
        loop.run_forever()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            ros_node.get_logger().info('Shutting down Nav2 Web Server...')
        except Exception:
            pass
        try:
            loop.run_until_complete(runner.cleanup())
        except Exception:
            pass
        try:
            ros_node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
