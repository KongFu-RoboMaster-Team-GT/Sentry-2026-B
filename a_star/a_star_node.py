import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import numpy as np
import tf_transformations
import math
import heapq
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
# ... (MapManager 和 AStarPlanner 類別保持不變，此處省略以節省篇幅) ...
# 請將原來的 MapManager 和 AStarPlanner 複製過來使用

# ==========================================
# 1. 环境与地图管理 (MapManager)
# ==========================================
class MapManager:
    def __init__(self):
        self.width = 0
        self.height = 0
        self.res = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.grid_map = None 

    def update_map(self, msg: OccupancyGrid):
        self.width = msg.info.width
        self.height = msg.info.height
        self.res = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        raw_data = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        self.grid_map = np.zeros((self.width, self.height), dtype=int)
        self.grid_map = raw_data.T 
        self.grid_map = np.where(self.grid_map > 50, 1, 0)

    def world_to_grid(self, x, y):
        if self.res == 0: return 0, 0
        i = int((x - self.origin_x) / self.res)
        j = int((y - self.origin_y) / self.res)
        i = max(0, min(self.width - 1, i))
        j = max(0, min(self.height - 1, j))
        return i, j

    def grid_to_world(self, i, j):
        x = i * self.res + self.origin_x + self.res / 2.0
        y = j * self.res + self.origin_y + self.res / 2.0
        return x, y

    def is_obstacle(self, x_idx, y_idx):
        if 0 <= x_idx < self.width and 0 <= y_idx < self.height:
            return self.grid_map[x_idx, y_idx] == 1
        return True

# ==========================================
# 2. 全局规划 (AStarPlanner)
# ==========================================
class AStarPlanner:
    def __init__(self, map_mgr):
        self.map_mgr = map_mgr

    def plan(self, start_world, goal_world):
        if self.map_mgr.grid_map is None: return None
        start = self.map_mgr.world_to_grid(*start_world)
        goal = self.map_mgr.world_to_grid(*goal_world)
        
        if self.map_mgr.is_obstacle(goal[0], goal[1]): return None 

        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set, came_from = set(), {}
        g_score = {start: 0}
        f_score = {start: np.hypot(goal[0]-start[0], goal[1]-start[1])}
        pq = [(f_score[start], start)]
        max_iter = 1e9 # 稍微降低一點避免太卡
        iter_count = 0

        while pq:
            iter_count += 1
            if iter_count > max_iter: break 
            curr = heapq.heappop(pq)[1]
            if curr == goal:
                path = []
                while curr in came_from:
                    path.append(self.map_mgr.grid_to_world(*curr))
                    curr = came_from[curr]
                path.append(self.map_mgr.grid_to_world(*start))
                return path[::-1]
            
            close_set.add(curr)
            for dx, dy in neighbors:
                neighbor = (curr[0] + dx, curr[1] + dy)
                if not (0 <= neighbor[0] < self.map_mgr.width and 0 <= neighbor[1] < self.map_mgr.height): continue
                if self.map_mgr.grid_map[neighbor[0], neighbor[1]] == 1 or neighbor in close_set: continue
                
                tentative_g = g_score[curr] + np.hypot(dx, dy)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = curr
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + np.hypot(goal[0]-neighbor[0], goal[1]-neighbor[1])
                    heapq.heappush(pq, (f_score[neighbor], neighbor))
        return None

# ==========================================
# 3. 局部控制 (DWALocalPlanner)
# ==========================================
class DWALocalPlanner(): 
    def __init__(self, config ,marker):
        self.cfg = config
        self.locked_v, self.locked_w = 0.0, 0.0
        self.lock_timer = 0  
        self.last_target = None
        self.markerid = 0
        self.lock_duration_frames = int(1.0 / config["dt"])
        self.marker_array = MarkerArray()
        self.marker = marker
        
    def publish_trajectory(self,traj,score):
        if score < 1:
            return
        marker_array = self.marker_array
        self.markerid += 1

        # 假設我們要發佈 3 條預測軌跡
        marker = Marker()
        marker.header.frame_id = "lidar"
        marker.ns = "prediction"
        marker.id = self.markerid  # 每一條軌跡必須有唯一的 ID
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        
        # 設定線條粗細
        marker.scale.x = 0.01
        # print("score: %f",score)
        # self.get_logger().info("score: %f",score)
        
        # 設定顏色與透明度 (例如：第一條最亮，後面的變暗)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = np.log(score) / 9 # 越後面的預測越透明
        # 設定存活時間 (若 0.5 秒內沒收到新消息就消失，保持畫面乾淨)
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

        # 生成預測點 (這裡模擬不同的轉向路徑)
        for pt in traj:
            pose = Point()
            pose.x = float(pt[0])
            pose.y = float(pt[1])
            pose.z = 0.0
            marker.points.append(pose)

        marker_array.markers.append(marker)
        if len(marker_array.markers) > 50:
            marker_array.markers.pop(0)

        self.marker.publish(marker_array)
    def _predict_trajectory(self, state, v, w):
        dt = self.cfg["dt"]
        # 這裡會根據 predict_time 生成足夠多的點
        steps = int(self.cfg["predict_time"] / dt)
        times = np.linspace(dt, self.cfg["predict_time"], steps)
        
        x0, y0, theta0 = state[0], state[1], state[2]
        
        if abs(w) < 0.001:
            traj_x = x0 + v * np.cos(theta0) * times
            traj_y = y0 + v * np.sin(theta0) * times
            final_theta = theta0
        else:
            r = v / w
            traj_x = x0 - r * np.sin(theta0) + r * np.sin(theta0 + w * times)
            traj_y = y0 + r * np.cos(theta0) - r * np.cos(theta0 + w * times)
            final_theta = theta0 + w * times[-1]
            
        trajectory = np.stack([traj_x, traj_y], axis=1)
        return trajectory, final_theta

    def _check_collision(self, trajectory, map_mgr, emergency=False):
        if map_mgr.grid_map is None: return False, 0.0
        for point in trajectory:
            u, v = map_mgr.world_to_grid(point[0], point[1])
            if not (0 <= u < map_mgr.width and 0 <= v < map_mgr.height):
                return False, 0.0
            if map_mgr.grid_map[u, v] == 1:
                return False, 0.0
        return True, 1.0 

    def compute_control(self, state, current_vel, local_target, final_goal, map_mgr):
        if self.last_target is not None:
            if np.hypot(local_target[0] - self.last_target[0], local_target[1] - self.last_target[1]) > 0.3:
                self.lock_timer = 0
        self.last_target = local_target

        if self.lock_timer > 0:
            test_traj, _ = self._predict_trajectory(state, self.locked_v, self.locked_w)
            is_safe, _ = self._check_collision(test_traj, map_mgr)
            if is_safe:
                self.lock_timer -= 1
                return self.locked_v, self.locked_w, test_traj
            self.lock_timer = 0

        dx, dy = local_target[0] - state[0], local_target[1] - state[1]
        target_theta = np.arctan2(dy, dx)
        
        min_v = max(self.cfg["min_speed"], current_vel[0] - self.cfg["max_accel"] * self.cfg["dt"])
        max_v = min(self.cfg["max_speed"], current_vel[0] + self.cfg["max_accel"] * self.cfg["dt"])
        min_w = max(-self.cfg["max_yaw_rate"], current_vel[1] - self.cfg["max_delta_yaw_rate"] * self.cfg["dt"])
        max_w = min(self.cfg["max_yaw_rate"], current_vel[1] + self.cfg["max_delta_yaw_rate"] * self.cfg["dt"])

        vs = np.linspace(min_v, max_v, num=10)
        ws = np.linspace(min_w, max_w, num=20)

        best_v, best_w, max_score, best_traj = 0.0, 0.0, -float('inf'), None

        for v in vs:
            for w in ws:
                if v < 0.05 and abs(w) < 0.1: continue

                traj, final_theta = self._predict_trajectory(state, v, w)
                is_safe, _ = self._check_collision(traj, map_mgr)
                
                if not is_safe: continue

                heading_align = (np.cos(target_theta - final_theta) + 1.0) / 2.0
                effective_progress = v * (heading_align ** 2) 
                dist_gain = (np.hypot(dx, dy) - np.hypot(traj[-1,0]-local_target[0], traj[-1,1]-local_target[1]))
                dist_to_final = np.linalg.norm(traj[-1] - final_goal)
                arrival_bonus = 100.0 if dist_to_final < self.cfg["robot_radius"] else 0.0
                
                score = (effective_progress * 5.0) + (dist_gain * 3.0) + arrival_bonus
                self.publish_trajectory(traj,score)

                if score > max_score:
                    max_score = score
                    best_v = v
                    best_w = w
                    best_traj = traj

        if best_traj is not None:
            if abs(best_w - self.locked_w) > 0.5: 
                self.lock_timer = self.lock_duration_frames
            self.locked_v, self.locked_w = best_v, best_w
            return best_v, best_w, best_traj
        
        return 0.0, 0.0, None

# ==========================================
# 4. ROS 2 节点实现
# ==========================================
class AdvancedDWAPlannerNode(Node):
    def __init__(self):
        super().__init__('advanced_dwa_node')

        # --- 配置參數調整 ---
        self.config = {
            "max_speed": 0.5,
            "min_speed": 0.0,
            "max_yaw_rate": 1.0,
            "max_accel": 1.0,
            "max_delta_yaw_rate": 3.0,
            "dt": 0.1,
            # [重要修改] 增加預測時間，從 0.5 改為 3.0
            # 這樣軌跡線會變得足夠長
            "predict_time": 10,  
            "robot_radius": 0.3,
            "look_ahead_dist": 1.0
        }

        self.current_pose = None
        self.current_vel = [0.0, 0.0]
        self.goal = None
        self.global_path = []
        self.path_msg = Path()
        self.marker = self.create_publisher(MarkerArray, '/predict_marker', 10)
        
        self.map_mgr = MapManager()
        self.planner = AStarPlanner(self.map_mgr)
        self.dwa = DWALocalPlanner(self.config,self.marker)


        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 發佈最佳預測軌跡
        self.path_pub = self.create_publisher(Path, '/best_trajectory', 10)
        self.global_path_pub = self.create_publisher(Path, '/global_path', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/fastlio2/lio_odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/projected_map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.timer = self.create_timer(self.config["dt"], self.control_loop)
        self.get_logger().info("DWA Node Started")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.current_pose = np.array([pos.x, pos.y, yaw])
        self.current_vel = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]

    def map_callback(self, msg):
        self.map_mgr.update_map(msg)
        if self.goal is not None and self.current_pose is not None:
            self.control_loop(True)

    def goal_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.control_loop(True)

    def trigger_global_plan(self):
        if self.current_pose is None or self.map_mgr.grid_map is None: return
        path = self.planner.plan(self.current_pose[:2], self.goal)
        if path:
            self.global_path = path
            self.publish_global_path(self.global_path)
        else:
            self.global_path = []

    def get_local_target(self):
        if not self.global_path: return self.goal
        dists = [np.hypot(p[0]-self.current_pose[0], p[1]-self.current_pose[1]) for p in self.global_path]
        nearest_idx = np.argmin(dists)
        look_ahead = self.config["look_ahead_dist"]
        for i in range(nearest_idx, len(self.global_path)):
            dist = np.hypot(self.global_path[i][0]-self.current_pose[0], 
                            self.global_path[i][1]-self.current_pose[1])
            if dist >= look_ahead:
                return np.array(self.global_path[i])
        return np.array(self.global_path[-1])

    def control_loop(self,new_path = False):
        if self.current_pose is None or self.goal is None: return

        dist_to_goal = np.linalg.norm(self.goal - self.current_pose[:2])
        if dist_to_goal < 0.2:
            self.publish_cmd(0.0, 0.0)
            return

        if new_path :self.trigger_global_plan()
        local_target = self.get_local_target() if self.global_path else self.goal
        
        v, w, best_traj = self.dwa.compute_control(
            self.current_pose, self.current_vel, local_target, self.goal, self.map_mgr
        )
        self.publish_cmd(v, w)
        if best_traj is not None:
            self.publish_path_viz(best_traj)

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def publish_path_viz(self, traj):
        """
        [重要修正]
        創建一個新的 Path 訊息，而不是使用成員變量。
        這保證了每次發佈的都是當前瞬間預測的完整軌跡，不會有殘影或錯誤的線條。
        """
        path_msg = self.path_msg
        # 這裡的 frame_id 必須與你的機器人座標系或地圖座標系一致
        # 如果你的軌跡點是基於 map 計算的，就用 "map"
        # 如果是基於里程計計算的，就用 "odom"
        # 由於 DWA 計算通常使用 map 座標 (因從 current_pose 衍生)，這裡建議用 "lidar" 或 "map" 取決於你的 tf 樹
        path_msg.header.frame_id = "lidar" 
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for pt in traj:
            pose = PoseStamped()
            pose.header = path_msg.header # 繼承 header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 # 保持默認方向
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

    def publish_global_path(self, points):
        path_msg = Path()
        path_msg.header.frame_id = "lidar" 
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for pt in points:
            pose = PoseStamped()
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.global_path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedDWAPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()