### A_Star src

## 1.顯目說明
### 1.1. 代码结构


### 1.1. 代码结构
``` file-tree
.
├── a_star
│   ├── a_star_node.py
│   └── __init__.py
├── launch
│   ├── a_star.launch.py
│   └── Octomap2.launch.py
├── package.xml
├── README.md
├── resource
│   └── a_star
├── rviz
│   └── dwa_config.rviz
├── setup.cfg
├── setup.py
├── src
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

6 directories, 13 files
```

### 1.2. 代码说明
- `a_star_node.py`：DWM+A*算法节点，用于计算路径。
- `a_star.launch.py`：启动文件，用于启动算法节点。
- `Octomap2.launch.py`：启动文件，用于启动Octomap节点。

### 1.3. 使用说明
1. 将代码克隆到本地。(`git clone`)
2. 在终端中进入代码目录。
3. 运行`colcon build`命令编译代码。
4. 运行`source install/setup.bash`命令激活环境。(zsh: `source install/setup.zsh`)
5. 运行`ros2 a_star Octomap2.launch.py`命令启动Octomap节点。
6. 运行`ros2 launch a_star a_star.launch.py`命令启动算法节点。

### 1.4. 注意事项
- 请确保在运行代码之前已经安装了ROS 2和相关的依赖库。(Octomap、nav2、rviz2)
- 请确保在运行代码之前已经启动了livox_ros_driver2节点。
- 请确保在运行代码之前已经启动了fastlio2节点。
- 请确保在运行代码之前已经启动了Octomap节点。
- 请确保在运行代码之前已经启动了ROS 2的通信节点。

### 2. 默認運行指令

``` bash
colcon build
source install/setup.zsh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fastlio2 lio_launch.py
ros2 launch a_star Octomap2.launch.py
ros2 launch a_star a_star.launch.py
```

### 3. 自定義參數文件
```
src/livox_ros_driver2/config/MID360_config.json
src/fastlio2/config/lio_config.yaml
src/a_star/a_star/a_star_node.py
```

### 4.
## 4.1 顯目原理終結
1. A*算法：用于计算全局路径。
2. DWM算法：用于優化局部路径。
3. Octomap：用于构建环境地图。
4. livox_ros_driver2：用于接收livox激光雷达数据。
5. fastlio2：用于接收livox激光雷达数据并生成点云地图。

## 4.2 顯目數學原理

1. 預測曲線源代碼 (1:1 提取)
`DWALocalPlanner` 類中的原始代碼：

```python
    def _predict_trajectory(self, state, v, w):
        dt, steps = self.cfg["dt"], int(self.cfg["predict_time"] / self.cfg["dt"])
        times = np.linspace(dt, self.cfg["predict_time"], steps)
        thetas = state[2] + w * times
        traj_x = state[0] + np.cumsum(np.cos(thetas) * v * dt)
        traj_y = state[1] + np.cumsum(np.sin(thetas) * v * dt)
        return np.stack([traj_x, traj_y], axis=1), thetas[-1]
```
---

2. 解析曲線繪製流程

這段代碼利用 **Numpy 的向量化操作 (Vectorization)** 來一次性計算整條軌跡，而不是使用 `for` 循環逐步迭代。

1.  **時間切片 (Time Slicing):**
    *   `times = np.linspace(dt, ...)`
    *   生成一個時間數組：`[0.1, 0.2, 0.3, ..., 1.0]` (假設 dt=0.1, predict_time=1.0)。
    *   **注意**：時間是從 `dt` 開始的，不包含 0。

2.  **航向角推演 (Heading Prediction):**
    *   `thetas = state[2] + w * times`
    *   基於當前角度 `state[2]` ($\theta_0$) 和角速度 `w` ($\omega$)，計算未來每個時間點的角度。
    *   生成數組：$[\theta_1, \theta_2, \theta_3, ...]$，其中 $\theta_t = \theta_0 + \omega \cdot t$。

3.  **位移分量計算 (Displacement Components):**
    *   `np.cos(thetas) * v * dt`
    *   計算每一個微小時間段 `dt` 內，機器人在 X 軸和 Y 軸上的**增量 (Delta)**。
    *   $dx_t = v \cdot \cos(\theta_t) \cdot dt$

4.  **路徑積分 (Trajectory Integration via Cumsum):**
    *   `np.cumsum(...)` (累積求和)
    *   將所有的增量疊加起來，加上初始位置 `state[0]`。
    *   $X_1 = X_0 + dx_1$
    *   $X_2 = X_0 + dx_1 + dx_2$
    *   最終形成一系列的世界坐標點。
