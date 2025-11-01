# Arm Planning-ros2+rviz+gazebo

## Create the workspace

```bash
# 设定工作目录（你可以改名字）
export ROS2_WS=~/ros2_ws

# 创建工作空间和 src
mkdir -p $ROS2_WS/src
cd $ROS2_WS
```
## robot_description Package
We will use this package to save the URDF and the launch.
```bash
ros2 pkg create --build-type ament_python robot_description
```

### URDF

In this package, firstly, we design a 6-DOF robotic arm.
```bash
# 进入 robot_description 包目录
cd $ROS2_WS/src/robot_description

# 创建 urdf 目录
mkdir -p urdf

# 使用编辑器创建文件
gedit urdf/simple_arm.urdf
```

Copy the code below *simple_arm.urdf* and save the file.
```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- base link -->
  <link name="base_link" />
  <!-- chain of 6 revolute joints and links -->
  <!-- link1 -->
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2.0"/>
  </joint>

  <link name="link2"/>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2.0"/>
  </joint>

  <link name="link3"/>
  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2.0"/>
  </joint>

  <link name="link4"/>
  <joint name="joint4" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2.0"/>
  </joint>

  <link name="link5"/>
  <joint name="joint5" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin xyz="0 0 0.12" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2.0"/>
  </joint>

  <link name="link6"/>
  <joint name="joint6" type="revolute">
    <parent link="link5"/>
    <child link="link6"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2.0"/>
  </joint>

  <!-- simple visual geometry for each link (small box) -->
  <material name="gray">
    <color rgba="0.6 0.6 0.6 1.0"/>
  </material>

  <link name="link_visual_1">
    <visual>
      <geometry><box size="0.05 0.05 0.2"/></geometry>
      <material name="gray"/>
    </visual>
  </link>

  <!-- (We keep visuals simple: robot_state_publisher will still publish frames) -->
</robot>
```

Particularly, this URDF is simplified; the main aim is to generate the names of joints and links and assume the *robot_state_publisher* can show the framework and coordinate in the RViz.

### launch

```bash
cd ~/ros2_ws/src/robot_description
mkdir -p launch
gedit launch/display.launch.py
```

Copy the content below and save the file.
```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 获取包路径（这里直接用相对路径，因为包就在 workspace）
    pkg_share = os.path.join(os.path.dirname(__file__), '..')

    urdf_path = os.path.join(pkg_share, 'urdf', 'simple_arm.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # robot_state_publisher 会根据 robot_description 参数发布 TF（link frames）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
    ])
```

Particularly, the *robot_state_publisher* node gets the parameter of *robot_description*, which is the URDF string, and publishes the frame of every link. RViz can show every coordinate and RobotModel by TF.

## arm_planner

```bash
cd $ROS2_WS/src
ros2 pkg create --build-type ament_python arm_planner
```
Now, our package tree is as follows:
```bash
arm_planner
├── arm_planner
│   └── __init__.py
├── package.xml
├── resource
│   └── arm_planner
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

We need to write some nodes to realize more specific functions.

### joint_planner.py
```bash
cd ~/ros2_ws/src/arm_planner/arm_planner
gedit joint_planner.py
```
In the text editor:
```python
#!/usr/bin/env python3
"""
Joint-space planner (详细解释版)
功能：
- 订阅 /joint_states (当前关节位置)
- 订阅 /arm_target (目标关节位置，使用 sensor_msgs/JointState)
- 生成线性插值的 JointTrajectory 并发布到 controller_topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import threading

# Define the node class, which inherits from the Node in ROS2 
class JointPlanner(Node):
    def __init__(self):
        super().__init__('joint_planner')
        # 1) 参数：控制器 topic、轨迹时长、插值步数
        # declare_parameter: declare the parameter
        # 'controller_topic'（默认 '/arm_controller/command'）：planner 发布 JointTrajectory 的目标话题名；控制器/仿真应订阅该话题。
        # 'trajectory_duration'（默认 4.0 秒）：整条轨迹从起点到终点的总时长。
        # 'interpolation_steps'（默认 40）：插值点数量（轨迹的采样点数）

        self.declare_parameter('controller_topic', '/arm_controller/command')
        self.declare_parameter('trajectory_duration', 4.0)
        self.declare_parameter('interpolation_steps', 40)

        # get_parameter(name)：读取已声明的参数，返回一个 Parameter 对象。
        # .get_parameter_value()：从 Parameter 对象获取 ParameterValue。
        # .string_value / .double_value / .integer_value：访问具体的内嵌值字段（ROS2 的 ParameterValue 支持多种类型）。

        self.controller_topic = self.get_parameter('controller_topic').get_parameter_value().string_value
        self.trajectory_duration = float(self.get_parameter('trajectory_duration').get_parameter_value().double_value)
        self.interpolation_steps = int(self.get_parameter('interpolation_steps').get_parameter_value().integer_value)

        # 2) 存储当前 joint_states（初始为空）
        self.current_joint_state = None
        # threading.Lock()：创建互斥锁（mutex），用来保护 current_joint_state 的并发访问（订阅回调可能同时被不同线程调用）。在多线程环境下读写共享内存时必须加锁以避免数据竞争或半更新读取。
        self.joint_state_lock = threading.Lock()

        # 3) 订阅 joint_states（来自 fake_controller 或真实控制器）
        # create_subscription(msg_type, topic, callback, qos)：在节点上创建订阅器（subscriber）,该订阅器用于接收实时的当前关节状态。
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)

        # 4) 订阅 arm_target（外部发来的目标关节）
        # 建立对 /arm_target 的订阅。外部（比如 send_goal_example）会把目标关节用 JointState 消息发布到这个话题，planner 收到后进行规划。
        self.create_subscription(JointState, '/arm_target', self.target_cb, 10)

        # 5) 发布 JointTrajectory 到控制器
        # create_publisher(msg_type, topic, qos)：创建发布者（publisher）。
        self.traj_pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)

        self.get_logger().info(f'JointPlanner ready, controller_topic={self.controller_topic}')

    # 回调函数：joint_states_cb
    def joint_states_cb(self, msg: JointState):
        # 很重要：收到当前关节状态后保存（线程安全）
        with self.joint_state_lock:
            self.current_joint_state = msg

    # 回调函数：target_cb
    def target_cb(self, msg: JointState):
        # 收到目标，马上规划并发布
        with self.joint_state_lock:
            if self.current_joint_state is None:
                self.get_logger().warn('No current joint_states yet; ignoring target')
                return

            # 我们以 current_joint_state 的 joint order 为准
            current = self.current_joint_state

            joint_names = current.name
            # 读取当前状态中关节的名字列表（name 字段是一个字符串列表，顺序非常重要）。使用它作为轨迹 joint_names 的标准顺序。

            # 构造 name->pos 的字典，便于重排序
            cur_pos_map = {n: p for n, p in zip(current.name, current.position)}

            # 将目标的 position 映射到 joint_names 顺序（若缺则使用当前值）
            try:
                target_pos = [msg.position[msg.name.index(n)] if n in msg.name else cur_pos_map[n] for n in joint_names]
            except ValueError:
                self.get_logger().error('Target joint names mismatch; ignoring')
                return

        # 外面拿到 current & target 后，我们生成线性插值轨迹
        total_t = max(0.001, float(self.trajectory_duration))
        steps = max(2, int(self.interpolation_steps))
        times = np.linspace(0.0, total_t, steps)

        start_positions = np.array([cur_pos_map[n] for n in joint_names], dtype=float)
        goal_positions = np.array(target_pos, dtype=float)

        # 创建 JointTrajectory 消息对象（空），并设置 joint_names（控制器需要知道每个 positions 列表的关节对应顺序）。
        traj = JointTrajectory()
        traj.joint_names = joint_names

        for t in times:
            alpha = t / total_t
            pos = (1 - alpha) * start_positions + alpha * goal_positions
            point = JointTrajectoryPoint()
            point.positions = pos.tolist()
            # time_from_start 要是 duration（rclpy Duration 对象）
            from rclpy.duration import Duration
            # 把从起点到当前点的时间转为 ROS 时间消息（builtin_interfaces/Duration），并赋给 time_from_start 字段。time_from_start 告诉控制器这个点应在执行开始后多少秒到达。
            point.time_from_start = Duration(seconds=float(t)).to_msg()
            traj.points.append(point)

        # 发布 trajectory
        self.traj_pub.publish(traj)
        self.get_logger().info(f'Published trajectory: duration={total_t}s, steps={steps}')

def main(args=None):
    # 初始化 rclpy（必须调用一次，通常在程序入口执行），它会初始化底层 ROS 客户端库、参数解析、信号处理等。
    rclpy.init(args=args)
    node = JointPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

```

**More detailed explanation:**

1. rclpy
   - ROS2 的 Python API 顶层包。主要函数：rclpy.init()、rclpy.shutdown()、rclpy.spin(node)。
   - rclpy.spin(node) 会在内部运行一个 executor 来处理回调。
2. Node（类）: ROS2 中的核心对象，表示一个运行实体节点。常用方法：
   - declare_parameter(name, default)：声明参数。
   - get_parameter(name)：获取参数对象。
   - create_subscription(msg_type, topic, cb, qos)：创建订阅者。
   - create_publisher(msg_type, topic, qos)：创建发布者。
   - get_logger()：获取 logger（其上有 .info(), .warn(), .error() 等方法）。
   - create_timer(period, callback)：创建周期性回调（未在此脚本使用）。

3. 消息类型
   - sensor_msgs.msg.JointState：字段包括 header, name[], position[], velocity[], effort[]。用于描述关节的状态或也可被用作目标（此脚本把目标也用 JointState 发送）。
   - trajectory_msgs.msg.JointTrajectory：字段 joint_names[]（关节顺序）和 points[]（JointTrajectoryPoint 列表）。
   - trajectory_msgs.msg.JointTrajectoryPoint：字段 positions[], velocities[], accelerations[], time_from_start（builtin_interfaces/Duration）等。
4. QoS（简写的 10）
   - 本脚本中传 10 是简化写法，表示队列深度；更严谨做法是创建 QoSProfile。在可靠性要求或仿真中请根据需要定制 QoS（比如 reliable／best_effort、durability 等）。
5. 线程与锁
   - rclpy 的回调可能并发执行（具体取决于 executor），因此保护共享数据（如 self.current_joint_state）建议用 threading.Lock()。with self.joint_state_lock: 是常见模式。
6. 时间类型
   - ROS2 用 builtin_interfaces/Duration 描述持续时间。rclpy 中通过 Duration(seconds=...).to_msg() 获取消息格式。注意：time_from_start 应是相对时间（从轨迹开始算起）。
7. 常见问题
   - joint name mismatch：发送的目标关节名字必须和 /joint_states 的 name 列表对齐或能映射，否则 planner 会出错或忽略目标。
   - 缺少 /joint_states：planner 需要当前关节状态才能规划（否则会忽略目标）。可在 fake_controller 启动时发布初始零位以避免这一点。
   - 控制器接受消息格式：真实控制器可能期望 positions + velocities 或更严格时间序列；务必检查控制器文档或仿真订阅的话题名与消息格式。

**Some personal ideas**

This code designs a simple **linear interpolation** motion planner. Similarly, by changing the content of the code, we can design a more complex motion planner.

```python
for t in times:
    alpha = t / total_t
    pos = (1 - alpha) * start_positions + alpha * goal_positions
```
The code designs a linear interpolation, which means the joint moves at a constant speed over time without any changes in acceleration or smooth transitions.

$$
p(t) = (1-\alpha)p_0+\alpha p_1
$$

Example for changing:

1. *Polynomial Interpolation*

$$
p(t) = \alpha_0 + \alpha_1 t+\alpha_2 t^2+\alpha_3 t^3
$$

with the bound conditions, $p(0) = p_0$, $p(T) = p_T$, $\dot{p}(0) = 0$, $\dot{p}(T) = 0$, the $\alpha_i$ can be solved.

```python
alpha = t / total_t
pos = start_positions + (3*alpha**2 - 2*alpha**3) * (goal_positions - start_positions)
```

2. *Key Points Interpolation*

With a math model, solve a set of key points that influence the total trajectory. And use these key points to complete interpolation.

```python
from scipy.interpolate import CubicSpline
cs = CubicSpline(times_key, joint_positions_key, axis=0)
pos = cs(t)
```

3. *RL Key Points Interpolation*

Introduced the RL / MPC (Model Predictive Control) to predict the future key points and use the interpolation to get a complete trajectory.

```python
target_keypoints = rl_model.predict_future_states(current_state)
traj = interpolate_spline(target_keypoints)
```

4. *Basic Framework*

```python
def generate_trajectory(self, start, goal, times):
    # 👉 在这里你可以换成任意模型
    positions = []
    for t in times:
        alpha = t / times[-1]
        pos = self.cubic_interpolation(start, goal, alpha)
        positions.append(pos)
    return positions

def cubic_interpolation(self, start, goal, alpha):
    return start + (3*alpha**2 - 2*alpha**3) * (goal - start)

```

### fake_controller.py

当你没有实际控制器或仿真时，这个节点能接收 JointTrajectory 并按点更新一个“虚拟 joint_states”用于测试

```bash
cd ~/ros2_ws/src/arm_planner/arm_planner
gedit fake_controller.py
```

```python
#!/usr/bin/env python3
"""
Fake controller: 接收 JointTrajectory，简单地把最终点设为当前 joint_states 并周期性发布
主要用于测试 pipeline（没有动力学仿真，仅做逻辑验证）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import threading

class FakeController(Node):
    def __init__(self):
        super().__init__('fake_controller')
        self.declare_parameter('trajectory_topic', '/arm_controller/command')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.trajectory_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value

        self.joint_names = []
        self.positions = []
        self.lock = threading.Lock()

        self.create_subscription(JointTrajectory, self.trajectory_topic, self.traj_cb, 10)
        self.pub = self.create_publisher(JointState, self.joint_state_topic, 10)

        # 每 0.05s 发布一次 joint_states
        self.create_timer(0.05, self.publish_state)
        self.get_logger().info('FakeController ready')

    def traj_cb(self, traj: JointTrajectory):
        with self.lock:
            if not traj.joint_names:
                return
            self.joint_names = traj.joint_names
            # 直接使用最后一个 point 的 positions 作为最终位置（简化）
            if traj.points:
                self.positions = list(traj.points[-1].positions)
            else:
                self.positions = [0.0] * len(self.joint_names)
            self.get_logger().info(f'Received trajectory for joints: {self.joint_names}')

    def publish_state(self):
        with self.lock:
            if not self.joint_names:
                return
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = self.positions
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
```

### send_goal_example.py

发送目标，用于测试。

```bash
cd ~/ros2_ws/src/arm_planner/arm_planner
gedit send_goal_example.py
```

```python
#!/usr/bin/env python3
"""
发送一个目标 joint state 到 /arm_target，示例用
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class SendGoal(Node):
    def __init__(self):
        super().__init__('send_goal_example')
        self.pub = self.create_publisher(JointState, '/arm_target', 10)
        self.get_logger().info('SendGoal ready')

    def send_goal(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.pub.publish(msg)
        self.get_logger().info(f'Sent goal: {list(zip(names, positions))}')

def main(args=None):
    rclpy.init(args=args)
    node = SendGoal()
    time.sleep(1.0)  # 等待 publisher 建立
    # 这里请把 joint 名称改成和 URDF 中一致（joint1..joint6）
    joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
    target_positions = [0.1, 0.2, -0.2, 0.0, 0.5, -0.1]
    node.send_goal(joint_names, target_positions)
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()
```

## Prepare for running

```bash

```



