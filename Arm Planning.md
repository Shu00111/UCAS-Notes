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
We will use this package to 


## Python Package

```bash
cd $ROS2_WS/src
# 使用 ROS2 提供的命令创建 Python 包
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
cd arm_planner
gedit joint_planner.py
```
In the text editor:
```python
#!/usr/bin/env python3
"""
Joint-space trajectory planner (simple).
- 订阅 /joint_states （sensor_msgs/JointState）用于读取当前关节状态
- 订阅 /arm_target (sensor_msgs/JointState) 用于接收目标关节角
- 生成 JointTrajectory 并发布到 topic (默认 /arm_controller/command)
- 参数化： trajectory_duration, publish_rate, controller_topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import threading
import time

class JointPlanner(Node):
    def __init__(self):
        super().__init__('joint_planner')
        # 参数
        self.declare_parameter('controller_topic', '/arm_controller/command')
        self.declare_parameter('trajectory_duration', 5.0)   # seconds total
        self.declare_parameter('interpolation_steps', 50)   # number of trajectory points
        self.declare_parameter('queue_size', 10)

        self.controller_topic = self.get_parameter('controller_topic').get_parameter_value().string_value
        self.trajectory_duration = self.get_parameter('trajectory_duration').get_parameter_value().double_value
        self.interpolation_steps = int(self.get_parameter('interpolation_steps').get_parameter_value().integer_value)
        qsize = int(self.get_parameter('queue_size').get_parameter_value().integer_value)

        # 数据
        self.current_joint_state = None
        self.joint_state_lock = threading.Lock()

        # 订阅 joint_states (当前关节)
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, qsize)

        # 接收目标关节（使用 JointState 类型，fields: name, position）
        self.create_subscription(JointState, '/arm_target', self.target_cb, qsize)

        # 发布 JointTrajectory 到控制器
        self.traj_pub = self.create_publisher(JointTrajectory, self.controller_topic, qsize)

        self.get_logger().info(f'JointPlanner initialized. controller_topic={self.controller_topic}')

    def joint_states_cb(self, msg: JointState):
        with self.joint_state_lock:
            self.current_joint_state = msg

    def target_cb(self, msg: JointState):
        """
        接收到目标关节角，生成 trajectory 并发布。
        msg.name 列表和 msg.position 列表应该与 current_joint_state.name 对齐或能够映射。
        """
        with self.joint_state_lock:
            if self.current_joint_state is None:
                self.get_logger().warn('No current joint_states received yet; cannot plan.')
                return
            current = self.current_joint_state

            # 找到共同的关节顺序：使用当前 joint_states 的 order为基础
            joint_names = current.name
            # Build current positions dict
            cur_pos_map = {n: p for n, p in zip(current.name, current.position)}
            # Build target positions mapped to same order
            try:
                target_pos = [msg.position[msg.name.index(n)] if n in msg.name else cur_pos_map[n] for n in joint_names]
            except ValueError:
                self.get_logger().error('Target message joint names mismatch in indexing.')
                return

        # Create trajectory (linear interpolation in joint space, per-joint)
        traj = JointTrajectory()
        traj.joint_names = joint_names

        # times and points
        total_t = float(self.trajectory_duration)
        steps = max(2, int(self.interpolation_steps))
        times = np.linspace(0.0, total_t, steps)

        start_positions = np.array([cur_pos_map[n] for n in joint_names], dtype=float)
        goal_positions = np.array(target_pos, dtype=float)

        for t in times:
            alpha = t / total_t if total_t > 0 else 1.0
            # linear interpolation
            pos = (1 - alpha) * start_positions + alpha * goal_positions
            point = JointTrajectoryPoint()
            point.positions = pos.tolist()
            # velocities/accelerations left empty (controller may compute)
            point.time_from_start = rclpy.duration.Duration(seconds=float(t)).to_msg()
            traj.points.append(point)

        # Publish trajectory
        self.traj_pub.publish(traj)
        self.get_logger().info(f'Published trajectory to {self.controller_topic}, duration={total_t}s, steps={steps}')

def main(args=None):
    rclpy.init(args=args)
    node = JointPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
```
