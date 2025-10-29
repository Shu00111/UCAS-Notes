# ROS2 + MoveIt2 + Gazebo Basic Simulation

## 基础概念

### 基本定义

**ROS2 Robot Operating System 2**: 一种用于机器人软件开发的中间件/生态。它把机器人系统拆成“节点（node）”，节点之间通过话题（topic）、服务（service） 和 动作（action） 来通信。ROS2 是 ROS 的新版，改进了实时性、跨平台、多机器人等能力。

**节点（Node）**：运行的进程，例如一个负责读取传感器，一个负责控制电机。每个节点通常是一个可执行程序。

**话题（Topic）**：节点间发布/订阅的异步消息通道。比如 /joint_states 常发布关节角度。

**服务（Service）**：同步的请求-应答机制，类似 RPC。

**动作（Action）**：用于长时间运行、可预empt的任务（例如移动到目标姿态）；包含反馈和结果。

**TF / TF2**：坐标变换系统，用来管理机器人各部分（base、link、tool）的坐标系关系并随时间广播。

**URDF（Unified Robot Description Format）**：用 XML 描述机器人的连杆（link）和关节（joint）以及视觉/碰撞模型。MoveIt、Gazebo 等都消费 URDF。

**SRDF（Semantic Robot Description Format）**：MoveIt 使用，定义关节组（group）、默认姿态、虚拟关节等语义信息。

**Gazebo**：物理仿真引擎，能在虚拟环境中运行机器人并模拟动力学、碰撞、传感器等。

**MoveIt2**：运动规划框架，用于做**逆运动学、规划轨迹、碰撞检测、运动执行与仿真集成**（可与 Gazebo、真实控制器对接）。

**ros2_control / controller_manager**：一套标准化的接口，把控制器（如位置/速度控制器）管理起来，支持 Gazebo 插件或真实机器人硬件驱动。

**joint_state_publisher / robot_state_publisher**：joint_state_publisher 负责发布关节状态（joint positions），robot_state_publisher 根据 URDF 和关节状态发布 TF。

**Planning Scene（规划场景）**：MoveIt 管理的环境状态（包含物体、碰撞体等），影响路径规划的碰撞检测。

### 文件/包的结构与职责

**my_robot_description/:** 通常包含 URDF/XACRO、SRDF、视觉/碰撞网格。

**my_robot_gazebo/:** 包含 Gazebo world 文件、Gazebo 插件、spawn 脚本。

**my_robot_moveit_config/：** MoveIt 生成的配置（规划组、规划器参数、kinematics.yaml、ompl_planning.yaml）。

**my_robot_control/：** ros2_control 配置（硬件接口、控制器），controller spawners。

**launch/：** ROS2 的 launch 文件（Python），用于启动一组节点，例如：启动 Gazebo + spawn 机器人 + 启动 MoveIt planning execution + 启动控制器。

**src/：** 自定义节点源码（例如接收目标并调用 move_group action 发起规划）。

## 实践过程

参考[CSDN:ROS2+MoveIt2+Gazebo中的仿真（快速通关版](https://blog.csdn.net/qq_45253884/article/details/146335840)

### 创建工作空间

```bash
mkdir -p myRobot/src
cd myRobot/src
ros2 pkg create mybot_description --build-type ament_python
cd mybot_description
mkdir -p urdf
cd urdf
gedit six_arm.urdf
```

#### ROS2 工作空间（workspace）的概念

在 ROS2 中，所有的功能包（package）都必须位于某个工作空间（workspace）里，并通过 colcon build 统一编译。工作空间其实就是一个文件夹结构。

```bash
myRobot/                ← 你的工作空间 (workspace)
├── src/                ← 放源代码的地方 (source space)
├── install/            ← 编译后安装文件 (install space)
├── build/              ← 中间编译文件 (build space)
└── log/                ← 日志文件
```

#### 解释

```bash
ros2 pkg create mybot_description --build-type ament_python
```
创建一个新的 **ROS2 功能包**（package），名为 mybot_description。

- ros2 pkg create：这是 ROS2 创建包的命令。
- mybot_description：包名。通常机器人相关包会有如下命名习惯：
  - xxx_description → 放机器人模型（URDF、Xacro）
  - xxx_gazebo → 放仿真配置（world、launch）
  - xxx_moveit_config → MoveIt 相关配置
- --build-type ament_python：指定构建类型为 Python。ROS2 包可分为两种主要构建类型：
  - ament_cmake：C++代码包；
  - ament_python：Python节点包。
  这里选择 ament_python，说明之后包里的节点（launch 文件、工具脚本）将用 Python 实现。

执行完后，ROS2 自动生成的结构如下：
```bash
mybot_description/
├── package.xml        ← 包的元信息：名字、依赖、版本等
├── setup.py           ← Python 包的安装脚本
├── resource/          ← Python 包的资源目录
│   └── mybot_description
├── mybot_description/  ← Python 模块目录
│   └── __init__.py
└── test/              ← 单元测试目录
```

```bash
mkdir -p urdf
```
在包内创建一个文件夹 urdf/。

- urdf 文件夹用于放机器人的模型文件，即 .urdf 或 .xacro 文件。
- URDF（Unified Robot Description Format）是 ROS 里描述机器人结构（连杆、关节、坐标、碰撞体等）的核心格式。
- 其他系统（MoveIt、Gazebo）都会读取它来生成可视化模型和动力学模型。

```bash
gedit six_arm.urdf
```
- 你即将编写你的第一个 URDF 文件。
- 通常命名为机器人模型名，比如 “six_arm” 表示一个六关节机械臂。
- .urdf 文件**描述整个机械臂的结构**：基座、连杆、关节、尺寸、坐标关系。

###  URDF（Unified Robot Description Format）

six_arm.urdf 文件，完整描述了一个六自由度机械臂的物理结构。

```bash
<?xml version="1.0"?>
<robot name="six_arm">
 
    <!-- Base link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.1"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <material name="blue">
                <color rgba="0 0 1.0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.1"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
        </collision>
        <inertial>
            <mass value="10"/>
            <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
    </link>
 
    <!-- Link 1 -->
    <link name="link1">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <material name="green">
                <color rgba="0 0.8 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
        </collision>
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
    </link>
 
    <!-- Joint 1: rotation around X-axis -->
    <joint name="joint1" type="continuous">
        <parent link="base_link"/>
        <child link="link1"/>
        <axis xyz="0 0 1"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </joint>
 
    <!-- Link 2 -->
    <link name="link2">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <material name="red">
                <color rgba="0.8 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
        </collision>
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
    </link>
 
    <!-- Joint 2: rotation around Y-axis -->
    <joint name="joint2" type="continuous">
        <parent link="link1"/>
        <child link="link2"/>
        <axis xyz="1 0 0"/>
        <origin xyz="0 0 0.1"/>
    </joint>
 
    <!-- Link 3 -->
    <link name="link3">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <material name="yellow">
                <color rgba="0.8 0.8 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
        </collision>
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
    </link>
 
    <!-- Joint 3: rotation around x-axis -->
    <joint name="joint3" type="continuous">
        <parent link="link2"/>
        <child link="link3"/>
        <axis xyz="1 0 0"/>
        <origin xyz="0 0 0.1"/>
    </joint>
 
    <!-- Link 4 -->
    <link name="link4">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <material name="green">
                <color rgba="0 0.8 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
        </collision>
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
    </link>
 
    <!-- Joint 4: rotation around X-axis -->
    <joint name="joint4" type="continuous">
        <parent link="link3"/>
        <child link="link4"/>
        <axis xyz="0 1 0"/>
        <origin xyz="0 0 0.1"/>
    </joint>
 
    <!-- Link 5 -->
    <link name="link5">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <material name="purple">
                <color rgba="0.8 0 0.8 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
        </collision>
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
    </link>
 
    <!-- Joint 5: rotation around Y-axis -->
    <joint name="joint5" type="continuous">
        <parent link="link4"/>
        <child link="link5"/>
        <axis xyz="1 0 0"/>
        <origin xyz="0 0 0.1"/>
    </joint>
 
    <!-- Link 6 -->
    <link name="link6">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.2"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.1"/>
            <material name="pink">
                <color rgba="0.8 0.4 0.8 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.2"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.1"/>
        </collision>
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
    </link>
 
    <!-- Joint 6: rotation around Z-axis -->
    <joint name="joint6" type="continuous">
        <parent link="link5"/>
        <child link="link6"/>
        <axis xyz="0 0 1"/>
        <origin xyz="0 0 0.1"/>
    </joint>
 
 
    <!-- Used for fixing robot to Gazebo 'base_link' 将机械手的基座固定在世界坐标上-->
    <link name="world"/>
 
    <joint name="fixed" type="fixed">
        <parent link="world"/>
        <child link="base_link"/>
    </joint>
 
</robot>
```

#### URDF基本定义

URDF 是 ROS 中用于描述机器人的一种 XML 格式文件，全称 Unified Robot Description Format。

它用来告诉 ROS：我的机器人长什么样？每个部分叫什么？它们怎么连接？可以转动哪个方向？

简而言之：

- link（连杆） = 机器人的一个刚体（如机械臂的某一节）
- joint（关节） = 连接两个连杆的转动或滑动约束
- origin = 关节或部件的坐标位置
- axis = 关节的旋转或移动方向

#### 解释

1. 文件整体结构
   ```xml
    <?xml version="1.0"?>
    <robot name="six_arm">
       ...
    </robot>
   ```
   - <?xml version="1.0"?>：声明这是一个 XML 文件；
   - <robot name="six_arm">：定义机器人名称为 “six_arm”；
   - 所有 link、joint 都在这个 <robot> 标签里。

2. 机器人部件结构
  机械臂一般由：
  - 一个底座（base_link）
  - 六个连杆（link1 ~ link6）
  - 六个旋转关节（joint1 ~ joint6）
  - 一个固定关节（fixed）：把底座固定在世界坐标上。

  ```xml
    <!-- Base link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.1"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <material name="blue">
                <color rgba="0 0 1.0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.1"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
        </collision>
        <inertial>
            <mass value="10"/>
            <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
    </link>
  ```

  - visual：视觉显示用（在 RViz 或 Gazebo 中看到的形状与颜色）
  - collision：碰撞体积（Gazebo 物理引擎计算碰撞时使用）
  - inertial：惯性参数（质量和惯性矩，用于动力学计算）

  ```xml
    <geometry><box size="0.1 0.1 0.1"/></geometry>
  ```
  → 表示底座是一个 10cm (0.1m) 的立方体。

  ```xml
    <material name="blue"><color rgba="0 0 1.0 1"/></material>
  ```
  → RGBA = 红绿蓝透明度 = 蓝色。

  ```xml
    <collision>
        <geometry>
            <box size="0.1 0.1 0.1"/>
        </geometry>
        <origin rpy="0 0 0" xyz="0 0 0.05"/>
    </collision>
  ```
  | 标签                   | 含义                               |
  | -------------------- | -------------------------------- |
  | `<collision>`        | 定义“碰撞几何体”（用于物理计算，不用于视觉显示）        |
  | `<geometry>`         | 几何形状，这里是一个立方体（box）               |
  | `size="0.1 0.1 0.1"` | 立方体的长、宽、高，单位是 **米**（0.1m = 10cm） |
  | `<origin>`           | 定义这个几何体相对于 link 坐标系的偏移和旋转        |
  | `xyz="0 0 0.05"`     | 表示这个方块中心在 Z 轴方向上抬高 5cm           |
  | `rpy="0 0 0"`        | 表示没有旋转（roll、pitch、yaw 都为 0）      |

  ```xml
    <inertial>
        <mass value="10"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  ```
  | 标签                   | 含义                          |
  | -------------------- | --------------------------- |
  | `<mass value="10"/>` | 质量为 10 kg                   |
  | `<inertia>`          | 惯性张量（inertia tensor），描述转动惯量 |
  | `ixx, iyy, izz`      | 分别是绕 X/Y/Z 轴的转动惯量           |
  | `ixy, ixz, iyz`      | 是“耦合项”，描述不同轴之间的惯性交叉影响       |

  - 惯性矩阵是一个 3x3 对称矩阵，用来描述物体对旋转的“抵抗能力”。
  - 这里都写成了 0，意味着这是个简化模型（方便入门，不考虑真实物理转动效果）。
  - 实际中，惯性矩一般用工具（如 SolidWorks、URDF inertia calculator）计算得到。

  ```xml
    <!-- Link 1 -->
    <link name="link1">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
            <material name="green">
                <color rgba="0 0.8 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder length="0.1" radius="0.03"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.05"/>
        </collision>
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
        </inertial>
    </link>
  ```
  - 机械臂的第一节；
  - 形状是圆柱体；
  - 颜色为绿色；
  - 长度 10cm，半径 3cm；物理碰撞部分与visual相同
  - origin xyz="0 0 0.05" 表示圆柱体从底部偏移 5cm，这样几何中心就对齐了。
  - link 的质量（mass）是 0.2 千克（也就是 200 克）。

  ```xml
    <!-- Joint 1: rotation around X-axis -->
    <joint name="joint1" type="continuous">
        <parent link="base_link"/>
        <child link="link1"/>
        <axis xyz="0 0 1"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </joint>
  ```
  - type="continuous" 表示旋转关节（可无限转动）；
  - parent 和 child 表示这个关节连接谁和谁；
  - axis xyz="0 0 1" 表示围绕 Z 轴旋转；
  - origin xyz="0 0 0.1" 表示关节在父坐标系中的位置（即底座顶端）。

  **同理，link2-link6,joint2-joint6。六个关节分别让机械臂能绕不同轴旋转（Z、X、Y交替），形成一个灵活的六自由度结构。**

  ```xml
    <link name="world"/>
    <joint name="fixed" type="fixed">
        <parent link="world"/>
        <child link="base_link"/>
    </joint>
  ```
  固定到世界坐标,让 Gazebo/RViz 知道：“这个机械臂不是漂浮在空中，而是固定在地面（世界坐标）上。”

