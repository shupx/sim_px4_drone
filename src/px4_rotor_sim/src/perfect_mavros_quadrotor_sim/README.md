# Perfect MAVROS Drone Simulator

完美理想的MAVROS无人机模拟器 - 直接将期望值映射到实际状态

## 功能特性

这个节点实现了一个理想的无人机模拟器，具有以下特性：

1. **订阅话题**：
   - `mavros/setpoint_raw/local` (mavros_msgs/PositionTarget)

2. **发布话题**：
   - `mavros/local_position/pose` (geometry_msgs/PoseStamped) - 位置和姿态
   - `mavros/local_position/velocity_local` (geometry_msgs/TwistStamped) - 速度
   - `mavros/local_position/odom` (nav_msgs/Odometry) - 里程计
   - `mavros/state` (mavros_msgs/State) - 状态（armed=true, mode=OFFBOARD）

3. **完美响应**：
   - 无延迟：接收到setpoint后立即发布到状态话题
   - 无误差：期望值直接作为实际值发布
   - 无动力学：不模拟任何物理动力学，适合测试规划算法

## 使用方法

### 单架无人机

```bash
# 不带命名空间
roslaunch px4_rotor_sim perfect_mavros_drone_no_namespace.launch
# 带命名空间
roslaunch px4_rotor_sim perfect_mavros_drone.launch namespace:=uav1
```

### 多架无人机

```bash
# 启动3架无人机（默认）
roslaunch px4_rotor_sim perfect_mavros_drone_multi.launch

# 启动5架无人机
roslaunch px4_rotor_sim perfect_mavros_drone_multi.launch num_drones:=5
```

## 示例：发送控制指令

使用rostopic发送位置指令：

```bash
# 发送到位置(1, 2, 3)，偏航角0度
rostopic pub /uav1/mavros/setpoint_raw/local mavros_msgs/PositionTarget "
header:
  stamp: now
  frame_id: 'map'
coordinate_frame: 1
type_mask: 0b000000000  # 使用所有字段
position: {x: 1.0, y: 2.0, z: 3.0}
velocity: {x: 0.0, y: 0.0, z: 0.0}
acceleration_or_force: {x: 0.0, y: 0.0, z: 0.0}
yaw: 0.0
yaw_rate: 0.0"
```

使用Python脚本：

```python
#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import PositionTarget

rospy.init_node('test_perfect_drone')

pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', 
                      PositionTarget, queue_size=10)

rate = rospy.Rate(10)  # 10 Hz

while not rospy.is_shutdown():
    msg = PositionTarget()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    msg.type_mask = 0  # 使用所有字段
    
    msg.position.x = 1.0
    msg.position.y = 2.0
    msg.position.z = 3.0
    msg.yaw = 0.0
    
    pub.publish(msg)
    rate.sleep()
```

## 查看输出

```bash
# 查看位置
rostopic echo /uav1/mavros/local_position/pose

# 查看速度
rostopic echo /uav1/mavros/local_position/velocity_local

# 查看状态
rostopic echo /uav1/mavros/state

# 查看里程计
rostopic echo /uav1/mavros/local_position/odom
```

## 应用场景

此模拟器适用于以下场景：

1. **快速测试控制算法**：无需考虑动力学延迟和误差
2. **验证多机协同逻辑**：专注于高层规划而非底层控制
3. **轨迹规划验证**：确认规划的轨迹是否符合预期
4. **通信协议测试**：测试MAVROS消息接口
5. **教学演示**：展示理想情况下的无人机行为

## 注意事项

- 此模拟器不考虑任何物理约束（加速度、速度限制等）
- 不适合用于测试需要真实动力学响应的场景
- 状态始终为armed=true和mode=OFFBOARD
- 所有setpoint字段都会被立即应用（除非type_mask设置为忽略）

## PositionTarget type_mask说明

type_mask用于指定忽略哪些字段：

- `IGNORE_PX = 1` (0b0000001) - 忽略位置X
- `IGNORE_PY = 2` (0b0000010) - 忽略位置Y  
- `IGNORE_PZ = 4` (0b0000100) - 忽略位置Z
- `IGNORE_VX = 8` (0b0001000) - 忽略速度X
- `IGNORE_VY = 16` (0b0010000) - 忽略速度Y
- `IGNORE_VZ = 32` (0b0100000) - 忽略速度Z
- `IGNORE_AFX = 64` (0b1000000) - 忽略加速度/力X
- `IGNORE_AFY = 128` (0b10000000) - 忽略加速度/力Y
- `IGNORE_AFZ = 256` (0b100000000) - 忽略加速度/力Z
- `FORCE = 512` (0b1000000000) - 使用力代替加速度,一般应该设为0从而使用加速度
- `IGNORE_YAW = 1024` (0b10000000000) - 忽略偏航角
- `IGNORE_YAW_RATE = 2048` (0b100000000000) - 忽略偏航角速率

例如：
- `type_mask = 0`：使用所有字段
- `type_mask = 7` (0b111)：只使用速度，忽略位置
- `type_mask = 4088` (0b110111111000)：只使用位置，忽略其他

PX4支持的掩码请参考https://blog.csdn.net/benchuspx/article/details/115750466

## 联系与支持

如有问题或建议，请提交Issue。
