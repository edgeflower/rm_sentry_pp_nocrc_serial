# rm_sentry_pp_nocrc_serial 节点接口参考

> 节点名: `rm_sentry_pp_nocrc_serial`
> 功能: 哨兵机器人串口通信桥接节点，负责与下位机双向数据传输、云台路径跟随、漂移校正、Chiral 视觉目标分发。

---

## 1. ROS 订阅（外部 → 本节点）

### 1.1 `cmd_vel_chassis` (默认话题名，可配)
- **类型**: `geometry_msgs/msg/Twist`
- **回调**: `onCmd()`
- **用途**: 接收底盘速度指令
- **字段使用**:
  - `linear.x` → 底盘 vx
  - `linear.y` → 底盘 vy
  - `angular.z` 不使用，wz 由内部 `target_spin_vel_` 替代
- **参数**: `cmd_vel_chassis_topic` (默认 `"cmd_vel_chassis"`)

### 1.2 `robot_control` (默认话题名，可配)
- **类型**: `rm_decision_interfaces/msg/RobotControl`
- **回调**: `onRobotControl()`
- **用途**: 接收决策层的控制指令
- **字段使用**:
  - `gimbal_big_yaw_vel` → gimbal_big 手动角速度控制（路径超时回退时使用）
  - `chassis_spin_vel` → 底盘旋转速度 wz（替代 Twist.angular.z）
  - `follow_gimbal_big` → 底盘是否跟随 gimbal_big 朝向（转发给下位机）
  - `track_status` → 是否启动履带（转发给下位机）
- **参数**: `robot_control_topic` (默认 `"robot_control"`)

### 1.3 `plan` (默认话题名，可配)
- **类型**: `nav_msgs/msg/Path`
- **回调**: `onPath()`
- **用途**: 全局路径，gimbal_big 跟随路径朝向
- **字段使用**:
  - `poses[]` → 路径点序列，每个点含 `position.x/y` 和 `orientation`（提取 yaw）
- **超时**: `gimbal_path_timeout_ms` (默认 1000ms)，超时后停止更新角度
- **参数**: `gimbal_follow_path_topic` (默认 `"plan"`)

### 1.4 `lidar_odometry` (默认话题名，可配)
- **类型**: `nav_msgs/msg/Odometry`
- **回调**: `onOdom()`
- **用途**: 底盘里程计，用于路径跟随计算和 gimbal_big 漂移校正
- **字段使用**:
  - `pose.pose.position.x/y` → 底盘位置
  - `pose.pose.orientation` → 提取 yaw 角
  - `twist.twist.linear.x/y` → 底盘速度 vx/vy（用于速度自适应前瞻）
- **超时**: `odom_timeout_ms` (默认 500ms)
- **稳定检测**: 需要 50 帧后才标记稳定，漂移校正在稳定前不启动
- **参数**: `odom_topic` (默认 `"lidar_odometry"`)

### 1.5 `enemy_in_forbidden_area`
- **类型**: `rm_decision_interfaces/msg/EnemyForbiddenArea`
- **回调**: `updateEnemyForbiddenArea()`
- **用途**: 接收敌方装甲板禁射区信息，写入 Chiral 共享内存
- **字段使用**:
  - `armors_num` → 装甲板类型（转为 `ArmorName` 枚举）
  - `is_forbidden` → 是否禁射

### 1.6 `robot_area_status`
- **类型**: `rm_decision_interfaces/msg/RobotAreaStatus`
- **回调**: `onRobotAreaStatus()`
- **用途**: 机器人是否在特定区域，控制感知状态
- **字段使用**:
  - `is_in_area` → 是否在区域内
  - `area_name` → 区域名称
  - `matched_area_names` → 匹配到的区域列表
- **逻辑**: 若机器人在 `robot_area_name_` 指定区域且仅匹配该区域 → `perception_status_ = false`（无法感知），否则 `perception_status_ = true`
- **参数**: `robot_area_name` (默认 `"bumpy_area"`)

---

## 2. ROS 服务（外部 → 本节点）

### 2.1 `set_sentry_posture` (默认服务名，可配)
- **类型**: `rm_decision_interfaces/srv/SetSentryPosture`
- **回调**: `handleSetSentryPosture()`
- **用途**: 设置哨兵姿态
- **Request 字段**:
  - `posture` (uint8) → 目标姿态值: 1=进攻, 2=防御, 3=移动
  - `override_mode` (bool) → true 时不等待下位机确认直接返回
- **Response 字段**:
  - `accepted` (bool) → 是否成功
  - `message` (string) → 结果描述
- **确认机制**: 非 override 模式下阻塞等待下位机上报姿态与目标一致，超时 `posture_confirm_timeout_ms` (默认 500ms)
- **参数**: `set_posture_service_name` (默认 `"set_sentry_posture"`)

---

## 3. ROS 发布（本节点 → 外部）

### 3.1 `target_tracking`
- **类型**: `armor_interfaces/msg/Target`
- **触发**: Chiral 共享内存有新数据（100Hz 轮询）
- **用途**: 发布目标追踪信息
- **字段**:
  - `header.stamp` → 当前时间
  - `header.frame_id` → `"gimbal_yaw"`
  - `tracking` (bool) → 是否正在追踪
  - `tracking_status` (uint8) → 0=Idle, 1=Detecting, 2=Tracking, 3=TempLost
  - `confidence` (float) → 置信度（Tracking/TempLost=1.0, 其余=0.0）
  - `position.x/y/z` (float) → 目标在 gimbal_yaw 坐标系下的位置
  - `velocity.x/y/z` (float) → 目标速度
  - `yaw` (float) → 目标装甲板朝向
  - `v_yaw` (float) → 目标旋转角速度
  - `radius_1` / `radius_2` (float) → 装甲板半径
  - `dz` (float) → 装甲板高度差
  - `armors_num` (uint8) → 装甲板数量
  - `id` (string) → 目标ID: "hero"/"engineer"/"standard_3"/"standard_4"/"standard_5"/"sentry"/"outpost"/"base"/"unknown"

### 3.2 `sentry_posture_status`
- **类型**: `rm_decision_interfaces/msg/SentryPostureStatus`
- **触发**: 串口收到 `ID_ROBOT_INFO` 帧
- **字段**:
  - `current_posture` (uint8) → 下位机上报的当前姿态: 1=进攻, 2=防御, 3=移动

### 3.3 `robot_status`
- **类型**: `rm_decision_interfaces/msg/RobotStatus`
- **触发**: 串口收到 `ID_ROBOT_INFO` 帧
- **字段**:
  - `robot_id` (uint8) → 机器人 ID
  - `team_color` (uint8) → 队伍颜色: 0=红, 1=蓝, 2=未知
  - `is_attacked` (bool) → 是否被击打
  - `current_hp` (uint16) → 当前血量
  - `shot_allowance` (uint16) → 17mm 弹丸剩余量
  - `shooter_heat` (uint16) → 枪管热量限制

### 3.4 `game_status`
- **类型**: `rm_decision_interfaces/msg/GameStatus`
- **触发**: 串口收到 `ID_GAME_STATUS` 帧
- **字段**:
  - `game_progress` (uint8) → 当前比赛阶段
  - `stage_remain_time` (uint16) → 当前阶段剩余时间 (秒)

### 3.5 `self_robot_hp`
- **类型**: `rm_decision_interfaces/msg/SelfRobotHP`
- **触发**: 串口收到 `ID_ALL_ROBOT_HP` 帧
- **字段**:
  - `hero_hp` (uint16) → 英雄血量
  - `engineer_hp` (uint16) → 工程血量
  - `standard_3_hp` (uint16) → 3号步兵血量
  - `standard_4_hp` (uint16) → 4号步兵血量
  - `sentry_hp` (uint16) → 哨兵血量
  - `outpost_hp` (uint16) → 前哨站血量
  - `base_hp` (uint16) → 基地血量

### 3.6 `start_save_map`
- **类型**: `std_msgs/msg/Bool`
- **触发**: 串口收到 `ID_ROBOT_INFO` 帧（由下位机 `start_save_map` 字段控制）

### 3.7 `robot_location`
- **类型**: `rm_decision_interfaces/msg/FriendLocation`
- **触发**: 串口收到 `ID_ROBOT_LOCATION` 帧
- **字段**: 各友方机器人 x/y 坐标 (float)
  - `hero_x`, `hero_y`
  - `engineer_x`, `engineer_y`
  - `standard_3_x`, `standard_3_y`
  - `standard_4_x`, `standard_4_y`
  - `sentry_x`, `sentry_y`

### 3.8 `rfid`
- **类型**: `rm_decision_interfaces/msg/RFIDParse`
- **触发**: 串口收到 `ID_RFID` 帧
- **字段**: 38 个 bool 位，表示各 RFID 增益点触发状态:
  - 基础点: `base_self`, `highland_self`, `highland_enemy`, `slope_self`, `slope_enemy`
  - 飞坡: `fly_self_front`, `fly_self_back`, `fly_enemy_front`, `fly_enemy_back`
  - 中央高地地形跨越: `center_low_self`, `center_high_self`, `center_low_enemy`, `center_high_enemy`
  - 公路: `road_low_self`, `road_high_self`, `road_low_enemy`, `road_high_enemy`
  - 战略点: `fortress_self`, `outpost_self`, `resource_isolated`, `resource_overlap`, `supply_self`, `supply_enemy`, `center_bonus`
  - 敌方点: `fortress_enemy`, `outpost_enemy`
  - 隧道己方: `tunnel_self_1` ~ `tunnel_self_6`
  - 隧道敌方: `tunnel_enemy_1` ~ `tunnel_enemy_6`

### 3.9 `enemy_location`
- **类型**: `rm_decision_interfaces/msg/EnemyLocation`
- **触发**: 串口收到 `ID_ENEMY_LOCATION` 帧
- **字段**: 各敌方机器人 x/y 坐标 (float)
  - `hero_x`, `hero_y`
  - `engineer_x`, `engineer_y`
  - `standard_3_x`, `standard_3_y`
  - `standard_4_x`, `standard_4_y`
  - `sentry_x`, `sentry_y`

### 3.10 可视化 Marker（调试用）
- **`expected_gimbal_yaw`** (`visualization_msgs/msg/Marker`) → 红色箭头，gimbal_big 期望朝向
- **`lookahead_point_marker`** (`visualization_msgs/msg/Marker`) → 绿色球体，路径前瞻点
- **`enemy_marker`** (`visualization_msgs/msg/Marker`) → 黄色半透明球体，敌人位置；无目标时发送 DELETE

### 3.11 `imu`（当前已注释）
- **类型**: `sensor_msgs/msg/Imu`
- **触发**: 串口收到 `ID_IMU` 帧
- **状态**: publish 代码已注释，仅用于 TF 广播和内部漂移校正

---

## 4. TF 广播

| 父帧 | 子帧 | 触发 | 内容 |
|---|---|---|---|
| `gimbal_big` | `gimbal_yaw` | 串口收到 IMU 帧 | gimbal_yaw 相对于 gimbal_big 的 yaw 旋转角 |

---

## 5. 串口发送（本节点 → 下位机）

### 5.1 `ID_ROBOT_CMD` (0x11)
- **帧结构**: `[SoF=0x5A][data_len][id=0x11][time_stamp:u32][data][EoF=0xA5]`
- **发送频率**: 200Hz (send_period_ms=5)
- **data 字段**:
  - `speed_vector.vx` (float) → 底盘前后速度
  - `speed_vector.vy` (float) → 底盘左右速度
  - `speed_vector.wz` (float) → 底盘旋转速度（来自 `target_spin_vel_`）
  - `gimbal_big.yaw_angle` (float) → gimbal_big 目标角度 (rad)，有效时为路径跟随角度+漂移预测，超时时冻结在最后有效角度
  - `gimbal_big.yaw_vel` (float) → gimbal_big 角速度 (rad/s)，当前始终为 0（使用位置控制）

### 5.2 `ID_ROBOT_POSTURE` (0x12)
- **帧结构**: `[SoF=0x5A][data_len][id=0x12][time_stamp:u32][data][EoF=0xA5]`
- **发送频率**: 200Hz（与 ROBOT_CMD 同频）
- **data 字段**:
  - `posture` (uint8) → 机器人姿态: 1=进攻, 2=防御, 3=移动
  - `follow_gimbal_big` (bool) → 底盘是否跟随 gimbal_big 朝向
  - `track_status` (bool) → 是否启动履带
  - `perception_status` (bool) → 大云台是否跟随全向感知（进入飞坡区域时关闭）

---

## 6. 串口接收（下位机 → 本节点）

### 6.1 `ID_IMU` (0x10)
- **帧大小**: 20 字节
- **data 字段**:
  - `gimbal_yaw` (float) → 小云台机械角 (°)，相对于上电零位
  - `chassis_yaw` (float) → 底盘位姿角 (rad)，范围 -π ~ π
  - `yaw` (float) → gimbal_big 的 IMU 原始 yaw (rad)
- **处理**:
  - `chassis_yaw` 取负后归一化 → `gimbal_big_yaw_`（gimbal_big 机械角）
  - `gimbal_yaw` 归一化 → `gimbal_yaw_`（小云台机械角）
  - 发布 TF: gimbal_big → gimbal_yaw
  - 记录 `latest_imu_raw_yaw_` 用于漂移校正

### 6.2 `ID_ROBOT_INFO` (0x06)
- **帧大小**: 22 字节
- **data 字段**:
  - `id` (uint8) → 机器人 ID
  - `color` (uint8) → 队伍颜色: 0=红, 1=蓝
  - `attacked` (bool) → 是否被击打
  - `hp` (uint16) → 当前血量
  - `heat` (uint16) → 枪管热量
  - `heat_limit` (uint16) → 热量限制
  - `shot_allowance` (uint16) → 17mm 弹丸剩余量
  - `posture` (uint8) → 当前姿态: 1=进攻, 2=防御, 3=移动
  - `nav_status` (bool) → 初始化状态: false=未初始化, true=已初始化
  - `start_save_map` (bool) → 是否开始保存地图
- **特殊处理**:
  - `nav_status` 上升沿检测 → 记录 IMU 和 odom 参考值，启动漂移校正

### 6.3 `ID_GAME_STATUS` (0x07)
- **帧大小**: 11 字节
- **data 字段**:
  - `game_progress` (uint8) → 比赛阶段
  - `stage_remain_time` (uint16) → 剩余时间

### 6.4 `ID_ALL_ROBOT_HP` (0x08)
- **帧大小**: 22 字节
- **data 字段** (`self_robot_hp` 结构):
  - `hero_hp`, `engineer_hp`, `standard_3_hp`, `standard_4_hp`, `sentry_hp`, `outpost_hp`, `base_hp` (均为 uint16)

### 6.5 `ID_ROBOT_LOCATION` (0x09)
- **帧大小**: 48 字节
- **data 字段**: 各友方机器人 x/y 坐标 (float)

### 6.6 `ID_RFID` (0x13)
- **帧大小**: 13 字节
- **data 字段**:
  - `rfid_status` (uint32) → bit0~bit31
  - `rfid_status_2` (uint8) → bit32~bit39（扩展位）
- **解析**: 逐位拆分为 38 个 bool 字段发布

### 6.7 `ID_ENEMY_LOCATION` (0x14)
- **帧大小**: 48 字节
- **data 字段** (`enemy` 结构): 各敌方机器人 x/y 坐标 (float)

---

## 7. Chiral 共享内存

### 7.1 读取（Chiral → 本节点）
- **频率**: 100Hz 轮询
- **数据**: `talos::chiral::navigation::TalosData`
- **字段**:
  - `state.status` → `TrackerStatus` 枚举: Idle / Detecting / Tracking / TempLost
  - `state_kind` → `TargetStateKind` 枚举: Robot / Outpost / 其他
  - `state.robot.position.x/y/z` → 目标位置（gimbal_yaw 坐标系下）
  - `state.robot.velocity.x/y/z` → 目标速度
  - `state.robot.yaw` → 装甲板朝向
  - `state.robot.v_yaw` → 旋转角速度
  - `state.robot.radius0`, `radius1` → 装甲板半径
  - `state.robot.z1` → 装甲板高度差
  - `state.robot.armor_num` → 装甲板数量
  - `state.robot.name` → `ArmorName` 枚举: Sentry/One/Two/Three/Four/Five/Outpost/Base
  - `state.outpost.*` → 前哨站目标（字段结构类似 robot）

### 7.2 写入（本节点 → Chiral）
- `invincible[]` 数组 → 各装甲板类型是否禁射（来自 `enemy_in_forbidden_area` 话题）

---

## 8. 核心内部逻辑

### 8.1 gimbal_big 路径跟随
- **定时器**: 50Hz (20ms)
- **流程**:
  1. 检查路径是否超时（1000ms）
  2. 获取底盘在 map 坐标系下的位姿
  3. 计算速度自适应前瞻距离: `lookahead = base + k * speed`（默认 0.8 + 0.4*v）
  4. 找最近路径点（warm-start 从上次位置搜索）
  5. 沿路径弧长前进 lookahead 距离，插值得到目标点
  6. 提取目标点的 yaw 作为目标朝向
  7. 低通滤波（alpha=0.3）处理角度跳变
  8. 写入 `gimbal_big_yaw_angle_`

### 8.2 gimbal_big 漂移校正
- **定时器**: 20Hz (50ms)
- **前提**: IMU 归中参考已记录 + odom 已稳定（50帧）
- **方法**: `compensation = (IMU_当前 - odom_当前) - (IMU_归中时刻 - odom_归中时刻)`
- **滤波**: 低通滤波 (alpha=0.15)，同时估算漂移速率

### 8.3 target_tracking 发布逻辑
三条互斥路径:
1. **Tracking/TempLost + 未知目标类型** → tracking=false, confidence=0.0 → publish → return
2. **Idle/Detecting** → tracking=false, confidence=0.0, 清零位置速度, DELETE marker → publish → return
3. **Tracking/TempLost + Robot/Outpost** → tracking=true, confidence=1.0, 填充位置速度和装甲板信息, 可视化 marker → publish

### 8.4 置信度衰减（已定义未使用）
- `calculateDecayedConfidence(conf, dt) = max(0.0, conf * exp(-lambda * dt))`
- 参数: `confidence_decay_lambda=0.5`, `min_confidence_threshold=0.3`

---

## 9. 可配置参数汇总

| 参数名 | 类型 | 默认值 | 用途 |
|---|---|---|---|
| `port` | string | `"/dev/ttyACM0"` | 串口设备路径 |
| `baud` | int | `115200` | 波特率 |
| `imu_frame` | string | `"gimbal_big"` | IMU frame ID |
| `imu_parent_frame` | string | `"chassis"` | IMU 父帧 |
| `cmd_vel_chassis_topic` | string | `"cmd_vel_chassis"` | 底盘速度话题 |
| `robot_control_topic` | string | `"robot_control"` | 机器人控制话题 |
| `set_posture_service_name` | string | `"set_sentry_posture"` | 姿态服务名 |
| `imu_topic` | string | `"imu"` | IMU 话题 |
| `send_period_ms` | int | `5` | 串口发送周期 (ms) |
| `enable_dtr_rts` | bool | `true` | 是否启用 DTR/RTS |
| `gimbal_angle_timeout_ms` | int | `300` | gimbal_big 角度有效超时 (ms) |
| `gimbal_follow_path_topic` | string | `"plan"` | 路径跟随话题 |
| `gimbal_follow_lookahead` | double | `1.5` | 固定前瞻距离（未使用） |
| `gimbal_lookahead_base` | double | `0.8` | 速度自适应前瞻基础距离 (m) |
| `gimbal_lookahead_k` | double | `0.4` | 速度自适应前瞻系数 |
| `gimbal_yaw_smooth_alpha` | double | `0.3` | gimbal_big 角度低通滤波系数 |
| `gimbal_path_timeout_ms` | int | `1000` | 路径超时 (ms) |
| `robot_area_name` | string | `"bumpy_area"` | 禁区区域名 |
| `odom_topic` | string | `"lidar_odometry"` | 里程计话题 |
| `relocalization_mode` | bool | `false` | 是否启用重定位模式 |
| `odom_timeout_ms` | int | `500` | 里程计超时 (ms) |
| `posture_confirm_timeout_ms` | int | `500` | 姿态确认超时 (ms) |
| `confidence_decay_lambda` | double | `0.5` | 置信度衰减速率 |
| `min_confidence_threshold` | double | `0.3` | 最低置信度阈值 |

---

## 10. 线程模型

| 线程 | 频率/触发 | 职责 |
|---|---|---|
| `protect_thread_` | 2Hz (500ms) | 串口断线重连 |
| `rx_thread_` | 持续阻塞读取 | 串口数据接收和帧解析 |
| `tx_thread_` | 200Hz (5ms) | 串口数据发送（CMD + POSTURE） |
| `chiral_thread_` | 100Hz | Chiral 共享内存读取和目标发布 |
| `gimbal_path_timer_` | 50Hz (20ms) | gimbal_big 路径跟随角度计算 |
| `drift_timer_` | 20Hz (50ms) | gimbal_big 漂移校正 |
| `map_odom_timer_` | 10Hz (100ms) | map→odom TF 查询（仅重定位模式） |
