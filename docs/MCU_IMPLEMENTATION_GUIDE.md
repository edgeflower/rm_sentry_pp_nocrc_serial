# 下位机姿态控制实现指南

## 说明

`ReceiveRobotInfoData` 数据包中的 `posture` 字段（uint16_t）应由下位机控制，反映机器人实际姿态。

## 数据包结构

```c
struct ReceiveRobotInfoData
{
    HeaderFrame frame_header;  // id = 0x06
    uint32_t time_stamp;

    struct
    {
        uint8_t id;
        uint8_t color;
        bool attacked;
        uint16_t hp;
        uint16_t heat;
        uint16_t projectile_allowance_17mm;
        uint16_t posture;        // ⭐ 下位机实际姿态 1=进攻, 2=防御, 3=移动
    } data;

    uint8_t eof;
};
```

## 姿态值定义

```
0 = 无效/未初始化
1 = 进攻姿态
2 = 防御姿态
3 = 移动姿态
```

## 下位机实现示例

### 1. 全局变量

```c
// 当前实际姿态
static uint16_t g_current_posture = 0;
static uint16_t g_target_posture = 0;

// 姿态切换控制
static uint32_t g_posture_switch_start_time = 0;
static bool g_is_switching = false;
static const uint32_t POSTURE_SWITCH_TIMEOUT_MS = 3000;  // 3秒超时
```

### 2. 收到上位机命令 (ID_ROBOT_POSTURE = 0x12)

```c
void OnReceivePostureCommand(uint16_t posture)
{
    if (g_current_posture == posture) {
        // 已经是目标姿态，无需切换
        return;
    }

    // 设置目标姿态，开始切换
    g_target_posture = posture;
    g_is_switching = true;
    g_posture_switch_start_time = GetTick();

    // 执行姿态切换动作
    ExecutePostureSwitch(posture);

    printf("[Posture] Start switching: %d -> %d\r\n", g_current_posture, g_target_posture);
}
```

### 3. 执行姿态切换

```c
void ExecutePostureSwitch(uint16_t target_posture)
{
    switch (target_posture) {
        case 1:  // 进攻姿态
            Gimbal_SetAngle(ATTACK_YAW, ATTACK_PITCH);
            Chassis_SetSpeedLimit(2.0f);
            break;

        case 2:  // 防御姿态
            Gimbal_SetAngle(DEFENSE_YAW, DEFENSE_PITCH);
            Chassis_SetSpeedLimit(1.0f);
            break;

        case 3:  // 移动姿态
            Gimbal_SetAngle(MOVE_YAW, MOVE_PITCH);
            Chassis_SetSpeedLimit(3.0f);
            break;

        default:
            g_is_switching = false;
            printf("[Posture] Invalid target: %d\r\n", target_posture);
            break;
    }
}
```

### 4. 定期检查切换状态

```c
// 在主循环或定时器中调用（建议 10ms 周期）
void UpdatePostureStatus(void)
{
    if (!g_is_switching) {
        return;
    }

    // 检查超时
    if (GetTick() - g_posture_switch_start_time > POSTURE_SWITCH_TIMEOUT_MS) {
        printf("[Posture] Switch timeout!\r\n");
        g_is_switching = false;
        return;
    }

    // 检查是否到达目标姿态
    if (IsPostureReached(g_target_posture)) {
        g_current_posture = g_target_posture;
        g_is_switching = false;

        printf("[Posture] Switch complete: %d\r\n", g_current_posture);
    }
}

// 检查姿态是否到达目标位置
bool IsPostureReached(uint16_t target_posture)
{
    float current_yaw = Gimbal_GetYaw();
    float current_pitch = Gimbal_GetPitch();

    switch (target_posture) {
        case 1:  // 进攻姿态
            return (fabs(current_yaw - ATTACK_YAW) < 0.1f &&
                    fabs(current_pitch - ATTACK_PITCH) < 0.1f);

        case 2:  // 防御姿态
            return (fabs(current_yaw - DEFENSE_YAW) < 0.1f &&
                    fabs(current_pitch - DEFENSE_PITCH) < 0.1f);

        case 3:  // 移动姿态
            return (fabs(current_yaw - MOVE_YAW) < 0.1f &&
                    fabs(current_pitch - MOVE_PITCH) < 0.1f);

        default:
            return false;
    }
}
```

### 5. 填充并发送数据包

```c
void FillAndSendRobotInfo(void)
{
    ReceiveRobotInfoData pkt;

    pkt.frame_header.sof = 0x5A;
    pkt.frame_header.id = ID_ROBOT_INFO;
    pkt.frame_header.data_len = 12;  // sizeof(data)
    pkt.time_stamp = GetTick();

    pkt.data.id = GetRobotID();
    pkt.data.color = GetTeamColor();
    pkt.data.attacked = IsAttacked();
    pkt.data.hp = GetHP();
    pkt.data.heat = GetHeat();
    pkt.data.projectile_allowance_17mm = GetProjectileCount();

    // ⭐ 关键：填充下位机实际姿态
    pkt.data.posture = g_current_posture;

    pkt.eof = 0xA5;

    UART_Send((uint8_t*)&pkt, sizeof(pkt));
}
```

### 6. 主循环

```c
int main(void)
{
    System_Init();

    // 初始化默认姿态
    g_current_posture = 3;  // 默认移动姿态

    while (1) {
        // 处理串口接收（包含解析 ID_ROBOT_POSTURE 命令）
        UART_RxProcess();

        // 更新姿态状态
        UpdatePostureStatus();

        // 定期发送机器人信息（建议 20-50Hz）
        static uint32_t last_send = 0;
        if (GetTick() - last_send > 20) {
            FillAndSendRobotInfo();
            last_send = GetTick();
        }
    }
}
```

## 上位机侧

上位机行为树通过以下流程确认姿态切换：

1. `SetPosture` 节点发送 `ID_ROBOT_POSTURE` 命令
2. `SubRobotPosture` 节点订阅 `sentry_posture_status` 话题
3. 下位机更新 `posture` 字段后，话题会发布新值
4. `SetPosture` 读取 `current_posture`，对比目标值确认成功

## 调试建议

1. **姿态切换时打印日志**
```c
printf("[Posture] %d -> %d (is_switching=%d)\r\n",
       g_current_posture, g_target_posture, g_is_switching);
```

2. **超时时间**根据实际机械响应调整 `POSTURE_SWITCH_TIMEOUT_MS`

3. **角度容差**调整 `IsPostureReached()` 中的 `0.1f` 阈值

4. **通信周期**建议 20-50Hz 发送 `ReceiveRobotInfoData`

## 注意事项

1. **初始化**：上电时设置一个默认姿态（推荐 3=移动）

2. **状态机**：
   ```
   收到命令 → 执行切换 → 更新 g_current_posture → 发送给上位机
   ```

3. **切换期间收到新命令**：应取消当前切换，开始新的切换

4. **通信频率**：确保上位机能及时收到姿态更新
