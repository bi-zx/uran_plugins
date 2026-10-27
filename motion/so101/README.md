# SO101：不依赖 Placo 的 URAN 控制实现

本插件保留通用 `arm_cartesian_velocity`、`arm_pose`、`gripper_action(toggle)` 和 `safety_action` 合约；SO101 的标定、运动学、限速和碰撞模型均在插件内部。Python 3.6 可运行，无需 Placo、NumPy、SciPy 或 PyBullet。硬件串口使用 pyserial，配置使用 PyYAML，消息校验使用 URAN SDK/interfaces。

## 与 lerobot-so101-ik 对齐的行为

| 内容 | 当前实现 | 与参考实现的差异 |
|---|---|---|
| 标定 | 前五轴使用 `(raw - (range_min + range_max)/2) * 360 / 4095`；夹爪归一化 0–100%，遵守夹爪 drive_mode | 不重复叠加硬件已生效的 homing_offset；旧 COM9 文件不再自动覆盖用户标定 |
| 模型 | 从 `assets/so101_new_calib.urdf` 导出完整关节树，包含活动夹爪 | 运行时不加载渲染 STL |
| IK | 6×5 几何 Jacobian、SO(3) 姿态误差、加权 DLS、阻尼和线搜索 | 替代 Placo/QP；不保证解轨迹或权重数值效果完全相同 |
| 姿态 | 初始化为实测 FK 姿态；角速度绕基座 Z 轴左乘；四元数描述完整末端姿态 | 不再将目标 yaw/roll 直接写入第五关节；基座 Z 旋转不等于夹爪自身轴旋转 |
| 连续控制 | 独立固定周期线程，累积目标、低通速度滤波、状态回退 | CyberDog2 默认 20 Hz；参考默认 50 Hz。低通时间常数 0.07 s，约对应 50 Hz 下 alpha=0.25 |
| 限速 | 每帧总关节变化 ≤3°，同时受 60°/s 限制；夹爪 profile 为 150%/s | 不把 IK 内部迭代步长当作真实帧限速 |
| 关节范围 | 默认取标定范围与 URDF 交集；profile 可针对明确命名的关节选择标定范围 | 放宽是逐关节、显式配置的，不会全局忽略 URDF 限位 |
| 自碰撞 | STL 派生的胶囊粗检 + 定向包围盒分离轴复核；包含夹爪；检查实测至量化目标的插值路径 | 保守几何近似，非 PyBullet 网格接触；不沿用索引豁免和 8 mm 穿透容差 |
| 命令结果 | `accepted` 表示排队；位姿/夹爪依据实测到位发布 `succeeded`，截止时间未到位发布 `failed` | 不再将写串口成功当作到位 |

5 个运动关节无法满足任意六维位姿。IK 回报位置与姿态残差，位置容差 1.5 mm、姿态容差 5°只用于到位判定，并非可达性保证。姿态权重作用于平方误差，位置以米、姿态以弧度计。

## 控制与保护

- MQTT 字段和组件路由不变：`component_id=arm` 的 `move_cmd` 线速度是基座坐标系 m/s，`angular_vel_z` 是基座 Z 轴 rad/s；`target_roll/pitch/yaw` 为可选绝对末端 RPY 约束。只使用角速度时应把这三个字段设为 `null`，避免同时固定某一姿态角。`action=toggle` 控制夹爪。
- 速度指令在 `min(消息截止时间, 最近一次接收+0.2 s)` 到期。发送端应持续以至少 10 Hz 更新；停止包的三个线速度和角速度为零，并不携带绝对姿态约束。
- 目标位置最多领先反馈 2 cm，姿态最多领先 0.35 rad；超时、零速度、stop 均清除累积目标。无法通过碰撞检查时尝试 1、1/2、1/4、1/8、1/16 步长，全部失败则恢复目标、关节和滤波器状态。
- 默认硬件标定限位与 URDF 限位取交集。`calibrated_limit_joints` 可逐项选择使用标定范围替代对应关节的 URDF 角度限制；当前 CyberDog2 profile 仅为 `shoulder_lift` 启用，范围约为 −114.77°～114.77°。这不是取消安全限制：标定编码器边界、速度限制、起始姿态检查和自碰撞检查仍然生效。安全范围缩放不再改变坐标零点或夹爪百分比。原始目标超限会被拒绝，不再静默截断。
- 起始反馈在有效限位外时拒绝运动并报告关节名/角度。不会自动把折叠姿态跳到合法边界；需在扭矩关闭时恢复合法起始姿态。状态的 `joint_limits_deg` 和 `calibrated_limit_joints` 会记录实际生效的范围与覆盖项。
- 启动只读取，不开启扭矩。首个合法目标先写入实测保持位置，再开扭矩，再写目标。读取失败/舵机告警暂停运动；纯电压告警可自动恢复，其他故障保持锁定。扭矩已开启或部分开启时尝试逐个关闭全部舵机。**关闭扭矩后机械臂可能因重力下落**，这同样适用于 emergency_stop/停服务。
- emergency_stop 锁定，需要 Runtime deactivate/activate 或修复后重启服务。普通 stop 不解除锁定；自动电压恢复也不解除急停。
- `allow_motion: false` 是只读演算模式：不写目标、扭矩或 EEPROM；反馈仍来自实物，所以未实际移动的位姿任务不会被报告为到位。
- 去重覆盖 controller/sequence 和最近 2048 个 operation_id，状态分开报告实测与目标。进程重启不保留操作去重历史。

## 电压告警自动恢复

默认 `voltage_auto_recovery: true`，仅 Feetech 已校验状态包中的纯 `0x01` 电压错误参与自动恢复。电压与过热/过载等组合告警、串口错误、IK/标定错误保持故障锁定，不通过错误文本猜测错误类型。

告警时结束当前 operation（failed），清除控制器目标和滤波状态，停止接收运动指令。如果曾开启扭矩（包括部分舵机开启失败），尝试关闭全部舵机，并在恢复前读取六个 Torque_Enable 寄存器确认关闭；纯只读/未启用扭矩时不主动开启或写目标。扭矩关闭可能让机械臂因重力下落，恢复流程不会自动抬回原姿态。

每 `voltage_recovery_interval_s: 0.5` 秒重试一次读取；连续 `voltage_recovery_healthy_samples: 3` 次六轴全部无告警后，清除 `faulted/error`、恢复 connected，并将控制器重置到最新实测姿态。中间再次告警则连续成功次数归零，持续电压告警会继续按周期等待，不需要重启服务。恢复时不自动开扭矩或执行旧目标；后续新指令走正常的实测保持、碰撞检查和限速流程。发出时间早于恢复时刻的在途运动指令也会被拒绝。

`control.status.voltage_recovery` 提供 pending、healthy_samples、required_samples、retry_interval_s、attempts、recoveries、alarms、last_error。历史错误放在该诊断对象里；成功恢复后的主 `error` 清空。解除电压故障不等于所有目标可达，`motion_ready` 仍需满足起始限位和急停检查。

此项变更本地 69 项回归通过；CyberDog2 Python 3.6 下 51 项通过、1 项离线 STL 检查跳过。故障注入测试覆盖启动/运行/收指令时电压告警、反复波动、连续健康采样、部分扭矩开启失败、急停与严重故障保留、旧指令拒绝和只读模式。部署后被动观察到 connected=true、faulted=false、motion_ready=true、torque.enabled=false，观察期未出现新的真实电压告警，因此本次没有声称完成真实波动后的恢复验收或运动验收。备份和观察记录在 `/SDCARD/URAN/validation/so101-voltage-recovery-20261007/`。

## 碰撞模型与边界

`config/collision_capsules.json` 记录 17 个 STL 碰撞元素的包络、源文件 SHA256、顶点数及 URDF 哈希。离线生成脚本对每个 STL 使用凸胶囊和包围盒；全部三角形顶点被包含，因此三角面也被包含。包络会有误报，尤其在折叠/夹爪附近。

忽略同 link 和 URDF 直接相邻 link，其他 link 配对均检查。没有“夹爪高于基座就忽略基座”的动态豁免。已存在的包络重叠只允许保持或改善，基准由实测更新且不会放宽；受阻目标不会更新基准。重新激活同一个 worker 保留该基准，重建 worker 才重置。

路径采样间距 ≤0.5°（前五轴）/2%（夹爪），在每个候选帧检查实际编码器量化后的目标。这不是连续碰撞自由的数学证明，也不包含 CyberDog2 身体、安装支架、线缆、地面或外部障碍。当前 `motion_ready` 只表示连接、故障和起始关节限位检查通过，候选目标仍可能被碰撞检查拒绝。

重新生成（在参考 STL 所在开发机执行，参数为真实路径）：

```bash
python3 tools/generate_collision_model.py /path/to/lerobot-so101-ik/assets/so101_new_calib.urdf \
  config/collision_capsules.json config/so101_collision.urdf
```

不要手工改 URDF 后沿用旧碰撞 JSON；哈希不一致会阻止配置。参考模型来自 SO101 的 onshape-to-robot 导出，来源标识和哈希保存在 JSON；资产授权沿用原项目，本实现不附加新的第三方资产授权声明。

## 状态诊断

`device.<device_id>.component.<component_id>.` 下有：

- `position.raw`、`position.values`：实测编码器、五轴角度（度）及夹爪百分比。
- `pose.measured`：实测 FK 位置/四元数。
- `connection.connected`、`torque.enabled`、`calibration.complete`。
- `control.status`：`motion_ready`、`readiness_error`、`faulted`、`estop_latched`、`error`、`dry_run`、`target_raw`、控制帧耗时/超期次数，以及 IK 残差、碰撞配对、回退比例。
- `control.status.last_command_timing`：最近指令的 operation_id、原始 issued/valid_until、进入 invoke 的时间/消息年龄、锁等待及成功返回的处理耗时。过期错误也包含迟到时间、进入 invoke 时的年龄和锁等待，便于区分上游排队和插件处理延迟。

`torque.enabled` 表示本 worker 的已确认操作状态，不代替每个舵机 Torque_Enable 寄存器遥测。电源断开/串口不可达时不能保证扭矩关闭指令成功，具体错误在状态中保留。

## 指令排队过期修复（2026-10-07）

针对 `mqtt-1791359063341473792-318`，下行 `timestamp_ns=-1304280064` 按有符号低 32 位还原为 `1791359063184000000`。到达 MQTT 时消息年龄约 157.47 ms，剩余有效期约 842.53 ms；拒绝时间为 `1791359064297364474`，此时已超期 113.36 ms。该样本的时间戳转换正常，接收后到拒绝经历约 955.89 ms。

旧版在共享锁内完成 IK 和全部碰撞回退，折叠姿态计算会阻塞 invoke，并使串行分发的后续指令排队。新版在独立控制器快照中计算，短锁只保护指令状态与串口；stop、急停、撤销、停用、故障或目标变化会使旧帧失效，计算中的检查点尽快中止它，提交前再校验版本和有效期。相同 controller 的同内容速度指令可续期；续期不额外积分，过期后重发则从实测重新开始。若计算期间收到了不同的编码器反馈，该候选路径丢弃后重算。碰撞恢复基准和试算状态隔离，只有有效帧可以提交。

同时复用同一帧五次回退的实测起点碰撞距离，并优化等价的 15 轴包围盒计算。未删减碰撞配对、路径采样、原有消息有效期或 0.2 s 速度看门狗；慢计算超期（包括没有候选目标）会保持实测位置，不能发送过期目标。复杂姿态仍可能低于目标 20 Hz，不构成硬实时保证。

CyberDog2 隔离模拟复现：采用实际标定与折叠编码器位置、相同的全零速度加 `orientation_rad.roll=0`，以约 20 Hz 入队 30 条消息，每条模拟 157 ms 的到达前延迟。旧版 9/30 条拒绝，最大消息年龄 1240 ms；候选版 0/30 条拒绝，invoke 平均 27.45 ms、最大 61.48 ms，最大消息年龄 221.46 ms。模拟不打开串口、不写扭矩或目标；这些耗时是该轮测量，不是性能上界。

该消息在修复版的计算结果为 `self_collision`，配对 `upper_arm_link:0` / `base_link:0`。`target_roll: 0` 是明确的绝对姿态约束，即使所有速度为零也会求解；不控制姿态时应发送 `target_roll/target_pitch/target_yaw: null`。全零速度且无姿态约束表示停止，测试平移需要非零线速度并持续刷新。

备份、候选测试和复现记录目录：`/SDCARD/URAN/validation/so101-command-latency-20261007/`。此修复不更改标定、身份信息、MQTT 配置或关节安全范围。

最终验证：本地 SO101/MQTT/control_arbiter 共 85 项通过，CyberDog2 Python 3.6 下 SO101 68 项通过（包含已在设备上的参考 STL 包含性检查）。模块 SHA256 与测试版本一致，标定和 secrets 文件哈希未变。完整时间戳及有符号低 32 位时间戳各 3 条 MQTT stop 指令均返回 succeeded。

部署前旧服务及新服务启动阶段另观察到串口 `Write timeout`，属于非电压通信错误，按现有规则锁定。排除其他串口占用后，独占只读采样 100/100 成功，平均 4.51 ms、最大 8.60 ms，电压 5.1–5.2 V，六轴扭矩寄存器全为 0；具体超时诱因尚未确定。随后通过 Runtime deactivate/activate 重新激活 arm，连续 15 秒、16 份快照均无故障，最终 connected=true、faulted=false、motion_ready=true、torque=false。没有新增普通串口故障自动解锁，也没有执行实体机械臂运动验收。

## superseded、碰撞反馈与串口读取超时（2026-10-07）

`cancelled / reason=superseded` 表示该 operation 被更新的指令替换，连续速度流中属于正常生命周期；不等于整个机械臂停止。回包新增 `superseded_by` 指向替换它的 operation，`continuous_velocity=true` 表示同内容速度续期，控制帧不会因此作废。旧 operation 的回包携带它被替换前的真实控制状态，不再引用新指令的状态或遗留的 idle 标签。

发生碰撞阻挡时，每个 operation 最多发一次 `running` 通知，包含 `reason=self_collision`、`blocked=true` 和碰撞配对；它表示指令仍有效但该候选运动被阻挡，不能当成正在发生实体运动。后续替换或超时回包保留碰撞信息，空闲状态保留最后一次停止/失败诊断，直到新指令到达。故障锁定拒绝消息附带具体串口或舵机错误。

对实际折叠编码器位置 `[2057,787,3012,2771,1973,1328]` 进行真实反馈、禁止扭矩/位置写入的验证：以约 10 Hz 更新全零速度加绝对 `roll=0`，25 条全部接收，0 帧因续期而丢弃，19 条事件明确报告 `upper_arm_link:0` / `base_link:0` 碰撞包络阻挡，未产生目标写入。该结果解释了“有 superseded 回包但不动”；仅平移时必须将不用的 target_roll/pitch/yaw 设为 null，并实际下发非零线速度。

同时观察到独立的串口 `Write timeout` 锁定。设备 pyserial 3.4 在 `os.write` 已经接受完整数据后仍会检查 30 ms 写入时限；线程被调度延迟时，可能已有有效舵机回复却报错。实机只读注入 40 ms 的写后延迟，旧实现报 Write timeout，但缓冲区中存在 ID 1 的校验正确位置回复。新版仅对 READ 请求的 `SerialTimeoutException` 继续等待并校验回复，不重发请求：ID、长度、校验及舵机错误均照常检查；无有效回复仍故障锁定，电压/过热/过载告警不会被吞掉。扭矩和位置写入不使用该处理，也不会自动重试。

`control.status.serial.read_write_timeouts` 和 `read_replies_after_write_timeout` 记录读取路径的发送超时次数与随后收到完整有效回复的次数。已验证 5 次实机注入全部收到正确位置、计数为 5/5；这是复现的可行原因，不能证明历史每次超时均由调度引起。

本地 SO101/MQTT/control_arbiter 97 项通过；CyberDog2 Python 3.6 下 79 项通过、1 项因候选目录缺少参考 STL 跳过。备份、候选源码和硬件只读验证记录在 `/SDCARD/URAN/validation/so101-command-latency-20261007/superseded/`。本次没有执行实体运动验收。

部署后三十秒观察收到 31 份快照，全部无故障，最终 connected=true、faulted=false、motion_ready=true、torque=false；其间状态记录本次启动后的一次 ID 6 纯电压告警已按 3 次健康采样自动恢复。观察窗口内未再发生普通串口写超时，因此真实负载下的长期发生率还需继续观察，不能用注入实验替代长期验证。

## 验证

本地 2026-10-07：SO101 25 项测试 + MQTT/control_arbiter 17 项回归，共 42 项通过。覆盖 LeRobot 换算与边界、FK/Jacobian 数值校验、可达/不可达姿态、帧总限速、STL 顶点包含性、碰撞路径中间点、滤波和回退、收包频率独立性、超时/急停、去重、首帧扭矩顺序、只读模式、故障和线程生命周期。

CyberDog2 Python 3.6.9/aarch64：24 项插件测试通过，1 项 STL 顶点包含性离线测试跳过（该项已在本地通过）。100 帧（合法姿态、小速度、含量化反馈、IK、碰撞检查）平均 22.6 ms、P95 25.8 ms、最大 26.3 ms；串口读取六轴约 4–6 ms。20 Hz 是默认目标频率，复杂碰撞回退/奇异位姿可能超期，计入 `overruns`，不会追赶发送成串目标；该测量不是最坏情况实时保证。

本次部署只做模拟、只读硬件反馈与 Runtime 状态验证，**没有执行机械臂运动验收或 Placo/PyBullet 数值等价验收**。首次连续读取捕获 ID 6 的 0x01 电压告警；随后三轮读取六舵机电压为 5.1–5.2 V、扭矩寄存器均为 0。重启后短时通信恢复，最终连续检查再次捕获 ID 6 的 0x01 电压告警，故障锁定生效。同时肩关节实测约 −113.45°，超出 URDF −100° 下限。最终 `uran.service` 为 active，但 arm 的 `connected=false`、`faulted=true`、`motion_ready=false`、`torque.enabled=false`，没有发送运动目标。

上述是初次部署的验证记录。随后按用户要求，CyberDog2 的 `calibrated_limit_joints: [shoulder_lift]` 已启用，肩关节有效范围由约 ±100° 调整为标定的约 ±114.77°。该覆盖项作用于起始校验和 IK，不修改参考 URDF/碰撞几何或用户标定；其他关节仍取交集，电压故障、限速和自碰撞保护继续生效。

这次调整本地 50 项测试通过，CyberDog2 Python 3.6 下 32 项通过、1 项离线 STL 检查跳过。部署时只读实测肩关节约 −113.19°，通过新的起始角度校验；当前折叠姿态的碰撞包络仍有重叠，不能将 `motion_ready=true` 理解为所有方向均可运动。限位调整未发送运动目标。新备份和验证记录在 `/SDCARD/URAN/validation/so101-calibrated-limits-20261007/`。

只读诊断命令（应先停止串口所有者；脚本不发送目标、扭矩或标定写入）：

```bash
sudo systemctl stop uran.service
export PYTHONPATH=/SDCARD/URAN/current/source/uran_plugins/motion/so101/src:/SDCARD/URAN/python
python3 /SDCARD/URAN/current/source/uran_plugins/motion/so101/tools/verify_runtime.py \
  --calibration /SDCARD/URAN/current/source/uran_profiles/profiles/cyberdog2/arm/so101_follower_arm.json \
  --read-port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79017505-if00 \
  --calibrated-limit-joint shoulder_lift
sudo systemctl start uran.service
```

部署前备份在 `/SDCARD/URAN/validation/so101-20261007/before.tar.gz`，诊断/基准/快照也保存在该目录。回滚应停止 `uran.service`，将该归档解压到 `/SDCARD/URAN/current/source`，然后再启动服务；用户标定文件和身份密钥未被部署覆盖。
