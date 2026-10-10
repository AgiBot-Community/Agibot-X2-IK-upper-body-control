# V2.1 桌面 MDI（Linux AppImage）

桌面客户端运行在 Linux x86_64 电脑，通过 SSH 标准输入/输出调用机器人模块。
不新增网页或 HTTP 服务，桌面机无需安装 Python、ROS 或 AimDK。
AppImage 的构建环境见 `desktop/APPIMAGE_BUILD_INFO.json`；并非所有旧版 Linux 都兼容该构建。

## 启动与连接

在交付目录先离线查看界面：

```bash
chmod +x desktop/X2-MDI-2.1.0-x86_64.AppImage
./desktop/X2-MDI-2.1.0-x86_64.AppImage --demo
```

`--demo` 是本地模型展示，不建立 SSH 或机器人连接。不带参数启动时只显示连接界面，不自动连接机器人。
当前 AppImage 使用官方静态 type-2 runtime，无需系统安装 `libfuse2`。
如果系统不允许 FUSE 挂载，可用以下命令解包运行：

```bash
./desktop/X2-MDI-2.1.0-x86_64.AppImage --appimage-extract-and-run
```

若旧文件报 `dlopen(): error loading libfuse.so.2`，请替换为当前交付文件，或先用上面的命令启动。
真实连接前，先由用户完成主机身份验证和公钥登录：

```bash
ssh -i /path/to/key user@robot
```

然后在应用中填写自己的连接参数，或从命令行传入：

```bash
./desktop/X2-MDI-2.1.0-x86_64.AppImage \
  --host user@robot \
  --remote /path/to/robot/runtime \
  --config /path/to/robot/x2ik.conf \
  --identity /path/to/key
```

| 字段 | 含义 |
| --- | --- |
| SSH 地址 | `user@robot`，替换为自己的机器人用户与地址 |
| 运行目录 | `/path/to/robot/runtime`；该目录下的 Python 必须能导入 `x2ik` |
| 机器人配置 | `/path/to/robot/x2ik.conf`，机器人上的绝对路径 |
| 本机私钥 | `/path/to/key`，桌面电脑上的密钥路径；也可留空使用 SSH 默认认证 |

应用使用已知主机校验和非交互认证，不自动接受未知主机、不保存密码。
私钥由接收者自行准备，交付包不包含任何密钥。机器人端必须先按
[模块指南](MODULE_GUIDE.md) 安装模块、准备 ROS / AimDK 环境和本机配置。
桌面会启动 `python3 -m x2ik mdi --stdio`，无需再启动另一个 MDI 进程。
位置与姿态固定为对应侧机器人腕原点，不提供末端工具配置或标定。
更换机器人前须由维护者完成[换机配置与现场核验](MODULE_GUIDE.md#更换机器人与现场标定)。
桌面与公开 MoveJ 统一使用内置 `40 N·m/rad / 12° / pelvis` 补偿，不读取或要求 SN 标定文件。
尚无自动运动标定功能。不同末端工具的机构、质量、质心与负载不同，
必须单独建模、解析和验证重力补偿；本项目不支持跨末端工具泛化。
连接成功和显示正常不代表新机或工具精度已经验收。

## 3D 反馈与输入布局

双臂使用真实连杆 STL 来源的简化三角网格显示，保留骨架、关节角、TCP 位置与姿态数值。
视图显示躯干坐标轴、参考网格和腕原点坐标轴，来自现有反馈的模型 FK。
用户提供的目录中没有 STEP；本版网格来自 URDF 配套 STL，经减面处理用于显示，
不是 STEP CAD 实体或碰撞模型。旧反馈未提供连杆变换时可用已核对模型做本地 FK，并注明来源；
变换非法或外观资源不可用时退回骨架。这里只显示双臂连杆，腕原点由点和方向轴表示，不显示工具本体。
左右按机器人自身定义；正面面对机器人时，机器人的左臂在画面右侧。
相机视角不会交换左右臂数据或控制目标。
左键拖动旋转，滚轮缩放，右键或中键拖动平移，双击复位。
视角按钮提供默认 3D、正视、侧视、俯视；这些操作只调整观察视角，不改变目标或发送运动。
反馈过期、断开后不保留看似实时的手臂姿态。

每个目标的标签、单位和输入框放在同一个边框中，例如 `X [mm] | 目标值`。
模式、手臂、位置单位、运动时长和稳定时间也采用相同布局。
位姿分两行显示 XYZ / RxRyRz，七关节分两行显示 J1–J4 / J5–J7。

## 操作

1. 点击连接，默认只读显示双臂关节、腕原点位置/姿态、3D 双臂和运控状态。
2. 选择手臂与 MDI 模式，核对位置单位、目标、运动时长和稳定时间。
3. 点击预检验证数值、限位和目标 IK；预检不发送运动，会关闭此前的保持。
4. 显式启用发送，再下发 MDI。首次运动前仍检查 URS、新鲜反馈、IMU 和发布独占。
5. 双臂 HOME 按钮会让两条手臂实际运动，与当前选中的单臂无关。

| 模式 | 输入 | 含义 |
| --- | --- | --- |
| xyz | x y z | torso 系绝对位置，保持当前姿态 |
| pose | x y z rx ry rz | torso 系绝对 TCP 位姿 |
| d | dx dy dz | torso 系位置增量 |
| R | rx ry rz | 绕 torso 固定轴的旋转增量 |
| t | rx ry rz | 绕当前 TCP 自身轴的旋转增量 |
| rpy | rx ry rz | torso 系绝对姿态 |
| j | q1 … q7 | 所选侧七关节目标 |

输入位置可选择 m 或 mm，程序转成 m 后发送；切换单位后须重新核对输入数值。
显示的 XYZ 反馈为 m，所有界面角度为 deg，时长为 s。Python MoveJ 的关节角使用 rad。
关节顺序、坐标和 RPY 约定见 [坐标说明](COORDINATES.md)。
笛卡尔模式统一针对对应侧机器人腕原点；`j` 与 HOME 使用固定关节目标。

姿态不自动放宽，目标不裁剪。预检通过并不保证整条轨迹通过或路径无碰撞，
运行中的轨迹仍可能被检查拒绝。界面的 TCP 和骨架来自关节反馈的模型 FK，
不代表外部测量，也不包含碰撞检测。

## 保持、停止和运控状态

运动完成后，只要发送仍启用，会话持续保持。忙时的其他动作被拒绝，不排队回放。
心跳中断超过 2 秒、输入 EOF、停止发送或关闭连接会停止发送；
重新连接回到只读，不自动重试先前指令。

桌面关闭、断线或停止发送不追加 HOME、卸力或状态切换；停止发送不是硬件急停，
机器人后续保持与超时行为由当前运控决定。SSH 和桌面程序仍具有实际控制能力，
本地应用形态本身不保证运动安全，现场仍需保留原有急停与操作措施。

**仅在 URS 模式下支持运动。** 客户必须通过机器人原有操作工具自行切换到
`UPPERBODY_REMOTE_SPLIT`（URS），然后在 MDI 显式启用发送。
MDI 不带模式切换功能：没有切换按钮、不接受切换命令，也没有可开启该功能的配置开关。
非 URS 或状态过期时只允许反馈查看与预检；MDI 下发及 HOME 被拒绝。
外部离开 URS 后停止发送，恢复 URS 后须重新启用。

## 源码与第三方组件

交付目录 `source/` 包含桌面源码和构建脚本，`licenses/` 与
`THIRD_PARTY_NOTICES.txt` 记录随包组件的许可信息。桌面软件使用和修改不需要现场配置或原项目历史结果。

重新构建时需显式提供官方静态 `runtime-x86_64`，不要复用旧版 appimagetool 内置的 FUSE 2 runtime：

```bash
python3 source/tools/build_appimage.py \
  --appimagetool /path/to/appimagetool-x86_64.AppImage \
  --runtime-file /path/to/runtime-x86_64
```

本次 runtime 来自 [AppImage/type2-runtime](https://github.com/AppImage/type2-runtime/tree/8f39b89)，
版本和 SHA256 记录在 `desktop/APPIMAGE_BUILD_INFO.json`。构建脚本不会自动下载依赖。
