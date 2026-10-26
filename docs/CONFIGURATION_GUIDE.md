# V2.1 机器人配置与 MDI 连接填写

交付包提供 `config/x2ik.conf` 待填写模板。客户须复制到机器人，按该机器的
ROS / AimDK 环境填写，再把文件的绝对路径填入 MDI 的“机器人配置”。
安装 wheel 不会生成这份现场配置；

## 1. 三种路径的用途

| 路径 | 用途 | 示例 |
| --- | --- | --- |
| wheel 文件路径 | `pip install` 读取的安装包，可存放在任意目录 | `/home/agi/downloads/x2ik-2.1.1-py3-none-any.whl` |
| MDI 运行目录 | SSH 登录机器人后，启动 `python3 -m x2ik` 的工作目录 | wheel 安装：`/home/agi/x2ik/run`；runtime 解压：`/home/agi/x2ik/runtime` |
| 机器人配置 | 机器人上已填写的 `x2ik.conf` 文件路径 | `/home/agi/x2ik/x2ik.conf` |

wheel 安装到所选 Python 的包目录（通常为 `site-packages` 或 `dist-packages`），
不会安装到执行 `pip install` 时所在的目录。

**“运行目录”是启动工作目录，不是 wheel 所在目录、安装命令所在目录或 Python 包目录。**
使用 wheel 时可以选择一个存在且 SSH 用户有权限进入的工作目录，要求启动用的 Python 能导入 `x2ik`。
使用 runtime 时，填写解压后包含 `x2ik/` 子目录的 `runtime/`，不要填到 `runtime/x2ik/`。

以下 `/home/agi`、`user@robot` 和 `/opt/aimdk` 均为示例，须替换为目标机器的实际值。

## 2. 获取并填写 x2ik.conf

在**机器人上的交付包根目录**执行：

```bash
mkdir -p /home/agi/x2ik/run
cp -n config/x2ik.conf /home/agi/x2ik/x2ik.conf
```

`cp -n` 保留目标位置已有文件。随后用文本编辑器打开
`/home/agi/x2ik/x2ik.conf`，按下表填写。不要直接使用模板中的占位路径。

| 字段 | 要求 | 如何获取 |
| --- | --- | --- |
| `X2_AIMDK_MSGS` | 必须把模板占位值替换为该机器的消息包安装前缀 | 在已加载 ROS/AimDK 环境的机器人终端执行 `ros2 pkg prefix aimdk_msgs` |
| `X2_ROS_PYTHON` | 可选，优先尝试的 ROS Python 解释器绝对路径 | 由维护者确认，例如 `/usr/bin/python3`；必须匹配 ROS Python 版本 |
| `X2_ROBOT_SN` | 可选，人工核对后的机器人编号 | 查看该机器的铭牌或设备管理信息；不填写时保持整行注释 |
| `X2_RMW` | 必须核对；现用值不同于默认值时须填写 | 标准真机启动默认 `rmw_fastrtps_cpp`，由维护者确认机器人现用值 |
| `X2_URS_ONLY` | 保留 `1` | MDI 和 MoveJ 仅支持机器人已处于 URS 时运动 |

例如，若 `ros2 pkg prefix aimdk_msgs` 的输出是 `/opt/aimdk/install/aimdk_msgs`，填写：

```ini
X2_AIMDK_MSGS=/opt/aimdk/install/aimdk_msgs
X2_URS_ONLY=1
```

`X2_AIMDK_MSGS` 指向包含已编译消息库和 Python 消息模块的安装前缀，
不是 `src/aimdk_msgs` 源码目录，也不是 `lib/python3.x/site-packages/aimdk_msgs`。
合并安装的前缀可能直接是 `/opt/aimdk/install`，以查询结果为准。
如果命令找不到包，请先让维护者加载该机器的 ROS 与 AimDK 安装环境，确认消息包已安装。

可在已加载环境的机器人终端验证 Python 消息库是否可用，以下命令不连接或驱动机器人：

```bash
python3 -c 'import sys, rclpy; print(sys.executable); print(rclpy.__file__)'
python3 -c 'from rosidl_generator_py import import_type_support; import_type_support("aimdk_msgs"); print("aimdk_msgs OK")'
```

配置文件每行写 `KEY=value`，不写 `export`，值不加引号。路径使用机器人绝对路径，
不使用 `$HOME`、`${变量}` 或命令替换。注释放在单独一行；路径中不能包含 `#`。
已有进程环境中的同名 `X2_*` 变量优先，修改文件后须重新启动会话才生效。
客户端启动器按 `X2_RMW` 选择通信实现；请维护者核对现用值，不能仅凭已设置
`RMW_IMPLEMENTATION` 就认定启动器会沿用它。`ROS_DOMAIN_ID`、DDS 配置路径等由现有启动环境提供，
不要写入本配置文件。非交互 SSH 会话也须具备机器人要求的环境。

配置用于选择本模块使用的环境和消息包，不包含 SSH 密钥或机器人登录密码。
TCP 工具 JSON 另行填写，见 [TCP 标定指南](TCP_CALIBRATION_GUIDE.md)。
机器人原有 ROS/DDS 环境由维护者保留并提供给客户端；不要为本模块修改其他机器人模块的配置。

## 3. 按安装方式填写运行目录

### wheel 安装

在机器人交付包根目录中，使用后续启动模块的 Python 安装：

```bash
python3 -m pip install module/x2ik-2.1.1-py3-none-any.whl
cd /home/agi/x2ik/run
export X2IK_CONFIG=/home/agi/x2ik/x2ik.conf
test -f "$X2IK_CONFIG"
python3 -c 'import x2ik; print(x2ik.__file__)'
python3 -m x2ik movej
```

最后一条只打印目标，不连接或驱动机器人。以上检查确认文件存在与模块可导入，
不验证机器人通信或实际运动条件。导入结果应指向安装的 `x2ik/__init__.py`。
工作目录中不要放同名 `x2ik.py` 或其他会遮蔽安装包的文件。

此例 MDI 填写：运行目录 `/home/agi/x2ik/run`，机器人配置 `/home/agi/x2ik/x2ik.conf`。
如果在虚拟环境中安装，启动 AppImage 时用 `--python /机器人上的虚拟环境/bin/python`
指定能导入模块的解释器；桌面本机 Python 不参与机器人模块运行。

### runtime 解压

在机器人交付包根目录执行：

```bash
mkdir -p /home/agi/x2ik
tar -xzf module/x2ik-runtime-2.1.1.tar.gz -C /home/agi/x2ik
cd /home/agi/x2ik/runtime
export X2IK_CONFIG=/home/agi/x2ik/x2ik.conf
test -f "$X2IK_CONFIG"
python3 -c 'import x2ik; print(x2ik.__file__)'
python3 -m x2ik movej
```

配置文件仍按第 2 节单独准备。导入结果应指向
`/home/agi/x2ik/runtime/x2ik/__init__.py`。
此例 MDI 填写：运行目录 `/home/agi/x2ik/runtime`，机器人配置 `/home/agi/x2ik/x2ik.conf`。

## 4. MDI 窗口填写示例

| MDI 字段 | wheel 安装示例 | runtime 解压示例 | 路径所在机器 |
| --- | --- | --- | --- |
| SSH 地址 | `agi@机器人IP` | `agi@机器人IP` | 机器人账户与地址 |
| 运行目录 | `/home/agi/x2ik/run` | `/home/agi/x2ik/runtime` | 机器人 |
| 机器人配置 | `/home/agi/x2ik/x2ik.conf` | `/home/agi/x2ik/x2ik.conf` | 机器人 |
| 本机私钥 | `/home/客户用户名/.ssh/id_ed25519`，或留空使用默认认证 | 同左 | 运行 AppImage 的电脑 |
| TCP 工具 | 按实际安装选择无、灵巧手、夹爪或自定义 | 同左 | 会话选项 |
| TCP 文件 | 仅自定义时填写，例如 `/home/agi/x2ik/my_tcp.json` | 同左 | 机器人；离线 `--demo` 才使用桌面本机文件 |

界面的路径框填写完整绝对路径，不用 `~` 或 `$HOME`。配置文件名必须包含在路径中。
先按 [桌面 MDI 指南](DESKTOP_MDI_GUIDE.md) 完成 SSH 主机验证和公钥认证，再点击“连接（只读）”。

wheel 示例的等效桌面启动命令为：

```bash
./desktop/X2-MDI-2.1.0-x86_64.AppImage \
  --host user@robot \
  --remote /home/agi/x2ik/run \
  --config /home/agi/x2ik/x2ik.conf
```

运行目录和配置文件可以位于不同目录。桌面会把“机器人配置”作为 `X2IK_CONFIG`
传给机器人端程序，不会自动上传桌面上的文件。

## 5. Python MoveJ 使用同一份配置

运行业务脚本前，先加载该机器人已有的 ROS / AimDK 环境，并在启动 Python 前设置：

```bash
export X2IK_CONFIG=/home/agi/x2ik/x2ik.conf
```

Python `Robot.moveJ()` 不代替 ROS 环境初始化。命令行与桌面启动器默认使用
`/opt/ros/humble`；若 ROS 安装在其他位置，须由维护者在启动进程环境中设置
`X2_ROS_ROOT`，不能仅把它加到 `x2ik.conf` 中。SSH 非交互会话也须具备对应环境。

使用终端命令时，`X2IK_CONFIG` 未设置则从当前工作目录寻找 `x2ik.conf`。
为使终端、桌面与 Python 业务程序使用同一配置，建议始终显式指定文件绝对路径。
