# X2 IK / MDI / MoveJ

灵犀 X2 双臂逆运动学与运动接口，提供机器人端 MDI、Python `Robot.moveJ()` 和 Linux x86_64 桌面客户端，桌面通过 SSH 连接机器人。

产品版本 V2.1，机器人模块 `2.1.1`，桌面客户端 `2.1.0`，Git 分支 `V2.0`。

## IK 2.1.1

这一版在原求解器中加入位姿复核、原位保持和局部连续求解，改善部分可达目标求解失败和连续运动跳解的问题，MDI 已接入新求解器，`Robot.moveJ()` 仍按七关节目标进行关节空间插值。

离线验证通过 112 个回归目标、200 个随机目标和双臂连续路径检查，验证样本的模型内位置残差不超过 0.01 mm、姿态残差不超过 0.001°，实机定位还受模型、关节零位、TCP 和负载影响，测试记录见 [IK 验证说明](docs/IK_VALIDATION.md)。

## 目录

```text
module/        wheel 和独立 runtime 安装包
source/        模块源码、测试及构建工具
desktop/      桌面 AppImage
examples/      MoveJ、MDI 和自定义 TCP 示例
docs/          使用说明与 IK 验证
verification/  安装包验证记录
licenses/      第三方许可
```

文件完整性可用 `sha256sum -c SHA256SUMS` 检查，再分发时保留 `licenses/` 和 `THIRD_PARTY_NOTICES.txt`。

## 安装

机器人端需要 Python 3.10 或以上、NumPy 和适配的 ROS 2 / AimDK 环境，运动前自行切换到 URS（`UPPERBODY_REMOTE_SPLIT`），以下命令在仓库根目录执行：

```bash
python3 -m pip install --upgrade module/x2ik-2.1.1-py3-none-any.whl
export X2IK_CONFIG=/path/to/robot/x2ik.conf
python3 -m x2ik --help
```

将 `X2IK_CONFIG` 替换为本机配置路径，也可以使用独立 runtime，二选一：

```bash
tar -xzf module/x2ik-runtime-2.1.1.tar.gz
cd runtime
export X2IK_CONFIG=/path/to/robot/x2ik.conf
python3 -m x2ik --help
```

查看实际加载的模块：

```bash
python3 -c "import x2ik; print(x2ik.__version__, x2ik.__file__)"
```

MDI / MoveJ 使用固定重力补偿参数：刚度 40 N·m/rad、补偿上限 12°、重力来源 pelvis，换机或换工具时按 [模块指南](docs/MODULE_GUIDE.md) 核对配置。

## Python MoveJ

下面的调用会驱动右臂到 HOME：

```python
from x2ik import Robot, HOME

with Robot(tcp_mode="gripper") as robot:
    result = robot.moveJ(HOME, side="right", duration=8, settle=2)
```

目标为七个关节角，单位 rad，顺序为肩 pitch / roll / yaw、肘、腕 yaw / pitch / roll，时长单位 s，双臂运动使用同一个实例：

```python
with Robot(tcp_mode="gripper") as robot:
    result = robot.moveJ(q_left=HOME, q_right=HOME, duration=8, settle=2)
```

TCP 可选 `none`（腕原点，默认）、`hand`、`gripper` 或 `custom`，自定义工具配合 `tcp_file`，TCP 用于末端坐标计算，夹爪开合需另行控制。

返回结果中，`err_max` 是关节误差（rad），`position_m` 和 `rotation_matrix` 是由关节反馈计算的 TCP 位姿，MoveJ 返回后不再后台发送保持命令。

`examples/movej_minimal.py` 默认只打印目标，添加 `--execute` 后连接机器人并运动。

## MDI 与桌面

在仓库根目录查看帮助或只读反馈：

```bash
sh examples/mdi_minimal.sh
sh examples/mdi_minimal.sh --dry --side right
```

进入在线控制：

```bash
sh examples/mdi_minimal.sh --execute --side right
```

MDI 支持绝对位姿、相对平移/旋转、关节目标和双臂 HOME，位置可用 m / mm，角度用 deg，目标位置指所选 TCP，在线会话中 `q!` 停止发送并退出，`q` 先让双臂回 HOME 再退出。

桌面离线预览：

```bash
chmod +x desktop/X2-MDI-2.1.0-x86_64.AppImage
./desktop/X2-MDI-2.1.0-x86_64.AppImage --demo
```

实际连接时填写 SSH 目标和机器人运行目录，选择 TCP，连接后先显示反馈，启用发送后才下发动作，关闭窗口不会自动 HOME，操作步骤见 [桌面指南](docs/DESKTOP_MDI_GUIDE.md)。

## 开发与文档

在工作站建立独立 Python 环境后测试和构建，不需要 ROS 或机器人：

```bash
cd source
python3 -m venv .venv
. .venv/bin/activate
python -m pip install "numpy>=1.21" "setuptools>=61" wheel
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m unittest discover -s tests -v
python tools/build_module.py
python tools/verify_release.py --output ../verification/2.1.1-local.json
python tools/update_checksums.py
```

wheel 与 runtime 来自同一份 `source/x2ik/`，验证结果文件已存在时换一个输出文件名。

- [项目介绍与 IK 原理](docs/PROJECT_OVERVIEW.md)
- [模块接口与机器人配置](docs/MODULE_GUIDE.md)
- [坐标系与单位](docs/COORDINATES.md)
- [TCP 标定与工具配置](docs/TCP_CALIBRATION_GUIDE.md)
- [IK 验证结果](docs/IK_VALIDATION.md)
