# contrlSDK

Marvin 系列机器人控制 SDK，提供两种使用方式：

| 方式 | 产物 | 适用场景 |
|------|------|---------|
| C++ 动态库 | `libMarvinSDK.so` | C++ 程序直接调用 |
| Python 扩展 | `marvin_sdk.cpython-*.so` | Python 控制脚本 |

---

## 目录结构

```
contrlSDK/
├── src/                     # SDK C++ 源文件及头文件
│   ├── MarvinSDK.h/cpp      # extern "C" 导出 API
│   ├── Robot.h/cpp          # CRobot 核心类（单例，全静态方法）
│   ├── PointSet.h/cpp       # 轨迹点集（PLN 规划用）
│   ├── FxRtCSDef.h          # 数据结构（DCSS / RT_IN / RT_OUT / ArmState）
│   ├── FxType.h             # 基础类型定义
│   └── ...                  # 内部模块（ACB / TCPAgent / ShMem 等）
├── scripts/
│   └── make_deb.sh          # 本地打包脚本
├── marvin_bind.cpp          # pybind11 绑定实现
├── makefile                 # 编译 libMarvinSDK.so（C++ 用）
├── CMakeLists.txt           # 编译 Python 扩展模块
├── pyproject.toml           # pip install . 支持
└── build/                   # CMake 构建目录（.gitignore 中忽略）
```

仓库根目录：

```
.github/workflows/
└── build_deb.yml            # CI/CD：构建 → 打包 deb → 发布 Release
```

---

## 编译

### 依赖

```bash
# 编译工具
sudo apt install g++ make cmake dpkg-dev

# Python 扩展额外需要
pip install pybind11
```

### C++ 动态库

```bash
make
# → libMarvinSDK.so
```

### Python 扩展

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
# → build/marvin_sdk.cpython-310-x86_64-linux-gnu.so
```

安装到系统 site-packages（可直接 `import marvin_sdk`）：

```bash
cmake --install build/
```

或开发模式（不安装，临时加入路径）：

```python
import sys
sys.path.insert(0, '/path/to/contrlSDK/build')
import marvin_sdk as sdk
```

### 打包为 .deb

```bash
bash scripts/make_deb.sh 1.0.0
# → marvin-sdk_1.0.0_amd64.deb          libMarvinSDK.so + 头文件
# → python3-marvin-sdk_1.0.0_amd64.deb  Python 扩展（自包含）
```

安装：

```bash
sudo dpkg -i marvin-sdk_1.0.0_amd64.deb
sudo dpkg -i python3-marvin-sdk_1.0.0_amd64.deb
```

| 包 | 安装路径 | 依赖 |
|----|---------|------|
| `marvin-sdk` | `/usr/lib/x86_64-linux-gnu/libMarvinSDK.so`<br>`/usr/include/marvin/` | libc6, libstdc++6 |
| `python3-marvin-sdk` | `/usr/lib/python3/dist-packages/marvin_sdk.cpython-*.so` | python3 ≥ 3.8 |

---

## CI/CD

流水线配置：`.github/workflows/build_deb.yml`

```
push to main ──→ 编译 → 冒烟测试 → 上传 artifact（保留 30 天）

push tag v* ──→ 编译 → 冒烟测试 → 上传 artifact
                                └──→ 创建 GitHub Release（附 .deb 文件）

workflow_dispatch ──→ 编译 → 冒烟测试 → [可选 SSH 部署到机器人]
```

### 触发版本发布

```bash
git tag v1.0.0
git push origin v1.0.0
# GitHub Actions 自动编译并在 Releases 页面发布两个 .deb
```

### 启用 SSH 部署

取消 workflow 中 `deploy` job 的注释，在仓库 **Settings → Secrets** 添加：

| Secret | 说明 |
|--------|------|
| `ROBOT_SSH_HOST` | 目标机器 IP |
| `ROBOT_SSH_USER` | SSH 用户名 |
| `ROBOT_SSH_KEY` | SSH 私钥（PEM 格式） |

---

## 控制逻辑

```
连接机器人 (connect)
    │
    ├─ sleep(0.5s)
    ├─ 清错 (clear_err)
    │
    ├─ 设置模式参数（速度/加速度/阻抗系数等）
    │     clear_set()
    │     set_vel_acc / set_joint_kd / set_cart_kd ...
    │     send_cmd()
    │
    ├─ 切换控制模式
    │     clear_set() → set_state(POSITION/TORQ/PVT) → send_cmd()
    │
    ├─ 循环下发指令
    │     clear_set() → set_joint_cmd / set_force_cmd → send_cmd()
    │     get_buf(dcss)  ← 订阅反馈（1KHz）
    │
    └─ 释放 (release)
```

> **注意**：所有设置类接口必须包裹在 `clear_set()` 和 `send_cmd()` 之间，接口之间至少间隔 1ms。

---

## Python 快速上手

### 连接与订阅

```python
import marvin_sdk as sdk
import numpy as np
import time

sdk.connect('192.168.1.190')
time.sleep(0.5)

sdk.clear_set()
sdk.clear_err('A')   # 连接后必须清错
sdk.send_cmd()

# 订阅反馈数据（复用 DCSS 对象，高频调用推荐）
dcss = sdk.DCSS()
sdk.get_buf(dcss)

state  = dcss.states[0].cur_state       # 当前状态，对照 ArmState 枚举
joints = dcss.outputs[0].fb_joint_pos   # 左臂A 关节位置 list[7]，单位度
torque = dcss.outputs[0].fb_joint_stoq  # 关节传感器扭矩 list[7]，N·m
```

### 位置模式

```python
sdk.clear_set()
sdk.set_vel_acc('A', vel_ratio=10, acc_ratio=10)   # 速度/加速度 10%
sdk.set_state('A', sdk.ArmState.POSITION)
sdk.send_cmd()
time.sleep(0.5)

# 下发目标关节角
target = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

sdk.clear_set()
sdk.set_joint_cmd('A', target)
sdk.send_cmd()

# 等待到位（轮询低速标志）
while True:
    sdk.get_buf(dcss)
    if dcss.outputs[0].low_speed_flag:
        break
    time.sleep(0.001)
```

### PLN 关节规划（消除抖动）

```python
sdk.pln_init('ccs_m3.MvKDCfg')   # 根据机型选择配置文件

current = dcss.outputs[0].fb_joint_pos
target  = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

sdk.clear_set()
sdk.set_pln_joint('A',
    start=np.array(current),
    stop=np.array(target),
    vel=0.1, acc=0.1)   # 10%
sdk.send_cmd()
```

### 关节阻抗模式

```python
K = np.array([2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 1.0])  # 刚度 N·m/°
D = np.array([0.5, 0.5, 0.5, 0.3, 0.3, 0.3, 0.3])  # 阻尼

sdk.clear_set()
sdk.set_vel_acc('A', 10, 10)
sdk.set_joint_kd('A', K, D)
sdk.set_imp_type('A', 1)           # 1 = 关节阻抗
sdk.set_state('A', sdk.ArmState.TORQ)
sdk.send_cmd()
```

### 笛卡尔阻抗模式

```python
K = np.array([1000., 1000., 1000., 50., 50., 50., 0.])  # XYZ(N/m), RxRyRz(N·m/rad), Kn
D = np.array([0.7,   0.7,   0.7,   0.5,  0.5,  0.5, 0.])

sdk.clear_set()
sdk.set_vel_acc('A', 10, 10)
sdk.set_cart_kd('A', K, D, type=2)
sdk.set_imp_type('A', 2)           # 2 = 笛卡尔阻抗
sdk.set_state('A', sdk.ArmState.TORQ)
sdk.send_cmd()
```

### 力控模式

```python
fxdir = np.array([0., 0., 1., 0., 0., 0.])   # 沿 Z 轴力控
para  = np.zeros(7)

sdk.clear_set()
sdk.set_force_ctrl('A', fc_type=0, fxdir=fxdir, para=para, adj_lmt=50.0)
sdk.set_imp_type('A', 3)           # 3 = 力控
sdk.set_state('A', sdk.ArmState.TORQ)
sdk.send_cmd()
time.sleep(0.5)

# 下发目标力
sdk.clear_set()
sdk.set_force_cmd('A', force=5.0)  # 5 N
sdk.send_cmd()
```

### 数据采集

```python
# 采集左臂关节位置(0-6)和关节扭矩(50-56)，共14列，10000行
ids = [0,1,2,3,4,5,6, 50,51,52,53,54,55,56] + [0]*21

sdk.clear_set()
sdk.start_gather(num=14, ids=ids, rows=10000)
sdk.send_cmd()

time.sleep(12)  # 等待采集完成（10000 行 @ 1KHz ≈ 10s）

sdk.clear_set()
sdk.stop_gather()
sdk.send_cmd()

time.sleep(0.2)
sdk.save_gather('/tmp/robot_data.csv')
```

### 末端模组通信（CAN / RS485）

```python
# 发送 HEX 指令到 CAN 端（ch=1）
sdk.clear_ch_data('A')
sdk.set_ch_data('A', data=bytes([0x01, 0x06, 0x00, 0x01, 0x00, 0x01]), ch=1)

# 接收回复
data, ch = sdk.get_ch_data('A')
print(data.hex(' ').upper())
```

---

## Python API 速查

### 连接

| 函数 | 说明 |
|------|------|
| `connect(ip)` | 连接机器人，如 `'192.168.1.190'` |
| `release()` | 释放连接（程序结束必须调用） |
| `sdk_version()` | 返回 SDK 版本号 |

### 指令帧

| 函数 | 说明 |
|------|------|
| `clear_set()` | 指令帧开始 |
| `send_cmd()` | 指令帧结束并发送 |
| `send_cmd_wait(ms)` | 发送并等待响应，返回实际延时 |

### 模式切换

| 函数 | 说明 |
|------|------|
| `set_state(arm, state)` | 切换模式，`state` 见 `ArmState` 枚举 |
| `set_imp_type(arm, type)` | 阻抗类型：1=关节 2=笛卡尔 3=力控 |
| `set_drag(arm, dtype)` | 拖动：0=退出 1=关节 2/3/4/5=笛卡尔XYZ/旋转 |

### 参数设置（需在 clear_set/send_cmd 内）

| 函数 | 参数 |
|------|------|
| `set_vel_acc(arm, vel, acc)` | 速度/加速度百分比 0-100 |
| `set_tool(arm, kine[6], dyn[10])` | 工具运动学和动力学参数 |
| `set_joint_kd(arm, K[7], D[7])` | 关节阻抗刚度/阻尼 |
| `set_cart_kd(arm, K[7], D[7], type)` | 笛卡尔阻抗刚度/阻尼 |
| `set_force_ctrl(arm, fc_type, fxdir[6], para[7], adj_lmt)` | 力控参数 |

### 运动指令（需在 clear_set/send_cmd 内）

| 函数 | 说明 |
|------|------|
| `set_joint_cmd(arm, joints[7])` | 目标关节角度（度） |
| `set_force_cmd(arm, force)` | 目标力（N 或 N·m） |
| `set_pvt(arm, id)` | 运行已上传的 PVT 轨迹 |
| `set_pln_joint(arm, start[7], stop[7], vel, acc)` | 关节规划指令（防抖） |
| `set_pln_cart(arm, pset)` | 笛卡尔直线规划 |

### 反馈订阅

| 函数 | 说明 |
|------|------|
| `get_buf(dcss)` | 原地填充 DCSS（高频推荐） |
| `subscribe()` | 返回新 DCSS 对象（低频/调试） |

### ArmState 枚举

| 值 | 含义 |
|----|------|
| `ArmState.IDLE` (0) | 下伺服 |
| `ArmState.POSITION` (1) | 位置跟随 |
| `ArmState.PVT` (2) | PVT 离线轨迹 |
| `ArmState.TORQ` (3) | 扭矩/阻抗 |
| `ArmState.RELEASE` (4) | 协作释放 |
| `ArmState.ERROR` (100) | 报错，需调用 `clear_err()` |

---

## C++ 调用

```cpp
#include "MarvinSDK.h"

int main() {
    OnLinkTo(192, 168, 1, 190);

    DCSS dcss = {};
    OnGetBuf(&dcss);

    // 切换位置模式
    OnClearSet();
    OnClearErr_A();
    OnSetJointLmt_A(10, 10);
    OnSetTargetState_A(1);
    double joints[7] = {0};
    OnSetJointCmdPos_A(joints);
    OnSetSend();

    OnRelease();
}
```

编译：

```bash
g++ main.cpp -L. -lMarvinSDK -Isrc -Wl,-rpath,. -o robot_ctrl
```

---

## 注意事项

1. **连接后先清错**：`connect()` 后 sleep 500ms，再调用 `clear_err()`
2. **释放机器人**：程序结束必须调用 `release()`，否则其他进程无法连接
3. **接口间隔**：API 调用之间至少间隔 1ms；`save_param()` 后至少 sleep 1s
4. **不要混用**：SDK 和上位机软件不可同时连接同一机器人（端口冲突）
5. **模式切换**：不同大类切换（如位置→扭矩）建议先切到 IDLE 再切目标
6. **后缀规则**：`_A` = 左臂，`_B` = 右臂；单臂系统使用 `_A`
