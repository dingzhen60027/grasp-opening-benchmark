# 受限开口搬运与抓持位姿不确定性 benchmark

**当前主场景已重建为仓储料架长条工件取放 v0.2。** 使用 UR5e＋官方 Robotiq 2F-85、空心方管、备料台、多层货架、邻近库存和双支承。入口见 [货架 benchmark 说明](RACK_BENCHMARK.zh-CN.md) 与 [实际验证结果](RACK_RESULTS.zh-CN.md)。

以下 v0.1 内容保留为几何敏感性的受控诊断集，不再代表主应用场景。

## v0.1 受控诊断集

研究任务：**面向受限开口搬运的抓持位姿不确定性鲁棒机械臂规划**。

本目录是独立、自建、可重复生成的仿真 benchmark。采用 **UR5e 六自由度机械臂**、明确尺寸的合成夹具和长条工件。借鉴 TAPOM 的“携带工件穿过受限空间”任务设定，环境与任务均由本项目独立生成；不依赖 TAPOM、ETH 私有子模块、Isaac Sim 或 ROS 运行时。

它是可运行的研究起点，当前版本不等于最终论文的完整实验集。所有误差参数是合成设定，尚未由真实抓持标定。

## 首版任务

| 场景族 | 几何结构 | 主要考察因素 |
|---|---|---|
| single_opening | 一个矩形框架开口 | 横向与下边缘净空、工件端点偏移 |
| deep_tunnel | 加厚至 130–190 mm 的开口 | 工件在有限厚度通道中的扫掠碰撞 |
| offset_pair | 两个横向错位开口 | 穿出一个开口后的横移、重新对准 |
| shelf_entry | 开口后增加背板和顶板 | 进入有边界的货架空间、终点碰撞 |
| turning_pair | 两个法向不同的开口 | 连续对准与转向，姿态误差的影响 |

每类包含 easy / medium / hard 三档，下边缘及横向名义净空分别为 **25 / 12 / 6 mm**。每档 4 个不同几何实例，共 **60 个任务**：30 个开发实例、30 个测试实例。不同实例还改变开口位置、工件长度和抓持偏移。难度标签是净空参数，不是预先测得的规划成功率等级。

工件为 35 mm × (320–440 mm) × 45 mm 的刚性长方体，长轴沿物体局部 y 方向。夹具为显式建模的固定平行夹爪，夹爪的接触区域从工件—机器人碰撞检查中排除，其余机械臂部分仍检查。当前只包含这一类工件形状，尚未加入 L 形、板件或复杂 CAD 工件。

生成时先构造穿过开口的笛卡尔可行性路径，再用真实 UR5e IK 求关节路径，检查关节限位、机械臂—障碍物、工件—障碍物、非相邻机械臂自碰撞及工件—机械臂碰撞。只有整条名义路径通过才收录；每个候选的失败原因保留在 `rejections.json`。没有“尝试失败后仍保存最后一个候选”的分支。

这种可行性路径构造会使数据偏向具有平滑穿越路径的任务；本版旨在隔离抓持误差，并不覆盖一般运动规划的全部拓扑难题。`reference_witnesses/` 仅为可行性证据，正式对比规划器不得读取。公开基线只读取任务 JSON。

## 抓持误差定义

用 T_WE(q) 表示末端在世界坐标系中的位姿，T_EO 表示名义工件抓持变换：

`T_WO(q, ξ) = T_WE(q) · Δ(ξ) · T_EO`

`Δ(ξ) = [Exp_SO3(φ), δt; 0, 1]`，δt 的单位为米，φ 是以弧度表示的旋转向量，均在末端坐标系中定义。这里的平移与旋转独立定义，**不是**完整 SE(3) twist 的指数映射。旋转围绕末端抓持参考点发生，能够产生长工件端点的杠杆放大效应。

一次执行只抽一个 ξ，并在整条轨迹中保持固定。默认每轴平移标准差 3 mm、每轴旋转向量标准差 2°，使用独立零均值高斯分布；支持仅平移、仅旋转、两者联合，以及 0 / 0.5 / 1 / 2 倍误差。标准差不是最大误差，高斯分布也不是有界不确定集。

规划器只能获得误差模型；执行时的具体误差不可提前提供。比较不同方法时使用相同的执行误差样本。若鲁棒规划方法内部采样，必须使用另一组独立种子。几何、误差和规划随机性分别记录，不能把同一几何的多次扰动当成多个独立场景。

## 成功判据与统计

- 输入关节路径必须从指定 q_start 到指定 q_goal，满足六个关节限位。
- 工件必须从所有开口前侧完整搬到后侧，并按顺序穿过指定开口；从框架外绕行不计成功。
- 整次执行中机械臂、夹具、工件与环境不发生禁用接触以外的碰撞。
- 实际工件终点相对名义目标的位置误差 ≤ 20 mm、姿态误差 ≤ 10°。
- 一个 episode 只产生一个成功/失败和一个整路径碰撞事件，不把轨迹点当作独立试验。

主要指标为规划成功率、执行任务成功率、整路径碰撞率及其逐实例 95% Clopper–Pearson 区间；辅助指标为真实规划墙钟时间、关节路径长度和碰撞查询次数。未找到路径应计入总任务失败，不能只展示返回路径的安全率。跨实例汇总应保留场景族和难度分层；本文档中的均值仅为开发集描述统计，不是跨场景置信区间。

正式实验建议 10 个规划种子、统一 600 秒预算、每条路径 1024 个独立误差 episode；这些是协议建议，本次没有运行完整方法对比。当前开发集健康检查使用每条名义可行性路径 256 个 episode，步长 0.005 rad；这是初步健康检查。测试集仅验证构造可行性，没有根据误差结果调参。

评估在关节插值后以每关节最大 0.00125 rad 的默认步长（`evaluate --joint-step` 可修改）进行离散检查。UR5e 使用公开模型原始碰撞网格，PyBullet 的网格碰撞表示遵循其加载规则；不声称精确 CAD 连续碰撞认证。零次观测碰撞也不能解释为风险为零。未模拟关节跟踪误差、动态滑移、抓持力学、执行速度或力矩限制。

所有任务已知名义可行；**不预先宣称每个实例在任意误差等级下都存在鲁棒可行解**。求解超时也不能视为不可行证明。

## 运行

推荐 Python 3.10，在本目录执行：

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
bash tools/python.sh -m pytest -q tests
```

运行包装脚本清除外部 PYTHONPATH，禁止用户级包注入，设置单线程 BLAS。可通过 `GOBENCH_PYTHON` 指定其他已安装依赖的解释器。

生成新实例（输出目录必须不存在）：

```bash
bash tools/python.sh -m gobench.generate --output data/my_suite --per-cell 4 --seed 251105052
bash tools/python.sh -m gobench.audit --data data/my_suite --output results/my_audit --episodes 256
```

运行不读取参考路径的 RRT-Connect：

```bash
bash tools/python.sh -m gobench.plan \
  --scene data/v0.1/tasks/dev/single_opening_hard_000.json \
  --output results/my_rrt --seed 0 --budget 20
bash tools/python.sh -m gobench.evaluate \
  --scene data/v0.1/tasks/dev/single_opening_hard_000.json \
  --path results/my_rrt/trajectory.npy --output results/my_rrt/uncertainty.json --episodes 512
```

`plan --margin 0.008` 提供统一膨胀 8 mm 的保守对照；该参数不是经过证明的风险界。`evaluate --mode rotation_only --scale 2` 可做旋转误差消融。算法失败不会生成轨迹，应读取 `result.json` 中的失败状态。

回放真实几何与关节轨迹：

```bash
bash tools/python.sh -m gobench.render \
  --scene data/v0.1/tasks/dev/single_opening_hard_000.json \
  --path results/my_rrt/trajectory.npy --output previews/my_rrt --video
```

回放按关节路径弧长重采样，仅为运动学动画，不代表执行速度。

## 资产与证据

- `assets/ur5e/`：自包含 URDF、7 个 STL、原许可证、模型参数与来源哈希；六个机械臂关节和碰撞网格来自 [Universal Robots 官方描述库](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)，固定提交见 `provenance.json`。显示模型使用原碰撞网格；合成夹具另行添加。`tools/prepare_ur5e.py` 记录资产准备流程，日常运行不需要 xacro。
- `data/v0.1/manifest.json`：任务清单、几何种子、划分、文件 SHA-256。
- `configs/protocol_v0.1.json`：评测口径与合成误差参数。
- `results/`：独立复查、误差 episode 样本、碰撞事件和基线运行记录。
- `previews/`：根据实际模型和任务生成的渲染。
- `tests/`：变换约定、单位、零误差一致性、碰撞敏感性、禁止绕框架、种子划分、真实机器人 IK/FK 检查。

`demo_detour` 另外提供一个单开口的固定“抬升 45 mm、偏航 0.45 rad”示范路径，用于检查同一个任务是否容许不同安全裕度的解。它从任务几何直接生成，不读取执行误差或参考路径；它不代表本研究拟提出的新鲁棒算法，也不是对五类场景均适用的通用规划器。

```bash
bash tools/python.sh -m gobench.demo_detour \
  --scene data/v0.1/tasks/dev/single_opening_hard_000.json \
  --output results/my_detour
bash tools/python.sh -m gobench.preview_suite
```

参考任务动机：[TAPOM，arXiv:2511.05052，第 V 节](https://arxiv.org/html/2511.05052v1#S5)。本 benchmark 不复制其未公开模型或实验结果，也不将现有合成结果当作本文拟研究鲁棒方法的性能结论。
