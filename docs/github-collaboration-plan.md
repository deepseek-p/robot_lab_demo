# GitHub 协作与写作分工方案

本文档用于指导 `robot_lab_demo` 项目的 GitHub 协作、报告写作和模块分工。项目包含视觉感知、导航与场景、机械臂控制、大语言模型语义理解四条主线，协作方式参考 MoveIt 2、Nav2、ROS 2 等成熟机器人开源项目的做法，并按本项目规模简化。

## 1. PR 是什么

PR 是 Pull Request，中文可以理解为“合并请求”。

团队成员不要直接把修改提交到 `main` 主分支，而是先从 `main` 创建自己的工作分支，在分支上完成修改后，在 GitHub 上发起 PR，请其他成员审查。审查通过后，再把该分支合并回 `main`。

基本流程如下：

```text
main 主分支
  ↑
PR 审查通过后合并
  ↑
个人工作分支
```

示例：

```bash
git checkout -b docs/perception-section
# 修改视觉章节
git add docs/report/03-perception.md
git commit -m "docs: add perception section"
git push origin docs/perception-section
```

然后在 GitHub 页面上创建 Pull Request。

PR 的作用：

- 让修改先被其他成员检查，减少错误进入主分支。
- 保留讨论记录，方便后续写报告和答辩。
- 明确每次修改解决了哪个 Issue。
- 避免多人直接改 `main` 导致冲突和混乱。

## 2. 成熟机器人项目的协作特点

成熟机器人项目通常不是按“谁随便改哪个文件”协作，而是通过模块边界、Issue、PR、CI 和文档来协作。

典型做法：

- MoveIt 2 按运动规划、ROS 接口、插件、教程等模块拆分目录。
- Nav2 要求先创建 Issue，说明任务、方案和时间，再提交 PR。
- ROS 2 要求主分支保持可构建，代码修改必须经过 PR 和测试。
- GitHub 官方推荐 GitHub Flow：创建分支、提交修改、发 PR、审查、合并、删除分支。

本项目采用轻量版规则：

```text
main 不直接提交
每个任务先建 Issue
每个 Issue 一个主负责人
每个改动开一个分支
每个分支只解决一个 Issue
写完开 PR
至少一个其他成员 review
通过后合并
```

## 3. 推荐项目文件分布

建议在现有代码基础上补充文档和 GitHub 协作文件：

```text
robot_lab_demo/
├── README.md
├── CONTRIBUTING.md
├── CODEOWNERS
├── .github/
│   ├── PULL_REQUEST_TEMPLATE.md
│   └── ISSUE_TEMPLATE/
│       ├── docs_task.md
│       ├── experiment_task.md
│       └── feature_task.md
├── docs/
│   ├── architecture.md
│   ├── github-collaboration-plan.md
│   ├── report/
│   │   ├── 00-abstract.md
│   │   ├── 01-introduction.md
│   │   ├── 02-system-architecture.md
│   │   ├── 03-perception.md
│   │   ├── 04-navigation-and-scene.md
│   │   ├── 05-manipulation-control.md
│   │   ├── 06-llm-semantic-planning.md
│   │   ├── 07-experiments.md
│   │   └── 08-conclusion.md
│   ├── figures/
│   └── experiments/
├── results/
├── src/
│   ├── robot_lab_description/
│   ├── robot_lab_bringup/
│   ├── robot_lab_perception/
│   ├── robot_lab_tasks/
│   └── robot_lab_sim_tools/
└── tests/
```

其中：

- `src/` 放 ROS 2 源码。
- `results/` 放实验结果数据。
- `docs/report/` 放最终报告正文。
- `docs/figures/` 放报告图片、流程图、架构图。
- `docs/experiments/` 放实验设计、实验记录和结果解释。
- `.github/` 放 GitHub Issue 和 PR 模板。
- `CODEOWNERS` 放文件负责人规则。
- `CONTRIBUTING.md` 放团队协作规则。

## 4. 四个分工与文件归属

### 4.1 视觉感知负责人

负责范围：

- RGB-D 相机输入。
- 点云处理。
- 目标检测。
- 6D 位姿估计。
- 感知实验结果。

主要文件：

```text
src/robot_lab_perception/
results/perception_eval.csv
docs/report/03-perception.md
docs/experiments/perception-evaluation.md
docs/figures/perception-*.png
```

需要写清楚：

- `/bench_camera/points` 点云输入来自哪里。
- `object_pose_estimator.py` 如何处理点云。
- 检测后端是颜色检测，ONNX 后端是预留接口。
- 发布了哪些 topic，例如 `/perception/*/pose`。
- `results/perception_eval.csv` 中的准确率和误差结果。

### 4.2 导航与场景负责人

负责范围：

- Gazebo 实验室环境。
- 麦克纳姆底盘变体。
- 动态障碍物。
- 障碍物同步到 MoveIt planning scene。
- 后续自主导航扩展方案。

主要文件：

```text
src/robot_lab_description/worlds/lab_minimal.sdf
src/robot_lab_description/urdf/lab_ur_mecanum.urdf.xacro
src/robot_lab_bringup/launch/lab_ur_mecanum_gz.launch.py
src/robot_lab_tasks/scripts/obstacle_monitor.py
results/obstacle_latency.csv
docs/report/04-navigation-and-scene.md
docs/experiments/obstacle-latency.md
```

需要写清楚：

- 实验室环境包含哪些对象。
- 动态障碍物如何进入规划场景。
- `obstacle_monitor.py` 的作用。
- 麦克纳姆底盘目前是实验性路径。
- 后续如果扩展自主导航，需要补充哪些模块。

### 4.3 机械臂控制负责人

负责范围：

- UR5e 机械臂模型。
- 夹爪模型和控制。
- MoveIt 运动规划。
- 抓取、搬运、放置流程。
- 抓取成功率评估。

主要文件：

```text
src/robot_lab_description/urdf/lab_ur_gripper.urdf.xacro
src/robot_lab_description/srdf/lab_ur_gripper.srdf.xacro
src/robot_lab_bringup/launch/lab_ur_moveit_gz.launch.py
src/robot_lab_bringup/config/lab_ur_controllers.yaml
src/robot_lab_tasks/src/pick_place_node.cpp
src/robot_lab_tasks/src/scripted_pick_demo.cpp
src/robot_lab_tasks/src/preset_joint_demo.cpp
results/pick_place_eval.csv
docs/report/05-manipulation-control.md
docs/experiments/pick-place-evaluation.md
```

需要写清楚：

- UR5e 和夹爪如何建模。
- MoveIt group、控制器和 launch 文件如何配合。
- `pick_place_node.cpp` 的抓取放置流程。
- 遇到障碍物时如何中止、重规划和重试。
- `results/pick_place_eval.csv` 中的成功率结果。

### 4.4 大语言模型语义理解负责人

负责范围：

- 自然语言任务输入。
- 任务分解。
- 动作 JSON schema。
- 规则解析器。
- LLM Planner 预留接口。
- 执行前的对象和目标校验。

主要文件：

```text
src/robot_lab_tasks/scripts/task_orchestrator.py
docs/report/06-llm-semantic-planning.md
docs/architecture.md
docs/figures/task-orchestration-flow.png
```

需要写清楚：

- 用户指令通过 `/task/instruction` 输入。
- `RuleBasedPlanner` 当前如何按关键词解析任务。
- `LlmPlanner` 是未来接入大模型的接口。
- 解析结果如何变成 `/task/command`。
- 动作格式应保持为结构化 JSON，不能直接让大模型控制机械臂底层动作。

示例动作格式：

```json
{
  "action": "pick_place",
  "object": "sample_block",
  "target": "target_pad"
}
```

### 4.5 项目整合负责人

负责范围：

- README 总览。
- 报告整体结构。
- 系统架构图。
- GitHub 协作规则。
- 最终文档统一。

主要文件：

```text
README.md
CONTRIBUTING.md
CODEOWNERS
docs/architecture.md
docs/github-collaboration-plan.md
docs/report/00-abstract.md
docs/report/01-introduction.md
docs/report/02-system-architecture.md
docs/report/07-experiments.md
docs/report/08-conclusion.md
```

需要写清楚：

- 项目整体目标。
- 系统模块之间如何通信。
- 实验数据如何支撑结论。
- 各章节术语和图表编号保持一致。

## 5. 建议的 GitHub 标签

建议创建以下 Labels：

```text
perception
navigation
manipulation
llm
writing
experiment
bug
enhancement
documentation
review-needed
blocked
```

使用方式：

- 写视觉章节：`perception` + `writing`
- 补充抓取实验：`manipulation` + `experiment`
- 修改任务解析代码：`llm` + `enhancement`
- 修复启动错误：`bug`
- 等待别人审查：`review-needed`

## 6. Issue 模板示例

文档任务 Issue 示例：

```text
标题：docs: 完成视觉感知章节

负责人：视觉成员
标签：perception, writing

涉及文件：
- docs/report/03-perception.md
- results/perception_eval.csv
- src/robot_lab_perception/robot_lab_perception/object_pose_estimator.py

任务说明：
补充视觉感知章节，说明 RGB-D 点云输入、目标检测、6D 位姿估计和实验结果。

验收标准：
- 说明 /bench_camera/points 点云输入
- 说明目标检测方法
- 说明 /perception/*/pose 输出
- 引用 perception_eval.csv 的实验结果
- 至少包含一张流程图或系统图
```

功能任务 Issue 示例：

```text
标题：feat: 设计 LLM Planner 输出 schema

负责人：LLM 成员
标签：llm, enhancement

涉及文件：
- src/robot_lab_tasks/scripts/task_orchestrator.py
- docs/report/06-llm-semantic-planning.md

任务说明：
定义大模型输出的动作 JSON schema，并说明如何校验 object 和 target。

验收标准：
- 给出 pick_place 和 home 两类动作格式
- 说明未知物体和未知目标如何拒绝执行
- 不改变现有 RuleBasedPlanner 的默认行为
```

## 7. PR 模板建议

每个 PR 至少包含：

```text
## 修改内容

-

## 关联 Issue

Closes #

## 验证方式

- [ ] 已阅读修改后的文档
- [ ] 已运行相关测试或说明无需运行
- [ ] 已检查没有无关文件改动

## 影响范围

-

## 需要重点 Review 的地方

-
```

## 8. 分支命名规则

建议使用：

```text
docs/perception-section
docs/manipulation-control
docs/llm-semantic-planning
feat/llm-planner-schema
fix/obstacle-monitor-latency
experiment/pick-place-eval
```

命名原则：

- 分支名能看出任务内容。
- 文档任务用 `docs/`。
- 功能任务用 `feat/`。
- 修复任务用 `fix/`。
- 实验任务用 `experiment/`。

## 9. CODEOWNERS 建议

可在仓库根目录创建 `CODEOWNERS`：

```text
src/robot_lab_perception/                         @vision-member
results/perception_eval.csv                       @vision-member
docs/report/03-perception.md                      @vision-member

src/robot_lab_description/worlds/                 @navigation-member
src/robot_lab_description/urdf/lab_ur_mecanum.urdf.xacro @navigation-member
src/robot_lab_bringup/launch/lab_ur_mecanum_gz.launch.py @navigation-member
src/robot_lab_tasks/scripts/obstacle_monitor.py   @navigation-member
docs/report/04-navigation-and-scene.md            @navigation-member

src/robot_lab_description/urdf/lab_ur_gripper.urdf.xacro @arm-member
src/robot_lab_description/srdf/lab_ur_gripper.srdf.xacro @arm-member
src/robot_lab_bringup/                            @arm-member
src/robot_lab_tasks/src/                          @arm-member
results/pick_place_eval.csv                       @arm-member
docs/report/05-manipulation-control.md            @arm-member

src/robot_lab_tasks/scripts/task_orchestrator.py  @llm-member
docs/report/06-llm-semantic-planning.md           @llm-member

README.md                                         @project-lead
CONTRIBUTING.md                                   @project-lead
CODEOWNERS                                        @project-lead
docs/report/00-abstract.md                        @project-lead
docs/report/01-introduction.md                    @project-lead
docs/report/02-system-architecture.md             @project-lead
docs/report/07-experiments.md                     @project-lead
docs/report/08-conclusion.md                      @project-lead
```

使用前需要把 `@vision-member`、`@navigation-member`、`@arm-member`、`@llm-member`、`@project-lead` 替换成真实 GitHub 用户名。

## 10. 每周协作节奏

建议一周一个小周期：

```text
周初：
  项目负责人整理本周 Issue
  每个成员认领 1 到 2 个 Issue

周中：
  成员在自己的分支上修改
  遇到问题在 Issue 里更新状态

周末前：
  提交 PR
  其他成员 review
  合并通过的 PR

周末：
  项目负责人整理 README、报告目录和实验结果
```

## 11. 最小可执行版本

如果时间有限，先做这几件事：

1. 创建 `docs/report/` 报告目录。
2. 创建四个章节文件：
   - `03-perception.md`
   - `04-navigation-and-scene.md`
   - `05-manipulation-control.md`
   - `06-llm-semantic-planning.md`
3. 每个成员负责一个章节。
4. 每个章节对应一个 Issue。
5. 每个章节修改开一个 PR。
6. 项目负责人最后统一 `README.md` 和 `02-system-architecture.md`。

这样既能保证每个人有明确分工，也能在 GitHub 上留下完整协作记录。
