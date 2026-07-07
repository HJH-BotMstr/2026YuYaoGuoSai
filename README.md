# 2026 中国高校智能机器人创意大赛（四足大型组）—— 余姚国赛

本项目为参赛用 ROS2 工作区，目标是在绝影 Lite3 四足机器人平台上完成感知、运动控制与任务执行。

## 目录结构

- `lite3_ws/` — 未来 ROS2 工作区，内部 `src/` 目前为空，待后续放入参赛包代码。
- `assets/old_code/` — 往届/旧代码资产，包含参考实现、子模块和辅助资料。
  - `DeepRobotDog/` 为 Git 子模块（来源：[WanliZhong/DeepRobotDog](https://github.com/WanliZhong/DeepRobotDog)）。
- `doc/` — 赛事与机器人平台文档（PDF / Excel），仅作参考，不纳入代码迭代。
- `TODO.md` — 后续工作计划，当前为占位文件，未来完善。
- `.gitignore` — 已配置 ROS2 / Python / IDE / Obsidian 等常见忽略规则。

## 当前状态

- 仓库已初始化，完成首次提交。
- 由于当前在 Windows 环境下，尚未安装 `colcon`，暂不做构建。
- 后续将在 `lite3_ws/src/` 中逐步添加功能包并完善 `README.md` 与 `TODO.md`。

## 如何贡献

本仓库在早期阶段采用 Pull Request 工作流。请所有合作者先阅读 [CONTRIBUTING.md](CONTRIBUTING.md)，了解分支命名、提交规范与 PR 流程。
