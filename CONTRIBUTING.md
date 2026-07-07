# 贡献指南

欢迎参与 2026 中国高校智能机器人创意大赛（四足大型组）项目！本仓库在早期阶段采用 **Pull Request（PR）** 工作流，以便核心维护者统一把控代码质量与版本节奏。

## 前置要求

- 熟悉 Git 基本操作（clone、branch、commit、push）。
- 已安装并配置好 Git。
- 拥有本仓库的写权限（由仓库所有者邀请）。

## 工作流概览

所有功能开发、bug 修复、文档更新都应通过 **feature 分支 + Pull Request** 提交到 `master`。

```bash
# 1. 切换到 master 并拉取最新代码
git checkout master
git pull origin master

# 2. 创建功能分支，命名规范见下方
git checkout -b feature/<你的名字>/<功能简述>

# 3. 进行开发并提交改动
git add .
git commit -m "type: 简述本次改动"

# 4. 推送到远程
git push -u origin feature/<你的名字>/<功能简述>

# 5. 在 GitHub 上发起 Pull Request，目标分支选择 master
# 6. 等待至少 1 位维护者 review 并批准后合并
```

## 分支命名规范

| 类型 | 命名示例 | 说明 |
|---|---|---|
| 功能开发 | `feature/alice/vision-node` | 新增功能或模块 |
| Bug 修复 | `fix/bob/motion-crash` | 修复已有问题 |
| 文档更新 | `docs/carol/readme-update` | 仅修改文档 |
| 配置/工具 | `config/dave/gitignore` | 构建、CI、配置等 |

## 提交信息规范

提交信息使用中文或英文均可，建议格式：

```
<type>: <简短描述>

<可选的详细说明>
```

常见 `type`：

- `feat`：新增功能
- `fix`：修复 bug
- `docs`：文档更新
- `style`：代码格式调整（不影响功能）
- `refactor`：重构
- `test`：测试相关
- `chore`：构建/工具/配置

## 代码审查（Review）

- 发起 PR 时，请填写清晰的标题和描述，说明改动目的、影响范围和测试结果。
- 维护者会在 PR 中提出修改意见，请及时响应。
- 在获得至少 1 个 approval 之前，请勿自行合并。

## 注意事项

- 不要直接向 `master` 推送代码（仓库已开启分支保护）。
- 提交前请确认没有将 `build/`、`install/`、`log/`、`__pycache__/` 等构建产物加入版本控制。
- 大文件（视频、数据集、大型模型等）请勿直接提交到仓库，请联系维护者协商存放方案。

## 遇到问题？

如有任何疑问，请在 PR 描述中说明，或通过团队沟通渠道联系维护者。
