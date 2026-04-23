---
name: commit
description: Git commit with gitmoji convention. Use when committing code changes.
allowed-tools: Bash
---

# Git Commit 规范

## 基本格式

```
<emoji> <type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

## Emoji + Type 对照表

| Emoji | Type | 说明 |
|-------|------|------|
| ✨ | `feat` | 新增功能 |
| 🐛 | `fix` | 修复 bug |
| 📝 | `docs` | 文档更新 |
| 💄 | `style` | 代码格式调整(不影响功能) |
| ♻️ | `refactor` | 代码重构(不增加功能,不修复bug) |
| ⚡️ | `perf` | 性能优化 |
| ✅ | `test` | 测试相关 |
| 🔧 | `chore` | 构建/工具/依赖更新 |
| 🔨 | `build` | 构建系统修改 |
| 👷 | `ci` | CI/CD 配置修改 |
| 💥 | `BREAKING CHANGE` | 破坏性变更(使用感叹号!) |

## 规则

1. **类型(必填)**: 使用上述 type 之一
2. **范围(可选)**: 用圆括号标注影响范围,如 `(api)` `(user)`
3. **描述(必填)**: 简短说明变更内容,建议不超过50字
4. **破坏性变更**: 在类型后加 `!` 或在 footer 中使用 `BREAKING CHANGE:`

## 执行流程

1. 运行 `git status` 查看变更文件
2. 运行 `git diff --staged` 查看暂存的变更内容（如无暂存则查看 `git diff`）
3. 分析变更，确定合适的 type 和 scope
4. 生成符合规范的 commit message
5. 执行 `git add` 添加相关文件（按需）
6. 执行 `git commit`

## 禁止事项

- 禁止添加 "🤖 Generated with [Codex]" 标记
- 禁止添加 "Co-Authored-By: Codex" 签名
- 禁止添加任何 AI 工具生成的标记
- 只包含人为编写的提交内容

## 示例

```bash
# 基础功能
git commit -m "✨ feat: 增加用户搜索功能"

# 带范围
git commit -m "🐛 fix(auth): 修复登录超时问题"

# 多行（使用 heredoc）
git commit -m "$(cat <<'EOF'
✨ feat(payment): 新增支付宝支付方式

功能详情:
1. 集成支付宝 SDK
2. 实现扫码支付流程

Closes: #234
EOF
)"
```
