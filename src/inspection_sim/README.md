# inspection_sim（历史/可选）

当前课题方向已改回**大型工件视觉检测/巡检**，且当前论文和演示不依赖 fake driver。因此本包不作为主线说明。

保留本包的原因：

- 避免破坏历史代码和构建结构。
- 后续若确实需要离线联调，可重新评估是否恢复。

当前建议：

- 论文实验优先使用真机数据、半实物数据或离线图像/深度回放。
- 不要把本包作为 `task_coordinator` 的主流程依赖。
- 不要在本包中实现真实缺陷检测或深度微调算法。

如需恢复无硬件联调能力，先更新：

- `TODO.md`
- `docs/WORKSPACE_OVERVIEW.md`
- `docs/ARCHITECTURE.md`
- `src/inspection_sim/CLAUDE.md`

