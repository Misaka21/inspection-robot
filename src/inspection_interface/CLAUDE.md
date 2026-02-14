# inspection_interface/CLAUDE.md

本包是机器人端的“内部契约层”：所有跨包通信都应优先使用本包定义的 msg/srv。

目标：
- 避免各包各自定义同义字段导致漂移
- 让 `inspection_gateway` 有稳定的 ROS2 接口可调用/订阅

## 1. 边界与优先级

优先级（从高到低）：
1. `inspection-api/proto/inspection_gateway.proto`（对外契约）
2. `inspection_interface`（对内 ROS2 契约）
3. 厂商协议（只允许出现在各 driver 内）

原则：
- `inspection_interface` 应尽量能无损映射到 gRPC（至少语义一致）

## 2. 演进规则（必须遵守）

1. 只追加字段，不复用字段号
2. 不在 README/文档里复制字段表（以 `.msg/.srv` 为准）
3. 修改字段语义必须同步更新：
   - `inspection-api` proto（若对外语义变化）
   - `docs/WORKSPACE_OVERVIEW.md`（端到端约定）
   - `docs/IMPLEMENTATION_STATUS.md`（落地缺口）

## 3. 推荐组织方式

当 msg/srv 增长后，建议按域拆分命名：
- `AgvStatus/ArmStatus/SystemState`
- `Plan*` / `Task*` / `Capture*` / `NavMap*`

并保持“对外 gRPC 语义优先”的命名。

