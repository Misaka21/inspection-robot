# Architecture Skill

This skill ensures all development follows the project architecture defined in `docs/ARCHITECTURE.md`.

## Key Rules

1. **Always check ARCHITECTURE.md before implementing new features**
2. **Follow the package layer structure**: Drivers → Control → Algo → Coordination
3. **Private members must use underscore prefix**: `_member_name`
4. **Use relative topic names**: `~/topic` instead of absolute paths

## Package Structure

- 驱动层: `*_driver` (agv_driver, arm_driver, hikvision_driver)
- 控制层: `*_controller` (arm_controller)
- 算法层: `*_detector`, `*_planner` (pose_detector, path_planner, defect_detector)
- 协调层: `task_coordinator`
- 基础设施: `inspection_*`

## Code Organization

- Headers: `include/<package>/`
- Source: `src/`
- Config: `config/`
- Launch: `launch/`

## Implementation Checklist

When implementing new features:
- [ ] Check docs/ARCHITECTURE.md first
- [ ] Follow layer hierarchy
- [ ] Use underscore prefix for private members
- [ ] Add unit tests for new logic
