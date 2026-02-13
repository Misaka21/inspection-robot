# Architecture Skill

This skill ensures all development follows the project architecture defined in `docs/ARCHITECTURE.md`.

## Key Rules

1. **Always check docs/ARCHITECTURE.md before implementing new features**
2. **Follow the package layer structure**:
   - 驱动层 (Drivers): `*_driver` - agv_driver, arm_driver, hikvision_driver
   - 控制层 (Control): `*_controller` - arm_controller
   - 算法层 (Algo): `*_detector`, `*_planner` - pose_detector, path_planner, defect_detector
   - 协调层 (Coordination): task_coordinator
   - 基础设施: `inspection_*`
3. **Private members must use underscore prefix**: `_member_name`
4. **Use relative topic names**: `~/topic` instead of absolute paths
5. **Namespace**: `/inspection/*`

## ROS Interface Standards

- All topics under `/inspection/*` namespace
- Use relative topics `~/topic` inside nodes
- Follow message definitions in `inspection_interface`

## Package Structure

```
<package_name>/
├── include/<package>/     # Headers
├── src/                   # Source
├── config/                # YAML parameters
├── launch/                # Launch files
└── package.xml
```

## Implementation Checklist

When implementing new features:
- [ ] Check docs/ARCHITECTURE.md first
- [ ] Follow layer hierarchy
- [ ] Use underscore prefix for private members
- [ ] Add unit tests for new logic
- [ ] Update docs/ARCHITECTURE.md if interface changes
