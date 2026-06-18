# F06 — Localization Convergence Status

Feature file name: `F06-localization-status.md`

**Priority**: Medium
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: `nav_manager_node` subscribes to `/amcl_pose`, checks covariance, and
publishes two topics:
- `/dome_nav/localization_status` (`String`): `"converged"` or `"localizing"`
- `/dome_nav/localization_score` (`Float32`): continuous 0.0–1.0

Pure logic lives in `NavManager.check_localization(covariance) -> tuple[str, float]`
(testable without ROS).

## Score formula

```python
score = max(0.0, 1.0 - max(covariance[0], covariance[7]) / MAX_COV)
```

`MAX_COV = 1.0` (constant in `NavManager`). Score is continuous — any float in [0.0, 1.0].

Status threshold: score >= 0.9 → `"converged"`, else `"localizing"`.

Notable values:
- cov = 0.0 → score = 1.0 (perfect)
- cov = 0.1 → score = 0.9 (converged threshold)
- cov = 0.5 → score = 0.5
- cov >= 1.0 → score = 0.0 (clamped, lost)

## Publish behavior

Event-driven: publishes both topics on every `/amcl_pose` callback. No separate timer.

## How to Demo

**Setup**: `bl dome_nav robot_nav.launch.py` running, AMCL active.

**Steps**:
1. `ros2 topic echo /dome_nav/localization_status`
2. `ros2 topic echo /dome_nav/localization_score`
3. Before AMCL converges: status = `"localizing"`, score near 0.0
4. Robot moves, AMCL particle cloud tightens
5. After convergence: status = `"converged"`, score near 1.0

**Expected output**: score rises continuously as AMCL converges. Status flips at 0.9.
