# F06 — Localization Convergence Status

Feature file name: `F06-localization-status.md`

**Priority**: Medium
**Done:** no
**Tasks File Created:** no
**Tests Written:** no
**Test Passing:** no
**Description**: `nav_manager_node` subscribes to `/amcl_pose`, checks covariance, and
publishes `/dome_nav/localization_status` with value `"converged"` or `"localizing"`.
Pure logic lives in `NavManager.check_localization(covariance) -> str` (testable without
ROS). Threshold: covariance[0] < 0.1 and covariance[7] < 0.1 = converged.

## How to Demo

**Setup**: `bl dome_nav robot_nav.launch.py` running, AMCL active.

**Steps**:
1. `ros2 topic echo /dome_nav/localization_status`
2. Before giving AMCL an initial pose: should show `"localizing"`
3. Use RViz 2D Pose Estimate or `reinitialize_global_localization` + robot moves
4. After AMCL converges: should show `"converged"`

**Expected output**: status transitions `localizing` → `converged` as particle cloud tightens.
