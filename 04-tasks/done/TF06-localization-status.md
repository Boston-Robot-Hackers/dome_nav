# TF06 — Localization Convergence Status (Feature F06)

## T01 — add check_localization to NavManager
**Status**: done
**Description**: Add `check_localization(covariance: list[float]) -> tuple[str, float]`
to `dome_nav/nav_manager.py`. Formula: `score = max(0.0, 1.0 - max(cov[0], cov[7]) / MAX_COV)`
where `MAX_COV = 1.0` (class constant). Status: score >= 0.9 → `"converged"`, else `"localizing"`.
**Test**: unit tests in `test/test_nav_manager_pure.py` — see T02.

## T02 — write pure unit tests for check_localization
**Status**: done
**Description**: Add tests to `test/test_nav_manager_pure.py`. Cover:
- cov[0]=0.0, cov[7]=0.0 → score=1.0, status="converged"
- cov[0]=0.1, cov[7]=0.05 → score=0.9, status="converged"
- cov[0]=0.5, cov[7]=0.3 → score=0.5, status="localizing"
- cov[0]=1.0, cov[7]=1.0 → score=0.0, status="localizing"
- cov[0]=2.0, cov[7]=2.0 → score=0.0 (clamped), status="localizing"
- max of cov[0] vs cov[7] used (not average): cov[0]=0.05, cov[7]=0.8 → score=0.2
**Test**: `python3 -m pytest test/test_nav_manager_pure.py -v` — no rclpy, runs in <1s.

## T03 — subscribe /amcl_pose and publish both topics in nav_manager_node
**Status**: done
**Description**: In `dome_nav/nav_manager_node.py`:
- Add subscriber: `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`)
- Add publishers: `/dome_nav/localization_status` (`std_msgs/String`),
  `/dome_nav/localization_score` (`std_msgs/Float32`)
- Callback: extract `msg.pose.covariance` (flat 36-element list), call
  `self.nav_manager.check_localization(covariance)`, publish both results.
**Test**: `colcon build --packages-select dome_nav` passes. Topics visible in `ros2 topic list`.

## T05 — clamp localization score to [0.0, 1.0] (I07)
**Status**: done — already implemented (min(1.0, max(0.0, ...))). Test test_check_localization_negative_cov_clamped_to_1 passes.
**Description**: check_localization formula max(0.0, 1.0 - worst/MAX_COV) only clamps the low end. Negative covariance (theoretically impossible from AMCL but unguarded) produces score > 1.0. Change to min(1.0, max(0.0, ...)).
**Test**: add test to test_nav_manager_pure.py: negative covariance input → score clamped to 1.0.

## T06 — add file headers to test files (I08)
**Status**: done — all 4 test files have correct headers.
**Description**: test_nav_manager_pure.py and test_slam_manager_pure.py missing required header (module name, description, Author, MIT license). Add standard header block to both.
**Test**: visual inspection.

## T04 — manual smoke test
**Status**: done — /dome_nav/localization_status publishing "converged", score ≥ 0.9 on live robot
**Description**: With `bl dome_nav robot_nav.launch.py` running and AMCL active:
1. `ros2 topic echo /dome_nav/localization_status` — should show `"localizing"` initially
2. `ros2 topic echo /dome_nav/localization_score` — should show score near 0.0 initially
3. Move robot (or give 2D Pose Estimate in RViz), watch score rise toward 1.0
4. Verify status flips to `"converged"` when score crosses 0.9
**Test**: manual observation. Cannot be automated without full robot stack.
