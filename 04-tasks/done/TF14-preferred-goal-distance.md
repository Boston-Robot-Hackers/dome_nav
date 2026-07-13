# TF14 — Preferred Goal Distance for F14

## T01 — Add preferred_goal_distance to ExploreParams; deprecate prefer_farthest
**Status**: done

## T02 — Replace prefer_farthest in pick_best_frontier with preferred_dist
**Status**: done
**Description**: Selection criterion is now `min |d - preferred_dist|`. preferred_dist=0.0 → nearest-first. preferred_dist=large → farthest-first.

## T03 — Wire preferred_dist through frontier_algorithm.py
**Status**: done

## T04 — Add preferred_goal_distance ROS param; deprecate prefer_farthest in node
**Status**: done
**Description**: prefer_farthest kept as deprecated alias. If True, maps to preferred_goal_distance = max_frontier_dist (or 1000.0 if unlimited) and logs a warning.

## T05 — Update launch files
**Status**: done
**Description**: sim default 2.0 m, real default 1.0 m. prefer_farthest arg removed.

## T06 — Tests
**Status**: done

## T07 — Update feature file and current.md
**Status**: done
