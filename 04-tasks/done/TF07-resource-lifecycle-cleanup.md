# TF07 — Resource Lifecycle and Cleanup (Feature F07)

## T01 — fix temp-file leak in utils.py (I11)
**Status**: done
**Description**: Replace `_write_temp` (NamedTemporaryFile delete=False, leaks) with
`write_config(data)` that writes into `~/.dome/launch_cache/<sha1(content)>.yaml`.
Identical merged configs map to one stable file (reused, no growth); distinct configs
get distinct names. Bounded by number of distinct configs. Update `yaml_override` and
`yaml_patch_dict` to call it. Drop `import tempfile`, add `import hashlib`.
**Test**: pure unit tests in `test/test_utils_pure.py` — same dict → same path; different
dict → different path; file content round-trips through yaml; cache dir created.

## T02 — convert SlamManagerNode to LifecycleNode (I01)
**Status**: done
**Description**: `SlamManagerNode(LifecycleNode)`. on_configure: create subscription,
lifecycle publisher, service client, manager. on_activate: start 30s save timer.
on_deactivate: destroy timer. on_cleanup: destroy entities. on_shutdown: synchronous
final save via `save_map_sync()` using `spin_until_future_complete`. main() drives
trigger_configure → trigger_activate → spin → (finally) trigger_shutdown → destroy.
Pure `SlamManager` logic unchanged.
**Test**: update `test/test_slam_manager.py` fixture to drive transitions; add test that
on_shutdown calls the synchronous save when map_ready.

## T03 — convert NavManagerNode to LifecycleNode
**Status**: deferred
**Description**: nav_manager has no shutdown-critical work; Mode B was live-tested and
working. Converting adds activation-wiring risk for no concrete bug payoff. Deferred
until lifecycle coordination with the Nav2 lifecycle_manager is actually needed.
**Test**: n/a (deferred).

## T04 — manual shutdown verify (I01)
**Status**: done
**Description**: Launch Mode A, build partial map, Ctrl-C, confirm
`~/.dome/slam_maps/basement1.posegraph` and `.data` have fresh mtime and the
"Pose graph saved" log appeared.
**Test**: manual — cannot automate without slam_toolbox live.
