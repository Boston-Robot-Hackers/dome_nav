---
version: "2.0"
generated: "2026-06-17"
---

# SlamManagerNode — ROS2 Wrapper for SLAM Persistence

## Overview

`slam_manager_node.py` is a thin ROS2 adapter around `SlamManager` (see
`04-slam_manager.md`). It wires ROS subscriptions, publishers, a timer, and
the slam_toolbox service call to the pure-Python state machine. No SLAM logic
lives here — only I/O.

## Delegation Pattern

The node holds `self._manager: SlamManager` and delegates all decisions to it.
Properties on the node forward to the manager so test code can inspect state
through the node's public interface without reaching into `_manager`.

```python
class SlamManagerNode(Node):
    def __init__(self):
        ...
        self._manager = SlamManager(_path)

    @property
    def map_persist_path(self) -> str:
        return self._manager.map_persist_path

    @map_persist_path.setter
    def map_persist_path(self, value: str):
        self._manager.map_persist_path = value
```

## Map Subscription

Every `/map` message is forwarded to the manager. The node logs the first
reception but stays silent on subsequent messages to avoid log spam.

```python
    def on_map(self, msg: OccupancyGrid):
        was_ready = self._manager.map_ready
        status_str = self._manager.on_map_received()
        if not was_ready:
            self.get_logger().info("Map received — slam_toolbox is mapping.")
        status = String()
        status.data = status_str
        self.status_pub.publish(status)
```

## Periodic Save

A 30-second timer drives map persistence. The manager gates saving — before
the first `/map` message, `should_save()` returns `False` and no service call
is made.

```python
    def periodic_save(self):
        if self._manager.should_save():
            self.save_map()
```

The 30s interval exists because SIGINT kills slam_toolbox before slam_manager,
making shutdown-triggered saves unreliable.

## Async Service Call

`save_map` calls slam_toolbox's `SerializePoseGraph` service asynchronously
to avoid blocking the spin loop.

```python
    def save_map(self):
        self._manager.ensure_map_dir()
        if not self.serialize_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning("serialize_map service not available.")
            return False
        req = SerializePoseGraph.Request()
        req.filename = self.map_persist_path
        future = self.serialize_client.call_async(req)
        future.add_done_callback(self._on_save_done)
        return True
```

`ensure_map_dir()` creates the parent directory before the service call so
slam_toolbox never fails on a missing path.

## Observations

- `save_map` returns `bool` but callers (timer, shutdown) ignore it. Either
  make it `None` or have callers act on the return value.
- Shutdown save in `main()` calls `save_map()` but the async callback may not
  fire before `destroy_node()`. A spin-until-future pattern would make shutdown
  saves reliable.
