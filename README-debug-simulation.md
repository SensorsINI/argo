# Debugging Argo Simulation Nodes

This guide explains how to debug simulation nodes (including the lifecycle manager and helper modules such as `mock_sailboat_simulator.py`) using the new debug-friendly launch flow.

---

## Prerequisites

- `debugpy` installed in the Python environment that runs Argo nodes:
  ```bash
  pip install debugpy
  ```
- Cursor / VS Code configured with the project’s `.vscode/launch.json`.
- ROS 2 Humble environment available under `/opt/ros/humble`.

> **Heads-up:** Cursor’s integrated terminal currently triggers a debug launcher timeout ([debugpy#1677](https://github.com/microsoft/debugpy/issues/1677), [Cursor forum report](https://forum.cursor.com/t/cursor-ide-cant-launch-debugpy-debugging-configurations/137001)). The `asimmock` launch configuration therefore uses `internalConsole` and relies on attach debugging for node processes.

---

## 1. Launch the Simulation Manager with Debug Flags

1. Open the `Run and Debug` panel.
2. Choose **`asimmock`**.
3. Press **Run**.

This configuration executes:

```bash
python3 launch/argo_lifecycle_manager.py \
  simulate_local \
  --debug-nodes \
  --debug-node-port-base 5680 \
  --debug-node-wait
```

Note: Mock simulation is now the default. Use `--real` to use the real simulator instead.

### What the flags do

- `--debug-nodes`: wraps each Python node (`argo_unified_simulator_bridge.py`, etc.) with `debugpy`.
- `--debug-node-port-base`: first debug port (`5680` by default); each subsequent node increments the port.
- `--debug-node-wait`: pauses each node at startup until a debugger is attached.

While nodes start, the lifecycle manager prints lines similar to:

```
🐞 argo_unified_simulator_bridge.py waiting for debugger attach on 127.0.0.1:5680
```

Make note of the port for the node you want to inspect.

---

## 2. Attach to a Node Process

1. In the `Run and Debug` panel, choose **`Python: Attach to Argo Node`**.
2. Edit the config’s port if necessary (default is `5680`).
3. Press **Run**.

Once attached, the debugger stops immediately (because `--debug-node-wait` holds the process) and loads breakpoints for files such as `nodes/mock_sailboat_simulator.py`. Press **Continue** to let execution flow.

> Tip: Add additional attach configurations if you routinely target multiple nodes simultaneously (e.g., ports `5681`, `5682`, …).

---

## 3. Working with Breakpoints

- Set breakpoints directly in the node source file (e.g., `mock_sailboat_simulator.py`).
- When the simulation hits the breakpoint (for example inside `MockSailboatSimulator.step()`), inspect variables via the standard debugger UI.
- If execution already passed the line before you attached, press **Continue** to wait for the next invocation, or trigger the relevant behavior at runtime.

---

## 4. Troubleshooting

| Symptom | Action |
| --- | --- |
| `Timed out waiting for launcher to connect` dialog | Ensure you launched **`asimmock`** (manager) with `internalConsole` and attach using the dedicated configuration. |
| No `🐞` lines in console | Confirm you passed `--debug-nodes` and `--debug-node-wait` to the lifecycle manager. |
| Attach fails with “connection refused” | Port mismatch—check the console for the actual port or verify that the node is still waiting. |
| Breakpoints grey (unbound) | Ensure the attach session is active and the source path matches (`/home/tobi/argo`). |

---

## 5. Exiting Cleanly

1. Stop the attach debugger session(s).
2. Terminate the `asimmock` launch in the Run and Debug panel.
3. Confirm the lifecycle manager prints `✅ Lifecycle manager shutdown complete - exiting`.

You can now restart the workflow or switch back to normal (non-debug) launches.

---

Happy debugging! Let the team know if additional scenarios should be covered in this guide. 

