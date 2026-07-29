---
name: restore-runtime-after-tests
description: Use in /home/fuzzrobo/uraki_ws whenever running Docker, ROS 2 launch/node/topic/service, simulator, dev-server, or other runtime tests. Snapshot the relevant pre-test runtime state, bound and track spawned work, clean it up on every exit path, and verify that no test process, ROS node, container, daemon, or listener remains unless the user explicitly asks to keep it running.
---

# Restore Runtime After Tests

Treat restoration of the pre-test runtime state as part of test completion, not as optional cleanup.

## Required Workflow

1. Record the relevant baseline before starting:
   - target container existence and running state
   - relevant host and container processes, including PID, PPID, and command
   - relevant ROS nodes or ports only when the test can affect them
   - whether the ROS 2 daemon was already running
2. Run the test with a finite timeout and retain ownership information:
   - use a unique marker, PID, process group, container ID, or exec session
   - prefer graceful `SIGINT`, followed by bounded `SIGTERM` and `SIGKILL` only if required
   - wait for every tool execution session to finish or explicitly terminate it
3. Execute cleanup after success, failure, timeout, or interruption:
   - stop only the process group and descendants created by the test
   - stop and remove only temporary containers created by the test
   - restore a pre-existing stopped container if the test started it
   - stop the ROS 2 daemon if the test caused it to start and it was absent before
   - remove only temporary files, sockets, and listeners created by the test
4. Compare the post-test state with the baseline:
   - no new test process or descendant remains
   - container running states match
   - relevant ROS nodes and listeners match after discovery has settled
5. Report the cleanup result. Do not claim the test is complete when restoration was not verified.

## Safety Rules

- Never use broad cleanup such as unscoped `pkill -f`, `killall`, or stopping all ROS nodes.
- Never terminate or restart a process or container that existed before the test merely because its name matches.
- Do not leave a launch process running to preserve logs; capture its output and clean it up.
- A failed test still requires the same cleanup and verification.
- If exact restoration is impossible, state what remains and continue cleanup until resolved or user action is required.

## Explicit Exception

Keep a process running only when the user explicitly asks for a persistent server, launch, or interactive test. State what was intentionally left running and how it differs from the baseline.
