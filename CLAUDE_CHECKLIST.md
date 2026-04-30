# Claude Readiness Checklist

Use this before exporting the repo or uploading to a Claude project.

- Run CLEANUP.sh to remove build/install/log and Python caches.
- Verify no ZED recordings or model files are tracked (see .gitignore).
- Ensure runtime docs are up to date: RUNTIME_COMMANDS.md and src/zed_aruco/README.md.
- Confirm motion defaults are safe (motion_enabled=false, publish_on_action=false).
