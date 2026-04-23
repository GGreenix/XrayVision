$ErrorActionPreference = "Stop"

$env:XRAY_MODE = "sim"
$env:XRAY_SIM_BACKEND = "gazebo"
$env:XRAY_USE_FAKE_POSE = "false"
$env:XRAY_USE_FAKE_DETECTIONS = "true"
$env:XRAY_USE_SIM_TIME = "true"
$env:BUILDKIT_PROGRESS = "plain"
$env:COMPOSE_DOCKER_CLI_BUILD = "1"

Write-Host "=== XrayVision: starting Gazebo sim stack (headless) ===" -ForegroundColor Cyan
Write-Host "First build can take 10-15 min. Logs below." -ForegroundColor Cyan

docker compose --profile sim up --build sim_backend ground unity_bridge
