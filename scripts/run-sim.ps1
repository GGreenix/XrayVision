$env:XRAY_MODE = "sim"
$env:XRAY_SIM_BACKEND = "mock"
$env:XRAY_USE_FAKE_POSE = "true"
$env:XRAY_USE_FAKE_DETECTIONS = "true"
$env:XRAY_USE_SIM_TIME = "false"
docker compose --profile sim up --build sim_backend ground unity_bridge video_gateway
