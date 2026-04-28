$env:XRAY_MODE = "real"
$env:XRAY_SIM_BACKEND = "mock"
$env:XRAY_USE_FAKE_POSE = "false"
$env:XRAY_USE_FAKE_DETECTIONS = "false"
$env:XRAY_USE_SIM_TIME = "false"
docker compose --profile real up --build drone ground unity_bridge video_gateway
