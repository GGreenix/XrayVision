$ErrorActionPreference = "Stop"

# WSLg sockets only exist inside WSL, so we shell through WSL to run the bash
# runner. The Gazebo window still appears on the Windows desktop via WSLg.

$repoRoot = Split-Path -Parent $PSScriptRoot
$wslPath = (wsl wslpath -a "$repoRoot").Trim()

if (-not $wslPath) {
    Write-Error "Could not translate '$repoRoot' to a WSL path. Is WSL installed?"
    exit 1
}

wsl bash -c "cd '$wslPath' && ./scripts/run-gazebo-gui.sh"
