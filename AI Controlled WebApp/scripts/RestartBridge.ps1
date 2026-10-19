$ErrorActionPreference = "Stop"
$AppRoot = Split-Path -Parent $PSScriptRoot
$RepoRoot = Split-Path -Parent $AppRoot
$BackendRoot = Join-Path $RepoRoot "back end"
$PythonPath = "python"
$CachePath = Join-Path $RepoRoot ".backend-python.txt"
if (Test-Path -LiteralPath $CachePath) { $PythonPath = (Get-Content -LiteralPath $CachePath -Raw).Trim() }
$Listener = Get-NetTCPConnection -LocalPort 8000 -State Listen -ErrorAction SilentlyContinue | Select-Object -First 1
if ($Listener) {
    $BridgeProcessId = $Listener.OwningProcess
    $BridgeProcess = Get-CimInstance Win32_Process -Filter "ProcessId = $BridgeProcessId"
    if ($BridgeProcess.CommandLine -notlike '*uvicorn bridge_server:app*') { throw "Port 8000 belongs to another application." }
    $ReadState = @'
import asyncio, json, websockets
async def Read():
    async with websockets.connect('ws://127.0.0.1:8000/ws', open_timeout=3) as Socket:
        State = json.loads(await Socket.recv())
        if State.get('arm_connected') or State.get('torque'):
            raise RuntimeError('Disconnect the arm before restarting the bridge')
        print('disconnected')
asyncio.run(Read())
'@
    $ReadResult = $ReadState | & $PythonPath -
    if ($LASTEXITCODE -ne 0 -or $ReadResult -ne "disconnected") { throw "Bridge restart cancelled: arm is not confirmed disconnected." }
    # Confirmed disconnected, so terminating this bridge cannot leave a motor stream active.
    Stop-Process -Id $BridgeProcessId -Force
}
# The code version the running bridge must report: the same file the bridge and the page read (back end/BridgeVersion.json).
# Every older version string below also matched a bridge running day-old code (2026-10-06), so this one is required too.
$ExpectedBridgeVersion = (Get-Content -LiteralPath (Join-Path $BackendRoot "BridgeVersion.json") -Raw | ConvertFrom-Json).Version
# Start-Process empties the logs it writes to: keep the stopped bridge's instead (a few, size-capped; LogRotation.py).
try { & $PythonPath (Join-Path $BackendRoot "LogRotation.py") (Join-Path $AppRoot "api-output.log") (Join-Path $AppRoot "api-error.log") --always --cap-mb 20 --keep 3 | Out-Null } catch { }
# --no-proxy-headers: the bridge is behind no proxy, so the client address RemoteAccess.py sees is the real peer's
# (uvicorn's default rewrote a loopback client's from X-Forwarded-For).
$Bridge = Start-Process -PassThru -WindowStyle Hidden -FilePath $PythonPath `
    -ArgumentList @("-m", "uvicorn", "bridge_server:app", "--host", "0.0.0.0", "--port", "8000", "--no-proxy-headers") `
    -WorkingDirectory $BackendRoot `
    -RedirectStandardOutput (Join-Path $AppRoot "api-output.log") `
    -RedirectStandardError (Join-Path $AppRoot "api-error.log")
for ($Attempt = 0; $Attempt -lt 40; $Attempt++) {
    Start-Sleep -Milliseconds 250
    if ($Bridge.HasExited) { throw "Bridge exited. Check api-error.log." }
    try {
        $Health = Invoke-RestMethod -Uri "http://127.0.0.1:8000/health" -TimeoutSec 1
        if ($Health.bridge_version -eq $ExpectedBridgeVersion -and $Health.safety_version -eq "hold-v4" -and $Health.access_version -eq "remote-password-v1" -and $Health.protection_version -eq "auto-coast-v2-confirmed" -and $Health.calibration_version -eq "archive-v2" -and $Health.cad_version -eq "so101-urdf-v1" -and $Health.ik_solver_version -eq "all-poses-v2" -and $Health.collision_version -eq "robot-body-cad-exact-v8" -and $Health.route_planner_version -eq "flyby-cad-exact-v8" -and $Health.speed_control_version -eq "throttle-v2" -and $Health.motion_speed_maximum_multiplier -eq 25) {
            Write-Output "Updated bridge loaded. Arm remains disconnected."
            exit 0
        }
    } catch { }
}
throw "Updated bridge did not become ready."
