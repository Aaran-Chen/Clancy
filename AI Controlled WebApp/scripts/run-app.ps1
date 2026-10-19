$ErrorActionPreference = "Stop"

# This script lives in "AI Controlled WebApp\scripts", so the app root is one
# level up and the repo root is two levels up.
$appRoot = Split-Path -Parent $PSScriptRoot
$repoRoot = Split-Path -Parent $appRoot
$backendRoot = Join-Path $repoRoot "back end"
# Remembers the interpreter that actually had the backend packages, so later
# launches skip probing every candidate. Delete it to force a fresh search.
$pythonCache = Join-Path $repoRoot ".backend-python.txt"
$apiUrl = "http://127.0.0.1:8000"
$webUrl = "http://127.0.0.1:5181"
$apiOut = Join-Path $appRoot "api-output.log"
$apiErr = Join-Path $appRoot "api-error.log"
$webOut = Join-Path $appRoot "vite-output.log"
$webErr = Join-Path $appRoot "vite-error.log"

$startedApi = $null
$startedWeb = $null

function Test-HttpOk($url) {
    try {
        $response = Invoke-WebRequest -UseBasicParsing -Uri $url -TimeoutSec 2
        return [int]$response.StatusCode -ge 200 -and [int]$response.StatusCode -lt 500
    } catch {
        return $false
    }
}

function Test-BridgeOk {
    try {
        $health = Invoke-RestMethod -Uri "$apiUrl/health" -TimeoutSec 2
        return $health.service -eq "so101-backend-bridge"
    } catch {
        return $false
    }
}

function Stop-StartedProcess($process) {
    if ($null -eq $process) {
        return
    }
    try {
        Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
    } catch {
    }
}

function Test-BackendPython {
    param(
        [string]$FilePath,
        [string[]]$Arguments
    )

    & $FilePath @Arguments -c "import fastapi, uvicorn, serial, scservo_sdk, numpy, cv2, scipy" *> $null
    return $LASTEXITCODE -eq 0
}

function Resolve-BackendPython {
    # Cached hit first — probing costs a full interpreter start plus a fastapi
    # import per candidate, which dominated launch time.
    if (Test-Path -Path $pythonCache) {
        $cached = (Get-Content -Path $pythonCache -First 1).Trim()
        if ($cached -and (Test-Path -Path $cached) -and (Test-BackendPython $cached @())) {
            return [pscustomobject]@{ Label = "cached ($cached)"; FilePath = $cached; Arguments = @() }
        }
        Remove-Item -Path $pythonCache -Force -ErrorAction SilentlyContinue
    }

    $candidates = @()

    # "python" on PATH is checked first: it is the interpreter pip installs into
    # for a normal shell, so it is the one most likely to carry the packages.
    $pathPython = Get-Command python -ErrorAction SilentlyContinue
    if ($pathPython) {
        $candidates += [pscustomobject]@{
            Label = "python on PATH"
            FilePath = $pathPython.Source
            Arguments = @()
        }
    }

    # Then any 64-bit CPython the launcher knows about, newest first. Versions
    # are discovered rather than hardcoded so this keeps working as Python is
    # upgraded, and 32-bit builds are skipped (numpy/opencv wheels want 64-bit).
    $pyLauncher = Get-Command py -ErrorAction SilentlyContinue
    if ($pyLauncher) {
        $found = @()
        try {
            $found = & $pyLauncher.Source -0p 2>$null | ForEach-Object {
                if ($_ -match '^\s*-V:(\S+)\s+\*?\s*(.+)$') {
                    $tag = $Matches[1]
                    $exe = $Matches[2].Trim()
                    # Skip 32-bit and ARM builds — the backend needs numpy and
                    # opencv wheels, which are published for 64-bit x86.
                    if ($tag -notmatch '-(32|arm)' -and (Test-Path -Path $exe)) {
                        [pscustomobject]@{
                            Version = [version]($tag -replace '[^\d.].*$', '')
                            Exe = $exe
                        }
                    }
                }
            }
        } catch {
        }
        foreach ($entry in ($found | Sort-Object -Property Version -Descending)) {
            $candidates += [pscustomobject]@{
                Label = "$($entry.Exe)"
                FilePath = $entry.Exe
                Arguments = @()
            }
        }
    }

    foreach ($candidate in $candidates) {
        if (Test-BackendPython $candidate.FilePath $candidate.Arguments) {
            if ($candidate.Arguments.Count -eq 0) {
                Set-Content -Path $pythonCache -Value $candidate.FilePath -Encoding utf8
            }
            return $candidate
        }
    }

    throw "Could not find a Python installation with the backend packages. Try: python -m pip install -r `"$backendRoot\requirements.txt`""
}

Write-Host "Starting AI Controlled Arm..." -ForegroundColor Cyan

try {
    $python = Resolve-BackendPython
    Write-Host "Using backend Python: $($python.Label)" -ForegroundColor DarkCyan

    if (Test-BridgeOk) {
        Write-Host "Python bridge already running at $apiUrl" -ForegroundColor DarkCyan
    } else {
        $portInUse = Get-NetTCPConnection -LocalPort 8000 -ErrorAction SilentlyContinue
        if ($portInUse) {
            throw "Port 8000 is already in use, but it is not responding as the SO-101 bridge. Stop that process and run this again."
        }

        Write-Host "Starting Python bridge at $apiUrl ..." -ForegroundColor Cyan
        # Start-Process empties the logs it writes to: keep the last runs' instead (a few, size-capped; LogRotation.py).
        try { & $python.FilePath @($python.Arguments + @((Join-Path $backendRoot "LogRotation.py"), $apiOut, $apiErr, "--always", "--cap-mb", "20", "--keep", "3")) | Out-Null } catch { }
        $env:PYTHONPATH = $backendRoot
        $startedApi = Start-Process -PassThru -WindowStyle Hidden `
            -FilePath $python.FilePath `
            -ArgumentList @($python.Arguments + @("-m", "uvicorn", "bridge_server:app", "--host", "0.0.0.0", "--port", "8000", "--no-proxy-headers")) `
            -WorkingDirectory $backendRoot `
            -RedirectStandardOutput $apiOut `
            -RedirectStandardError $apiErr

        # Polled finely: the bridge is ready ~0.4s in, so a coarse interval
        # spends most of its time waiting on an already-live server.
        $ready = $false
        for ($i = 0; $i -lt 75; $i++) {
            Start-Sleep -Milliseconds 100
            if (Test-BridgeOk) {
                $ready = $true
                break
            }
        }
        if (!$ready) {
            Write-Host "Python bridge did not become ready. Last error log:" -ForegroundColor Yellow
            Get-Content -Path $apiErr -ErrorAction SilentlyContinue
            throw "Python bridge failed to start."
        }
    }

    if (Test-HttpOk $webUrl) {
        Write-Host "Web app already running at $webUrl" -ForegroundColor DarkCyan
    } else {
        Write-Host "Starting web app at $webUrl ..." -ForegroundColor Cyan
        $startedWeb = Start-Process -PassThru -WindowStyle Hidden `
            -FilePath "npm.cmd" `
            -ArgumentList @("run", "dev:web", "--", "--port", "5181") `
            -WorkingDirectory $appRoot `
            -RedirectStandardOutput $webOut `
            -RedirectStandardError $webErr

        $ready = $false
        for ($i = 0; $i -lt 75; $i++) {
            Start-Sleep -Milliseconds 100
            if (Test-HttpOk $webUrl) {
                $ready = $true
                break
            }
        }
        if (!$ready) {
            Write-Host "Web app did not become ready. Last error log:" -ForegroundColor Yellow
            Get-Content -Path $webErr -ErrorAction SilentlyContinue
            throw "Web app failed to start."
        }
    }

    Write-Host ""
    Write-Host "Open $webUrl" -ForegroundColor Green
    Write-Host "The Python bridge is running at $apiUrl" -ForegroundColor Green
    Write-Host "Press Ctrl+C in this terminal to stop processes started by this launcher." -ForegroundColor Yellow
    Start-Process $webUrl | Out-Null

    while ($true) {
        Start-Sleep -Seconds 1
        if ($startedApi -and $startedApi.HasExited) {
            throw "Python bridge exited."
        }
        if ($startedWeb -and $startedWeb.HasExited) {
            throw "Web app exited."
        }
    }
} finally {
    Stop-StartedProcess $startedWeb
    if ($startedApi -and !$startedApi.HasExited) {
        try {
            $StopResult = Invoke-RestMethod -Method Post -Uri "$apiUrl/emergency-stop" -TimeoutSec 10
            if (!$StopResult.ok) { throw $StopResult.error }
            Stop-StartedProcess $startedApi
        } catch {
            Write-Warning "Bridge left running because motor torque-off could not be confirmed: $_"
        }
    }
}
