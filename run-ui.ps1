# Launches the backend bridge and the AI Controlled Arm web app together.
# Usage: .\run-ui.ps1

$root = $PSScriptRoot
$appRoot = Join-Path $root "AI Controlled WebApp"

Push-Location $appRoot
try {
    npm run dev
} finally {
    Pop-Location
}
