# Build, sync, and reload the X-Plane NXi Stream Deck plugin.
# Restarts Stream Deck so its Node plugin host picks up changes.
# Does NOT install or reinstall the Stream Deck application.
$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$PluginRoot = Join-Path $RepoRoot "streamdeck-plugin"
$Source = Join-Path $PluginRoot "com.andy.xplane-nxi.sdPlugin"
$Dest = Join-Path $env:APPDATA "Elgato\StreamDeck\Plugins\com.andy.xplane-nxi.sdPlugin"
$TutorialLibs = Join-Path $env:APPDATA "Elgato\StreamDeck\Plugins\com.elgato.tutorial.sdPlugin\libs"

function Restart-StreamDeckPluginHost {
    Get-Process -Name "StreamDeck" -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep -Seconds 2

    $candidates = @()
    $installDir = (Get-ItemProperty "HKCU:\Software\Elgato Systems GmbH\StreamDeck" -ErrorAction SilentlyContinue).InstallDir
    if ($installDir) {
        $candidates += (Join-Path $installDir "StreamDeck.exe")
    }
    $candidates += @(
        "C:\Program Files\Elgato\StreamDeck\StreamDeck.exe",
        "C:\Program Files (x86)\Elgato\StreamDeck\StreamDeck.exe",
        (Join-Path $env:LOCALAPPDATA "Programs\Elgato\StreamDeck\StreamDeck.exe")
    )

    foreach ($exe in $candidates | Select-Object -Unique) {
        if (-not $exe) { continue }
        if (Test-Path $exe) {
            Start-Process -FilePath $exe
            Write-Host "Restarted Stream Deck plugin host: $exe"
            return
        }
    }

    Write-Warning "Stream Deck is not running and StreamDeck.exe was not found. Open Stream Deck manually to load the updated plugin."
}

if (-not (Test-Path $Source)) {
    throw "Plugin source not found: $Source"
}
if (-not (Test-Path $TutorialLibs)) {
    throw "Stream Deck tutorial libs not found: $TutorialLibs"
}

Write-Host "Building plugin..."
Push-Location $PluginRoot
try {
    npm run build
} finally {
    Pop-Location
}

New-Item -ItemType Directory -Force -Path (Join-Path $Source "libs") | Out-Null
if (Test-Path $Dest) {
    Remove-Item $Dest -Recurse -Force
}
Copy-Item $Source $Dest -Recurse -Force
Copy-Item (Join-Path $TutorialLibs "*") (Join-Path $Dest "libs") -Recurse -Force

Write-Host "Synced plugin to $Dest"
Restart-StreamDeckPluginHost
