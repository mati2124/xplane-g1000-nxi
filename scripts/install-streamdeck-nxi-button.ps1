# Builds and installs the X-Plane NXi Stream Deck plugin.
$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$PluginRoot = Join-Path $RepoRoot "streamdeck-plugin"
$Source = Join-Path $PluginRoot "com.andy.xplane-nxi.sdPlugin"
$Dest = Join-Path $env:APPDATA "Elgato\StreamDeck\Plugins\com.andy.xplane-nxi.sdPlugin"
$ProfilesRoot = Join-Path $env:APPDATA "Elgato\StreamDeck\ProfilesV2"
$Launcher = Join-Path $RepoRoot "scripts\restart-xplane-release.bat"
$TutorialLibs = Join-Path $env:APPDATA "Elgato\StreamDeck\Plugins\com.elgato.tutorial.sdPlugin\libs"

function Register-RestartProtocol {
    param([string]$BatPath)
    $classes = "HKCU:\Software\Classes\nxirestart"
    New-Item -Path $classes -Force | Out-Null
    Set-Item -Path $classes -Value "URL:NXi Restart"
    New-ItemProperty -Path $classes -Name "URL Protocol" -Value "" -PropertyType String -Force | Out-Null
    $commandKey = Join-Path $classes "shell\open\command"
    New-Item -Path $commandKey -Force | Out-Null
    Set-Item -Path $commandKey -Value ('cmd.exe /c ""{0}"" "%1"' -f $BatPath)
}

function Install-ProfileButton {
    param(
        [string]$ProfileName,
        [string]$ProfileId,
        [string]$PageFolder,
        [string]$ButtonKey
    )

    $pageManifest = Join-Path $ProfilesRoot "$ProfileId\Profiles\$PageFolder\manifest.json"
    if (-not (Test-Path $pageManifest)) {
        Write-Warning "Skipping $ProfileName profile page: $pageManifest"
        return
    }

    $raw = [System.IO.File]::ReadAllText($pageManifest)
    if ($raw.Length -gt 0 -and [int][char]$raw[0] -eq 0xFEFF) {
        $raw = $raw.Substring(1)
    }
    $manifest = $raw | ConvertFrom-Json
    $actions = $manifest.Controllers[0].Actions

    $newAction = [ordered]@{
        ActionID    = [guid]::NewGuid().ToString()
        LinkedTitle = $false
        Name        = "Open"
        Settings    = [ordered]@{
            openInBrowser = $false
            path          = $Launcher
        }
        State       = 0
        States      = @(
            [ordered]@{
                FontFamily       = ""
                FontSize         = 9
                FontStyle        = ""
                FontUnderline    = $false
                OutlineThickness = 2
                ShowTitle        = $true
                Title            = "NXI`nRESTART"
                TitleAlignment   = "middle"
                TitleColor       = "#00ff88"
            }
        )
        UUID = "com.elgato.streamdeck.system.open"
    }

    $actions | Add-Member -NotePropertyName $ButtonKey -NotePropertyValue ([pscustomobject]$newAction) -Force
    $manifest.Controllers[0].Actions = $actions

    $json = $manifest | ConvertTo-Json -Depth 20 -Compress
    $utf8NoBom = New-Object System.Text.UTF8Encoding $false
    [System.IO.File]::WriteAllText($pageManifest, $json, $utf8NoBom)
    Write-Host "Pinned button on $ProfileName at $ButtonKey"
}

function Restart-StreamDeckApp {
    Get-Process -Name "StreamDeck" -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep -Seconds 2

    $candidates = @(
        "C:\Program Files\Elgato\StreamDeck\StreamDeck.exe",
        "C:\Program Files (x86)\Elgato\StreamDeck\StreamDeck.exe",
        (Join-Path $env:LOCALAPPDATA "Programs\Elgato\StreamDeck\StreamDeck.exe")
    )

    $installDir = (Get-ItemProperty "HKCU:\Software\Elgato Systems GmbH\StreamDeck" -ErrorAction SilentlyContinue).InstallDir
    if ($installDir) {
        $candidates = @((Join-Path $installDir "StreamDeck.exe")) + $candidates
    }

    foreach ($exe in $candidates | Select-Object -Unique) {
        if (-not $exe) { continue }
        try {
            if (Test-Path $exe) {
                Start-Process -FilePath $exe
                Write-Host "Started Stream Deck: $exe"
                return
            }
        } catch {
            continue
        }
    }

    Write-Warning "Could not locate StreamDeck.exe automatically. Please open Stream Deck manually."
}

if (-not (Test-Path $Source)) {
    throw "Plugin source not found: $Source"
}
if (-not (Test-Path $TutorialLibs)) {
    throw "Stream Deck tutorial libs not found: $TutorialLibs"
}

Write-Host "Generating PNG icons..."
& (Join-Path $PluginRoot "scripts\generate-icons.ps1") `
  -Paths @(
    (Join-Path $Source "imgs\plugin\category"),
    (Join-Path $Source "imgs\plugin\marketplace"),
    (Join-Path $Source "imgs\actions\restart\icon"),
    (Join-Path $Source "imgs\actions\restart\key"),
    (Join-Path $Source "imgs\actions\kfmy-final\icon"),
    (Join-Path $Source "imgs\actions\kfmy-final\key")
  ) `
  -Size 144

if (Test-Path $Dest) {
    Remove-Item $Dest -Recurse -Force
}

New-Item -ItemType Directory -Force -Path (Join-Path $Source "libs") | Out-Null
Copy-Item $Source $Dest -Recurse -Force
Copy-Item (Join-Path $TutorialLibs "*") (Join-Path $Dest "libs") -Recurse -Force

Register-RestartProtocol -BatPath $Launcher

Install-ProfileButton -ProfileName "MSFS PH100" -ProfileId "52BA750F-550A-4E61-93E8-80E362D6289F.sdProfile" -PageFolder "0TRBJDS6DP1HP90CSAQQ0NL79KZ" -ButtonKey "2,0"
Install-ProfileButton -ProfileName "X-Plane TBM" -ProfileId "8824E97F-72CC-4460-B443-C5F5F93A56E6.sdProfile" -PageFolder "3FIT03F2AH31D3CM6CJ7B4I8KCZ" -ButtonKey "4,1"

Write-Host "Installed plugin to $Dest"
Restart-StreamDeckApp

Write-Host ""
Write-Host "Look for:"
Write-Host "  1. A key labeled NXI RESTART on MSFS PH100 (row 3, left) and X-Plane TBM (bottom center)"
Write-Host "  2. Or drag actions from the X-Plane NXi category:"
Write-Host "       - Restart NXi"
Write-Host "       - KFMY RNAV 05 Final (C172 G1000 long final ~90 kt)"
