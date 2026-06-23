# Kill running NXi windows, rebuild Release, and launch PFD + MFD for X-Plane.
$ErrorActionPreference = "Stop"

$Repo = Split-Path -Parent $PSScriptRoot
$Cmake = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$Exe = Join-Path $Repo "build-standalone\shell-standalone\Release\avionics-standalone.exe"
$Log = Join-Path $Repo "build-standalone\restart-xplane-release.log"

function Write-Log([string]$Message) {
    $line = "[$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')] $Message"
    Add-Content -Path $Log -Value $line
    Write-Host $line
}

try {
    Write-Log "Stopping avionics-standalone..."
    Get-Process -Name "avionics-standalone" -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep -Seconds 1

    Write-Log "Building Release..."
    Set-Location $Repo
    & $Cmake --build build-standalone --target avionics-standalone --config Release -j8
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed with exit code $LASTEXITCODE"
    }

    if (-not (Test-Path $Exe)) {
        throw "Expected executable not found: $Exe"
    }

    Write-Log "Launching PFD and MFD..."
    $env:AVIONICS_SKIP_UPDATE_CHECK = "1"
    Start-Process $Exe -ArgumentList @("--no-mfd", "--debug-menu") -WorkingDirectory $Repo
    Start-Process $Exe -ArgumentList @("--no-pfd", "--debug-menu") -WorkingDirectory $Repo
    Write-Log "Done."
}
catch {
    Write-Log "ERROR: $($_.Exception.Message)"
    Write-Error $_.Exception.Message
    exit 1
}
