<#
.SYNOPSIS
    Script to automatically handle SSH and SCP tasks for interacting
    with .aftershockauto files on a WPI RoboRIO.

.DESCRIPTION
    USAGE STEPS
    1. Right click on the syncRecordings.ps1 file in the VSCode workspace.
    2. Left click "Reveal in file explorer".
    3. Right click on the file for the context menu.
    4. Click "Run with PowerShell".
    5. Follow the prompts that appear in your terminal.

.NOTES
    Version 2024-03-16

.EXAMPLE
    .\syncRecordings.ps1

.NOTES
    Author: Adrian Dayao
    Date:   2024-03-16
#>

$robotIP = "10.2.63.2"

# Go to the src/main/deploy/aftershockauto directory
Set-Location -Path $PSScriptRoot
$currentDir = Get-Location

while (-not (Test-Path "$currentDir\deploy")) {
    if ($currentDir -eq "C:\") {
        Write-Error "Error: 'deploy' directory not found. Quitting!"
        exit 1
    }
    $currentDir = Split-Path $currentDir
}

Set-Location "$currentDir\deploy"
mkdir -Force "aftershockauto" # Make if it doesn't exist
Set-Location "aftershockauto"

# Begin SCP



Write-Host "Beginning SCP file transfer from RoboRIO --> local computer."
Write-Host "WARNING! Files may be overwritten with new ones." -ForegroundColor DarkYellow
Read-Host    "Continue? (Control+C to stop.)"
scp "lvuser@${robotIP}:*.aftershockauto" .