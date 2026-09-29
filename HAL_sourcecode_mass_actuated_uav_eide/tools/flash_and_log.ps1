param(
    [string]$HexPath = ".\build\Firmware405\Firmware_F405.hex",
    [string]$OpenOCDPath = "",
    [string]$BuildCommand = "",
    [switch]$SkipFlash,
    [switch]$Monitor,
    [string]$Port = "",
    [int]$Baud = 115200,
    [int]$MonitorSeconds = 20,
    [string]$LogDir = ".\logs"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$ProjectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $ProjectRoot

function Write-Step {
    param([string]$Message)
    Write-Host ""
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Resolve-ExistingFile {
    param(
        [string]$Path,
        [string]$Name
    )

    $resolved = Resolve-Path -Path $Path -ErrorAction SilentlyContinue
    if (-not $resolved) {
        throw "$Name not found: $Path"
    }
    return $resolved.Path
}

function Find-OpenOCD {
    if ($OpenOCDPath) {
        return Resolve-ExistingFile -Path $OpenOCDPath -Name "OpenOCD"
    }

    if ($env:OPENOCD_EXE) {
        $candidate = Resolve-Path -Path $env:OPENOCD_EXE -ErrorAction SilentlyContinue
        if ($candidate) { return $candidate.Path }
    }

    $cmd = Get-Command openocd -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    $knownPaths = @(
        (Join-Path $env:USERPROFILE ".eide\tools\openocd_7a1adfbec_mingw32\bin\openocd.exe")
    )
    foreach ($path in $knownPaths) {
        $candidate = Resolve-Path -Path $path -ErrorAction SilentlyContinue
        if ($candidate) { return $candidate.Path }
    }

    $found = Get-ChildItem -Path (Join-Path $env:USERPROFILE ".eide") -Recurse -Filter openocd.exe -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($found) { return $found.FullName }

    throw "OpenOCD not found. Pass -OpenOCDPath 'C:\path\to\openocd.exe' or set OPENOCD_EXE."
}

function Get-OpenOCDScriptsDir {
    param([string]$ExePath)

    $root = Split-Path -Parent (Split-Path -Parent $ExePath)
    $candidates = @(
        (Join-Path $root "share\openocd\scripts"),
        (Join-Path $root "scripts")
    )

    foreach ($path in $candidates) {
        $resolved = Resolve-Path -Path $path -ErrorAction SilentlyContinue
        if ($resolved) { return $resolved.Path }
    }

    throw "OpenOCD scripts directory not found under: $root"
}

function Invoke-LoggedCommand {
    param(
        [string]$Name,
        [scriptblock]$Command,
        [string]$LogPath
    )

    Write-Step $Name
    $output = & $Command 2>&1
    $exitCode = $LASTEXITCODE
    $output | Tee-Object -FilePath $LogPath

    if ($exitCode -ne 0) {
        throw "$Name failed with exit code $exitCode. See log: $LogPath"
    }
}

function Find-SerialPort {
    if ($Port) { return $Port }

    $ports = Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.Name -match "\(COM\d+\)" } |
        Select-Object Name

    $preferred = $ports |
        Where-Object { $_.Name -match "STMicroelectronics|ST-Link|STM32|USB Serial|CH340|CP210|FTDI" } |
        Select-Object -First 1

    if ($preferred -and $preferred.Name -match "(COM\d+)") {
        return $Matches[1]
    }

    $fallback = [System.IO.Ports.SerialPort]::GetPortNames() | Select-Object -First 1
    if ($fallback) { return $fallback }

    throw "No serial port found. Pass -Port COMx."
}

function Read-SerialLog {
    param(
        [string]$SerialPortName,
        [int]$SerialBaud,
        [int]$Seconds,
        [string]$LogPath
    )

    Write-Step "Reading serial log from $SerialPortName at $SerialBaud baud for $Seconds seconds"

    Add-Type -AssemblyName System.IO.Ports
    $serial = New-Object System.IO.Ports.SerialPort($SerialPortName, $SerialBaud, "None", 8, "One")
    $serial.ReadTimeout = 300
    $serial.DtrEnable = $true
    $serial.RtsEnable = $true

    $deadline = (Get-Date).AddSeconds($Seconds)
    $buffer = New-Object System.Text.StringBuilder

    try {
        $serial.Open()
        while ((Get-Date) -lt $deadline) {
            try {
                $text = $serial.ReadExisting()
                if ($text.Length -gt 0) {
                    [void]$buffer.Append($text)
                    Write-Host $text -NoNewline
                }
            }
            catch [System.TimeoutException] {
            }
            Start-Sleep -Milliseconds 100
        }
    }
    finally {
        if ($serial.IsOpen) { $serial.Close() }
    }

    $buffer.ToString() | Set-Content -Path $LogPath -Encoding UTF8
    Write-Host ""
    Write-Host "Serial log saved: $LogPath"
}

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$buildLog = Join-Path $LogDir "build_$timestamp.log"
$flashLog = Join-Path $LogDir "flash_$timestamp.log"
$serialLog = Join-Path $LogDir "serial_$timestamp.log"

if ($BuildCommand.Trim().Length -gt 0) {
    Invoke-LoggedCommand -Name "Building firmware" -LogPath $buildLog -Command {
        powershell.exe -NoProfile -ExecutionPolicy Bypass -Command $BuildCommand
    }
}
else {
    Write-Step "No build command supplied; using existing hex"
}

$resolvedHex = Resolve-ExistingFile -Path $HexPath -Name "Firmware hex"

if (-not $SkipFlash) {
    $resolvedOpenOCD = Find-OpenOCD
    $scriptsDir = Get-OpenOCDScriptsDir -ExePath $resolvedOpenOCD

    $openocdArgs = @(
        "-s", $scriptsDir,
        "-f", "interface/stlink.cfg",
        "-f", "target/stm32f4x.cfg",
        "-c", "program `"$resolvedHex`" verify reset exit"
    )

    Invoke-LoggedCommand -Name "Flashing $resolvedHex" -LogPath $flashLog -Command {
        & $resolvedOpenOCD @openocdArgs
    }
}

if ($Monitor) {
    $serialPortName = Find-SerialPort
    Read-SerialLog -SerialPortName $serialPortName -SerialBaud $Baud -Seconds $MonitorSeconds -LogPath $serialLog
}

Write-Host ""
Write-Host "Done."
