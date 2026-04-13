param(
  [string]$Fqbn = "esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashMode=qio120,PartitionScheme=min_spiffs,LoopCore=0,EventsCore=0",
  [string]$Sketch = "",
  [string]$Port = "COM3",
  [int]$UploadSpeed = 921600,
  [string]$LogFile = "",
  [switch]$VerboseOutput = $true,
  [switch]$ForceFreePort = $true,
  [string]$CliPath = ""
)

if ($args.Count -gt 0) {
  throw "Unknown arguments for upload.ps1: $($args -join ' '). Use: -Port, -UploadSpeed, -Fqbn, -Sketch, -VerboseOutput, -ForceFreePort, -LogFile, -CliPath"
}

$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

function Resolve-ArduinoCliPath {
  param([string]$ExplicitCliPath)

  if (-not [string]::IsNullOrWhiteSpace($ExplicitCliPath)) {
    $candidate = (Resolve-Path $ExplicitCliPath).Path
    if (!(Test-Path $candidate)) {
      throw "arduino-cli not found at: $candidate"
    }
    return $candidate
  }

  $default = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
  if (Test-Path $default) {
    return $default
  }

  $fromPath = Get-Command arduino-cli.exe -ErrorAction SilentlyContinue
  if ($fromPath) {
    return $fromPath.Source
  }

  throw "arduino-cli not found. Set -CliPath or install Arduino IDE / arduino-cli."
}

$cli = Resolve-ArduinoCliPath -ExplicitCliPath $CliPath

if ([string]::IsNullOrWhiteSpace($Sketch)) {
  $Sketch = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
} else {
  $Sketch = (Resolve-Path $Sketch).Path
}

function Invoke-KillSerialTools {
  param([string]$ComPort)

  $killed = New-Object System.Collections.Generic.List[string]
  $targets = @(
    "arduino-cli",
    "serial-monitor",
    "putty",
    "kitty",
    "ttermpro",
    "coolterm",
    "python"
  )

  foreach ($name in $targets) {
    $procs = Get-Process -Name $name -ErrorAction SilentlyContinue
    foreach ($p in $procs) {
      try {
        Stop-Process -Id $p.Id -Force -ErrorAction Stop
        $killed.Add("$name#$($p.Id)") | Out-Null
      } catch {
      }
    }
  }

  if ($killed.Count -gt 0) {
    Write-Warning "[upload] Closed processes blocking ${ComPort}: $($killed -join ', ')"
  } else {
    Write-Warning "[upload] No known serial tool process found to close for $ComPort"
  }
}

Write-Host "[upload] Uploading to $Port"
if ($VerboseOutput) {
  Write-Host "[upload] FQBN: $Fqbn"
  Write-Host "[upload] Speed: $UploadSpeed"
}

$cliArgs = @(
  "upload",
  "-p", $Port,
  "--fqbn", $Fqbn,
  "--upload-property", "upload.speed=$UploadSpeed"
)
if ($VerboseOutput) {
  $cliArgs += "--verbose"
}
$cliArgs += $Sketch

Write-Host ("[upload] cmd: {0} {1}" -f $cli, ($cliArgs -join " "))

function Invoke-UploadAttempt {
  $previousErrorActionPreference = $ErrorActionPreference
  $ErrorActionPreference = "SilentlyContinue"
  $lines = New-Object System.Collections.Generic.List[string]
  try {
    if ($LogFile.Length -gt 0) {
      & $cli @cliArgs 2>&1 | ForEach-Object {
        $text = [string]$_
        $lines.Add($text) | Out-Null
        Write-Host $text
        Add-Content -Path $LogFile -Value $text -Encoding utf8
      }
    } else {
      & $cli @cliArgs 2>&1 | ForEach-Object {
        $text = [string]$_
        $lines.Add($text) | Out-Null
        Write-Host $text
      }
    }
  }
  finally {
    $ErrorActionPreference = $previousErrorActionPreference
  }

  return @{
    ExitCode = $LASTEXITCODE
    Output = (($lines.ToArray()) -join [Environment]::NewLine)
  }
}

$result = Invoke-UploadAttempt
$uploadExit = $result.ExitCode
$uploadText = $result.Output

$looksLikeCliHelp = $uploadText -match "(?s)Arduino Command Line Interface \(arduino-cli\).*Usage:"
$looksLikeEsptoolSuccess = $uploadText -match "Hard resetting via RTS pin|Hash of data verified|Wrote [0-9]+ bytes"
$looksLikeUploadFailure = $uploadText -match "Failed uploading|A fatal error occurred"

if ($uploadExit -ne 0 -and $ForceFreePort) {
  $isPortBusy = $uploadText -match "Could not open .*port is busy|Access is denied|PermissionError\(13"
  if ($isPortBusy) {
    Write-Warning "[upload] Detected busy $Port. Attempting to free serial port and retry once..."
    Invoke-KillSerialTools -ComPort $Port
    Start-Sleep -Milliseconds 1200
    $result = Invoke-UploadAttempt
    $uploadExit = $result.ExitCode
    $uploadText = $result.Output
    $looksLikeCliHelp = $uploadText -match "(?s)Arduino Command Line Interface \(arduino-cli\).*Usage:"
    $looksLikeEsptoolSuccess = $uploadText -match "Hard resetting via RTS pin|Hash of data verified|Wrote [0-9]+ bytes"
    $looksLikeUploadFailure = $uploadText -match "Failed uploading|A fatal error occurred"
  }
}

if ($uploadExit -ne 0 -or $looksLikeUploadFailure -or $looksLikeCliHelp -or -not $looksLikeEsptoolSuccess) {
  throw "Upload failed or not verified (exit $uploadExit). No esptool success markers detected. Check COM port access and see log: $LogFile"
}

Write-Host "[upload] Upload success"
