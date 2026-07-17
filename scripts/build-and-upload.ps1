param(
  [string]$Fqbn = "esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashMode=qio120,PartitionScheme=min_spiffs,LoopCore=0,EventsCore=0",
  [string]$Sketch = "",
  [string]$Port = "COM3",
  [int]$UploadSpeed = 921600,
  [int]$Jobs = 0,
  [switch]$RetryWithLowerBaud = $true,
  [switch]$VerboseOutput = $true,
  [switch]$ForceFreePort = $true,
  [string]$CliPath = ""
)

$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

if ([string]::IsNullOrWhiteSpace($Sketch)) {
  $Sketch = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
} else {
  $Sketch = (Resolve-Path $Sketch).Path
}

Write-Host "[pipeline] Compile -> Upload"
$logsDir = Join-Path $Sketch "logs"
if (!(Test-Path $logsDir)) { New-Item -ItemType Directory -Path $logsDir | Out-Null }
$ts = Get-Date -Format "yyyyMMdd_HHmmss"
$logFile = Join-Path $logsDir "build_upload_$ts.log"
$latest = Join-Path $logsDir "build_upload_latest.log"
"[$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')] pipeline start" | Out-File -FilePath $logFile -Encoding utf8

try {
  & "$PSScriptRoot\build.ps1" -Fqbn $Fqbn -Sketch $Sketch -Jobs $Jobs -LogFile $logFile -VerboseOutput:$VerboseOutput -CliPath $CliPath

  try {
    & "$PSScriptRoot\upload.ps1" -Fqbn $Fqbn -Sketch $Sketch -Port $Port -UploadSpeed $UploadSpeed -LogFile $logFile -VerboseOutput:$VerboseOutput -ForceFreePort:$ForceFreePort -CliPath $CliPath
  }
  catch {
    if ($RetryWithLowerBaud) {
      Write-Warning "[pipeline] Upload failed at $UploadSpeed. Retrying with 460800..."
      & "$PSScriptRoot\upload.ps1" -Fqbn $Fqbn -Sketch $Sketch -Port $Port -UploadSpeed 460800 -LogFile $logFile -VerboseOutput:$VerboseOutput -ForceFreePort:$ForceFreePort -CliPath $CliPath
    }
    else {
      throw
    }
  }

  Write-Host "[pipeline] Done: compile + upload successful"
}
finally {
  "[$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')] pipeline end (last exit: $LASTEXITCODE)" | Out-File -FilePath $logFile -Encoding utf8 -Append
  if (Test-Path $logFile) { Copy-Item -Force $logFile $latest }
  Write-Host "[pipeline] Log: $logFile"
}
