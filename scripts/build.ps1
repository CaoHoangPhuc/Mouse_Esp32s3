param(
  [string]$Fqbn = "esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashMode=qio120,PartitionScheme=min_spiffs,LoopCore=0,EventsCore=0",
  [string]$Sketch = "",
  [int]$Jobs = 0,
  [string]$LogFile = "",
  [switch]$VerboseOutput = $true,
  [string]$CliPath = ""
)

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

if ($Jobs -le 0) {
  $Jobs = [Math]::Max(1, [Environment]::ProcessorCount * 4)
}

Write-Host "[build] Compiling $Sketch"
if ($VerboseOutput) {
  Write-Host "[build] FQBN: $Fqbn"
  Write-Host "[build] Jobs: $Jobs (4x logical processors)"
}

$cliArgs = @("compile", "--fqbn", $Fqbn, "--jobs", $Jobs)
if ($VerboseOutput) {
  $cliArgs += "--verbose"
}
$cliArgs += $Sketch

Write-Host ("[build] cmd: {0} {1}" -f $cli, ($cliArgs -join " "))

$previousErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = "SilentlyContinue"
try {
  if ($LogFile.Length -gt 0) {
    & $cli @cliArgs 2>&1 | ForEach-Object {
      $text = [string]$_
      Write-Host $text
      Add-Content -Path $LogFile -Value $text -Encoding utf8
    }
  } else {
    & $cli @cliArgs 2>&1 | ForEach-Object { Write-Host ([string]$_) }
  }
}
finally {
  $ErrorActionPreference = $previousErrorActionPreference
}

$buildExit = $LASTEXITCODE
if ($buildExit -ne 0) {
  throw "Compile failed (exit $buildExit). See log: $LogFile"
}

Write-Host "[build] Compile success"
