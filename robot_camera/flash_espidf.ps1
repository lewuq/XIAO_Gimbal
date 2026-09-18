param(
    [Parameter(Mandatory = $true)]
    [string]$Port,
    [int]$Baud = 460800,
    [switch]$Monitor
)

$ErrorActionPreference = 'Stop'

$pioRoot = Join-Path $env:USERPROFILE '.platformio'
$python = Join-Path $pioRoot 'penv\Scripts\python.exe'
$buildDir = Join-Path $PSScriptRoot 'build_idf'
$bootloader = Join-Path $buildDir 'bootloader\bootloader.bin'
$partitionTable = Join-Path $buildDir 'partition_table\partition-table.bin'
$application = Join-Path $buildDir 'gimbal_recamera.bin'

foreach ($requiredFile in @($python, $bootloader, $partitionTable, $application)) {
    if (!(Test-Path -LiteralPath $requiredFile)) {
        throw "Required file is missing: $requiredFile"
    }
}

# Invoke esptool directly. This avoids idf.py's Python package constraint check.
& $python -m esptool `
    --chip esp32s3 `
    --port $Port `
    --baud $Baud `
    --before default-reset `
    --after hard-reset `
    write-flash `
    --flash-mode dio `
    --flash-freq 80m `
    --flash-size 8MB `
    0x0 $bootloader `
    0x8000 $partitionTable `
    0x10000 $application

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

if ($Monitor) {
    $platformio = Join-Path $pioRoot 'penv\Scripts\platformio.exe'
    if (!(Test-Path -LiteralPath $platformio)) {
        throw "PlatformIO executable is missing: $platformio"
    }
    & $platformio device monitor --port $Port --baud 115200
    exit $LASTEXITCODE
}
