$ErrorActionPreference = 'Stop'

$pioRoot = Join-Path $env:USERPROFILE '.platformio'
$idfPath = Join-Path $pioRoot 'packages\framework-espidf'
$pythonEnv = Join-Path $pioRoot 'penv\.espidf-5.5.5'
$python = Join-Path $pythonEnv 'Scripts\python.exe'
$idfPy = Join-Path $idfPath 'tools\idf.py'

$env:IDF_PATH = $idfPath
$env:IDF_PYTHON_ENV_PATH = $pythonEnv
$env:IDF_MAINTAINER = '1'
$env:IDF_PYTHON_CHECK_CONSTRAINTS = 'no'
$env:CMAKE_BUILD_PARALLEL_LEVEL = '1'
$env:ESP_ROM_ELF_DIR = Join-Path $pioRoot 'packages\tool-esp-rom-elfs'
$env:PATH = @(
    (Join-Path $pioRoot 'packages\toolchain-xtensa-esp-elf\bin')
    (Join-Path $pioRoot 'packages\tool-cmake\bin')
    (Join-Path $pioRoot 'packages\tool-ninja')
    (Join-Path $pythonEnv 'Scripts')
    $env:PATH
) -join [IO.Path]::PathSeparator

if (!(Test-Path -LiteralPath $python) -or !(Test-Path -LiteralPath $idfPy)) {
    throw 'ESP-IDF 5.5.5 PlatformIO runtime is missing. Install espressif32@55.3.311 first.'
}

$ninja = Join-Path $pioRoot 'packages\tool-ninja\ninja.exe'
if ((Test-Path -LiteralPath 'build_idf\build.ninja') -and
    (Test-Path -LiteralPath $ninja)) {
    # Reuse the configured build tree and avoid conflicts with a separately
    # installed ESP-IDF constraint file in the parent PowerShell environment.
    # ESP-IDF 5.5.5 on this Windows host can start ldgen while a static
    # archive is still being finalized.  Serial execution prevents ldgen
    # from parsing a partially-written .a file.
    & $ninja -C build_idf -j 1
} else {
    & $python $idfPy -B build_idf build
}
exit $LASTEXITCODE
