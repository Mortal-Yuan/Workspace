$ErrorActionPreference = 'Stop'

$repository = (Resolve-Path "$PSScriptRoot\..\..").Path
$compilerCandidates = @(
    'C:\Xilinx\Vivado\2019.1\msys64\mingw64\bin\gcc.exe',
    'C:\altera\13.0\modelsim_ase\gcc-4.2.1-mingw32vc9\bin\gcc.exe'
)
$compiler = $compilerCandidates |
    Where-Object { Test-Path -LiteralPath $_ } |
    Select-Object -First 1
if (-not $compiler) {
    $gcc = Get-Command gcc -ErrorAction SilentlyContinue
    if ($gcc) {
        $compiler = $gcc.Source
    }
}
if (-not $compiler) {
    throw 'A native Windows GCC compiler is required for host tests.'
}
$env:PATH = (Split-Path -Parent $compiler) + ';' + $env:PATH
$outputDirectory = Join-Path $repository 'build\host-tests'
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
$output = Join-Path $outputDirectory 'firmware_tests.exe'

& $compiler `
    -std=c11 -Wall -Wextra -Werror `
    "-I$PSScriptRoot\fakes" `
    "-I$repository\main\app" `
    "-I$repository\main\control" `
    "-I$repository\main\core" `
    "-I$repository\main\drivers" `
    "-I$repository\main\platform" `
    "$PSScriptRoot\test_firmware.c" `
    "$repository\main\app\app_config.c" `
    "$repository\main\control\kiwi_kinematics.c" `
    "$repository\main\control\line_follow.c" `
    "$repository\main\control\obstacle_supervisor.c" `
    "$repository\main\drivers\motor_driver.c" `
    "$repository\main\drivers\start_button.c" `
    "$repository\main\drivers\ultrasonic.c" `
    -o $output

if ($LASTEXITCODE -ne 0) {
    throw "Host test compilation failed with exit code $LASTEXITCODE"
}

& $output
if ($LASTEXITCODE -ne 0) {
    throw "Host tests failed with exit code $LASTEXITCODE"
}
