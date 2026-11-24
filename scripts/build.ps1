param(
    [ValidateSet('debug', 'release')]
    [string]$Preset = 'debug'
)

$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot

function Get-VsInstallPath {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere)) { return $null }
    $path = & $vswhere -latest -products * `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
        -property installationPath 2>$null
    if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($path)) { return $null }
    return $path.Trim()
}

function Get-MinimalWindowsPathEntries {
    return @(
        (Join-Path $env:SystemRoot 'System32'),
        $env:SystemRoot,
        (Join-Path $env:SystemRoot 'System32\Wbem'),
        (Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0')
    )
}

function Import-VsDevEnvironment([string]$Install) {
    $vsdevcmd = Join-Path $Install 'Common7\Tools\VsDevCmd.bat'
    if (-not (Test-Path $vsdevcmd)) { throw 'VsDevCmd.bat not found.' }
    $env:PATH = (Get-MinimalWindowsPathEntries) -join ';'
    $envFile = Join-Path $env:TEMP "bpla-vsdev-env-$PID.txt"
    $cmd = "call `"$vsdevcmd`" -arch=amd64 -host_arch=amd64 -no_logo && set > `"$envFile`""
    & cmd.exe /c $cmd
    if ($LASTEXITCODE -ne 0) { throw "VsDevCmd failed with exit code $LASTEXITCODE" }
    $vsPathEntries = @()
    foreach ($raw in Get-Content -LiteralPath $envFile) {
        $line = [string]$raw
        if ($line -notmatch '^([A-Za-z0-9_().]+)=(.*)$') { continue }
        $name = $matches[1]
        $value = $matches[2]
        if ($name -eq 'PATH') {
            foreach ($entry in ($value -split ';')) {
                if ([string]::IsNullOrWhiteSpace($entry)) { continue }
                if ($entry.StartsWith($Install, [System.StringComparison]::OrdinalIgnoreCase) -or
                    $entry -match 'Windows Kits' -or
                    $entry -match 'WindowsSDK') {
                    $vsPathEntries += $entry
                }
            }
            continue
        }
        [System.Environment]::SetEnvironmentVariable($name, $value, 'Process')
    }
    Remove-Item -LiteralPath $envFile -Force -ErrorAction SilentlyContinue
    $env:PATH = ((Get-MinimalWindowsPathEntries) + $vsPathEntries) -join ';'
}

$install = Get-VsInstallPath
if (-not $install) { throw 'MSVC Build Tools not found.' }
Import-VsDevEnvironment -Install $install

$cmake = Join-Path $install 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
$ninja = Join-Path $install 'Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe'
if (-not (Test-Path $cmake)) { throw 'CMake not found.' }
if (-not (Test-Path $ninja)) { throw 'Ninja not found.' }
$env:PATH = (@((Split-Path $cmake), (Split-Path $ninja)) + ($env:PATH -split ';')) -join ';'

Set-Location $Root
& $cmake --preset $Preset
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $cmake --build --preset $Preset
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "Built calibrate.exe in $(Join-Path $Root "out\$Preset")"
