param(
    [string]$CubeIde = 'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\stm32cubeidec.exe'
)
$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $CubeIde -PathType Leaf)) {
    throw 'Pass -CubeIde with the installed stm32cubeidec.exe path.'
}
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
# Fresh copy per run: preserve upstream sources and all previous build results.
$runName = 'stm32-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0, 6)
$runRoot = Join-Path $repoRoot ('.build\' + $runName)
New-Item -ItemType Directory -Path $runRoot | Out-Null
$projectPath = Join-Path $runRoot 'project'
$workspacePath = Join-Path $runRoot 'workspace'
Copy-Item -LiteralPath (Join-Path $repoRoot 'drone-firmware') -Destination $projectPath -Recurse
$buildOutput = & $CubeIde --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data $workspacePath -import $projectPath -build 'program/Debug' 2>&1
$buildExit = $LASTEXITCODE
$logPath = Join-Path $runRoot 'build.log'
$buildOutput | Out-File -LiteralPath $logPath -Encoding utf8
$buildOutput | Select-Object -Last 16
$elfPath = Join-Path $projectPath 'Debug\program.elf'
if ($buildExit -ne 0 -or -not (Test-Path -LiteralPath $elfPath)) {
    throw "STM32 build failed; inspect $logPath"
}
Write-Output "Built (not flashed): $elfPath"
Write-Output "Log: $logPath"
