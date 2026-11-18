param([string]$Keil = 'C:\Keil_v5\UV4\UV4.exe')
$ErrorActionPreference = 'Stop'
$repoDir = Split-Path -Parent $PSScriptRoot
$projectPath = Join-Path $repoDir 'MDK-ARM\dual_board.uvprojx'
if (!(Test-Path -LiteralPath $projectPath)) { & (Join-Path $PSScriptRoot 'prepare_dual_board.ps1') }
foreach ($role in @('UPPER', 'LOWER')) {
    $logPath = Join-Path $repoDir "MDK-ARM\build_$($role.ToLower()).log"
    $argsList = @('-b', ('"{0}"' -f $projectPath), '-t', $role, '-o', ('"{0}"' -f $logPath))
    $process = Start-Process -FilePath $Keil -ArgumentList $argsList -WindowStyle Hidden -Wait -PassThru
    $logText = Get-Content -LiteralPath $logPath -Raw
    if ($process.ExitCode -gt 1 -or $logText -notmatch '- 0 Error\(s\), 0 Warning\(s\)') {
        throw "Build failed for $role`n$logText"
    }
    $hexPath = Join-Path $repoDir "MDK-ARM\build\$role\$role.hex"
    $firmwarePath = Join-Path $repoDir "firmware\$role.hex"
    Copy-Item -LiteralPath $hexPath -Destination $firmwarePath -Force
    Write-Output "$role : 0 errors, 0 warnings -> $firmwarePath"
    Get-FileHash -LiteralPath $firmwarePath -Algorithm SHA256 | Select-Object Path,Hash
}
