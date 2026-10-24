# Cleanup: remove modular-controller sources and sync the Keil project file list.
# Run from anywhere; paths are absolute on purpose.
$ErrorActionPreference = 'Stop'
$root = 'D:\RM\new_remoto\new_remoto'
$projDir = Join-Path $root 'MDK-ARM'

# --- 1. modules that only exist to drive gimbal / chassis / shoot / arm / calibrate / music
$dirs = @(
    'application\calibrate',
    'application\chassis',
    'application\gimbal',
    'application\mechanical_arm',
    'application\music',
    'application\shoot'
)

# --- 2. single dead files
$files = @(
    'application\assist\uart_receive.c',
    'application\assist\uart_receive.h',
    'application\communication\supervisory_computer_cmd.h',
    'application\communication\usb_debug.h',
    'application\other\remoto.c',
    'application\other\remoto.h',
    'application\typedef_robot\remoto.h',
    'application\typedef_robot\robot_param_engineer.h',
    'application\typedef_robot\robot_param_fly.h',
    'application\typedef_robot\robot_param_foot.h',
    'application\typedef_robot\robot_param_mecannum_hero.h',
    'application\typedef_robot\robot_param_mini_pro.h',
    'application\typedef_robot\robot_param_omni_infantry.h',
    'application\typedef_robot\robot_param_omni_mini.h',
    'application\typedef_robot\robot_param_omni_sentry.h',
    'application\typedef_robot\robot_param_omni_sentry_two.h',
    'application\typedef_robot\robot_param_water.h',
    '_check\deltest2.txt'
)

$removedDirs = 0
foreach ($d in $dirs) {
    $p = Join-Path $root $d
    if (Test-Path $p) { Remove-Item -LiteralPath $p -Recurse -Force; $removedDirs++; Write-Host "rmdir  $d" }
}

$removedFiles = 0
foreach ($f in $files) {
    $p = Join-Path $root $f
    if (Test-Path $p) { Remove-Item -LiteralPath $p -Force; $removedFiles++; Write-Host "rm     $f" }
}

Write-Host "removed dirs=$removedDirs files=$removedFiles"

# --- 3. drop <File> entries whose FilePath no longer resolves, then empty groups
$uv = Join-Path $projDir 'StandardRobotPro.uvprojx'
[xml]$doc = Get-Content -LiteralPath $uv -Raw -Encoding UTF8
$dropped = New-Object System.Collections.Generic.List[string]

function Resolve-ProjPath([string]$rel) {
    if ([string]::IsNullOrWhiteSpace($rel)) { return $null }
    $p = $rel -replace '\\', '\'
    return [System.IO.Path]::GetFullPath((Join-Path $projDir $p))
}

$fileNodes = $doc.SelectNodes('//File')
foreach ($n in $fileNodes) {
    $fp = $n.FilePath
    if ([string]::IsNullOrWhiteSpace($fp)) { continue }
    $full = Resolve-ProjPath $fp
    if (-not (Test-Path -LiteralPath $full)) {
        $dropped.Add($fp)
        [void]$n.ParentNode.RemoveChild($n)
    }
}
Write-Host "uvprojx: dropped $($dropped.Count) missing file entries"

# remove groups left without any <File> children (repeat for nesting)
for ($pass = 0; $pass -lt 3; $pass++) {
    $emptyGroups = @()
    foreach ($g in $doc.SelectNodes('//Group')) {
        if ($null -eq $g.SelectSingleNode('.//File')) { $emptyGroups += $g }
    }
    if ($emptyGroups.Count -eq 0) { break }
    foreach ($g in $emptyGroups) {
        $name = if ($g.GroupName) { $g.GroupName } else { '?' }
        [void]$g.ParentNode.RemoveChild($g)
        Write-Host "uvprojx: dropped empty group $name"
    }
}

$doc.Save($uv)
Write-Host "uvprojx saved"
