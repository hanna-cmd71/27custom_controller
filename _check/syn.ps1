# 语法检查：用 arm-none-eabi-gcc 对工程里的 C 文件做 -fsyntax-only 检查。
# 用法:
#   pwsh -NoProfile -File .\_check\syn.ps1                # 检查 application/bsp/components 下所有 .c
#   pwsh -NoProfile -File .\_check\syn.ps1 -Files a.c,b.c
param(
    [string[]]$Files,
    [switch]$All
)

$ErrorActionPreference = 'Continue'
$root = 'D:\RM\new_remoto\new_remoto'
$gcc  = 'D:\Tools\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe'

# 从 uvprojx 里取 include 路径和宏定义
[xml]$proj = Get-Content "$root\MDK-ARM\StandardRobotPro.uvprojx" -Raw
$vc = $proj.Project.Targets.Target.TargetOption.TargetArmAds.Cads.VariousControls
$defs = $vc.Define -split ',' | Where-Object { $_ -and $_ -ne '__CC_ARM' } | ForEach-Object { "-D$_" }
$incs = @("-I$root\_check\gccport") + ($vc.IncludePath -split ';' | Where-Object { $_ } | ForEach-Object {
    $p = Join-Path $root ($_ -replace '^\.\.[\\/]', '')
    if (Test-Path $p) { "-I$p" }
})

$common = @(
    '-fsyntax-only', '-std=c99', '-mcpu=cortex-m4', '-mthumb',
    '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
    '-Wno-pragma-pack', '-Wno-attributes', '-Wno-pointer-sign',
    '-include', "$root\_check\gccport\compat.h"
) + $defs + $incs

if (-not $Files) {
    $dirs = @('application', 'bsp', 'components') | ForEach-Object { Join-Path $root $_ }
    $Files = $dirs | ForEach-Object { Get-ChildItem $_ -Recurse -Filter *.c -ErrorAction SilentlyContinue } | ForEach-Object { $_.FullName }
}

$fail = 0
foreach ($f in $Files) {
    if (-not (Test-Path $f)) { "MISSING $f"; $fail++; continue }
    $out = & $gcc @common $f 2>&1
    if ($LASTEXITCODE -ne 0) {
        $fail++
        $rel = $f.Substring($root.Length + 1)
        "FAIL $rel"
        $out | Where-Object { $_ -match 'error' } | Select-Object -First 12 | ForEach-Object { "     $_" }
    }
}
"----- checked $(@($Files).Count) file(s), failures: $fail -----"
exit $fail
