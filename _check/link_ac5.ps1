#requires -Version 5.0
<#
    AC5 (armcc/armasm/armlink) full link check for StandardRobotPro.

    Compiles every C file in the Keil project, assembles the startup file, then
    links everything (incl. AHRS.lib / arm_cortexM4lf_math.lib) with a minimal
    STM32F407IGHx scatter file.  This proves the source set links, i.e. that no
    symbol was left dangling by the module cleanup.

    Objects and the .axf/.map go to a temp folder; the project tree is untouched.
    Usage: powershell -ExecutionPolicy Bypass -File _check\link_ac5.ps1
#>
[CmdletBinding()]
param()

$ErrorActionPreference = 'Continue'

$projRoot = Split-Path -Parent $PSScriptRoot
$mdk      = Join-Path $projRoot 'MDK-ARM'
$uvprojx  = Join-Path $mdk 'StandardRobotPro.uvprojx'
$armDir   = 'D:\RM\Keil\Core\ARM\ARMCC'
$armcc    = Join-Path $armDir 'bin\armcc.exe'
$armasm   = Join-Path $armDir 'bin\armasm.exe'
$armlink  = Join-Path $armDir 'bin\armlink.exe'
$fromelf  = Join-Path $armDir 'bin\fromelf.exe'

[xml]$xml = Get-Content -LiteralPath $uvprojx -Raw -Encoding UTF8

$defines  = ($xml.SelectSingleNode('//Cads/VariousControls/Define').InnerText) -split ',' |
            Where-Object { $_ -and $_.Trim() } | ForEach-Object { "-D$($_.Trim())" }
$includes = ($xml.SelectSingleNode('//Cads/VariousControls/IncludePath').InnerText) -split ';' |
            Where-Object { $_ -and $_.Trim() } | ForEach-Object { "-I" + [IO.Path]::GetFullPath((Join-Path $mdk $_.Trim())) }
$incDirs  = ($xml.SelectSingleNode('//Cads/VariousControls/IncludePath').InnerText) -split ';' |
            Where-Object { $_ -and $_.Trim() } | ForEach-Object { "--pd", "`"$([IO.Path]::GetFullPath((Join-Path $mdk $_.Trim())))`"" }

$files = $xml.SelectNodes('//File')
$sources = $files | Where-Object { $_.FileType -eq 1 } |
           ForEach-Object { [IO.Path]::GetFullPath((Join-Path $mdk ($_.FilePath -replace '/', '\'))) } |
           Where-Object { $_ -like '*.c' }
$asmFile = $files | Where-Object { $_.FilePath -like '*.s' } | Select-Object -First 1 |
           ForEach-Object { [IO.Path]::GetFullPath((Join-Path $mdk ($_.FilePath -replace '/', '\'))) }
$libs    = $files | Where-Object { $_.FilePath -like '*.lib' } |
           ForEach-Object { [IO.Path]::GetFullPath((Join-Path $mdk ($_.FilePath -replace '/', '\'))) }

$work = Join-Path ([IO.Path]::GetTempPath()) 'robotpro-link'
if (Test-Path $work) { Remove-Item $work -Recurse -Force }
New-Item -ItemType Directory -Path $work | Out-Null

$scatter = Join-Path $work 'stm32f407.sct'
@'
LR_IROM1 0x08000000 0x00100000  {
  ER_IROM1 0x08000000 0x00100000  {
    startup_stm32f407xx.o (RESET, +First)
    *(InRoot$$Sections)
    .ANY (+RO)
  }
  RW_IRAM1 0x20000000 0x00020000  {
    .ANY (+RW +ZI)
  }
}
'@ | Set-Content -LiteralPath $scatter -Encoding ASCII

# ---- compile -------------------------------------------------------------
$objs = @()
$i = 0
$failed = @()
foreach ($src in $sources) {
    $i++
    $obj = Join-Path $work ("f{0:D3}.o" -f $i)
    $log = & $armcc --c99 -c --cpu Cortex-M4.fp --apcs=interwork -O0 `
                    --diag_suppress=1 --diag_suppress=1295 @defines @includes -o $obj $src 2>&1
    if ($LASTEXITCODE -ne 0) { $failed += $src; $log | Select-Object -First 4 | Write-Host }
    else { $objs += $obj }
}
Write-Host ("compiled {0}/{1}" -f $objs.Count, $sources.Count)
if ($failed.Count) { Write-Host "COMPILE FAILURES:"; $failed | ForEach-Object { Write-Host "  $_" }; exit 1 }

# ---- assemble ------------------------------------------------------------
$startupObj = Join-Path $work 'startup_stm32f407xx.o'
& $armasm --cpu Cortex-M4.fp --apcs=interwork -o $startupObj $asmFile 2>&1 | Write-Host
if ($LASTEXITCODE -ne 0) { Write-Host "ASSEMBLE FAILED"; exit 1 }

# ---- link ---------------------------------------------------------------
$axf = Join-Path $work 'StandardRobotPro.axf'
$map = Join-Path $work 'StandardRobotPro.map'
$out = & $armlink --cpu=Cortex-M4.fp --entry=Reset_Handler --scatter $scatter `
                  --output $axf --map --list $map `
                  --info=sizes,totals,unused,veneers `
                  ($startupObj) @objs @libs `
                  --libpath (Join-Path $armDir 'lib') 2>&1
$code = $LASTEXITCODE
$errs = $out | Where-Object { $_ -match 'Error:|Fatal' }
$out | Where-Object { $_ -match 'Error:|Warning: L|Total RO|Total RW|Total ROM|Image component sizes' } |
     Select-Object -First 30 | ForEach-Object { Write-Host $_ }
Write-Host ("----- armlink exit {0}, errors: {1} -----" -f $code, $errs.Count)
if ($code -ne 0) {
    $errs | Select-Object -First 25 | ForEach-Object { Write-Host $_ }
    exit 1
}
& $fromelf --text -z --verbose $axf 2>&1 | Select-String 'Total|Image' | ForEach-Object { Write-Host $_ }
Write-Host "axf: $axf"
exit 0
