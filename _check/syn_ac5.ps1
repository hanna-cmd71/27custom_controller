#requires -Version 5.0
<#
    AC5 (armcc) syntax/compile check for the StandardRobotPro Keil project.

    Compiles exactly the C files listed in MDK-ARM/StandardRobotPro.uvprojx with
    the toolchain the project really uses (ARMCC 5, as in Keil MDK), using the
    Define / IncludePath of the C target. Objects go to a temp folder, nothing in
    the project tree is touched.

    Usage:  powershell -ExecutionPolicy Bypass -File _check\syn_ac5.ps1 [-Quiet]

    Exit code 0  -> every file compiled
    Exit code 1  -> at least one file failed (details printed)
#>
[CmdletBinding()]
param([switch]$Quiet)

$ErrorActionPreference = 'Continue'   # armcc writes diagnostics to stderr; do not let that abort the run

$projRoot = Split-Path -Parent $PSScriptRoot          # ...\new_remoto\new_remoto
$mdk      = Join-Path $projRoot 'MDK-ARM'
$uvprojx  = Join-Path $mdk 'StandardRobotPro.uvprojx'
$armcc    = 'D:\RM\Keil\Core\ARM\ARMCC\bin\armcc.exe'

if (-not (Test-Path $armcc)) { throw "armcc.exe not found: $armcc" }
if (-not (Test-Path $uvprojx)) { throw "uvprojx not found: $uvprojx" }

[xml]$xml = Get-Content -LiteralPath $uvprojx -Raw -Encoding UTF8

# ---- flags straight from the Keil project -------------------------------
$defines = ($xml.SelectSingleNode('//Cads/VariousControls/Define').InnerText) -split ',' |
           Where-Object { $_ -and $_.Trim() } | ForEach-Object { "-D$($_.Trim())" }
$includes = ($xml.SelectSingleNode('//Cads/VariousControls/IncludePath').InnerText) -split ';' |
            Where-Object { $_ -and $_.Trim() } | ForEach-Object { "-I" + [IO.Path]::GetFullPath((Join-Path $mdk $_.Trim())) }

$sources = $xml.SelectNodes('//File') |
           Where-Object { $_.FileType -eq 1 } |
           ForEach-Object { [IO.Path]::GetFullPath((Join-Path $mdk ($_.FilePath -replace '/', '\'))) } |
           Where-Object { $_ -like '*.c' }

$outDir = Join-Path ([IO.Path]::GetTempPath()) 'robotpro-ac5'
if (Test-Path $outDir) { Remove-Item $outDir -Recurse -Force }
New-Item -ItemType Directory -Path $outDir | Out-Null

$common = @('--c99', '-c', '--cpu', 'Cortex-M4.fp', '--apcs=interwork', '-O0',
            '--diag_suppress=1', '--diag_suppress=1295') + $defines + $includes

$fail = @()
$i = 0
foreach ($src in $sources) {
    $i++
    $obj = Join-Path $outDir ("f{0:D3}.o" -f $i)
    $log = & $armcc @common "-o" $obj $src 2>&1
    $err = ($log | Where-Object { $_ -match ': error:|Fatal error' })
    if ($LASTEXITCODE -ne 0 -or $err) {
        $fail += [pscustomobject]@{ File = $src.Substring($projRoot.Length + 1); Log = $log }
    }
}

if (-not $Quiet) {
    foreach ($f in $fail) {
        Write-Host "FAIL $($f.File)" -ForegroundColor Red
        $f.Log | Select-Object -First 6 | ForEach-Object { Write-Host "    $_" }
    }
}
Write-Host ("----- AC5 checked {0} file(s), failures: {1} -----" -f $sources.Count, $fail.Count)
if ($fail.Count -gt 0) { exit 1 } else { exit 0 }
