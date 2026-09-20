# auto_test.ps1 -- 验证 26 档的自动降档：开机 4 档，看它能不能自己降到 0 档
#
#   powershell -ExecutionPolicy Bypass -File "<本文件绝对路径>"
#
# 输出文件默认存在本脚本旁边，不受 PowerShell 当前目录影响。

param(
    [string]$Out     = "",
    [int]   $Seconds = 30,
    [string]$Port    = "COM3",
    [int]   $Baud    = 230400
)

$ErrorActionPreference = "Stop"
if ([string]::IsNullOrEmpty($Out)) { $Out = Join-Path $PSScriptRoot "auto_test.txt" }

Get-Process -ErrorAction SilentlyContinue |
    Where-Object { $_.Name -match 'UartAssist|flymcu|mcuisp' } |
    ForEach-Object { Write-Host ("[!] 请先关掉: " + $_.Name + " (PID " + $_.Id + ")") -ForegroundColor Red }

$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,'None',8,'One'
$sp.DtrEnable = $false; $sp.RtsEnable = $false; $sp.ReadBufferSize = 262144
try   { $sp.Open() }
catch { Write-Host ("[X] 打开 " + $Port + " 失败: " + $_.Exception.Message) -ForegroundColor Red; exit 1 }

Write-Host ("[OK] " + $Port + " @ " + $Baud) -ForegroundColor Green
Start-Sleep -Milliseconds 2500
Write-Host $sp.ReadExisting()

Write-Host ""
Write-Host "================================================" -ForegroundColor Cyan
Write-Host " 车上操作：" -ForegroundColor Cyan
Write-Host "   1. 拧轮子到 26.Adapt Load" -ForegroundColor Cyan
Write-Host "   2. 按一下 KEY1，手完全松开" -ForegroundColor Cyan
Write-Host "   3. 先别按第二次！" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Read-Host "做完前三步按回车"

$sp.Write("n -1`r`n"); Start-Sleep -Milliseconds 400
$st = $sp.ReadExisting()
Write-Host $st
if ($st -notmatch "mode=26") {
    Write-Host "[X] 车不在 26 档。复位、拧到 26.Adapt Load、重跑。" -ForegroundColor Red
    $sp.Close(); exit 1
}
Write-Host "[OK] 已设为自动模式 (force=-1)，档位保持开机的 4 档" -ForegroundColor Green

Write-Host ""
Write-Host "[!] 下一步车会从 4 档起步，开机瞬间会剧烈抖动。" -ForegroundColor Red
Write-Host "[!] 双手托住车，别按死。按预期它会在 2-3 秒内自己降到 0 档并安静。" -ForegroundColor Red
Write-Host ""
Read-Host "准备好按回车，然后立刻去按第二次 KEY1"

$sp.ReadExisting() | Out-Null
$sw = New-Object System.IO.StreamWriter($Out, $false, [System.Text.Encoding]::ASCII)
$sp.Write("r`r`n")

Write-Host "[..] 记录已开始 -- 现在去按第二次 KEY1 让车站起来" -ForegroundColor Yellow
$t0 = Get-Date; $bytes = 0; $fell = $false
while (((Get-Date) - $t0).TotalSeconds -lt $Seconds) {
    if ($sp.BytesToRead -gt 0) {
        $chunk = $sp.ReadExisting(); $bytes += $chunk.Length; $sw.Write($chunk)
        if ($chunk -match "#FELL") { $fell = $true }
    }
    Start-Sleep -Milliseconds 25
    $el = [int]((Get-Date) - $t0).TotalSeconds
    Write-Host -NoNewline ("`r     " + $el + "/" + $Seconds + " 秒   " + $bytes + " 字节   " + $(if($fell){"[摔了]"}else{"       "}))
}
Write-Host ""

$sp.Write("?`r`n"); Start-Sleep -Milliseconds 400
$final = $sp.ReadExisting(); $sw.Write($final)
$sp.Write("s`r`n")
$tail = ""
for ($i = 0; $i -lt 40; $i++) {
    Start-Sleep -Milliseconds 150
    if ($sp.BytesToRead -gt 0) { $tail += $sp.ReadExisting() }
    if ($tail -match "#end") { break }
}
$sw.Write($tail); $sw.Close(); $sp.Close()

Write-Host ""
Write-Host "--- 结束时状态 ---" -ForegroundColor Yellow
Write-Host $final
Write-Host $tail
Write-Host ("[OK] 已保存: " + (Resolve-Path $Out)) -ForegroundColor Green
if ($final -match "lvl=(\d+)") { Write-Host ("[i] 最终停在 " + $Matches[1] + " 档") -ForegroundColor Cyan }
if ($fell) { Write-Host "[!] 过程中摔过，数据仍有用" -ForegroundColor Yellow }
