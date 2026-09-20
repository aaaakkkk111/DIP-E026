# shift_test.ps1 -- 一次录完降档 / 升档 / 回降 的完整双向测试
#
#   powershell -ExecutionPolicy Bypass -File "<本文件绝对路径>"
#
# 输出存在本脚本旁边 shift_test.txt，不受 PowerShell 当前目录影响。

param(
    [string]$Out  = "",
    [int]   $Div  = 4,        # 每 4 拍记一行 = 50 Hz，避免丢包污染时间戳
    [string]$Port = "COM3",
    [int]   $Baud = 230400
)

$ErrorActionPreference = "Stop"
if ([string]::IsNullOrEmpty($Out)) { $Out = Join-Path $PSScriptRoot "shift_test.txt" }

Get-Process -ErrorAction SilentlyContinue |
    Where-Object { $_.Name -match 'UartAssist|flymcu|mcuisp' } |
    ForEach-Object { Write-Host ("[!] 请先关掉: " + $_.Name + " (PID " + $_.Id + ")") -ForegroundColor Red }

$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,'None',8,'One'
$sp.DtrEnable = $false; $sp.RtsEnable = $false; $sp.ReadBufferSize = 262144
try   { $sp.Open() } catch { Write-Host ("[X] 打开失败: " + $_.Exception.Message) -ForegroundColor Red; exit 1 }
Write-Host ("[OK] " + $Port + " @ " + $Baud) -ForegroundColor Green
Start-Sleep -Milliseconds 2500
Write-Host $sp.ReadExisting()

function Phase($sec, $label) {
    $t0 = Get-Date
    while (((Get-Date) - $t0).TotalSeconds -lt $sec) {
        if ($sp.BytesToRead -gt 0) { $script:sw.Write($sp.ReadExisting()) }
        Start-Sleep -Milliseconds 25
        $e = [int]((Get-Date) - $t0).TotalSeconds
        Write-Host -NoNewline ("`r    " + $label + "  " + $e + "/" + $sec + " 秒    ")
    }
    Write-Host ""
}

Write-Host ""
Write-Host "================================================" -ForegroundColor Cyan
Write-Host " 车上：1. 先按板子复位键（否则档位不是从 4 档开始）" -ForegroundColor Cyan
Write-Host "       2. 拧到 26.Adapt Load，按一下 KEY1，手松开" -ForegroundColor Cyan
Write-Host "       3. 先别按第二次！" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Read-Host "做完按回车"

$sp.Write("n -1`r`n"); Start-Sleep -Milliseconds 400
$st = $sp.ReadExisting(); Write-Host $st
if ($st -notmatch "mode=26") { Write-Host "[X] 不在 26 档" -ForegroundColor Red; $sp.Close(); exit 1 }
if ($st -match "boot=(\d+)") { $bl = $Matches[1] } else { $bl = "2" }
if ($st -notmatch ("lvl=" + $bl + "/")) {
    Write-Host ("[X] 档位不是开机值 " + $bl + " -- 没复位板子，降档段会测不到。按复位键后重跑。") -ForegroundColor Red
    $sp.Close(); exit 1
}
$sp.Write(("z " + $Div + "`r`n")); Start-Sleep -Milliseconds 400
Write-Host $sp.ReadExisting()

Write-Host ""
Write-Host "[!] 按回车后记录开始，然后立刻按第二次 KEY1，托住车" -ForegroundColor Red
Read-Host "准备好按回车"

$sp.ReadExisting() | Out-Null
$script:sw = New-Object System.IO.StreamWriter($Out, $false, [System.Text.Encoding]::ASCII)
$sp.Write("r`r`n")

Write-Host "[阶段1] 现在按第二次 KEY1 -- 测降档速度" -ForegroundColor Yellow
Phase 20 "阶段1 降档+空车稳定"

Write-Host ""
Write-Host "[阶段2] 把负重轻轻放上去，压中间别滑，手留在旁边" -ForegroundColor Yellow
Write-Host "        放好后直接等，不用按任何键" -ForegroundColor Yellow
Phase 30 "阶段2 升档+载重稳定"

Write-Host ""
Write-Host "[阶段3] 现在把负重拿下来" -ForegroundColor Yellow
Phase 25 "阶段3 回降+空车稳定"

$sp.Write("?`r`n"); Start-Sleep -Milliseconds 500
$fin = $sp.ReadExisting(); $script:sw.Write($fin)
$sp.Write("s`r`n")
$tail = ""
for ($i = 0; $i -lt 40; $i++) {
    Start-Sleep -Milliseconds 150
    if ($sp.BytesToRead -gt 0) { $tail += $sp.ReadExisting() }
    if ($tail -match "#end") { break }
}
$script:sw.Write($tail); $script:sw.Close(); $sp.Close()

Write-Host ""
Write-Host $fin
Write-Host $tail
Write-Host ("[OK] 已保存: " + (Resolve-Path $Out)) -ForegroundColor Green
