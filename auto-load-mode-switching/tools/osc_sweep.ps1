# osc_sweep.ps1 -- 逐档测抖动强度，给 26 档的降档阈值找依据
#
# 用法（PowerShell，先 cd 到本文件夹）：
#   .\osc_sweep.ps1
#   .\osc_sweep.ps1 -Hold 20        每档录 20 秒
#   .\osc_sweep.ps1 -MaxLevel 3     只测到 3 档（怕 4 档太猛）
#
# 运行前关掉 UartAssist 和 FlyMcu。

param(
    [string]$Out      = "osc_sweep.txt",
    [int]   $Hold     = 15,     # 每档记录秒数
    [int]   $Settle   = 4,      # 换档后先等几秒再开始记
    [int]   $MaxLevel = 4,
    [string]$Port     = "COM3",
    [int]   $Baud     = 230400
)

$ErrorActionPreference = "Stop"

Get-Process -ErrorAction SilentlyContinue |
    Where-Object { $_.Name -match 'UartAssist|flymcu|mcuisp' } |
    ForEach-Object { Write-Host ("[!] 请先关掉: " + $_.Name + " (PID " + $_.Id + ")") -ForegroundColor Red }

$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,'None',8,'One'
$sp.DtrEnable = $false; $sp.RtsEnable = $false; $sp.ReadBufferSize = 262144
try   { $sp.Open() }
catch { Write-Host ("[X] 打开 " + $Port + " 失败: " + $_.Exception.Message) -ForegroundColor Red; exit 1 }

function Send($c) { $sp.Write($c + "`r`n"); Start-Sleep -Milliseconds 400; return $sp.ReadExisting() }

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

$st = Send "?"
Write-Host $st
if ($st -notmatch "mode=26") {
    Write-Host "[X] 车不在 26 档。复位，拧到 26.Adapt Load，重跑本脚本。" -ForegroundColor Red
    $sp.Close(); exit 1
}

# 先锁到最轻档再让它启动，避免一上电就 19200
Write-Host (Send "n 0")
Write-Host "[OK] 已锁定 0 档（原厂增益，已知稳）" -ForegroundColor Green
Write-Host ""
Read-Host "现在扶正车、按第二次 KEY1 让它站稳，松手后按回车"

$sp.ReadExisting() | Out-Null
$sw = New-Object System.IO.StreamWriter($Out, $false, [System.Text.Encoding]::ASCII)
$sp.Write("r`r`n")
Start-Sleep -Milliseconds 500

$fell = $false
for ($lv = 0; $lv -le $MaxLevel; $lv++) {
    if ($fell) { break }
    Write-Host ""
    if ($lv -ge 2) {
        Write-Host ("[!] 下一档 " + $lv + " 会明显抖，双手扶住车（别按住，托着就行）") -ForegroundColor Red
        Read-Host "扶好按回车"
    }
    $sp.Write(("n " + $lv + "`r`n"))
    Write-Host ("--- 档位 " + $lv + "：先稳 " + $Settle + " 秒，再录 " + $Hold + " 秒 ---") -ForegroundColor Yellow

    $t0 = Get-Date
    $total = $Settle + $Hold
    while (((Get-Date) - $t0).TotalSeconds -lt $total) {
        if ($sp.BytesToRead -gt 0) {
            $chunk = $sp.ReadExisting()
            $sw.Write($chunk)
            if ($chunk -match "#FELL") { $fell = $true }
        }
        Start-Sleep -Milliseconds 30
        $el = [int]((Get-Date) - $t0).TotalSeconds
        $tag = if ($el -lt $Settle) { "稳定中" } else { "记录中" }
        Write-Host -NoNewline ("`r    " + $tag + " " + $el + "/" + $total + " 秒  " + $(if($fell){"[摔了]"}else{"      "}))
    }
    Write-Host ""
    if ($fell) { Write-Host ("[!] " + $lv + " 档摔了，停止上升") -ForegroundColor Red }
}

$sp.Write("n 0`r`n"); Start-Sleep -Milliseconds 300      # 回到安全档
$sp.Write("s`r`n")
$tail = ""
for ($i = 0; $i -lt 40; $i++) {
    Start-Sleep -Milliseconds 150
    if ($sp.BytesToRead -gt 0) { $tail += $sp.ReadExisting() }
    if ($tail -match "#end") { break }
}
$sw.Write($tail); $sw.Close(); $sp.Close()

Write-Host ""
Write-Host $tail
Write-Host ("[OK] 已保存: " + (Resolve-Path $Out)) -ForegroundColor Green
Write-Host "[OK] 已把档位拉回 0（原厂增益）" -ForegroundColor Green
if ($tail -match "dropped=(\d+)" -and [int]$Matches[1] -ne 0) {
    Write-Host ("[!] dropped=" + $Matches[1]) -ForegroundColor Red
}
