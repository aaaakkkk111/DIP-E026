[CmdletBinding()]
param(
    [string]$Address
)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Runtime.WindowsRuntime

[void][Windows.Devices.Bluetooth.BluetoothLEDevice,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Devices.Bluetooth.BluetoothCacheMode,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Devices.Bluetooth.Advertisement.BluetoothLEAdvertisementWatcher,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Devices.Bluetooth.Advertisement.BluetoothLEScanningMode,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Devices.Bluetooth.GenericAttributeProfile.GattSession,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]
[void][Windows.Devices.Bluetooth.GenericAttributeProfile.GattDeviceServicesResult,Windows.Devices.Bluetooth,ContentType=WindowsRuntime]

$script:AsTaskGeneric = [System.WindowsRuntimeSystemExtensions].GetMethods() |
    Where-Object {
        $_.Name -eq 'AsTask' -and $_.IsGenericMethod -and $_.GetParameters().Count -eq 1
    } |
    Select-Object -First 1

function Await-WinRT {
    param(
        [Parameter(Mandatory)]$Operation,
        [Parameter(Mandatory)][Type]$ResultType
    )
    $method = $script:AsTaskGeneric.MakeGenericMethod($ResultType)
    $task = $method.Invoke($null, @($Operation))
    try {
        $task.Wait()
    }
    catch {
        if ($null -ne $task.Exception) {
            throw $task.Exception.Flatten().InnerExceptions[0]
        }
        throw
    }
    return $task.Result
}

function ConvertTo-BluetoothAddress {
    param([Parameter(Mandatory)][string]$Text)
    return [Convert]::ToUInt64(($Text -replace '[:-]', ''), 16)
}

function Format-BluetoothAddress {
    param([Parameter(Mandatory)][UInt64]$Value)
    $parts = for ($index = 5; $index -ge 0; $index--) {
        '{0:X2}' -f (($Value -shr ($index * 8)) -band 0xff)
    }
    return $parts -join ':'
}

$device = $null
$session = $null
$servicesResult = $null
try {
    if (-not $Address) { throw 'A fresh BLE address is required. Run run-v2.cmd.' }
    $numericAddress = ConvertTo-BluetoothAddress $Address
    Write-Host ('Using address {0}' -f (Format-BluetoothAddress $numericAddress))

    $device = Await-WinRT `
        ([Windows.Devices.Bluetooth.BluetoothLEDevice]::FromBluetoothAddressAsync($numericAddress)) `
        ([Windows.Devices.Bluetooth.BluetoothLEDevice])
    if ($null -eq $device) { throw 'Windows returned a null BluetoothLEDevice.' }
    Write-Host ('Device="{0}" status={1}' -f $device.Name, $device.ConnectionStatus)

    $session = Await-WinRT `
        ([Windows.Devices.Bluetooth.GenericAttributeProfile.GattSession]::FromDeviceIdAsync($device.BluetoothDeviceId)) `
        ([Windows.Devices.Bluetooth.GenericAttributeProfile.GattSession])
    if ($null -eq $session) { throw 'Windows could not create a GattSession.' }
    $session.MaintainConnection = $true
    Write-Host 'GattSession.MaintainConnection=true'

    $serviceUuid = [Guid]'0000ffe0-0000-1000-8000-00805f9b34fb'
    Write-Host 'Requesting FFE0 with BluetoothCacheMode.Uncached...'
    $servicesResult = Await-WinRT `
        ($device.GetGattServicesForUuidAsync(
            $serviceUuid,
            [Windows.Devices.Bluetooth.BluetoothCacheMode]::Uncached)) `
        ([Windows.Devices.Bluetooth.GenericAttributeProfile.GattDeviceServicesResult])

    Write-Host ('Status={0}; Count={1}; ProtocolError={2}; Connection={3}' -f `
        $servicesResult.Status,
        $servicesResult.Services.Count,
        $servicesResult.ProtocolError,
        $device.ConnectionStatus)

    if ($servicesResult.Status -eq 'Success' -and $servicesResult.Services.Count -gt 0) {
        Write-Host -ForegroundColor Green 'PASS: Windows uncached service discovery found FFE0.'
        exit 0
    }

    Write-Host -ForegroundColor Red 'FAIL: Windows could not discover FFE0 even with MaintainConnection and Uncached mode.'
    exit 2
}
catch {
    Write-Host -ForegroundColor Red ('FATAL: ' + $_.Exception.Message)
    exit 1
}
finally {
    if ($null -ne $servicesResult) {
        foreach ($service in $servicesResult.Services) { $service.Dispose() }
    }
    if ($null -ne $session) { $session.Dispose() }
    if ($null -ne $device) { $device.Dispose() }
}
