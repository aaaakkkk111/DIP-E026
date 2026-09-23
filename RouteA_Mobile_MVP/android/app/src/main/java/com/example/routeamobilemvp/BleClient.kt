package com.example.routeamobilemvp

import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.Context
import android.os.Handler
import android.os.Looper
import java.util.ArrayDeque
import java.util.UUID

/** JDY-23 transport matching the official BalanceBot app: FFE0 service and FFE1 read/write/notify. */
@SuppressLint("MissingPermission")
class BleClient(private val context: Context, private val events: Events) {
    interface Events {
        fun device(d: BluetoothDevice)
        fun status(s: String)
        fun ready()
        fun bytes(b: ByteArray)
        fun rssi(v: Int)
    }

    private val adapter =
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
    private val handler = Handler(Looper.getMainLooper())
    private var gatt: BluetoothGatt? = null
    private var io: BluetoothGattCharacteristic? = null
    private var notificationsReady = false
    private val writes = ArrayDeque<ByteArray>()
    private var writing = false

    private val service = UUID.fromString("0000ffe0-0000-1000-8000-00805f9b34fb")
    private val characteristic = UUID.fromString("0000ffe1-0000-1000-8000-00805f9b34fb")
    private val cccd = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    private val scan = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val name = result.device.name?.trim()
            val advertisesUart = result.scanRecord?.serviceUuids?.any { it.uuid == service } == true
            if (name.equals("YahBoom_BL", ignoreCase = true) || advertisesUart) {
                events.device(result.device)
            }
        }

        override fun onScanFailed(errorCode: Int) = events.status("scan failed=$errorCode")
    }

    fun scan() {
        val scanner = adapter.bluetoothLeScanner
        if (!adapter.isEnabled || scanner == null) {
            events.status("Bluetooth is disabled or BLE scanner is unavailable")
            return
        }
        scanner.startScan(scan)
        handler.removeCallbacksAndMessages(SCAN_TOKEN)
        handler.postAtTime(
            { stopScan(); events.status("scan complete") },
            SCAN_TOKEN,
            android.os.SystemClock.uptimeMillis() + 10_000
        )
        events.status("scanning for YahBoom_BL")
    }

    fun stopScan() {
        handler.removeCallbacksAndMessages(SCAN_TOKEN)
        adapter.bluetoothLeScanner?.stopScan(scan)
    }

    fun connect(device: BluetoothDevice) {
        stopScan()
        disconnectGatt(false)
        events.status("connecting ${device.address}")
        gatt = device.connectGatt(context, false, callback, BluetoothDevice.TRANSPORT_LE)
    }

    fun disconnect() = disconnectGatt(true)

    private fun disconnectGatt(report: Boolean) {
        notificationsReady = false
        writing = false
        writes.clear()
        io = null
        gatt?.disconnect()
        gatt?.close()
        gatt = null
        if (report) events.status("disconnected")
    }

    fun send(text: String) {
        if (!notificationsReady || gatt == null || io == null) {
            events.status("send rejected: BLE UART is not ready")
            return
        }
        // Use the default ATT MTU and the same FFE1 packet size expected by the official FastBle path.
        text.toByteArray(Charsets.US_ASCII).toList().chunked(20)
            .forEach { writes.add(it.toByteArray()) }
        pump()
    }

    private fun pump() {
        val g = gatt ?: return
        val c = io ?: return
        if (writing || writes.isEmpty()) return
        writing = true
        c.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        c.value = writes.removeFirst()
        if (!g.writeCharacteristic(c)) {
            writing = false
            writes.clear()
            events.status("write could not be started")
        }
    }

    private val callback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS &&
                newState == BluetoothProfile.STATE_CONNECTED
            ) {
                events.status("connected; discovering FFE0/FFE1")
                if (!g.discoverServices()) events.status("service discovery could not be started")
            } else {
                notificationsReady = false
                io = null
                events.status("disconnected status=$status state=$newState")
                g.close()
                if (gatt === g) gatt = null
            }
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                events.status("service discovery failed=$status")
                return
            }
            val c = g.getService(service)?.getCharacteristic(characteristic)
            val descriptor = c?.getDescriptor(cccd)
            if (c == null || descriptor == null) {
                events.status("FFE0/FFE1 or notification descriptor unavailable")
                return
            }
            io = c
            if (!g.setCharacteristicNotification(c, true)) {
                events.status("local notification enable failed")
                return
            }
            descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            if (!g.writeDescriptor(descriptor)) events.status("CCCD write could not be started")
        }

        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) {
            if (d.uuid != cccd) return
            if (status != BluetoothGatt.GATT_SUCCESS) {
                events.status("notification enable failed=$status")
                return
            }
            notificationsReady = true
            events.status("BLE UART ready: FFE0/FFE1 notifications enabled")
            events.ready()
            g.readRemoteRssi()
        }

        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            if (c.uuid == characteristic) events.bytes(c.value.copyOf())
        }

        override fun onCharacteristicWrite(
            g: BluetoothGatt,
            c: BluetoothGattCharacteristic,
            status: Int
        ) {
            writing = false
            if (status != BluetoothGatt.GATT_SUCCESS) {
                writes.clear()
                events.status("write failed=$status")
                return
            }
            pump()
        }

        override fun onReadRemoteRssi(g: BluetoothGatt, rssi: Int, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) events.rssi(rssi)
        }
    }

    private companion object { val SCAN_TOKEN = Any() }
}
