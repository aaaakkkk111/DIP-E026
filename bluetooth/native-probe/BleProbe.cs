using System;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.Advertisement;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Storage.Streams;

internal static class BleProbe
{
    private static readonly Guid ServiceUuid = Guid.Parse("0000ffe0-0000-1000-8000-00805f9b34fb");
    private static readonly Guid CharacteristicUuid = Guid.Parse("0000ffe1-0000-1000-8000-00805f9b34fb");

    [MTAThread]
    private static void Main()
    {
        try
        {
            RunAsync().GetAwaiter().GetResult();
        }
        catch (Exception error)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine("FATAL: " + error.GetType().Name + ": " + error.Message);
            Console.ResetColor();
            Console.WriteLine("Press Enter to exit.");
            Console.ReadLine();
            Environment.ExitCode = 1;
        }
    }

    private static async Task RunAsync()
    {
        Console.OutputEncoding = Encoding.UTF8;
        Console.WriteLine("YahBoom / JDY-23 Windows native BLE probe");
        Console.WriteLine("Close the Android app and turn off phone Bluetooth first.");
        Console.WriteLine("Scanning for 20 seconds...");

        var found = new TaskCompletionSource<Tuple<ulong, string>>();
        var watcher = new BluetoothLEAdvertisementWatcher
        {
            ScanningMode = BluetoothLEScanningMode.Active
        };

        watcher.Received += (sender, args) =>
        {
            var name = args.Advertisement.LocalName ?? "";
            if (name.IndexOf("YahBoom", StringComparison.OrdinalIgnoreCase) >= 0 ||
                name.IndexOf("JDY", StringComparison.OrdinalIgnoreCase) >= 0)
            {
                found.TrySetResult(Tuple.Create(args.BluetoothAddress, name));
            }
        };

        watcher.Start();
        var completed = await Task.WhenAny(found.Task, Task.Delay(TimeSpan.FromSeconds(20)));
        watcher.Stop();
        if (completed != found.Task)
            throw new TimeoutException("No YahBoom/JDY advertisement was found.");

        var match = await found.Task;
        Console.WriteLine("Found: " + Escape(match.Item2) + "  address=" + FormatAddress(match.Item1));
        Console.WriteLine("Creating Windows BluetoothLEDevice...");

        var device = await BluetoothLEDevice.FromBluetoothAddressAsync(match.Item1).AsTask();
        if (device == null)
            throw new InvalidOperationException("Windows returned a null BluetoothLEDevice.");

        using (device)
        {
            device.ConnectionStatusChanged += (sender, args) =>
                Console.WriteLine("ConnectionStatusChanged: " + sender.ConnectionStatus);

            Console.WriteLine("Device name: " + Escape(device.Name));
            Console.WriteLine("Initial status: " + device.ConnectionStatus);

                Console.WriteLine("Reading GATT services through the Windows native API...");

                var services = device.GattServices;
                Console.WriteLine("Service discovery count: " + services.Count);
                if (services.Count == 0)
                    throw new InvalidOperationException("Windows returned no GATT services.");

                var service = services.FirstOrDefault(item => item.Uuid == ServiceUuid);
                if (service == null)
                    throw new InvalidOperationException("GATT connected, but FFE0 was not found.");
                using (service)
                {
                    Console.WriteLine("FFE0 found. Reading characteristics...");
                    var characteristics = service.GetAllCharacteristics();
                    Console.WriteLine("Characteristic discovery count: " + characteristics.Count);
                    if (characteristics.Count == 0)
                        throw new InvalidOperationException("Windows returned no FFE0 characteristics.");

                    var characteristic = characteristics.FirstOrDefault(
                        item => item.Uuid == CharacteristicUuid);
                    if (characteristic == null)
                        throw new InvalidOperationException("GATT connected, but FFE1 was not found.");
                    Console.WriteLine("FFE1 properties: " + characteristic.CharacteristicProperties);

                    var echoReceived = new TaskCompletionSource<bool>();
                    var token = "PC-NATIVE-ECHO-" + DateTime.UtcNow.Ticks.ToString("x") + "\r\n";
                    var received = new StringBuilder();
                    characteristic.ValueChanged += (sender, args) =>
                    {
                        var reader = DataReader.FromBuffer(args.CharacteristicValue);
                        var bytes = new byte[reader.UnconsumedBufferLength];
                        reader.ReadBytes(bytes);
                        var text = Encoding.UTF8.GetString(bytes);
                        received.Append(text);
                        Console.WriteLine("RX " + bytes.Length + " B: " + Escape(text));
                        if (received.ToString().Contains(token)) echoReceived.TrySetResult(true);
                    };

                    var notifyStatus = await characteristic.WriteClientCharacteristicConfigurationDescriptorAsync(
                        GattClientCharacteristicConfigurationDescriptorValue.Notify).AsTask();
                    Console.WriteLine("Enable notifications: " + notifyStatus);
                    if (notifyStatus != GattCommunicationStatus.Success)
                        throw new InvalidOperationException("Could not enable FFE1 notifications: " + notifyStatus);

                    var payload = Encoding.UTF8.GetBytes(token);
                    Console.WriteLine("TX " + payload.Length + " B: " + Escape(token));
                    for (var offset = 0; offset < payload.Length; offset += 20)
                    {
                        var count = Math.Min(20, payload.Length - offset);
                        var writer = new DataWriter();
                        writer.WriteBytes(payload.Skip(offset).Take(count).ToArray());
                        var buffer = writer.DetachBuffer();
                        writer.Dispose();
                        var writeStatus = await characteristic.WriteValueAsync(
                            buffer, GattWriteOption.WriteWithoutResponse).AsTask();
                        Console.WriteLine("Write chunk: " + writeStatus);
                        if (writeStatus != GattCommunicationStatus.Success)
                            throw new InvalidOperationException("FFE1 write failed with status " + writeStatus + ".");
                        await Task.Delay(25);
                    }

                    var echoCompleted = await Task.WhenAny(echoReceived.Task, Task.Delay(TimeSpan.FromSeconds(8)));
                    if (echoCompleted == echoReceived.Task)
                    {
                        Console.ForegroundColor = ConsoleColor.Green;
                        Console.WriteLine("PASS: native Windows GATT and STM32 UART5 echo both work.");
                    }
                    else
                    {
                        Console.ForegroundColor = ConsoleColor.Yellow;
                        Console.WriteLine("PARTIAL: GATT works, but the UART5 echo was not received in 8 seconds.");
                    }
                    Console.ResetColor();
                }
        }

        Console.WriteLine("Press Enter to exit.");
        Console.ReadLine();
    }

    private static string Escape(string text)
    {
        return "\"" + (text ?? "").Replace("\\", "\\\\").Replace("\r", "\\r").Replace("\n", "\\n") + "\"";
    }

    private static string FormatAddress(ulong address)
    {
        return string.Join(":", Enumerable.Range(0, 6)
            .Select(index => ((address >> ((5 - index) * 8)) & 0xff).ToString("X2")));
    }
}
