using System;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Bluetooth.Advertisement;

internal static class BleScan
{
    [MTAThread]
    private static void Main()
    {
        Console.OutputEncoding = Encoding.UTF8;
        try
        {
            RunAsync().GetAwaiter().GetResult();
        }
        catch (Exception error)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine("SCAN_FAIL: " + error.Message);
            Console.ResetColor();
            Environment.ExitCode = 1;
        }
    }

    private static async Task RunAsync()
    {
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
                found.TrySetResult(Tuple.Create(args.BluetoothAddress, name));
        };

        Console.WriteLine("Scanning for YahBoom/JDY for 20 seconds...");
        watcher.Start();
        var completed = await Task.WhenAny(found.Task, Task.Delay(TimeSpan.FromSeconds(20)));
        watcher.Stop();
        if (completed != found.Task) throw new TimeoutException("No YahBoom/JDY advertisement was found.");

        var match = await found.Task;
        var address = FormatAddress(match.Item1);
        File.WriteAllText(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "last-address.txt"), address);
        Console.WriteLine("Found: \"" + match.Item2 + "\" address=" + address);
    }

    private static string FormatAddress(ulong address)
    {
        var parts = new string[6];
        for (var index = 0; index < 6; index++)
            parts[index] = ((address >> ((5 - index) * 8)) & 0xff).ToString("X2");
        return string.Join(":", parts);
    }
}
