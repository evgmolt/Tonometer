using Microsoft.Win32;
using System.IO.Ports;
using System.Management;

namespace TTestApp
{
    public interface IMessageHandler
    {
        event Action<Message> WindowsMessageHandler;
    }

    public class USBSerialPort
    {
        private const int _USBTimerInterval = 25;
        public SerialPort PortHandle;
        public string[] PortNames;
        public byte[] PortBuf;
        public int CurrentPort;
        public int BytesRead;
        public Boolean ReadEnabled;
        private readonly System.Threading.Timer ReadTimer;

        private readonly int _portBufSize = 10000;
        private readonly int _baudRate;

        public event Action<Exception> ConnectionFailure;
        public event Action ConnectionOk;

        private readonly string _connectionString;

        public USBSerialPort(IMessageHandler messageHandler, int baudRate, string connectionString)
        {
            _baudRate = baudRate;
            _connectionString = connectionString;
            messageHandler.WindowsMessageHandler += OnMessage;
            ReadEnabled = false;
            PortBuf = new byte[_portBufSize];
            ReadTimer = new System.Threading.Timer(ReadPort, null, 0, Timeout.Infinite);
        }

        private void ReadPort(object state)
        {
            if (BytesRead > 0) return;
            if (!ReadEnabled) return;
            if (PortHandle == null) return;
            if (PortHandle.IsOpen)
            {
                if (PortHandle.BytesToRead > 0)
                {
                    int bytesToRead = PortHandle.BytesToRead;
                    if (bytesToRead > PortBuf.Length)
                    {
                        bytesToRead = PortBuf.Length;
                    }
                    try
                    {
                        BytesRead = PortHandle.Read(PortBuf, 0, bytesToRead);
                    }
                    catch (Exception)
                    {
                    }
                }
            }
        }

        public bool WriteBuf(byte[] buf)
        {
            try
            {
                PortHandle?.Write(buf, 0, buf.Length);
                return true;
            }
            catch (Exception)
            {
                return false;
            }
        }

        public bool WriteByte(byte b)
        {
            byte[] buf = { b }; // new byte[1];
            try
            {
                PortHandle?.Write(buf, 0, 1);
                return true;
            }
            catch (Exception)
            {
                return false;
            }
        }

        private string GetUSBSerialPortName()
        {
            string portName;
            string usbSerialString = "USB-SERIAL";
            ManagementObjectCollection collection = null;
            try
            {
                using (var searcher = new ManagementObjectSearcher(
                        "root\\CIMV2",
                        @"Select Caption,DeviceID,PnpClass From Win32_PnpEntity WHERE DeviceID like '%USB%'")) 
                    collection = searcher.Get();
            }
            catch (Exception)
            {
                portName = "";
            }
            List<string> list = new();
            if (collection != null)
            {
                foreach (var device in collection)
                {
                    foreach (var p in device.Properties)
                    {
                        if (p.Value != null)
                        {
                            list.Add(p.Value.ToString());
                        }
                    }
                }
            }
            try
            {
                portName = list.Where(p => p.IndexOf(usbSerialString) >= 0).First();
            }
            catch (Exception)
            {
                portName = "";
            }
            return portName;
        }

        public void Connect()
        {
            PortNames = GetPortsNames();
            if (PortNames == null) return;
            for (int i = 0; i < PortNames.Length; i++)
            {
                PortHandle = new SerialPort(PortNames[i], _baudRate)
                {
                    DataBits = 8,
                    Parity = Parity.None,
                    StopBits = StopBits.One
                };
                try
                {
                    PortHandle.Open();
                    ReadEnabled = true;
                    ReadTimer.Change(0, _USBTimerInterval);
                    CurrentPort = i;
                    ConnectionOk?.Invoke();
                    break;
                }
                catch (Exception e)
                {
                    ConnectionFailure?.Invoke(e);
                }
            }
        }

        private string[]? GetPortsNames()
        {
            const string ArduinoSerialString0 = "Serial0";

            RegistryKey r_hklm   = Registry.LocalMachine;
            RegistryKey r_hard   = r_hklm.OpenSubKey("HARDWARE");
            RegistryKey r_device = r_hard.OpenSubKey("DEVICEMAP");
            RegistryKey r_port   = r_device.OpenSubKey("SERIALCOMM");
            if (r_port == null) 
            {
                return null;
            }
            string[] portvalues = r_port.GetValueNames();
            List<string> portNames = new List<string>();
            for (int i = 0; i < portvalues.Length; i++)
            {
                if (portvalues[i].IndexOf(_connectionString) >= 0 && portvalues[i].IndexOf(ArduinoSerialString0) < 0)
                {
                    portNames.Add((string)r_port.GetValue(portvalues[i]));
                }
            }
            return portNames.ToArray();
        }

        private void OnMessage(Message m)
        {
            if (PortHandle == null)
            {
                BytesRead = 0;
                Connect();
            }
            else
            if (!PortHandle.IsOpen)
            {
                BytesRead = 0;
                Connect();
            }
        }
    }
}
