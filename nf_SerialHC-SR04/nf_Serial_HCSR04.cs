// NOTE: when working with ESP32 this define needs to be uncommented
#define BUIID_FOR_ESP32

using System.Threading;
using System.Diagnostics;
using System.IO.Ports;
using Driver.nf_Serial_HCSR04.Constants;
using Driver.nf_Serial_HCSR04.Config;

namespace Driver.nf_Serial_HCSR04
{
    public delegate void _NewValuesCallback(int distance, Status sta);

    public class Serial_HCSR04
    {
        byte[] data;
        uint sum;
        uint dataCheck;
        private readonly byte[] ping = new byte[1];
        private readonly string port;
        public readonly SensorType sensorType;
        public readonly Mode sensorMode;

        readonly ConfigSerial Serial = new();
        readonly ConfigRMT RMT = new();
        private delegate int GetDistanceDelegate();
        GetDistanceDelegate _GetDistance;
        private static Timer _GetDistanceTimer;

        private int TriggerPin = 0;
        private int EchoPin = 0;

        public Status status;

        private int distance;
        public int Distance
        {
            set { distance = value; }
            get { return distance; }
        }

        private int readInterval;
        public int ReadInterval
        {
            set
            {
                readInterval -= 100;
                readInterval = readInterval < 100 ? 100 : value;
                _GetDistanceTimer.Change(0, readInterval);
            }
            get
            {
                return readInterval;
            }
        }

        // * For debuging raw data from sensor *
        PrintRaw PR = new();

        /// <summary>
        /// Constructor module
        /// </summary>
        public Serial_HCSR04(SensorType type, Mode mode)
        {
            status = Status.Ok;
            // Define Tx ping byte
            sensorType = type;
            // Define Sensor mode
            sensorMode = mode;
            // Define ping byte accord sensorType
            ping[0] = (byte)sensorType;
            // Default ReadInterval to GetDistanceAUTO;
            readInterval = 1000;

            // List available ports
            var serialPorts = SerialPort.GetPortNames();

            foreach (string port in serialPorts)
            {
                Debug.WriteLine("Avail. Port - " + port);
            }
#if BUIID_FOR_ESP32
            ////////////////////////////////////////////////////////////////////////////////////////////////////
            // COM2 in ESP32-WROVER-KIT mapped to free GPIO pins
            // mind to NOT USE pins shared with other devices, like serial flash and PSRAM
            // also it's MANDATORY to set pin funcion to the appropriate COM before instanciating it

            // set GPIO functions for COM2 (this is UART2 on ESP32 WROOM32)
            //Configuration.SetPinFunction(17, DeviceFunction.COM2_TX);
            //Configuration.SetPinFunction(16, DeviceFunction.COM2_RX);

            // open COM2
            port = "COM2";
            ConfigPins(sensorMode);
            ConfigMode(sensorMode);
#else
			///////////////////////////////////////////////////////////////////////////////////////////////////
			// COM6 in STM32F769IDiscovery board (Tx, Rx pins exposed in Arduino header CN13: TX->D1, RX->D0)
			
			// open COM6
			port = "COM6";
			ConfigPins(sensorMode);
			ConfigMode(sensorMode);
#endif
        }

        private void ConfigPins(Mode sensorMode)
        {
            if ((sensorMode == Mode.Pulse) || (sensorMode == Mode.Pulse_LP))
            {
                // Change to suit your hardware set-up
                // if one of them is not configured, both will default to 16 and 17 
                if (TriggerPin == 0 || EchoPin == 0)
                {
                    TriggerPin = 17;
                    EchoPin = 16;
                }
                RMT.Config(TriggerPin, EchoPin);
            }
            else
            {
                if (sensorMode != Mode.Serial_Auto)
                {
                    Serial.Config(port);
                }
            }
        }

        private void ConfigMode(Mode sensorMode)
        {
            switch (sensorMode)
            {
#if BUIID_FOR_ESP32
                case Mode.Pulse:
                    _GetDistance = GetDistancePULSE;
                    return;

                case Mode.Pulse_LP:
                    _GetDistance = GetDistancePULSE;
                    return;
#endif
                case Mode.Serial_Auto:
                    GetDistanceAUTO gda = new(port, sensorType, NewValues);
                    _GetDistanceTimer = new Timer(gda.ThreadProcess, null, 0, readInterval);
                    return;

                case Mode.Serial_LP_Bin:
                    _GetDistance = GetDistanceBIN;
                    return;

                case Mode.Serial_LP_ASCII:
                    if (sensorType == SensorType.AJ_SR04M)
                    {
                        _GetDistance = GetDistanceASCII;
                    }
                    else
                    {
                        _GetDistance = GetDistanceBIN;
                    }
                    return;
            }

        }

        private void NewValues(int dist, Status sta)
        {
            distance = dist;
            status = sta;
        }

        public int GetDistance()
        {
            status = Status.Ok;
            SensorPing(ping);
            return _GetDistance();
        }

        private void SensorPing(byte[] ping)
        {
            if (TriggerPin == 0)
            {
                Serial.Device.Write(ping, 0, ping.Length);
                Thread.Sleep(100);
            }
            else
            {
                // Send 10us pulse
                RMT._txChannel.Send(true);
            }
        }

        private int GetDistancePULSE()
        {
            var response = RMT.response;
            RMT._rxChannel.Start(true);

            // Try 5 times to get valid response
            for (int count = 0; count < 5; count++)
            {
                response = RMT._rxChannel.GetAllItems();
                if (response != null)
                    break;

                // Retry every 60 ms
                Thread.Sleep(60);
            }

            RMT._rxChannel.Stop();

            if (response == null)
            {
                status = Status.TimeOut;
                return -1;
            }
            // Echo pulse width in micro seconds
            int duration = response[0].Duration0;

            // Calculate distance in milimeters
            // Distance calculated as  (speed of sound) * duration(miliseconds) / 2 
            return (int)Constant.Speed_of_Sound * duration / (1000 * 2);
        }

        private int GetDistanceBIN()
        {
            // Attempt to read 4 bytes from the Serial Device input stream
            // Format: 0XFF + H_DATA + L_DATA + SUM
            if (Serial.Device.BytesToRead == 4)
            {
                data = new byte[Serial.Device.BytesToRead];
                Serial.Device.Read(data, 0, data.Length);

                distance = (data[1] << 8) | data[2];
                sum = data[3];
                dataCheck = (uint)(data[0] + data[1] + data[2] + 1) & 0x00ff;

                // * For debuging data received *
                PR.ViewData(data, sum, dataCheck);

                if (dataCheck == sum)
                {
                    return distance;
                }
                status = Status.DataCheckError;
                return -1;
            }
            status = Status.DataError;
            return -1;
        }
        private int GetDistanceASCII()
        {
            // Attempt to read 12 bytes from the Serial Device input stream
            // For mode 5, computer printer mode (ASCII) with trigger
            // Data Stream format.										Note: Not in data sheet!!
            //RX = 71 97 112 61 49 56 56 49 109 109 13 10
            //      G  a   p  =  1  8  8  1   m   m CR LF

            if (Serial.Device.BytesToRead == 12)
            {
                data = new byte[Serial.Device.BytesToRead];
                Serial.Device.Read(data, 0, data.Length);

                distance = ((data[4] - 48) * 1000) + ((data[5] - 48) * 100) + ((data[6] - 48) * 10) + ((data[7] - 48));
                sum = (uint)data[0] + data[1] + data[2] + data[3] + data[8] + data[9] + data[10] + data[11];
                dataCheck = 582;

                    // * For debuging data received *
                    PR.ViewData(data, sum, dataCheck);

                if (dataCheck == sum)
                {
                    return distance;
                }
                status = Status.DataCheckError;
                return -1;
            }
            status = Status.DataError;
            return -1;
        }
    }

    class GetDistanceAUTO
    {
        private readonly _NewValuesCallback NewValues;
        readonly ConfigSerial Serial = new();
        readonly byte[] pingByte = new byte[1];
        readonly byte[] data = new byte[4]; //To save incoming bytes
        uint dataCheck;
        private int distance;
        uint sum;

        // * To debug raw data from sensor *
        PrintRaw PR = new PrintRaw();

        /// <summary>
        /// Constructor module
        /// </summary>
        public GetDistanceAUTO(string port, SensorType sensorType, _NewValuesCallback NV)
        {
            Serial.Config(port);
            pingByte[0] = (byte)sensorType;

            NewValues = NV;
        }

        public void ThreadProcess(object state)
        {
            Serial.Device.Write(pingByte, 0, pingByte.Length);
            Thread.Sleep(100);

            if (Serial.Device.BytesToRead == 4)
            {
                Serial.Device.Read(data, 0, data.Length);

                distance = (data[1] << 8) | data[2];
                sum = data[3];
                dataCheck = (uint)(data[0] + data[1] + data[2] + 1) & 0x00ff;

                PR.ViewData(data, sum, dataCheck);

                if (dataCheck == sum)
                {
                    NewValues?.Invoke(distance, Status.Ok);
                    Debug.WriteLine("distanceByte = " + distance.ToString());
                }
                else
                {
                    distance = -1;
                    NewValues?.Invoke(distance, Status.DataCheckError);
                }
            }
        }
    }

    public class PrintRaw
    {
        public void ViewData(byte[] data, uint sum, uint dataCheck)
        {
            if (data.Length == 4)
            {
                Debug.WriteLine("Datacheck = " + dataCheck.ToString() + " -- Sum = " + sum.ToString());

                Debug.WriteLine($"RX = {(byte)data[0] + " " + (byte)data[1] + " " + (byte)data[2] + " " + (byte)data[3]}");
            }
            else
            {
                Debug.WriteLine("Datacheck = " + dataCheck.ToString() + " -- Sum = " + sum.ToString());

                Debug.Write($"RX = {(byte)data[0] + " " + (byte)data[1] + " " + (byte)data[2] + " " + (byte)data[3] }");
                Debug.Write(" " + $"{(byte)data[4] + " " + (byte)data[5] + " " + (byte)data[6] + " " + (byte)data[7] }");
                Debug.WriteLine(" " + $"{(byte)data[8] + " " + (byte)data[9] + " " + (byte)data[10] + " " + (byte)data[11] }");
            }
        }
    }
}
