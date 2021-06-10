// NOTE: when working with ESP32 this define needs to be uncommented
#define BUIID_FOR_ESP32

using System;
using System.IO.Ports;
#if BUIID_FOR_ESP32
using nanoFramework.Hardware.Esp32;
using nanoFramework.Hardware.Esp32.Rmt;
#endif

namespace Driver.nf_SerialHC_SR04
{
    namespace Constants
    {
        /// <summary>
        /// Sensor available types
        /// </summary>
        public enum SensorType
        {
            JSN_SR04T = 0x55,
            AJ_SR04M = 0x01,
        }

        public enum Mode
        {
            Pulse,
            Pulse_LP,
            Serial_Auto,
            Serial_LP_Bin,
            Serial_LP_ASCII,
        }

        public enum Status
        {
            Ok,
            TimeOut,
            DataError,
            ConfigError,
            DataCheckError,
        }

        public static class Constant
        {
            public const float Speed_of_Sound = 340.29F;
        }
    }

    namespace Config
    {
        public class ConfigRMT
        {
            public ReceiverChannel _rxChannel;
            public TransmitterChannel _txChannel;
            public RmtCommand _txPulse;
            public RmtCommand[] response;
            public  void Config(int TxPin, int RxPin)
            {
                // Set-up TX & RX channels
                // We need to send a 10us pulse to initiate measurement
                _txChannel = new TransmitterChannel(TxPin);
                _txPulse = new RmtCommand(10, true, 10, false);
                _txChannel.AddCommand(_txPulse);
                _txChannel.AddCommand(new RmtCommand(20, true, 15, false));

                _txChannel.ClockDivider = 80;
                _txChannel.CarrierEnabled = false;
                _txChannel.IdleLevel = false;

                // The received echo pulse width represents the distance to obstacle
                // 150us to 38ms for HC-SR04
                _rxChannel = new ReceiverChannel(RxPin);

                _rxChannel.ClockDivider = 80; // 1us clock ( 80Mhz / 80 ) = 1Mhz
                _rxChannel.EnableFilter(true, 100); // filter out 100Us / noise 
                _rxChannel.SetIdleThresold(40000);  // 40ms based on 1us clock
                _rxChannel.ReceiveTimeout = new TimeSpan(0, 0, 0, 0, 60);
            }

        }

        public class ConfigSerial
        {
            public SerialPort Device;
            public void Config(string port)
            {
                Configuration.SetPinFunction(17, DeviceFunction.COM2_TX);
                Configuration.SetPinFunction(16, DeviceFunction.COM2_RX);

                Device = new SerialPort(port)
                {
                    // set parameters
                    BaudRate = 9600,
                    Parity = Parity.None,
                    StopBits = StopBits.One,
                    Handshake = Handshake.None,
                    DataBits = 8,
                    Mode = SerialMode.Normal,
                    // set Timout,
                    WriteTimeout = 1000,
                    ReadTimeout = 1000,
                    //WatchChar = 'X',      // \xff
                };
                Device.Open();

            }
        }
    }
}
