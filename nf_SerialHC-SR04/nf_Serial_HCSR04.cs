// NOTE: when working with ESP32 this define needs to be uncommented
#define BUIID_FOR_ESP32

using System;
using System.Threading;
using System.Diagnostics;
using Windows.Storage.Streams;
using Windows.Devices.SerialCommunication;
using Windows.Devices.Gpio;
using nanoFramework.Runtime.Events; /// New. Needs Packages
using Driver.nf_SerialHC_SR04.Constants;
#if BUIID_FOR_ESP32
using nanoFramework.Hardware.Esp32;
using nanoFramework.Hardware.Esp32.Rmt;
#endif

namespace Driver.nf_Serial_HCSR04
{

    public class Serial_HCSR04
    {
		static SerialDevice _serialDevice;
		byte[] data; //To save incoming bytes
		public event NativeEventHandler DataReceived;
		// setup data writer for Serial Device output stream to ping device
		static DataWriter outputDataWriter;
		// setup data read for Serial Device input stream to receive the distance
		static DataReader inputDataReader;
		public SensorType sensorType;
		public readonly Mode sensorMode;
		public int Distance;
		private delegate int GetDistanceDelegate();
		private GetDistanceDelegate _GetDistance;
		uint sum;
		uint dataCheck;
		int distanceByte;
		uint bytesRead;

		ReceiverChannel _rxChannel;
		TransmitterChannel _txChannel;
		RmtCommand _txPulse;

		public int TriggerPin = 0;
		public int EchoPin = 0;
		const float _speedOfSound = 340.29F;
		public Status status;

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
			// get available ports
			var serialPorts = SerialDevice.GetDeviceSelector();
			Debug.WriteLine("Avail. Ports = " + serialPorts);

#if BUIID_FOR_ESP32
			////////////////////////////////////////////////////////////////////////////////////////////////////
			// COM2 in ESP32-WROVER-KIT mapped to free GPIO pins
			// mind to NOT USE pins shared with other devices, like serial flash and PSRAM
			// also it's MANDATORY to set pin funcion to the appropriate COM before instanciating it

			// set GPIO functions for COM2 (this is UART2 on ESP32 WROOM32)
			//Configuration.SetPinFunction(17, DeviceFunction.COM2_TX);
			//Configuration.SetPinFunction(16, DeviceFunction.COM2_RX);

			// open COM2
			ConfigPins(sensorMode);

			ConfigMode(sensorMode);
#else
			///////////////////////////////////////////////////////////////////////////////////////////////////
			// COM6 in STM32F769IDiscovery board (Tx, Rx pins exposed in Arduino header CN13: TX->D1, RX->D0)
			
			// open COM6
			ConfigPins(sensorMode);

			ConfigMode(sensorMode);
#endif
		}

		private void ConfigPins(Mode sensorMode)
		{
			if ((sensorMode == Mode.Pulse) || (sensorMode == Mode.Pulse_LP))
			{
				// Change to suit you hardware set-up
				// if one of them is not configured, both will default to 16 and 17
				if (TriggerPin == 0 || EchoPin == 0)
				{ 
					TriggerPin = 17;
					EchoPin = 16;
				}
				ConfigRMT(TriggerPin, EchoPin);
			}
			else
			{
				// set GPIO functions for COM2 (this is UART2 on ESP32 WROOM32)
				Configuration.SetPinFunction(17, DeviceFunction.COM2_TX);
				Configuration.SetPinFunction(16, DeviceFunction.COM2_RX);
#if BUIID_FOR_ESP32
				ConfigPort("COM2");
#else
				ConfigPort("COM6");
#endif
			}
		}

		private void ConfigRMT(int TxPin, int RxPin)
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
			// 150us to 38ms
			_rxChannel = new ReceiverChannel(RxPin);

			_rxChannel.ClockDivider = 80; // 1us clock ( 80Mhz / 80 ) = 1Mhz
			_rxChannel.EnableFilter(true, 100); // filter out 100Us / noise 
			_rxChannel.SetIdleThresold(40000);  // 40ms based on 1us clock
			_rxChannel.ReceiveTimeout = new TimeSpan(0, 0, 0, 0, 60);
		}
		private static void ConfigPort(string port)
		{
			_serialDevice = SerialDevice.FromId(port);
			// set parameters
			_serialDevice.BaudRate = 9600;
			_serialDevice.Parity = SerialParity.None;
			_serialDevice.StopBits = SerialStopBitCount.One;
			_serialDevice.Handshake = SerialHandshake.None;
			_serialDevice.DataBits = 8;
			_serialDevice.WatchChar = '\xff';
			// set Timouts
			//_serialDevice.WriteTimeout = new TimeSpan(0, 0, 0, 500);
			//_serialDevice.ReadTimeout = new TimeSpan(0, 0, 0, 500);

			// setup data writer for Serial Device output stream to ping device
			outputDataWriter = new DataWriter(_serialDevice.OutputStream);
			// setup data read for Serial Device input stream to receive the distance
			inputDataReader = new DataReader(_serialDevice.InputStream)
			{
				InputStreamOptions = InputStreamOptions.Partial
			};
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
					_serialDevice.DataReceived += GetDistanceAUTO;
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
		public int GetDistance()
        {
			status = Status.Ok;
			SensorPing(sensorType);
			return _GetDistance();
		}

		private void SensorPing(SensorType ping)
		{	if (TriggerPin == 0)
			{
				outputDataWriter.WriteByte((byte)ping);
				_ = outputDataWriter.Store();
				Thread.Sleep(100);
			}
			else
            {

				// Send 10us pulse
				_txChannel.Send(true);
			}
		}

		private int GetDistancePULSE()
		{
			RmtCommand[] response = null;

			_rxChannel.Start(true);

			// Try 5 times to get valid response
			for (int count = 0; count < 5; count++)
			{
				response = _rxChannel.GetAllItems();
				if (response != null)
					break;

				// Retry every 60 ms
				Thread.Sleep(60);
			}

			_rxChannel.Stop();

			if (response == null)
			{
				status = Status.TimeOut;
				return -1;
			}
			// Echo pulse width in micro seconds
			int duration = response[0].Duration0;

			// Calculate distance in milimeters
			// Distance calculated as  (speed of sound) * duration(miliseconds) / 2 
			return (int)_speedOfSound * duration / (1000 * 2);
		}

		public int GetDistanceBIN()
			{
				// Attempt to read 4 bytes from the Serial Device input stream
				// Format: 0XFF + H_DATA + L_DATA + SUM
				bytesRead = inputDataReader.Load(_serialDevice.BytesToRead);
				Debug.WriteLine("Bytes Read = " + bytesRead);

				if (bytesRead == 4) //For modes Serial_Auto, Serial_LP_Bin, serial binary with trigger
				{
					data = new byte[bytesRead];
					inputDataReader.ReadBytes(data);

					distanceByte = (data[1] << 8) | data[2];
					sum = data[3];
					dataCheck = (uint)(data[0] + data[1] + data[2] + 1) & 0x00ff;
					Debug.WriteLine("Datacheck = " + dataCheck.ToString() + " -- Sum = " + sum.ToString());

					// For mode 4, serial binary with trigger
					Debug.WriteLine($"RX = {(byte)data[0] + " " + (byte)data[1] + " " + (byte)data[2] + " " + (byte)data[3]}");

					if (dataCheck == sum)
					{
						return distanceByte;
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
			bytesRead = inputDataReader.Load(_serialDevice.BytesToRead);
			Debug.WriteLine("Bytes Read = " + bytesRead);

			if (bytesRead == 12)
			{
				data = new byte[bytesRead];
				inputDataReader.ReadBytes(data);

				distanceByte = ((data[4]-48)*1000) + ((data[5]-48) * 100) + ((data[6]-48) * 10) + ((data[7] - 48));
				sum =(uint) data[0] + data[1] + data[2] + data[3] + data[8] + data[8] + data[10] + data[11];
				dataCheck = 582;
				Debug.WriteLine("Datacheck = " + dataCheck.ToString() + " -- Sum = " + sum.ToString());
				
				//Quick and dirt check to data received 

				Debug.Write($"RX = {(byte)data[0] + " " + (byte)data[1] + " " + (byte)data[2] + " " + (byte)data[3] }");
				Debug.Write(" " + $"{(byte)data[4] + " " + (byte)data[5] + " " + (byte)data[6] + " " + (byte)data[7] }");
				Debug.WriteLine(" " + $"{(byte)data[8] + " " + (byte)data[9] + " " + (byte)data[10] + " " + (byte)data[11] }");
				if (dataCheck == sum)
				{
					return distanceByte;
				}
				status = Status.DataCheckError;
				return -1;
			}
			status = Status.DataError;
			return -1;

		}

		/// <summary>
		/// Serial data received event
		/// </summary>
		/// <param name="Sender"></param>
		/// <param name="EventData"></param>
		int count = 0;
		bool watchCharRecv;
		byte [] dataauto = new byte[4];
		private void GetDistanceAUTO(object sender, SerialDataReceivedEventArgs e)
		{
			//bytesRead = inputDataReader.Load(_serialDevice.BytesToRead);

			//if (bytesRead == 4)
			//{
			//	Distance = GetDistanceBIN();

			//	DataReceived(0, 0, new DateTime());
			//}
		
			count++;
			Debug.WriteLine(count.ToString());


			// Attempt to read 4 bytes from the Serial Device input stream
			// Format: 0XFF + H_DATA + L_DATA + SUM

			//*************************
			if (e.EventType == SerialData.WatchChar)
			{
				Debug.WriteLine("rx chars");
				watchCharRecv = true;
				//dataauto[0] = 0xff;
			}
			if (watchCharRecv)
			{
				Debug.WriteLine("rx data");

				using (inputDataReader)
				{
					//SerialDevice serialDevice = (SerialDevice)sender;


					//inputDataReader.InputStreamOptions = InputStreamOptions.Partial;

					// read all available bytes from the Serial Device input stream
					bytesRead = inputDataReader.Load(_serialDevice.BytesToRead);

					Debug.WriteLine("Read completed: " + bytesRead + " bytes were read from " + _serialDevice.PortName + ".");

					if (bytesRead > 4)
					{
						inputDataReader.ReadBytes(data);
						

						Debug.WriteLine($"RX = {(byte)data[0] + " " + (byte)data[1] + " " + (byte)data[2] + " " + (byte)data[3]}");
					}
					watchCharRecv = false;
				}
			}
            //************************

            #region Old code
            //         Debug.WriteLine("Bytes to Read on Buffer = " + _serialDevice.BytesToRead);
            //bytesRead = inputDataReader.Load(_serialDevice.BytesToRead);
            //Debug.WriteLine("Bytes Read = " + bytesRead);


            //if (bytesRead == 4) //For modes Serial_Auto, Serial_LP_Bin, serial binary with trigger
            //{
            //	data = new byte[bytesRead];
            //	inputDataReader.ReadBytes(data);

            //	distanceByte = (data[1] << 8) | data[2];
            //	sum = data[3];
            //	dataCheck = (uint)(data[0] + data[1] + data[2] + 1) & 0x00ff;
            //	Debug.WriteLine("Datacheck = " + dataCheck.ToString() + " -- Sum = " + sum.ToString());

            //	// For mode 4, serial binary with trigger
            //	Debug.WriteLine($"RX = {(byte)data[0] + " " + (byte)data[1] + " " + (byte)data[2] + " " + (byte)data[3]}");

            //	if (dataCheck == sum)
            //	{
            //		Distance =  distanceByte;
            //		DataReceived(0, 0, new DateTime());
            //	}
            //	status = Status.DataCheckError;
            //}

            //status = Status.DataError;
            #endregion
        }
    }
}
