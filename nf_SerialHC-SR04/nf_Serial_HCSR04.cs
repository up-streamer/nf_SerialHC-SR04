using System;
using System.Threading;
using System.Diagnostics;
using Windows.Storage.Streams;
using Windows.Devices.SerialCommunication;
using nanoFramework.Hardware.Esp32;
using Driver.nf_SerialHC_SR04.Constants;


namespace Driver.nf_Serial_HCSR04
{

	public class nf_Serial_HCSR04 
	{
		private readonly SensorType pingByte;
		static SerialDevice _serialDevice;
		
		// setup data writer for Serial Device output stream to ping device
		DataWriter outputDataWriter = new DataWriter(_serialDevice.OutputStream);
		// setup data read for Serial Device input stream to receive the distance
		DataReader inputDataReader = new DataReader(_serialDevice.InputStream)
		{
			InputStreamOptions = InputStreamOptions.Partial
		};

		/// <summary>
		/// Constructor module
		/// </summary>
		public nf_Serial_HCSR04(SensorType pingByte)
		{	
			// Define Tx ping byte
			this.pingByte = pingByte;
			// get available ports
			var serialPorts = SerialDevice.GetDeviceSelector();
			// set GPIO functions for COM2 (this is UART1 on ESP32)
			Configuration.SetPinFunction(Gpio.IO04, DeviceFunction.COM2_TX);
			Configuration.SetPinFunction(Gpio.IO05, DeviceFunction.COM2_RX);
			// open COM2
			_serialDevice = SerialDevice.FromId("COM2");
			// set parameters
			_serialDevice.BaudRate = 9600;
			_serialDevice.Parity = SerialParity.None;
			_serialDevice.StopBits = SerialStopBitCount.One;
			_serialDevice.Handshake = SerialHandshake.None;
			_serialDevice.DataBits = 8;
			// set Timouts
			_serialDevice.WriteTimeout = new TimeSpan(0, 0, 0, 500);
			_serialDevice.ReadTimeout = new TimeSpan (0, 0, 0, 500);
		}

		private void sensorPing(SensorType ping)
        {
			outputDataWriter.WriteByte((byte) ping);
			//Thread.Sleep(50);
        }

		public int getDistance()
        {
			sensorPing(pingByte);
			byte[] data = new byte[4];
			int distanceValue;
			// Attempt to read 4 bytes from the Serial Device input stream
			// Format: 0XFF + H_DATA + L_DATA + SUM
			var bytesRead = inputDataReader.Load(4);

			if (bytesRead > 0)
			{
				inputDataReader.ReadBytes(data);

				var distanceByte = (data[1] << 8) | data[2];
				var sum = data[3];
				var dataCheck = (data[0] + data[1] + data[2]) & 0x00ff;

				if (dataCheck == sum)
                {
					distanceValue = (int)(data[1] + data[2]);
					return distanceValue;
				}
			}
				return 0;
		}
	}
}
