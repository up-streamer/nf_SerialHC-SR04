﻿// NOTE: when working with ESP32 this define needs to be uncommented
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
#endif

namespace Driver.nf_Serial_HCSR04
{

    public class Serial_HCSR04
    {
		private readonly SensorType pingByte;
		static SerialDevice _serialDevice;
		readonly byte[] data = new byte[4];
		public event NativeEventHandler DataReceived;
		static DataWriter outputDataWriter;
		static DataReader inputDataReader;
		//// setup data writer for Serial Device output stream to ping device
		//DataWriter outputDataWriter = new DataWriter(_serialDevice.OutputStream);
		//// setup data read for Serial Device input stream to receive the distance
		//DataReader inputDataReader = new DataReader(_serialDevice.InputStream)
		//{
		//	InputStreamOptions = InputStreamOptions.Partial
		//};

		/// <summary>
		/// Constructor module
		/// </summary>
		public Serial_HCSR04(SensorType pingByte)
		{
            // Define Tx ping byte
            this.pingByte = pingByte;
			// get available ports
			var serialPorts = SerialDevice.GetDeviceSelector();
			Debug.WriteLine("Avail. Ports = " + serialPorts);

#if BUIID_FOR_ESP32
            ////////////////////////////////////////////////////////////////////////////////////////////////////
            // COM2 in ESP32-WROVER-KIT mapped to free GPIO pins
            // mind to NOT USE pins shared with other devices, like serial flash and PSRAM
            // also it's MANDATORY to set pin funcion to the appropriate COM before instanciating it

            // set GPIO functions for COM2 (this is UART2 on ESP32 WROOM32)
            Configuration.SetPinFunction(17, DeviceFunction.COM2_TX);
			Configuration.SetPinFunction(16, DeviceFunction.COM2_RX);
			
			serialPorts = SerialDevice.GetDeviceSelector();
			Debug.WriteLine("Avail. Ports after setPinfunction = " + serialPorts);
			// open COM2
			// _serialDevice = SerialDevice.FromId("COM2");
			ConfigPort("COM2");
#else
			///////////////////////////////////////////////////////////////////////////////////////////////////
			// COM6 in STM32F769IDiscovery board (Tx, Rx pins exposed in Arduino header CN13: TX->D1, RX->D0)
			// open COM6
			_serialDevice = SerialDevice.FromId("COM6");
#endif
		}

		private static void ConfigPort(string port)
		{
			//var serialPorts = SerialDevice.GetDeviceSelector();
			_serialDevice = SerialDevice.FromId(port);
			// set parameters
			_serialDevice.BaudRate = 9600;
			_serialDevice.Parity = SerialParity.None;
			_serialDevice.StopBits = SerialStopBitCount.One;
			_serialDevice.Handshake = SerialHandshake.None;
			_serialDevice.DataBits = 8;
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


		private void sensorPing(SensorType ping)
        {
			outputDataWriter.WriteByte((byte) ping);
			_ = outputDataWriter.Store();
		}

		public int GetDistance()
        {
			uint sum;
			uint dataCheck;
			int distanceByte;

			sensorPing(pingByte);
			Thread.Sleep(100);

			// Attempt to read 4 bytes from the Serial Device input stream
			// Format: 0XFF + H_DATA + L_DATA + SUM
			var bytesRead = inputDataReader.Load(_serialDevice.BytesToRead);
			Debug.WriteLine("Bytes Read = " + bytesRead);

			if (bytesRead == 4)
			{
				inputDataReader.ReadBytes(data);

				distanceByte = (data[1] << 8) | data[2];
				sum = data[3];
				dataCheck = (uint) (data[0] + data[1] + data[2] + 1) & 0x00ff;
				Debug.WriteLine("Datacheck = " + dataCheck.ToString() + " -- Sum = " + sum.ToString());
				Debug.WriteLine($"RX = {(byte)data[0] + " " + (byte)data[1] + " " + (byte)data[2] + " " + (byte)data[3]}");
				if (dataCheck == sum)
                {
					return distanceByte;
				}
			}
			return 0;
		}

		/// <summary>
		/// Serial data received event
		/// </summary>
		/// <param name="Sender"></param>
		/// <param name="EventData"></param>
		private void _Serial_DataReceived(object Sender, SerialDataReceivedEventArgs EventData)
		{
			int distanceValue;
			int i;
			// Attempt to read 4 bytes from the Serial Device input stream
			// Format: 0XFF + H_DATA + L_DATA + SUM
			var bytesRead = inputDataReader.Load(_serialDevice.BytesToRead);

			if (bytesRead == 4)
			{
				inputDataReader.ReadBytes(data);

				var distanceByte = (data[1] << 8) | data[2];
				var sum = data[3];
				var dataCheck = (data[0] + data[1] + data[2]) & 0x00ff;

				if (dataCheck == sum)
				{
					distanceValue = (int)(data[1] + data[2]);
					if (DataReceived != null)
					{
						DataReceived(0, 0, new DateTime());
					}
				}
				else
				{
					for (i = 0; i < 3; ++i) // Need correct loop sequence
					{
						sensorPing(pingByte);
					}
					// Handle Error!
				}
			}
		}
	}
}
