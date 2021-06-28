using System;
using System.Diagnostics;
using System.Threading;
using Driver.nf_Serial_HCSR04;
using Driver.nf_Serial_HCSR04.Constants;

namespace testnf_Serial_HCSR04
{
    public class Program
    {
        private static Serial_HCSR04 sensor;
        public static void Main()
        {
            sensor = new Serial_HCSR04(SensorType.AJ_SR04M, Mode.Serial_Auto);
            sensor.ReadInterval = 1000;
            
            // * To test receive data on demand, by pinging the device. *
            //for (int i = 0; i < 500; i++)
            //{
            //    int distance = sensor.GetDistance();
            //    Debug.WriteLine($"distance = {distance} mm" + $"--> count = {i}");
            //    Thread.Sleep(1000);
            //}

            // * To test receive data when in auto mode, new data got from Timer thread.* 
            for (int i = 0; i < 500; i++)
            {
                Debug.WriteLine($"distance = {sensor.Distance} mm " + $"Status. {sensor.status}" + $" --> count = {i}");
                Thread.Sleep(2000);
            }
        }
    }
}
