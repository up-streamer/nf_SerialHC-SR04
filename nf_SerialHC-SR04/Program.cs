using System;
using System.Diagnostics;
using System.Threading;
using Driver.nf_Serial_HCSR04;
using Driver.nf_SerialHC_SR04.Constants;

namespace testnf_SerialHC_SR04
{
    public class Program
    {
        private static Serial_HCSR04 sensor;
        public static void Main()
        {

            sensor = new Serial_HCSR04(SensorType.AJ_SR04M, Mode.Serial_Auto);
            sensor.DataReceived += PrintData;

            //for (int i = 0; i < 500; i++)
            //{
            //    int distance = sensor.GetDistance();
            //    Debug.WriteLine($"distance = {distance} mm" + $"--> count = {i}");
            //    Thread.Sleep(1000);
            //}

        }

        static void PrintData(uint data1, uint data2, DateTime date)
        {
            Debug.WriteLine($"Distance = {sensor.Distance} mm" + $"--> Date = {date.ToString()}");

            //Thread.Sleep(1000);
        }
    }
}
