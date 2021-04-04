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
            sensor = new Serial_HCSR04(SensorType.JSN_SR04T);

            for (int i = 0; i < 20; i++)
            {
                int distance = sensor.getDistance();
                Debug.WriteLine($"Distance = {distance} mm");
                Thread.Sleep(1000);
            }
        }
    }
}
