
namespace Driver.nf_SerialHC_SR04
{
    namespace Constants
    {

        /// <summary>
        /// nf_Serial_HCSR04 module
        /// </summary>
   
            public enum SensorType
            {
                JSN_SR04T = 0x55,
                AJ_SR04M  = 0x01,
            }

            public enum Mode
            {
                Pulse,
                Pulse_LP,
                Serial_Auto,
                Serial_LP_Bin,
                Serial_LP_ASCII,

        }
    }
}
