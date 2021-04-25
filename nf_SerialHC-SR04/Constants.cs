
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
            AJ_SR04M = 0x01,
        }

        public enum AJ_SR04M
        {
            Mode1,
            Mode2,
            Mode3,
            Mode4,
            Mode5,
            Ping = 0x01,
        }

    }
}
