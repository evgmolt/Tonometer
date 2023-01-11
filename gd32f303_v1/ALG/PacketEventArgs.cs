namespace TTestApp
{
    internal class PacketEventArgs : EventArgs
    {
        public uint MainIndex { get; set; }
        public int PacketCounter { get; set; }
        public double RealTimeValue { get; set; }
        public double PressureViewValue { get; set; }
        public double DerivativeValue { get; set; }
        public double DCValue { get; set; }
    }
}
