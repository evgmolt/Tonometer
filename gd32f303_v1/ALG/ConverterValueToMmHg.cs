using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TTestApp
{
    internal class ConverterValueToMmHg
    {
        private double Coefficient = CoeffTonometer;
        private const double CoeffA = 59021;
        private const double CoeffB = 55736;
        private const double CoeffTonometer = 43215;
        private const double _pressure = 2210;

        public EventHandler<ConverterValueToMmHgEventArgs> CoeffChanged;

        public void ChangeCoeff(string fileName)
        {
            char filePrefix = fileName[0];
            Coefficient = filePrefix switch
            {
                'A' => CoeffA,
                'B' => CoeffB,
                _ => CoeffTonometer
            };
            CoeffChanged(this, new ConverterValueToMmHgEventArgs { Coeff = Coefficient });
        }
        public int Convert(double value) => (int)(value / (Coefficient / _pressure));
    }
}
