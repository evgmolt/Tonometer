namespace TTestApp
{
    internal class DataArrays
    {
        private readonly int _size;

        public double[] RealTimeArray;
        public double[] DCArray;
        public double[] PressureViewArray;
        public double[] EnvelopeArray;
        public double[] DerivArray;
        public double[] DebugArray;

        public int Size { get { return _size; } }
        
        public DataArrays(int size)
        {
            _size = size;
            RealTimeArray = new double[_size];
            DCArray = new double[_size];
            PressureViewArray = new double[_size];
            EnvelopeArray = new double[_size];
            DerivArray = new double[_size];  
            DebugArray = new double[_size];
        }

        public static DataArrays CreateDataFromLines(string[] lines)
        {
            DataArrays a = new(lines.Length);
            try
            {
                for (int i = 0; i < lines.Length; i++)
                {
                    a.RealTimeArray[i] = Convert.ToInt32(lines[i]);
                }
                return a;
            }
            catch (Exception)
            {
                return null;
            }
        }

        public void CountViewArrays()
        {
            int DCArrayWindow = 60;
            int ACArrayWindow = 6;
            for (int i = DCArrayWindow; i < _size; i++)
            {
                double DCLevel = 0;
                for (int j = 0; j < DCArrayWindow; j++)
                {
                    DCLevel += RealTimeArray[i - j];
                }
                DCLevel /= DCArrayWindow;
                DCArray[i] = DCLevel;
                double ACLevel = 0;
                for (int j = 0; j < ACArrayWindow; j++)
                {
                    ACLevel += RealTimeArray[i - j];
                }
                ACLevel /= ACArrayWindow;
                PressureViewArray[i] = ACLevel - DCLevel;
            }

            for (uint i = 0; i < PressureViewArray.Length; i++)
            {
                DerivArray[i] = DataProcessing.GetDerivative(PressureViewArray, i);
            }
        }

        public void CountEnvelopeArray(int[] arrayOfIndexes, double[] arrayOfValues)
        {
            for (int i = 1; i < arrayOfIndexes.Length; i++)
            {
                int x1 = arrayOfIndexes[i - 1];
                int x2 = arrayOfIndexes[i];
                double y1 = arrayOfValues[i - 1];
                double y2 = arrayOfValues[i];
                double coeff = (y2 - y1) / (x2 - x1);
                for (int j = x1 - 1; j < x2; j++)
                {
                    int ind = i + j;
                    if (ind >= Size)
                    {
                        break;
                    }
                    EnvelopeArray[i + j] = y1 + coeff * (j - x1);
                }
            }
        }

        public String GetDataString(uint index)
        {
            return RealTimeArray[index].ToString();
        }
    }
}
