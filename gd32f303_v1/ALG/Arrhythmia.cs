using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TTestApp
{
    internal static class Arrhythmia
    {
        private const int numOfIntervals = 10;
        private const double level = 0.06;

        //Мерцательная аритмия, алгоритм Microlife
        internal static bool AtrialFibrillation(int[] arrayOfIndexes) 
        {
            int length = arrayOfIndexes.Length;
            if (numOfIntervals > length - 1)
            {
                return false;
            }
            List<int> intervals = new();
            for (int i = 0; i < numOfIntervals; i++)
            {
                intervals.Add(arrayOfIndexes[length - i - 1] - arrayOfIndexes[length - i - 2]);
            }
            double aver = intervals.Average();
            double TwentyFivePercent = aver / 4;
            intervals.RemoveAll(x => Math.Abs(x - aver) > TwentyFivePercent);
            double SKO = Math.Sqrt(intervals.Select(x => (x - aver) * (x - aver)).Sum() / numOfIntervals);
            double res = SKO / aver;
            return res > level;
        }
    }
}
