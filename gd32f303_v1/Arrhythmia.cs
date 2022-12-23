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
        internal static bool AtrialFibrillation(int[] arrayOfIndexes, ref double res) 
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
            res = SKO / aver;
            return res > level;
        }

        internal static bool AtrialFibrillation2(int[] arrayOfIndexes, ref double res)
        {
            if (arrayOfIndexes.Length < numOfIntervals + 1)
            {
                return false;
            }
            int[] Intervals = new int[numOfIntervals];
            for (int i = 1; i < numOfIntervals; i++)      //Цикл с 1 !!!
            {
                Intervals[i - 1] = arrayOfIndexes[numOfIntervals - i] - arrayOfIndexes[numOfIntervals - i - 1];
            }
            double Aver = 0;
            for (int i = 0; i < numOfIntervals; i++)
            {
                Aver += Intervals[i];
            }
            Aver /= numOfIntervals;
            double TwentyFivePercent = Aver / 4;
            int Counter = 0;
            double SumSqr = 0;
            for (int i = 0; i < numOfIntervals; i++)
            {
                double Diff = Intervals[i] - Aver;
                if (Math.Abs(Diff) < TwentyFivePercent)
                {
                    SumSqr += Diff * Diff;
                    Counter++;
                }
            }
            double SKO = Math.Sqrt(SumSqr / numOfIntervals);
            res = SKO / Aver;
            return res > level;
        }
    }
}
