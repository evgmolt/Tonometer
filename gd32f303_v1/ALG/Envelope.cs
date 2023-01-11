void CountEnvelopeArray(int[] arrayOfIndexes, double[] arrayOfValues)
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