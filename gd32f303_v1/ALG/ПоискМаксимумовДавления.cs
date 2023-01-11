//valuesArray - массив пульсаций давления (без постоянной составляющей)
//indexesArray - массив индексов максимумов производной
//Возвращает новый массив максимумов 
static int[] GetArrayOfWaveIndexes(double[] valuesArray, int[] indexesArray)
{
    int[] indexes = new int[indexesArray.Length];
    for (int i = 0; i < indexesArray.Length; i++)
    {
        indexes[i] = GetMaxIndexInRegion(valuesArray, indexesArray[i]);
    }
    return indexes;
}

static int GetMaxIndexInRegion(double[] sourceArray, int index)
{
    int range = 50;
    double max = 0;
    int maxIndex = 0;
    for (int i = 0; i < range; i++)
    {
        if (i - range / 2 < 0) continue;
        if (i - range / 2 > sourceArray.Length) continue;
        if (sourceArray[i - range / 2] > max)
        {
            max = sourceArray[i - range / 2];
            maxIndex = i;
        }
    }
    return index + maxIndex;
}
