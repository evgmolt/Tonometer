double PSys = 0;
double PDia = 0;
int indexPSys = 0;
int indexPDia = 0;
double ValueSys = 0.4 * MaximumAmplitude;
double ValueDia = 0.8 * MaximumAmplitude;
//XMax - индекс максимальной амплитуды

//Определение систолического давления (влево от XMax)
for (int i = XMax; i >= 0; i--)
{
    if (DataA.EnvelopeArray[i] < ValueSys)
    {
        PSys = DataA.DCArray[i];
        indexPSys = i;
        break;
    }
}
if (PSys == 0)
{
    PSys = DataA.DCArray[0];
    ShowError(BPMError.Sys);
}
//Определение диастолического давления (вправо от XMax)
for (int i = XMax; i < DataA.Size; i++)
{
    if (DataA.EnvelopeArray[i] < ValueDia)
    {
        PDia = DataA.DCArray[i];
        indexPDia = i;
        break;
    }
}
if (PDia == 0)
{
    PDia = DataA.DCArray[DataA.Size - 1];
    ShowError(BPMError.Dia);
}
