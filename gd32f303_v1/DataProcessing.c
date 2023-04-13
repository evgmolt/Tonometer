#include "main.h"
#include "DataProcessing.h"

#define N_COEF 2 
#define DIA_COEF 0.62
#define SYS_COEF 0.46

uint16_t CountPulse(void)
{
    uint8_t decrease_size = 3; //Количество отбрасываемых интервалов слева и справа
    uint8_t num_of_intervals = 10; //Количество интервалов для оценки аритмии
    double level = 0.06;
    uint16_t intervals[50]={0};
    double first_pulse = 0;
    double second_pulse = 0;
    double cur_puls=0;
    uint32_t counter=0;
    uint16_t pcounter;
    if (puls_counter<10) return 0;
    for (int m = decrease_size; m < puls_counter - decrease_size; m++)
    {
        cur_puls = puls_buff[m] - puls_buff[m-1];
        if (cur_puls > lo_limit & cur_puls < hi_limit)
        {
            first_pulse += cur_puls;
            counter++;
        }
    }        
    first_pulse /= counter;
    mean_interval = first_pulse;
    first_pulse /= frequency;
    first_pulse = 60 / first_pulse;
    
    if (puls_counter < num_of_intervals)
    {
        arrhythmia = false;
        return first_pulse;
    }
    
    counter = 0;
    for (int m = 0; m < num_of_intervals; m++)
    {
        cur_puls = puls_buff[puls_counter - m - 1] - puls_buff[puls_counter - m - 2];
        if (cur_puls > lo_limit & cur_puls < hi_limit & cur_puls * 1.5 > first_pulse & cur_puls / 1.5 < first_pulse)
        {
            second_pulse += cur_puls;
            intervals[counter]=cur_puls;
            counter++;    
        }
    }
    
    double aver= second_pulse / counter;
    double TwentyFivePercent = aver / 4;
    double SumSqr = 0;
    for (int i = 0; i < counter; i++)
    {
        double Diff = intervals[i] - aver;
        if (abs(Diff) < TwentyFivePercent)
        {
            SumSqr += Diff * Diff;
        }
    }
    
    double SKO = sqrt(SumSqr / counter);
    arrhythmia = (SKO / aver)>level;
    
    return first_pulse;
}

int16_t GetDerivative(int16_t *dataArr, int32_t Ind){
   if (Ind < (DERIVATIVE_AVER_WIDTH + DERIVATIVE_SHIFT))
   {
       return 0;
   }
   int32_t val1 = 0;
   int32_t val2 = 0;
   for (int i = 0; i < DERIVATIVE_AVER_WIDTH; i++){
       val1 += dataArr[Ind - DERIVATIVE_AVER_WIDTH + i];
       val2 += dataArr[Ind - DERIVATIVE_AVER_WIDTH - DERIVATIVE_SHIFT + i];
   }
   val1 /= DERIVATIVE_AVER_WIDTH;
   val2 /= DERIVATIVE_AVER_WIDTH;
   return (int16_t)(val1 - val2);
}

void GetArrayOfWaveIndexes(int16_t *valuesArray, int16_t *indexesArray, int16_t *indexes)
{    
    uint16_t index_of_min;
    for (int i=0; i<puls_counter; i++)
    {
        index_of_min = GetMinIndexInRegion(valuesArray, indexesArray[i]);
        puls_buff_IND_MIN[i] = index_of_min;
        puls_buff_AMP_MIN[i] = valuesArray[index_of_min];
        indexes[i] = GetMaxIndexInRegion(valuesArray, indexesArray[i]);
        puls_buff_AMP[i]=valuesArray[indexes[i]];                
    }    
}

int GetMaxIndexInRegion(int16_t *sourceArray, int index)
{ 
    int range = 50;
    int16_t max = -200;
    int maxIndex = 0;
    for (int i1 = 0; i1 < range; i1++)
    {
        if (sourceArray[index + i1 - range / 2] > max)
        {
            max = sourceArray[index + i1 - range / 2];
            maxIndex = i1 - range / 2;
        }
    }        
    return index + maxIndex;
}

int GetMinIndexInRegion(int16_t *sourceArray_MIN,int index)
{        
    int range_MIN=100;
    int16_t min = 1000;
    int minIndex = 0;
    for (int i1 = 0; i1 < range_MIN; i1++)
    {
        if (sourceArray_MIN[index+i1] < min)
        {
            min=sourceArray_MIN[index+i1];
            minIndex=i1;
        }
    }            
    return index + minIndex;
}

void f_sorting_MAX(void){
        int16_t MaximumAmplitude=-100;
        uint8_t FLAG=1;    
        uint16_t mini_XMAX=0;
        int16_t z=0;
        uint8_t buff1[10]={0};        
        
        int level = 8;
    for (int i = 1; i < puls_counter - 1; i++){
                if (abs(puls_buff_AMP[i] - puls_buff_AMP[i - 1]) > level)
                {
                        puls_buff_AMP[i] = (puls_buff_AMP[i - 1] + puls_buff_AMP[i + 1]) / 2;
                }
    }        
        
        for (int i=0; i<puls_counter; i++){
                puls_buff_AMP[i]=puls_buff_AMP[i]-puls_buff_AMP_MIN[i];
        }
        
        for (int i=0; i<puls_counter; i++){
                if (puls_buff_AMP[i]>MaximumAmplitude){
                        MaximumAmplitude=puls_buff_AMP[i];                            
                        mini_XMAX=i;
                }        
        }            
        
        while (FLAG==1){
                FLAG=0;
                for (int i=1; i<mini_XMAX; i++){
                        if (puls_buff_AMP[i-1]>puls_buff_AMP[i]){
                                z=puls_buff_AMP[i-1];
                                puls_buff_AMP[i-1]=puls_buff_AMP[i];
                                puls_buff_AMP[i]=z;
                                //swap(puls_buff_AMP[i-1],puls_buff_AMP[i]);
                                FLAG=1;
                        }
                }
        }
        FLAG=1;
        while (FLAG==1){
                FLAG=0;
                for (int i=mini_XMAX+2; i<puls_counter; i++){
                        if (puls_buff_AMP[i-1]<puls_buff_AMP[i]){
                                z=puls_buff_AMP[i-1];
                                puls_buff_AMP[i-1]=puls_buff_AMP[i];
                                puls_buff_AMP[i]=z;
                                //swap(puls_buff_AMP[i-1],puls_buff_AMP[i]);
                                FLAG=1;
                        }
                }
        }        
}

void CountEnvelopeArray(int16_t *arrayOfIndexes, int16_t *arrayOfValues)
{
    for (int i = 1; i < puls_counter; i++)
    {
        int x1 = arrayOfIndexes[i - 1];
        int x2 = arrayOfIndexes[i];
        double y1 = arrayOfValues[i - 1];
        double y2 = arrayOfValues[i];
        double coeff = (y2 - y1) / (x2 - x1);
        for (int j = x1 - 1; j < x2; j++) 
        {
            int ind = i + j;
            if (ind >= 10000)
            {
                break;
            }
            envelope_array[i + j] = y1 + coeff * (j - x1);
        }                
    }
}

int16_t GetAverAroundPoint(int16_t *in_array, int point)
{
    int aver_size_half = mean_interval / 2;
    int index = 0;
    int32_t sum = 0;
    int start = point - aver_size_half;
    int stop = point + aver_size_half;
    if (start < 0) start = 0;
    if (stop > total_size - 1) stop = total_size - 1;
    for (int i = start; i < stop; i++)
    {
        sum += in_array[i];
        index++;
    }
    return (int16_t)(sum / index);
}

void GetSysDia(void)
{
    int16_t index_of_max;
    double MaximumAmplitude = -100;
    int skip = 3;
    int x1;
    int x2;
    int p1;
    int p2;
    int xSys;
    double deltaY;
    double coeff;
    
    for (int i = skip; i < puls_counter - skip; i++)
    {
        if (puls_buff_AMP[i] > MaximumAmplitude)
        {
            MaximumAmplitude = puls_buff_AMP[i];
            XMax = puls_buff_NEW[i];      
            index_of_max = i;
        }        
    }        
    
    int16_t ValueSys = SYS_COEF * MaximumAmplitude;
    int16_t ValueDia = DIA_COEF * MaximumAmplitude;

    PSys = 0;
    PDia = 0;
/*    
    for (int i = index_of_max; i >= 0; i--)
    {
        if (puls_buff_AMP[i] <ValueSys)
        {
            x1 = puls_buff_IND_MIN[i];
            x2 = puls_buff_IND_MIN[i + 1];
            p1 = pressure_array[puls_buff_IND_MIN[i]];
            p2 = pressure_array[puls_buff_IND_MIN[i + 1]];
            coeff = (puls_buff_AMP[i + 1] - puls_buff_AMP[i]);
            coeff = coeff / (x2 - x1);
            xSys = x1 + ValueSys * coeff;
            PSys = (p1 + coeff * (xSys - x1)) / rate;
            indexPSys = i;
            break;
        }
    }
    */
    for (int i = XMax; i >= 0; i--)
    {
        if (envelope_array[i] < ValueSys)
        {
            PSys = GetAverAroundPoint(pressure_array, i) / rate;
            indexPSys = i;
            break;
        }
    }
    if (PSys == 0)
    {
        PSys = pressure_array[0] / rate;        
    }

    for (int i = XMax; i < main_index; i++)
    {
        if (envelope_array[i] < ValueDia)
        {
            PDia = GetAverAroundPoint(pressure_array, i) / rate;
            indexPDia = i;
            break;
        }
    }
    if (PDia == 0)
    {
        PDia = pressure_array[main_index - 1] / rate;
    }
}

int16_t SmoothAndRemoveDC(uint16_t *mass_in, int16_t DC, int16_t AC)
{
    int32_t DCLevel = 0;
    int32_t ACLevel = 0;                    
    for(int r = 0; r < DC; r++)
    {
        DCLevel += mass_in[main_index - 1 - r];
    }
    DCLevel /= DC;    
    for (int j = 0; j < AC; j++)
    {
       ACLevel += mass_in[main_index - 1 - j];
    }
    ACLevel /= AC;
    i2c_out = (int16_t)ACLevel;
    current_pressure=(int16_t)(i2c_out / rate);
    if (current_pressure < 0 & main_index < 500) current_pressure = 0;
    mass_in[main_index-1] = (uint16_t)ACLevel;            
    
    return ACLevel - DCLevel;
}

