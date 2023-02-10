#include "main.h"
#include "DataProcessing.h"

#define N_COEF 2 
#define DIA_COEF 0.62
#define SYS_COEF 0.46

uint16_t CountPulse(void)
{
    double level = 0.06;
    uint16_t intervals[50]={0};
    double first_puls=0;
    int16_t cur_puls=0;
    puls_out=0;
    puls_cur_counter=0;
    if (puls_counter<10) return 0;
    for (int m = 3; m < puls_counter - 3; m++)
    {
        cur_puls = puls_buff[m] - puls_buff[m-1];
        if (cur_puls > lo_limit & cur_puls < hi_limit)
        {
            first_puls+=cur_puls;
            puls_cur_counter++;
        }
    }        
    first_puls=first_puls / puls_cur_counter;
    
    puls_cur_counter = 0;
    for (int m=1;m<puls_counter;m++)
    {
        cur_puls=puls_buff[m]-puls_buff[m-1];
        if (cur_puls > lo_limit & cur_puls < hi_limit & cur_puls * 1.5 > first_puls & cur_puls / 1.5 < first_puls)
        {
            puls_out+=cur_puls;
            intervals[puls_cur_counter]=cur_puls;
            puls_cur_counter++;    
        }
    }
    
    double Aver=puls_out / puls_cur_counter;
    double TwentyFivePercent = Aver / 4;
    int Counter = 0;
    double SumSqr = 0;
    for (int i = 0; i < puls_cur_counter; i++)
    {
        double Diff = intervals[i] - Aver;
        if (abs(Diff) < TwentyFivePercent)
        {
            SumSqr += Diff * Diff;
            Counter++;
        }
    }
    
    double SKO = sqrt(SumSqr/Counter);
    arrhythmia = (SKO/Aver)>level;
    
    puls_out=60/(puls_out/(puls_cur_counter*frequency));
    return puls_out;
}


int16_t GetAver(int16_t nextValue) {
    ArrayForAver[ArrayForAverIndex] = nextValue;
    ArrayForAverIndex++;
    if (ArrayForAverIndex > AVER_SIZE - 1) ArrayForAverIndex = 0;
    int16_t sum = 0;
    for (int i = 0; i < AVER_SIZE; i++) sum += ArrayForAver[i];
    return sum / AVER_SIZE;
}

int16_t GetDerivative(int16_t *dataArr, int32_t Ind){
   if (Ind < (DerivativeAverageWidth+DerivativeShift)){
       return 0;
   }
   int32_t val1 = 0;
   int32_t val2 = 0;
   for (int i = 0; i < DerivativeAverageWidth; i++){
       val1 += dataArr[Ind - DerivativeAverageWidth + i];
       val2 += dataArr[Ind - DerivativeAverageWidth - DerivativeShift + i];
   }
   val1 /= DerivativeAverageWidth;
   val2 /= DerivativeAverageWidth;
   return (int16_t)(val1 - val2);
}

void GetArrayOfWaveIndexes(int16_t *valuesArray, int16_t *indexesArray, int16_t *indexes){    
    for (int i=0; i<puls_counter; i++)
    {
        puls_buff_NEW_MIN[i] = GetMinIndexInRegion(valuesArray, indexesArray[i]);
                puls_buff_AMP_MIN[i] = valuesArray[puls_buff_NEW_MIN[i]];
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
            EnvelopeArray[i + j] = y1 + coeff * (j - x1);
        }                
    }
}

int16_t GetAverAroundPoint(int16_t *in_array, int point)
{
    int aver_size_half = 8;
    int index = 0;
    int32_t sum = 0;
    for (int i = point - aver_size_half; i < point + aver_size_half; i++)
    {
        sum += in_array[i];
        index++;
    }
    return (int16_t)(sum / index);
}

void GetSysDia(void)
{
    double MaximumAmplitude = -100;
    int skip = 3;
    for (int i = skip; i < puls_counter - skip; i++)
    {
        if (puls_buff_AMP[i] > MaximumAmplitude)
        {
            MaximumAmplitude = puls_buff_AMP[i];
            XMax=puls_buff_NEW[i];                    
        }        
    }        
    
    int16_t ValueSys = SYS_COEF * MaximumAmplitude;
    int16_t ValueDia = DIA_COEF * MaximumAmplitude;    

    PSys = 0;
    PDia = 0;
    
    for (int i = XMax; i >= 0; i--)
    {
        if (EnvelopeArray[i] < ValueSys)
        {
            PSys = GetAverAroundPoint(save_clear, i)/rate;
            indexPSys = i;
            break;
        }
    }
    if (PSys == 0)
    {
        PSys = save_clear[0]/rate;        
    }

    for (int i = XMax; i < main_index; i++)
    {
        if (EnvelopeArray[i] < ValueDia)
        {
            PDia = GetAverAroundPoint(save_clear, i)/rate;
            indexPDia = i;
            break;
        }
    }
    if (PDia == 0)
    {
        PDia = save_clear[main_index - 1] / rate;
    }
}

int16_t SmoothAndRemoveDC(uint16_t *mass_in, int16_t DC, int16_t AC){
        int32_t DCLevel = 0;
        int32_t ACLevel = 0;                    
        for(int r=0;r<DC;r++){
                DCLevel+=mass_in[main_index-1-r];
        }
        DCLevel/=DC;    
        for (int j=0;j<AC;j++){
       ACLevel+=mass_in[main_index-1-j];
    }
    ACLevel/=AC;
    i2c_out = (int16_t)ACLevel;
    current_pressure=(int16_t)(i2c_out/rate);
    if (current_pressure<0 & main_index<500) current_pressure=0;
    mass_in[main_index-1] = (uint16_t)ACLevel;    
        
/*    
    float ACoef[N_COEF+1] = { 
        0.97913295295553560000, 
        -1.95826590591107120000, 
        0.97913295295553560000 
    }; 
 
    float BCoef[N_COEF+1] = { 
        1.00000000000000000000, 
        -1.95778812550116580000, 
        0.95837795232608958000 
    }; 
 
    static float y[N_COEF+1]; //output samples 
    static float x[N_COEF+1]; //input samples 
    int n; 
 
    //shift the old samples 
    for(n=N_COEF; n>0; n--) { 
       x[n] = x[n-1]; 
       y[n] = y[n-1]; 
    } 
 
    //Calculate the new output 
    x[0] = ACLevel; 
    y[0] = ACoef[0] * x[0]; 
    for(n=1; n<=N_COEF; n++) 
        y[0] += ACoef[n] * x[n] - BCoef[n] * y[n]; 
        
//    return (int16_t)y[0];*/
    
    return ACLevel-DCLevel;
}

