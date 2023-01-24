#include "main.h"
#include "DataProcessing.h"

#define NCoef 2 
#define DiaCoeff 0.62
#define SysCoeff 0.46

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

int GetMaxIndexInRegion(int16_t *sourceArray, int index){ 
    int range = 50;
    int16_t max = -200;
    int maxIndex = 0;
    for (int i1 = 0; i1 < range; i1++){
        //if (i - range / 2 < 0) continue;
        //if (i - range / 2 > strlen(sourceArray)) continue;
        if (sourceArray[index + i1 - range / 2] > max){
            max = sourceArray[index + i1 - range / 2];
            maxIndex = i1 - range / 2;
        }
    }		
    return index + maxIndex;
}

int GetMinIndexInRegion(int16_t *sourceArray_MIN,int index){		
		int range_MIN=100;
		int16_t min = 1000;
		int minIndex = 0;
		for (int i1 = 0; i1 < range_MIN; i1++){
				if (sourceArray_MIN[index+i1] < min){
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

void CountEnvelopeArray(int16_t *arrayOfIndexes, int16_t *arrayOfValues){
    for (int i = 1; i < puls_counter; i++){
        int x1 = arrayOfIndexes[i - 1];
        int x2 = arrayOfIndexes[i];
        double y1 = arrayOfValues[i - 1];
        double y2 = arrayOfValues[i];
        double coeff = (y2 - y1) / (x2 - x1);
        for (int j = x1 - 1; j < x2; j++) {
            EnvelopeArray[j] = y1 + coeff * (j - x1);
        }				
    }
}


void f_PSys_Dia(void){
	double MaximumAmplitude=-100;
	
	for (int i=0; i<puls_counter; i++){
			if (puls_buff_AMP[i]>MaximumAmplitude){
					MaximumAmplitude=puls_buff_AMP[i];
					XMax=puls_buff_NEW[i];					
			}		
	}		
	
	int16_t ValueSys = SysCoeff * MaximumAmplitude;
	int16_t ValueDia = DiaCoeff * MaximumAmplitude;	
	
	for (int i = XMax; i >= 200; i--){
			if (EnvelopeArray[i] < ValueSys){
					PSys = save_clear[i]/rate;
					indexPSys = i;
					break;
			}
	}
	for (int i = XMax; i < main_index; i++)
	{
    if (EnvelopeArray[i] < ValueDia)
    {
        PDia = save_clear[i]/rate;
        indexPDia = i;
        break;
    }
	}
}

int16_t slim_mas(uint16_t *mass_in, int16_t DC, int16_t AC){
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
		
		
		float ACoef[NCoef+1] = { 
        0.97913295295553560000, 
        -1.95826590591107120000, 
        0.97913295295553560000 
    }; 
 
    float BCoef[NCoef+1] = { 
        1.00000000000000000000, 
        -1.95778812550116580000, 
        0.95837795232608958000 
    }; 
 
    static float y[NCoef+1]; //output samples 
    static float x[NCoef+1]; //input samples 
    int n; 
 
    //shift the old samples 
    for(n=NCoef; n>0; n--) { 
       x[n] = x[n-1]; 
       y[n] = y[n-1]; 
    } 
 
    //Calculate the new output 
    x[0] = ACLevel; 
    y[0] = ACoef[0] * x[0]; 
    for(n=1; n<=NCoef; n++) 
        y[0] += ACoef[n] * x[n] - BCoef[n] * y[n]; 
		
		
		
		return (int16_t)y[0];
		//return ACLevel-DCLevel;
}

