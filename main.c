/**
  ******************************************************************************
  * @file       main.c
  * @author     Burak Keten
  * @version    V1.0.0
  * @date       27-April-2018
  * @compiler   IAR STM8
  * @brief      This file contains the moving average filter for ADC input.
  ******************************************************************************
**/ 


#include <iostm8l152c6.h>
#include <intrinsics.h>

#define Timer4_Start (TIM4_CR1 |= (1 << 0))
#define windowSize 8

unsigned int    ADCR, ADCRL, ADCRH; 
int j=0;
unsigned int Mem[windowSize];
unsigned int sum=0;
unsigned int movAvg = 0;

void InitClock(void)
{
   CLK_ICKCR = 0x01;            //High-speed internal RC oscillator ON
   while(!(CLK_ICKCR & 0x02));  //HSI clock haz�r olana kadar bekle
   CLK_SWR = 0x01;              //HSI selected as systemclock source
   while(CLK_SCSR!=0x01);       //HSI Systemclock i�in stabilizasyonu sa�lanana kadar bekle
   CLK_CKDIVR = 0x00;           //System clock source/1
   
   CLK_PCKENR2 |= (1 << 0);      //adc i�in clock aktif ediliyor
}
 
void InitGPIO(void)
{
   // LED �IKI�I (PC7)  BLUE_LED
   PC_DDR |= (1 << 7);  //Output olarak ayarlan�yor
   PC_CR1 |= (1 << 7);  //Push-pull
   PC_ODR &= (0 << 7);  //��k�� de�eri
    
   // ADC G�R���(ADC_IN0) (PA6)   A6
   PA_DDR &= ~(1 << 6); //Input olarak ayarlan�yor
   PA_CR1 &= ~(1 << 6); //Floating Input ayarlan�yor
   PA_CR2 &= ~(1 << 6);
}

void InitADC(void)
{  
  ADC1_CR1 |= (1 << 3);         // ADC Interrupt Enable
  
  ADC1_CR3 &= ~(0x1F);          // ADC Selection S�f�rlan�yor
  ADC1_CR3 |= (0 << 0);         // Channel 0 select
  
  ADC1_CR2 |= (1 << 7);         // Fmaster/2
  
  ADC1_CR2 &= ~(0x07);          // Sampling time selection s�f�rlan�yor
  ADC1_CR2 |= (7 << 0);         // 384 ADC Clock Cycles
  
  ADC1_CR1 |= (1 << 2);         // Continuous Conversion
  
  ADC1_CR1 &= ~(0x60);          //Resolution s�f�rlan�yor
  ADC1_CR1 |= (0 << 5);         //12bit adc
  
  ADC1_CR1 |= (1 << 0);         // ADC ON
  
  ADC1_CR1 |= (1 << 1);         //Conversion start
  
  ADC1_SQR1 |= (1 << 7);        //DMA Disable
  ADC1_SQR4 |= (1 << 0);        //Channel 0 is assigned in the scan sequence
  
}
 
void main(void)
{
  __disable_interrupt();
  InitClock();
  InitGPIO();
  InitADC();
  PC_ODR |= (1 << 7);   //Mavi Led Yak�l�yor
  Timer4_Start;
  
  __enable_interrupt();
  for(;;)
  {
    if(movAvg>1000)       PC_ODR |= (1 << 7);   //Mavi Led Yak�l�yor 
    else if(movAvg<=1000) PC_ODR &= ~(1 << 7);   //Mavi Led S�nd�r�l�yor 
  }
}

#pragma vector=ADC1_EOC_vector
__interrupt void ADC_ISR(void)
{
  ADCRL = ADC1_DRL; ADCRH = ADC1_DRH;   //ADC Verisinin Y�ksek ve D���k de�erlikli bitleri atan�yor
  ADCR = (ADCRH <<8) | ADCRL;
  ADC1_SR &= ~(1 << 0);                 //ADC bayra��n� temizleme
  
  Mem[j] = ADCR;                        //Okunan ADC degeri dairesel tampona(circular buffer) aktariliyor.
  j += 1;                               //Tampon indisi arttiriliyor
  j = j & (windowSize-1) ;              //Tampon indisi, pencere boyutu ile sinirlandiriliyor
  for (int b =0; b <= windowSize; b++){ //Tampondaki ADC degerleri d�ng� i�erisinde toplaniyor
            sum += Mem[b];
        }
  movAvg = sum / windowSize;            //Toplam degerin pencere boyutuna b�l�nmesiyle filtrelenmis deger bulunuyor.
  sum = 0;
  
}