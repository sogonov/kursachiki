#include <MDR32F9Qx_port.h> 
#include <MDR32F9Qx_rst_clk.h> 
#include <MDR32F9Qx_adc.h>
#include <MDR32F9Qx_timer.h>


uint16_t DU;
char Buffer[100];

void TimerInit(void);
void Timer1_IRQHandler(void);

int main() 
{
RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC | RST_CLK_PCLK_PORTD, ENABLE);
PORT_InitTypeDef Nastroyka; 
Nastroyka.PORT_Pin = PORT_Pin_7; 
Nastroyka.PORT_OE = PORT_OE_IN; 
Nastroyka.PORT_MODE = PORT_MODE_ANALOG; 
PORT_Init (MDR_PORTD, &Nastroyka);
ADC_InitTypeDef sADC; 
ADCx_InitTypeDef sADCx;
ADC_DeInit(); 
ADC_StructInit(&sADC); 
sADC.ADC_SynchronousMode= ADC_SyncMode_Independent; 
ADCx_StructInit (&sADCx); 
sADCx.ADC_ClockSource= ADC_CLOCK_SOURCE_CPU; 
sADCx.ADC_SamplingMode= ADC_SAMPLING_MODE_SINGLE_CONV; 
sADCx.ADC_ChannelNumber= ADC_CH_ADC7; 
sADCx.ADC_Channels= 0; 
sADCx.ADC_VRefSource= ADC_VREF_SOURCE_INTERNAL; 
sADCx.ADC_IntVRefSource= ADC_INT_VREF_SOURCE_INEXACT; 
sADCx.ADC_Prescaler= ADC_CLK_div_None; 
ADC1_Init (&sADCx);
ADC1_Cmd (ENABLE); 
TimerInit ();
LCD_Init();


while (1) 
{ 
//ADC1_Start();
//while ((ADC1_GetStatus() & ADC_STATUS_FLG_REG_EOCIF) == 0);
//DU = ADC1_GetResult() & 0x0FFF;
//printf("U=0x%03x\r\n",DU); // for simulation

sprintf(Buffer,"U=0x%03X",DU);
LCD_PutString(Buffer,4);
	
} 
}


void TimerInit() 
{ 
// Indication the type of structure and structure name
TIMER_CntInitTypeDef TIM1Init; 
// Enabling of clocking
RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER1, ENABLE); 
// Filling the structure with default values
TIMER_CntStructInit(&TIM1Init); 
// Setup of clock frequency divider 
TIMER_BRGInit (MDR_TIMER1, TIMER_HCLKdiv1); 
// Setup of clock frequency prescaler
TIM1Init.TIMER_Prescaler = 8000; 
// Setup the timer period
TIM1Init.TIMER_Period = 200; 
// Initialization of a timer port by a declared structure
TIMER_CntInit (MDR_TIMER1, &TIM1Init); 
// Run interrupts 
NVIC_EnableIRQ (Timer1_IRQn); 
// Setup Interrupt Priority
NVIC_SetPriority (Timer1_IRQn, 0); 
// Enabling interrupt when value TIMER1 is zero 
TIMER_ITConfig(MDR_TIMER1, TIMER_STATUS_CNT_ZERO, ENABLE); 
// Run timer 
TIMER_Cmd(MDR_TIMER1, ENABLE); 
}

void Timer1_IRQHandler() 
{ 
if (TIMER_GetITStatus(MDR_TIMER1, TIMER_STATUS_CNT_ZERO)) 
TIMER_ClearITPendingBit(MDR_TIMER1, TIMER_STATUS_CNT_ZERO); 
ADC1_Start();
while((ADC1_GetStatus() & ADC_STATUS_FLG_REG_EOCIF) == 0);
DU = ADC1_GetResult() & 0x0FFF;
}
