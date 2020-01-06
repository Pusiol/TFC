// UFJF - ENGENHARIA ELÉTRICA - NÚCLEO DE ILUMINAÇÃO MODERNA
//

//                          |
//         __----__         |            /\
//       _-        -_       |          /   \     /\
//      -  ||\   ||  -      |   |    |  +--+\  /   +--+
//     /-  ||\\  ||  -\     |   |  / |  |    /     |\
//     |   || \\ ||   |     |   |/   | -+- /  |   -+-\
//     \_  ||  \\||  _/     |  /_\__/ __|/    |    |  \
//      -_ ||   \|| _-      |          /| ___ | __ | __\
//        -__     _-        |                 |
//           -----          |             +---+
//

//
// Programa de acionamento para um buck entrelaçado
// com controle do tipo integrador
//
// ============Pinagem===================
// PWM Principal B6 Complementar B7
// PWM Auxiliar  A6
// Leitura ADC   E5

#define PWM_FREQUENCY 40000
#define DIMM 0

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "driverlib/uart.h"

#include "driverlib/debug.h"
#include "driverlib/adc.h"

#include "driverlib/fpu.h"


uint32_t ui32Period,lact,DUTY,PWMPERIOD;
char l[6];


int duty = 5,dutyi,dead,FLAG=5;


//=============================================
#define A  10.2f//9.04f
#define B  24832.0f//22815.0f

//#define REF 2000.0f
int REF=6000.0f;
//#define REFO 5100.0f

#define D_MAX 0.30
#define D_MIN 0.05

#define GANHO 0.00000000538


float corrente,corrente_sum,corrente_med;
int i=0;
float x[3];
float ybp[3];
float yap[3];
float ymv[2];


// Configura o ADC
void acordaADC(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ADCSequenceDisable(ADC0_BASE, 3);
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCReferenceSet(ADC0_BASE, ADC_REF_INT);

	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);
	ADCSequenceDisable(ADC0_BASE, 3);
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0,ADC_CTL_CH8 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCIntClear(ADC0_BASE, 3);
}



// Configura as gpio's que serão usadas
void acordaGPIO(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinConfigure(GPIO_PB6_M0PWM0); // PWM 1 Modulo 0 Gerador 0
	GPIOPinConfigure(GPIO_PB7_M0PWM1); // PWM 1 Modulo 0 Gerador 0 complementar
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}



// Interrupção do controle. Realiza a leitura da corrente e a compensação. Roda a 40KHz.
int cont = 0;
void PWM0IntHandler(void)
{
    PWMGenIntClear(PWM0_BASE,PWM_GEN_0,PWM_INT_CNT_ZERO);
	ADCProcessorTrigger(ADC0_BASE, 3);
	while (!ADCIntStatus(ADC0_BASE, 3, false)){}
	ADCSequenceDataGet(ADC0_BASE, 3, &lact);
	ADCIntClear(ADC0_BASE, 3);

	//====Miolo=do=controle==================================
	x[2]=x[1];
	x[1]=x[0];
	x[0]= REF - A*lact + B;

	//Malha de controle

	ymv[1]=ymv[0];

	ymv[0] = GANHO*(x[0]+x[1])+ymv[1];

	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

	// Hard - Limiter
	if (ymv[0] > D_MAX) {
		ymv[0] = D_MAX;
	    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
	} else if (ymv[0] < D_MIN) {
    	ymv[0] = D_MIN;
	}


	//Protecao contra picos de corrente
	if((A*lact - B)>10000){
	//	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0); /////   BOTAR A OUTRA
	}

	{
		dead=PWMPERIOD/2-ymv[0]*PWMPERIOD;
		PWMDeadBandEnable(PWM0_BASE, PWM_GEN_0, dead, dead);
	}
}



// Interrupção acessória, atualiza os leds de status e
// configura a dimerização, se habilitada. Roda a 10Hz.
int tipe=1;
int countdown=0,up=0;
void Timer0IntHandler(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	FLAG=!FLAG;
	countdown++;
	if(FLAG){
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	}
	else GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

	if(DIMM){
	    if(countdown>20){
		    countdown=0;
		    if(REF>9900)up=0; else if(REF<0)up=1;
		    if(up)REF=REF+1000; else REF=REF-1000;
		    }
	}

}






int main(void)
{
	volatile uint32_t ui32PWMClock;

	// Habilita a FPU
	FPUEnable();
	FPULazyStackingEnable();

        // Configura o Timer0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	ui32Period = (SysCtlClockGet());
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period/10 -1);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	TimerEnable(TIMER0_BASE, TIMER_A);

        // Configura gpio's e adc
	acordaGPIO();
	acordaADC();


	// Configura o PWM auxiliar
	ui32PWMClock = SysCtlClockGet();
	PWMPERIOD = (ui32PWMClock / PWM_FREQUENCY) -1;
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWMPERIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinConfigure(GPIO_PA6_M1PWM2);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);

	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWMPERIOD);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWMPERIOD / 2);
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);


	// Calcula um período inicial
	dutyi=duty*PWMPERIOD/100;
	dead=PWMPERIOD/2-dutyi;

	// Configura o PWM
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWMPERIOD/2);
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);

	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	PWMDeadBandEnable(PWM0_BASE, PWM_GEN_0, dead, dead);

	PWMIntEnable(PWM0_BASE,  PWM_INT_GEN_0);
	PWMGenIntTrigEnable(PWM0_BASE,PWM_GEN_0,PWM_INT_CNT_LOAD);

	// Ativa a interrupção do controle
	PWMGenIntRegister(PWM0_BASE,PWM_GEN_0,PWM0IntHandler);

	while(1){}
}
