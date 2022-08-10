#include "derivative.h" /* include peripheral declarations */
#include "utilities.h"
#include "clock.h"
#include <stdio.h>

// System clock frequency
#define SYSTEM_CLOCK_FREQUENCY (48000000UL)
// Choice of prescale value (FTM0_SC.PS)
#define FTM0_PRESCALE_MASK (6)
#define FTM0_PRESCALE_VALUE (1<<FTM0_PRESCALE_MASK)
#define FTM0_CLK_FREQUENCY (SYSTEM_CLOCK_FREQUENCY/FTM0_PRESCALE_VALUE)
#define PWM_TEN_MICROSECOND ((FTM0_CLK_FREQUENCY/100000) + 1)
#define PWM_PERIOD (8000*PWM_TEN_MICROSECOND)

#define FTM1_PRESCALE_MASK (4)
#define FTM1_PRESCALE_VALUE (1<<FTM1_PRESCALE_MASK)
#define FTM1_FREQUENCY (SYSTEM_CLOCK_FREQUENCY / FTM1_PRESCALE_VALUE)
#define ECHO_ONE_MICROSECOND (FTM1_FREQUENCY / 1000000)

static volatile uint16_t echoHighTimeTicks;

#define BUZZER_MASK (1<<0)
#define RED_MASK (1<<3)
#define GREEN_MASK (1<<4)
#define BLUE_MASK (1<<2)

unsigned int count = 0;

#if (PWM_PERIOD<1000) // Check resolution
#error "PWM_PERIOD is too small"
#endif
#if (PWM_PERIOD>65535)
#error "PWM_PERIOD is too large"
#endif

/*
* Initialise ports
*/
void initialiseFTM0_PWM(int period) {
	// Initialise C.1(alt4) = FTM0.Ch1
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC_PCR1 = PORT_PCR_MUX(4)|PORT_PCR_DSE_MASK;
	
	// Enable clock to FTM
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
	// Common registers
	FTM0_SC = FTM_SC_CLKS(0); // Disable FTM so changes are immediate
	FTM0_CNTIN = 0;
	FTM0_CNT = 0;
	FTM0_MOD = period - 1;
	// Left aligned PWM since CPWMS not selected
	FTM0_SC = FTM_SC_CLKS(1)|FTM_SC_PS(FTM0_PRESCALE_MASK);
}

void setDutyCycle(int channel, float dutyCycle) {
	// High-true PWM pulses
	FTM0_CnSC(channel) = FTM_CnSC_MSB_MASK | // Edge-aligned PWM mode
						 FTM_CnSC_ELSB_MASK; // High-true pulses
	
	// PWM pulse width
	FTM0_CnV(channel) = (uint16_t)((dutyCycle*(FTM0_MOD+1)) / 100);
}

void initialiseFTM1(void) {
	// Initialise A.12(alt3) = FTM1.Ch0
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	PORTA_PCR12 = PORT_PCR_MUX(3)|PORT_PCR_DSE_MASK;
	
	// Enable clock to FTM
	SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;
	// Common registers
	FTM1_SC = FTM_SC_CLKS(0); // Disable FTM so changes are immediate
	FTM1_CNTIN = 0;
	FTM1_CNT = 0;
	FTM1_MOD = 0xFFFF;
	// Left aligned PWM since CPWMS not selected
	FTM1_SC = FTM_SC_CLKS(1)|FTM_SC_PS(FTM1_PRESCALE_MASK);
	
	NVIC_EnableIrq(INT_FTM1);
	
	FTM1_C0SC = FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK;
}

void FTM1_IRQHandler(void){
	
	static uint16_t risingEdge = 0, fallingEdge = 0;
	static unsigned short int edgeState = 1;
	
	// If a rising or falling edge is detected 
	if ((FTM1_C0SC & FTM_CnSC_CHF_MASK) != 0) {
		FTM1_C0SC = FTM_CnSC_CHIE_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK;
		
		// check if it is a rising edge
		if (edgeState == 1) {
			risingEdge = FTM1_C0V; // set rising edge as counter value from channel 0
			edgeState = 0;
		}
		
		// if it is not rising edge	
		else if (edgeState == 0){
			fallingEdge = FTM1_C0V;  // set falling edge as counter value from channel 0
			echoHighTimeTicks = fallingEdge - risingEdge; // calculate echo high time
			edgeState = 1;
		}
	}
	
	// unexpected interrupts
	if (FTM1_STATUS != 0) {
		__asm__("bkpt");
	}
}

int getDistanceCentimeters () {
	// Disable Interrupt to store the current echo tick
	NVIC_DisableIrq(INT_FTM1);
	unsigned int tempHighTimeTicks = echoHighTimeTicks;
	
	// Enable Interrupt and perform distance calculation and conversion
	NVIC_EnableIrq(INT_FTM1);
	int highTimeMs = tempHighTimeTicks / ECHO_ONE_MICROSECOND;
	int Distance = ((highTimeMs / 29.0) / 2.0); // Converts to centimeter
	return Distance;
}

// SysTick Functions
void SysTick_Handler(void) 
{
   count++;
}

void delayMS(unsigned int delay)
{
	count = 0;
	while(delay>count);
}

uint32_t SysTick_Config(uint32_t ticks) {
   if ((ticks - 1) > SysTick_RVR_RELOAD_MASK) {
      /* Reload value impossible */
      return (1);
   }
   /* Set reload register */
   
   SYST_RVR = SysTick_RVR_RELOAD(ticks-1);
   /* Set Priority for Systick Interrupt */
   NVIC_SetIrqPriority (INT_SysTick, (1<<4) - 1);
   /* Load the SysTick Counter Value */
   SYST_CVR = 0;
   /* Configure Systick */
   SYST_CSR =
         SysTick_CSR_CLKSOURCE_MASK |   // Use system core clock
         SysTick_CSR_TICKINT_MASK   |   // Enable interrupts
         SysTick_CSR_ENABLE_MASK;       // Enable timer
   /* Function successful */
   return (0);
}

void initialisePIT0() {
	// Enable clock to PIT
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	// Enable PIT module
	PIT_MCR = ~PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK;
	// Enable this channel with interrupts
	PIT_TCTRL0 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;
}

void PIT_Ch0_IRQHandler(void) {
//    Toggle the Buzzer
   GPIOE_PTOR = BUZZER_MASK;

//    Clear the interrupt flag
   PIT_TFLG0 = PIT_TFLG_TIF_MASK;
}

void playTone(unsigned short int frequency, unsigned short int duration){
	// Calculate interval for buzzer
	float Period = 1.0 / frequency;
	int interval = (int)((Period / 2) * 48000000);
	PIT_LDVAL0 = (uint32_t)interval - 1;

	NVIC_EnableIrq(INT_PIT0);
	delayMS(duration);
	NVIC_DisableIrq(INT_PIT0);
}

void playToneAlways(unsigned short int frequency){
	// Calculate interval for buzzer
	float Period = 1.0 / frequency;
	int interval = (int)((Period / 2) * 48000000);
	PIT_LDVAL0 = (uint32_t)interval - 1;

	NVIC_EnableIrq(INT_PIT0);
}

//most important function for car reverse sensor program
//controls necessary output actions depending on distance input received
void reverseSensorResponse(int distance) {
	//declare baseDuration of 100ms, this will be multiplied 
	//and scaled for different buzzing duration required
	unsigned short int baseDuration = 100;
	//constant 2000Hz frequency value
	unsigned short int frequency = 2000;
	unsigned int currentstatus;
	
	char status[4][10] = {"Far", "Near", "Very Near", "Stop"};
	
	//object is far away
	if (distance > 25){
		currentstatus = 0;
	}
	//object is near
	else if((18 < distance) && (distance <= 25)){
		currentstatus = 1;
	}
	//object is very near
	else if((10 < distance) && (distance <= 18)){
		currentstatus = 2;
	}
	//object is extremely near
	else {
		currentstatus = 3;
	}
	
	printf("Distance: %d cm | Status: %s\n", distance, status[currentstatus]);
	
	switch (currentstatus) {
		case 0:
			//play for 500ms
			playTone(frequency,baseDuration * 5);
			GPIOC_PSOR = RED_MASK;
			GPIOD_PSOR = GREEN_MASK;
			GPIOA_PCOR = BLUE_MASK;
			delayMS(100); // Provides small delay between tones
			break;
			
		case 1:
			//play for 300ms
			playTone(frequency,baseDuration * 3);
			GPIOC_PSOR = RED_MASK;
			GPIOD_PCOR = GREEN_MASK;
			GPIOA_PSOR = BLUE_MASK;
			delayMS(100); // Provides small delay between tones
			break;
			
		case 2:
			//play for 100ms
			playTone(frequency,baseDuration);
			GPIOC_PCOR = RED_MASK;
			GPIOD_PCOR = GREEN_MASK;
			GPIOA_PSOR = BLUE_MASK;
			delayMS(100); // Provides small delay between tones
			break;
			
		case 3:
			//play to simulate constant tone
			playToneAlways(frequency);
			GPIOC_PCOR = RED_MASK;
			GPIOD_PSOR = GREEN_MASK;
			GPIOA_PSOR = BLUE_MASK;
			delayMS(100); // Provides small delay between tones
			break;
	}
}

int main(void) {
	clock_initialise();
	
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTC_MASK;
	PORTE_PCR0 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	GPIOE_PDDR |= BUZZER_MASK;
	
	PORTA_PCR2 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTD_PCR4 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	PORTC_PCR3 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
	GPIOA_PDDR |= BLUE_MASK;
	GPIOD_PDDR |= GREEN_MASK;
	GPIOC_PDDR |= RED_MASK;
	
	SysTick_Config(48000); // set sysTick as 48000 ticks
	
	// initialise FTM0, FTM1 and PIT0
	initialisePIT0();
	initialiseFTM1();
	initialiseFTM0_PWM(PWM_PERIOD);
	
	// 10us/80ms * 100% is 0.0125%
	setDutyCycle(0,0.0125);
	
	GPIOC_PCOR = WARNING_MASK | NORMAL_MASK;
	
	for(;;) {
		if	(getDistanceCentimeters() < 5000){
			GPIOC_PSOR = NORMAL_MASK;
			GPIOC_PCOR = WARNING_MASK;
		}
		else {
			GPIOC_PSOR = WARNING_MASK;
			GPIOC_PCOR = NORMAL_MASK;
		}
		reverseSensorResponse(getDistanceCentimeters());
	}
}
