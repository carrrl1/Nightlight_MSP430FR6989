/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//***************************************************************************************
//  Blink the LED Demo - Software Toggle P1.0
//
//  Description; Toggle P1.0 inside of a software loop.
//  ACLK = n/a, MCLK = SMCLK = default DCO
//
//                MSP430x5xx
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |             P1.0|-->LED
//
//  E. Chen
//  Texas Instruments, Inc
//  March 2015
//  Built with Code Composer Studio v6
//***************************************************************************************

#include <driverlib.h>
#include "HAL_I2C.hh"
#include "HAL_OPT3001.hh"

// Initialization calls
void Init_Clock(void);
void Init_GPIO(void);
void Init_Light_Sensor(void);
void Init_State(void);

void blinkingIndicator(void);
void runningIndicator(void);
void toggleLED(void);

// Defines
/* Define the wattage, it will define wich LED to turn on or off. The wattage could be 5 (RED), 10 (GREEN) or 15 (BLUE). */
#define wattage 15

// Global variables
extern volatile bool g_bState=false;
extern volatile int g_iTimeCounter=0;
extern volatile float g_fLux=100;
extern volatile bool g_bButtonPressed=false;

//Timer A configuration.
Timer_A_initUpModeParam initUpParam_A0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/4 = 2MHz
        30000,                                  // 30ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR,                       // Clear value
        true                                    // Start Timer
};

int main(void) {

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    __disable_interrupt();

    // Configure GPIO
    Init_GPIO();

    // Configure light sensor i2c
    Init_Light_Sensor();

    //Configure the first state of the lamp. If there is not enough
    Init_State();

    // Configure clocks
    Init_Clock();

    __enable_interrupt();

    // To indicate the setup has finished
    blinkingIndicator();

    while(1)
    {
        runningIndicator();
    }
}

/****************************************
 *  INIT METHODS
 ****************************************/
/*
 * Clock System Initialization
 */
void Init_Clock(void)
{
    // Set PJ.4 and PJ.5 as Secondary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_PJ,
           GPIO_PIN4 + GPIO_PIN5,
           GPIO_PRIMARY_MODULE_FUNCTION
           );

    CS_setExternalClockSource(32768, 0);
    // Intializes the XT1 crystal oscillator
    CS_turnOnLFXT(CS_LFXT_DRIVE_3);

    /* Initializes Clock System DCO = 8MHz */
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
}

// Method to initialize GPIO
void Init_GPIO(void) {
    // Set P1.0 to output direction
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P9,GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9,GPIO_PIN7);

    switch (wattage) {
        case 5:
            GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
            break;
        case 10:
            GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN3);
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3);
            break;
        case 15:
            GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
            break;
        default:
            GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
    }

    // Configure button S2 (P1.2) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}

/* Method to initialize the the light sensor and I2C  */
void Init_Light_Sensor(void) {
    /* Initialize I2C communication */
    I2C_initGPIO();
    I2C_init();

    /* Initialize OPT3001 digital ambient light sensor */
    OPT3001_init();
}

/* Method to initialize the state of the lamp */
void Init_State(void) {
    /* Obtain lux value from OPT3001 */
    g_fLux = OPT3001_getLux();

    /* If less than 5 lux then turn on*/
    if(g_fLux<5) {
        toggleLED();
    }
}

/* Initializes TIMER
void TIMER_init()
{
    //Start timer in continuous mode sourced by ACLK
    Timer_A_clearTimerInterrupt(TIMER_A1_BASE);

    Timer_A_initUpModeParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = 32768;
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = true;
    Timer_A_initUpMode(TIMER_A1_BASE, &param);
} */

/****************************************
 *  FUNCTIONS
 ****************************************/

/* Method to blink the LED  */
void blinkingIndicator(void) {
    volatile uint32_t j;
    volatile uint32_t n;
    for(n=6; n>0; n--) {
        // Toggle P1.0 output
        GPIO_toggleOutputOnPin(GPIO_PORT_P9,GPIO_PIN7);
        // Delay
        for(j=40000; j>0; j--);
    }
    GPIO_setOutputLowOnPin(GPIO_PORT_P9,GPIO_PIN7);
}

/* Method to blink the LED  */
void runningIndicator(void) {
    volatile uint32_t i;
    // Toggle P1.0 output
    GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);

    // Delay
    for(i=10000; i>0; i--);
}

/* Method to toggle the LED  */
void toggleLED(void) {
    switch (wattage) {
        case 5:
            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN6);
            break;
        case 10:
            GPIO_toggleOutputOnPin(GPIO_PORT_P3, GPIO_PIN3);
            break;
        case 15:
            GPIO_toggleOutputOnPin(GPIO_PORT_P3, GPIO_PIN6);
            break;
        default:
            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN6);
    }
    g_bState=!g_bState;
}

/****************************************
 *  INTERRUPT SERVICE ROUTINES
 ****************************************/

/*
 * Timer A0 Interrupt Service Routine
 * Used as button debounce timer
 */
extern "C" {

/*
 * PORT1 Interrupt Service Routine
 * Handles S1 and S2 button press interrupts
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    if(__even_in_range(P1IV, P1IV_P1IFG7)==P1IV_P1IFG2)
    {
        if(g_bButtonPressed){
            // Set debounce flag on first high to low transition
            g_bButtonPressed=false;

            //Lights up!
            toggleLED();
        }

        // Start debounce timer
        Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
    }
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
    // Button S2 released
    if (P1IN & BIT2)
    {
        g_bButtonPressed=true;
        if(g_bState){
                if(g_iTimeCounter==2000){
                    g_iTimeCounter=0;
                    //After 30s lights down!
                    toggleLED();
                    Timer_A_stop(TIMER_A0_BASE);
                } else {
                    g_iTimeCounter++;
                    Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
                    __bic_SR_register_on_exit(LPM3_bits);
                }
        } else Timer_A_stop(TIMER_A0_BASE);
    }
}
}
