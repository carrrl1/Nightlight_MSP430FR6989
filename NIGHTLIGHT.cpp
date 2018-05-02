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
void ADC_init(void);
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
extern signed short audio_past=0;

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

// TimerA UpMode Configuration Parameter
Timer_A_initUpModeParam initUpParam_A1 =
{
    TIMER_A_CLOCKSOURCE_ACLK,               // ACLK Clock Source
    TIMER_A_CLOCKSOURCE_DIVIDER_1,          // ACLK/1 = 32768Hz
    0x2000,                                 // Timer period
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE ,   // Disable CCR0 interrupt
    TIMER_A_DO_CLEAR                        // Clear value
};

Timer_A_initCompareModeParam initCompParam =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_1,        // Compare register 1
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE, // Disable Compare interrupt
    TIMER_A_OUTPUTMODE_RESET_SET,             // Timer output mode 7
    0x1000                                    // Compare value
};

// Configure the ADC12B Module
    /*
     * Base address of ADC12B Module
     * Use internal ADC12B bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider/pre-divider of 1
     * Use Temperature Sensor internal channel
     */
ADC12_B_initParam initParam =
{
     ADC12_B_SAMPLEHOLDSOURCE_4,
     ADC12_B_CLOCKSOURCE_ADC12OSC,
     ADC12_B_CLOCKDIVIDER_1,
     ADC12_B_CLOCKPREDIVIDER__1,
     ADC12_B_TEMPSENSEMAP
};

// Configure Memory Buffer
    /*
     * Base address of the ADC12B Module
     * Configure memory buffer 0
     * Map input A30 to memory buffer 0
     * Vref+ = VRef+
     * Vref- = Vref-
     * Memory buffer 0 is the end of a sequence
     */
ADC12_B_configureMemoryParam configureMemoryParam =
{
     ADC12_B_MEMORY_0,
     ADC12_B_INPUT_A11,
     ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS,
     ADC12_B_ENDOFSEQUENCE,
     ADC12_B_WINDOW_COMPARATOR_DISABLE,
     ADC12_B_DIFFERENTIAL_MODE_DISABLE
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

    //Configure microphone ADC
    ADC_init();

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
    /*
    // Set PJ.4 and PJ.5 as Secondary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_PJ,
           GPIO_PIN4 + GPIO_PIN5,
           GPIO_PRIMARY_MODULE_FUNCTION
           );

    CS_setExternalClockSource(32768, 0);
    // Intializes the XT1 crystal oscillator
    CS_turnOnLFXT(CS_LFXT_DRIVE_3);

    // Initializes Clock System DCO = 8MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);*/
    // Set DCO frequency to default 8MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);

    // Configure MCLK and SMCLK to default 2MHz
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_8);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_8);

    // Intializes the XT1 crystal oscillator
    CS_turnOnLFXT(CS_LFXT_DRIVE_3);
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

/* Initializes the ADC for the microphone data */
void ADC_init(void) {

    // Configuring GPIOs (9.3 Microphone A11)
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P9, GPIO_PIN3,GPIO_TERNARY_MODULE_FUNCTION);

    // Set P4.1 and P4.2 as Secondary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_PJ,
           GPIO_PIN4 + GPIO_PIN5,
           GPIO_PRIMARY_MODULE_FUNCTION
           );

    // Select internal ref = 1.2V
    Ref_A_setReferenceVoltage(REF_A_BASE,
                              REF_A_VREF1_2V);
    // Internal Reference ON
    Ref_A_enableReferenceVoltage(REF_A_BASE);

    // Enables the internal temperature sensor
    Ref_A_enableTempSensor(REF_A_BASE);

    while(!Ref_A_isVariableReferenceVoltageOutputReady(REF_A_BASE));

    // Initialize the ADC12B Module
    /*
     * Base address of ADC12B Module
     * Use internal ADC12B bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider/pre-divider of 1
     * Use Temperature Sensor internal channel
     */
    ADC12_B_init(ADC12_B_BASE, &initParam);

    // Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);

    /*
     * Base address of ADC12B Module
     * For memory buffers 0-7 sample/hold for 256 clock cycles
     * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
     * Disable Multiple Sampling
     */
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
                               ADC12_B_CYCLEHOLD_16_CYCLES,
                               ADC12_B_CYCLEHOLD_4_CYCLES,
                               ADC12_B_MULTIPLESAMPLESDISABLE);



    // Initialize memory buffer
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    // Clear any interrupt
    ADC12_B_clearInterrupt(ADC12_B_BASE,
                              0,
                              ADC12_B_IFG0
                              );

    // Enable memory buffer 0 interrupt
    ADC12_B_enableInterrupt(ADC12_B_BASE,
                            ADC12_B_IE0,
                            0,
                            0);
    // Start ADC conversion
    ADC12_B_startConversion(ADC12_B_BASE, ADC12_B_START_AT_ADC12MEM0, ADC12_B_REPEATED_SINGLECHANNEL);

    // TimerA1.1 (125ms ON-period) - ADC conversion trigger signal
    Timer_A_initUpMode(TIMER_A1_BASE, &initUpParam_A1);

    // Initialize compare mode to generate PWM1
    Timer_A_initCompareMode(TIMER_A1_BASE, &initCompParam);

    // Start timer A1 in up mode
    Timer_A_startCounter(TIMER_A1_BASE,
        TIMER_A_UP_MODE
        );
}


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
        for(j=10000; j>0; j--);
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
                    //__bic_SR_register_on_exit(LPM3_bits);
                }
        } else Timer_A_stop(TIMER_A0_BASE);
    }
}

/*
 * ADC 12 Interrupt Service Routine
 * Wake up from LPM3 to display temperature
 */
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    if (__even_in_range(ADC12IV,12)==12) {
        ADC12_B_clearInterrupt(ADC12_B_BASE, 0, ADC12_B_IFG0);
        //Get the result
        signed short audio =  0;
        audio=ADC12MEM0;
        GPIO_toggleOutputOnPin(GPIO_PORT_P9,GPIO_PIN7);
        if (audio>audio_past) {
            //GPIO_toggleOutputOnPin(GPIO_PORT_P9,GPIO_PIN7);
            audio_past=audio;
        }
        // Start another ADC conversion
        //ADC12_B_startConversion(ADC12_B_BASE, ADC12_B_START_AT_ADC12MEM0, ADC12_B_REPEATED_SINGLECHANNEL);
        //__bic_SR_register_on_exit(LPM3_bits);   // Exit active CPU
    }
    /*
    switch(__even_in_range(ADC12IV,12))
    {
    case  0: break;                         // Vector  0:  No interrupt
    case  2: break;                         // Vector  2:  ADC12BMEMx Overflow
    case  4: break;                         // Vector  4:  Conversion time overflow
    case  6: break;                         // Vector  6:  ADC12BHI
    case  8: break;                         // Vector  8:  ADC12BLO
    case 10: break;                         // Vector 10:  ADC12BIN
    case 12:                                // Vector 12:  ADC12BMEM0 Interrupt
        ADC12_B_clearInterrupt(ADC12_B_BASE, 0, ADC12_B_IFG0);
        //Get the result
        signed short audio =  0;
        audio=ADC12MEM0;
        GPIO_toggleOutputOnPin(GPIO_PORT_P9,GPIO_PIN7);
        if (audio>0) {
            GPIO_toggleOutputOnPin(GPIO_PORT_P9,GPIO_PIN7);
        }
        // Start another ADC conversion
        ADC12_B_startConversion(ADC12_B_BASE, ADC12_B_START_AT_ADC12MEM0, ADC12_B_REPEATED_SINGLECHANNEL);
        //__bic_SR_register_on_exit(LPM3_bits);   // Exit active CPU
        break;                              // Clear CPUOFF bit from 0(SR)
    case 14: break;                         // Vector 14:  ADC12BMEM1
    case 16: break;                         // Vector 16:  ADC12BMEM2
    case 18: break;                         // Vector 18:  ADC12BMEM3
    case 20: break;                         // Vector 20:  ADC12BMEM4
    case 22: break;                         // Vector 22:  ADC12BMEM5
    case 24: break;                         // Vector 24:  ADC12BMEM6
    case 26: break;                         // Vector 26:  ADC12BMEM7
    case 28: break;                         // Vector 28:  ADC12BMEM8
    case 30: break;                         // Vector 30:  ADC12BMEM9
    case 32: break;                         // Vector 32:  ADC12BMEM10
    case 34: break;                         // Vector 34:  ADC12BMEM11
    case 36: break;                         // Vector 36:  ADC12BMEM12
    case 38: break;                         // Vector 38:  ADC12BMEM13
    case 40: break;                         // Vector 40:  ADC12BMEM14
    case 42: break;                         // Vector 42:  ADC12BMEM15
    case 44: break;                         // Vector 44:  ADC12BMEM16
    case 46: break;                         // Vector 46:  ADC12BMEM17
    case 48: break;                         // Vector 48:  ADC12BMEM18
    case 50: break;                         // Vector 50:  ADC12BMEM19
    case 52: break;                         // Vector 52:  ADC12BMEM20
    case 54: break;                         // Vector 54:  ADC12BMEM21
    case 56: break;                         // Vector 56:  ADC12BMEM22
    case 58: break;                         // Vector 58:  ADC12BMEM23
    case 60: break;                         // Vector 60:  ADC12BMEM24
    case 62: break;                         // Vector 62:  ADC12BMEM25
    case 64: break;                         // Vector 64:  ADC12BMEM26
    case 66: break;                         // Vector 66:  ADC12BMEM27
    case 68: break;                         // Vector 68:  ADC12BMEM28
    case 70: break;                         // Vector 70:  ADC12BMEM29
    case 72: break;                         // Vector 72:  ADC12BMEM30
    case 74: break;                         // Vector 74:  ADC12BMEM31
    case 76: break;                         // Vector 76:  ADC12BRDY
    default: break;
    }*/
}

}
