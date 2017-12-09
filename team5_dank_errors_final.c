/*
 *************************************************************************************
 * ECE 4437
 * TEAM 5: DANK ERRORS
 * MEMBERS: ADITI TYAGI, BRANDON KAIN, AND HENRY NGUYEN
 *************************************************************************************
 */

/*
 *************************************************************************************
 * TO RUN THIS FILE, YOU NEED TO HAVE THESE RTOS PRODUCTS ENABLED
 *
 * SWI:
 * {{ Buffer_SWI | switchBuffers | 15 | 0x0 }}
 *
 * TIMER:
 * {{ Light_Timer | lightSensorCalculation | 2 | 10000 | timer starts automatically |
 *      periodic and continuous }}
 *
 * CLK: period = 50000 (us) | ANY | Timer Interrupt Every Period | SWI priority = 14
 * {{ Buffer_Clk | OutputBuffer | 1 | 40 | DO NOT start at boot time }}
 * {{ PID_Clk | prepPID | 1 | 1 | start at boot time }}
 *
 *
 *************************************************************************************
 */

/*
 *************************************************************************************
 * BIOS HEADER FILES
 *************************************************************************************
 */
#include <xdc/std.h>               //mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h>       //mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>       //needed for any Log_info() call
#include <xdc/cfg/global.h>        //header file for statically defined objects/handles
#include <xdc/runtime/Timestamp.h> //used for Timestamp() calls
/*
 *************************************************************************************
 * C HEADER FILES
 *************************************************************************************
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
/*
 *************************************************************************************
 * TIVA C HEADER FILES
 *************************************************************************************
 */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.c"
#include "driverlib/uart.c"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.c"
#include "driverlib/pwm.c"
#include "inc/hw_ints.h"
#include "driverlib/timer.c"
#include "driverlib/udma.h"
#include "inc/hw_udma.h"
#include "inc/hw_uart.h"
#include "driverlib/systick.h"

/*
 *************************************************************************************
 * DEFINING CONSTANTS
 *************************************************************************************
 */
//used constants to make code easy to read for PWM config
#define PWM_FREQ            10000
#define PWM_ADJUST          80
#define TARGET_VALUE        2000
#define SEQ1                1
#define SEQ2                2
#define SEQ3                3
#define SEQ4                4
#define PRI_0               0
#define PRI_1               1
#define STEP_0              0
#define BUFFER_SIZE         20 //size of each buffer

/*
 *************************************************************************************
 * MISC.
 *************************************************************************************
 */
char command[2] = "  "; //2-char command array

/*
 *************************************************************************************
 * ADC VALUE
 *************************************************************************************
 */
uint32_t rightSensorValue = 0; //right distance sensor ADC values
uint32_t frontSensorValue = 0; //right distance sensor ADC values

/*
 *************************************************************************************
 * PWM VALUES
 *************************************************************************************
 */
volatile uint32_t PWM_CLOCK, PWM_LOAD;

/*
 *************************************************************************************
 * PID VALUES
 *************************************************************************************
 */
volatile float proportionalRight;
volatile float lastProportionalRight;
volatile float integralRight = 0;
volatile float derivativeRight;
volatile float pidRight;

/*
 *************************************************************************************
 * LIGHT SENSOR VALUES
 *************************************************************************************
 */
int blkLineCounter = 0;
int readData = 1; //robot should read data on the first pass of thin line

/*
 *************************************************************************************
 * PING PONG BUFFER VALUES
 *************************************************************************************
 */
volatile int buffer[BUFFER_SIZE];
volatile int buffer_2[BUFFER_SIZE];
volatile int temp_buffer[BUFFER_SIZE];
signed int error = 0;
int i = 0;
int j = 0;
int k = 0;
int m = 0;
int swap = 0;
int error_count = 1;
int bufferCt = 0;

/*
 *************************************************************************************
 * DECLARING FUNCTIONS
 *************************************************************************************
 */
void ConfigurePeripherals(void);
void ConfigureUART(void);
void ConfigureADC(void);
void ConfigurePWM(void);
void PID(int RightValue, int FrontValue);
void prepPID(void);
void OutputBuffer(void);
void lightSensorCalculation(void);
void switchBuffers(void);

/*
 *************************************************************************************
 * INITIALIZE EVERYTHING
 *************************************************************************************
 */
void ConfigurePeripherals(void) {

    // Set system clock
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // LEDs for reading data
    // Configuring red, blue, and green LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    // LEDs initially set to input to make sure LEDs are not displaying

    // Initialize UART, PWM, and ADC functionalities
    ConfigureUART();
    ConfigurePWM();
    ConfigureADC();
}

/*
 *************************************************************************************
 * UART CONFIG
 *************************************************************************************
 */
// Enabling UART peripheral
void ConfigureUART(void)
{
    // Enable the clocks to PortB and UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    // Configure PortB pins 0 & 1 for UART1 RX & TX, respectively
    GPIOPinConfigure(GPIO_PB0_U1RX); //goes to TX pin
    GPIOPinConfigure(GPIO_PB1_U1TX); //goes to RX pin
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Set the UART1 module's clock source
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));

    // Configure UART1
    // UART module: 1
    // Baud rate: 115200
    // UART clock speed: 16 [MHz]
    UARTStdioConfig(1, 115200, 16000000);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_TX);
}

/*
 *************************************************************************************
 * ADC CONFIG
 *************************************************************************************
 */
// Enabling ADC peripheral
void ConfigureADC(void) {

    // Enable the clock for ADC0 and PortE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Configure PortE pins 2 & 3 for ADC usage | Pin3 = right sensor Pin2 = front sensor
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

    // Disable sequencers to ensure safe reconfiguration of them
    ADCSequenceDisable(ADC0_BASE, SEQ1); //disable sequence 1
    ADCSequenceDisable(ADC0_BASE, SEQ2); //disable sequence 2
    ADCSequenceDisable(ADC0_BASE, SEQ3); //disable sequence 3
    ADCSequenceDisable(ADC0_BASE, SEQ4); //disable sequence 4

    // Configure sequence priorities and triggers
    // SS1 - to sample right sensor | priority = 0
    // SS2 - to sample front sensor | priority = 1
    ADCSequenceConfigure(ADC0_BASE, SEQ1, ADC_TRIGGER_PROCESSOR,
                         PRI_0);
    ADCSequenceConfigure(ADC0_BASE, SEQ2, ADC_TRIGGER_PROCESSOR,
                         PRI_1);

    // Configuring sequence steps for sequence 1
    // Step 0: sample right sensor, end of sequence
    ADCSequenceStepConfigure(ADC0_BASE, SEQ1, STEP_0,
                             (ADC_CTL_CH0 | ADC_CTL_END));

    // Configuring sequence steps for sequence 2
    // Step 0: sample front sensor, end of sequence
    ADCSequenceStepConfigure(ADC0_BASE, SEQ2, STEP_0,
                             (ADC_CTL_CH1 | ADC_CTL_END));

    // Re-enable configured sequences
    ADCSequenceEnable(ADC0_BASE, SEQ1);
    ADCSequenceEnable(ADC0_BASE, SEQ2);
}

/*
 *************************************************************************************
 * PWM CONFIG
 *************************************************************************************
 */
// Enabling PWM peripheral
void ConfigurePWM(void)
{
    // Set the PWM module's clock divider
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64); //16[MHz]/64

    // Enable the clock for PWM1 and PortA, PortB, PortE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    //Phase pins and mode
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6|GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //L motor set forward
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //R motor set forward
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7); //PHASE/ENABLE Mode

    // Configure PortA pins 6 & 7 for PWM module 1, generator 1 usage
    GPIOPinConfigure(GPIO_PA6_M1PWM2);
    GPIOPinConfigure(GPIO_PA7_M1PWM3);
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    // Configure M1PWM0 for count down mode
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);

    // Calculate the PWM clock and load values
    PWM_CLOCK = SysCtlClockGet() / 64;
    PWM_LOAD = (PWM_CLOCK / PWM_FREQ) - 1;

    // Set the period of the PWM generator
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWM_LOAD);

    // Specify the duty cycle for the PWM signal
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, PWM_ADJUST * PWM_LOAD / 100); //left motor
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, PWM_ADJUST * PWM_LOAD / 100); //right motor

    // Enable PWM output
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);

    // Enable the timer/counter for M1PWM0
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);

}

/*
 *************************************************************************************
 * CLOCK FUNCTION 1 - OUTPUT BUFFER TO TERMINAL (PC)
 *************************************************************************************
 */
    /*
     *********************************************************************************
     *  Print the error values to the terminal (PuTTy)
     *
     *  A variable "swap" ensures that ping-pong mechanism is followed.
     *
     *  buffer[] and buffer_2[] prints out alternate error values.
     *
     *  Buffer_Clk trigger this interrupt every 2 seconds.
     *
     *  When the UARTBusy function is 0 or NULL
     *      it implies that values are being printed onto PuTTY
     *          while the other buffer is being filled.
     *********************************************************************************
     */
void OutputBuffer(void) {


    if (swap == 0) {

        // Reset LEDs
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
        // Constant green LED to signal that it's transmitting to PC
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

        // Transmitting buffer to PC in specified format
        UARTprintf(": ");
        for (j = 0; j < BUFFER_SIZE; ++j) {
            UARTprintf("%X, ", buffer[j]);

            //If UART is busy, store values into second buffer
            if(UARTBusy(UART1_BASE)) {
                buffer_2[j] = temp_buffer[j];
            }
        }
        UARTprintf("\r\n\n");
        // End of transmission

        //Post to Buffer_SWI which calls switchBuffers function
        Swi_post(Buffer_SWI);
    }

    else if (swap == 1) {
        // Reset
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
        // Constant green LED to signal that it's transmitting to PC
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

        // Transmitting buffer to PC in specified format
        UARTprintf(": ");
        for (j = 0; j < BUFFER_SIZE; ++j) {
            UARTprintf("%X, ", buffer_2[j]);

            //If UART is busy, store values into first buffer
            if(UARTBusy(UART1_BASE)) {
                buffer[j] = temp_buffer[j];
            }
        }
        UARTprintf("\r\n\n");
        // End of transmission

        //Post to Buffer_SWI which calls switchBuffers function
        Swi_post(Buffer_SWI);
    }

    // Resets index to 0 for temp_buffer[] array in the PID function
    i = 0;
}

/*
 *************************************************************************************
 * SWI INTERRUPT FUNCTION - SWITCH BUFFERS
 *************************************************************************************
 */
    /*
     *********************************************************************************
     * A simple function that handles the ping-pong mechanic
     *  which allows switch the active buffer.
     *********************************************************************************
     */
void switchBuffers(void) {
    // Use other buffer
    if (swap == 0) {
        swap = 1;
    }
    else if (swap == 1) {
        swap = 0;
    }
}

/*
 *************************************************************************************
 * CLOCK FUNCTION 2 - GET ADC VALUES AND RUN PID
 *************************************************************************************
 */
    /*
     *********************************************************************************
     * PID_Clk triggers the ADC0_Base to acquire the ADC samples from the
     *  front and right sensors.
     *
     * Runs every 50[ms].
     *
     * ADC values sent to PID function
     *********************************************************************************
     */
void prepPID(void) {

    // Trigger the sample sequence 1
    ADCProcessorTrigger(ADC0_BASE, SEQ1);

    // Get results from sample sequence 1
    ADCSequenceDataGet(ADC0_BASE, SEQ1, &rightSensorValue);

    // Trigger sample sequence 2
    ADCProcessorTrigger(ADC0_BASE, SEQ2);

    // Get results from sample sequence 2
    ADCSequenceDataGet(ADC0_BASE, SEQ2, &frontSensorValue);

    // Calls PID function to perform PID using the given sensor values
    PID(rightSensorValue, frontSensorValue);
}

/*
 *************************************************************************************
 * PID
 *************************************************************************************
 */
    /*
     *  This function essentially follows a PID controller.
     *      Gain values for the proportional, integral and
     *          derivative terms are selected to tune the robot to follow the wall.
     *
     *  The proportional gain = 1/20
     *  The integral gain = 1/10000
     *  The differential gain = 3/2
     *
     *  The target value chosen is 2000.
     *
     *  The (current ADC values - target value) is defined as the error values
     *      which will be stored in a buffer.
     *
     *  The ranges determine how close it is to the wall
     *      and what actions must be taken to prevent the robot from crashing into the wall.
     *
     *  The error values are stored in the buffer which is used in the OutputBuffer()
     */
void PID(int RightValue, int FrontValue) {

    // Calculate proportional terms
    proportionalRight = (RightValue - TARGET_VALUE) / 20;

    // Calculate integral terms
    integralRight = (RightValue - TARGET_VALUE);

    // Calculate derivative terms
    derivativeRight = ((RightValue - TARGET_VALUE) - lastProportionalRight) * (3 / 2);

    // Calculate PID result
    pidRight = proportionalRight + (integralRight / 10000) + derivativeRight;

    // Update some values for proper calculations of the next PID update
    lastProportionalRight = (RightValue - TARGET_VALUE);

    // Check if dead end & U-Turn
    if ((pidRight < 25) && (FrontValue > 2000))
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0); //left-motor-backward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //right-motor-forward


        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 99 * PWM_LOAD / 100); //left motor
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 99 * PWM_LOAD / 100); //right motor
    }
    // Turn Left
    else if ((pidRight > 27) && (FrontValue < 1000))
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //forward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //forward

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 70 * PWM_LOAD / 100); //left slow
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 80 * PWM_LOAD / 100); //right
    }
    // Turn Right
    else if ((pidRight > -80) && (pidRight < -20) && (FrontValue < 1000))
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //forward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //forward

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 75 * PWM_LOAD / 100); //left
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 65 * PWM_LOAD / 100); //right slow
    }
    // Sharp Right (used when the robot encounters an intersection)
    else if (pidRight < -100 && FrontValue < 1400)
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //forward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //forward

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 99 * PWM_LOAD / 100); //left
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 17 * PWM_LOAD / 100); //right slow
        SysCtlDelay(500); //make sure robot does not exit out of a turn too early
    }
    // Go straight
    else if ((pidRight > -20) && (pidRight < 20) && (FrontValue < 1800))
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //forward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //forward

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 70 * PWM_LOAD / 100);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 70 * PWM_LOAD / 100);
    }
    // Special straight (used to prevent robot from hitting wall during sharp right turn)
    else if ((pidRight > -50) && (pidRight < 0) && (FrontValue > 1000) && (FrontValue < 1500))
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //forward
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6); //forward

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 80 * PWM_LOAD / 100);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 80 * PWM_LOAD / 100);
    }

    /*
     * Calculates error and stores in buffer
     *
     * Only runs every 100[ms] by only running when error_count is even
     *
     * Manually fill both buffers once to prevent outputting empty buffers initially
     *  fills first buffer when bufferCt is between 0 and 19
     *  fills second buffer when bufferCt is between 20 and 39
     *
     */
    if ((error_count % 2) == 0) {
        // Reset LEDs
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
        // Constant blue LED to signal that it's collecting data
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        // Error = measured distance - desired distance
        error = RightValue - TARGET_VALUE;
        // Makes sure error is positive (absolute value)
        if (error < 0) {
            error = error * (-1);
        }
        // Fills the first buffer manually once
        if (bufferCt < 20) {
            buffer[k] = error;
            ++k;
        }
        // Fills the second buffer manually once
        if ((bufferCt >= 20) && (bufferCt < 40)) {
            buffer_2[m] = error;
            ++m;
        }
        // Fill temporary buffer for the remainder of the program
        if (i < BUFFER_SIZE) {
            temp_buffer[i] = error;
        }
        ++i; //increment temp_buffer[] index
        ++bufferCt; //used to fill first two buffers manually
        error_count = 0; //restart error counter

    }
    error_count += 1; //increment error counter
}

/*
 *************************************************************************************
 * TIMER FUNCTION - LIGHT SENSOR
 *************************************************************************************
 */
    /*
     *********************************************************************************
     *  Gets light sensor value from light sensor by timing how long it takes for
     *      the light sensor's voltage reading to drop to zero
     *
     *  Use light sensor value to determine surface (white or black)
     *
     *  If it's a black surface, increment blkLineCounter to determine the width
     *      Through testing, we were able to determine the range that blkLineCounter
     *          could fall into for a thin and thick line
     *
     * If the robot cross the thin line for a first time, then read data
     *
     * If the robot cross the thin line for a second time, then stop reading data
     *  and output partial buffer.
     *
     * If the robot cross the thick line, then stop program.
     *
     *********************************************************************************
     */
void lightSensorCalculation(void) {
    // Clear timer2A interrupt flag
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    // Light sensor config
    uint32_t lightSensorValue = 0;
    uint32_t lightCounter = 0;
    // Set light sensor pin to output
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
    // Output voltage to light sensor
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
    SysCtlDelay(100); //give time to charge
    // Set to input to read changes in voltage
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    // Time how long it takes for voltage to drop to zero
    while (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) != 0) {
        lightCounter++;
    }
    lightSensorValue = lightCounter;

    // Determine White or Black Surface
    if (lightSensorValue > 2000) {
        // Black Surface
        ++blkLineCounter; //used to determine width of black line
    }
    else {
        // White Surface
        // If black surface is a thin line, read data
        if ((blkLineCounter > 1) && (blkLineCounter < 10) && (readData == 1)) {
            // Start Buffer_Clk function in RTOS
            Clock_start(Buffer_Clk);
            readData = 0; //indicate that data has been read
            blkLineCounter = 0; //reset counter

            // Set to output to display LEDs
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
            // Used to indicate where reading starts on PuTTY
            UARTprintf("\n\n*********READING DATA*********\n\n");
        }
        // If thin line has been crossed the 2nd time
        else if ((blkLineCounter > 1) && (blkLineCounter < 10) && (readData == 0)) {
            // Stops Buffer_Clk
            Clock_stop(Buffer_Clk);

            // Outputs Partially filled buffer
            if (swap == 0) {
                // Reset LEDs
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
                // Constant green LED to signal that it's transmitting to PC
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
                UARTprintf("Partial Buffer: ");
                for (j = 0; j < BUFFER_SIZE; ++j) {
                    UARTprintf("%X, ", buffer[j]);
                    if(UARTBusy(UART1_BASE)) {
                        buffer_2[j] = temp_buffer[j];
                    }
                }
                UARTprintf("\r\n\n");
            }

            else if (swap == 1) {
                // Reset LEDs
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
                // Constant green LED to signal that it's transmitting to PC
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
                UARTprintf("Partial Buffer: ");
                for (j = 0; j < BUFFER_SIZE; ++j) {
                    UARTprintf("%X, ", buffer_2[j]);

                    if(UARTBusy(UART1_BASE)) {
                        buffer[j] = temp_buffer[j];
                    }
                }
                UARTprintf("\r\n\n");
            }
            i = 0;
            // Output of partial buffer is complete

            blkLineCounter = 0; //reset counter

            // Reset LEDs
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
            // Set to input to prevent LED from turning back on
            GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
            // Used to indicate where reading stops on PuTTY
            UARTprintf("\n\n!!!!!!!!!!!!!STOPPED READING!!!!!!!!!!!!!\n\n");
        }
        // If black surface is a thick line, stop program
        else if (blkLineCounter > 10) {
            PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);
            Clock_stop(PID_Clk);
            Clock_stop(Buffer_Clk);

            /*Reset Counter*/
            blkLineCounter = 0;

            // Set to output to display LEDs
            GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
            // Reset LEDs
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
            // Turn on red LED to signal the robot has stopped
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

            // Used to indicate that program has stopped on PuTTY
            UARTprintf("\n\n===========RUN COMPLETED===========\n\n");
        }
    }

    lightCounter = 0; //reset light sensor value
}

/*
 *************************************************************************************
 * MAIN
 *************************************************************************************
 */
 int main(void) {

    // Initialize everything
    ConfigurePeripherals();

    // Turn off motor to prevent robot resuming from last run
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);
    // Reset LEDs
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);

    // Menu on terminal
    while(true) {

        UARTprintf("\n");
        UARTprintf("Version: FINAL\n");
        UARTprintf("The following is a list of commands:\n"
                "GO - Run Maze\n");
        UARTgets(command, strlen(command) + 1);
        UARTprintf("\n");
        // Start run
        if (!strcmp(command, "GO"))
        {
            PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
            // Initialize RTOS
            BIOS_start();
        }
    }
}
