/*-----------------------------------------------------------------------
 * File:        Lab2_main.c
 * Developers:	M. Batchelder
                S. Thornburg
   
   Processor:	PIC32MX79F512L
   Board:       Max32
   Compiler:	XC32
   IDE:         MPLAB-X
   Date:        March 3, 2021
   Status:      Adapted for EE 456L/556L Lab 2
            
   Description:
  		Input/Output Demonstrator
  		Timer 2 generates interrupts at specified frequency
  		The ISR: 
            1. Reads the ADC and outputs the value to UART1
            2. Generates a reference signal from a waveform lookup table
  			3. Sends reference to a MCP4822 DAC on the SPI2 port
  		Illustrates how to
            1. Configure oscillator
            2. Use Timer
            3. Use SPI port
            4. Use interrupts with an ISR
            5. Generate a reference signal using direct digital synthesis
            6. Read values from an ADC channel
            7. Write data to a UART 
            7. Write values to an external DAC via SPI
        
-----------------------------------------------------------------------*/

//-----------------------------------------------------------------------
// INCLUDES
//-----------------------------------------------------------------------
#include <xc.h>         // Microchip hardware config for XC compilers
#include <stdio.h>      // IO functions for UART comms
#include <plib.h>       // Peripheral library functions
#include <stdint.h>     // Standard integer variable typedefs
// #include <math.h>       // Mathematical function library
#include "INCLUDES.h"   // Max32-specific functions

// ------------------------------------------------------------
// Global Variables
// ------------------------------------------------------------
// Sample period or frequency:
#define T           (0.2)   // Sample period (s) < 0.209 s
// Alternatively, define sample frequency (Hz) (uncomment next 2 lines)
// #define F_S        (500)    // Sample frequency (Hz) > 4.77 Hz
// #define T          (1/F_S)  // Sample period (s)

// Reference signal definition:
#define F_R         (0.1)   // Reference waveform frequency (Hz) < 1/(2*T)
#define R_HI_V      (3.0)   // High magnitude of reference wave (V) <= 3.3
#define R_LO_V      (1.0)   // Low magnitude of reference wave (V) >= 0.0

// Variables for DDS--used in this lab to create reference signal r(k)
uint16_t volatile phaseAccumulator = 0; // Holds DDS phase
uint16_t          phaseStep;            // DDS tuning word
uint8_t  volatile waveTableIndex = 0;   // index for wave lookup (upper 8 bits of phaseAccumulator)
int32_t           rHiInt;               // high integer value of reference signal
int32_t           rLoInt;               // low integer value of reference signal
int32_t  volatile rk = 0;               // current value of reference signal, r(k)

// DDS table for generating a sine waveshape using DDS
// One cycle of sine tone, (256) 8-bit samples, in hex values
// Any desired waveform could be placed here
uint8_t const SINETABLE[] = {
    0x80, 0x83, 0x86, 0x89, 0x8c, 0x8f, 0x92, 0x95, 0x98, 0x9b, 0x9e, 0xa2, 0xa5, 0xa7, 0xaa, 0xad,
    0xb0, 0xb3, 0xb6, 0xb9, 0xbc, 0xbe, 0xc1, 0xc4, 0xc6, 0xc9, 0xcb, 0xce, 0xd0, 0xd3, 0xd5, 0xd7,
    0xda, 0xdc, 0xde, 0xe0, 0xe2, 0xe4, 0xe6, 0xe8, 0xea, 0xeb, 0xed, 0xee, 0xf0, 0xf1, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf8, 0xf9, 0xfa, 0xfa, 0xfb, 0xfc, 0xfd, 0xfd, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfe, 0xfd, 0xfd, 0xfc, 0xfb, 0xfa, 0xfa, 0xf9, 0xf8, 0xf6,
    0xf5, 0xf4, 0xf3, 0xf1, 0xf0, 0xee, 0xed, 0xeb, 0xea, 0xe8, 0xe6, 0xe4, 0xe2, 0xe0, 0xde, 0xdc,
    0xda, 0xd7, 0xd5, 0xd3, 0xd0, 0xce, 0xcb, 0xc9, 0xc6, 0xc4, 0xc1, 0xbe, 0xbc, 0xb9, 0xb6, 0xb3,
    0xb0, 0xad, 0xaa, 0xa7, 0xa5, 0xa2, 0x9e, 0x9b, 0x98, 0x95, 0x92, 0x8f, 0x8c, 0x89, 0x86, 0x83,
    0x80, 0x7c, 0x79, 0x76, 0x73, 0x70, 0x6d, 0x6a, 0x67, 0x64, 0x61, 0x5d, 0x5a, 0x58, 0x55, 0x52,
    0x4f, 0x4c, 0x49, 0x46, 0x43, 0x41, 0x3e, 0x3b, 0x39, 0x36, 0x34, 0x31, 0x2f, 0x2c, 0x2a, 0x28,
    0x25, 0x23, 0x21, 0x1f, 0x1d, 0x1b, 0x19, 0x17, 0x15, 0x14, 0x12, 0x11, 0x0f, 0x0e, 0x0c, 0x0b,
    0x0a, 0x09, 0x07, 0x06, 0x05, 0x05, 0x04, 0x03, 0x02, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x02, 0x02, 0x03, 0x04, 0x05, 0x05, 0x06, 0x07, 0x09,
    0x0a, 0x0b, 0x0c, 0x0e, 0x0f, 0x11, 0x12, 0x14, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f, 0x21, 0x23,
    0x25, 0x28, 0x2a, 0x2c, 0x2f, 0x31, 0x34, 0x36, 0x39, 0x3b, 0x3e, 0x41, 0x43, 0x46, 0x49, 0x4c,
    0x4f, 0x52, 0x55, 0x58, 0x5a, 0x5d, 0x61, 0x64, 0x67, 0x6a, 0x6d, 0x70, 0x73, 0x76, 0x79, 0x7c
}; // SINETABLE

// Plant output y(k) is read from sensor attached to ADC
// ADC values are configured as unsigned 10-bit integers, expanded to 1 word
int32_t volatile yk;        // current plant output y(k)

// Parameters for MCP4822 12-bit DAC
#define DAC_NBITS   (12)    // number of bits used by DAC
#define DAC_MAX_INT (4095)  // Max MCP4822 (12-bits) output integer
#define DAC_MAX_V   (4.096) // Max MCP4822 output (V)

// Control signal u(k) amplitude is limited by the DAC's range, and can be
// further restricted if desired:
int32_t volatile uk = 0;    // current integer value of control, u(k)
#define U_MIN       (0)     // min allowable integer value of control signal >= 0
#define U_MAX       (4095)  // max allowable integer value of control signal <= 2^DAC_NBITS - 1

// ------------------------------------------------------------
// Timer2initialize
// ------------------------------------------------------------
// Set up Timer2 to generate interrupt signals at a rate of Fs = 1/T
void Timer2Initialize(void) {
    T2CON = 0x8070; // 80 MHz = 12.5 ns per clock.  Prescale = divide by 256
    // T2 count period = (12.5 ns per clock)/256 = 3.2 us per count
    // Counts needed per desired T2 period = T/T2 - 1
    PR2 = T/0.0000032-1;
} // Timer2Initialize

// ------------------------------------------------------------
// PortsInitialize
// ------------------------------------------------------------
void PortsInitialize(void) {
    //TRIS_LED5 = 0;  // output
} // PortsInitialize

// ------------------------------------------------------------
// InterruptInitialize
// ------------------------------------------------------------
// Initializes Timer2 interrupt
void InterruptInitialize(void) {
    mT2SetIntPriority(1);
    INTEnableSystemMultiVectoredInt();
    mT2IntEnable(1);
} // InterruptInitialize

// ------------------------------------------------------------
// SystemInitialize
// ------------------------------------------------------------
void SystemInitialize(void) {
    PortsInitialize();
    SPI2initialize();
    Timer2Initialize();
    InterruptInitialize();
    initU1(); // Setup UART1
    initADC(AINPUTS);
} // SystemInitialize

// ------------------------------------------------------------
// T2InterruptHandler
// ------------------------------------------------------------
// This program uses the interrupts generated by Timer2 to execute
// the control algorithm
void __ISR(_TIMER_2_VECTOR, ipl1AUTO) T2InterruptHandler(void) {
    // T2 handler code here

    // CALCULATE CURRENT REFERENCE SIGNAL
    
    // Calculate sinusoidal reference signal via DDS
    // get 8-bit sample from wave table and left shift 4 bits to match
    // 12-bit fixed-point format for DAC,
    // then scale to desired high and low amplitudes.
    // This is an integer value (not in units of Volts)
    rk = ((SINETABLE[waveTableIndex] << 4)*(rHiInt - rLoInt))/DAC_MAX_INT + rLoInt;
    
    // Alternative square wave reference signal
    // First half of wave has rLowInt amplitude,
    // second half has amplitude rHighInt.
    // if(phaseAccumulator < 0x8000)
    //    rk = rLowInt;
    // else
    //    rk = rHighInt;
    
    // Alternative constant reference signal at midrange
    // rk = (rLowInt + rHighInt) >> 1;
    
    // READ CURRENT OUTPUT FROM ADC
    // Read current value from 10-bit ADC and scale to be consistent with DAC.
    // We want a 1 Volt signal read from the ADC to create a 1 Volt output from
    // the DAC if we pass the value straight through.
    // The ADC uses a 3.3 V reference into a 10-bit binary representation,
    // so the scale factor is (3.3 V)/(2^10 - 1) = 3.226 mV/bit.
    // The DAC can produce a 4.095 V max output from a 12-bit binary number,
    // so its scale factor is (4.095 V)/(2^12-1) = 1.000 mV/bit.
    // To put the values from the ADC onto the same scale factor as the DAC,
    // multiply the readings by the ratio of the scale factors:
    // yk = valueADC * SFDAC / SFADC = valueADC * 3.226 / 1.000
    yk = readADC(1)*3.226;
    
    // Calculate current control signal u(k)
    // This is where your control algorithm D(z) would go
    uk = rk; // Open-loop (no feedback)
    // Check for control saturation and limit it
    if(uk > U_MAX)
        uk = U_MAX;
    else if(uk < U_MIN)
        uk = U_MIN;

    // Transmit value out the SPI port
    // writeDAC_SPI2(uint16_t val, uint8_t channel, uint8_t gain, uint8_t shutdown)
    writeDAC_SPI2(uk, 0, 0, 1);


    // Remainder of calculations were saved until after the DAC output to
    // minimize the time delay from reading y(k) to writing u(k)
    
    // Advance DDS phase and lookup sample in wave table for next sample
    phaseAccumulator += phaseStep; // increment the phase accumulator
    waveTableIndex = phaseAccumulator >> 8; // use top 8 bits as wavetable index
    
    // Write detected value y(k) to UART (use for testing)
    printf("y(k) = %4d mV\n\r", yk); // transmit yk to UART1
    
    // clear the interrupt flag and exit
    mT2ClearIntFlag();

} // T2InterruptHandler

// ------------------------------------------------------------
// main
// ------------------------------------------------------------
int main(void) {
    SystemInitialize();

    // Send message to UART1 indicating main loop is running
    printf("Main fcn executing!\n\n\r"); // message to UART
    
    // Set up phaseStep for DDS output frequency
    // DDS waveshape table holds 256 values, and phaseAccumulator is 16 bits
    // so output frequency will be Fr = M*Fs/2^16, or M = Fout*2^16/Fs.
    // These need to be calculated just once at the beginning of code execution
    // then is used by the ISR.
    phaseStep = F_R*0x10000*T;
    rHiInt = (R_HI_V*DAC_MAX_INT)/DAC_MAX_V; // Integer value for high reference signal
    rLoInt = (R_LO_V*DAC_MAX_INT)/DAC_MAX_V; // Integer value for low reference signal

    while(TRUE) { // This loop runs except when the timer interrupt occurs
        
        // Empty loop allow ISR to do all processing
        
    } // while

} // main
