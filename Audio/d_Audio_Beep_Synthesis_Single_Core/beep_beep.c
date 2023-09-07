/**
 *  V. Hunter Adams (vha3@cornell.edu)
 
    This is an experiment with the multicore capabilities on the
    RP2040. The program instantiates a timer interrupt on each core.
    Each of these timer interrupts writes to a separate channel
    of the SPI DAC and does DDS of two sine waves of two different
    frequencies. These sine waves are amplitude-modulated to "beeps."

    No spinlock is required to mediate the SPI writes because of the
    SPI buffer on the RP2040. Spinlocks are used in the main program
    running on each core to lock the other out from an incrementing
    global variable. These are "under the hood" of the PT_SEM_SAFE_x
    macros. Two threads ping-pong using these semaphores.

    Note that globals are visible from both cores. Note also that GPIO
    pin mappings performed on core 0 can be utilized from core 1.
    Creation of an alarm pool is required to force a timer interrupt to
    take place on core 1 rather than core 0.

 */

// Include necessary libraries
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 40000            // sample rate

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             200
#define DECAY_TIME              200
#define SUSTAIN_TIME            10000
#define BEEP_DURATION           10400
#define BEEP_REPEAT_INTERVAL    40000

// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int count_0 = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define SPI_PORT spi0

// Two variables to store core number
volatile int corenum_0  ;

// Global counter for spinlock experimenting
volatile int global_counter = 0 ;

// Debouncing state machine variables
#define DB_UNPRESSED 0
#define DB_MAYBE 1
#define DB_PRESSED 2
#define DB_MAYBE_UNPRESSED 3
volatile unsigned int DB_STATE = DB_UNPRESSED;
volatile unsigned int BEEP_FLAG = 0;
static int keycode ;
static int possible ;

// // ----------Keypad------------- //
// // VGA graphics library
// #include "vga_graphics.h"
// #include "pt_cornell_rp2040_v1.h"


// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

#define LED             25

unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
unsigned int button = 0x70 ;


char keytext[40];
int prev_key = 0;

// function PTs
void debouncing_fsm();
static int scan_keypad();


// This timer ISR is called on core 0
bool repeating_timer_callback_core_0(struct repeating_timer *t) {
    // BEEP
    if (BEEP_FLAG) {

        if (STATE_0 == 0) {
            // DDS phase and sine table lookup
            phase_accum_main_0 += phase_incr_main_0  ;
            DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
                sin_table[phase_accum_main_0>>24])) + 2048 ;

            // Ramp up amplitude
            if (count_0 < ATTACK_TIME) {
                current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
            }
            // Ramp down amplitude
            else if (count_0 > BEEP_DURATION - DECAY_TIME) {
                current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
            }

            // Mask with DAC control bits
            DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;
            DAC_data_1 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

            // SPI write (no spinlock b/c of SPI buffer)
            spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;
            spi_write16_blocking(SPI_PORT, &DAC_data_1, 1) ;

            // Increment the counter
            count_0 += 1 ;

            // State transition?
            if (count_0 == BEEP_DURATION) {
                STATE_0 = 1 ;
                count_0 = 0 ;
            }
        }

        // State transition?
        else {
            current_amplitude_0 = 0 ;
            STATE_0 = 0 ;
            count_0 = 0 ;
            BEEP_FLAG = 0;
            
        }

        // retrieve core number of execution
        corenum_0 = get_core_num() ;
    }
    return true;
}


// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
     // Indicate thread beginning
    PT_BEGIN(pt) ;

    while(1) {

        gpio_put(LED, !gpio_get(LED)) ;
        debouncing_fsm();

        // // Write key to VGA
        // if (i != prev_key) {
        //     prev_key = i ;
        //     fillRect(250, 20, 176, 30, RED); // red box
        //     sprintf(keytext, "%d", i) ;
        //     setCursor(250, 20) ;
        //     setTextSize(2) ;
        //     writeString(keytext) ;
        // }


        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

// This function scans the keypad and returns if there is a valid press.
// returns -1 for invalid, and the index of the valid keycode if valid. 
static int scan_keypad(){
    static int i;
    static uint32_t keypad ;

    // Scan the keypad!
    for (i=0; i<KEYROWS; i++) {
        // Set a row high
        gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                        (scancodes[i] << BASE_KEYPAD_PIN)) ;
        // Small delay required
        sleep_us(1) ; 
        // Read the keycode
        keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
        // Break if button(s) are pressed
        if (keypad & button) break ;
    }
    // If we found a button . . .
    if (keypad & button) {
        // Look for a valid keycode.
        for (i=0; i<NUMKEYS; i++) {
            if (keypad == keycodes[i]) break;
        }
        // If we don't find one, report invalid keycode
        if (i==NUMKEYS) (i = -1) ;
    }
    // Otherwise, indicate invalid/non-pressed buttons
    else (i=-1) ;


    return i;

}

// debouncing state machine
// changes state based on keypad code
void debouncing_fsm(){

    keycode = scan_keypad();

    switch(DB_STATE){
        case DB_UNPRESSED:
            if (keycode != -1){
                possible = keycode;
                DB_STATE = DB_MAYBE;
            }
            else {
                DB_STATE = DB_UNPRESSED;
            }
            break;
            

        case DB_PRESSED:
            if (keycode == possible){
                DB_STATE = DB_PRESSED;
            }
            else {
                DB_STATE = DB_MAYBE_UNPRESSED;
            }
            break;
            

        case DB_MAYBE:
            if (keycode == possible){
                // beep
                BEEP_FLAG = 1;
                DB_STATE = DB_PRESSED;
            }
            else {
                DB_STATE = DB_UNPRESSED;
            }
            break;

        case DB_MAYBE_UNPRESSED:
            if (keycode==possible){
                DB_STATE = DB_PRESSED;
            }
            else{
                DB_STATE  = DB_UNPRESSED;
            }
            break;


        // default:
        //     DB_STATE = DB_UNPRESSED;
    }
}


// Core 0 entry point
int main() {
    // Initialize stdio/uart (printf won't work unless you do this!)
    stdio_init_all();

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Create a repeating timer that calls 
    // repeating_timer_callback (defaults core 0)
    struct repeating_timer timer_core_0;

    // Negative delay so means we will call repeating_timer_callback, and call it
    // again 25us (40kHz) later regardless of how long the callback took to execute
    add_repeating_timer_us(-25, 
        repeating_timer_callback_core_0, NULL, &timer_core_0);

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN)) ;
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_down((BASE_KEYPAD_PIN + 4)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 5)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 6)) ;

    // Add core 0 threads
    pt_add_thread(protothread_core_0) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;

}