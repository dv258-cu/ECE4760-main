/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * Keypad Demo
 * 
 * KEYPAD CONNECTIONS
 *  - GPIO 9   -->  330 ohms  --> Pin 1 (button row 1)
 *  - GPIO 10  -->  330 ohms  --> Pin 2 (button row 2)
 *  - GPIO 11  -->  330 ohms  --> Pin 3 (button row 3)
 *  - GPIO 12  -->  330 ohms  --> Pin 4 (button row 4)
 *  - GPIO 13  -->     Pin 5 (button col 1)
 *  - GPIO 14  -->     Pin 6 (button col 2)
 *  - GPIO 15  -->     Pin 7 (button col 3)
 * 
 * VGA CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green 
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 * 
 * SERIAL CONNECTIONS
 *  - GPIO 0        -->     UART RX (white)
 *  - GPIO 1        -->     UART TX (green)
 *  - RP2040 GND    -->     UART GND
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"

// VGA graphics library`
#include "vga16_graphics_v2.h"
#include "pt_cornell_rp2040_v1_4.h"

//code imported from beep_beep.c
// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

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
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)


//parabola constant for swoop sound frequency approximation
volatile double parabola_const = (double)260 / (double)(3250 * 3250);


// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.

volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_accum_swoop = 0;
volatile unsigned int phase_accum_chirp = 0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variable
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             250
#define DECAY_TIME              250
#define SUSTAIN_TIME            10000
#define BEEP_DURATION           10500
#define BEEP_REPEAT_INTERVAL    50000

//Timing parameters for swoop 
#define SWOOP_DURATION 6500

//Timing parameters for chirp
#define CHIRP_DURATION 6500

//Timing parameters for pause
#define PAUSE_DURATION 6500



// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int count_0 = 0 ;

// Sound state machine
volatile enum {
    BEEP,
    SWOOP,
    CHIRP,
    SILENCE,
    PAUSE,
}   SOUND = SILENCE;

// MODE state machine
volatile enum {
    PLAY,
    RECORD,
}   MODE = PLAY;


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

//GPIO for timing the ISR
#define ISR_GPIO 3


// This timer ISR is called on core 0
static void alarm_irq(void) {

    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;


    // Beep Sound
    if (SOUND == BEEP) {
        // DDS phase and sine table lookup
        phase_accum_main_0 += phase_incr_main_0;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0, sin_table[phase_accum_main_0 >> 24])) + 2048;


        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc);
        }
        // Ramp down amplitude
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc);
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff));

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);

        // Increment the counter
        count_0 += 1;

        // State transition?
        if (count_0 == BEEP_DURATION) {
            SOUND = SILENCE;
            count_0 = 0;
        }
    }

    // Swoop Sound
    else if (SOUND == SWOOP) {

        //Parabolic fit for swoop sound 
        volatile double frequency = 
                2000 - parabola_const * ((count_0 - 3250) * (count_0 -3250));
        
        phase_accum_swoop += (unsigned int) (frequency * (85899.34592));

        //Sin table lookup
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0, sin_table[phase_accum_swoop >> 24])) + 2048;


        // Ramp up amplitude
        // Attack phase - prevent a sudden jump in amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc);
        }

        // Ramp down amplitude
        // Decay phase - allow amplitude to decrease smoothly
        else if (count_0 > SWOOP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc);
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff));

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);

        // Increment the counter
        count_0 += 1;

        // State transition?
        if (count_0 == SWOOP_DURATION) {
            SOUND  = SILENCE;
            count_0 = 0;
        }
    } 
    
    // Chirp Sound
    else if (SOUND == CHIRP) {
        
        // DDS phase and sine table lookup
        volatile unsigned int frequency  = 
                two32 * ((1.6 * 0.0001) * (count_0 * count_0) + 2000) / Fs;

        phase_accum_chirp += frequency;

        //Sin table lookup
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0, sin_table[phase_accum_chirp >> 24])) + 2048;

        //Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc);
        }
        //Ramp down amplitude
        else if (count_0 > CHIRP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc);
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff));

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);

        // Increment the counter
        count_0 += 1;

        // State transition?
        if (count_0 == CHIRP_DURATION) {
            SOUND = SILENCE;
            count_0 = 0;
        }
    }

    // Pause Sound
    else if (SOUND == PAUSE) {
        count_0 += 1;
        
        // State transition?
        if (count_0 == PAUSE_DURATION) {
            SOUND = SILENCE;
            count_0 = 0;
        }
    }

    else {

        // SILENCE
        count_0 += 1;

    }

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);

}

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


// RECORDING CONFIGS
#define RECORD_KEYPAD_PIN 10      // * button
#define RECORD_MAX_LIMIT 50

// Playback Buffer
volatile int keys_to_playback[RECORD_MAX_LIMIT];
volatile int keys_to_playback_index = 0;

// Playback Semaphore
volatile struct pt_sem thread_playback;

// Playback Thread
static PT_THREAD (protothread_playback(struct pt *pt)){
    // Indicate thread beginning
    PT_BEGIN(pt) ;
    static int i;

    while (1) {
        // Playback thread whill not process until semaphore is given
        PT_SEM_WAIT(pt , &thread_playback);

        // Play the keys in order
        while (i < keys_to_playback_index) {
            if (keys_to_playback[i] == 1) {
                SOUND = SWOOP;

                while (SOUND != SILENCE) {      //to ensure sound is finished before next sound
                    continue;
                }
            }

            else if (keys_to_playback[i] == 2) {
                SOUND = CHIRP;

                while (SOUND != SILENCE) {
                    continue;
                }
            }

            // any other key is a pause 
            else {
                SOUND = PAUSE;
                while (SOUND != SILENCE) {
                    continue;
                }
            }
            // Zero out the current index for good measure
            keys_to_playback[i] = 0;
            i++;
        }

        // Reset buffer index and return to PLAY mode once playback is complete
        i = 0;
        keys_to_playback_index = 0;
        MODE = PLAY;
        printf(">>> Playback finished, returning to PLAY mode\n");
    }

    PT_END(pt) ;
}






// =============================================================
// Protothread running on core 0
// Scans the keypad, debounces button presses,
// and triggers sounds (swoop/chirp) when certain keys are pressed.
// Also displays the last key on VGA and prints it over UART.
// =============================================================
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    static int i ;
    static uint32_t keypad ;

    // Debouce state machine 
    static enum {
        NOT_PRESSED,
        MAYBE_PRESSED,
        PRESSED,
        MAYBE_NOT_PRESSED,
    } debounce_state  = NOT_PRESSED;

    // Possible starts as -1 because initial state is not pressed
    static int possible = -1;

    while(1) {

        // Toggle the onboard LED every loop to show thread is alive
        gpio_put(LED, !gpio_get(LED)) ;

        // =============================================================
        // STEP 1: Scan the keypad
        // - Drive one row HIGH at a time
        // - Read columns to see which key (if any) closes a switch
        // =============================================================
        for (i=0; i<KEYROWS; i++) {
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN)) ;
            
            // Small delay required
            sleep_us(2) ; 
            
            // Read all pins to see which key (if any) closes a switch and mask out the 7 keypad bits 
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
            
            // if any column line is HIGH, a button is pressed
            if (keypad & button) break ;
        }
        
        // =============================================================
        // STEP 2: Decode which button was pressed
        // =============================================================
        if (keypad & button) {
            
            // compare keycode against all valid keycodes
            for (i=0; i < NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ;
            }
            // If no match, mark invalid 
            if (i==NUMKEYS) 
            (i = -1) ;
        }

        // Otherwise, no key pressed
        else (i =-1) ;

        // =============================================================
        // STEP 3: Debounce finite-state machine
        // Ensures one clean event per physical button press
        // =============================================================
        switch (debounce_state) {

            case NOT_PRESSED:
                if (i != -1) {                         // saw a press 
                    debounce_state = MAYBE_PRESSED;
                    possible = i;                      // candidate key
                }
                break;

            case MAYBE_PRESSED:
                if (i == possible) {                   // if the candidate key is pressed
                    debounce_state = PRESSED;
                    
                    current_amplitude_0 = 0;          // reset amplitude and count
                    count_0 = 0;

                    // CHECK MODE
                    if (MODE == PLAY) {
                        // State transition for swoop and chirp and * for enter record mode
                        if (i == 1){                       // key 1 is swoop
                            SOUND = SWOOP;
                        }
                        else if (i == 2){                  // key 2 is chirp
                            SOUND = CHIRP;
                        }
                        else if (i == 10){                  // * is record key
                            SOUND = SILENCE;
                            MODE = RECORD;
                            printf(">>>Entering RECORD mode\n");
                        }
                    }
                    // We are in RECORD mode and if we press record key again we stop recording and play back else we continue recording
                    else if (MODE == RECORD) { 
                        if (i == 10){
                            PT_SEM_SIGNAL(pt , &thread_playback);
                            printf(">>> Stopping recording, starting playback\n");
                        }
                        else {
                            keys_to_playback[keys_to_playback_index] = i;
                            keys_to_playback_index++;
                            printf("Recorded key: %d (index %d)\n", i, keys_to_playback_index);
                        }
                    }
                } 

                else {
                    // Glitch / wrong button we reject the press 
                    debounce_state = NOT_PRESSED;

                }
                break;

            case PRESSED:
                printf("Pressed index=%d\n", i);

                if (i != possible) {                  // saw release
                    debounce_state = MAYBE_NOT_PRESSED;
                }
                break;

            case MAYBE_NOT_PRESSED:
                if (i == possible) {                   // still pressed
                    debounce_state = PRESSED;
                } else {
                    // confirmed release
                    debounce_state = NOT_PRESSED;
                }
                break;
            default:
                break;
        }

        // Print key to terminal
        // printf("\n%d", i) ;

        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}



int main() {
    ////////////////// CLOCK, I/O AND VGA UI ///////////////////////
    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize the VGA screen
    initVGA() ;


    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;
    
    
    ////////////////// ISR INITS ///////////////////////
    // Initialize stdio/uart (printf won't work unless you do this!)
    stdio_init_all();
    printf("Hello, friends!\n");

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

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO) ;
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    ////////////////// DDS INITS ///////////////////////
    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM) ;
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq) ;
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true) ;
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    
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

    // Start in play mode
    MODE = PLAY;
    // Set semaphore to zero so that playback thread is not running
    PT_SEM_INIT(&thread_playback, 0);

    // Add core 0 threads
    pt_add_thread(protothread_core_0) ;
    pt_add_thread(protothread_playback) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;

}