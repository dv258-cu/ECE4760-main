/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input
 * 
 * This demonstration sets a PWM duty cycle to a
 * user-specified value.
 * 
 * HARDWARE CONNECTIONS
 *   - GPIO 4 ---> PWM output
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pwm.h"
#include "hardware/irq.h"

#include "pt_cornell_rp2040_v1_4.h"

#include <stdint.h>
#include "hardware/adc.h"


// PWM wrap value and clock divide value
// For a CPU rate of 125 MHz, this gives
// a PWM frequency of 1 kHz.
#define WRAPVAL 6000
#define CLKDIV 25.0f

// GPIO we're using for PWM
#define PWM_OUT 4

//Define for potentiometer
#define POTENTIOMETER_OUTPUT_PIN 27

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
//#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
#define divfix(a,b) ((fix15)((((int32_t)(a)) << 15) / (b)))
volatile int pwm_read = 0;

// Variable to hold PWM slice number
uint slice_num ;

// PWM duty cycle
volatile int control ;
volatile int old_control ;

// PWM interrupt service routine
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));
    // Update duty cycle
    if (control!=old_control) {
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    }
}

    void change_PWM() {
        //0 to 4096
        //pwm_read = fix2int15(divfix(int2fix15(adc_read()), int2fix15(4096)));
        pwm_read = adc_read();
        printf("%d\n" , pwm_read);
        control = pwm_read;
    }

// User input thread
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static int test_in ;
    while(1) {
        //sprintf(pt_serial_out_buffer, "input a duty cycle (0-6000): ");
        //serial_write ;
        // spawn a thread to do the non-blocking serial read
        //serial_read ;
        // convert input string to number
        //sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        //if (test_in > 6000) continue ;
        //else if (test_in < 0) continue ;
        //else control = test_in ;
        change_PWM();


    }
    PT_END(pt) ;
}

int main() {

    // Initialize stdio
    stdio_init_all();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO PWM_OUT that it is allocated to the PWM
    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO PWM_OUT (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;


    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 1000);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    //OUR CODE
    adc_init();
    adc_gpio_init(POTENTIOMETER_OUTPUT_PIN);
    adc_select_input(1);
    
    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
