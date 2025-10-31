/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
// Include custom libraries
#include "vga16_graphics_v2.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// character array
char screentext[40];

// draw speed
int threshold = 10 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;
// static struct pt_sem runtime_semaphore;

// Some paramters for PWM
#define WRAPVAL 6000
#define CLKDIV  30.0f
uint slice_num ;

// Floats for raw IMU data
float rotation_rate;
float gs;
float net_angle;

fix15 accel_angle;
fix15 gyro_angle_delta;
fix15 complementary_angle;
fix15 f_accel_y;
fix15 f_accel_z;

// PID Coefficients
float kp = 10.0;
float ki = 1/32.0;
float kd = 1.0;
float target_angle = 0.0;
float prev_error = 0.0;

// PWM duty cycle
#define PWM_OUT 4
volatile int control ;
volatile int old_control ;

#define PI 3.141592653589

int pid() {
    float error = target_angle - net_angle;
    float d_error = error - prev_error;

    float i_error = i_error + error;
    

    float p_term = (int)(kp*error);
    float i_term = (int)(ki * (i_error / 0.001));
    float d_term = (int)(kd * (d_error / 0.001));

    prev_error = error;
    float pid_net = p_term + i_term + d_term;
    
    if (error < 0) { return 0; }
    if (pid_net >= 3000) { return 2500; }
    else { return pid_net; }
}

// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // SMALL ANGLE APPROXIMATION
    // accel_angle = -multfix15(divfix(acceleration[1], acceleration[2]), oneeightyoverpi) ;

    // filtered = filtered + (raw - filtered) >> precision
    f_accel_y = f_accel_y + ((acceleration[1] - f_accel_y) >> 4);
    f_accel_z = f_accel_z + ((acceleration[2] - f_accel_z) >> 4);

    accel_angle = multfix15(float2fix15(atan2(fix2float15(-f_accel_y), fix2float15(f_accel_z))), oneeightyoverpi);

    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    gyro_angle_delta = multfix15(gyro[0], zeropt001) ;

    // Complementary angle (degrees - 15.16 fixed point)
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);
    
    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);

    // Signal Runtime
    // PT_SEM_SIGNAL(pt, &runtime_semaphore);

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));
    // Update duty cycle
    if (control!=old_control) {
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    }

    // PID Loop Cal
    control = pid();
    
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 500. ; // (+/- 250)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = -250. ;
    static float OldMax = 250. ;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "+90") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "-90") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+250") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "-250") ;
    setCursor(45, 225) ;
    writeString(screentext) ;
    

    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK) ;

            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[0])*120.0)-OldMin)/OldRange)), WHITE) ;
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[1])*120.0)-OldMin)/OldRange)), RED) ;
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[2])*120.0)-OldMin)/OldRange)), GREEN) ;

            net_angle = fix2float15(complementary_angle - int2fix15(7));    // Angle value in degrees relative to horizontal out of the table edge (+Y)
            float graph_angle = net_angle * 0.83333 - 5.0;                  // Not used for computations -- Scaled value for accurate graph depiction
            
            // Draw Angle
            drawPixel(xcoord, 350 - (int)(graph_angle), WHITE);

            // Draw top plot
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[0]))-OldMin)/OldRange)), WHITE) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[1]))-OldMin)/OldRange)), RED) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[2]))-OldMin)/OldRange)), GREEN) ;

            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    
    static char classifier ;
    static int test_in ;
    static float float_in ;
    while(1) {
        // PT_SEM_WAIT(pt, &runtime_semaphore);

        sprintf(pt_serial_out_buffer, "input a command: ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%c", &classifier) ;

        // num_independents = test_in ;
        if (classifier=='p') {
            sprintf(pt_serial_out_buffer, "Proportional Coefficient: ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%f", &float_in) ;
            if (float_in > 0) {
                kp = float_in;
            }
        }
        else if (classifier=='a') {
            sprintf(pt_serial_out_buffer, "Target Angle: ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%f", &float_in) ;
            if (float_in > 0) {
                target_angle = float_in;
            }
        }
        else if (classifier=='i') {
            sprintf(pt_serial_out_buffer, "Integral Coefficient: ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%f", &float_in) ;
            if (float_in > 0) {
                ki = float_in;
            }
        }
        else if (classifier=='d') {
            sprintf(pt_serial_out_buffer, "Derivative Coefficient: ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%f", &float_in) ;
            if (float_in > 0) {
                kd = float_in;
            }
        }
    }

    PT_END(pt) ;
}

void init_accel_filter() {
    f_accel_y = int2fix15(0);
    f_accel_z = int2fix15(0);
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    init_accel_filter();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // Pullup resistors on breakout board, don't need to turn on internals
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

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
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
