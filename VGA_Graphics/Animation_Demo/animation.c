
/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
  - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga16_graphics_v2.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/spi.h"
#include "hardware/adc.h"


// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

#define topHistogramTop(b) (b>int2fix15(HISTOGRAM_Y_START))

// uS per frame
#define FRAME_RATE 33000
#define POTENTIOMETER_OUTPUT_PIN 27


/*
  DMA CODE Imported from dma_code.c provided by Hunter Adams
*/

#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size] ;

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size] ;

// Pointer to the address of the DAC data table
unsigned short * dma_address_pointer = &DAC_data[0] ;

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

//SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size ;

int data_chan;
int ctrl_chan;


// int data_chan, ctrl_chan;
void DMA_setup() {

    // Initialize stdio
    stdio_init_all();

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Build sine table and DAC data table
    int i ;
    for (i=0; i<(sine_table_size); i++){
        raw_sin[i] = (int)(2047 * sin((float)i*6.283/(float)sine_table_size) + 2047); //12 bit
        DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff) ;
    }

    // Select DMA channels
    data_chan = dma_claim_unused_channel(true);;
    ctrl_chan = dma_claim_unused_channel(true);;

    // Setup the control channel 
    dma_channel_config c = dma_channel_get_default_config(ctrl_chan);   // default configs
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);             // 32-bit txfers
    channel_config_set_read_increment(&c, false);                       // no read incrementing
    channel_config_set_write_increment(&c, false);                      // no write incrementing
    channel_config_set_chain_to(&c, data_chan);                         // chain to data channel

    dma_channel_configure(
        ctrl_chan,                          // Channel to be configured
        &c,                                 // The configuration we just created
        &dma_hw->ch[data_chan].read_addr,   // Write address (data channel read address)
        &dma_address_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers
        false                               // Don't start immediately
    );

    // Setup the data channel
    dma_channel_config c2 = dma_channel_get_default_config(data_chan);  // Default configs
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);            // 16-bit txfers
    channel_config_set_read_increment(&c2, true);                       // yes read incrementing
    channel_config_set_write_increment(&c2, false);                     // no write incrementing
    // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
    // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
    dma_timer_set_fraction(0, 0x0017, 0xffff) ;
    // 0x3b means timer0 (see SDK manual)
    channel_config_set_dreq(&c2, 0x3b);                                 // DREQ paced by timer 0
    // chain to the controller DMA channel
    // channel_config_set_chain_to(&c2, ctrl_chan);                        // Chain to control channel


    dma_channel_configure(
        data_chan,                  // Channel to be configured
        &c2,                        // The configuration we just created
        &spi_get_hw(SPI_PORT)->dr,  // write address (SPI data register)
        DAC_data,                   // The initial read address
        sine_table_size,            // Number of transfers
        false                       // Don't start immediately.
    );


}


#define BALL_RADIUS 6
#define PEG_RADIUS  8
#define GRAVITY float2fix15(0.75)
#define BOUNCINESS float2fix15(0.5)

#define X_CENTER int2fix15(320)



fix15 fixmag(fix15 x, fix15 y) {

  // Some constants
  fix15 sqrt_alpha = float2fix15(.9604);
  fix15 sqrt_beta = float2fix15(.3978);

  fix15 r;
  if (absfix15(x) <= absfix15(y)) {
    r = multfix15(absfix15(y), sqrt_alpha) + multfix15(absfix15(x), sqrt_beta);
  } else {
     r  = multfix15(absfix15(x), sqrt_alpha) + multfix15(absfix15(y), sqrt_beta);
  }

  return r;
}

// Ball Struct 
typedef struct {
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
} Ball;

typedef struct {
  fix15 x;
  fix15 y;
} Peg;

// ball starts at the top(centered) and falls down with a small random x velocity
// void spawn_ball(Ball* b) {
//   b->x = int2fix15(320);
//   b->y = int2fix15(0);
//   fix15 rand_vx = ((int2fix15(rand() & 0xffff)>>16) - (int2fix15(1)>>1));
//   b->vx = rand_vx;
//   b->vy = int2fix15(0);
// }


// Set to true to activate debug mode
bool debug_flag = false;

Peg peg;

// Sets inital peg parameters
void init_peg() {
  peg.x = X_CENTER;
  peg.y = int2fix15(75);
  
  fillCircle(fix2int15(peg.x), fix2int15(peg.y), PEG_RADIUS, GREEN);
}

void update_peg() {
  fillCircle(fix2int15(peg.x), fix2int15(peg.y), PEG_RADIUS, GREEN);
}

void animate_peg() {
  update_peg();
}


Ball ball;

void spawn_ball() {
  fillCircle(fix2int15(ball.x), fix2int15(ball.y), BALL_RADIUS, BLACK);

  fix15 rand_vx = ((int2fix15(rand() & 0xffff)>>16) - (int2fix15(1)>>1));

  ball.x = X_CENTER;
  ball.y = int2fix15(0);
  ball.vx = rand_vx;
  ball.vy = int2fix15(0);

  fillCircle(fix2int15(ball.x), fix2int15(ball.y), BALL_RADIUS, LIGHT_BLUE);
}

void clear_ball() { fillCircle(fix2int15(ball.x), fix2int15(ball.y), BALL_RADIUS, BLACK); }
void draw_ball() { fillCircle(fix2int15(ball.x), fix2int15(ball.y), BALL_RADIUS, LIGHT_BLUE); }

void update_ball() {
  // Update ball position with velocity
  ball.x = ball.x + ball.vx;
  ball.y = ball.y + ball.vy;

  // Include gravity
  ball.vy = ball.vy + GRAVITY;
}

void animate_ball() {
  // Erase previous ball first
  clear_ball();

  // Update ball's physical parameters
  update_ball();

  // Draw new ball with updated information
  draw_ball();

  if (ball.y >= int2fix15(480)) {
    spawn_ball();
  }
}

// Collision variables
fix15 next_ball_x() { return ball.x + ball.vx; }
fix15 next_ball_y() { return ball.y + ball.vy; }

fix15 dx;
fix15 dy;
fix15 dr;

// Do this on collision
void on_collision() {
  clear_ball();

  fix15 normal_x = divfix(dx, dr);
  fix15 normal_y = divfix(dy, dr);

  fix15 intermediate_term = multfix15(int2fix15(-2), (multfix15(normal_x, ball.vx) + multfix15(normal_y, ball.vy)));
  if (intermediate_term > int2fix15(0)) {
    ball.x = peg.x + multfix15(normal_x, dr + int2fix15(1));
    ball.y = peg.y + multfix15(normal_y, dr + int2fix15(1));

    ball.vx = ball.vx + multfix15(normal_x, intermediate_term);
    ball.vy = ball.vy + multfix15(normal_y, intermediate_term);
  }
  update_peg();
}

// Checks if the ball collided every frame
bool check_collision() {

  // 1D deltas between center of ball and center of peg
  fix15 next_dx = next_ball_x() - peg.x;
  fix15 next_dy = next_ball_y() - peg.y;
  fix15 next_dr = fixmag(next_dx, next_dy);
  dx = ball.x - peg.x;
  dy = ball.y - peg.y;
  dr = fixmag(dx, dy);
  
  fix15 avg_dx = divfix(next_dx + dx, int2fix15(2));
  fix15 avg_dy = divfix(next_dy + dy, int2fix15(2));
  fix15 avg_dr = divfix(next_dr + dr, int2fix15(2));


 
  if (absfix15(avg_dr) <= int2fix15(PEG_RADIUS + BALL_RADIUS)) {
    return true;
  }
  return false;
}


void init_collision_debugger() { drawCircle(320, 100, 17, RED); }
void update_collision_debugger() { drawCircle(320, 100, 17, RED); }

// Called once at program start
void start() {
  spawn_ball();
  init_peg();

  if (debug_flag) { init_collision_debugger(); }
}


// Called every frame
void update_frame() {
  if (debug_flag) { update_collision_debugger(); }

  animate_ball(); 

  if (check_collision()) { on_collision(); }
}


// Write to screen thread -  we could use this for vga screen setup
/*
  The VGA should display:
    - The current number of balls being animated
    - The total number of balls that have fallen through the board since reset 
    - Time since boot
*/
static PT_THREAD (protothread_write_to_screen(struct pt *pt)){
    // Mark beginning of thread
    PT_BEGIN(pt);

    // start();

    while(1) {
      PT_YIELD_usec(1000000);

      
    } // END WHILE(1)
  PT_END(pt);
}

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time;
    static int spare_time;

    start();

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;   

      update_frame();

      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread




// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  // Add animation thread
  // pt_add_thread(protothread_anim);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  set_sys_clock_khz(150000, true) ;
  // initialize stio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;
  DMA_setup(); // setup DMA for audio

  //initialie adc
  adc_init();
  adc_gpio_init(POTENTIOMETER_OUTPUT_PIN);
  adc_select_input(1);

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_write_to_screen);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 

