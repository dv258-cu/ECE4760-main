
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
#define VERTICAL_SEPARATION 19
#define HORIZONTAL_SEPARATION 38
#define NUMBER_OF_PEGS 136   // number of pegs = n(n+1)/2 = 16 (16 + 1) / 2 = 16 * 17 / 2 = 136 pegs



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

Peg pegs[136];

// Sets inital peg parameters
void init_pegs() {
  const int screen_center_x = 320;         // middle of screen( 640 / 2 )
  const int start_y = 60;                  // start y position for the first peg

  int row = 0;
  int peg_index = 0;

  while (peg_index < NUMBER_OF_PEGS){
    int pegs_in_row = row + 1;              // peg in this row so row 0 has 1 peg, row 1 has 2 pegs, row 2 has 3 and so on
    int start_x = screen_center_x - ((pegs_in_row - 1) * HORIZONTAL_SEPARATION) / 2;  // center each row by shifting first peg to the left so 1st row is at 320, 
                                                                                      // 2nd row is at 301 and 339 , 3rd row is at 282, 320 and 358 etc

    // inner loop to draw each peg in the row
    for (int col = 0; col < pegs_in_row; col ++){
      pegs[peg_index].x = int2fix15(start_x + col * HORIZONTAL_SEPARATION);  // space the pegs horizontally starting from start_x and incrementing by HORIZONTAL_SEPARATION
      pegs[peg_index].y = int2fix15(start_y + row * VERTICAL_SEPARATION);    // space the pegs vertically starting from start_y and incrementing by VERTICAL_SEPARATION

      fillCircle(fix2int15(pegs[peg_index].x), fix2int15(pegs[peg_index].y), PEG_RADIUS, GREEN);
      
      peg_index++;
    }
    row++;
  }
}

void update_peg(Peg peg) {
  fillCircle(fix2int15(peg.x), fix2int15(peg.y), PEG_RADIUS, GREEN);
}


Ball balls[4096];
void init_balls() { 
  for (int i = 0; i < 4096; i++) {
    fix15 rand_vx = ((int2fix15(rand() & 0xffff)>>16) - (int2fix15(1)>>1));

    balls[i].x = X_CENTER;
    balls[i].y = int2fix15(0);
    balls[i].vx = rand_vx;
    balls[i].vy = int2fix15(0);

    fillCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), BALL_RADIUS, LIGHT_BLUE);
  }
}

void spawn_ball(Ball ball) {
  clear_ball(ball);

  fix15 rand_vx = ((int2fix15(rand() & 0xffff)>>16) - (int2fix15(1)>>1));

  ball.x = X_CENTER;
  ball.y = int2fix15(0);
  ball.vx = rand_vx;
  ball.vy = int2fix15(0);

  draw_ball(ball);
}

void clear_ball(Ball ball) { fillCircle(fix2int15(ball.x), fix2int15(ball.y), BALL_RADIUS, BLACK); }
void draw_ball(Ball ball) { fillCircle(fix2int15(ball.x), fix2int15(ball.y), BALL_RADIUS, LIGHT_BLUE); }

void update_ball(Ball ball) {
  // Update ball position with velocity
  ball.x = ball.x + ball.vx;
  ball.y = ball.y + ball.vy;

  // Include gravity
  ball.vy = ball.vy + GRAVITY;
}

void animate_ball(Ball ball) {
  // Erase previous ball first
  clear_ball(ball);

  // Update ball's physical parameters
  update_ball(ball);

  // Draw new ball with updated information
  draw_ball(ball);

  if (ball.y >= int2fix15(480)) {
    spawn_ball(ball);
  }
}



// Do this on collision
void on_collision(Ball ball, Peg peg) {
  clear_ball(ball);

  // 1D deltas between center of ball and center of peg
  fix15 dx = ball.x - peg.x;
  fix15 dy = ball.y - peg.y;
  fix15 dr = fixmag(dx, dy);

  fix15 normal_x = divfix(dx, dr);
  fix15 normal_y = divfix(dy, dr);

  fix15 intermediate_term = multfix15(int2fix15(-2), (multfix15(normal_x, ball.vx) + multfix15(normal_y, ball.vy)));
  if (intermediate_term > int2fix15(0)) {
    ball.x = peg.x + multfix15(normal_x, dr + int2fix15(1));
    ball.y = peg.y + multfix15(normal_y, dr + int2fix15(1));

    ball.vx = ball.vx + multfix15(normal_x, intermediate_term);
    ball.vy = ball.vy + multfix15(normal_y, intermediate_term);
  }

  update_peg(peg);
}

// Checks if the ball collided with peg
bool check_collision(Ball ball, Peg peg) {

  // 1D deltas between center of ball and center of peg
  fix15 dx = ball.x - peg.x;
  fix15 dy = ball.y - peg.y;
  fix15 dr = fixmag(dx, dy);
 
  if (absfix15(dx) <= int2fix15(PEG_RADIUS + BALL_RADIUS) || absfix15(dy) <= int2fix15(PEG_RADIUS + BALL_RADIUS)) {
    return true;
  }
  return false;
}

void init_collision_debugger() { drawCircle(320, 100, 17, RED); }
void update_collision_debugger() { drawCircle(320, 100, 17, RED); }

// Called once at program start
void start() {
  init_balls();
  init_pegs();

  if (debug_flag) { init_collision_debugger(); }
}


// Called every frame
void update_frame() {

  int numballs = adc_read();

  if (debug_flag) { update_collision_debugger(); }
  for (int i = 0; i < NUMBER_OF_PEGS; i++) {
    for (int j = 0; j < numballs; j++) {
      if (check_collision(balls[j], pegs[i])) { on_collision(balls[j], pegs[i]); }
    }
  }

  for (int i = 0; i < numballs; i++) {
    animate_ball(balls[i]);
  }
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

