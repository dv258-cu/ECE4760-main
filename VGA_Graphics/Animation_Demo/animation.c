
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

// Wall detection
#define hitBottom(b) (b>int2fix15(480))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))
#define topHistogramTop(b) (b>int2fix15(HISTOGRAM_Y_START))

// uS per frame
#define FRAME_RATE 33000
#define POTENTIOMETER_OUTPUT_PIN 27


// the color of the boid
char color = WHITE;


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


#define BALL_RADIUS 4
#define PEG_RADIUS  6
#define GRAVITY float2fix15(0.5)
#define BOUNCINESS float2fix15(0.5)
#define VERTICAL_SEPARATION 19
#define HORIZONTAL_SEPARATION 38
#define NUMBER_OF_PEGS 136   // number of pegs = n(n+1)/2 = 16 (16 + 1) / 2 = 16 * 17 / 2 = 136 pegs 


//global variable for number of balls
uint16_t result;

// Histogram Specs
#define NUM_BINS 17                               // total no of bins 17 since we have 15 spaces btwn the 16 pegs plus two more for the edges
int balls_in_bins[NUM_BINS];                      //counter for each bin

/*
  Math for space left for histogram
  first peg starts at y = 60 and vertival separation = 19 so bottom row y is  = start_y + (16 -1 ) * vertical_separation 
  = 60 + 15 * 19 = 60 + 285 = 345: 
  but each peg is circular with radi of 6 so end of last peg is 345 + 6 = 351 
*/

#define HISTOGRAM_Y_START 360 
#define HISTOGRAM_HEIGHT  100 

// Ball Struct 
typedef struct {
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
  int last_peg;            // last peg that the ball hit 
} Ball;

typedef struct {
  fix15 x;
  fix15 y;
} Peg;

// Global one ball + one peg 
Ball balls[4096];

//initialize the ball member values
void initBalls(){
  for (int i  = 0; i < 4096; i++){
    balls[i].x = int2fix15(320);
    balls[i].y = int2fix15(0);
    fix15 rand_vx = ((int2fix15(rand() & 0xffff)>>16) - (int2fix15(1)>>1));
    balls[i].vx = rand_vx;
    balls[i].vy = int2fix15(0);
    balls[i].last_peg = -1;                        //here I reset the last peg to -1 when we spawn a new ball
  }
}



Peg pegs[NUMBER_OF_PEGS];        // global array that stores the position of all 136 pegs 

int total_balls = 0;              // total number of balls that have fallen through the board

// ball starts at the top(centered) and falls down with a small random x velocity
void spawn_ball(Ball* b) {
  b->x = int2fix15(320);
  b->y = int2fix15(0);
  fix15 rand_vx = ((int2fix15(rand() & 0xffff)>>16) - (int2fix15(1)>>1));
  b->vx = rand_vx;
  b->vy = int2fix15(0);
  b->last_peg = -1;                        //here I reset the last peg to -1 when we spawn a new ball
}


// function to draw pegs on screen: 16 rows 
void draw_pegs(){
  const int screen_center_x = 320;         // middle of screen( 640 / 2 )
  const int start_y = 60;                  // start y position for the first peg

  int row = 0;
  int peg_index = 0;

  while (peg_index < NUMBER_OF_PEGS){
    int pegs_in_row = row + 1;              // peg in this row so row 0 has 1 peg, row 1 has 2 pegs, row 2 has 3 and so on
    int start_x = screen_center_x - ((pegs_in_row - 1) * HORIZONTAL_SEPARATION) / 2;  // center each row by shifting first peg to the left so 1st row is at 320, 
                                                                                      // 2nd row is at 301 and 339 , 3rd row is a7 282, 320 and 358 etc

    // inner loop to draw each peg in the row
    for (int col = 0; col < pegs_in_row; col ++){
      pegs[peg_index].x = int2fix15(start_x + col * HORIZONTAL_SEPARATION);  // space the pegs horizontally starting from start_x and incrementing by HORIZONTAL_SEPARATION
      pegs[peg_index].y = int2fix15(start_y + row * VERTICAL_SEPARATION);    // space the pegs vertically starting from start_y and incrementing by VERTICAL_SEPARATION

      fillCircle(fix2int15(pegs[peg_index].x), fix2int15(pegs[peg_index].y), PEG_RADIUS, GREEN);
      //drawCircle(fix2int15(pegs[peg_index].x), fix2int15(pegs[peg_index].y), PEG_RADIUS, GREEN);


      peg_index++;

    }

    row++;
  }

}

// Histogram
void draw_histogram(){
  //find largest value for scaling every bar height relative to the largest bin
  int max_count = 1;

  for (int i= 0; i < NUM_BINS; i++){
    if (balls_in_bins[i] > max_count){
      max_count = balls_in_bins[i];
    }
  }
    
  int bin_width = HORIZONTAL_SEPARATION;                     //same spacingg as pegs
  int histogram_width = NUM_BINS * bin_width;                //total width of histogram

  int start_bin_x = 322 - histogram_width / 2;                       //start x position of histogram

  // draw the histogram
  for (int i = 0; i < NUM_BINS; i++){
    int bin_height = balls_in_bins[i] * HISTOGRAM_HEIGHT / max_count;   //scale the height of each bar

    // // clear old column

    // draw new column (bars grow upward)
    fillRect(start_bin_x + i * bin_width,                           // top left x position
            HISTOGRAM_Y_START + HISTOGRAM_HEIGHT - bin_height,     // top left y position
            bin_width - 2,                                          // width
            bin_height,                                             // height   
            BLUE);                                                  // color      

  }
  
}

fix15 sqrt_alpha = float2fix15(.9604);
fix15 sqrt_beta = float2fix15(.3978);

// update balls based on collision
void update_ball_based_on_collision() {

  // Each frame we update every ball
  for (int i  = 0; i < result; i++){

    // Every ball we loook at every peg
    for (int j = 0; j < NUMBER_OF_PEGS; j++){

      // Compute x and y distance between ball and peg  
      fix15 dx = balls[i].x - pegs[j].x;
      fix15 dy = balls[i].y - pegs[j].y;

      fix15 abs_dx = absfix15(dx);
      fix15 abs_dy = absfix15(dy);


      // distance approximation using alpha and beta: (alpha * max(dx , dy) + beta * min(dx, dy))
      fix15 distance ;
      if (abs_dx <= abs_dy) {
        distance = multfix15(abs_dy , sqrt_alpha) + multfix15(abs_dx , sqrt_beta);
      } else {
        distance  = multfix15(abs_dx , sqrt_alpha) + multfix15(abs_dy , sqrt_beta);
      }
  

      if (distance <= int2fix15(BALL_RADIUS + PEG_RADIUS)) {
            // calculate the normal vector that points from the ball to the peg
            fix15 normal_x = divfix(dx, distance);
            fix15 normal_y = divfix(dy, distance);

            // collision physics
            fix15 dot = multfix15(normal_x, balls[i].vx) + 
                        multfix15(normal_y, balls[i].vy);

            fix15 intermediate_term = multfix15(int2fix15(-2), dot);

          // Are the ball velocity and normal vectors in opposite directions?
          if (intermediate_term > 0) {
            
            // Teleport it outside the collision distnace with the peg
            balls[i].x =
                  pegs[j].x+ multfix15(normal_x , (distance + int2fix15(3)));
            balls[i].y =
                  pegs[j].y + multfix15(normal_y , (distance + int2fix15(3)));

            // Update the ball velocity
            balls[i].vx = balls[i].vx + multfix15(normal_x , intermediate_term);
            balls[i].vy = balls[i].vy + multfix15(normal_y , intermediate_term);

            //Did we just strike a new peg?
            if (j != balls[i].last_peg){
              //Make a thunk sound by starting the control channel
              dma_start_channel_mask(1u << ctrl_chan);

              // update last peg
              balls[i].last_peg = j;
            }

            // Remove some energy from the ball
            balls[i].vx = multfix15(balls[i].vx , BOUNCINESS);
            balls[i].vy = multfix15(balls[i].vy , BOUNCINESS);
          }
      }
    }
  
      //Re-spwan any balls that hit thru bottom
      if (topHistogramTop(balls[i].y)){
        total_balls += 1;

        //find which bin the ball fallls into
        int bin_width = HORIZONTAL_SEPARATION;                     //same spacingg as pegs
        int start_bin_x = 322 - NUM_BINS * bin_width / 2;                       //start x position of histogram

        int bin_index = (fix2int15(balls[i].x) - start_bin_x) / bin_width;
        if (bin_index >= 0 && bin_index < NUM_BINS){
          balls_in_bins[bin_index] += 1;
        }

        spawn_ball(&balls[i]);
      }

      //Apply gravity 
      balls[i].vy = balls[i].vy + GRAVITY;

      //use the ball's velocity to update it's position
      balls[i].x  = balls[i].x + balls[i].vx ;
      balls[i].y  = balls[i].y + balls[i].vy ;
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

  // some varibales
  static char balls_being_animated[40];
  static char total_number_of_balls_since_reset[40];
  static char time_since_reset[40];
  static char bounciness_string[40];
  static char gravity_string[40];
  static int time = 0;

  while(1) {  
      //wait for 0.1 sec
      PT_YIELD_usec(1000000);

      fillRect(0, 0, 160, 100, BLACK);

      time += 1;

      setTextColor(WHITE);

      // number of balls being animated
      setTextSize(1);
      setCursor(20 , 20);
      sprintf(balls_being_animated, "Active Particles: %d", result);
      writeString(balls_being_animated);

      // total number of balls that have fallen through the board since reset
      setTextSize(1);
      setCursor(20 , 40);
      sprintf(total_number_of_balls_since_reset, "Total Balls: %d", total_balls);
      writeString(total_number_of_balls_since_reset);

      //time since boot 
      setTextSize(1);
      setCursor(20 , 60);
      sprintf(time_since_reset, "Time elapsed: %d", time);
      writeString(time_since_reset);

      // Bounciness 
      setTextSize(1);
      setCursor(20 , 80);
      sprintf(bounciness_string, "Bounciness: %.2f", fix2float15(BOUNCINESS));
      writeString(bounciness_string);

      //Gravity
      setTextSize(1);
      setCursor(20 , 100);
      sprintf(gravity_string, "Gravity: %.2f", fix2float15(GRAVITY));
      writeString(gravity_string);

  }

  PT_END(pt);

}

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    //intialze the balls
    initBalls();

    // spawn balls 
    for (int i  = 0; i < result; i++){
      spawn_ball(&balls[i]);
    }
    
    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;   

      //read potentiometer
      result = adc_read();
      printf("%d\n", result);

      //reset the histogram bins after getting the new potentiometer value
      // for (int i= 0; i < NUM_BINS; i++){
      //   balls_in_bins[i] = 0;
      // }

      // erase the old balls
      for (int i  = 0; i < result; i++){
        fillCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), BALL_RADIUS, BLACK);
        //drawCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), BALL_RADIUS, BLACK);
      }

      // update all balls once 
      update_ball_based_on_collision();
  
      // redraw new balls 
      for (int i  = 0; i < result; i++){
        fillCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), BALL_RADIUS, RED);
        //drawCircle(fix2int15(balls[i].x), fix2int15(balls[i].y), BALL_RADIUS, RED);
      }

      //draw new peg 
      draw_pegs();

      //clear old histogram
      //clear old colums
      clearLowFrame(HISTOGRAM_Y_START, BLACK);

      //draw histogram
      draw_histogram();
    
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
  // pt_add_thread(protothread_anim1);
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

