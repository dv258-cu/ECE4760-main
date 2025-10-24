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
#define BUTTON_PIN 14 // GPIO pin for button


// the color of the boid
char boid_color = WHITE;


// Global variables for animation
int new_ball_count = 0;
int prev_frame_balls = 0;
float new_bounciness = 0.5f;


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


// #define BALL_RADIUS 4
// #define PEG_RADIUS  6
#define BALL_RADIUS int2fix15(2)
#define PEG_RADIUS int2fix15(6)
#define GRAVITY float2fix15(0.6)
fix15 BOUNCINESS = float2fix15(0.5);
#define VERTICAL_SEPARATION 19
#define HORIZONTAL_SEPARATION 38
#define NUMBER_OF_PEGS 136   // number of pegs = n(n+1)/2 = 16 (16 + 1) / 2 = 16 * 17 / 2 = 136 pegs 

#define SCREEN_CENTER 320
#define AVAILABLE_HEIGHT 90
//global variable for number of balls
uint16_t adc_result;
#define MAX_BALLS_30FPS 100  // Maximum balls that can run at 30fps



const int number_of_pegs = 136;   // Changed to get 16 row triangle peg
const int num_pegs_last_row = 16; // Last row should have 16 pegs ([0...15])

// Histogram Specs
#define BINS 17                               // total no of bins 17 since we have 15 spaces btwn the 16 pegs plus two more for the edges
int balls_in_bins[BINS];                      //counter for each bin

/*
  Math for space left for histogram
  first peg starts at y = 60 and vertival separation = 19 so bottom row y is  = start_y + (16 -1 ) * vertical_separation 
  = 60 + 15 * 19 = 60 + 285 = 345: 
  but each peg is circular with radi of 6 so end of last peg is 345 + 6 = 351 
*/

#define HISTOGRAM_Y_START 360 
#define HISTOGRAM_HEIGHT  100 


// Variable to determine how many balls have spawned in total
int TOTAL_BALLS = 0;

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

typedef struct {
  fix15 x;
  fix15 y;
  fix15 height;
  fix15 width;
} Histogram;


volatile enum {
  RESET,
  ADJUST_BALLS,
  ADJUST_BOUNCINESS,
} button_state = RESET;

uint16_t adc_return = 0;

// Global one ball + one peg 
Ball balls[4096];
Peg last_peg = {0, 0};
// histogram
Histogram histogram[BINS];
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

// Spawn a ball from top of screen
void spawn_ball(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy) {
  *x = int2fix15(320);
  *y = int2fix15(0);  

  fix15 rand_vx = ((int2fix15(rand() & 0xffff)>>16) - (int2fix15(1)>>1));
  *vx = rand_vx << 2;
  
  *vy = int2fix15(0);

  TOTAL_BALLS += 1;
}


// Calculate the starting x position for a row of pegs
int calculate_row_start_x(int pegs_in_row, int screen_center_x) {
  return screen_center_x - ((pegs_in_row - 1) * HORIZONTAL_SEPARATION / 2);
}

// Calculate peg position and draw a single peg
void draw_single_peg(int peg_index, int start_x, int start_y, int col, int row) {
  pegs[peg_index].x = int2fix15((start_x + (col * HORIZONTAL_SEPARATION)));
  pegs[peg_index].y = int2fix15((start_y + (row * VERTICAL_SEPARATION)));
  fillCircle(fix2int15(pegs[peg_index].x), fix2int15(pegs[peg_index].y),
             fix2int15(PEG_RADIUS), GREEN);
}

// Draw all pegs in a single row
void draw_peg_row(int *peg_index, int pegs_in_row, int start_x, int start_y, int row) {
  for (int col = 0; col < pegs_in_row; col++) {
    draw_single_peg(*peg_index, start_x, start_y, col, row);
    (*peg_index)++;
  }
}

// Draw the pegs on the screen
void draw_pegs() {
  const int screen_center_x = 320;
  const int start_y = 60;

  int row = 0;
  int peg_index = 0;

  while (peg_index < number_of_pegs) {
    int pegs_in_row = row + 1;
    int start_x = calculate_row_start_x(pegs_in_row, screen_center_x);
    
    draw_peg_row(&peg_index, pegs_in_row, start_x, start_y, row);
    row++;
  }
}

// Calculate the maximum ball count across all bins
int calculate_max_ball_count() {
  int max_ball_count = 0;
  for (int bin_index = 0; bin_index < BINS; bin_index++) {
    if (balls_in_bins[bin_index] > max_ball_count) {
      max_ball_count += balls_in_bins[bin_index];
    }
  }
  return max_ball_count;
}

// Calculate dynamic height based on total balls
int calculate_dynamic_height() {
  int dynamic_height = AVAILABLE_HEIGHT;
  if (TOTAL_BALLS > 1000) {
    // Reduce height by 5 pixels for every 1000 balls
    dynamic_height = AVAILABLE_HEIGHT - (TOTAL_BALLS / 1000) * 5;
    // Set minimum height to prevent the histogram from disappearing
    if (dynamic_height < 30) {
      dynamic_height = 30;
    }
  }
  return dynamic_height;
}

// Calculate scale factor for histogram bars
fix15 calculate_histogram_scale_factor(int max_ball_count, int dynamic_height) {
  int effective_max = (max_ball_count < 100) ? 100 : max_ball_count;
  return divfix(int2fix15(dynamic_height), int2fix15(effective_max));
}

// Draw a single histogram bar
void draw_histogram_bar(int bin_index, int start_x, fix15 scale_factor) {
  drawRect(fix2int15(histogram[bin_index].x), fix2int15(histogram[bin_index].y),
           fix2int15(histogram[bin_index].width), fix2int15(histogram[bin_index].height),
           BLACK);
  
  fix15 bar_height = multfix15(int2fix15(balls_in_bins[bin_index]), scale_factor);
  // Ensure a minimum height (2 pixels) for nonzero bins.
  if (balls_in_bins[bin_index] > 0 && bar_height < int2fix15(2)) {
    bar_height = int2fix15(0);
  }

  // TODO: Room for improvement: only redraw difference in height
  histogram[bin_index].height = bar_height;
  // TODO: no need to do this everytime since width is constant
  histogram[bin_index].width = int2fix15((HORIZONTAL_SEPARATION));

  histogram[bin_index].x = int2fix15((start_x - 1));
  histogram[bin_index].y = int2fix15(479) - bar_height;

  drawRect(fix2int15(histogram[bin_index].x), fix2int15(histogram[bin_index].y),
           fix2int15(histogram[bin_index].width), fix2int15(histogram[bin_index].height),
           GREEN);
}

// Draw the histogram on the board starting from the left.
void draw_histogram() {
  int start_x = SCREEN_CENTER - ((BINS) * (HORIZONTAL_SEPARATION / 2));

  int max_ball_count = calculate_max_ball_count();
  int dynamic_height = calculate_dynamic_height();
  fix15 scale_factor = calculate_histogram_scale_factor(max_ball_count, dynamic_height);

  for (int bin_index = 0; bin_index < BINS; bin_index++) {
    draw_histogram_bar(bin_index, start_x, scale_factor);
    // Add the 2 pixels of separation
    start_x += HORIZONTAL_SEPARATION + 1;
  }
}

// Reset the histogram
void reset_histogram() {
  for (int i = 0; i < BINS; i++) {
    balls_in_bins[i] = 0;
    TOTAL_BALLS = 0;
    drawRect(fix2int15(histogram[i].x), fix2int15(histogram[i].y),
             fix2int15(histogram[i].width), AVAILABLE_HEIGHT, BLACK);

    histogram[i].height = 0;
    histogram[i].width = 0;
  }
  // fillRect(0, 479 - AVAILABLE_HEIGHT, 640, AVAILABLE_HEIGHT, BLACK);
}

fix15 sqrt_alpha = float2fix15(.9604);
fix15 sqrt_beta = float2fix15(.3978);

// Check if a ball is colliding with a peg
bool check_ball_peg_collision(int ball_index, int peg_index) {
  // Compute x and y distances between ball and peg
  fix15 delta_x = balls[ball_index].x - pegs[peg_index].x;
  fix15 delta_y = balls[ball_index].y - pegs[peg_index].y;

  fix15 abs_delta_x = absfix15(delta_x);
  fix15 abs_delta_y = absfix15(delta_y);

  // Are both the x and y distances less than the collision distance
  return (abs_delta_x < (BALL_RADIUS + PEG_RADIUS)) &&
         (abs_delta_y < (BALL_RADIUS + PEG_RADIUS));
}

// Calculate collision distance between ball and peg
fix15 calculate_collision_distance(fix15 delta_x, fix15 delta_y) {
  fix15 abs_delta_x = absfix15(delta_x);
  fix15 abs_delta_y = absfix15(delta_y);
  
  if (abs_delta_x > abs_delta_y) {
    return abs_delta_x + (abs_delta_y >> 2);
  } else {
    return abs_delta_y + (abs_delta_x >> 2);
  }
}

// Handle collision response between ball and peg
void handle_collision_response(int ball_index, int peg_index, fix15 delta_x, fix15 delta_y, fix15 collision_distance) {
  // Generate the normal vector that points from peg to ball
  fix15 normal_x = divfix(delta_x, collision_distance);
  fix15 normal_y = divfix(delta_y, collision_distance);

  // collision physics
  fix15 velocity_dot_normal =
      multfix15(-int2fix15(2), (multfix15(normal_x, balls[ball_index].vx) +
                                multfix15(normal_y, balls[ball_index].vy)));

  // Are the ball and velocity and normal vectors in opposite
  if (velocity_dot_normal > 0) {
    // Teleport it outside the collision distance with the peg
    balls[ball_index].x =
        pegs[peg_index].x + multfix15(normal_x, (collision_distance + int2fix15(1)));
    balls[ball_index].y =
        pegs[peg_index].y + multfix15(normal_y, (collision_distance + int2fix15(1)));

    // Update its velocity
    balls[ball_index].vx = balls[ball_index].vx + multfix15(normal_x, velocity_dot_normal);
    balls[ball_index].vy = balls[ball_index].vy + multfix15(normal_y, velocity_dot_normal);

    // Did we just strike a new peg?
    if (peg_index == 0 || (last_peg.x != pegs[peg_index].x && last_peg.y != pegs[peg_index].y)) {
      // start the control channel
      dma_start_channel_mask(1u << ctrl_chan);
      // Remove some energy from the ball
      balls[ball_index].vx = multfix15(balls[ball_index].vx, BOUNCINESS);
      balls[ball_index].vy = multfix15(balls[ball_index].vy, BOUNCINESS);

      last_peg.x = pegs[peg_index].x;
      last_peg.y = pegs[peg_index].y;
    }
  }
}

// Handle ball falling through bottom and binning
void handle_ball_fall_through(int ball_index) {
  // Determine which bin the ball fell into
  for (int bin_index = 0; bin_index < BINS; bin_index++) {
    if (balls[ball_index].x >= histogram[bin_index].x &&
        balls[ball_index].x <= histogram[bin_index].x + histogram[bin_index].width) {
      balls_in_bins[bin_index] += 1;
      break;
    }
  }
  spawn_ball(&balls[ball_index].x, &balls[ball_index].y, &balls[ball_index].vx, &balls[ball_index].vy);
}

// Handle ball wall collisions
void handle_wall_collisions(int ball_index) {
  // Bounce any balls that hit the top/sides
  if (hitTop(balls[ball_index].y)) {
    balls[ball_index].vy = multfix15(-int2fix15(1), balls[ball_index].vy);
  }

  if (hitRight(balls[ball_index].y) || hitLeft(balls[ball_index].y)) {
    balls[ball_index].vx = -balls[ball_index].vx;
  }
}

// Apply physics to a single ball
void update_single_ball_physics(int ball_index) {
  // Apply gravity
  balls[ball_index].vy = balls[ball_index].vy + GRAVITY;

  // Use the ball's update velocity to update its position
  balls[ball_index].x = balls[ball_index].x + balls[ball_index].vx;
  balls[ball_index].y = balls[ball_index].y + balls[ball_index].vy;
}

// update balls based on collision
void update_based_on_collision_physics(uint16_t adc_val) {
  int number_of_balls = (int)adc_val;
  // Each frame, we update every ball
  for (int ball_index = 0; ball_index < number_of_balls; ball_index++) {
    // Every ball looks at every peg ...
    for (int peg_index = 0; peg_index < number_of_pegs; peg_index++) {
      if (check_ball_peg_collision(ball_index, peg_index)) {
        // Compute x and y distances between ball and peg
        fix15 delta_x = balls[ball_index].x - pegs[peg_index].x;
        fix15 delta_y = balls[ball_index].y - pegs[peg_index].y;
        
        fix15 collision_distance = calculate_collision_distance(delta_x, delta_y);
        handle_collision_response(ball_index, peg_index, delta_x, delta_y, collision_distance);
      }
    }

    // Re-spawn any balls that fall thru bottom
    if (hitBottom(balls[ball_index].y)) {
      handle_ball_fall_through(ball_index);
    }

    // Handle wall collisions and apply physics
    handle_wall_collisions(ball_index);
    update_single_ball_physics(ball_index);
  }
}


// Write to screen thread -  we could use this for vga screen setup
/*
  The VGA should display:
    - The current number of balls being animated
    - The total number of balls that have fallen through the board since reset 
    - Time since boot
*/
// Write to screen thread
static PT_THREAD(protothread_write_to_screen(struct pt *pt)) {
  PT_BEGIN(pt);

  static char time_since_reset[40];
  static char balls_to_string[40];
  static char total_number_since_reset[40];
  static char bounciness_string[40];
  static char state_string[40];
  static int time = 0;

  while(1) {
    // Clear the text area
    fillRect(0, 0, 200, 150, BLACK);

    time += 1;
    setTextColor(WHITE);
    setTextSize(1);
    
    // Display current number of balls being animated
    setCursor(20, 20);
    sprintf(balls_to_string, "Active Balls: %d", adc_result);
    writeString(balls_to_string);
    
    // Display total number of balls that have fallen through since reset
    setCursor(20, 40);
    sprintf(total_number_since_reset, "Total Balls: %d", TOTAL_BALLS);
    writeString(total_number_since_reset);
    
    // Display time since boot
    setCursor(20, 60);
    sprintf(time_since_reset, "Time: %ds", time);
    writeString(time_since_reset);
    
    // Display bounciness (tunable parameter)
    setCursor(20, 80);
    sprintf(bounciness_string, "Bounciness: %.2f", fix2float15(BOUNCINESS));
    writeString(bounciness_string);
    
    // Display current state
    setCursor(20, 100);
    switch(button_state) {
      case RESET:
        sprintf(state_string, "State: RESET");
        break;
      case ADJUST_BALLS:
        sprintf(state_string, "State: ADJUST BALLS");
        break;
      case ADJUST_BOUNCINESS:
        sprintf(state_string, "State: ADJUST BOUNCINESS");
        break;
      default:
        sprintf(state_string, "State: UNKNOWN");
        break;
    }
    writeString(state_string);
    
    // Update once per second
    PT_YIELD_usec(1000000);
  }

  PT_END(pt);
} // timer thread
// Key press debouncer
static PT_THREAD(protothread_switch(struct pt *pt)) {
  PT_BEGIN(pt);

  static enum {
    NOT_PRESSED,
    MAYBE_PRESSED,
    PRESSED,
    MAYBE_NOT_PRESSED
  } debounce_state = NOT_PRESSED;

  static int possible_press = 1;

  while (1) {
    // Note to self key press down is 0
    bool keypress = gpio_get(BUTTON_PIN);

    switch (debounce_state) {
    case NOT_PRESSED:
      if (keypress == 0) {
        possible_press = keypress;
        debounce_state = MAYBE_PRESSED;
      }
      break;

    case MAYBE_PRESSED:
      if (keypress == possible_press && keypress == 0) {
        switch (button_state) {
        case RESET:
          reset_histogram();
          button_state = ADJUST_BALLS;
          break;
        case ADJUST_BALLS:
          reset_histogram();

          button_state = ADJUST_BOUNCINESS;
          break;
        case ADJUST_BOUNCINESS:
          reset_histogram();
          button_state = RESET;
          break;
        default:
          break;
        }
        debounce_state = PRESSED;
      } else {
        debounce_state = NOT_PRESSED;
      }
      break;
    case PRESSED:
      // If key is still press do nothing, otherwise transition to
      // MAYBE_NOT_PRESSED
      if (keypress == 1) {
        debounce_state = MAYBE_NOT_PRESSED;
      }
      break;

    case MAYBE_NOT_PRESSED:
      if (keypress == 0) {
        debounce_state = PRESSED;
      } else {
        debounce_state = NOT_PRESSED;
        possible_press = 1;
      }
      break;

    default:
      break;
    }

    // printf("Button state: %d\n", button_state);

    // Schedule to be called again in 30ms.
    PT_YIELD_usec(30000);
  }
  PT_END(pt);
}


// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt)) {
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;

  static int prev_frame_balls = 0;
  static int new_ball_count = 0;
  static float prev_bounciness = 0;
  static uint16_t filtered_adc = 0;
  static fix15 computed_bounciness = float2fix15(0.5);

  // zero balls to the bins at the start
  for (int bin_index = 0; bin_index < BINS; bin_index++) {
    balls_in_bins[bin_index] = 0;
  }

  while (1) {
    // Measure time at start of thread
    begin_time = time_us_32();

    // Choose what potentiometer is addressing based on button state
    float new_bounciness;
    switch (button_state) {
    case RESET:
      // reset_histogram();
      break;
    case ADJUST_BALLS:
      new_ball_count = new_ball_count + (((adc_read())-new_ball_count) >> 4);
      if (abs(new_ball_count - prev_frame_balls) > 8) {
        reset_histogram();
      }
      break;
    case ADJUST_BOUNCINESS:
      new_bounciness =
          new_bounciness + ((int)((adc_read() >> 4) - new_bounciness) >> 3);
      new_bounciness = new_bounciness / 15.0;
      computed_bounciness = float2fix15(new_bounciness);
      if (fabs(new_bounciness - prev_bounciness) > 0.1) {
        reset_histogram();
        prev_bounciness = new_bounciness;
      }
      BOUNCINESS = computed_bounciness;
      break;
    default:
      break;
    }

    for (int ball_index = 0; ball_index < prev_frame_balls; ball_index++) {
      fillCircle(fix2int15(balls[ball_index].x), fix2int15(balls[ball_index].y),
               fix2int15(BALL_RADIUS), BLACK);
    }
    // Spawn difference in balls
    for (int ball_index = prev_frame_balls; ball_index < new_ball_count; ball_index++) {
      spawn_ball(&balls[ball_index].x, &balls[ball_index].y, &balls[ball_index].vx, &balls[ball_index].vy);
    }
    update_based_on_collision_physics(new_ball_count);

    // Draw updated balls
    for (int ball_index = 0; ball_index < new_ball_count; ball_index++) {
      fillCircle(fix2int15(balls[ball_index].x), fix2int15(balls[ball_index].y),
               fix2int15(BALL_RADIUS), RED);
    }
    prev_frame_balls = new_ball_count;
    adc_return = new_ball_count;

    draw_pegs();
    draw_histogram();
    spare_time = FRAME_RATE - (time_us_32() - begin_time);
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread



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

  //initialize adc
  adc_init();
  adc_gpio_init(POTENTIOMETER_OUTPUT_PIN);
  adc_select_input(1);

  //initialize button
  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);
  gpio_pull_up(BUTTON_PIN);


  //initialize LED
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  // start core 1 
  // multicore_reset_core1();
  // multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_write_to_screen);
  pt_add_thread(protothread_anim);
  pt_add_thread(protothread_switch);

  // start scheduler
  pt_schedule_start ;
} 