
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

// uS per frame
#define FRAME_RATE 33000

// the color of the boid
char color = WHITE;
char ball_color = BLUE;
char peg_color = RED;
char clear_color = BLACK;



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

#define BALL_RADIUS 4
#define PEG_RADIUS 6
#define GRAVITY 0.75
#define BOUNCINESS 0.5
#define V_SEP 19 
#define H_SEP 38 


// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(240) ;
  // Choose left or right
  if (direction) *vx = int2fix15(3) ;
  else *vx = int2fix15(-3) ;
  // Moving down
  *vy = int2fix15(1) ;
}


Ball ball = {
    int2fix15(320),   // x position
    int2fix15(0),     // y position
    int2fix15(0),     // vx
    int2fix15(0),     // vy
};

Peg peg =  {
    int2fix15(320),    // x position
    int2fix15(100),    // y position
};


void updateBall() {
    float dx = fix2float15(ball.x) - fix2float15(peg.x);
    float dy = fix2float15(ball.y) - fix2float15(peg.y);

    // --- collision check ---
    if (absfix15(float2fix15(dx)) < (int2fix15(BALL_RADIUS + PEG_RADIUS)) && absfix15(float2fix15(dy)) < (int2fix15(BALL_RADIUS + PEG_RADIUS))) {

        fix15 distance = float2fix15(sqrt((dx * dx) + (dy * dy)));
        fix15 normal_x = divfix(float2fix15(dx), distance);
        fix15 normal_y = divfix(float2fix15(dy), distance);

        fix15 intermediate_term = multfix15(int2fix15(-2), (multfix15(normal_x, ball.vx) + multfix15(normal_y, ball.vy)));

        if (intermediate_term > 0) {
            // bounce logic
            ball.x = peg.x + multfix15(normal_x, (distance + int2fix15(1)));
            ball.y = peg.y + multfix15(normal_y, (distance + int2fix15(1)));

            ball.vx = ball.vx + multfix15(normal_x, intermediate_term);
            ball.vy = ball.vy + multfix15(normal_y, intermediate_term);
        }
    }

    // --- always run physics ---
    if (hitBottom(ball.y - int2fix15(BALL_RADIUS))) {
        ball.x = int2fix15(320);
        ball.y = int2fix15(0);
        ball.vx = (int2fix15(rand() & 0xffff) >> 15) - int2fix15(1);
        ball.vy = 0;
    }

    // apply gravity and move
    ball.vy = ball.vy + float2fix15(GRAVITY);
    ball.x  = ball.x + ball.vx;
    ball.y  = ball.y + ball.vy;
}


// Draw the boundaries
void drawArena() {
  drawVLine(100, 100, 280, WHITE) ;
  drawVLine(540, 100, 280, WHITE) ;
  drawHLine(100, 100, 440, WHITE) ;
  drawHLine(100, 380, 440, WHITE) ;
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Reverse direction if we've hit a wall
  if (hitTop(*y)) {
    *vy = (-*vy) ;
    *y  = (*y + int2fix15(5)) ;
  }
  if (hitBottom(*y)) {
    *vy = (-*vy) ;
    *y  = (*y - int2fix15(5)) ;
  } 
  if (hitRight(*x)) {
    *vx = (-*vx) ;
    *x  = (*x - int2fix15(5)) ;
  }
  if (hitLeft(*x)) {
    *vx = (-*vx) ;
    *x  = (*x + int2fix15(5)) ;
  } 

  // Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // update boid color
        if ((user_input > 0) && (user_input < 16)) {
          color = (char)user_input ;
        }
      } // END WHILE(1)
  PT_END(pt);
} // timer thread





// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;


    // draw the peg globally 
    fillCircle(fix2int15(peg.x), fix2int15(peg.y), PEG_RADIUS, peg_color);

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;   


      // erase the old ball
      fillCircle(fix2int15(ball.x), fix2int15(ball.y), BALL_RADIUS, clear_color);

      // update 
      updateBall();

      //draw the new ball
      fillCircle(fix2int15(ball.x), fix2int15(ball.y), BALL_RADIUS, ball_color);
    
      // detect collision

      // drawArena() ;
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// // Animation on core 1
// static PT_THREAD (protothread_anim1(struct pt *pt))
// {
//     // Mark beginning of thread
//     PT_BEGIN(pt);

//     // Variables for maintaining frame rate
//     static int begin_time ;
//     static int spare_time ;

//     // Spawn a boid
//     // spawnBoid(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy, 1);

//     while(1) {
//       // Measure time at start of thread
//       begin_time = time_us_32() ;      
//       // // erase boid
//       // drawRect(fix2int15(boid1_x), fix2int15(boid1_y), 2, 2, BLACK);
//       // // update boid's position and velocity
//       // wallsAndEdges(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy) ;
//       // // draw the boid at its new position
//       // drawRect(fix2int15(boid1_x), fix2int15(boid1_y), 2, 2, color); 
//       // // delay in accordance with frame rate
//       // spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
//       // yield for necessary amount of time
//       PT_YIELD_usec(spare_time) ;
//      // NEVER exit while
//     } // END WHILE(1)
//   PT_END(pt);
// } // animation thread

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

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 

