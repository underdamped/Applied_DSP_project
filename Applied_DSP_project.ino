/*
 * CET4190C Applied Digital Signal Processing
 * Final Project:  using Teensy 3.6 as filter, isolate bird sounds from audio file
 * 
 * Notes:
 * 
 * This program is designed to run on a Teensy 3.6 (ARM Cortex-M4 core with FPU).
 * The program waits for data on the serial bus, reading a byte at a time until
 * the number of frames (samples) received equals the BLOCK_SIZE.  The frames are
 * filtered with arm_fir_f32(), using the filter coefficients stored in coeff[],
 * and the filtered output is sent back out the serial bus in BLOCK_SIZE bursts.
 * 
 * The program expects incoming data to be the raw binary representation of IEEE754
 * single-precision (32-bit) floats; thus, each frame consists of 4 bytes.  Likewise,
 * it sends the filtered data as raw bytes.
 * 
 * The end-of-data marker is the float value +Inf (which should never appear in audio).
 * 
 * Javier Lombillo
 * 2017-02-15
 */

// we're using the libarm_cortexM4lf_math.a library
// (the 'l' stands for little-endian, and the 'f' for FPU)
#include "arm_math.h"

// global constants
#define BLOCK_SIZE    512 // number of samples to process at a time
#define NUM_TAPS      129 // filter order 128
#define SIZE_OF_FLOAT 4

// quick way to flash the LED
#define LED_ON     GPIOC_PSOR=(1<<5)
#define LED_OFF    GPIOC_PCOR=(1<<5) 

// serial data comes in bytes, but we're interested in floats; this union lets us
// treat 4 contiguous bytes of memory as either type.  both x86 and the cortex-m4
// toolchain are little-endian, so byte order is not an issue.
static union bits32
{
  float f;
  byte bytes[4];
} u;

// these filter coefficients implement a 128th-order BPF with corner frequencies 3 kHz and 4.5 kHz
// derived from MATLAB with: fdesign.bandpass('N,Fc1,Fc2',128,3e3,4.5e3,48e3)
const float32_t coeff[NUM_TAPS] = {
-0.00000000, -0.00007008, -0.00009113, -0.00002513,  0.00013749,  0.00036728,
 0.00059456,  0.00072185,  0.00065115,  0.00032121, -0.00025584, -0.00097354,
-0.00163953, -0.00201906, -0.00190718, -0.00120906,  0.00000000,  0.00146253,
 0.00278979,  0.00356917,  0.00349872,  0.00250404,  0.00079139, -0.00119145,
-0.00288479, -0.00380031, -0.00369574, -0.00267418, -0.00116140,  0.00024356,
 0.00100039,  0.00085944, -0.00000000, -0.00099470, -0.00134035, -0.00037793,
 0.00208847,  0.00557760,  0.00895038,  0.01070074,  0.00945882,  0.00455724,
-0.00353842, -0.01311741, -0.02152878, -0.02587113, -0.02389583, -0.01485427,
 0.00000000,  0.01746785,  0.03307833,  0.04224553,  0.04160534,  0.03014013,
 0.00972686, -0.01511104, -0.03824238, -0.05351210, -0.05644398, -0.04557407,
-0.02300955,  0.00598386,  0.03428674,  0.05478172,  0.06225073,  0.05478172,
 0.03428674 , 0.00598386, -0.02300955, -0.04557407, -0.05644398, -0.05351210,
-0.03824238, -0.01511104,  0.00972686,  0.03014013,  0.04160534,  0.04224553,
 0.03307833,  0.01746785,  0.00000000, -0.01485427, -0.02389583, -0.02587113,
-0.02152878, -0.01311741, -0.00353842,  0.00455724,  0.00945882,  0.01070074,
 0.00895038,  0.00557760,  0.00208847, -0.00037793, -0.00134035, -0.00099470,
-0.00000000,  0.00085944,  0.00100039,  0.00024356, -0.00116140, -0.00267418,
-0.00369574, -0.00380031, -0.00288479, -0.00119145,  0.00079139,  0.00250404,
 0.00349872,  0.00356917,  0.00278979,  0.00146253,  0.00000000, -0.00120906,
-0.00190718, -0.00201906, -0.00163953, -0.00097354, -0.00025584,  0.00032121,
 0.00065115,  0.00072185,  0.00059456,  0.00036728,  0.00013749, -0.00002513,
-0.00009113, -0.00007008, -0.00000000
};

//static uint32_t blockSize = BLOCK_SIZE;

// filter state buffer
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

// filter structure
static arm_fir_instance_f32 S;
  
// I/O buffers
static float32_t input_buffer[BLOCK_SIZE];
static float32_t output_buffer[BLOCK_SIZE];

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);

  init_filter();
  
  // wait until the serial port is available
  while (!Serial);

  blinkenlights();
  Serial.begin(115200);
}

void loop(void)
{
  // get a block of data from matlab, storing it in input_buffer[]
  LED_OFF;  get_serial_data();  LED_ON;

  // filter the data, storing it in output_buffer[]
  arm_fir_f32( &S, &input_buffer[0], &output_buffer[0], BLOCK_SIZE );

  // send it back to matlab
  send_serial_data();
}

// do nothing until data is on the bus, then slurp up BLOCK_SIZE chunks
static void get_serial_data(void)
{
  int i, j;
  
  for ( i = 0; i < BLOCK_SIZE; i++ )
  {
    // read a float value as a sequence of four bytes
    for ( j = 0; j < SIZE_OF_FLOAT; j++ )
    {
      // spin until a byte is available on the bus
      while ( !Serial.available() );

      // get the next byte
      u.bytes[j] = Serial.read();
    }
    
    input_buffer[i] = u.f;

    // check for EOF marker (IEEE754 +Inf)
    if ( isinf(input_buffer[i]) )
      return;
  }
}

// push the filtered data back through the bus in BLOCK_SIZE chunks
static void send_serial_data(void)
{
  int i;

  for ( i = 0; i < BLOCK_SIZE; i++ )
  {
    // end of data stream, re-initialize the filter and go back to waiting mode
    if ( isinf(output_buffer[i]) || isnan(output_buffer[i]) )
    {
      blinkenlights();
      init_filter();
      return;
    }

    u.f = output_buffer[i];
    Serial.write( u.bytes, SIZE_OF_FLOAT );
  }
}

// initialize the filter state
static void init_filter(void)
{
  arm_fir_init_f32( &S, NUM_TAPS, (float32_t *)&coeff[0], &firStateF32[0], BLOCK_SIZE );
}

// diagnostics via LED
static void blinkenlights(void)
{
  LED_ON; delay(250);
  LED_OFF; delay(100);
  LED_ON; delay(250);
  LED_OFF; delay(100);
  LED_ON; delay(250);
  LED_OFF;
}

