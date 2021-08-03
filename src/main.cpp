// -----------------------
// ARDUINO OLED NOTE TUNER
// -----------------------

//       ,´ `.
// ______|___|______________________________________________
//       |  /                       _..-´|
//       | /                  _..-´´_..-´|
// ______|/__________________|_..-´´_____|__________|\______
//      ,|                   |           |          | \     
//     / |                   |           |          | ´
// ___/__|___________________|___________|__________|_______
//   / ,´| `.                |      ,d88b|          |
//  | .  |   \            __ |      88888|       __ |
// _|_|__|____|_________,d88b|______`Y88P'_____,d88b|_______
//  |  ` |    |         88888|                 88888|
//  `.   |   /          `Y88P'                 `Y88P'
// ___`._|_.´_______________________________________________
//       |
//     , |                                  GP
//     '.´

// === PROJECT HISTORY ===
// This tuner code was adapted from excellent the sine wave
// frequency detection project by Amanda Ghassaei (July 2012)
// https://www.instructables.com/member/amandaghassaei/
// as well as contributions from Sam / LookMumNoComputer
// and Jos Bouten aka Zaphod B who adapted it into an LED-based
// frequency indicator (Jan-Feb 2020) 
// https://www.lookmumnocomputer.com/projects#/1222-performance-vco 
// https://github.com/josbouten/Tune-O-Matic

// Modification of Sam / Jos' tuner code to accommodate an OLED
// display was made by Ryan P. Marchildon Jan-Feb 2021. The note
// mapping logic was rewritten so that the octave, sharps, 
// and a visual offset indicator could be included.

// === PIN SETUP ===
// Audio Signal input is pin A0 (Ardino Nano or Uno)
// and must be between 0 and 5 volts to avoid clipping
// (refer to schematic on github page for example of how
// to bias the input).

// For an I2C OLED with pinout (VDD, GND, SCK, SDA), these
// map to Arduino pins (5V, GND, A5, A4) respectively. 


// === LICENSE ===
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.

#include <Arduino.h>
#include <U8g2lib.h>
#include <movingAvg.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


// --------------------- OLED SETUP ---------------------
// Refer to the official U8G2 library documentation here for
// more information: https://github.com/olikraus/u8g2/wiki

// specify your OLED model, comms protocal (I2C or SPI), buffer mode, and pin configuration
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// adjust these constants given your OLED screen size
#define SCREEN_PIXEL_WIDTH 128  // how many horizontal pixels total
#define SCREEN_PIXEL_HEIGHT 64 // how many vertical pixels total


// --------------------- FREQUENCY MEASUREMENT LOGIC ---------------------
// for smoothing of the frequency reading; set to zero to disable
#define MOVING_AVERAGE_SAMPLES 5
movingAvg smoothedFrequency(MOVING_AVERAGE_SAMPLES);

// Audio signal amplitude storage variables, used for determining slope
byte newData = 0;
byte prevData = 0;

// Freq variables.
unsigned int period;
int frequency;

#define HALF_SAMPLE_VALUE 127 // mid-point of [0, 255] ADC range
#define TIMER_RATE 38462
#define TIMER_RATE_10 TIMER_RATE * 10

// Data storage variables.
unsigned int time = 0; // Keeps time and sends values to store in timer[] occasionally.
#define BUFFER_SIZE 10
int timer[BUFFER_SIZE];  // Storage for timing of events.
int slope[BUFFER_SIZE];  // Storage for slope of events.
unsigned int totalTimer; // Used to calculate period.
byte index = 0;          // Current storage index.
int maxSlope = 0;        // Used to calculate max slope as trigger point.
int newSlope;            // Storage for incoming slope data.

// Variables for decided whether you have a match.
#define MAX_NO_MATCH_VALUE 9
byte noMatch = 0;  // Counts how many non-matches you've received to reset variables if it's been too long.
byte slopeTol = 3; // Slope tolerance - adjust this if you need.
int timerTol = 10; // Timer tolerance - adjust this if you need.

// Variables for amp detection.
unsigned int ampTimer = 0;
byte maxAmp = 0;
byte checkMaxAmp;
byte ampThreshold = 30; // Raise if you have a very noisy signal.
long clippingTimer = 0;

// Clipping indicator variables.
boolean clipping = true;
#define CLIPPING_TIME 5 * TIMER_RATE // This should amount to 2 seconds.

void reset()
{               // Clear out some variables.
  index = 0;    // Reset index.
  noMatch = 0;  // Reset match counter.
  maxSlope = 0; // Reset slope.
}

// Define Interrupt Routine for Frequency Detection
ISR(ADC_vect)
{                     // Interrupt Triggers when new ADC value ready.
  
  PORTB &= B11101111; // Set pin 12 low.
  prevData = newData; // Store previous value.
  newData = ADCH;     // Get new audio signal value from A0.

  if (prevData < HALF_SAMPLE_VALUE && newData >= HALF_SAMPLE_VALUE)
  {                                // if increasing and crossing midpoint
    newSlope = newData - prevData; // Calculate slope
    
    if (abs(newSlope - maxSlope) < slopeTol)
    { // If slopes are ==
      // Record new data and reset time.
      slope[index] = newSlope;
      timer[index] = time;
      time = 0;
      if (index == 0)
      {                     // New max slope just reset.
        PORTB |= B00010000; // Set pin 12 high.
        noMatch = 0;
        index++; // Increment index.
      }
      else if (abs(timer[0] - timer[index]) < timerTol && abs(slope[0] - newSlope) < slopeTol)
      { //if timer duration and slopes match
        // Sum timer values.
        totalTimer = 0;
        for (byte i = 0; i < index; i++)
        {
          totalTimer += timer[i];
        }
        period = totalTimer; // Set period.
        // Reset new zero index values to compare with.
        timer[0] = timer[index];
        slope[0] = slope[index];
        index = 1;          // Set index to 1.
        PORTB |= B00010000; // Set pin 12 high.
        noMatch = 0;
      }
      else
      {          // Crossing midpoint but not match.
        index++; // Increment index.
        if (index > BUFFER_SIZE - 1)
        {
          reset();
        }
      }
    }
    else if (newSlope > maxSlope)
    { // If new slope is much larger than max slope.
      maxSlope = newSlope;
      time = 0; // Reset clock.
      noMatch = 0;
      index = 0; // Reset index.
    }
    else
    {            // Slope not steep enough.
      noMatch++; // Increment no match counter.
      if (noMatch > MAX_NO_MATCH_VALUE)
      {
        reset();
      }
    }
  }

  if (newData == 0 || newData == 1023)
  {                     // If clipping
    PORTB |= B00100000; // set pin 13 high, i.e. turn on clipping indicator led.
    clipping = true;    // Currently clipping.
  }

  time++; // Increment timer at rate of 38.5kHz
  clippingTimer++;
  if (clippingTimer > CLIPPING_TIME)
  {
    PORTB &= B11011111; // Set pin 13 low, i.e. turn off clipping indicator led.
    clipping = false;   // Currently not clipping.
    clippingTimer = 0;
  }

  ampTimer++; // Increment amplitude timer.
  if (abs(HALF_SAMPLE_VALUE - ADCH) > maxAmp)
  {
    maxAmp = abs(HALF_SAMPLE_VALUE - ADCH);
  }
  if (ampTimer == 1000)
  {
    ampTimer = 0;
    checkMaxAmp = maxAmp;
    maxAmp = 0;
  }
}


// --------------------- NOTE MAPPING LOGIC ---------------------
#define NOTES_PER_OCTAVE 12

// reference: https://pages.mtu.edu/~suits/notefreqs.html
const int frequencyTable[] = {
    // frequencies of each note in Hz, multiplied by ten
    // (so we can represent the float to one decimal place as an int),
    // formatted with one full octave per row for readability, i.e.
    // C, C#,  D,   D#,  E,   F,   F#,  G,  G#,  A,   A#,   B
    164, 173, 183, 195, 206, 218, 231, 245, 260, 275, 291, 309,                         // 0th octave
    327, 347, 367, 389, 412, 436, 462, 490, 519, 550, 583, 617,                         // 1st octave
    654, 693, 734, 778, 824, 873, 925, 980, 1038, 1100, 1165, 1235,                     // 2nd octave
    1308, 1386, 1468, 1556, 1648, 1746, 1850, 1960, 2076, 2200, 2330, 2469,             // 3rd octave
    2616, 2772, 2937, 3111, 3296, 3492, 3670, 3920, 4153, 4400, 4662, 4939,             // 4th octave
    5232, 5544, 5873, 6222, 6592, 6985, 7400, 7840, 8306, 8800, 9323, 9878,             // 5th octave
    10465, 11087, 11747, 12445, 13185, 13969, 14800, 15680, 16612, 17600, 18647, 19755, // 6th octave
};

int FREQUENCY_TABLE_LENGTH = sizeof(frequencyTable) / sizeof(frequencyTable[0]);

// we define the lookups this way to save memory
const String noteTable[] = {"C", "C", "D", "D", "E", "F", "F", "G", "G", "A", "A", "B"};
const int sharpTable[] = {0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0}; // 1 = sharp, otherwise 0

String indexToNote(int index)
{
  // converts frequency table index to note
  return noteTable[index % NOTES_PER_OCTAVE];
}

int indexToSharp(int index)
{
  // whether the note at this index is a sharp or not
  return sharpTable[index % NOTES_PER_OCTAVE];
}

int indexToOctave(int index)
{
  // the octave of the note at this index
  // (we want the minimum number of times it divides,
  // and dividing two ints produces this result through
  // truncation, i.e. 9/12 = 0.75 -> 0, 27/12 = 2.25 -> 2)
  int divisor = NOTES_PER_OCTAVE;
  return index / divisor;
}

int getClosestFrequencyIndex(int frequency)
{
  // find the closest match in frequencyTable
  int diff;
  int minDiff = 30000;
  int bestIndex = 0;
  for (int i = 0; i < FREQUENCY_TABLE_LENGTH; i++)
  {
    diff = abs(frequency - frequencyTable[i]);
    if (diff <= minDiff)
    {
      minDiff = diff;
      bestIndex = i;
    }
    else
    {
      // diff is increasing thus we've just passed the best match
      break;
    }
  }
  return bestIndex;
}

int getIndicatorPosition(int rangeStart, int rangeStop, int value)
{
  // given a range of frequencies spanning rangeStart to rangeStop,
  // maps 'value' to a number between 0 and 100, where 50 is ~ in-tune,
  // so we can display the present frequency relative to the target
  // note as an X-coordinate on the OLED
  return 100 * (value - rangeStart) / (rangeStop - rangeStart);
}


// --------------------- OLED DISPLAY LOGIC ---------------------
// adjust size and position for note display
// (defaults below are for 128x64 pixel display);
// for list of available fonts see:
// https://github.com/olikraus/u8g2/wiki/fntlistall
#define NOTE_FONT u8g2_font_ncenB24_tr
#define SHARP_FONT u8g2_font_ncenB12_tr
#define OCTAVE_FONT u8g2_font_ncenB12_tr

#define NOTE_X 46 // bottom-left coordinates of note relative to screen
#define NOTE_Y 48

#define SHARP_X_OFFSET 26 // position relative to note
#define SHARP_Y_OFFSET -18 

#define OCTAVE_X_OFFSET 26 // position relative to note
#define OCTAVE_Y_OFFSET 4 


void drawNote(String note, int sharp, int octave){
  // display note value, e.g. G, A, B ...
  u8g2.setFont(NOTE_FONT);
  u8g2.drawStr(NOTE_X, NOTE_Y, note.c_str());

  // show sharp value if a sharp
  u8g2.setFont(SHARP_FONT);
  if (sharp == 1){
      u8g2.drawStr(NOTE_X + SHARP_X_OFFSET, NOTE_Y + SHARP_Y_OFFSET, "#");
  }
  
  // show what octave we are in
  u8g2.setFont(OCTAVE_FONT);
  char oct_buf[1];
  sprintf(oct_buf, "%d", octave);
  u8g2.drawStr(NOTE_X + OCTAVE_X_OFFSET, NOTE_Y + OCTAVE_Y_OFFSET, oct_buf);
}


// adjust tuning indicator settings
#define INDICATOR_HEIGHT 5 // how tall to make indicator bar
#define BUCKETS 5 // number of buckets for tuning indicator
#define TARGET_START 40 // in terms of indicator position
#define TARGET_STOP 60
#define EMPHASIS_OFFSET 5 // adding "oomph" lines when we hit target

// derived constants
#define BUCKET_WIDTH SCREEN_PIXEL_WIDTH / BUCKETS
#define TARGET_X_START SCREEN_PIXEL_WIDTH * TARGET_START / 100
#define TARGET_X_STOP SCREEN_PIXEL_WIDTH * TARGET_STOP / 100
#define INDICATOR_Y_TOP SCREEN_PIXEL_HEIGHT - INDICATOR_HEIGHT
#define INDICATOR_Y_BOTTOM SCREEN_PIXEL_HEIGHT
#define TARGET_PIXEL_WIDTH TARGET_X_STOP - TARGET_X_START


void drawIndicator(int indicatorPosition){
  // provides visualization of current tuning versus target for displayed note
  // NOTE: assumes indicatorPosition is in range [0, 100]

  // draw bounds of target
  u8g2.drawLine(TARGET_X_START, INDICATOR_Y_BOTTOM, TARGET_X_START, INDICATOR_Y_TOP);
  u8g2.drawLine(TARGET_X_STOP, INDICATOR_Y_BOTTOM, TARGET_X_STOP, INDICATOR_Y_TOP);
 
  // draw box indicating where we are relative to target
  if (TARGET_START < indicatorPosition  && indicatorPosition < TARGET_STOP) {
    // fill the central target position
    u8g2.drawBox(TARGET_X_START, INDICATOR_Y_TOP, TARGET_PIXEL_WIDTH, INDICATOR_HEIGHT); // x of upper left, y of upper left, width, height
  
    // add a little emphasis to get those sweet victory feelz
    u8g2.drawLine(TARGET_X_START - EMPHASIS_OFFSET, INDICATOR_Y_BOTTOM, TARGET_X_START - EMPHASIS_OFFSET, INDICATOR_Y_TOP);
    u8g2.drawLine(TARGET_X_STOP + EMPHASIS_OFFSET, INDICATOR_Y_BOTTOM, TARGET_X_STOP + EMPHASIS_OFFSET, INDICATOR_Y_TOP);
  
  }
  else {
    // determine indicator box based on offset
    int bucket_id = (indicatorPosition * BUCKETS) / 100;// should be in range [0, BUCKETS]
    u8g2.drawBox(bucket_id * BUCKET_WIDTH, INDICATOR_Y_TOP, BUCKET_WIDTH, INDICATOR_HEIGHT); // x of upper left, y of upper left, width, height
  }

}

// adjust frequency display settings
#define FREQ_FONT u8g2_font_ncenB08_tr
#define FREQ_HEIGHT 12
#define FREQ_TARG_OFFSET 2 // adds extra vertical space
#define NUM_DECIMALS 1

void drawFrequency(int freq, int target){
  
  u8g2.setFont(FREQ_FONT);

  // we will include decimals
  float freqFloat;
  float targetFloat;
  freqFloat = (float) freq;
  targetFloat = (float) target;

  // display actual frequency
  char strBuff1[10];
  char strBuff2[10];
  dtostrf(freqFloat / 10, 2, NUM_DECIMALS, strBuff1); // needed to convert float to string
  sprintf(strBuff2, "%s Hz", strBuff1);
  u8g2.drawStr(0, FREQ_HEIGHT, strBuff2);

  // also show the target
  char strBuff3[10];
  char strBuff4[10];
  dtostrf(targetFloat / 10, 2, NUM_DECIMALS, strBuff3); // needed to convert float to string
  sprintf(strBuff4, "(%s)", strBuff3);
  u8g2.drawStr(0, 2 * FREQ_HEIGHT, strBuff4);

}

// range warning messages
void drawFreqTooLow(int limit){

  u8g2.setFont(FREQ_FONT);
  
  u8g2.drawStr(0, FREQ_HEIGHT, "Out of Range:");

  char buf[16];
  sprintf(buf, "Too Low (<%d Hz)", limit / 10);
  u8g2.drawStr(0, 2 * FREQ_HEIGHT, buf);
}

void drawFreqTooHigh(int limit){

  u8g2.setFont(FREQ_FONT);
  
  u8g2.drawStr(0, FREQ_HEIGHT, "Out of Range:");

  char buf[16];
  sprintf(buf, "Too High (>%d Hz)", limit / 10);
  u8g2.drawStr(0, 2 * FREQ_HEIGHT, buf);
}

// clipping warning display
#define CLIPPING_MSG_X_START 95 // horizontal location of message

void drawClippingWarning(){
  u8g2.setFont(FREQ_FONT);
  u8g2.drawStr(CLIPPING_MSG_X_START, FREQ_HEIGHT, "!CLIP");
}

// --------------------- PROGRAM SETUP ---------------------
void setup()
{
  Serial.begin(9600);

  cli(); // Disable interrupts.

  // Set up continuous sampling of analog pin 0.

  // Clear ADCSRA and ADCSRB registers.
  ADCSRA = 0;
  ADCSRB = 0;

  ADMUX |= (1 << REFS0); // Set reference voltage.
  ADMUX |= (1 << ADLAR); // Left align the ADC value so we can read highest 8 bits from ADCH register only

  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); // Set ADC clock with 32 prescaler -> 16mHz / 32 = 500kHz.
  ADCSRA |= (1 << ADATE);                // Enable auto trigger.
  ADCSRA |= (1 << ADIE);                 // Enable interrupts when measurement complete.
  ADCSRA |= (1 << ADEN);                 // Enable ADC.
  ADCSRA |= (1 << ADSC);                 // Start ADC measurements.

  sei(); // Enable interrupts.

  // initialize the OLED
  u8g2.begin();

  // initialize the moving average filter
  if (MOVING_AVERAGE_SAMPLES > 0) {
    smoothedFrequency.begin();
    for (int i=0; i <= MOVING_AVERAGE_SAMPLES; i++){
      // we also pre-seed it so it's never undefined
      smoothedFrequency.reading(50);
    }
  }

}



// --------------------- MAIN PROGRAM ---------------------
// loop variables
int closestIndex;
String note;
int sharp;
int targetFreq;
int octave;
int freqRangeStart;
int freqRangeEnd;
int indicatorPosition;

// NOTE: there are 3 main OLED buffer modes (full screen, page buffer, and u8x8)
// with tradeoffs between speed and RAM usage; here we use the
// page buffer mode; if you wanted full screen, you'd have to implement the
// OLED code in the main loop differently; see here for more details:
// https://github.com/olikraus/u8g2/wiki/setup_tutorial
void loop()
{
  
  // Extract the frequency for this loop iteration
  // Note: this is actually frequency * 10
  frequency = TIMER_RATE_10 / period; // Timer rate with an extra zero/period.

  // Smooth the frequency if moving average enabled
  if (MOVING_AVERAGE_SAMPLES > 0){
    smoothedFrequency.reading(frequency);
    frequency = smoothedFrequency.getAvg();
  }

  if (frequency < 158)
  {
    // display out of range (too low) message
    u8g2.firstPage();
    do
    {
      drawFreqTooLow(158);
      if (clipping) {
        drawClippingWarning();
      }
    } while (u8g2.nextPage());
  }
  else if (frequency > 12000)
  {
    // display out of range (too high) message
    u8g2.firstPage();
    do
    {
      drawFreqTooHigh(12000);
      if (clipping) {
        drawClippingWarning();
      }
    } while (u8g2.nextPage());
  }
  else
  {
    // in range, determine note and offset
    closestIndex = getClosestFrequencyIndex(frequency);
    note = indexToNote(closestIndex);
    sharp = indexToSharp(closestIndex);
    octave = indexToOctave(closestIndex);
    targetFreq = frequencyTable[closestIndex]; // freq diff in Hz times 10

    // determine frequencies halfway to adjacent notes
    // (i.e. the start and end of the frequency range of the offset indicator)
    freqRangeStart = (targetFreq + frequencyTable[closestIndex - 1]) / 2;
    freqRangeEnd = (targetFreq + frequencyTable[closestIndex + 1]) / 2;

    // map current (actual) frequency to range [0, 100] where
    // freqRangeStart = 0, freqRangeEnd = 100, and the target is ~50
    indicatorPosition = getIndicatorPosition(freqRangeStart, freqRangeEnd, frequency);

    // update the OLED display
    u8g2.firstPage();
    do
    {
      // see https://github.com/olikraus/u8g2/wiki/u8g2reference
      drawFrequency(frequency, targetFreq);
      drawNote(note, sharp, octave);
      drawIndicator(indicatorPosition);

      if (clipping) {
        drawClippingWarning();
      }

    } while (u8g2.nextPage());

  }

  delay(70);

  // uncomment these if you want to display the
  // frequency on the serial port, for debugging
  // Serial.print(frequency / 10);
  // Serial.println(F("Hz"));

}
