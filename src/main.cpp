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
// who adapted it for an LED frequency indicator
// https://www.lookmumnocomputer.com/projects#/1222-performance-vco
// and Jos Bouten aka Zaphod B who significantly cleaned and optimized
// the code (Jan-Feb 2020)
// https://github.com/josbouten/Tune-O-Matic

// Modification of Sam / Jos' tuner code to accommodate an OLED
// display was made by Ryan P. Marchildon Jan-Feb 2021

// === PIN SETUP ===
// TODO
// Frequency input is A0 on Arduino Uno using the defaults in this file.
// For I2C OLED: SCK is pin A5, SDA is A4

// === LICENSE ===
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.

#include <Arduino.h>
#include <U8g2lib.h>

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

// --------------------- FREQUENCY MEASUREMENT LOGIC ---------------------

int CLIPPING_LED = 13;

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

// Define Interrupt Routine
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

// --------------------- MAIN PROGRAM ---------------------
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
}

// loop variables
int closestIndex;
String note;
int sharp;
int offset;
int octave;
int halfNoteUp;
int halfNoteDown;

// NOTE: there are 3 main OLED buffer modes (full screen, page buffer, and u8x8)
// with tradeoffs between speed and RAM usage; here we use the
// page buffer mode; if you wanted full screen, you'd have to implement the
// OLED code in the main loop differently; see here for more details:
// https://github.com/olikraus/u8g2/wiki/setup_tutorial
void loop()
{

  // # note: this is actually frequency * 10
  frequency = TIMER_RATE_10 / period; // Timer rate with an extra zero/period.

  if ((frequency > 0) && (frequency < 158))
  {
    // too low, out of range
    Serial.println(F("LOW freq condition"));
  }
  else if ((frequency > 10180) && (frequency < 100000))
  {
    // too high, out of range
    Serial.println(F("HIGH freq condition"));
  }
  else
  {
    // in range, determine note and offset
    closestIndex = getClosestFrequencyIndex(frequency);
    note = indexToNote(closestIndex);
    sharp = indexToSharp(closestIndex);
    octave = indexToOctave(closestIndex);
    offset = frequency - frequencyTable[closestIndex];

    // determine frequencies halfway to adjacent notes
    // (used to scale the offset indicator)
    halfNoteDown = (frequencyTable[closestIndex] + frequencyTable[closestIndex - 1]) / 2;
    halfNoteUp = (frequencyTable[closestIndex] + frequencyTable[closestIndex + 1]) / 2;
  }

  // update the OLED display (dev/test)
  // char m_str[6];
  // strcpy(m_str, u8x8_u8toa(frequency / 10, 5)); /* convert m to a string with 5 digits */
  char buf1[9];
  char buf2[9];
  // sprintf(buf1, note);
  sprintf(buf2, "%d Hz", frequency / 10);
  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(0, 24, note.c_str());
    u8g2.drawStr(25, 48, buf2);
  } while (u8g2.nextPage());

  delay(70);
  Serial.print(frequency / 10);
  Serial.println(F("Hz"));

}
