/*******************************************************************************
  MidiVibes.ino
  
  MIDI Vibraphone
  https://github.com/BarryKVibes/MidiVibes
  Copyright 2022, Barry K Vibes
  
 *******************************************************************************
  
  This file is part of MidiVibes.
  
  MidiVibes is free software: you can redistribute it and/or 
  modify it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  MidiVibes is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License along 
  with MidiVibes. If not, see <https://www.gnu.org/licenses/>.
  
 ******************************************************************************/
// This code is used to convert an acoustic Vibraphone into a MIDI Controller.
// NOTE: This is prototype/proof-of concept code. Feel free to use/modify for your own project.

// A Piezo Disc is attached to each bar of the Vibraphone. The Piezo is input
// into a passive envelope follower, which is then input to an Analog pin of the Arduino Mega.
// https://drummagazine.com/dont-pull-that-trigger/
// An Arduino Mega has 16 Analog inputs, so each Arduino Mega supports up to 16 notes. Multiple Arduino Mega are needed for more notes.
// MIDI is output via USB and intended to be used with a computer. To support multiple Megas, a USB Hub is used.
// Software on the computer then converts the USB data to MIDI. On the Mac, the software is Hairless MIDI. 
// Unfortunately, Hairless MIDI is not supported on later versions of Mac OS. TODO: Find alternative to Hairless MIDI, or 
// merge Megas using I2C.
// See https://github.com/BarryKVibes/MidiElectronicAccordion to output MIDI on Pin 7 to a standard MIDI Output port.
// A single Arduino could be used if the Piezo Discs are thresholded and limited (to +5V) and input to the 
// Digital Inputs; Mega supports about 50 Digital IO pins.
// This version keeps note on as long as foot pedal is down (switch connected to Digital Input pin 2).

// TODO: Describe or reference envelope follower circuit. Piezo->Envelope Follower Circuit->Analog Input.
// TODO: Allow retrigger.

// The following compiler flags are used to log MIDI data to the Serial Monitor, instead of the USB port.
// SEND_MIDI and LOG_RAW_DATA should not be defined at the same time.
#define SEND_MIDI
#ifndef SEND_MIDI
  #define LOG_RAW_DATA
  #ifndef LOG_RAW_DATA
    #define PRINT_MIDI
  #endif
#endif

#ifndef SEND_MIDI
//  #define LOG_AVERAGE_TIMESTAMPS
#endif

#define COUNT_ENTRIES(ARRAY)        (sizeof(ARRAY) / sizeof(ARRAY[0]))

// This constant is used in a multi-Arduino system that shares code in this file.
// The idea is to use one Arduino for a range of notes, and hard-wire digital inputs to indicate which Arduino this code is for.
// Multiple Arduinos could be configured with a Master, and one or more Slaves, and communicate Note On/Off to the Master, via I2C, but this is not implemented in this code.
// See https://github.com/BarryKVibes/MidiElectronicAccordion on how multiple Arduinos are used to convey digital on/off states.
const uint8_t ARDUINO_NUMBER = 0; // TODO: Use Digital Inputs as address lines to determine which Arduino this is.  For now, just use Arduino 0 or 1.

// I intended to make the MIDI Vibes velocity sensitive to cut off the notes when the volume went below a threshold
// but the passive envelope follower cut off the notes too soon due to the voltage drop accross the diode in the envelope follower.
// https://en.wikipedia.org/wiki/Envelope_detector. https://www.youtube.com/watch?v=Fn5kHhNRsz0, https://www.youtube.com/watch?v=jllsqRWhjGM.
// My circuit used 1N4148 Diodes, 22nF capacitors, 1M Ohm resistor.
// TODO: Add an Op Amp to each note, as discussed in https://www.youtube.com/watch?v=Fn5kHhNRsz0 and enable velocity sensitivity.  
// Therefore, this feature is disabled. 
const bool velocitySensitive = false;

const uint32_t   minAmplitude = 30;
const uint32_t   curAmplitude = minAmplitude;

const int SustainPedalPin = 2;
const int LedPin = 13;

bool curPedalState = false;

struct pad_t
{
    uint32_t    active      :  1; //  1 false/true
    uint32_t    note        :  7; //  8 0 - 127
    uint32_t    minAmplitude : 10; // 18 0 - 1023
    uint32_t    unused1   :  7; // 25 0 - 127
    // curAmplitude used for tracking envelope to detect retrigger.  
    // It is the minimum amplitude while envelope is decaying.
    uint32_t    curAmplitude:  10; // 32 0 - 127
    uint32_t    unused2:  10; // 32 0 - 127
};

// Ardu0 = Low Naturals F3..C4..C5..G5; 
// Ardu1 = Accidentals; 
// Ardu2 = High Naturals A5..C6..B6; 
struct pad_t    pads[][3] =
{
      { { false, 53, minAmplitude, 0, curAmplitude }, { false, 54, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD   0, 16, 32
    , { { false, 55, minAmplitude, 0, curAmplitude }, { false, 56, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD   1, 17, 33
    , { { false, 57, minAmplitude, 0, curAmplitude }, { false, 58, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD   2, 18, 34
    , { { false, 59, minAmplitude, 0, curAmplitude }, { false, 61, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD   3, 19, 35
    , { { false, 60, minAmplitude, 0, curAmplitude }, { false, 63, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD   4, 20, 36
    , { { false, 62, minAmplitude, 0, curAmplitude }, { false, 66, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD   5, 21, 37
    , { { false, 64, minAmplitude, 0, curAmplitude }, { false, 68, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD   6, 22, 38
    , { { false, 65, minAmplitude, 0, curAmplitude }, { false, 70, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD   7, 23, 39
    , { { false, 67, minAmplitude, 0, curAmplitude }, { false, 73, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD   8, 24, 40
    , { { false, 69, minAmplitude, 0, curAmplitude }, { false, 75, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD   9, 25, 41
    , { { false, 71, minAmplitude, 0, curAmplitude }, { false, 78, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD  10, 26, 42
    , { { false, 72, minAmplitude, 0, curAmplitude }, { false, 80, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD  11, 27, 43
    , { { false, 74, minAmplitude, 0, curAmplitude }, { false, 82, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD  12, 28, 44
    , { { false, 76, minAmplitude, 0, curAmplitude }, { false, 85, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD  13, 29, 45
    , { { false, 77, minAmplitude, 0, curAmplitude }, { false, 87, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD  14, 30, 46
    , { { false, 79, minAmplitude, 0, curAmplitude }, { false,  0, minAmplitude, 0, curAmplitude }, { false, 0, minAmplitude, 0, curAmplitude } } // PAD  15, 31, 47
};

// MIDI channel from 0 to 15 ( 1-16 in "real world")
const uint8_t   MIDI_CHANNEL    = 0;

void midi_send(uint8_t midi_message, uint8_t pitch, uint8_t midi_velocity)
{
  uint8_t noteOnCommand = midi_message + MIDI_CHANNEL;

#ifdef SEND_MIDI
  Serial.write(noteOnCommand);      // BYTE 1
  Serial.write(pitch);              // BYTE 2
  Serial.write(midi_velocity);      // BYTE 3
#else
#ifdef PRINT_MIDI
//  Serial.println("MIDI Note: 0x" + String(noteOnCommand, HEX) + " 0x" + String(pitch, HEX) + " 0x" + String(midi_velocity, HEX));
  Serial.println("MIDI Note: 0x" + String(noteOnCommand, HEX) + " " + String(pitch, DEC) + " " + String(midi_velocity, DEC));
#endif
#endif
}

void do_channel(uint8_t const pinADC, struct pad_t& pad, int iROW)
{
  int analogValue = analogRead(pinADC);

  // A/D is 1 to 1023 but conditioning circuit limits voltage into Arduino.  
  // Therefore map smaller range into the MIDI velocity range.
  // int velocity = map(analogValue, 0, 1023 , 0, 127);
  
  int constrainedAnalogValue = constrain(analogValue, 0, 300);
  int velocity = map(constrainedAnalogValue, 0, 300 , 0, 127);
#ifdef LOG_RAW_DATA

    Serial.print(String(analogValue, DEC) + "\t");
    //Serial.print(String(velocity, DEC) + "\t");

#endif

  // Allow note to trigger if above specified threshold.
  if (analogValue > pad.minAmplitude)
  {
#ifdef LOG_RAW_DATA
    // Serial.print(String(analogValue, DEC) + "\t");
#endif
  
    if (!pad.active)
    {
      pad.active = true;
      
      // Start tracking envelope to detect retrigger.
      pad.curAmplitude = analogValue;
      
      // NOTE ON
      // Serial.print("AnalogValue: " + String(analogValue, DEC) + "\t" + "Velocity: " + String(velocity, DEC) + "\t");

      if (!velocitySensitive) {
        velocity = 0x7F;
      }
      
      midi_send(0x90, pad.note, velocity);
    }
  }
  else if (pad.active)
  {
    // Track min envelope value during decay.
    if (analogValue < pad.curAmplitude) {
      pad.curAmplitude = analogValue;
    }
    
    if (analogValue <= pad.minAmplitude) {
       pad.curAmplitude = pad.minAmplitude;

      // If pedal is depressed, prevent turning off note.
      if (!curPedalState) {
        pad.active = false;
        // NOTE OFF
        midi_send(0x90, pad.note, 0);
      }
    }
  }
}

void loop()
{
  // Rows represent 16 notes.  
  const int     ROWS    = COUNT_ENTRIES(pads);
  const int     COLUMN  = ARDUINO_NUMBER;
  const uint8_t pinsADC[ROWS] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15 };

  // If pedal changed from on to off, turn off MIDI notes
  bool newPedalState = checkFootSwitchOnD2();
  if (curPedalState && !newPedalState) {
#ifdef PRINT_MIDI
    Serial.println("Pedal Turned Off");
#endif
 //   for(size_t iCOLUMN = START_COLUMN; iCOLUMN < COLUMNS; iCOLUMN++) {
      for(size_t iROW = 0; iROW < ROWS; iROW++) {
        struct pad_t& pad = pads[iROW][COLUMN];
        if (pad.active) {
          pad.active = false;
          
#ifdef PRINT_MIDI
//          Serial.println("Turning Off Note: " + String(pad.note) );
#endif
          // NOTE OFF
          midi_send(0x90, pad.note, 0);
        }
      }
//    }
  }
  else if (!curPedalState && newPedalState) {
#ifdef PRINT_MIDI
    Serial.println("Pedal Turned On");
#endif 
  }
  
  curPedalState = newPedalState;

  // Check if any notes have been activated or stopped ringing.
 // for(size_t iCOLUMN = START_COLUMN; iCOLUMN < COLUMNS; iCOLUMN++) {
    for(size_t iROW = 0; iROW < ROWS; iROW++) {
       do_channel(pinsADC[iROW], pads[iROW][COLUMN], iROW);
    }
//  }

#ifdef LOG_RAW_DATA
   Serial.println();
#endif

}

// Returns true if sustain pedal switch is on, otherwise false.
bool checkFootSwitchOnD2() {

  int sustainPedalValue = digitalRead(SustainPedalPin);
  // Serial.println(sustainPedalValue);

  // Since the Sustain Pedal digital input is configured as INPUT_PULLUP,
  // the input's logic is inverted. The input goes HIGH when it's open,
  // and LOW when it's closed. Turn on LED when the pedal is pressed, otherwise off.
  if (sustainPedalValue == HIGH) {
    digitalWrite(LedPin, LOW);
    return false;
    
  } else {
    digitalWrite(LedPin, HIGH);
    return true;
  }
}

void setup()
{
  // Setup sustain pedal input.
  pinMode(SustainPedalPin, INPUT_PULLUP);

  // Setup LED pin as Output.
  pinMode(LedPin, OUTPUT);
  
  Serial.begin(115200);
}
