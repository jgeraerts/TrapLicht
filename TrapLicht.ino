/**
Copyright (c) 2017, Jo Geraerts
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <FastLED.h>

#define LED_PIN     3
#define NUM_LEDS    263
#define BRIGHTNESS  255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 300
#define LED_DELAY 30000
#define FADE_BY 1
#define DEBUG 0
#define PIN_IN_DOWNSTAIRS 4
#define PIN_IN_UPSTAIRS 5

enum State {
  IDLE,
  MOVE_UP_START,
  MOVE_DOWN_START,
  MOVING_UP,
  MOVING_DOWN,
  HOLD,
  DIMMING
};

CRGB     leds[NUM_LEDS];
uint8_t  gHue = 0;
uint16_t moveindex = 0;
State    state = IDLE;

void error() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void setup() {
  delay( 3000 ); // power-up safety delay
  pinMode(PIN_IN_UPSTAIRS, INPUT);
  pinMode(PIN_IN_DOWNSTAIRS, INPUT);
  Serial.begin(9600);
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
}

inline void poll_touch() {
  if (digitalRead(PIN_IN_DOWNSTAIRS)) {
    state = MOVE_UP_START;
  } else if (digitalRead(PIN_IN_UPSTAIRS)) {
    state = MOVE_DOWN_START;
  }
}

void loop() {
  if (DEBUG) {
    /** Serial.print("State: ");
      Serial.println(state,HEX);
      Serial.print("gHue: ");
      Serial.println(gHue,HEX);
      Serial.print("moveIndex: ");
      Serial.println(moveindex,HEX);
    **/
  }
  switch (state) {
    case IDLE:
      /**
      fadeToBlackBy(leds, NUM_LEDS, FADE_BY);
      EVERY_N_MILLISECONDS( 2000 ) {
        uint16_t pos = random16(NUM_LEDS);
        leds[pos] += CHSV( gHue + random8(64), 200, 255);
      }**/
      poll_touch();
      break;
    case MOVE_UP_START:
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      state = MOVING_UP;
      break;
    case MOVE_DOWN_START:
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      state = MOVING_DOWN;
      break;
    case MOVING_UP:
      //fill_rainbow( leds, NUM_LEDS, gHue, 7);
      leds[moveindex] = CRGB::White;
      moveindex++;
      if (moveindex == NUM_LEDS) {
        state = HOLD;
      }
      break;
    case MOVING_DOWN:
      leds[NUM_LEDS - 1 - moveindex] = CRGB::White;
      moveindex++;
      if (moveindex == NUM_LEDS) {
        state = HOLD;
      }
      break;
    case HOLD:
      FastLED.delay(LED_DELAY);
      moveindex = 0;
      state = DIMMING;
      break;
    case DIMMING:
      if (leds[0][0] > 5 || leds[0][1] > 5  || leds[0][2] > 5) {
        fadeToBlackBy(leds, NUM_LEDS, FADE_BY);
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        delay(1000);
        state = IDLE;
      }
      break;
    default:
      break;
  }
  EVERY_N_MILLISECONDS( 20 ) {
    gHue++;  // slowly cycle the "base color" through the rainbow
  }
  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);
}
