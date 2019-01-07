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



#define LED_PIN     4
#define NUM_LEDS    263
#define BRIGHTNESS  255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 300
#define LED_DELAY 120000
#define FADE_BY 1
#define DEBUG 0
#define PIN_IN_DOWNSTAIRS 5
#define PIN_IN_UPSTAIRS 6

#define MY_RADIO_NRF24
#define MY_REPEATER_FEATURE
#define MY_RF24_PA_LEVEL (RF24_PA_MAX)
#define MY_DEBUG
#define MY_TRANSPORT_WAIT_READY_MS 3000

#define MY_CHILD_ID_DOWNSTAIRS 1
#define MY_CHILD_ID_UPSTAIRS   2
#define MY_CHILD_ID_LIGHT      3
#define MY_NODE_ID 1

#define SN "TrapLicht"
#define SV "1.0"

#include <FastLED.h>
#include <MySensors.h>

MyMessage upstairs(MY_CHILD_ID_DOWNSTAIRS, V_TRIPPED);
MyMessage downstairs(MY_CHILD_ID_UPSTAIRS, V_TRIPPED);
MyMessage lightMsg(MY_CHILD_ID_LIGHT, V_LIGHT);
MyMessage lightColor(MY_CHILD_ID_LIGHT, V_RGB);

int currentLevel = 0;
unsigned long hold_start_time;

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

void presentation() {
  sendSketchInfo(SN, SV);
  present(MY_CHILD_ID_DOWNSTAIRS, S_DOOR);
  present(MY_CHILD_ID_UPSTAIRS,   S_DOOR);
  present(MY_CHILD_ID_LIGHT,      S_RGB_LIGHT);
}


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
  Serial.println("setup() enter");
  wait( 3000 ); // power-up safety delay
  pinMode(PIN_IN_UPSTAIRS, INPUT);
  pinMode(PIN_IN_DOWNSTAIRS, INPUT);
  //  Serial.begin(9600);
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  Serial.println("setup() exit");
}

void poll_touch() {
  if (digitalRead(PIN_IN_DOWNSTAIRS)) {
    //    Serial.println("downstairs triggered");
    Serial.flush();
    state = MOVE_UP_START;
    send(downstairs.set(1));
  } else if (digitalRead(PIN_IN_UPSTAIRS)) {
    //Serial.println("upstairs triggered");
    Serial.flush();
    state = MOVE_DOWN_START;
    send(upstairs.set(1));
  }
}

void loop() {
  switch (state) {
    case IDLE:
      poll_touch();
      break;
    case MOVE_UP_START:
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      state = MOVING_UP;
      send(lightMsg.set(1));
      break;
    case MOVE_DOWN_START:
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      state = MOVING_DOWN;
      send(lightMsg.set(1));
      break;
    case MOVING_UP:
      //fill_rainbow( leds, NUM_LEDS, gHue, 7);
      leds[moveindex] = CRGB::White;
      moveindex++;
      if (moveindex == NUM_LEDS) {
        hold_start_time = millis();
        state = HOLD;
      }
      break;
    case MOVING_DOWN:
      leds[NUM_LEDS - 1 - moveindex] = CRGB::White;
      moveindex++;
      if (moveindex == NUM_LEDS) {
        hold_start_time = millis();
        state = HOLD;
      }
      break;
    case HOLD:
      if(millis() - hold_start_time > LED_DELAY) {
        moveindex = 0;
        state = DIMMING;
      }
      break;
    case DIMMING:
      if (leds[0][0] > 5 || leds[0][1] > 5  || leds[0][2] > 5) {
        fadeToBlackBy(leds, NUM_LEDS, FADE_BY);
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        delay(1000);
        state = IDLE;
        send(lightMsg.set(0));
      }
      break;
    default:
      break;
  }
  EVERY_N_MILLISECONDS( 20 ) {
    gHue++;  // slowly cycle the "base color" through the rainbow
  }
  FastLED.show();
  wait(1000/ UPDATES_PER_SECOND );
}


void receive(const MyMessage &message)
{
  if (message.type == V_LIGHT || message.type == V_DIMMER) {
    int requestedLevel = atoi( message.data );
    requestedLevel *= ( message.type == V_LIGHT ? 100 : 1 );
     // Adjust incoming level if this is a V_LIGHT variable update [0 == off, 1 == on]
    requestedLevel *= ( message.type == V_LIGHT ? 100 : 1 );
    
    // Clip incoming level to valid range of 0 to 100
    requestedLevel = requestedLevel > 100 ? 100 : requestedLevel;
    requestedLevel = requestedLevel < 0   ? 0   : requestedLevel;
    
    currentLevel = requestedLevel;
    bool on = currentLevel > 0;
    Serial.print("turning light ");
    Serial.println(on);
    send(lightMsg.set(on));
    if(on) {
      hold_start_time = millis();
      state = HOLD;
      fill_solid(leds, NUM_LEDS, CRGB::White);
      FastLED.show();
    } else {
      moveindex = 0;
      state = DIMMING;
    }
  }
  if (message.type == V_RGB){
    const char * rgb = message.getString();
    Serial.print("color: "); Serial.println(rgb);
  }
}

