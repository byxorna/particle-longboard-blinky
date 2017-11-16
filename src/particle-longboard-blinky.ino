/*
* Project particle-longboard-blinky
* Description: longboard LEDs
* Author: Gabe Conradi
* Date: idklol
*/

#include "Particle.h"

// here be dragons. apparently arduino is picky about includes
// so this is some magic ordering... fuck c++
#include "led_sysdefs.h"
#include "colorutils.h"
#include "colorpalettes.h"
#include "pixeltypes.h"
#include "FastLED.h"

#include "LIS3DH.h"

SYSTEM_MODE(SEMI_AUTOMATIC);

SYSTEM_THREAD(ENABLED);

#define NUM_LEDS_PER_STRIP 15
#define NUM_STRIPS 2

#define LED_TYPE NSFastLED::WS2811
#define INDEX_RIGHT 1
#define INDEX_LEFT 0

#define UPDATES_PER_SECOND 100
#define MAX_BRIGHTNESS 255
#define MAX_SATURATION 255

#define PATTERN_CHANGE_INTERVAL_S 15
#define PALETTE_CHANGE_INTERVAL_S 15
#define AUTO_CHANGE_PALETTE 0
#define AUTO_CHANGE_PATTERNS 0


// accelerometer stuff
#define ACCEL_POSITION_NORMAL 5
#define ACCEL_POSITION_UPSIDEDOWN 4
#define ACCEL_POSITION_A 3
#define ACCEL_POSITION_B 2
#define ACCEL_POSITION_C 1

void accel_positionInterruptHandler();
LIS3DHSample accel_now; // accelerometer value last sampled
// LIS3DH is connected as in the AssetTracker, to the primary SPI with A2 as the CS (SS) pin, and INT connected to WKP
LIS3DHSPI accel(SPI, A2, WKP);
volatile bool accel_positionInterrupt = false;
uint8_t accel_lastPos = 0;

uint8_t gBrightness = 128; // global brightness
uint8_t gPattern = 0; // global pattern
uint8_t gPalette = 0; // global palette
uint8_t gAnimIndex = 0; // animation index for ColorFromPalette

unsigned long lastPrintSample = 0;
unsigned long t_now;   // time now in each loop iteration
unsigned long t_pattern_start = 0;   // time last pattern started

// for effects that are palette based
NSFastLED::CRGBPalette16 currentPalette; // current color palette
NSFastLED::TBlendType currentBlending;
NSFastLED::CRGB leds[NUM_STRIPS*NUM_LEDS_PER_STRIP]; //[NUM_STRIPS][NUM_LEDS_PER_STRIP];

void accel_positionInterruptHandler() {
  Serial.println("accelerometer position changed");
	accel_positionInterrupt = true;
}


// setup() runs once, when the device is first turned on.
void setup() {

  Serial.begin(9600);
  Serial.println("resetting");

  // chill for a sec
  delay( 1000 );

  // Initialize sensors
  LIS3DHConfig config;
  config.setAccelMode(LIS3DH::RATE_25_HZ);
  attachInterrupt(WKP, accel_positionInterruptHandler, RISING);
  config.setPositionInterrupt(16);
  bool setupSuccess = accel.setup(config);
  Serial.printlnf("accelerometer setup: %d", setupSuccess);
  // lets Initialize the accelerometer struct
  accel.getSample(accel_now);

  currentPalette = NSFastLED::RainbowColors_p;
  currentBlending = NSFastLED::LINEARBLEND;

  NSFastLED::FastLED.addLeds<LED_TYPE, 4>(leds[0], NUM_LEDS_PER_STRIP);
  NSFastLED::FastLED.addLeds<LED_TYPE, 5>(leds[NUM_LEDS_PER_STRIP], NUM_LEDS_PER_STRIP);
  NSFastLED::FastLED.setBrightness(gBrightness);

  // reset pattern
  gPattern = 0;
  gPalette = 0;
}

// pattern to display when we are flipped upside down
void pattern_beached_whale() {
  // pick a color, and just pulse it slowly
  // 5000ms per breath period
  uint8_t cBrightness = NSFastLED::quadwave8((millis()/5000)%256);
  uint8_t cHue = 0;
  for( int s = 0; s < NUM_STRIPS; s++) {
    for( int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      leds[s][i] = NSFastLED::CHSV(cHue, 255, cBrightness);
    }
  }
}

void pattern_from_palette() {
  for( int s = 0; s < NUM_STRIPS; s++) {
    for( int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      leds[s][i] = NSFastLED::ColorFromPalette(currentPalette, gAnimIndex, MAX_BRIGHTNESS, currentBlending);
      gAnimIndex += 255/(NUM_LEDS_PER_STRIP*NUM_STRIPS);
    }
  }
}


void loop() {
  t_now = millis();
  if (t_now - lastPrintSample >= 100) {
    lastPrintSample = t_now;
    if (accel.getSample(accel_now)) {
      Serial.printlnf("acc: %d,%d,%d", accel_now.x, accel_now.y, accel_now.z);
    }
  }

  // react to accel position interrupt and set accel_lastPos
  if (accel_positionInterrupt) {
    accel_positionInterrupt = false;
    // Test the position interrupt support. Normal result is 5.
		// 5: normal position, with the accerometer facing up
		// 4: upside down
		// 1 - 3: other orientations
		uint8_t pos = accel.readPositionInterrupt();
		if (pos != 0 && pos != accel_lastPos) {
			Serial.printlnf("acc pos=%d", pos);
			accel_lastPos = pos;
		}
  }

  // increment pattern every PATTERN_CHANGE_INTERVAL_S
  if (AUTO_CHANGE_PATTERNS && (t_now > t_pattern_start+PATTERN_CHANGE_INTERVAL_S*1000)) {
    gPattern++;
    Serial.printlnf("pattern->%d", gPattern);
  }

  // increment palette every PALETTE_CHANGE_INTERVAL_S
  if (AUTO_CHANGE_PALETTE && (t_now > t_palette_start+PALETTE_CHANGE_INTERVAL_S*1000)) {
    switch(gPalette) {
      case 0: currentPalette = NSFastLED::RainbowColors_p;  currentBlending = NSFastLED::LINEARBLEND; break;
      case 1: currentPalette = NSFastLED::PartyColors_p;    currentBlending = NSFastLED::LINEARBLEND; break;
      case 2: currentPalette = NSFastLED::CloudColors_p;    currentBlending = NSFastLED::LINEARBLEND; break;
      case 3: currentPalette = NSFastLED::ForestColors_p;   currentBlending = NSFastLED::LINEARBLEND; break;
      case 4: currentPalette = NSFastLED::OceanColors_p;    currentBlending = NSFastLED::LINEARBLEND; break;
      case 5: currentPalette = NSFastLED::LavaColors_p;     currentBlending = NSFastLED::LINEARBLEND; break;
      default:
      gPalette = 0;
      currentPalette = NSFastLED::RainbowColors_p; currentBlending = NSFastLED::LINEARBLEND; break;
    }
  }

  if (accel_lastPos == ACCEL_POSITION_UPSIDEDOWN) {
    // act like a beached turtle and breathe slowly
    pattern_beached_whale();
  } else {
    switch(gPattern) {
      case 0: pattern_from_palette();
      break;
      default:
      gPattern = 0;
      break;
    }
  }



  // TODO: make dynamic? FastLED.setBrightness(gBrightness);
  NSFastLED::FastLED.show();
  NSFastLED::FastLED.delay(1000 / UPDATES_PER_SECOND);
}
