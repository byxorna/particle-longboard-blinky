/*
* Project particle-longboard-blinky
* Description: longboard LEDs
* Author: Gabe Conradi
* Date: idklol
*/

#include "FastLED.h"
#include "LIS3DH.h"

SYSTEM_MODE(MANUAL);

#define NUM_LEDS_PER_STRIP 15
#define NUM_STRIPS 2

#define LED_TYPE NEOPIXEL
#define INDEX_RIGHT 1
#define INDEX_LEFT 0

#define UPDATES_PER_SECOND 100
#define MAX_BRIGHTNESS 255

#define PATTERN_CHANGE_INTERVAL_S 15
#define PALETTE_CHANGE_INTERVAL_S 15
#define AUTO_CHANGE_PALETTE 0
#define AUTO_CHANGE_PATTERNS 0

// 0 is L, 1 is R
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

// accelerometer stuff
// LIS3DH is connected as in the AssetTracker, to the primary SPI with A2 as the CS (SS) pin, and INT connected to WKP
LIS3DHSPI accel(SPI, A2, WKP);

uint8_t gBrightness = 128; // global brightness
uint8_t gPattern = 0; // global pattern
uint8_t gAnimIndex = 0; // animation index for ColorFromPalette

unsigned long lastPrintSample = 0;
unsigned long t_now;   // time now in each loop iteration
unsigned long t_pattern_start = 0;   // time last pattern started
LIS3DHSample accel_now; // accelerometer value last sampled

// for effects that are palette based
CRGBPalette16 currentPalette; // current color palette
TBlendType currentBlending;

// setup() runs once, when the device is first turned on.
void setup() {

  Serial.begin(9600);
  Serial.println("resetting");

  // chill for a sec
  delay( 1000 );

  // Initialize sensors
  LIS3DHConfig config;
  config.setAccelMode(LIS3DH::RATE_25_HZ);
  bool setupSuccess = accel.setup(config);
  Serial.printlnf("accelerometer setup: %d", setupSuccess);
  // lets Initialize the accelerometer struct
  accel.getSample(accel_now);

  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;

  FastLED.addLeds<LED_TYPE, 4>(leds[INDEX_LEFT], NUM_LEDS_PER_STRIP);
  FastLED.addLeds<LED_TYPE, 5>(leds[INDEX_RIGHT], NUM_LEDS_PER_STRIP);

  FastLED.setBrightness(gBrightness);

  // reset pattern
  gPattern = 0;
  gPalette = 0;
}



void loop() {
  t_now = millis();
  if (t_now - lastPrintSample >= 100) {
    lastPrintSample = t_now;
    if (accel.getSample(accel_now)) {
      Serial.printlnf("acc: %d,%d,%d", accel_now.x, accel_now.y, accel_now.z);
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
      case 0: currentPalette = RainbowColors_p; currentBlending = LINEARBLEND; break;
      case 1: currentPalette = PartyColors_p; currentBlending = LINEARBLEND; break;
      case 2: currentPalette = CloudColors_p; currentBlending = LINEARBLEND; break;
      case 3: currentPalette = ForestColors_p; currentBlending = LINEARBLEND; break;
      case 4: currentPalette = OceanColors_p; currentBlending = LINEARBLEND; break;
      case 5: currentPalette = LavaColors_p; currentBlending = LINEARBLEND; break;
      default:
      gPalette = 0;
      currentPalette = RainbowColors_p; currentBlending = LINEARBLEND; break;
    }
  }

  switch(gPattern) {
    case 0: pattern_from_palette(&leds);
    break;
    default:
    gPattern = 0;
    break;
  }

  // TODO: make dynamic? FastLED.setBrightness(gBrightness);
  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);
}

void pattern_from_palette(CRGB **leds) {
  for( int s = 0; s < NUM_STRIPS; s++) {

  for( int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
    leds[s][i] = ColorFromPalette(currentPalette, gAnimIndex, MAX_BRIGHTNESS, currentBlending);
    colorIndex += 255/(NUM_LEDS_PER_STRIP*NUM_STRIPS);
  }
}
}
