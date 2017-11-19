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
FASTLED_USING_NAMESPACE;


#include "LIS3DH.h"

SYSTEM_MODE(SEMI_AUTOMATIC);

SYSTEM_THREAD(ENABLED);

#define CLOCK_PIN D4
#define NUM_LEDS_PER_STRIP 15
#define NUM_STRIPS 2

#define LED_TYPE NEOPIXEL
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
CFastLED* gLED; // global CFastLED object

unsigned long lastPrintSample = 0;
unsigned long t_now;                // time now in each loop iteration
unsigned long t_pattern_start = 0;  // time last pattern changed
unsigned long t_palette_start = 0;  // time last palette changed

/* BRAKING STATE VARS */
bool braking = false;
unsigned long t_brake;              // time braking started
unsigned long t_brake_end;          // time braking ended
// when we activate the brakes, hold the brake light for X ms
// after we decide we arent braking anymore
#define BRAKE_HOLD_MS 3000

// for effects that are palette based
CRGBPalette16 currentPalette; // current color palette
TBlendType currentBlending;
CRGB leds[NUM_STRIPS*NUM_LEDS_PER_STRIP]; //[NUM_STRIPS][NUM_LEDS_PER_STRIP];

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

  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;

  // led controller, data pin, clock pin, RGB type (RGB is already defined in particle)
  gLED = new CFastLED();
  CFastLED::addLeds<LED_TYPE, D4>(leds, NUM_LEDS_PER_STRIP);
  CFastLED::addLeds<LED_TYPE, D5>(leds + NUM_LEDS_PER_STRIP, NUM_LEDS_PER_STRIP);
  gLED->setBrightness(gBrightness);

  // reset pattern
  gPattern = 0;
  gPalette = 0;
}

// pattern to display when we are flipped upside down
void pattern_flipped_over() {
  // pick a color, and just pulse it slowly
  // 5000ms per breath period
  uint8_t cBrightness = NSFastLED::quadwave8((millis()/5000)%256);
  uint8_t cHue = (millis()/30000) /256; // cycle color wheel every 30s
  NSFastLED::CHSV hsv_led = NSFastLED::CHSV(cHue, 255, cBrightness);
  NSFastLED::CRGB rgb_led;
  NSFastLED::hsv2rgb_rainbow(hsv_led, rgb_led);
  for( int s = 0; s < NUM_STRIPS; s++) {
    for( int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      leds[s*i] = rgb_led;
    }
  }
}

void pattern_from_palette() {
  for( int s = 0; s < NUM_STRIPS; s++) {
    for( int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      leds[s*i] = NSFastLED::ColorFromPalette(currentPalette, gAnimIndex, MAX_BRIGHTNESS, currentBlending);
      gAnimIndex += 255/(NUM_LEDS_PER_STRIP*NUM_STRIPS);
    }
  }
}

void pattern_brake_light() {
  // flicker brightness?
  uint8_t cBrightness = 255; //quadwave8((millis()/100)%256);
  // 0 is red hue
  CHSV hsv_led = CHSV(0, 255, cBrightness);
  CRGB rgb_led;
  hsv2rgb_rainbow(hsv_led, rgb_led);
  for (int i = 0; i < NUM_STRIPS*NUM_LEDS_PER_STRIP; ++i) {
    leds[i] = rgb_led;
  }
}

// determines if the global state of accelerometer xyz values
// indicate we are agressively braking
bool accelIsBraking() {
  //if (accel_now.x)
  //TODO(gabe) figure out thresholds and directions
  return false;
}


void loop() {
  t_now = millis();
  if (t_now - lastPrintSample >= 100) {
    lastPrintSample = t_now;
    if (accel.getSample(accel_now)) {
      Serial.printlnf("acc: %d,%d,%d", accel_now.x, accel_now.y, accel_now.z);
    }
    // handle identifying braking
    bool brakingDetected = accelIsBraking();
    if (!braking && brakingDetected) {
      braking = true;
      t_brake = t_now;
    }
    if (braking && !brakingDetected) {
      braking = false;
      t_brake_end = t_now;
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
    t_pattern_start = t_now;
    Serial.printlnf("pattern->%d", gPattern);
  }

  // increment palette every PALETTE_CHANGE_INTERVAL_S
  if (AUTO_CHANGE_PALETTE && (t_now > t_palette_start+PALETTE_CHANGE_INTERVAL_S*1000)) {
    switch(gPalette) {
      case 0: currentPalette = RainbowColors_p;  currentBlending = LINEARBLEND; break;
      case 1: currentPalette = PartyColors_p;    currentBlending = LINEARBLEND; break;
      case 2: currentPalette = CloudColors_p;    currentBlending = LINEARBLEND; break;
      case 3: currentPalette = ForestColors_p;   currentBlending = LINEARBLEND; break;
      case 4: currentPalette = OceanColors_p;    currentBlending = LINEARBLEND; break;
      case 5: currentPalette = LavaColors_p;     currentBlending = LINEARBLEND; break;
      default:
      gPalette = 0;
      currentPalette = NSFastLED::RainbowColors_p; currentBlending = NSFastLED::LINEARBLEND; break;
    }
    t_palette_start = t_now;
  }

  if (accel_lastPos == ACCEL_POSITION_UPSIDEDOWN) {
    // pause pattern
    pattern_flipped_over();
  } else if ( braking || !braking && (t_brake_end+BRAKE_HOLD_MS < t_now)) {
    pattern_brake_light();
  } else {
    switch(gPattern) {
      case 0: pattern_from_palette();
      break;
      default:
      gPattern = 0;
      break;
    }
  }

  NSFastLED::FastLED.show();
  NSFastLED::FastLED.delay(1000 / UPDATES_PER_SECOND);
}
