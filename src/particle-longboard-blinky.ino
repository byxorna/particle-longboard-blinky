/*
* Project particle-longboard-blinky
* Description: longboard LEDs
* Author: Gabe Conradi
* Date: idklol
*/

#include "Particle.h"
#include "FastLED.h"
#include "LIS3DH.h"

FASTLED_USING_NAMESPACE;
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
#define BOOTUP_ANIM_DURATION_MS 4000

#define PATTERN_CHANGE_INTERVAL_MS 15000
#define PALETTE_CHANGE_INTERVAL_MS 15000
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

uint8_t gBrightness = 10; // global brightness
uint8_t gPattern = 0; // global pattern
uint8_t gPalette = 0; // global palette
uint8_t gAnimIndex = 0; // animation index for ColorFromPalette
CFastLED* gLED; // global CFastLED object

unsigned long lastPrintSample = 0;
unsigned long t_now;                // time now in each loop iteration
unsigned long t_boot;               // time at bootup
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
  gLED->addLeds<LED_TYPE, D4>(leds, NUM_LEDS_PER_STRIP);
  gLED->addLeds<LED_TYPE, D5>(leds + NUM_LEDS_PER_STRIP, NUM_LEDS_PER_STRIP);
  gLED->setBrightness(gBrightness);

  // reset pattern
  gPattern = 0;
  gPalette = 0;

  t_boot = millis();
}

// pattern to display when we are flipped upside down
void pattern_flipped_over() {
  // pick a color, and just pulse it slowly
  // 5000ms per breath period
  uint8_t cBrightness = beatsin8(12, 0, 255);
  uint8_t cHue = beatsin8(60/30, 0, 255); // cycle colors every 30s
  CHSV hsv_led = CHSV(cHue, 255, cBrightness);
  CRGB rgb_led;
  hsv2rgb_rainbow(hsv_led, rgb_led);
  for( int s = 0; s < NUM_STRIPS; s++) {
    for( int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      leds[s*i] = rgb_led;
    }
  }
}

void pattern_cylon_eye() {
  // cylon eye is 3 pixels wide, +/++ base index
  // we map a 60bpm(1s) cycle into 0..num leds-1
  uint8_t mappedIndex = beatsin8(60, 0, NUM_LEDS_PER_STRIP*NUM_STRIPS-1);
  for(int i = 0; i < NUM_LEDS_PER_STRIP*NUM_STRIPS; ++i) {
    if (mappedIndex == i) {
      leds[i] = CRGB::Red;
    } else if (addmod8(mappedIndex, 1, 255) == i) {
      leds[i] = CRGB::Red;
    } else if (addmod8(mappedIndex, 2, 255) == i) {
      leds[i] = CRGB::Red;
    } else {
      leds[i] = CRGB::Black;
    }
  }
}

void pattern_bootup() {
  uint8_t baseHue = beatsin8(60, 0, 255);
  uint8_t iHue = 0;
  for(int i = 0; i < NUM_LEDS_PER_STRIP*NUM_STRIPS; ++i) {
    iHue = addmod8(baseHue, 1, 255);
    CHSV hsv_led = CHSV(iHue, 255, 255);
    CRGB rgb_led;
    hsv2rgb_rainbow(hsv_led, rgb_led);
    leds[i] = rgb_led;
  }
}

void pattern_from_palette() {
  for( int s = 0; s < NUM_STRIPS; s++) {
    for( int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      leds[s*i] = ColorFromPalette(currentPalette, gAnimIndex, MAX_BRIGHTNESS, currentBlending);
      gAnimIndex += 255/(NUM_LEDS_PER_STRIP*NUM_STRIPS);
    }
  }
}

void pattern_brake_light() {
  for (int i = 0; i < NUM_STRIPS*NUM_LEDS_PER_STRIP; ++i) {
    leds[i] = CRGB::Red;
  }
}

// determines if the global state of accelerometer xyz values
// indicate we are agressively braking
bool accelIsBraking() {
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
      Serial.printlnf("brake engaged: %d", t_now);
      braking = true;
      t_brake = t_now;
    }
    if (braking && !brakingDetected) {
      Serial.printlnf("brake released: %d", t_now);
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

  // increment pattern every PATTERN_CHANGE_INTERVAL_MS
  if (AUTO_CHANGE_PATTERNS && (t_now > t_pattern_start+PATTERN_CHANGE_INTERVAL_MS)) {
    gPattern++;
    t_pattern_start = t_now;
    Serial.printlnf("pattern->%d", gPattern);
  }

  // increment palette every PALETTE_CHANGE_INTERVAL_MS
  if (AUTO_CHANGE_PALETTE && (t_now > t_palette_start+PALETTE_CHANGE_INTERVAL_MS)) {
    switch(gPalette) {
      case 0: currentPalette = RainbowColors_p;  currentBlending = LINEARBLEND; break;
      case 1: currentPalette = PartyColors_p;    currentBlending = LINEARBLEND; break;
      case 2: currentPalette = CloudColors_p;    currentBlending = LINEARBLEND; break;
      case 3: currentPalette = ForestColors_p;   currentBlending = LINEARBLEND; break;
      case 4: currentPalette = OceanColors_p;    currentBlending = LINEARBLEND; break;
      case 5: currentPalette = LavaColors_p;     currentBlending = LINEARBLEND; break;
      default:
      gPalette = 0;
      currentPalette = RainbowColors_p; currentBlending = LINEARBLEND; break;
    }
    t_palette_start = t_now;
  }

  if (t_boot + BOOTUP_ANIM_DURATION_MS < t_now) {
    // display a bootup pattern for a bit
    pattern_bootup();
  } else if (accel_lastPos == ACCEL_POSITION_UPSIDEDOWN) {
    // pause pattern
    pattern_flipped_over();
  } else if (braking || (!braking && (t_brake_end+BRAKE_HOLD_MS < t_now))) {
    pattern_brake_light();
  } else {
    switch(gPattern) {
      case 0: pattern_from_palette();
      break;
      case 1: pattern_cylon_eye();
      default:
      gPattern = 0;
      break;
    }
  }

  gLED->show();
  delay(1000 / UPDATES_PER_SECOND);
  //gLED->delay(1000 / UPDATES_PER_SECOND);
}
