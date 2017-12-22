/*
* Project particle-longboard-blinky
* Description: longboard LEDs
* Author: Gabe Conradi
* Date: idklol
*/

/*
  Photon pins:
  WKP -> INT on LIS3DH
  GND->gnd
  VIN->+5v
  D0->SDA LIS3DH
  D1->SCL LIS3DH
*/

typedef void (*FP)();

#include "Particle.h"
#include "FastLED.h"
#include "LIS3DH.h"
#include "RunningAverage.h"

FASTLED_USING_NAMESPACE;
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

// the pots i have have an odd plateau at 3095, so to make things easier
// lets just assume this is the upper bound
#define ANALOG_POT_UPPER_BOUND 3095
#define ANALOG_POT_LOWER_BOUND 300

#define BRIGHTNESS_POT_PIN A1
#define MODE_POT_PIN A2
#define AUTO_PATTERN_PIN A3
#define CLOCK_PIN D4
#define NUM_LEDS_PER_STRIP 37
#define NUM_STRIPS 2

#define LED_TYPE NEOPIXEL
#define INDEX_RIGHT 1
#define INDEX_LEFT 0

#define UPDATES_PER_SECOND 100
#define MAX_BRIGHTNESS 255
#define MAX_SATURATION 255
#define BOOTUP_ANIM_DURATION_MS 2000

#define PATTERN_CHANGE_INTERVAL_MS 15000
#define PALETTE_CHANGE_INTERVAL_MS 15000
#define AUTO_CHANGE_PALETTE 1

// accelerometer stuff
#define ACCEL_POSITION_NORMAL 5
#define ACCEL_POSITION_UPSIDEDOWN 4
#define ACCEL_POSITION_A 3
#define ACCEL_POSITION_B 2
#define ACCEL_POSITION_C 1
#define ACCEL_POLL_INTERVAL_MS 100
#define BRAKING_SAMPLE_WINDOW 5

void accel_positionInterruptHandler();
LIS3DHSample accel_now; // accelerometer value last sampled
LIS3DHSample accel_prev;
// Connect the Adafruit LIS3DH breakout
// https://www.adafruit.com/products/2809
// VIN: 3V3
// GND: GND
// SCL: Connect to D1 (I2C SCL)
// SDA: Connect to D0 (I2C SDA)
// INT: WKP
//LIS3DHI2C* accel = new LIS3DHI2C(0, WKP);
LIS3DHI2C accel(Wire, 0, WKP);
volatile bool accel_positionInterrupt = false;
uint8_t accel_lastPos = 0;
RunningAverage xAccelAvg(BRAKING_SAMPLE_WINDOW);  // running average over 5*100ms polling windows

uint8_t gBrightness; // global brightness, read from potentiometer
uint8_t gPattern; // global pattern
bool autoPatternChange;

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
unsigned long t_brake = 0;          // time braking started
unsigned long t_brake_end = 0;      // time braking ended
// when we activate the brakes, hold the brake light for X ms
// after we decide we arent braking anymore
#define BRAKE_HOLD_MS 3000

/* custom color palettes */
// orange 255,102,0 FF6600
// pink 255,0,255 #ff00ff
// pornj 255,51,51 #ff3333
DEFINE_GRADIENT_PALETTE( Disorient_gp ) {
      0,   0,   0,   0,    // black
     75, 255,  26, 153,    // pink
    147, 255,  51,  51,    // pornj
    208, 255, 111,  15,    // orange
    255, 255, 255, 255, }; // white

// for effects that are palette based
CRGBPalette16 currentPalette; // current color palette
CRGBPalette16 palettes[6] = {
  Disorient_gp,
  RainbowColors_p,
  CloudColors_p,
  ForestColors_p,
  OceanColors_p,
  LavaColors_p,
};

TBlendType currentBlending = LINEARBLEND;
CRGB leds[NUM_STRIPS*NUM_LEDS_PER_STRIP]; //[NUM_STRIPS][NUM_LEDS_PER_STRIP];

void accel_positionInterruptHandler() {
  Serial.println("accelerometer position changed");
	accel_positionInterrupt = true;
}

// reads an analog pin and returns a clean value between 0-255 inclusive
uint8_t readPotValue(int pin, int low_threshold, int high_threshold) {
  int raw = analogRead(pin);
  // first constrain the values so we have a stable floor and ceiling
  int constrained = constrain(raw, low_threshold, high_threshold);
  uint8_t mapped = map(constrained, low_threshold, high_threshold, 0, 255);
  Serial.printlnf("pot %d, %d, %d", raw, constrained, mapped);
  return mapped;
}

// reads intended brightness level from potentiometer
// and maps it into an acceptable brightness value
uint8_t readBrightnessFromPot() {
  int raw = readPotValue(BRIGHTNESS_POT_PIN, ANALOG_POT_LOWER_BOUND, ANALOG_POT_UPPER_BOUND);
  if (raw < 10) {
    raw = 10; // set a minimum brightness
  }
  return raw;
}

bool readAutoPatternChange() {
  int raw = readPotValue(AUTO_PATTERN_PIN, 250, 2000);
  return (bool)constrain(map(raw, 0, 255, 0, 3), 0, 1);
}

// setup() runs once, when the device is first turned on.
void setup() {

  Serial.begin(9600);
  Serial.println("resetting");
  xAccelAvg.clear();

  // chill for a sec
  // delay( 4000 );
  Serial.println("setting up accelerometer");

  // initialize i2c wire; photon has only 1 wire bus WKP (D0,D1)
  Wire.setSpeed(CLOCK_SPEED_100KHZ);
  Wire.begin();

  // Initialize sensors
  LIS3DHConfig config;
  config.setAccelMode(LIS3DH::RATE_100_HZ);
  attachInterrupt(WKP, accel_positionInterruptHandler, RISING);
  config.setPositionInterrupt(16);
  bool setupSuccess = accel.setup(config);
  Serial.printlnf("accelerometer setup: %d", setupSuccess);
  // lets Initialize the accelerometer struct
  accel.getSample(accel_now);

  currentPalette = palettes[0];
  autoPatternChange = readAutoPatternChange();
  Serial.printlnf("auto pattern changing: %d", autoPatternChange);

  // read initial values from potentiometers for brightness
  gBrightness = readBrightnessFromPot();
  Serial.printlnf("init brightness: %d", gBrightness);

  // led controller, data pin, clock pin, RGB type (RGB is already defined in particle)
  gLED = new CFastLED();
  gLED->addLeds<LED_TYPE, D6>(leds, NUM_LEDS_PER_STRIP*NUM_STRIPS);
  gLED->setBrightness(gBrightness);
  pattern_clear();
  gLED->show();

  // reset pattern from potentiometer
  gPattern = readModeFromPot();
  gPalette = 0;
  Serial.printlnf("pattern initial setting: %d", gPattern);

  t_boot = millis();
  Serial.println("booted up");
  //RGB.color(0,255,0);
}

void pattern_slow_pulse() {
  // pick a color, and pulse it 
  //uint8_t bpm = beatsin8(4, 8, 16);
  uint8_t cBrightness = beatsin8(50, 100, 255);
  uint8_t cHue = beatsin8(6, 0, 255); // cycle colors every 30s
  CHSV hsv_led = CHSV(cHue, 255, cBrightness);
  CRGB rgb_led;
  hsv2rgb_rainbow(hsv_led, rgb_led);
  for( int i = 0; i < NUM_LEDS_PER_STRIP*NUM_STRIPS; i++) {
    leds[i] = rgb_led;
  }
}

void pattern_cylon_eye() {
  // cylon eye is 4 pixels wide, +/++ base index
  // we map a 60bpm(1s) cycle into 0..num leds-1
  uint8_t h = beatsin8(8, 0, 255);
  CHSV hsv_led = CHSV(h, 255, 255);
  CRGB rgb_led;
  hsv2rgb_rainbow(hsv_led, rgb_led);
  uint8_t mappedIndex = beatsin8(60, 0, NUM_LEDS_PER_STRIP*NUM_STRIPS-1);
  for(int i = 0; i < NUM_LEDS_PER_STRIP*NUM_STRIPS; ++i) {
    if (mappedIndex == i) {
      leds[i] = rgb_led;
    } else if (addmod8(mappedIndex, 1, 255) == i) {
      leds[i] = rgb_led;
    } else if (addmod8(mappedIndex, 2, 255) == i) {
      leds[i] = rgb_led;
    } else if (addmod8(mappedIndex, 3, 255) == i) {
      leds[i] = rgb_led;
    } else {
      leds[i] = CRGB::Black;
    }
  }
}

void pattern_bootup() {
  uint8_t baseHue = beatsin8(30, 0, 255);
  uint8_t iHue = 0;
  for(int i = 0; i < NUM_LEDS_PER_STRIP*NUM_STRIPS; ++i) {
    iHue = addmod8(baseHue, 1, 255);
    CHSV hsv_led = CHSV(iHue, 255, 255);
    CRGB rgb_led;
    hsv2rgb_rainbow(hsv_led, rgb_led);
    leds[i] = rgb_led;
  }
}

// cycle a rainbow, varying how quickly it rolls around the board
void pattern_rainbow_waves() {
  for(int i = 0; i < NUM_LEDS_PER_STRIP*NUM_STRIPS; ++i) {
    uint8_t h = (t_now/8+i)%256;
    CHSV hsv_led = CHSV(h, 255, 255);
    CRGB rgb_led;
    hsv2rgb_rainbow(hsv_led, rgb_led);
    leds[i] = rgb_led;
  }
}

void pattern_clear() {
  for( int i = 0; i < NUM_LEDS_PER_STRIP*NUM_STRIPS; i++) {
    leds[i] = CRGB::Black;
  }
}
void pattern_from_palette() {
  uint8_t b = beatsin8(4, 0, 255);
  for( int i = 0; i < NUM_LEDS_PER_STRIP*NUM_STRIPS; i++) {
    leds[i] = ColorFromPalette(currentPalette, gAnimIndex + i + b, MAX_BRIGHTNESS, currentBlending);
  }
  gAnimIndex = addmod8(gAnimIndex, 1, 255);
}

void pattern_brake_light() {
  for (int i = 0; i < NUM_STRIPS*NUM_LEDS_PER_STRIP; ++i) {
    leds[i] = CRGB::Red;
  }
}

// NOTE: lifted and tweaked from https://learn.adafruit.com/rainbow-chakra-led-hoodie/the-code
// This function draws color waves with an ever-changing,
// widely-varying set of parameters, using a color palette.
void pattern_palette_waves() {
  uint8_t numleds = NUM_LEDS_PER_STRIP*NUM_STRIPS;
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  //uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 300, 1500);

  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;

  for( uint16_t i = 0 ; i < numleds; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;
    uint16_t h16_128 = hue16 >> 7;
    if( h16_128 & 0x100) {
      hue8 = 255 - (h16_128 >> 1);
    } else {
      hue8 = h16_128 >> 1;
    }

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);

    uint8_t index = hue8;
    index = scale8( index, 240);

    CRGB newcolor = ColorFromPalette(currentPalette, index, bri8);

    uint16_t pixelnumber = i;
    pixelnumber = (numleds-1) - pixelnumber;

    nblend(leds[pixelnumber], newcolor, 128);
  }
}

// determines if the global state of accelerometer xyz values
// indicate we are agressively braking
bool accelIsBraking() {
  //TODO(gabe) figure out thresholds and directions
  int avg = xAccelAvg.getAverage();
  //Serial.printlnf("accel x=%d", avg);
  return avg < -15;
}

/** update this with patterns you want to be cycled through **/
#define NUM_PATTERNS sizeof(patternBank) / sizeof(FP)
const FP patternBank[] = {
  &pattern_from_palette,
  &pattern_slow_pulse,
  &pattern_palette_waves,
  &pattern_rainbow_waves
};

// read from mode potentiometer, returning which program to run
uint8_t readModeFromPot() {
  int raw = readPotValue(MODE_POT_PIN, ANALOG_POT_LOWER_BOUND, ANALOG_POT_UPPER_BOUND);
  return map(raw, 0, 255, 0, NUM_PATTERNS-1);
}



void loop() {
  t_now = millis();

  // update brightness values
  gBrightness = readBrightnessFromPot();
  Serial.printlnf("brightness: %d", gBrightness);
  autoPatternChange = readAutoPatternChange();

  // get a sample from accelerometer
  if (t_now - lastPrintSample >= ACCEL_POLL_INTERVAL_MS) {
    lastPrintSample = t_now;
    accel_prev = accel_now;
    if (accel.getSample(accel_now)) {
      //Serial.printlnf("acc: %d,%d,%d", accel_now.x, accel_now.y, accel_now.z);
      int dx = (accel_prev.x - accel_now.x)/ACCEL_POLL_INTERVAL_MS;
      xAccelAvg.addValue(dx);
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
  if (autoPatternChange) {
    if (t_now > t_pattern_start+PATTERN_CHANGE_INTERVAL_MS) {
      gPattern++;
      t_pattern_start = t_now;
      Serial.printlnf("auto pattern->%d", gPattern);
    }
  } else {
    gPattern = readModeFromPot();
  }

  // increment palette every PALETTE_CHANGE_INTERVAL_MS
  if (AUTO_CHANGE_PALETTE && (t_now > t_palette_start+PALETTE_CHANGE_INTERVAL_MS)) {
    gPalette++;
    if (gPalette >= (sizeof(palettes)/sizeof(*palettes))) {
      gPalette = 0;
    }
    currentPalette = palettes[gPalette];
    Serial.printlnf("palette->%d", gPalette);
    t_palette_start = t_now;
  }

  if (t_boot + BOOTUP_ANIM_DURATION_MS > t_now) {
    // display a bootup pattern for a bit
    pattern_bootup();
  } else if (accel_lastPos == ACCEL_POSITION_NORMAL) {
    // pause pattern, cause we are actually upside down!
    pattern_cylon_eye();
  } else if (braking || (!braking && (t_brake_end+BRAKE_HOLD_MS > t_now))) {
    pattern_brake_light();
  } else {
    if (gPattern < NUM_PATTERNS) {
      patternBank[gPattern]();
    } else {
      gPattern = 0;
    }
  }

  gLED->setBrightness(gBrightness);
  gLED->show();
  delay(1000 / UPDATES_PER_SECOND);
  //gLED->delay(1000 / UPDATES_PER_SECOND);
}
