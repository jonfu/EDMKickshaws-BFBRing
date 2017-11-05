// Googly Eye Goggles
// By Bill Earl
// For Adafruit Industries
//
// The googly eye effect is based on a physical model of a pendulum.
// The pendulum motion is driven by accelerations in 2 axis.
// Eye color varies with orientation of the magnetometer

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include "LowPower.h"

#define ORIENTATION_OFFSET  5
const float magInitPos = 0.0;

const float circleLength = 16.0;


#define SPINNYCOLORRANDOM false
#define SPINNYDURATION 2000
#define SPINNYINTERVAL 20
#define DEBUG false
#define TRACE false

#define neoPixelPin 8

#define totalPixels 16
#define circlePixels 16

#define VCC1 10
#define VCC2 6

#define GND 3

#define BRI 30
#define DROPBRI 70
#define COMPASSMAX 10
#define COMPASSINTERVAL 50
#define SOUTHANISTEPS 12

#define DROPCYCLE 7
#define BLINK 6
#define FALLCOUNT 38

#define DROPDELAY 120 //orig 300
#define DROPDELAYDECREASE 100 //orig 250
#define FASTDROPCOUNT 75 //orig 160

int dropDelay = DROPDELAY;
int dropDelayDecrease = DROPDELAYDECREASE;
byte fastDropCount = FASTDROPCOUNT;

long counter = 0;
int faultyCounter = 10;

const byte leftFall[FALLCOUNT] = { 59, 59, 59, 59, 59, 59,
                                   60, 61, 62, 63, 48, 49, 50, 51,
                                   42, 41, 40, 39, 38, 37, 36, 35,
                                   28, 29, 30, 31, 16, 17, 18, 19,
                                   10,  9,  8,  7,  6,  5,  4,  3
                                 };

const byte rightFall[FALLCOUNT] = { 59, 59, 59, 59, 59, 59,
                                    58, 57, 56, 55, 54, 53, 52, 51,
                                    44, 45, 46, 47, 32, 33, 34, 35,
                                    26, 25, 24, 23, 22, 21, 20, 19,
                                    12, 13, 14, 15,  0,  1,  2,  3
                                  };

const byte southCirclePos[SOUTHANISTEPS] = { 0, 1, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3 };
const byte southCircleDim[SOUTHANISTEPS] = { 4, 3, 2, 1, 1, 2, 3, 4, 5, 10, 20, 30 };

int dropCount[DROPCYCLE];
byte dropWheel[DROPCYCLE];

byte colorWheel = 0;



// We could do this as 2 16-pixel rings wired in parallel.
// But keeping them separate lets us do the right and left
// eyes separately if we want.
Adafruit_NeoPixel strip = Adafruit_NeoPixel(totalPixels, neoPixelPin, NEO_GRB + NEO_KHZ800);

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

float pos = 5;  // Starting center position of pupil
float increment = 2 * 3.14159 / circleLength; // distance between pixels in radians
float MomentumH = 0; // horizontal component of pupil rotational inertia
float MomentumV = 0; // vertical component of pupil rotational inertia

// Tuning constants. (a.k.a. "Fudge Factors)
// These can be tweaked to adjust the liveliness and sensitivity of the eyes.
const float friction = 0.996; // frictional damping constant.  1.0 is no friction.
const float swing = 60;  // arbitrary divisor for gravitational force
const float gravity = 200;  // arbitrary divisor for lateral acceleration
const float nod = 13.0; // accelerometer threshold for toggling modes

long nodStart = 0;
long nodTime = 2000;
long nodThreshold = 800;
long nodStartPosX = 0;
long nodStartPosZ = 0;
long nodStartNegX = 0;
long nodStartNegZ = 0;
int nodCountPosX = 0;
int nodCountPosZ = 0;
int nodCountNegX = 0;
int nodCountNegZ = 0;
int nodActivate = 8;

bool antiGravity = false;  // The pendulum will anti-gravitate to the top.
bool mirroredEyes = false; // The left eye will mirror the right.
bool compass = false;
bool lastStraightUp = false;

const float halfWidth = 1.25; // half-width of pupil (in pixels)

// Pi for calculations - not the raspberry type
const float Pi = 3.14159;

byte mode;

byte compassLevel = 2;
bool compassUp = true;

unsigned long printTime = 0;
unsigned long compassTime = 0;

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void)
{

  pinMode(GND, OUTPUT);
  digitalWrite(GND, LOW);

  pinMode(VCC1, OUTPUT);
  digitalWrite(VCC1, HIGH);

  pinMode(VCC2, OUTPUT);
  digitalWrite(VCC2, HIGH);

  for (int i = 0; i < DROPCYCLE; i++) {
    dropCount[i] = -1;
  }

  strip.begin();
  strip.setBrightness(BRI); // Lower brightness and save eyeballs!
  strip.show(); // Initialize all pixels to 'off'  sensor_t sensor;

  // Initialize the sensors
  accel.begin();
  mag.begin();



  if (DEBUG) {

    Serial.begin(115200);
    printTime = millis();
    /* Display some basic information on this sensor */
    displaySensorDetails();

  }





  compass = false;

  mode = EEPROM.read(0);

  mode++;
  if (mode > 1) {
    mode = 0;
  }

  if (DEBUG) {
    Serial.print("mode is ");
    Serial.println(mode);

  }

  setMode();

  EEPROM.write(0, mode);

}

// main processing loop
void loop(void)
{

  if (TRACE) {
    Serial.println("begin loop");
  }

  if (mode == 1) {

    if (DEBUG) {
      Serial.println("Software Power OFF");
      delay(1000);
    }

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    return;
  }



  strip.setBrightness(BRI);

  sensors_event_t event;
  // Read the magnetometer and determine the compass heading:
  if (TRACE) {
    Serial.println("Before mag.getEvent");
  }

  //only making this call because I think I got a faulty chip
  faultyCounter--;
  if (faultyCounter < 0) {
    faultyCounter = 10;
    mag.begin();
  }
  
  mag.getEvent(&event);
  
  if (TRACE) {
    Serial.println("After mag.getEvent");
  }

  // Calculate the angle of the vector x.y from magnetic North
  float heading = (atan2(event.magnetic.x, event.magnetic.y) * 180) / Pi;

  if (TRACE) {
     Serial.print("MAGNATIC "); Serial.print("\t");
      Serial.print(event.magnetic.x); Serial.print("\t");
      Serial.println (event.magnetic.y);
    
  }

  // Normalize to 0-360 for a compass heading
  if (heading < 0)
  {
    heading = 360 + heading;
  }

  // Now read the accelerometer to control the motion.
  accel.getEvent(&event);

  // Check for mode change commands
  CheckForNods(event);
  //CheckForNodsOriginal(event);

  if (DEBUG) {
    if ( (millis() - printTime) > 200) {

      printTime = millis();

      Serial.print(event.acceleration.x); Serial.print("\t");
      Serial.print(event.acceleration.y); Serial.print("\t");
      Serial.print(event.acceleration.z); Serial.print("\t");
      Serial.println(heading); 

    }
  }



  if ( event.acceleration.z > 0.9 && event.acceleration.y < 3.0 && event.acceleration.x < 3.0 ) {

    pos = magInitPos - (heading * circleLength / 360);
    while (round(pos) < 0) {
      pos += circleLength;
    }


    /*

      if (pos > 10.9 && pos < 12.1) {

        if (millis() - compassTime > COMPASSINTERVAL) {
          compassTime = millis();
          for (int i = 0; i < totalPixels; i++) {
            strip.setPixelColor(i, 0);
          }
          for (int i = 0+circlePixels*southCirclePos[counter]; i < circlePixels+circlePixels*southCirclePos[counter]; i++) {
            strip.setPixelColor(i, strip.Color(255/southCircleDim[counter],255/southCircleDim[counter],255/southCircleDim[counter]));
          }
          counter++;
          if (counter >= SOUTHANISTEPS) {
            counter = 0;
          }
        }
      }
      else {
    */

    counter = 0;

    for (int i = 0; i < circlePixels; i++) {
      strip.setPixelColor(i, selectColorDim(heading, compassLevel));
    }

    for (int i = circlePixels; i < totalPixels; i++) {
      strip.setPixelColor(i, 0);
    }

    if (millis() - compassTime > COMPASSINTERVAL) {

      compassTime = millis();

      if (compassUp) {
        compassLevel++;
      }
      else {
        compassLevel--;
      }

      if (compassLevel > COMPASSMAX) {
        compassUp = false;
        compassLevel = COMPASSMAX;
      }
      if (compassLevel < 2) {
        compassUp = true;
        compassLevel = 2;
      }

    }
    strip.setPixelColor(pos, strip.Color(255, 255, 255));
    //////}
  }

  else
  {

    // apply a little frictional damping to keep things in control and prevent perpetual motion
    MomentumH *= friction;
    MomentumV *= friction;

    // Calculate the horizontal and vertical effect on the virtual pendulum
    // 'pos' is a pixel address, so we multiply by 'increment' to get radians.
    float TorqueH = cos(pos * increment);  // peaks at top and bottom of the swing
    float TorqueV = sin(pos * increment);    // peaks when the pendulum is horizontal

    // X-axis pointed forward.  Y-axis pointed upward, and Z-axis reads horizontal acceleration

    MomentumH += TorqueH * event.acceleration.z / swing;
    //if (antiGravity)
    //{
    //  MomentumV += TorqueV * event.acceleration.x / gravity;
    //}
    //else
    {
      MomentumV -= TorqueV * event.acceleration.x / gravity;
    }

    // Calculate the new position
    pos += MomentumH + MomentumV;

    // handle the wrap-arounds at the top
    while (round(pos) < 0) pos += circleLength;
    while (round(pos) > (circlePixels - 1)) pos -= circleLength;

    // Now re-compute the display
    for (int i = 0; i < circlePixels; i++)
    {
      // Compute the distance bewteen the pixel and the center
      // point of the virtual pendulum.
      float diff = i - pos;

      //for some reason with this orientation of the sensor, we need to offset by 8
      int j = i - ORIENTATION_OFFSET;
      if (j < 0) {
        j += circlePixels;
      }

      // Light up nearby pixels proportional to their proximity to 'pos'
      if (fabs(diff) <= halfWidth)
      {
        uint32_t color;
        float proximity = halfWidth - fabs(diff) * 200;

        // pick a color based on heading & proximity to 'pos'
        color = selectColor(heading, proximity);



        // do both eyes
        strip.setPixelColor(j, color);

      }
      else // all others are off
      {
        strip.setPixelColor(j, 0);

      }
    }

  } //END OF else

  // Now show it!
  strip.show();
}

// choose a color based on the compass heading and proximity to "pos".
uint32_t selectColor(float heading, float proximity)
{
  uint32_t color;

  // Choose eye color based on the compass heading
  if (heading < 60)
  {
    color = strip.Color(0, 0, proximity);
  }
  else if (heading < 120)
  {
    color = strip.Color(0, proximity, proximity);
  }
  else if (heading < 180)
  {
    color = strip.Color(0, proximity, 0);
  }
  else if (heading < 240)
  {
    color = strip.Color(proximity, proximity, 0);
  }
  else if (heading < 300)
  {
    color = strip.Color(proximity, 0, 0);
  }
  else // 300-360
  {
    color = strip.Color(proximity, 0, proximity);
  }
}

// choose a color based on the compass heading and proximity to "pos".
uint32_t selectColorDim(float heading, float dimLevel)
{
  uint32_t color;
  float proximity = 200;

  // Choose eye color based on the compass heading
  if (heading < 60)
  {
    color = strip.Color(0, 0, proximity / dimLevel);
  }
  else if (heading < 120)
  {
    color = strip.Color(0, proximity / dimLevel, proximity / dimLevel);
  }
  else if (heading < 180)
  {
    color = strip.Color(0, proximity / dimLevel, 0);
  }
  else if (heading < 240)
  {
    color = strip.Color(proximity / dimLevel, proximity / dimLevel, 0);
  }
  else if (heading < 300)
  {
    color = strip.Color(proximity / dimLevel, 0, 0);
  }
  else // 300-360
  {
    color = strip.Color(proximity / dimLevel, 0, proximity / dimLevel);
  }
}

// monitor orientation for mode-change 'gestures'
void CheckForNodsOriginal(sensors_event_t event)
{
  if (event.acceleration.x > nod)
  {
    if (millis() - nodStart > nodTime)
    {
      antiGravity = false;
      nodStart = millis(); // reset timer
      spinDown();
      //spinnyWheel(SPINNYDURATION, SPINNYINTERVAL, SPINNYCOLORRANDOM);
    }
  }
  else if (event.acceleration.x < -(nod + 1))
  {
    if (millis() - nodStart > nodTime)
    {
      antiGravity = true;
      spinUp();
      //spinnyWheel(SPINNYDURATION, SPINNYINTERVAL, SPINNYCOLORRANDOM);
      nodStart = millis(); // reset timer
    }
  }
  else if (event.acceleration.z > nod)
  {
    if (millis() - nodStart > nodTime)
    {
      mirroredEyes = false;
      //spinDown();
      spinnyWheel(SPINNYDURATION, SPINNYINTERVAL, SPINNYCOLORRANDOM);
      nodStart = millis(); // reset timer
    }
  }
  else if (event.acceleration.z < -nod)
  {
    if (millis() - nodStart > nodTime)
    {
      mirroredEyes = true;
      //spinUp();
      spinnyWheel(SPINNYDURATION, SPINNYINTERVAL, SPINNYCOLORRANDOM);
      nodStart = millis(); // reset timer
    }
  }
  else // no nods in progress
  {
    nodStart = millis(); // reset timer
  }
}


// monitor orientation for mode-change 'gestures'
void CheckForNods(sensors_event_t event)
{
  if (event.acceleration.x > nod)
  {
    if (nodCountPosX == 0) {
      nodStartPosX = millis();
    }

    if ( (millis() - nodStartPosX) < nodThreshold)
    {

      if (DEBUG) {
        Serial.println("nodThreshold acceleration.x");
      }
      
      nodCountPosX++;
      if (nodCountPosX == nodActivate) {

      if (DEBUG) {
        Serial.println("nodCountPosX == nodActivate");
      }
        
        resetNods();
        //spinDown();
        spinnyWheel(SPINNYDURATION, SPINNYINTERVAL, SPINNYCOLORRANDOM);
      }
    }

  }
  else if (event.acceleration.x < -(nod + 1))
  {

    if (nodCountNegX == 0) {
      nodStartNegX = millis();
    }

    if ( (millis() - nodStartNegX) < nodThreshold)
    {

      if (DEBUG) {
        Serial.println("nodThreshold acceleration.x NEG");
      }
      
      nodCountNegX++;
      if (nodCountNegX == nodActivate) {

      if (DEBUG) {
        Serial.println("nodCountNegX == nodActivate NEG");
      }
        
        resetNods();
        //spinUp();
        spinnyWheel(SPINNYDURATION, SPINNYINTERVAL, SPINNYCOLORRANDOM);
      }
    }


  }
  else if (event.acceleration.z > nod)
  {
    if (nodCountPosZ == 0) {
      nodStartPosZ = millis();
    }

    if ( (millis() - nodStartPosZ) < nodThreshold)
    {

      if (DEBUG) {
        Serial.println("nodThreshold acceleration.z");
      }
      
      nodCountPosZ++;
      if (nodCountPosZ == nodActivate) {

      if (DEBUG) {
        Serial.println("nodCountPosZ == nodActivate");
      }
        
        resetNods();
        spinUp();
        //spinDown();
        //spinnyWheel(SPINNYDURATION, SPINNYINTERVAL, SPINNYCOLORRANDOM);
      }
    }

  }
  else if (event.acceleration.z < -nod)
  {

    if (nodCountNegZ == 0) {
      nodStartNegZ = millis();
    }

    if ( (millis() - nodStartNegZ) < nodThreshold)
    {

      if (DEBUG) {
        Serial.println("nodThreshold acceleration.z NEG");
      }
      
      nodCountNegZ++;
      if (nodCountNegZ == nodActivate) {

      if (DEBUG) {
        Serial.println("nodCountNegZ == nodActivate NEG");
      }
        
        resetNods();
        spinUp();
        //spinnyWheel(SPINNYDURATION, SPINNYINTERVAL, SPINNYCOLORRANDOM);
      }
    }

  }
  else // no nods in progress
  {
    resetNods();
  }
}

void resetNods() {
  nodStartPosX = millis(); // reset timer
  nodStartPosZ = millis(); // reset timer
  nodStartNegX = millis(); // reset timer
  nodStartNegZ = millis(); // reset timer
  nodCountPosX = 0;
  nodCountPosZ = 0;
  nodCountNegX = 0;
  nodCountNegZ = 0;
}

void setMode() {


  switch (mode) {
    case 0:


      if (DEBUG) {
        Serial.println("spin to start");
      }
      /// spin-up
      //spin(strip.Color(255, 0, 0), 1, 100);
      //spin(strip.Color(0, 255, 0), 1, 100);
      //spin(strip.Color(0, 0, 255), 1, 100);
      //spinUp();
      //spinDown();
      spinUpDown();
      break;



    /*

      case 2:
      mirroredEyes = true;
      spinDown();
      break;

      case 3:
      mirroredEyes = true;
      antiGravity = true;
      break;

    */

    case 1:
      if (DEBUG) {
        Serial.println("No spin");
      }
      break;

    default:
      if (DEBUG) {
        Serial.println("Error: shouldn't be here");
      }
  }

}


void spinUpDown()
{
  
  float maxRGB = 255.0;
  
   int j = 0;
   
   for (int i = 300; i > 10;  i -= 30)
   {
     spin(strip.Color(maxRGB,maxRGB,maxRGB), 1, i);
     maxRGB *= 0.96;
     j=i;
   }
   spin(strip.Color(maxRGB,maxRGB,maxRGB), 7, j);
   /*

   for (int i = 1; i < 300; i+=20)
   {
     spin(strip.Color(maxRGB,maxRGB,maxRGB), 1, i);
     maxRGB *= 0.9;
   }
   */
   
   pos = 0;
   //stop it dead at the top and let it swing to the bottom on its own
   MomentumH = 4;
   MomentumV = 0;
}



// gradual spin up
void spinUp()
{
  for (int i = 60; i > 0;  i -= 4)
  {
    spin(strip.Color(255, 255, 255), 1, i);
  }
  pos = 0;
  // leave it with some momentum and let it 'coast' to a stop
  MomentumH = 3;
}

// Gradual spin down
void spinDown()
{
  for (int i = 1; i < 60; i++)
  {
    spin(strip.Color(255, 255, 255), 1, i += 4);
  }
  // Stop it dead at the top and let it swing to the bottom on its own
  pos = 0;
  MomentumH = MomentumV = 0;
}


// utility function for feedback on mode changes.
void spin(uint32_t color, int count, int time)
{
  for (int j = 0; j < count; j++)
  {
    for (int i = 0; i < circlePixels; i++)
    {
      strip.setPixelColor(i, color);

      strip.show();
      delay(max(time / circlePixels, 1));
      strip.setPixelColor(i, 0);

      strip.show();
    }
  }
}

//default is 8000 and 50, consider 4000 and 25
void spinnyWheel(uint32_t spinDur, int spinDelay, boolean randomColor) {

  uint32_t spinnyStartTime = millis();

  uint32_t color  = 0xFF69B4; // Starting color = hot pink

  if (randomColor) {
    color = Wheel(random(255), 1);
  }

  uint8_t  offset = 0;

  while ( (millis() - spinnyStartTime) < spinDur)  {

    for (int i = 0; i < circlePixels; i++) { // For each LED...
      uint32_t c = 0;              // Assume pixel will be "off" color
      if (((offset + i) & 7) < 2) { // For each 8 pixels, 2 will be...
        c = color;                 // ...assigned the current color
      }
      strip.setPixelColor(i, c);  // Set color of pixel 'i'
    }
    strip.show();                 // Refresh LED states
    delay(spinDelay);
    offset++;                      // Shift animation by 1 pixel on next frame


    //color >>= 8;                 // And change color
    //if(!color) color = 0xFF8000; // preiodically reset to amber

  }

}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, byte dimFactor)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return strip.Color((255 - WheelPos * 3) / dimFactor, 0, WheelPos * 3 / dimFactor);
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3 / dimFactor, (255 - WheelPos * 3) / dimFactor);
  }
  else
  {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3 / dimFactor, (255 - WheelPos * 3) / dimFactor, 0);
  }
}

byte getDimFactor(byte index) {

  if (index > (FALLCOUNT - 8)) {
    return 10 + (index - (FALLCOUNT - 8)) * 3;
  }

  switch (index) {
    /*
      case 0:
      return 4;
      case 1:
      return 2;
      case 2:
      return 1;
      case 3:
      return 5;
    */

    case 0:
      return 100;
    case 1:
      return 4;
    case 2:
      return 2;
    case 3:
      return 1;
    case 4:
      return 3;
    case 5:
      return 5;



    default:
      return 10;
  }

}
