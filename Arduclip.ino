
/*
 * Uses an Arduino Duemilanove clone
 * 
 */
/*
 * TODO
 * Change red led to green(Add green led) - OK
 * Battery monitor - OK
 * Check signals from cutter motor hall sensor
 * Control cutter motor
 * Add on/off switch
 * Front distance sensor HC-SR04 - OK
 * Back distance sensor HC-SR04 - ?
 */

/*
 * Battery monitor
 * Max V = 30V
 * Battery + --- 11k(10k+1k) --- sense --- 2.2k --- Ground
 * Gives 5 volt @sense @ 30V on battery
 */

// We set the Arduino to sleep if battery is low
#include <avr/sleep.h> 

// Connections for wheel motor drivers
int dirL = 7;
int pwmL = 5;
int enL = 9;
int dirR = 8;
int pwmR = 6;
int enR = 10;

// Indicator leds
int ylwLed=11;
int battLed=12;   // Red led for battery status
int statusLed=13; // Green led mounted on Arduino

// For battery heartbeat. The green led onboard blinks at an interval determined by the battery level
int statusLedState = LOW; 
unsigned long previousMillis = 0;
int interval = 1000; 
// Interval for checking battery voltage
int batt_timer;

// Motor driver has a current sensor resistor, used to determin load on drive wheels
int loadL;
int loadR;
// A0 and A1 reads the values
#define loadPinL 0
#define loadPinR 1
int loadlimit=80;  // Sets limit off the load on the drive wheels
int loadcounter; // Counter that gets added while high load. Used for not giving up directly on high load

// Measure battery voltage
int battv;  // Holds battery voltage value
int batteryVoltage;
#define voltsens 2

float vPow = 5.02; // Voltage at the Arduinos Vcc and Vref. 
float r1 = 11000;  // "Top" resistor, 11k (10+1)
float r2 = 2200;   // "Bottom" resistor (to ground), 2.2 kohm. 

// I2C display
int sda = A4;
int scl = A5;

// Distance sensor
#define trigPin 2
#define echoPin 4

/* Connections for cutter motor
   Mähmotor: 24V, 120W, 4000 rpm, Bürsten (FISE M5930C04004), Motorstrom etwa 3A mit Messer
   A3 - Cutter motor current sense
   D3(pwm) - Cutter motor control
*/
int cutterPower = A3;
int cutterCtrl = 3; 

/* 
Communicate with grass-sensor Mcu via I2C. 

How about a distance sensor in the back too?

Save pins with charlieplexing?
Or 
pin 1 --- 100ohm --- led1 anode --- led1 cathode --- 100ohm --- pin2
                  |--led2 cathode---led2 anode--- |    

*/

int speed;
String status="stop";

unsigned long startTime = 0;

void setup() {
  Serial.begin (57600);
  Serial.println("Welcome to Arduclip"); 

  // Setup pins for motor drivers
  pinMode(dirL,OUTPUT);
  pinMode(pwmL,OUTPUT);
  pinMode(enL,OUTPUT);
  pinMode(dirR,OUTPUT);
  pinMode(pwmR,OUTPUT);
  pinMode(enR,OUTPUT);

  // Cutter
  pinMode(cutterCtrl,OUTPUT); // Cutter on/off
  pinMode(cutterPower,INPUT); // Cutter load

  // Indicator leds
  pinMode(statusLed,OUTPUT);  
  pinMode(ylwLed,OUTPUT);  
  pinMode(battLed,OUTPUT);  

  // Load detection wheels
  pinMode(loadPinL, INPUT);
  pinMode(loadPinR, INPUT);

  // Battery voltage
  pinMode(voltsens, INPUT);

  battv = batt();       // Check
  Serial.print ("B: ");
  Serial.print(battv);
  Serial.println("V");

  // Define pins for HC-SR04
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Measure distance
  long dist = distance();
  Serial.print("Distance: ");
  Serial.println(dist);
  
  // Initial motor test
  /*
  digitalWrite(cutterCtrl, HIGH);
  Serial.println("Cutter motor test");
  delay(5000);
  digitalWrite(cutterCtrl, LOW);
  */
  /*
  analogWrite(pwmL, 100);//Sets speed variable via PWM 
  digitalWrite(dirL, HIGH);
  digitalWrite(enL, HIGH);
  delay(2000);
  digitalWrite(enL, LOW);
  delay(2000);
  digitalWrite(enL, LOW);
  delay(2000);
  
  analogWrite(pwmL, 180);//Sets speed variable via PWM 
  digitalWrite(dirL, HIGH);
  digitalWrite(enL, HIGH);
  delay(2000);
  digitalWrite(enL, LOW);
  delay(2000);

  analogWrite(pwmR, 180);//Sets speed variable via PWM 
  digitalWrite(dirR, HIGH);
  digitalWrite(enR, HIGH);
  delay(2000);
  digitalWrite(enR, LOW);
  delay(2000);
  
  
  rotateL(100);
  delay(2000);
  stop();
  delay(1000);
  rotateR(100);
  delay(2000);
  stop(); 
  delay(1000);
  goFwd(100);
  delay(2000);
  stop(); 
  delay(1000);
  goRew(100);
  delay(2000);
  stop(); 
  */
  
  //digitalWrite(onboardLed, HIGH);
  //delay(500);
  //digitalWrite(ylwLed, HIGH);
  //delay(500);
  //digitalWrite(redLed, HIGH);

  // Wait before we start running
  Serial.print("Wait...");
  for (int x=0;x<=5;x++) {
    digitalWrite(statusLed, HIGH);
    delay(300);
    digitalWrite(statusLed, LOW);
    delay(700);
  }
  Serial.println("Setup done"); 
}

void loop() {
  //startTime = millis(); //Calculate the time since last time the cycle was completed
  
  // For battery heartbeat
  unsigned long currentMillis = millis();

  // Check battery voltage
  if (batt_timer==10) {   // Dont check battery every time
    // Measure battery
    battv = batt();
    Serial.print ("Battery (times 10): ");
    Serial.print(battv);
    Serial.println("V");

    // Battery low?
    if (battv<=215){
      // Battery volt is to low!
      Serial.println("Battery low, stop!");
      digitalWrite(battLed, HIGH);          // Light up red battery led
      status="stop";
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Go to sleep to save power and stop execution
      sleep_enable();   
    }
    batt_timer=0;   // Reset counter
  }
  batt_timer=batt_timer+1;
  // Debug
  Serial.print("batt_timer: ");
  Serial.println(batt_timer);

  // Battery heartbeat
  /* battv is something between 290 and 215
  * If its low, blink fast
  * Remove 210 -> 5-80 
  * 
  */
  Serial.println(battv);
  interval = battv - 210;
  if (interval>=40) {     // Adjust at higher battery levels
    interval=interval/2;
  }
  interval=interval*100;  // Adjust to visible value
  
  // Debug
  Serial.print("statusLed interval:"); 
  Serial.println(interval);
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (statusLedState == LOW) {
      statusLedState = HIGH;
    } else {
      statusLedState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(statusLed, statusLedState);
  }
  
  // Start slowly
  speed = 100;
  if (status!="forward") {
    goFwd(speed);
  }

  //Serial.println(status);

  // Measure distance
  long dist = distance();
  Serial.print("Distance: ");
  Serial.println(dist);

  // We are near something
  if (dist<=13) {
    // Turn around
    Serial.println("We are close to something, turn around");
    digitalWrite(ylwLed, HIGH);
    turnAroundL();
    digitalWrite(ylwLed, LOW);
  }

  // Measure drive wheel load
  loadL = analogRead(loadPinL);
  loadR = analogRead(loadPinR);

  if (loadL>=loadlimit || loadR >=loadlimit) {  // High load, we are running in to something?
    // Add to load counter
    loadcounter++;
    if (loadcounter>=4) {
      // Turn around
      Serial.println("High drive wheel load, turn around");
      digitalWrite(ylwLed, HIGH);
      turnAroundL();
      digitalWrite(ylwLed, LOW);
      loadcounter=0;
    }
  }


  
/*
  unsigned long timeNow=millis();
  Serial.println(timeNow);
  
  while (timeNow -startTime<= 30000)  {
    timeNow=millis();
    loadL = analogRead(loadPinL);
    Serial.print("Load L: ");
    Serial.println(loadL);
    loadR = analogRead(loadPinR);
    Serial.print("Load R: ");
    Serial.println(loadR);
    
    delay(300);
  }
  stop();
  while(1){
    
  }
  */
  // Wait a while before next loop
  delay(500);
}

void goFwd(int speed) {
  // Start your engines
  analogWrite(pwmL, speed);//Sets speed variable via PWM 
  digitalWrite(dirL, LOW);
  digitalWrite(enL, HIGH);
  analogWrite(pwmR, speed);//Sets speed variable via PWM 
  digitalWrite(dirR, LOW);  
  digitalWrite(enR, HIGH);
  status="forward";

  // Start cutter motor after a small delay (to prevent a too big current rush)
  delay(600);
  digitalWrite(cutterCtrl, HIGH);
}
void goRew(int speed) {
  // Cutter off
  digitalWrite(cutterCtrl, LOW);
  // Drive
  analogWrite(pwmL, speed);//Sets speed variable via PWM 
  digitalWrite(dirL, HIGH);
  digitalWrite(enL, HIGH);
  analogWrite(pwmR, speed);//Sets speed variable via PWM 
  digitalWrite(dirR, HIGH);  
  digitalWrite(enR, HIGH);
  status="Reverse";

}
void rotateL(int speed) {
  // Cutter off
  digitalWrite(cutterCtrl, LOW);
  // RotateLeft
  analogWrite(pwmL, speed);//Sets speed variable via PWM 
  digitalWrite(dirL, HIGH);
  digitalWrite(enL, HIGH);
  analogWrite(pwmR, speed);//Sets speed variable via PWM 
  digitalWrite(dirR, LOW);  
  digitalWrite(enR, HIGH);
  status="Rotate Left";

}
void rotateR(int speed) {
  // Cutter off
  digitalWrite(cutterCtrl, LOW);
  // RotateLeft
  analogWrite(pwmL, speed);//Sets speed variable via PWM 
  digitalWrite(dirL, LOW);
  digitalWrite(enL, HIGH);
  analogWrite(pwmR, speed);//Sets speed variable via PWM 
  digitalWrite(dirR, HIGH);  
  digitalWrite(enR, HIGH);
  status="Rotate Right";

}

void turnAroundL() {
  // Cutter off
  digitalWrite(cutterCtrl, LOW);
    // Turn around, ccw
    Serial.println("Turn around ccw");
    stop();
    delay(1000);
    rotateL(100);
    delay(6000);
    stop();
    delay(1000);
}

void stop() {
  // Cutter off
  digitalWrite(cutterCtrl, LOW);
  // Stop
  digitalWrite(enL, LOW);
  digitalWrite(enR, LOW);
  status="Stop";

}
int batt() {
  int adcvalue = analogRead(voltsens);
  //Serial.print("Batt adc: ");
  //Serial.println(adcvalue);
  int volt = (adcvalue * vPow*10) / 1024.0;   // Multiply by ten -> gives a 'decimal' with an int
  int volt2 = volt / (r2 / (r1 + r2));
  //Serial.print("Battery: ");
  //Serial.print(volt2);
  //Serial.println("V");
  return volt2;
}

long distance() {
  // Measure distance to sensor
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  //  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}
