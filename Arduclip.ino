

/*
 * TODO
 * Change red led to green(Add green led)
 * Battery monitor
 * Check signals from cutter motor hall sensor
 * Control cutter motor
 * Add on/off switch
 * Distance sensor HC-SR04
 */

/*
 * Battery monitor
 * Max V = 30V
 * Battery + --- 11k(10k+1k) --- sense --- 2.2k --- Ground
 * Gives 5 volt @sense @ 30V on battery
 */

// Connections for wheel motor drivers
int dirL = 7;
int pwmL = 5;
int enL = 9;
int dirR = 8;
int pwmR = 6;
int enR = 10;

// Connections for cutter motor
// Mähmotor: 24V, 120W, 4000 rpm, Bürsten (FISE M5930C04004), Motorstrom etwa 3A mit Messer

// Indicator leds
int ylwLed=11;
int redLed=12;
int onboardLed=13;

// Motor driver has a current sensor resistor, used to determin load on drive wheels
int loadL;
int loadR;
// A0 and A1 reads the values
#define loadPinL 0
#define loadPinR 1
int loadlimit=80;  // Sets limit off the load on the drive wheels

// Measure battery voltage
int batteryVoltage;
#define voltsens 3

float vPow = 5.02; // Voltage at the Arduinos Vcc and Vref. 
float r1 = 11000;  // "Top" resistor, 11k (10+1)
float r2 = 2200;   // "Bottom" resistor (to ground), 2.2 kohm. 

// I2C display
int sda = A4;
int scl = A5;

/* Free pins
A2, A3, D2, D3, D4
Proposed use:
A2 - Cutter motor hall sensor
(A3 - Cutter motor current sense) - Has to be used for battery sensor!
D2 - Distance sensor trig
D3(pwm) - Cutter motor control
D4 - Distance sensor echo
#define trigPin 2
#define echoPin 4

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

  // Indicator leds
  pinMode(onboardLed,OUTPUT);  
  pinMode(ylwLed,OUTPUT);  
  pinMode(redLed,OUTPUT);  

  // Load detection
  pinMode(loadPinL, INPUT);
  pinMode(loadPinR, INPUT);

  // Battery voltage
  pinMode(voltsens, INPUT);

  int battv = batt();
  Serial.print ("B: ");
  Serial.print(battv);
  Serial.println("V");

  // Initial test
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
  
  digitalWrite(onboardLed, HIGH);
  delay(500);
  digitalWrite(ylwLed, HIGH);
  delay(500);
  digitalWrite(redLed, HIGH);

  // Wait before we start running
  Serial.print("Wait...");
  delay(1000);

  Serial.println("Setup done"); 


}

void loop() {
  startTime = millis(); //Calculate the time since last time the cycle was completed
  // Start slowly
  speed = 100;
  if (status=="Stop") {
    //goFwd(speed);
  }

  Serial.println(status);

  // Measure drive wheel load
  loadL = analogRead(loadPinL);
  loadR = analogRead(loadPinR);

  if (loadL>=loadlimit || loadR >=loadlimit) {  // High load, we are running in to something?
    // Turn around
    Serial.println("High drive wheel load, turn around");
    stop();
    delay(1000);
    rotateL(100);
    delay(2000);
    stop();
    delay(1000);
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
  digitalWrite(dirL, HIGH);
  digitalWrite(enL, HIGH);
  analogWrite(pwmR, speed);//Sets speed variable via PWM 
  digitalWrite(dirR, HIGH);  
  digitalWrite(enR, HIGH);
  status="Forward";
}
void goRew(int speed) {
  analogWrite(pwmL, speed);//Sets speed variable via PWM 
  digitalWrite(dirL, LOW);
  digitalWrite(enL, HIGH);
  analogWrite(pwmR, speed);//Sets speed variable via PWM 
  digitalWrite(dirR, LOW);  
  digitalWrite(enR, HIGH);
  status="Reverse";

}
void rotateL(int speed) {
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
  // RotateLeft
  analogWrite(pwmL, speed);//Sets speed variable via PWM 
  digitalWrite(dirL, LOW);
  digitalWrite(enL, HIGH);
  analogWrite(pwmR, speed);//Sets speed variable via PWM 
  digitalWrite(dirR, HIGH);  
  digitalWrite(enR, HIGH);
  status="Rotate Right";

}
void stop() {
  // Stop
  digitalWrite(enL, LOW);
  digitalWrite(enR, LOW);
  status="Stop";

}
int batt() {
  int adcvalue = analogRead(voltsens);
  Serial.print("Batt adc: ");
  Serial.println(adcvalue);
  int volt = (adcvalue * vPow) / 1024.0;
  int volt2 = volt / (r2 / (r1 + r2));
  Serial.print("Battery: ");
  Serial.print(volt2);
  Serial.println("V");
}
