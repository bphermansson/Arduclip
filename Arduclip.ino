

/*
 * TODO
 * Add green led
 * Control cutter motor
 * Add on/off switch
 */

// Connections for motor drivers
int dirL = 7;
int pwmL = 5;
int enL = 9;
int dirR = 8;
int pwmR = 6;
int enR = 10;

int onboardLed=13;
int ylwLed=11;
int redLed=12;

// Motor driver has a current sensor resistor, used to determin load on drive wheels
int loadL;
int loadR;

int loadlimit=80;  // Sets limit off the load on the drive wheels

int speed;
String status="stop";


#define loadPinL 0
#define loadPinR 1

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
