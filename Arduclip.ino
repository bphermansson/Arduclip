

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

int speed;

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
  */
  digitalWrite(onboardLed, HIGH);
  delay(500);
  digitalWrite(ylwLed, HIGH);
  delay(500);
  digitalWrite(redLed, HIGH);

  // Wait before we start running
  delay(10000);

  Serial.println("Setup done"); 


}

void loop() {
  startTime = millis(); //Calculate the time since last time the cycle was completed
  // Start slowly
  speed = 100;
  goFwd(speed);

  // Measure drive wheel load
  loadL = analogRead(loadPinL);
  loadR = analogRead(loadPinR);

  if (loadL>80 || loadR >80) {  // High load, we are running in to something?
    // Turn around
    
  }

  unsigned long timeNow=millis();
  Serial.println(timeNow);
  
  while (timeNow -startTime<= 30000)  {
    timeNow=millis();
    loadL = analogRead(loadPinL);
    Serial.println(loadL);
    delay(300);
  }
  // Stop
  digitalWrite(enL, LOW);
  digitalWrite(enR, LOW);
  while(1){
    
  }
}

void goFwd(int speed) {
  // Start your engines
  analogWrite(pwmL, speed);//Sets speed variable via PWM 
  digitalWrite(dirL, HIGH);
  digitalWrite(enL, HIGH);
  
  analogWrite(pwmR, speed);//Sets speed variable via PWM 
  digitalWrite(dirR, HIGH);  
  digitalWrite(enR, HIGH);
}


