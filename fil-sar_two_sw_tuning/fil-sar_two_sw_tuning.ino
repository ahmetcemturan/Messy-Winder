#define EN          23
#define dirPin      25
#define Step        27

#define Tur_pin_Top         18           // RPM Sensor Top Roll Tur-Switch
#define Tur_pin_Bottom      19        // RPM Sensor Bottom Roll Tur-Switch
#define Right_pin           21               // Proximity Switch for Right and Park/Load position
#define Left_pin            3                // Proximity Switch for Left Return position
#define Load_pin            2                // PushButton to move the carriage to Park/Load position

#define PotTurPin           A1
//#define PotDelayPin         A3

#define Distance            1.75 //Filament Diameter
#define interruptors        2// number of interruptors on opto wheel for Speed calculation
#define Pitch               4 // pitch of leadscrew
#define Resolution          200 // step motor resolution for 1.8deg step angle
#define Microsteps          8 //settings on Step driver
#define EndStopOffset       3
#define spoolWidth          1
long int AutonomousSteps   = Microsteps / Pitch  * Resolution * spoolWidth;// steps in the loop for movement
int endStopOffset = Microsteps / Pitch  * Resolution * EndStopOffset;

volatile unsigned long startTime;
volatile unsigned long endTime;
volatile boolean right = LOW, hall = LOW, left = HIGH, load = LOW;
volatile unsigned long duration = 0;

volatile int Speed = 0;
//volatile int dirChangeDelay = 0;

//float delayAdjust = 1;
volatile float speedAdjust = 1;

volatile int cTop = 0;
volatile int cBottom = 0;
volatile float setupNumber = Distance / (Pitch) * Resolution;


//-----SETUP----------------------
void setup()
{
  delay(2000);
  Serial.begin(115200);
  delay(200);
  Serial.println("System Initialized....");
  initPins();

}

//-----LOOP----------------------
void loop()
{
//  duration = endTime - startTime;
  Serial.print("duration");Serial.println(duration);
  speedAdjust = analogRead (PotTurPin) / 1023.0 + 0.5;
  Serial.print("loop speed:  "); Serial.println(Speed);
  //Speed = (1000000.0 / (Microsteps * (1000.0 / (duration * interruptors)) * setupNumber)) * speedAdjust; //FORMULA //FORMULA
  //Speed = 250;
  //    Serial.println("Speed:");
  //  Serial.println("loop");
  //  Serial.println (speedAdjust);
  //  dirChangeDelay = duration * interruptors * 1000.0 / 4.0 * delayAdjust;
  //  float RPS = 1000.0 / (duration * interruptors);
  //  Serial.print("Speed RPS: "); Serial.println(RPS);
  digitalWrite (EN, LOW);
  digitalWrite (dirPin, HIGH);
  digitalWrite (EN, HIGH);
  Serial.println("loop HIGH");
  for (int i = 0; i < (AutonomousSteps); i++) //ACT
  {
    digitalWrite(Step, HIGH);
    delayMicroseconds(Speed);
    digitalWrite(Step, LOW);
    delayMicroseconds(Speed);
  }
  digitalWrite (EN, LOW);
  digitalWrite (dirPin, LOW);
  digitalWrite (EN, HIGH);
  Serial.println("loop LOW");
  for (int i = 0; i < (AutonomousSteps); i++) //ACT
  {
    digitalWrite(Step, HIGH);
    delayMicroseconds(Speed);
    digitalWrite(Step, LOW);
    delayMicroseconds(Speed);
  }
}

//------INITIALIZE PINS---------------------
void initPins()
{
  pinMode(EN, OUTPUT);    // ENABLE AS OUTPUT
  pinMode(Step, OUTPUT);  // STEP AS OUTPUT
  pinMode(dirPin, OUTPUT); // DIRECTION AS OUTPUT
  digitalWrite(EN, HIGH);  // SET ENABLE TO HIGH FOR EXTERNAL DRIVER, LOW FOR STEPSTICKS

  pinMode(Tur_pin_Top , INPUT_PULLUP);
  pinMode(Tur_pin_Bottom , INPUT_PULLUP);
  pinMode(Right_pin, INPUT_PULLUP);
  pinMode(Left_pin, INPUT_PULLUP);
  pinMode(Load_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Tur_pin_Top), hallButTop, FALLING);
  attachInterrupt(digitalPinToInterrupt(Tur_pin_Bottom), hallButBottom, FALLING);

  attachInterrupt(digitalPinToInterrupt(Right_pin), hitRightEndstop, FALLING);
  attachInterrupt(digitalPinToInterrupt(Left_pin), leftBut, FALLING);
  attachInterrupt(digitalPinToInterrupt(Load_pin), loadBut, RISING);

  delay(1000);

  if (digitalRead(Right_pin) == LOW)
  { digitalWrite(dirPin, HIGH);
    firstMovementAfterEndstopHit();
    //    Serial.println("System is on RIGHT side initally ");
    Serial.println (" Already at START waiting for signal...112");
  }
  else
  {
    //   Serial.println("System is on LEFT side initally ");
    Serial.println ("Going right for START... 117");
    loadProcedure();
  }
}

//--------When Park-Load Button triggers-------------------
void loadBut()// ISR
{
  Serial.println("Load Button Pressed..");
  Serial.println("Move to Right-EndStop and Wait");
  digitalWrite (EN, LOW);
  left = LOW;
  right = HIGH;
  hall = LOW;
  load = HIGH;
  loadProcedure();
  //  delay(100);
}


//------PROCEDURE when PARK-Load button is pressed

void loadProcedure()//no ISR
{
  //detachInterrupt(digitalPinToInterrupt(Tur_pin_Top));
  //detachInterrupt(digitalPinToInterrupt(Tur_pin_Bottom));
  digitalWrite (EN, HIGH);
  if (!left)
  {
    digitalWrite(dirPin, LOW);
    Serial.println("LOW 169");
    movLoad();
  }
  else if (left)
  { movLoad();
    Serial.println("System should Wait unless Tur-Switch is detected ...");
    digitalWrite(dirPin, HIGH);
    Serial.println("HIGH 175");
  }
  //attachInterrupt(digitalPinToInterrupt(Tur_pin_Top), hallButTop, FALLING);
  //attachInterrupt(digitalPinToInterrupt(Tur_pin_Bottom), hallButTop, FALLING);
}

//---------PARKING MOVE----- Go Right and wait
void movLoad()//no ISR
{
  Serial.println("LOW 163");
  if ((digitalRead(Right_pin)) == HIGH)
  { digitalWrite(dirPin, LOW);
    Serial.println("going right 166");
    Speed = (1000000.0 / (Microsteps * (1000.0 / (180)) * setupNumber)) / 2;
    //attachInterrupt(digitalPinToInterrupt(Right_pin), hitRightEndstop, FALLING);
    delay (500);
    Serial.println("170");
    while (digitalRead(Right_pin) == HIGH)
      for (int i = 0; i < 20; i++)//ACT
      {
        digitalWrite(Step, HIGH);
        delayMicroseconds(Speed);
        digitalWrite(Step, LOW);
        delayMicroseconds(Speed);
      }
  }
  else
  { Serial.println("181");
    digitalWrite(dirPin, HIGH);
    Serial.println("HIGH 183");
    //digitalWrite(EN, HIGH);
    //moveit();
  }
}
//------When it hits right Enstop during Park/load-----
void hitRightEndstop()    //ISR
{ Serial.println("191 WTF!!!");
  hitRightEndstopnonISR();
}

void hitRightEndstopnonISR()    // NON SR
{
  Serial.println("Right Endstop hit !!");

  //-------
  while ((digitalRead(Right_pin)) == LOW)
  { Serial.println("195");
    delay (100);
    digitalWrite (EN, LOW);
    digitalWrite (dirPin, HIGH);
    digitalWrite (EN, HIGH);
    for (int i = 0; i < endStopOffset ; i++) //ACT
    {
      digitalWrite(Step, HIGH);
      delayMicroseconds(Speed);
      digitalWrite(Step, LOW);
      delayMicroseconds(Speed);
    }
  }
  digitalWrite(EN, LOW);
  Serial.println("211");
  loop();
//  firstMovementAfterEndstopHit();
  //   hallButTop();
  //   hallButBottom();
}

void firstMovementAfterEndstopHit()
{
  Serial.println ("FMAEH 226");
  digitalWrite (dirPin, HIGH);
  attachInterrupt(digitalPinToInterrupt(Tur_pin_Top), hallButTop, FALLING);
  attachInterrupt(digitalPinToInterrupt(Tur_pin_Bottom), hallButBottom, FALLING);
  loop();
  //   hallButTop();
  //   hallButBottom();
}







//------DETECT ROTATION/SPEED SIGNAL -------------------
//--------HALLs BEGIN----------------
void hallButTop()// ISR
{ Serial.println ("hallButTop()");
  digitalWrite (EN, HIGH);
  hall = HIGH;
  cTop++;
  if (cTop == 1)
  {
    startTime = millis();
  }
  else if (cTop == 2)
  {
    endTime = millis();
    duration = endTime - startTime;
      speedAdjust = analogRead (PotTurPin) / 1023.0 + 0.5;
      Speed = (1000000.0 / (Microsteps * (1000.0 / (duration * interruptors)) * setupNumber)) * speedAdjust; //FORMULA //FORMULA
      Serial.print("top speed:  "); Serial.println(Speed);
      cTop = 0;
  }
}


void hallButBottom()// ISR
{
  digitalWrite (EN, HIGH);
  hall = HIGH;
  cBottom++;
  if (cBottom == 1)
  {
    startTime = millis();
  }
  else if (cBottom == 2)
  {
    endTime = millis();
    duration = endTime - startTime;
      speedAdjust = analogRead (PotTurPin) / 1023.0 + 0.5;
      Speed = (1000000.0 / (Microsteps * (1000.0 / (duration * interruptors)) * setupNumber)) / 2 * speedAdjust; //FORMULA //FORMULA
        Serial.print("bottom speed:  "); Serial.println(Speed);
      cBottom = 0;
  }
}
//--------------HALLs END-----------------

//---------------------------
void rightBut()// ISR
{ Serial.println("Right EndStop Detected...");
  left = HIGH;
  right = LOW;
  hall = LOW;
  // delay(dirChangeDelay);
}

//---------------------------
void leftBut()// ISR
{ Serial.println("Left EndStop Detected...");
  left = LOW;
  right = HIGH;
  hall = LOW;
  // delay(dirChangeDelay);
}



//-----Main Movement procedure BOTH DIR---------------------
/*void moveit()//no ISR
  { digitalWrite(EN, HIGH);
  Serial.println("270");

  /* for (int i = 0; i < (10); i++)//ACT
    {
     digitalWrite(Step, HIGH);
     delayMicroseconds(Speed*4);
     digitalWrite(Step, LOW);
     delayMicroseconds(Speed*4);
    }

    for (int i = 0; i < (20); i++)//ACT
    {
     digitalWrite(Step, HIGH);
     delayMicroseconds(Speed*3);
     digitalWrite(Step, LOW);
     delayMicroseconds(Speed*3);
    }

    for (int i = 0; i < (40); i++)//ACT
    {
     digitalWrite(Step, HIGH);
     delayMicroseconds(Speed*2);
     digitalWrite(Step, LOW);
     delayMicroseconds(Speed*2);
    }
*/
/*  digitalWrite (dirPin, HIGH);
  for (int i = 0; i < (AutonomousSteps); i++) //ACT
  {
    digitalWrite(Step, HIGH);
    delayMicroseconds(Speed);
    digitalWrite(Step, LOW);
    delayMicroseconds(Speed);
  }
  digitalWrite (dirPin, LOW);
  for (int i = 0; i < (AutonomousSteps); i++) //ACT
  {
    digitalWrite(Step, HIGH);
    delayMicroseconds(Speed);
    digitalWrite(Step, LOW);
    delayMicroseconds(Speed);
  }
  //  Serial.println("TOGGLE 308 ");
  //digitalWrite (dirPin, !digitalRead(dirPin));// toggle direction
  }*/
