const byte ledPin = 17;
const byte sensor1Pin = 2;
const byte sensor2Pin = 3;
const byte sensor3Pin = 7;
const double SPEEDOFSOUND = 0.034029;
volatile byte riseIntOn = RISING;
double xCoordinate;
double yCoordinate;
volatile unsigned long sensor1Delay;
volatile unsigned long sensor2Delay;
volatile unsigned long sensor3Delay;
volatile unsigned long startTime;
volatile unsigned long endTime1;
volatile unsigned long endTime2;
volatile int poradi;
volatile bool measComplete=false;
bool failMeas = false;
volatile byte alreadyHit;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(sensor1Pin, INPUT);
  pinMode(sensor2Pin, INPUT);
  pinMode(sensor3Pin, INPUT);
  Serial.begin(9600);
  while (!Serial) { // wait for Serial port to connect. Needed for native USB port only

      digitalWrite(ledPin, HIGH);
      delay(1000);  
      digitalWrite(ledPin, LOW);
      delay(1000);  
  }
  TurnOnSensorslInterrupt();
}

void loop() {
  char doubleInteger[15];
  while(!measComplete)
  {
    delay(200);
    if(poradi == 1 || poradi == 2)
    {
      long wrongMeas = (micros()-startTime);
      bool isHigher = wrongMeas > 1000;
      if(isHigher)
      {
        //TurnOffSensorsInterrupt();
        measComplete = true;
        failMeas = true;
        //Serial.println(poradi,DEC);
        //Serial.println(wrongMeas,DEC);
        }
      }
    }
  if(failMeas)
  {
    Serial.print("Fail measurement");
    }
  else
  {
    if(sensor1Delay==0)
    {
      PositionFromLeftSensor();
      dtostrf(xCoordinate,3, 2, doubleInteger);
      Serial.print("x: ");
      Serial.println(doubleInteger);
      dtostrf(yCoordinate,3, 2, doubleInteger);
      Serial.print("y: ");
      Serial.println(doubleInteger);
      }
    else if(sensor2Delay==0)
    {
      PositionFromMiddleSensor();
      dtostrf(xCoordinate,3, 2, doubleInteger);
      Serial.print("x: ");
      Serial.println(doubleInteger);
      dtostrf(yCoordinate,3, 2, doubleInteger);
      Serial.print("y: ");
      Serial.println(doubleInteger);   
      }
    else
    {
      PositionFromRightSensor();
      dtostrf(xCoordinate,3, 2, doubleInteger);
      Serial.print("x: ");
      Serial.println(doubleInteger);
      dtostrf(yCoordinate,3, 2, doubleInteger);
      Serial.print("y: ");
      Serial.println(doubleInteger);   
      }
      
    Serial.print("sens1: ");
    dtostrf(sensor1Delay,3, 0, doubleInteger);
    Serial.println(doubleInteger);
    Serial.print("sens2: ");
    dtostrf(sensor2Delay,3, 0, doubleInteger); 
    Serial.println(doubleInteger);
    Serial.print("sens3: ");
    dtostrf(sensor3Delay,3, 0, doubleInteger); 
    Serial.println(doubleInteger);
    }
  Serial.println();

  measComplete = false;
  failMeas = false;
  poradi = 0;
  sensor1Delay = 0;
  sensor2Delay = 0;
  sensor3Delay = 0;
  startTime = 0;
  endTime1 = 0;
  endTime2 = 0;
  alreadyHit = 0;
}

///Calculates x and y coordinates of source of a sound when the left microphone catch the sound first.
void PositionFromLeftSensor()
{
  double shorterDistance = sensor2Delay * SPEEDOFSOUND;
  double furtherDistance = sensor3Delay * SPEEDOFSOUND;
  xCoordinate = (furtherDistance*(441+4*pow(shorterDistance,2)-4*shorterDistance*furtherDistance))/(84*(2*shorterDistance-furtherDistance));
  yCoordinate =-(double)1/84*sqrt(abs(-(16*pow(shorterDistance,4)*pow(furtherDistance,2))/pow((2*shorterDistance-furtherDistance),2)+(7056*pow(shorterDistance,4))/pow((2*shorterDistance-furtherDistance),2)+(32*pow(shorterDistance,3)*pow(furtherDistance,3))/pow((2*shorterDistance-furtherDistance),2)-(14112*pow(shorterDistance,3)*furtherDistance)/pow((2*shorterDistance-furtherDistance),2)-(16*pow(shorterDistance,2)*pow(furtherDistance,4))/pow((2*shorterDistance-furtherDistance),2)+(3528*pow(shorterDistance,2)*pow(furtherDistance,2))/pow((2*shorterDistance-furtherDistance),2)+(1555848*pow(shorterDistance,2))/pow((2*shorterDistance-furtherDistance),2)+(3528*shorterDistance*pow(furtherDistance,3))/pow((2*shorterDistance-furtherDistance),2)-(194481*pow(furtherDistance,2))/pow((2*shorterDistance-furtherDistance),2)-(1555848*shorterDistance*furtherDistance)/pow((2*shorterDistance-furtherDistance),2)+85766121/pow((2*shorterDistance-furtherDistance),2)+1764*pow(furtherDistance,2)-777924));
  }

void PositionFromMiddleSensor()
{
  double toLeftDistance = sensor1Delay * SPEEDOFSOUND;
  double toRightDistance = sensor3Delay * SPEEDOFSOUND;
  xCoordinate = (4*pow(toLeftDistance,2)*toRightDistance-4*toLeftDistance*pow(toRightDistance,2)+441*toLeftDistance-441*toRightDistance)/(84*(toLeftDistance+toRightDistance));
  yCoordinate = -sqrt(abs(16*pow(toLeftDistance,4)-(4*pow(toLeftDistance,2)*pow((4*pow(toLeftDistance,2)*toRightDistance-4*toLeftDistance*pow(toRightDistance,2)+441*toLeftDistance-441*toRightDistance),2))/(441*pow((toLeftDistance+toRightDistance),2))-(8*pow(toLeftDistance,2)*(4*pow(toLeftDistance,2)*toRightDistance-4*toLeftDistance*pow(toRightDistance,2)+441*toLeftDistance-441*toRightDistance))/(toLeftDistance+toRightDistance)+pow((4*pow(toLeftDistance,2)*toRightDistance-4*toLeftDistance*pow(toRightDistance,2)+441*toLeftDistance-441*toRightDistance),2)/pow((toLeftDistance+toRightDistance),2)+(882*(4*pow(toLeftDistance,2)*toRightDistance-4*toLeftDistance*pow(toRightDistance,2)+441*toLeftDistance-441*toRightDistance))/(toLeftDistance+toRightDistance)-3528*pow(toLeftDistance,2)+194481))/(8*toLeftDistance);
  }
  
void PositionFromRightSensor()
{
  double shorterDistance = sensor2Delay * SPEEDOFSOUND;
  double furtherDistance = sensor1Delay * SPEEDOFSOUND;
  xCoordinate = (furtherDistance*(4*shorterDistance*(furtherDistance-shorterDistance)-441))/(84*(2*shorterDistance-furtherDistance));
  yCoordinate = -(double)1/2*sqrt(abs(-(64*pow(shorterDistance,4)*pow(furtherDistance,2))/pow((168*shorterDistance-84*furtherDistance),2)+(28224*pow(shorterDistance,4))/pow((168*shorterDistance-84*furtherDistance),2)+(128*pow(shorterDistance,3)*pow(furtherDistance,3))/pow((168*shorterDistance-84*furtherDistance),2)-(56448*pow(shorterDistance,3)*furtherDistance)/pow((168*shorterDistance-84*furtherDistance),2)-(64*pow(shorterDistance,2)*pow(furtherDistance,4))/pow((168*shorterDistance-84*furtherDistance),2)+(14112*pow(shorterDistance,2)*pow(furtherDistance,2))/pow((168*shorterDistance-84*furtherDistance),2)+(6223392*pow(shorterDistance,2))/pow((168*shorterDistance-84*furtherDistance),2)+(14112*shorterDistance*pow(furtherDistance,3))/pow((168*shorterDistance-84*furtherDistance),2)-(777924*pow(furtherDistance,2))/pow((168*shorterDistance-84*furtherDistance),2)-(6223392*shorterDistance*furtherDistance)/pow((168*shorterDistance-84*furtherDistance),2)+343064484/pow((168*shorterDistance-84*furtherDistance),2)+pow(furtherDistance,2)-441));
  }
  
void TurnOnSensorslInterrupt()
{
  attachInterrupt(digitalPinToInterrupt(sensor1Pin), sensor1Interrupt, riseIntOn);
  attachInterrupt(digitalPinToInterrupt(sensor2Pin), sensor2Interrupt, riseIntOn);
  attachInterrupt(digitalPinToInterrupt(sensor3Pin), sensor3Interrupt, riseIntOn);  
  }

void TurnOffSensorsInterrupt()
{
  detachInterrupt(digitalPinToInterrupt(sensor1Pin)); 
  detachInterrupt(digitalPinToInterrupt(sensor2Pin)); 
  detachInterrupt(digitalPinToInterrupt(sensor3Pin)); 
  }  

long SaveTimeDifference()
{
    if(poradi==0)
  {
    startTime = micros();
    poradi++;
    measComplete = false;
    return 0;
    }
  else if (poradi == 1)
  {
    endTime1 = micros()-startTime;
    poradi++;
    measComplete = false;
    return endTime1;
    }
  else
  {
    endTime2 = micros()-startTime;
    poradi++;
    measComplete = true;
    return endTime2;
    }
  }

void sensor1Interrupt() {
  bool alreadyHitBool = alreadyHit&1;
  if(alreadyHitBool == 0)
  {
    alreadyHit += 1;
    sensor1Delay = SaveTimeDifference();
  }
}

void sensor2Interrupt() {
  bool alreadyHitBool = alreadyHit&2;
  if(alreadyHitBool == 0)
  {
    alreadyHit += 2;
    sensor2Delay = SaveTimeDifference();  
  }
}

void sensor3Interrupt() {
  bool alreadyHitBool = alreadyHit&4;
  if(alreadyHitBool == 0)
  {
    alreadyHit += 4;  
    sensor3Delay = SaveTimeDifference();
  }
}
