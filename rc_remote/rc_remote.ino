
#include <Wire.h>
#include <ADXL345.h>
#include <SPI.h>
#include "RF24.h"
#include <Messages.h>

//#define DEBUGGING

#define CE_PIN 10
#define CSN_PIN 9

RF24 radio(CE_PIN,CSN_PIN);
/**********************************************************/
byte my_address[7] = "Remote";
byte listener_address[7] = "Listnr";

int const ledPin = 3;
int const buttonPin = 4;
ADXL345 acc;

void setup() {
  #ifdef DEBUGGING
  Serial.begin(9600);
  #endif
  acc.begin();
  acc.setRange(ADXL345::RANGE_4G);
  int const beginStatus = radio.begin(); // Start up the radio
   
  radio.setAutoAck(1); // Ensure autoACK is enabled
  radio.setRetries(15,15); // Max delay between retries & number of retries
  radio.openWritingPipe(listener_address); // Write to device address '2Node'
  radio.openReadingPipe(1,my_address); // Read on pipe 1 for device address '1Node'
  radio.startListening(); // Start listening
  
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  digitalWrite(ledPin, LOW);
}

TiltRequest tiltRequest(TILT_REQUEST);
void sendTiltRequest(const TiltValues tilt, const int button)
{
    radio.stopListening();
    tiltRequest.header.transactionId++;
    tiltRequest.tilt = tilt;
    tiltRequest.button = button;
    radio.write( &tiltRequest, sizeof(tiltRequest) );  
    radio.startListening();
}

struct FailureSnooper
{
  FailureSnooper(): tOut(false), wResp(false), wTrId(false)
  {}
  void timeout()
  {
    tOut = true;
  }
  void wrongResponse()
  {
    wResp = true;
  }
  void wrongTransaction()
  {
    wTrId = true;
  }
  bool sawProblems()
  {
    return ((tOut == true) || (wResp == true) || (wTrId == true));
  }
  bool tOut, wResp, wTrId;
};

void reportProblem()
{
  digitalWrite(ledPin, HIGH);
}
void clearProblem()
{
  digitalWrite(ledPin, LOW);
}

double fXg, fYg, fZg;

TiltValues getTiltValues()
{
   TiltValues values;
   double pitch, roll = 0;
   int Xg, Yg, Zg = 0;
   acc.read(&Xg, &Yg, &Zg);
   //Low Pass Filter
   const float alpha = 0.5;
   fXg = Xg * alpha + (fXg * (1.0 - alpha));
   fYg = Yg * alpha + (fYg * (1.0 - alpha));
   fZg = Zg * alpha + (fZg * (1.0 - alpha));
   //Roll & Pitch Equations
   values.roll = (atan2(-fYg, fZg)*180.0)/M_PI;
   values.pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;
   return values;
}

TIME prevActionAt = 0;
TIME const timeLogThreshold = 1000;
TIME timePrevLog = 0;
bool idleSomeMore()
{
  const TIME idleThreshold = 100;
  const TIME timeNow = millis();
  return (timeNow - prevActionAt < idleThreshold) ? true : false;
}
void loop() {
  //if (idleSomeMore()) return;
  const int button = digitalRead(buttonPin);
  TiltValues tilt = getTiltValues();
  sendTiltRequest(tilt, button);
  unsigned long started_waiting_at = micros();
  FailureSnooper snooper;
  while ( !radio.available() ){ // While nothing is received
    if (micros() - started_waiting_at > 20000 ){ // If waited longer than 200ms, indicate timeout and exit while loop
      snooper.timeout();
      break;
    }

  }
  
  if ( !snooper.sawProblems() )
  {
    Response response(0,0);
    radio.read( &response, sizeof(response) );
    if (response.header.msgId != TILT_RESPONSE)
    {
      snooper.wrongResponse(); 
    }
    if (response.header.transactionId != tiltRequest.header.transactionId)
    {
      snooper.wrongTransaction(); 
    }
  }
  else
  {
    reportProblem();
  }
  clearProblem();
  #ifdef DEBUGGING
  TIME const timeNow = millis();
  if(timeNow - timePrevLog > timeLogThreshold)
  {
    Serial.print("still alive: "); Serial.println(timeNow);
    timePrevLog = timeNow;
  }
  #endif
}

