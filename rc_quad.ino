
/*
* Getting Started example sketch for nRF24L01+ radios
* This is a very basic example of how to send data from one node to another
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"
#include "Messages.h"

#define DEBUGGING

#ifdef DEBUGGING
#define _SERIAL Serial
#endif
#define CE_PIN 10
#define CSN_PIN 9
static const int pinMotorRightPoleTwo = 8;
static const int pinMotorRightPoleOne = 7;
static const int pinMotorRightEnable = 6;
static const int pinMotorLeftEnable = 5;
static const int pinMotorLeftPoleTwo = 4;
static const int pinMotorLeftPoleOne = 3;
static const int pinHeadlights = 2;

RF24 radio(CE_PIN,CSN_PIN);
/**********************************************************/
byte my_address[7] = "Listnr";
byte remote_address[7] = "Remote";

#include "DualMotors.h"

struct Tracer
{
  const char* name;
  int failed_writes;
  int wrong_message;
  int pitch;
  int roll;
  int button;
  int timeouts;
  Tracer(const char* n): 
   name(n),
   failed_writes(0),
   wrong_message(0),
   pitch(0),
   roll(0),
   button(0),
   timeouts(0)
   {}
   void log(const long timeNow) const
   {
    #ifdef DEBUGGING
        _SERIAL.print(name);
        _SERIAL.print(F(", time:"));
        _SERIAL.print(timeNow);
        _SERIAL.print(F(", failed writes: "));
        _SERIAL.print(failed_writes);
        _SERIAL.print(F(", timeouts: "));
        _SERIAL.print(timeouts);    
        _SERIAL.print(F(", wrong: "));
        _SERIAL.print(wrong_message);
        _SERIAL.print(F(", pitch: "));
        _SERIAL.print(pitch);
        _SERIAL.print(F(", roll: "));
        _SERIAL.print(roll);
        _SERIAL.print(F(", button: "));
        _SERIAL.println(button);    
    #endif
    }
};

Motor motorLeft("Left", pinMotorLeftEnable, pinMotorLeftPoleOne, pinMotorLeftPoleTwo);
Motor motorRight("Right", pinMotorRightEnable, pinMotorRightPoleOne, pinMotorRightPoleTwo);
DualMotors motors(motorLeft, motorRight);

void setup() {
  motorLeft.init();
  motorRight.init();
  pinMode(pinHeadlights, OUTPUT);
  digitalWrite(pinHeadlights, LOW);
  #ifdef DEBUGGING
  _SERIAL.begin(9600);
  _SERIAL.println("startup!"); 
  #endif

  radio.begin();
  radio.setAutoAck(1); // Ensure autoACK is enabled
  radio.setRetries(15,15); // Max delay between retries & number of retries
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses

  // Start the radio listening for data
  radio.openReadingPipe(1,my_address);
  radio.openWritingPipe(remote_address);
}

void write_response(Tracer& tracer, int transactionId)
{
    Response response(TILT_RESPONSE, transactionId);
    if (!radio.write( &response, sizeof(response) ))
     {
       tracer.failed_writes++;
     }  
}
int read_request(Tracer& tracer)
{
        TiltRequest request(TILT_REQUEST);
        radio.read( &request, sizeof(request) );
        if (request.header.msgId != TILT_REQUEST)
        {
          tracer.wrong_message++;
        }
        tracer.pitch = request.tilt.pitch;
        tracer.roll = request.tilt.roll;
        tracer.button = request.button;
        
        return request.header.transactionId;
}
bool communicateWithNode(const int respMsgId, Tracer& tracer)
{
    radio.startListening();
    unsigned long const started_waiting_at = micros();
    boolean timeout = false;

    while ( ! radio.available() ){
      if (micros() - started_waiting_at > 20000 )
      {
          timeout = true;
          break;
      }      
    }
        
    if ( timeout ){tracer.timeouts++;}
    else
    {
      int const transactionId = read_request(tracer);
      radio.stopListening(); 
      write_response(tracer, transactionId);
    }
    
    return !timeout;
}

TIME prevTimeMotion = 0;
TIME prevTimeHeadlights = 0;
static TIME const connectionLostThreshold = 500;
static TIME const headLightsThreshold = 500;
int forward_wise = 0;
int left_wise = 0;
bool lightsOn = false;
void switchLights()
{
  if(lightsOn)
  {
    lightsOn = false;
    digitalWrite(pinHeadlights, LOW);
  }
  else
  {
    lightsOn = true;
    digitalWrite(pinHeadlights, HIGH);
  }
}
void loop() {
    Tracer tiltTracer("TILT");
    bool const motion = communicateWithNode(TILT_RESPONSE, tiltTracer);
    TIME const timeNow = millis();
    //tiltTracer.log(timeNow);   
    if(motion)
    {
      prevTimeMotion = timeNow;
      forward_wise = tiltTracer.roll;
      left_wise = tiltTracer.pitch;
    }
    if(timeNow - prevTimeMotion > connectionLostThreshold)
    {
      motors.stop();
    }
    else
    {
      motors.go(forward_wise, left_wise);
    }
    if(tiltTracer.button && timeNow - prevTimeHeadlights > headLightsThreshold)
    {
      prevTimeHeadlights = timeNow;
#ifdef DEBUGGING      
      Serial.println("button");
#endif
       switchLights();
    }
        
}
