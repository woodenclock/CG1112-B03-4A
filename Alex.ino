#include "serialize.h"
#include "packet.h"
#include "constants.h"
#include "pitches.h"
#include <Adafruit_TCS34725.h>
#include <Wire.h>
#include <math.h>
#include <avr/sleep.h>
#include <Arduino.h>
#include <stdarg.h>

typedef enum {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

// Number of ticks per revolution from the
// wheel encoder.
// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIR
#define COUNTS_PER_REV      100      //48*4 = 192
#define WHEEL_CIRC          20.5

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  11  // Right forward pin
#define RR                  10 // Right reverse pin
#define ALEX_LENGTH  19
#define ALEX_BREADTH 13
//#define PI 3.141592654

//Color Detection
char color;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

//Alex's State Variables
// Store the ticks from Alex's left and
// right encoders for moving forwards and backwards.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//Variable to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

//Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;


//#define PRR_TWI_MASK            0b10000000          //Bit-7 is I2C Change to 0
#define PRR_SPI_MASK            0b00000100
#define ADCSRA_ADC_MASK         0b10000000
#define PRR_ADC_MASK            0b00000001
#define PRR_TIMER2_MASK         0b01000000
#define PRR_TIMER0_MASK         0b00100000
#define PRR_TIMER1_MASK         0b00001000
#define SMCR_SLEEP_ENABLE_MASK  0b00000001
#define SMCR_IDLE_MODE_MASK     0b11110111

// setup power saving
void setupPowerSaving() {
  WDT_off();
  //PRR |= PRR_TWI_MASK;      //To enable I2C for  Sensor
  PRR |= PRR_SPI_MASK;
  PRR |= PRR_ADC_MASK;
  ADCSRA |= ADCSRA_ADC_MASK;
  SMCR &= 0b11110000;         //Sleep not Enabled yet
  DDRB |= 0b00100000;         //Pin 5 is OUTPUT
  PORTB &= 110111111;         //Pin 5 LOW   So that LED tied to Arduino's Pin 13 is OFF
}

// watchdog thing
void WDT_off(void) {
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 0x00;
}

void putArduinoToIdle() {
  PRR |= 0b01101000;                //I2C is Enbled for Colour Sensor
  SMCR |= SMCR_SLEEP_ENABLE_MASK;   //Idle is Enabled
  sleep_cpu();
  SMCR &= 0b11110000;
  PRR &= 0b00010111;                //Changed Bit 7 to enable I2C
}

TResult readPacket(TPacket *packet) {
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

}

void sendStatus() {
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //

  TPacket statusPacket;

  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicks;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  statusPacket.params[10] = color;

  sendResponse(&statusPacket);
}


void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful
  // for debugging.
  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}



void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its
  // command sent to us.
  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;
  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pins 2 and 3
void enablePullups() {
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR() {
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * (float) WHEEL_CIRC);
  }
  else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * (float) WHEEL_CIRC);
  }
  else if (dir == LEFT){
    leftReverseTicksTurns++;
  }
  else if (dir == RIGHT){
    leftForwardTicksTurns++;
  }
}

void rightISR() {
  if (dir == FORWARD) {
    rightForwardTicks++;
    //dbprintf("%s", "RIGHT: ");
    //dbprintf("%ld \n", rightForwardTicks);
  }
  else if (dir == BACKWARD) {
    rightReverseTicks++;
    //dbprintf("%s", "LEFT: ");
    //dbprintf("%ld \n", rightReverseTicks);
  }
   else if (dir == LEFT){
    rightForwardTicksTurns++;
  }
  else if (dir == RIGHT){
    rightReverseTicksTurns++;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT() {
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  cli();
  EICRA = 0b00001010;
  EIMSK = 0b00000011;
  sei();
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect) {
  leftISR();
}

ISR(INT1_vect) {
  rightISR();
}


// Implement INT0 and INT1 ISRs above.

/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()        //////////////////////////////////////////////////
{
  // To replace later with bare-metal.
   Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.
void startSerial()        //////////////////////////////////////////////////
{
  // Empty for now. To be replaced with bare-metal code
  // later on.

}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.
int readSerial(char *buffer) {
    int count=0;
    while(Serial.available())
      buffer[count++] = Serial.read();
    return count;
}

// Write to the serial port. Replaced later with
// bare-metal code
void writeSerial(const char *buffer, int len)
{
  Serial.write (buffer, len);
}

/*
   Alex's motor drivers.

*/

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:
        A1IN - Pin 5, PD5, OC0B
        A2IN - Pin 6, PD6, OC0A
        B1IN - Pin 10, PB2, OC1B
        B2In - pIN 11, PB3, OC2A
  */
  DDRB |= 0b00000100;
  DDRD |= 0b00100000;
  //OC0A = 25;
  //OC0B = 25;

}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors() {

}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed) {
  dir = FORWARD;

  int right_val = pwmVal(speed - 12);
  int left_val = pwmVal(speed);

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;
  newDist = forwardDist + deltaDist;


  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.

  analogWrite(LF, left_val);
  analogWrite(RF, right_val);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed) {

  int right_val = pwmVal(speed - 10);
  int left_val = pwmVal(speed);
  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;
  newDist = reverseDist + deltaDist;

  dir = BACKWARD;

  int val = pwmVal(speed);

  // For now we will ignore dist and
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, left_val);
  analogWrite(RR, right_val);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

unsigned long computeDeltaTicks (float ang) {
  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed) {


  if (ang > 0)
    deltaTicks = computeDeltaTicks(ang)* 0.3;
  else
    deltaTicks = 99999999;

  dir = LEFT;
  int val = pwmVal(speed);
  targetTicks = leftReverseTicksTurns + deltaTicks;
  
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  analogWrite(LR, val);//220
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed) {
//int right_val = pwmVal(speed - 12);
//  int left_val = pwmVal(speed);

 if (ang > 0)
    deltaTicks = computeDeltaTicks(ang) * 0.3;
  else
    deltaTicks = 99999999;
  
  dir = RIGHT;
  int val = pwmVal(speed);
  targetTicks = rightReverseTicksTurns + deltaTicks + 30;
  
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop() {
  dir = STOP;
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
  putArduinoToIdle();

}

/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  leftRevs = 0;
  rightRevs = 0;
  forwardDist = 0;
  reverseDist = 0;
  color = 'N';
}

// Clears one particular counter
void clearOneCounter(int which) {
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState() {
  clearCounters();
}

void handleCommand(TPacket *command) {
  switch (command->command) {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;  
    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
      sendOK();
      stop();
      break;
    case COMMAND_GET_STATS:
      sendOK();
      dbprintf("GET STATUS");
      check_colour();
      sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;

   case COMMAND_PLAY:
      sendOK();
      dbprintf("Playing Song");
      play();
      break; 
      
    default:
      sendBadCommand();
  }
}

void check_colour() {
  float r, g, b, c;
  tcs.getRGB(&r, &g, &b);
      
  if (r > g && r > b)//(r >= 110 && g <= 80 && b <= 60)  //&& g <= 50 && b <= 50)
     color = 'R';
  else if ((g - 10 >= r && g >= b)) //&& r <= 50 && b <= 50)
     color = 'G';
   else //if ((r >= 80) && (g >= 88 && g <= 91) && (b >= 62 && b <= 65))
     color = 'N';


}



void play() {
  
  int melody[] = {
  NOTE_G4, 0, NOTE_G4, NOTE_D5,
  NOTE_C5, 0, NOTE_AS4, 0,
  NOTE_A4, 0, NOTE_A4, NOTE_A4,
  NOTE_C5, 0, NOTE_AS4, NOTE_A4, 
  NOTE_G4,0, NOTE_G4, NOTE_AS5,
  NOTE_A5, NOTE_AS5, NOTE_A5, NOTE_AS5,
  NOTE_G4,0, NOTE_G4, NOTE_AS5,
  NOTE_A5, NOTE_AS5, NOTE_A5, NOTE_AS5,

  NOTE_AS4, NOTE_AS4, NOTE_AS4, NOTE_AS4,
  NOTE_D5, NOTE_D5, NOTE_D5, NOTE_D5,
  NOTE_C5, NOTE_C5, NOTE_C5, NOTE_C5, 
  NOTE_F5, NOTE_F5, NOTE_F5, NOTE_F5, 
  NOTE_G5, NOTE_G5, NOTE_G5, NOTE_G5,
  NOTE_G5, NOTE_G5, NOTE_G5, NOTE_G5, 
  NOTE_G5, NOTE_G5, NOTE_G5, NOTE_G5, 
  NOTE_C5, NOTE_AS4, NOTE_A4, NOTE_F4
 };
 
  // note durations: 4 = quarter note, 8 = eighth note, etc.:
  int noteDurations[] = {
    4,4,4,4,
    4,4,4,4,
    4,4,4,4,
    4,4,4,4,
    4,4,4,4,
    4,4,4,4,
    4,4,4,4,
    4,4,4,4,
  
//    4,4,4,4,
//    4,4,4,4,
//    4,4,4,4,
//    4,4,4,4,
//    4,4,4,4,
//    4,4,4,4,
//    4,4,4,4,
//    4,4,4,4
  };
  
  for (int thisNote = 0; thisNote < 32; thisNote++) {
    int noteDuration = 500 / noteDurations[thisNote];
    tone(12, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(12);
  }
}

void waitForHello() {
  int exit = 0;

  while (!exit) {
    TPacket hello;
    TResult result;
  
    do {
      result = readPacket(&hello);
    } 
    while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK) {
      if (hello.packetType == PACKET_TYPE_HELLO) {
        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD) {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}



void setup() {
  // put your setup code here, to run once:
  // tcs.begin();
  //Compute the diagonal
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;
  cli();  
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  setupPowerSaving();
  startMotors();
  enablePullups();
  initializeState();

  PRR &= 0b01111111;            //Enabled I2C
  sei();
}



void handlePacket(TPacket *packet) {
  switch (packet->packetType) {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
  TPacket recvPacket; // This holds commands from the Pi
  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK) {
    handlePacket(&recvPacket);
  }
    
  else if (result == PACKET_BAD) {
    sendBadPacket();
  }
  
  else if (result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }
  
  if(deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if(dir==BACKWARD) {
      if(reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }

  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }    
}
