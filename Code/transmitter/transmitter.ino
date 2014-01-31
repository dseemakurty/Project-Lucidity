
#include <SPI.h>
#include <RFM12B.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/wdt.h>

//**********************************************************************************//

#define audioInPin A0 //input of audio signal from OpAmp
#define ampPin A1 // power pin of opAmp
#define silenceBarrier 200 //value under which silence is counted

//**if you change any of the bit values, the signal noises must be REDONE! ** WILL CAUSE ERRORS IF YOU DONT
#define oneBit 1000 // the micros time that signifies a 1
#define zeroBit 2000 //the micros time that signifies a 0
#define nullBit 3000 // the micros time that signifies a null, generally the end of a message
#define bitAccuracy 400 // the +/- of each of the microsecond times of the bits
long normalizer = 0; // gets rid of noise in audio data

#define alarmMsg 100 // predetermined message to receive from audio signal to denote alarm procedure
#define analyzeSamples 15 // how many messages to check at once per anaylzeData() call
#define continuityCheck 5 // how many messages must be same and continuous in order to transmit msg

//#define normalizerSamples 10000 // how many samples to take for noise reduction


#define NODEID 2
#define NETWORKID 99
#define GATEWAYID  1  //the node ID we're sending to
#define ACK_TIME 50  // # of ms to wait for an ack
unsigned int msg; // the message that will be received from the tranceiver
RFM12B radio;

boolean previewMode = true;
#define previewTransmitTimeout 2000 // how long to tansmit if acknowledgement is not received during preview mode
#define transmitTimeout 30000 // how long transmit should last if acknowledgment is not received
int transmitterTimeout = transmitTimeout;
// Watchdog timeout values
// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
#define wdTime 9 // watchdog timeout value
#define tempwdTime 5
#define wdMultiple 2 // how many times to execute wdTime (gets larger sleep times)
#define tempwdMultiple 1 // how many times to execute wdTime (gets larger sleep times)
#define idleTimeout 2440 //multiply by wdTime(ms value) and wdMultiple to get the time of the timeout, will put mask into deep sleep without WDT
#define tempIdleTimeout 120 // preview timeout allows for quick polling (set for 1 minute)
long idleTimer = 0; // counts how long the transmitter was idle
volatile boolean f_wdt = 0; // keeps track of WDT wakeups
byte adcsra;

void setup(){
   
  analogReference(INTERNAL); // allows for easier readings from audio
  Serial.begin(115200);
  Serial.println("hey");
  radio.Initialize(NODEID,RF12_433MHZ,NETWORKID);
  
  pinMode(ampPin, OUTPUT);
  pinMode(audioInPin,INPUT);
  radio.Sleep();
   
} 

void loop(){
  if (f_wdt == 1){
    f_wdt=0;
    wdt_disable();
    digitalWrite(ampPin,HIGH);
    analyzeBits(500); // check to see if any audio signal recieved, timeout at 200ms
    digitalWrite(ampPin,LOW);
  }
  else{
    if (idleTimer<=tempIdleTimeout){
      setup_watchdog(tempwdTime);
      idleTimer++;
      timedSleep(tempwdMultiple);
    }
    else if (idleTimer<=idleTimeout){ //if activity timeout hasnt occured, enable watchdog and sleep
      previewMode = false;
      transmitterTimeout = transmitTimeout;
      setup_watchdog(wdTime);
      idleTimer++;
      timedSleep(wdMultiple);
    }
    else{ // if timeout has occured, go into sleep without interrupts. can only be woken by reset button
      powerDown();
    }
  }
}

// maximum value that can be transmitted is 16bits = max value of 65535
void transmitData(char msg[])
{
  long startTime = millis();
  radio.Wakeup();
  Serial.println("transmitting");
  while (millis()-startTime<transmitterTimeout && !waitForAck())
  {
    radio.Send(GATEWAYID,msg,sizeof(msg)+1,true);
  } // try until acknowledgement is received or timeout at 2s
   Serial.println("finished");
   delay(100);
  radio.Sleep();
}

static bool waitForAck() {
  long now = millis();
  while (millis() - now <= ACK_TIME)
    if (radio.ACKReceived(GATEWAYID))
      return true;
  return false;
}

void analyzeBits(int timeOut){
  long startTime = millis();
  int continous = 0;
  int breakCount = 0;
  unsigned int msg = 0;
  while (msg==0 && millis()-startTime<timeOut){ // ensures that msg is a valid reading and has timeout
    msg = getBits();
  }
  long check = 0; 

  while(millis()-startTime<timeOut) // checks messages for continouity, has timeout
  {
    while(check==0 && millis()-startTime<timeOut) // ensures that check if a valid reading
      check = getBits();
    if (check == msg){ // tallies how many continous consistant readings there are
      continous++;
      startTime = millis(); // if valid readings are obtained, extend the timeout
    }
    else{
      msg = check;
      continous = 0;
      startTime = millis(); // if valid readings are obtained, extend the timeout
    }
    if (continous >= continuityCheck){ // if there are enough consistent readings, it must be accurate
      
      String message = String(msg);
      char cstr[16];
      String(msg).toCharArray(cstr,16);
      Serial.println(cstr);
      Serial.println(sizeof(cstr));
      transmitData(cstr);
      
      return; 
    }
  }
  return;
}

// read the bits from the signals and create an integer out of them
long getBits(){
  unsigned int message = 0;
  boolean contRead = true;
  int z = readSignal();
  if (z==-1){ //ensure that reading doesnt begin during middle of message
    while (contRead){
      z = readSignal(); // read the bit
      if (abs(z-oneBit) < bitAccuracy){ // check if its a 1
        message = message << 1; // shift bits over 1
        message += 1<<0; // place 1 at last bit
      }
      else if (abs(z-zeroBit) < bitAccuracy){ // check if its a 0
        message = message << 1; //shift bits over 1
        message += 0<<0; //place 0 at last bit
      }
      else if (z == -1){
        contRead = false;
        if (message > 0){
          return(message); // if valid message, return it
        }
        else return(0);
      }
    }
  }
  return 0;
}


int readSignal(){
  int outputValue;
  int startTime;
  int totalTime;
  startTime = 0;
  totalTime = 0;                 
  outputValue = analogRead(audioInPin); // read the value of the audio pin
  if (outputValue < silenceBarrier){ 
    startTime = micros(); // once the audio signal becomes close to silent, start timer
    while (outputValue < silenceBarrier){ // continue timer as long as audio signal is silent
      totalTime = micros();
      outputValue = analogRead(audioInPin); 
      if (totalTime-startTime > nullBit-bitAccuracy) // if signal is silent for too long, its a null pulse
        return -1;
    }
    totalTime = totalTime - startTime; // calculate total time and return it
  }  
  //Serial.println(totalTime);
  return(totalTime);
}


//*********************************************************************************************//
//*********************************************************************************************//

// allows for longer sleep times which are multiples of wdTime
void timedSleep(int waitTime)
{
  int waitCounter = 0;
  while (waitCounter != waitTime) 
  {
    enterSleep();
    waitCounter++;
  }
}

// puts the mC to power down sleep and has no interrupt or watchdog, can only be awoken by reset
void powerDown()
{

  enterSleep();
}


//*********************************************************************************************//
//*********************************************************************************************//

//sets up the watchdog to the correct sleep time
void setup_watchdog(int ii) 
{  
  // The prescale value is held in bits 5,2,1,0
  // This block moves ii itno these bits
  byte bb;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);

  // Reset the watchdog reset flag
  MCUSR &= ~(1<<WDRF);
  // Start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set new watchdog timeout value
  WDTCSR = bb;
  // Enable interrupts instead of reset
  WDTCSR |= _BV(WDIE);
}

void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Choose our preferred sleep mode:
  sleep_enable();    // Set sleep enable (SE) bit::
  adcsra = ADCSRA; // save ADCSRA settings
  ADCSRA = 0; // disable ADC
 
  //enables software disabled BOD - means that BOD is disabled during sleep and is reenabled during wake up
  MCUCR |= bit(BODSE) | bit(BODS); // timed sequence
  MCUCR = (MCUCR & ~bit(BODSE)) | bit(BODS);
  sleep_cpu(); // puts the mcu to sleep
  // **SLEEP** //
  sleep_disable();   // Upon waking up, sketch continues from this point.
  ADCSRA = adcsra; // renable ADC

}

// Watchdog Interrupt Service is executed and volatile switch is turned on
ISR(WDT_vect){
  f_wdt = 1;
}

//*********************************************************************************************//
//*********************************************************************************************//


