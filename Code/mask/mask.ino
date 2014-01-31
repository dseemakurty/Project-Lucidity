#include <SoftPWM.h>
#include <SPI.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <RFM12B.h>

//**********************************************************************************//

#define numRedLeds 4
#define ledPin1 3
#define ledPin2 4
#define ledPin3 7
#define ledPin4 8
#define statusLED A0
int LEDS[numRedLeds] = {
  ledPin1,ledPin2,ledPin3,ledPin4};
int startOn[numRedLeds]; // the time in ISR time units when the LEDs should activate

#define daylightPin1 5
#define daylightPin2 6
#define daylightMaxBrightness 255 // the maximum brightness the LEDs can reach
#define daylightIncreaseInterval 1 // how much the PWM should increase per refresh (out of 255)
#define daylightInterval 2 // (255*(600ms/1000ms))/60s = 2.55 minutes
int daylightRefreshIntervalT; //interval between refreshes in terms of ISR time units, function of the daylightInterval and settings assigned by user
int daylightBrightness; //brightness of the daylight simulation as a function of received setting
int daylightFadeLevel = 0; // the fade level of the daylight LEDs

// @16Mhz overflow of 155 = 10ms :::: @8Mhz overflow of 78 = 10ms
#define timerOverflow 78 // this overflow amount on the 16bit Timer1 will create 10ms overflows
#define interruptInterval 10 //one ISR time unit is this many milliseconds based on clock calculations

//fade speeds for the different patterns
#define flashAllFadeTime 200
#define sweepFadeTime 200
#define alternatingFadeTime 200
#define doubleFlashFadeTime 50
int sweepDelay; //delay between LED's on the sweep pattern
double patternBrightness; // will be modified by received settings
double patternSpeed; // will be modified by received settings

int fadeTime; // how long the fading process should take in ms
int fadeT; // fadeTime in ISR time units
double redBrightnessPercent; // will be modified by various settings, the actual brightness of the red LEDs
double redFadeTime; //the time sent to the SoftPWM library incorporating brightness level and actual fadeTime
#define patternRefillTime 30000 // how long a pattern will project after receiving the signal
#define previewPatternRefillTime 3000
int patternDisplayTime; // how long a pattern should be displayed, must be refilled with time after use
int patternT = 0; // patternDisplayTime in ISR time units

volatile long redT = 0; // ticks every timer overflow
volatile long daylightT = 0; // ticks only when runDaylight boolean is enabled

boolean runDaylight = false; //triggers Daylight Simulation
boolean flashAllP = false; //triggers all LEDs to flash pattern
boolean sweepRightP = false; // triggers sweep pattern (ONLY ENABLE RIGHT OR LEFT TO START SWEEP)
boolean sweepLeftP = false; // triggers sweep pattern (ONLY ENABLE RIGHT OR LEFT TO START SWEEP)
boolean alternatingFlashP = false; // triggers flashing of each eye's LEDs alternatingly
boolean doubleFlashP = false; // double flash then delayLong
int longCount = -1;
int delayLong;
boolean *patternPointer; // boolean to point to one of the above booleans (used in updateReds() method)
int resetT = -1; // when the pattern is finished one cycle in ISR time units

#define NODEID 1
#define NETWORKID 99
unsigned int msg; // the message that will be received from the tranceiver
RFM12B radio;
boolean keepListening = true; // 
long startListeningTime;
#define listenTimeout 200 //how long to listen before timeing out

// Watchdog timeout values
// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
#define wdTime 9 // watchdog timeout value
#define tempwdTime 6
#define wdMultiple 3 // how many times to execute wdTime (gets larger sleep times)
#define tempwdMultiple 1 // how many times to execute wdTime (gets larger sleep times)
#define idleTimeout 4320 //multiply by wdTime(ms value) and wdMultiple to get the time of the timeout, will put mask into deep sleep without WDT
#define tempIdleTimeout 60 // preview timeout allows for quick polling (set for 1 minute)
long idleTimer = 0; // counter that is compared to the sleepTimeout
volatile boolean f_wdt = 0; // WDT int
byte adcsra;

void setup()
{
  SoftPWMBegin();
  Serial.begin(115200);
  pinMode(statusLED,OUTPUT);
  setupTimerInterrupt();
  radio.Initialize(NODEID,RF12_433MHZ,NETWORKID);
  Serial.println("Listening..");
}

// sets up all the calculations around properly fading the LEDs
void setupSoftPWM(int fadetime, double brightness)
{
  fadeTime = fadetime;
  fadeT = fadeTime/interruptInterval;
  redBrightnessPercent = brightness;
  redFadeTime = fadeTime*(100/redBrightnessPercent); // recalculates the time for the SoftPWM library since library times things differently
  sweepDelay = fadeT/2;
  delayLong = fadeT*10;
  SoftPWMSet(ledPin1, 0);
  SoftPWMSetFadeTime(ledPin1, redFadeTime, redFadeTime);
  SoftPWMSet(ledPin2, 0);
  SoftPWMSetFadeTime(ledPin2, redFadeTime, redFadeTime);
  SoftPWMSet(ledPin3, 0);
  SoftPWMSetFadeTime(ledPin3, redFadeTime, redFadeTime);
  SoftPWMSet(ledPin4, 0);
  SoftPWMSetFadeTime(ledPin4, redFadeTime, redFadeTime);
}

// sets up timer 1 with 10ms overflow
void setupTimerInterrupt()
{
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = timerOverflow; // set to overflow ever 10ms
  TCCR1B |= (1 << WGM12); // turn on CTC mode:
  TCCR1B |= (1 << CS10);  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt:
  sei();          // enable global interrupts
}

void analyzeMessage(unsigned int msg)
{

  double brightSetting = msg%10;  // the ones digit of the int received is the brightness setting
  msg = msg/10;                   // remove ones digit
  double speedSetting = msg%10;   // the previously tens digit is speed setting
  msg = msg/10;                   // remove ones digit
  int patternSetting = msg;       // the rest is the pattern indicator

  if (patternSetting == 1){
    daylightBrightness = daylightMaxBrightness * (brightSetting+1)/10;
    daylightRefreshIntervalT = daylightInterval * (speedSetting+1);
    runDaylight =true;
  }
  if (patternSetting == 2){
    patternBrightness = (brightSetting+1)*10;
    patternSpeed = flashAllFadeTime*10/(speedSetting+1);
    if (idleTimer<=tempIdleTimeout)
      setPattern(&flashAllP,previewPatternRefillTime,patternBrightness,patternSpeed);
    else setPattern(&flashAllP,patternRefillTime,patternBrightness,patternSpeed);
  }
  else if (patternSetting == 3){
    patternBrightness = (brightSetting+1)*10;
    patternSpeed = sweepFadeTime*10/(speedSetting+1);
    if (idleTimer<=tempIdleTimeout)
      setPattern(&sweepRightP,previewPatternRefillTime,patternBrightness,patternSpeed);
    else setPattern(&sweepRightP,patternRefillTime,patternBrightness,patternSpeed);
  }
  else if (patternSetting == 4){
    patternBrightness = (brightSetting+1)*10;
    patternSpeed = alternatingFadeTime*10/(speedSetting+1);
    if (idleTimer<=tempIdleTimeout)
      setPattern(&alternatingFlashP,previewPatternRefillTime,patternBrightness,patternSpeed);
    else setPattern(&alternatingFlashP,patternRefillTime,patternBrightness,patternSpeed);
  }
  else if (patternSetting == 5){
    patternBrightness = (brightSetting+1)*10;
    patternSpeed = doubleFlashFadeTime*10/(speedSetting+1);
    if (idleTimer<=tempIdleTimeout)
      setPattern(&doubleFlashP,previewPatternRefillTime,patternBrightness,patternSpeed);
    else setPattern(&doubleFlashP,patternRefillTime,patternBrightness,patternSpeed);
  }
}

void listenMessage()
{
  startListeningTime = millis();
  keepListening = true;
  while(millis()-startListeningTime<200 && keepListening) // listen for data for set time
  {
    if (radio.ReceiveComplete())
    {
      keepListening = false;
      if (radio.CRCPass())
      {
        char buffer[16]; // create character array
        for (byte i = 0; i < radio.GetDataLen(); i++)
          buffer[i] = (char)radio.Data[i];  // populate character array with data array
        int msg = atoi(buffer); // convert char array into an integer
        Serial.println(msg);
        radio.SendACK(); // send acknowledgement packet
        analyzeMessage(msg);
      }
    }
  }
}

void setPattern(boolean *patternpointer, int time, int brightness, int fadeSpeed)
{
  patternDisplayTime = time; // sets how long the pattern will be shown
  patternT = patternDisplayTime/interruptInterval; // recalculates time in ISR time units
  setupSoftPWM(fadeSpeed,brightness); // adjusts the SoftPWM settings
  *patternpointer = true; // activates the boolean that is sent to the method
}

void updateDaylight()
{
  if (daylightFadeLevel<=daylightBrightness){
    daylightFadeLevel+=daylightIncreaseInterval; // increase the PWM of the daylight LEDs
    analogWrite(daylightPin1, daylightFadeLevel);
    analogWrite(daylightPin2, daylightFadeLevel);
  }
  else{
    analogWrite(daylightPin1, 0); // turn off daylight LEDs
    analogWrite(daylightPin2, 0);
    daylightFadeLevel = 0;
    runDaylight = false; // technically unnecessary
    powerDown(); // after a daylight simulation, put the mask into deep sleep without WDT, requires manual reset
  }
}

void updateReds()
{
  for(int i = 0; i < numRedLeds;i++){
    if (redT == startOn[i]){ // iterates through all the LEDs and updates them based on the ISR time units
      SoftPWMSetPercent(LEDS[i],redBrightnessPercent);
    }
    else if (redT == startOn[i]+fadeT){ // turn off LEDs if they have reached peak brightness
      SoftPWMSetPercent(LEDS[i],0);
    }
  }
  if (redT==resetT){
    *patternPointer = true; // the value of the boolean that patternBoolean is addressed to is equal to True
    patternT-=resetT; // subracts the time for pattern to complete one cycle from the allotted display time
  }
}

void loop()
{

  if (f_wdt == 1){
    f_wdt=0;
    wdt_disable();
    
    radio.Wakeup();
    listenMessage();
    radio.Sleep();
  }
  else if (patternT<=0 && runDaylight == false){ // if freshly booted, poll at faster rate
    if (idleTimer<=tempIdleTimeout){
      setup_watchdog(tempwdTime);
      idleTimer++;
      timedSleep(tempwdMultiple);
    }
    else if (idleTimer<=idleTimeout){ //if activity timeout hasnt occured, enable watchdog and sleep
      setup_watchdog(wdTime);
      idleTimer++; // iterate idle timer
      timedSleep(wdMultiple); // put MCU to sleep
    }
    else{ // if timeout has occured, go into sleep without interrupts. can only be woken by reset button
      powerDown();
    }
  }

  // check all the pattern booleans to display pattern if one is valid
  if (flashAllP){
    redT=0;
    startOn[0] = redT+10;
    startOn[1] = redT+10;
    startOn[2] = redT+10;
    startOn[3] = redT+10;
    resetT = redT+10+fadeT*2; // the last possible redT that this pattern requires
    patternPointer = &flashAllP; //patternBoolean is the address of flashAll
    flashAllP = false;
    redT=0;
  }

  else if (sweepRightP){
    redT=0;
    startOn[0] = redT;
    startOn[1] = redT+sweepDelay;
    startOn[2] = redT+sweepDelay*2;
    startOn[3] = redT+sweepDelay*3;
    resetT = redT+sweepDelay*3+fadeT*2; // the last possible redT that this pattern requires for one cycle
    patternPointer = &sweepLeftP; //patternBoolean is the address of left sweep so sweep continues
    sweepRightP = false;
    redT=0;
  }

  else if (sweepLeftP){
    redT=0;
    startOn[3] = redT;
    startOn[2] = redT+sweepDelay;
    startOn[1] = redT+sweepDelay*2;
    startOn[0] = redT+sweepDelay*3;
    resetT = redT+sweepDelay*3+fadeT*2; // the last possible redT that this pattern requires for one cycle
    patternPointer = &sweepRightP; //patternBoolean is the address of right sweep so sweep continues
    sweepLeftP = false;
    redT=0;
  }

  else if (alternatingFlashP){
    redT=0;
    startOn[0] = redT;
    startOn[1] = redT;
    startOn[2] = redT+fadeT*2;
    startOn[3] = redT+fadeT*2;
    resetT = redT+fadeT*4; // the last possible redT that this pattern requires for one cycle
    patternPointer = &alternatingFlashP; //patternBoolean is the address of alternating flash
    alternatingFlashP = false;
    redT=0;
  }

  else if (doubleFlashP){
    if (longCount == 1){ // after two blinks, take long pause
      longCount = 0;
      redT=0;
      startOn[0] = redT+delayLong+10; // delayLong is a multiple of the fade
      startOn[1] = redT+delayLong+10;
      startOn[2] = redT+delayLong+10;
      startOn[3] = redT+delayLong+10;
      resetT = redT+delayLong+fadeT*4; // the last possible redT that this pattern requires for one cycle
    }
    else { 
      longCount++;
      redT=0;
      startOn[0] = redT+10;
      startOn[1] = redT+10;
      startOn[2] = redT+10;
      startOn[3] = redT+10;
      resetT = redT+fadeT*4; // the last possible redT that this pattern requires for one cycle
    }
    patternPointer = &doubleFlashP; //patternBoolean is the address of double flash
    doubleFlashP = false;
    redT=0;
  }
}

// Main timer that allows for smooth updating of patterns, interrupts ever 10ms
ISR(TIMER1_COMPA_vect)
{
  if (runDaylight){
    daylightT++;
    if (daylightT >= daylightRefreshIntervalT)
    {
      updateDaylight();
      daylightT = 0;
    }
  }
  if (patternT>0){
    updateReds();
    redT++;
  }
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
  digitalWrite(statusLED,LOW);
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
  digitalWrite(statusLED,LOW);   // turn LED off to indicate sleep
  //enables software disabled BOD - means that BOD is disabled during sleep and is reenabled during wake up
  // cli(); // disable interrupts to softBOD disable
  MCUCR |= bit(BODSE) | bit(BODS); // timed sequence
  MCUCR = (MCUCR & ~bit(BODSE)) | bit(BODS);
  // sei(); // reenable interrupts to ensure wakeup from sleep
  sleep_cpu(); // puts the mcu to sleep
  // **SLEEP** //
  sleep_disable();   // Upon waking up, sketch continues from this point.
  ADCSRA = adcsra; // renable ADC
  digitalWrite(statusLED,HIGH);   // turn LED on to indicate awake
}

// Watchdog Interrupt Service is executed and volatile switch is turned on
ISR(WDT_vect){
  f_wdt = 1;
}

//*********************************************************************************************//
//*********************************************************************************************//




