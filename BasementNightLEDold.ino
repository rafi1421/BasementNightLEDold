#define DEBUG true
#define DEBUGreading true
#define DEBUGerror true
///////////////////////////////////////////////////////////////////////////////////////////////////////
///
/*
  ..::Basement LED Project::..
    ~~Raphael Codrean
  
  This project was made because frequently whenever i go back upstairss to bed or when i have friends 
  over at night and someone needs to go to the bathroom they walk down the hallway in complete darkness 
  and it is impossible to see and i would have to use my phone as a flashlight.
  So the purpose of this project is go give light when needed. it will sense you walk past the PIR sensors
  [not using  anymorelinebreak sensor(ultrasonic) , using PIR so now it can sensor farther, and in 180 degree
  sphere detection] and then it will turn on the lights for about a minute and shut back off. it will 
  shut down the power and sensors and run in sleepmode durring the daytime to save energy.

before aug 1-  a million things
aug 1: 
-removed the last tv sonic sensor and replaced with PIR because for some reason when the fixture was connected by itself, the tv sonic sensor kept triggering, and when the bag was over it, it would trigger probably cause it read 0 cm.  but when its connected to pc it worked fine.  so decided to swap to pir in heat of moment and turned out better
aug 2: removed a lot of unnessary things now that we are using pir sensors
-removed the sleepType variable because the clock is not that crucial for these sensors as was the sonic sensor was time and quickness dependant
-removed isSleeping because that was used for the "sleep for 5 minutes then check brightness." but not needed anymore because if a basement light turns turns on during the night and it enters that nap mode, then once its gonna stay n nap mode for 5 min and wont be able to trigger. and since the chip has to wake up eitherway let it do its scan and find the brightness.
*found bug where the clock would run fast after pressing the button. its because you held it for longer than the delay bounce time when the led is already on, so releasing the trigger would cause a confusion and it would turn off, but pressing it again would reset the settoigs and then you'd have to press it once again to turn it back on.  so to fix, initially delay it long but then if youd want to switch modes quickly youd be confused and break it on purpose. so to fix that, i added a different indicator when switching to strobe lights so then you know it happened.
+readded sleeptype because of problem. relay will switch off after a few seconds. current seems unstable. before it would stay at 1.4 mA. with pwrdown it wouuld flucuate between 1.4-3mA ever couple seconds

october: - changed light thresholds for less shadow triggering during ping pong.
dec 24 -2014: - removed lightOn +changed to allow to turn on and off the sensors by using the same framework as lightOn since the chip goes to sleep manually.
         so i set it up for one mode to be both lights on and the second mode to give a quick signal then turn off till you press button again. -allows me to not need to remove plug while playing ping pong
        + also added alarm reset when pushing the button since its a given that i am present and any errors could be reset.  because usualy by sun down light is tweaky so it keeps turning on and off since the sensors are wired positive trigger transistor. so it detects an auto on when lights turn on.
         tho it actually works to my advantage when turning off the lamps at night, gives it an auto on without needing to triger. also saves the fact i dont need to reset by pulling the plug
oct-16-2015: added a auto alarm-reset after a certain time so i dont have to push the button when a sensor turns off. 
oct-17-2015: made it so the lights will have a 10 second PIR check before the lights turn off so they dont turn off if the person is still walking. idealy it would be better if
the the pir check was longer and the lighttime was shorter, that way it would constantly check if ppl are still walking and when it doesnt see anything it was turn off after that grace period of allowance. maybe we do it later if this works well.
cause now it turn on for another whole 30sum seconds when it seees someone at the last second. idk well see.

july-5-2016: hey hey whats up again. here to make another improvment. so trying to sleep in the basement and the light suddenly click on scares me so im making a way to gradually brighten up.
			also to implement the one idea the project manager from RTC had to use a power transistor instead of power resistors.
			so things gonna add. tip120's (may change them later to better transistors if voltage drop is bad) and a potentiometer to change brightness(pwm to 120's)

august 22 2016: ruben said it takes too long for the sensor from the stairs to detect him and asked if it was possible to add another one so its better.
     I still had the phone jack wired together so i tested to see how the a sensor over there works and it works really well!! immediatly when walking out of the door it turns on!
sept 28 2016: bug: when led turns on and then the actual basement lights turn on, and it stay on longer than the 30 second cut off limit, the program glitches and doesnt turn off.
dec-15-16 damage? tele and bath sensors flickeer between on and off so i disabled them
dec-16-16 okay idk what happened maybe a wire got crossed temporarly or idk but it works again. added a cap to tele pin cause that was finiky, but i guess it was for the 
same reason as the bath sensor. even shoirted out the transistor switch cause i thought that would have messed it but now i realize it would mess all of them. 
the bath sensor was super hot too, but i switched it with the tele and then that signal was working fine so that means the pir was not broken and it worked fine after that too.
then i saw a peice of wood was on the wire so i moved it out of the way and all of a sudden it works again. well not yet. i also un taped the bath wire where the purple connector was
and re tapped it. then it started working. ya idk. cause when the purple connector was not plugged in, then i get 0's, then when i plug it ion, the bath was on even with no sensor connected. 
so it had to be a crossing wire or something like that. at least its working again.
*/



/* connections: 
 *  usb: 
 *  red= blue =  reset = pin 1
 *  white =yellow = rx = pin2
 *  green = green = tx = pin 2
 *  black = gray = gnd = pin gnd
pin 2 : light sensor
pin 3: manual button
pin 4: bathroom PIR sentton inturrupt - normally lsor to purple connector pin 4 (replace ground as is currently.
see how the sensor reacts for the first minute on the sensor pin)
pin 5: bath relay/led / now a tip120 transistor
pin 6: stair relay/led  / now a tip120 transistor
pin 7: bath sesnor [not anymore]
pin 8: stair sensor (replace with PIR)
pin 9: tv sensor
pin 10: NPN 3904 / PNP 3906 transistor for sensor switch
pin 11: piezo/led
pin 12: optional program debug switch to upload //althoughi never needed it?
pin 13: debug led



*/




///////////////////////////////////////////////////////////////////////////////////////////////////////

// powersaving by gammon  http://www.gammon.com.au/forum/?id=11497

// [ping ultrasonic sensor section]
// ---------------------------------------------------------------------------
// This example code was used to successfully communicate with 15 ultrasonic sensors. You can adjust
// the number of sensors in your project by changing SONAR_NUM and the number of NewPing objects in the
// "sonar" array. You also need to change the pins for each sensor for the NewPing objects. Each sensor
// is pinged at 33ms intervals. So, one cycle of all sensors takes 495ms (33 * 15 = 495ms). The results
// are sent to the "oneSensorCycle" function which currently just displays the distance data. Your project
// would normally process the sensor results in this function (for example, decide if a robot needs to
// turn and call the turn function). Keep in mind this example is event-driven. Your complete sketch needs
// to be written so there's no "delay" commands and the loop() cycles at faster than a 33ms rate. If other
// processes take longer than 33ms, you'll need to increase PING_INTERVAL so it doesn't get behind.
// ---------------------------------------------------------------------------


/*
[manual light button working code]
so the purpose of this code is so that when we manually press a button 
using the interupt button pin #3, it will a) turn on the light. and b) if held down
it will switch between having both led strips on, to just the left side, or just the right side

//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

var = !var 
uses more 10 more bytes than
var = true
*/

/* usage:
    #if DEBUG 
    Serial.println("  "); 
    #endif // DEBUG
*/






#include <avr/sleep.h>
#include <avr/wdt.h>
boolean watchdog = true;  //boolean for watchdog(true) or sleep till light/button(false)
//volatile boolean isSleeping = true; //for 10 min sleep
boolean sleepType = false; //false is pwr down. true is idle(keeps clock on)


/*
  waitTime = WDT_8_SEC;
  myWatchdogEnable (waitTime);
     //end of loop
*/

///////////////////////////////////////////////
/// Sensor Settings  (not unnseesary) 
///////////////////////////////////////////////

///the first sensor should be the stairs and the last sensor should be the tv since it is the last to be scan and will immediate turn on the lights with no lag.
/*
#--include <NewPing.h>  //if i ever need it back: earse the --
//28.8 cm per foot. 86.4 cm per meter
#define SONAR_NUM     1 // Number of sensors.
//#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
//#define bathSenDist 300 //max cm distance of bathroom sensor
#define tvSenDist 200  //max dist set
//#define stairSenDist 400 //ditto

//sensor stuff

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  //NewPing(stairSen, stairSen, stairSenDist),  // sensor 3 (stairs) (8 meter?)
  //NewPing(bathSen, bathSen, bathSenDist), //sensor 1 ( bathroom) (9 meters?) // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(tvSen, tvSen, tvSenDist) // sensor 2 (tv room) (2 meter?)
  
};


*/

///////////////////////////////////////////////
/// Pin Configureation
///////////////////////////////////////////////
int potPin = A4;  //potentiometer for controlling the brightness of led's which will pwm to the transistor pins
int potValue = 0;  // variable to store the value coming from the potentiometer
int rampOnValue = 0;  //value of pwm to ramp up led's on smoothly
int justOn = 0; // Which Led was just on? 0 = off, 1 =  tv, 2 = stair, 3 = bath
#define RampSpeedSlow 38  //delay amount to transistionj from off to on
#define RampSpeedMed 16
#define RampSpeedFast 10
int lightSen = A5; // anaglog pin 1 for light sensor
int lightSenValue;  // variable to store the value coming from the sensor
#define PIRtele 11 //PIR for ruben's room connected via initial telephone port
#define PIRbath 4  //pir for bath  //fish cat5 wire thru hole and new connections
#define PIRstair 8  //pir for stairs (angle more downward to the left
#define PIRtv 9
//#define bathSen 7  //bathroom sensor pin
//#define stairSen 8 //stairs sensor pin
//#define tvSen 9   //TV sensor pin
#define bathRelay 6  //relay pin for bathroom Led strip
#define stairRelay 5 //relay pin for stair Led strip
#define senRelay 10 //relay pin to power on the sensors when it is dark. but i think its really a transistor
//#define alarmPin 11  //buzzer for error //pwm

//#define debugswitch 12  //took it out cause never need it?
#define debugled 13



///////////////////////////////////////////////
/// Time fore light on
///////////////////////////////////////////////


//with 3 sensors 

//clock takes about 6637 per minute
//3 minutes 20000
//4 minutes 26532
//5 min 33200
//6 min 40000

/*
with 1 sensor: at 256 sleep
30 sec: 3400
1 minute:6800
2 minute: 13400
3 minute:20000
4 minute:26500
5 minute: 33240
10 minute: 66300

with 1 sensor: at 512 sleep
30 sec: 1825
1 minute: 3560
2 minute: 7137
3 minute: 10600
4 minute:15125
5 minute: 
10 minute: 

with only PIR and 1 sec sleep +30ms delay for serialprint
its litterally 31 ms between every scan. does this mean each clock is like 1 ms? yees
and that for the time i should just put like 40. maybe
10 sec: 300
30 sec: 887
1 minute: 1716
2 minute: 3400
3 minute: 5090
4 minute: 6747



*/
#define lightTime (30 * 1000) //2500UL  //about 45 seconds // sets the Led strip ON time to 1 minute
#define alarmTimeLimit ((lightTime * 4) + 9)  //set to 4 min + some extra

//so playing around i found out light to dark should be lower than dark to light.
//this allows a little play sleep area if the chip is awaken and it is not completely dark yet but not light enough either. so it will sleep until it is dark enough
#define lightLD 580  //580 for original small sensor  //400 //light to dark  //at home, 400/500 is a good time wake up.  appaerentky 580 is better
#define lightDL 650  //650 for original small sensor //dark to light   //not sure yet what is a good to sleep at.  apparently 650 is the best
// 300-400
//900-1000 //950




unsigned long comaTimer;
boolean comaTimerAct = false;
unsigned long ledTimer;
boolean ledTimerAct = false;

boolean strobe = true;
int strobeCount = 0;

//const unsigned int DEBOUNCE_TIME = 300;
//const unsigned int REFRESH_TIME = 500;
//static unsigned long last_interrupt_time = 0;

//[for man light]
byte manInterrupt = 1; //second interupt pin 3
byte lightInterrupt = 0; //light interrupt pin 2 
unsigned long int time1;  
unsigned long int pushStartTime;  
unsigned long int pushEndTime;
unsigned long int pushedTime;

volatile boolean pushActive = false;
volatile boolean isPushed = false;
volatile boolean lightActive = false;  //i can use the as the one to check where to run the whole script or not as orginaly variabled as manLight
int LEDmode = 1;




//boolean manLight = false;  //state of the manual light
boolean senRelayState = false;  //state of relay. use to determine wheater to run the ping program or just waiting till it gets darker
///remember to change it back to false after testing!

/////// Alarm variables
int alarmCountSen0 = 0;
unsigned long alarmTimeSen0;
int alarmCountSen1 = 0;
unsigned long alarmTimeSen1;
int alarmCountSen2 = 0;
unsigned long alarmTimeSen2;
int alarmCountSen3 = 0;
unsigned long alarmTimeSen3;
unsigned long alarmResetTime;

boolean alarmError0 = false;
boolean alarmError1 = false; //bath long error
boolean alarmError2 = false;
boolean alarmError3 = false;
//boolean alarmBuzz = false;
//unsigned long alarmBuzzTime;




///////////////////////////////


///////////////////////////////////////////////
/// WatchDog / Sleep Mode
///////////////////////////////////////////////

// watchdog intervals
// sleep bit patterns for WDTCSR
enum 
{
  WDT_16_MS  =  0b000000,
  WDT_32_MS  =  0b000001,
  WDT_64_MS  =  0b000010,
  WDT_128_MS =  0b000011,
  WDT_256_MS =  0b000100,
  WDT_512_MS =  0b000101,
  WDT_1_SEC  =  0b000110,
  WDT_2_SEC  =  0b000111,
  WDT_4_SEC  =  0b100000,
  WDT_8_SEC  =  0b100001, 
 };  // end of WDT intervals enum

 /*
  // sleep for a total of 64 seconds (8 x 8)
  for (int i = 0; i < 8; i++)
   if (issleep == false) break;
   else {
    myWatchdogEnable (WDT_8_SEC); 
   }
  */



// watchdog interrupt / is executed when  watchdog timed out
ISR (WDT_vect) 
  {
  wdt_disable();  // disable watchdog
  }


//to use in script use: goSleep(WDT_[time_SECorMS]); 


void goSleep (const byte interval) {
  #if DEBUG
  digitalWrite(debugled, LOW);
  #endif //debug
  if (watchdog == true) { //watchdog optional here
  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset
  WDTCSR = _BV (WDCE) | _BV (WDE);
  // set interrupt mode and an interval 
  WDTCSR = _BV (WDIE) | interval;    // set WDIE, and requested delay
  wdt_reset();  // pat the dog


}
  // disable ADC
  byte old_ADCSRA = ADCSRA;
  ADCSRA = 0;  
  
  // turn off various modules
  byte old_PRR = PRR;
  PRR = 0xFF; 

  // timed sequence coming up
  
  noInterrupts ();
  //this part caused the chip to keep waking up when pressing the manual button to sleep. so i move the wakelight interrupt only where it needs to be
  //if (watchdog == false) attachInterrupt(lightInterrupt, wakelight, LOW);  //because we want to only wake up with the light triger when it is sleeping forever(aka light outside)
  //attachInterrupt(manInterrupt, manLightRupt, CHANGE);  dont need this cause its always on
  
  // ready to sleep
  //set_sleep_mode (SLEEP_MODE_PWR_DOWN); 
  /////dont really need a standby for the clock anymore since sonic is gone
  if (sleepType) set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  if (!sleepType) set_sleep_mode (SLEEP_MODE_STANDBY);
  sleep_enable();

  // turn off brown-out enable in software
  MCUCR = _BV (BODS) | _BV (BODSE);
  MCUCR = _BV (BODS); 
  interrupts ();
  sleep_cpu ();  

  // cancel sleep as a precaution
  sleep_disable();
  PRR = old_PRR;
  ADCSRA = old_ADCSRA;  //return ACD enabled
  #if DEBUG
  digitalWrite(debugled, HIGH);   //turn off led
  #endif //debug
} // end of myWatchdogEnable
  
  
void wakelight() {
  detachInterrupt(lightInterrupt);
} 


///////////////////////////////////////////////
/// Setup
///////////////////////////////////////////////


void setup() {
  //pinMode(debugswitch, INPUT_PULLUP);
  pinMode(debugled, OUTPUT);
  digitalWrite(debugled, LOW);
  
  /*
  if (digitalRead(debugswitch) == LOW) {  //switch or hold button on power up on pin 12 to delay the program for 3 minutes to allow uploading program without choas of sleeping    
    delay(1800000);  //3minute delay
    digitalWrite(debugled, HIGH);
  }
  */
  #if DEBUG
  Serial.begin(19200);
  #endif // DEBUG
  attachInterrupt(manInterrupt, manLightRupt, CHANGE);  //manual interrupt interrupt interrupt interrupt interrupt
  //setting up relay pins and HIGH for off on start of program.
  pinMode(PIRbath, INPUT);
  pinMode(PIRstair, INPUT);
  pinMode(PIRtv, INPUT);
  pinMode(PIRtele, INPUT);
  pinMode(senRelay, OUTPUT);
  pinMode(bathRelay, OUTPUT);
  pinMode(stairRelay, OUTPUT);
  digitalWrite(bathRelay,LOW);
  digitalWrite(stairRelay,LOW);
  digitalWrite(senRelay,LOW); // >>>>>>> boolean senRelayState = true;
  //sensorReset();
  /*
  sensorReset() = this below
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    */
}




////////////////////////////////////////////////////
/////////////////////LOOP//////////////////////////
//////////////////////////////////////////////////

void loop() {
  #if DEBUGreading
  Serial.println("SSSSSSSSSSEEEEENNNNSSSSOORRR  TV is ");
  Serial.print(digitalRead(PIRtv));
    Serial.println("SSSSSSSSSSEEEEENNNNSSSSOORRR  bsth is ");
  Serial.print(digitalRead(PIRbath));
    Serial.println("SSSSSSSSSSEEEEENNNNSSSSOORRR  stair is ");
  Serial.print(digitalRead(PIRstair));
    Serial.println("SSSSSSSSSSEEEEENNNNSSSSOORRR  Tele is ");
  Serial.print(digitalRead(PIRtele));
  #endif
      // Other code that *DOESN'T* analyze ping results can go here.
      //alarmErrorBuzz(); 
      manCheck();  //runs when the button is pushed(sets pushActive to true)
      delay(1); //program is so fast that inorder to keep track of time without guessing i just add this delay to be able to make counts
      //sp in normal operation, 1 clock is 1 second because of sleep. but in other seconds 1 will be milli because then it would be operasting without sleep mode
      #if DEBUGreading
      //these words delay the ping so it will not give a reading for the first 2 sensors.
      Serial.println();
	  Serial.println("Loop:");
      Serial.print("is lightactrivelop?: ");
      Serial.print(lightActive);
      Serial.print(".  am i push active2loop?: ");
      Serial.print(pushActive);
      Serial.print(". am i pushed loop? ");
      Serial.println(isPushed);
      #endif //debugreading
      if (comaTimerAct) {   //oh this comatimer is for the flash on after deactivating manual light 
        if (millis() - comaTimer > 500) { lighton(); } //flash back on
        if ((millis() - comaTimer) > 5000) { 
          comaTimerAct = false;
          digitalWrite(stairRelay, LOW);  //make sure leds are off
          digitalWrite(bathRelay, LOW);
          delay(200); //delay to not trigger sensor sleep 
          //sensorReset();  
        /* the sensor will give readings of 0 cm after button pushes, so after a lot of testing and thinking i figured out 
        the sensor needs time to balance out, but not like how i did before by giving a minute to chill. no it actually needed
        to send a ping in order to reset itself. so thats why i added this here and i will add it whereever it needs to balance out also.
        Also now, it would trigger a go to sleep for a short time maybe because its still bright imidiately so i added a small delay. that with a sensor reset should be fine
        */
        }
      }
      if (!comaTimerAct) { //if the "light on" after just manually turning the off flag is off, continue with program
       //strobe light needs a loop so here we are
      // if ((LEDmode == 4) && (lightActive == true)) {  //if strobe mode is on and the light active button has been activated, have a party
      //  strobe = !strobe;
      //  strobeCount++;
      //  digitalWrite(stairRelay, strobe);  
      //  digitalWrite(bathRelay, strobe);
      //  delay(110);
      //  if (strobeCount == 31) {  //gives time to pulse around before adding something special :)
      //    /*strobe2xside();
      //    strobeSet();*/
      //    //strobeSet();
      //    digitalWrite(stairRelay, 1);  
      //    digitalWrite(bathRelay, 1);
      //    delay(50);
      //    strobeCount = 0;
      //  }
      // } //ends strobe
      //if ((LEDmode == 5) && (lightActive == true)) {  //if strobe mode is on and the light active button has been activated, have a party
      //  strobe = !strobe;
      //  digitalWrite(stairRelay, strobe);  
      //  digitalWrite(bathRelay, !strobe);
      //  delay(1500);    
      //} //ends strobe
    
      if (lightActive == false) { //meaning that the manual light control is off; contrinue with the rest of the script
         lightSenCheck();  //checks for light operation
         if (senRelayState == true) {  //if the sensor relay is on because it is currently dark as declared earlier.  
           //if (wasSleeped) { 
            //    wasSleeped = false; 
              //  return; //to skip the sleep for 8 seconds for the first time it wakes up just incase it was still in hat inbetween after switching off the basement lights
             // }
           PIRsensorCheck();
           #if DEBUG
           Serial.print("loop done scanning. Lets repeat");
           delay(30);
		   
           #endif
		   //alright so this bug got me going chasing for it for a while. so it would tuyrn off and on for no reason and nothing giving any hints in the serial print and then i saw this piece of
		   //sleep and fiugured this might be the issue because when theres no more power going to the tip120 it turns off, and so i tested it if i put it to sleep for 4 seconds and sure enough it shut off for 4 seconds
		   
           goSleep (WDT_1_SEC);  //sleep a bit inbetween motion sensor scans  //before this sleep code was inside the sonic ping section. so now i will also put it here
       
           /*   no more sonic sensor
                for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
                if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
                pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
                goSleep (WDT_512_MS);  //sleep a bit inbetween motion sensor scans
                if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
                sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
                currentSensor = i;                          // Sensor being accessed.
                cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
                sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
                }
              } 
          */
  
      } //closes if senser relay is true
    }  //closes manLight 
  }  //closes comatimer act       
}   //closes void loop



///////////////////////////////////////////////
/// Manual Button Check
///////////////////////////////////////////////

void manLightRupt() {  //interupt to manualy trigger the led strip on/off
  //wdt_disable(); //stop sleeping for a till we do our crap
  unsigned long interrupt_time = millis();
  time1 = interrupt_time; // - last_interrupt_time;
//  if (time1 > DEBOUNCE_TIME) {   // somehow i got it to work better without using this debounce methond
    //detachInterrupt(1);  //dont detach here because i need it be able to let go and detect the change also.
    if ((isPushed == false)  && (digitalRead(3) == HIGH)) {  //digitalread somehow helps with double bounce
      //so if the button has previously not been pushed and is now high:
      isPushed = true; //set that it is now pushed
      pushStartTime = time1;  //set the time it was pushed
      pushActive = true; //set push button active to run the loops
      //isSleeping = false;  //stop sleeping
    }
    else if (isPushed == true) { // && (digitalRead(3) == LOW)) { //but digitalread has no effect here
      detachInterrupt(1); //but detaching the Interrupt helps better than having it digital read
      pushEndTime = time1; //set the time that we released the button
      isPushed = false; //set that the button is no longer pressed
      //isSleeping = true; //say it was used in day time, let it go back to sleep.
      
    }
 //  last_interrupt_time = interrupt_time;  //once again , able to do it without debounce here
  //}
} 


void manCheck(){
  #if DEBUGreading
  //variable debugging checkspushActive
  Serial.println();
  //Serial.print("is the interrupt active?");
  //Serial.println(ba);
  Serial.print("am i active?:");
  Serial.println(pushActive);
  Serial.print("am i pushed?");
  Serial.println(isPushed);
  #endif //DEBUGreading
  if (pushActive == true) { //button is being pushed active
    if (isPushed == true) {  //checks if it pushed down 
     pushedTime = millis() - pushStartTime;  //measures how much time is currently being held
    
     #if DEBUGreading
     Serial.print("i am being pushed right now:");
     Serial.println(pushStartTime);
     Serial.print("currently held for seconds now : ");
     Serial.println(pushedTime);
     #endif //debugreading
     //if (!lightActive) lighton();  
     /*disabled  ^^^^^ because i think it would be more realistic
     that someone would click it once first then click and hold to change modes
     rather than change modes right away
     */
     
     if (pushedTime > 800) {  //if ive held it for more than 1.5 seconds :       
       LEDmode++; //change the led mode 
       if (LEDmode == 3) LEDmode = 1;  //if mode is on 3, reset to 1
       //if(LEDmode == 1) strobe2xside();  //to indicate it went from strobe to solid  -- dont need it now for the pong switch off
        lighton();  //run the code to turn on led             
        lightActive = true;  //sets the light as current active (this will be manlight)              
        pushActive = false;  //resets the push button checkers to default
        isPushed = false;  //ditto
        //isSleeping = true;
        #if DEBUG
        Serial.println("wow more than two seconds! im modifying the light design!"); 
        Serial.println("sleeping");        
        #endif //debug   
        //if (LEDmode > 3) return;     
        delay(1000);  //was 700 incase its not acting weird after changiong//to ignore double bounce to not wake up from sleeping right away
        attachInterrupt(manInterrupt, manLightRupt, CHANGE);
        if (LEDmode < 4) { 
         
         manComa();  //decided to add a sleep mode here to save more power while having it manually on(playing ping pong takes a long time)
         //lightActive = false;
        }
      } //end if held for 1.5seconds  
    } //end if push 
    else if (isPushed == false) {  //once letting go of the button it will become false(but the code is still active)
      pushedTime = pushEndTime - pushStartTime; //find the total resulting time 
      #if DEBUG
      Serial.print("ive been let go and standing up now for this much time :");
      Serial.println(pushedTime);
      #endif //debug
      pushActive = false; //set the code no longer active so this doesnt loop over again
      attachInterrupt(manInterrupt, manLightRupt, CHANGE); //reattach
     
      if (pushedTime < 1000) { // && (pushedTime > 1))   //if a quick press by using total resulting time:
         #if DEBUG
         Serial.println("less than 2 seconds? ok heres the light on");
         #endif //debug
         if (lightActive == false) {  //if light was off
          lightActive = true; //and variable for active.
          lighton(); //turn it on using the current led mode  
          alarmError0 = false;  //resets alarm count because if im pushing a button, its obvious that i am home and know about any errors
          alarmError1 = false;
          alarmError2 = false;        
          delay(700); //to ignore double bouncing to not wake up the sleep
          attachInterrupt(manInterrupt, manLightRupt, CHANGE); //reattach  because sometimes it would not be able to trigger anymore. maybe by pressing it too fast or something
          if (LEDmode < 4) {
           manComa();   //decided to add a sleep mode here to save more power while having it manually on(playing ping pong takes a long time)
           //and putting it here allows it to sleep only when turning the light on. not sleeping everytime as if when i put it at the end of quick push( cause thats undesiered effect)
          }  
        }
        else  { //if light is on  //else if (lightActive)
          lightActive = false;  //set var to be off
          digitalWrite(bathRelay, LOW);  //turn them off manually
          digitalWrite(stairRelay, LOW);
          //sensorReset();
        }  //end elseif lightactive
      
      } //ends quick push
    } //ends if released  
  } //ends push active
} //ends mancheck



  
void manComa() {
  watchdog = false;  
  wdt_disable(); //when waking up from that "twilight" sleep where its not dark yet, and manual mode was triggered,
 // the chip will wake up because the timer was still running. so to correct this, i added this watchdog disable code so it will disable the timer when running manual mode.
 //i suspect this can also be added in the the goSleep() portion, and instead of having the if watch dog is true, set up all those things,
 //i could just say, if watchdog is false, disable the wakeup timer. that would be pretty easy. but all in all is all the same.
  goSleep(WDT_8_SEC);
  watchdog = true;
  digitalWrite(bathRelay, LOW);  //turn them off manually
  digitalWrite(stairRelay, LOW);
  comaTimerAct = true;
  comaTimer = millis();
}

/*
if i really want to add a way to make manual light use the most power save mode while in use i could, 
but currently if its dark it will run as idle sleep but these power savings arnt crazy enough to do it since the lights are already running at .7-1Amp

SLEEP_MODE_STANDBY : 0.84 mA  clock keeps running
SLEEP_MODE_PWR_DOWN : 0.36 mA

*/
  
  
void lighton() {  //this turns on the lights (will be the relays)
  #if DEBUGreading
  Serial.print("turninging the light mode:");
  Serial.println(LEDmode);
  #endif //debug  
         
  switch (LEDmode) { //the number defines which led mode to use
	  //relay note: k so output LOW is on, and output HIGH is off.
	  //changing it to transistor so it will be opposite and with pwm. > Low = off and high = on

    case 1: 
    //lights stay on
		digitalWrite(bathRelay, HIGH);
        digitalWrite(stairRelay, HIGH);
        break;
    
    case 2:  
    //lights stay off
        //signal so we know
        digitalWrite(bathRelay, LOW);
        digitalWrite(stairRelay,LOW);
        delay(200);
        digitalWrite(bathRelay, HIGH);
        digitalWrite(stairRelay,HIGH);
        delay(200);
        digitalWrite(bathRelay, LOW);
        digitalWrite(stairRelay,LOW);
        delay(200);
        /*digitalWrite(bathRelay, HIGH);
        digitalWrite(stairRelay,HIGH);*/
        break;
  
  }
}  
  
  
  
  
  

 /* """"""""  Original lightOn with discos and all  """"
void lighton() {  //this turns on the lights (will be the relays)
  #if DEBUGreading
  Serial.print("turninging the light mode:");
  Serial.println(LEDmode);
  #endif //debug  
         
  switch (LEDmode) { //the number defines which led mode to use
    case 1: 
    //both sides
        digitalWrite(bathRelay, LOW);
        digitalWrite(stairRelay,LOW);
        break;
    
    case 2:  //just stair side
        digitalWrite(bathRelay, HIGH);
        digitalWrite(stairRelay, LOW);
        break;
    case 3:  
      //just bath side        
        digitalWrite(bathRelay, LOW);
        digitalWrite(stairRelay, HIGH);
        break;
    case 4:  //both sides after strobe light
        strobe2xside();
        /*
        digitalWrite(bathRelay, HIGH);
        digitalWrite(stairRelay,HIGH);
        delay(200);
        digitalWrite(bathRelay, LOW);
        digitalWrite(stairRelay,LOW);
        *---------------------------------------------------------/  remove these if using orignal again
        break;
    case 5:  //both sides after strobe light
    strobe2xside();
    /*
        digitalWrite(bathRelay, HIGH);
        digitalWrite(stairRelay,HIGH);
        delay(200);
        digitalWrite(bathRelay, LOW);
        digitalWrite(stairRelay,LOW);
        break;
        *-----------------------------------------------------------/
  }
}

""""""""""""""""""" end of lightOn """""""""""""""""""""""""""""""""""""'
*/

//void strobe2xside() {
//      digitalWrite(stairRelay, 1);  
//      digitalWrite(bathRelay, 1);
//      delay(110);
//      //double side flash=
//      digitalWrite(stairRelay, 0);  
//      digitalWrite(bathRelay, 1);
//      delay(110);
//      digitalWrite(stairRelay, 1);  
//      digitalWrite(bathRelay, 1);
//      delay(110);
//      digitalWrite(stairRelay, 0);  
//      digitalWrite(bathRelay, 1);
//      delay(110);
//      digitalWrite(stairRelay, 1);  
//      digitalWrite(bathRelay, 1);
//      delay(110);
//      digitalWrite(stairRelay, 1);  
//      digitalWrite(bathRelay, 0);
//      delay(110);
//      digitalWrite(stairRelay, 1);  
//      digitalWrite(bathRelay, 1);
//      delay(110);
//      digitalWrite(stairRelay, 1);  
//      digitalWrite(bathRelay, 0);
//      delay(110);
//}
//
//void strobeSet() {  //circle trail.
//      //circle
//      digitalWrite(stairRelay, 1);  
//      digitalWrite(bathRelay, 1);
//      delay(110);
//      digitalWrite(stairRelay, 0);  
//      digitalWrite(bathRelay, 1);
//      delay(110);
//      digitalWrite(stairRelay, 0);  
//      digitalWrite(bathRelay, 0);
//      delay(110);
//      digitalWrite(stairRelay, 1);  
//      digitalWrite(bathRelay, 0);
//      delay(110);
//}

///////////////////////////////////////////////
///Light Sensor Check
///////////////////////////////////////////////

void lightSenCheck() {
      lightSenValue = analogRead(lightSen); // read the value from the sensor 
	   //k so a little bug happened after seting the rampOn with the tip 120's. so after its fully on, since it takes longer or something, basically it screwed up with the timing system a bit so it would see it as too bright and go to sleep and then immediatly see it too dark and wake back up. so a quick fix, i can piggy back off my new justOn variable so if anylight is currently on, to skip this whole section of checking if its too bright, becasuse it gets handled in a different method.
	  //k actually found out it was the little part where u sleep the chip for a sec in the main loop to "power save"
	  //so essentially i dont need this thing here, but rather do something there instead
      #if DEBUGreading
      //[can put value checkers here as needed]
	  Serial.println("Light Sensor Check:");
      Serial.print(" LightV:");
      Serial.print(lightSenValue);
      Serial.print(" SensorRelay:");
      Serial.print(senRelayState);
      Serial.println("");
      #endif //DEBUGreading
      switch (senRelayState) {        
          case false:  //waiting for dark  //chip may be running because intrupt turned it on.  here it checks status of darkness
              if (pushActive) return; //was (push active == true)  //this is added because without it, then the manual button on switch is ignore when comming from sleep
              #if DEBUGreading
              Serial.println("button should not be pushhed now");
              #endif
              if (lightSenValue < lightLD) {  //if it is still light out, make sure the sensors dont have power connected until its dark(less than 500 i guess)
                //once it gets dark :: ACTIVATE
                senRelayState = true;  //added boolen state so it doesnt have to turn on the pins every second of time. idk if it is nesccary but whatever 
                sleepType = true;  //set to littlesleep mode (nap)                               
                digitalWrite(senRelay, HIGH);  //now its NPN //power transistor PNP with low Signal
                //wasSleeped = 0; //to prevent auto light on
                //delay(100); // delay 5 seconds after waking up from deep sleep to avoid the light auto turn on at sundown from having the sensors powered up.
                #if DEBUGreading
                Serial.println("Relay Turned On. TTTTTTTRRRRRUUUUUUUEEEE");  //this was to clarify that this code only ran once and not repeatedly. [success]
                digitalWrite(debugled,LOW);
                #endif //debug
                //sensorReset(); //to correct issue with reading zero after sleep
                return;  //exit
               //maybe add a break; here so it could break out of the case before sleeping for  another 5 minutes. or i can add below if lightsen is > 600 go to sleep. probably that
               }               
              ifTooBrightGoSleep(); //if woken up early in middle of day by button press, to prevent always sleepins check if it is too bright and sleep forever
              watchdog = true;  //for below so dont need time to set variable so many times
              #if DEBUG
              Serial.print("sleeping");
              delay(50);
              #endif //debug
              /*    //to sleep for more than once
              for (int i = 0; i < 2; i++) {  // 8 * (number) = total seconds sleeping try 10 min?
                 if (!isSleeping) return;  //could be interrupted by manLightRupt. but once it leg go, it should go back to sleep
                 else {
                   attachInterrupt(manInterrupt, manLightRupt, CHANGE);
                   //attachInterrupt(lightInterrupt, wakelight, LOW);
                   #if DEBUG
                   Serial.print("light sleep?:");
                   Serial.println(sleepType);
                   //digitalWrite(debugled, HIGH);                   
                   Serial.println("sleeping in daylight:");
                   delay(300);
                   #endif //debug
                   //watchdog = true; gonna set this down at dark to light
                   goSleep (WDT_4_SEC);
                   }
                  }
                  */
                  //if (!isSleeping) return;
                  //attachInterrupt(manInterrupt, manLightRupt, CHANGE);
              //if (wasSleeped < 10) { 
              //  wasSleeped++; 
              //  return; //to skip the sleep for 8 seconds for the first time it wakes up just incase it was still in hat inbetween after switching off the basement lights
              //}
              goSleep (WDT_4_SEC);  //sleep for 8 seconds. when waking up each time it will check the light condition
              break;
              
          case true: //waiting for light. currently dark
               if (pushActive) return;  //this is added because without it, then the manual button on switch is ignore when comming from sleep                              
                  //Serial.print("its dark");
                  ifTooBrightGoSleep();  //it got bright outside, so lets go to sleep                
                  break;                            
                  //cant nap during sensors here anymore because it freezed the chip.we nap right before the ping starts in loop(). thats thebest place             
        }  //closes switch
} // closes lightsencheck
        

void ifTooBrightGoSleep() {   //turned it into a function since im using it twice and ill save about 100 bytes of memory (with debug)
#if DEBUG                
	Serial.println("Started if too bright.");
	Serial.print("millis() - ledTimer");
	Serial.print(millis());
	Serial.print(" - ");
	Serial.println(ledTimer);
#endif //debug
	//if (((millis() - ledTimer) > (lightTime + 5))) {  //make sure it's not triggered to sleep when its bright just because the led's are on! because now the script is always running
		//Serial.println("made it in the toobright timer analyzer");
		if (lightSenValue > lightDL) {  //once its bright enough turn off relay/sensors
                //need to watch out for ambient light from tv  
                //does not include manual switch light because if its on by manual switch, the rest of program is not running
                 
                //and probably add sleepmode() script here
                //once its bright in basement, turn off relays/sensors, go to sleep
                //then it will have to wake up from here.
                
                #if DEBUG                
                //Serial.println("Relay FFFFFFAAAAAAAALLLLLLSSSSEEEEE");                
                Serial.println("getting ready to sleep forever");
                delay(200);
                #endif //debug
                //DEACTIVE PROGRAM   
                senRelayState = false;  //sets flag so when it is waken up, the program will begin at the light stage
                watchdog = false;  //sleep forever
                sleepType = false;  //deep sleep mode
                
                digitalWrite(stairRelay, LOW);  //make sure leds are off
                digitalWrite(bathRelay, LOW);
                digitalWrite(senRelay, LOW);  //turns off sensors (save 7-15mA per sensor)
                attachInterrupt(lightInterrupt, wakelight, LOW);
                goSleep(WDT_8_SEC); //8seconds cause it needed something inside the ()
                //and theoretical it will also resume here
                //so when it wakes up(because it is dark), set the watchdog on for either long sleep or short sleep
                
                watchdog = true;  //so watchdog is always true except when it needs to be sleeping forever as in above
                //wasSleeped = true;
                //sensorReset();                
                }
     }
//}



///////////////////////////////////////////////
/// PIR Sensor Check
///////////////////////////////////////////////


void PIRsensorCheck() {
    #if DEBUGreading
	Serial.println("PIR sensoir Check:");
    Serial.print("CountSen0:");
    Serial.print(alarmCountSen0);
    #endif
    #if DEBUGerror
    Serial.print(" timeSen0:");
    Serial.print(millis() - alarmTimeSen0);
    #endif //debug error
    if (!ledTimerAct) {
	  ReadBrightnessPot(); //read at set value of pot value
      PIRscan();
      /////////////////////////////////
   } //ends ledtimeract false
   //
   //if ((ledTimerAct) && ((millis() - ledTimer) > (lightTime - 15 ))) { //here we check if the led is active and if the current time the light has been on is more than a little less that the time alloted for the led.
   //  //this way the sensors will scan for movement before the light goes out. if noone moves then it'll turn off at the next if statment. but if it does sense someone, the current time will reset and will remain on for another duration.
   //  PIRscan();
   //  if ((ledTimerAct) && ((millis() - ledTimer) < lightTime)) return;  //if current time has niot reached its max yet, skip the shutoff
   //  else {
   //    //digitalWrite(bathRelay, LOW);  //used to be HIGH for relay, now LOW for transistor
   //    //digitalWrite(stairRelay, LOW);
	  // if (justOn == 1) {

		 //  for (size_t i = 0; i < potValue; i++)
		 //  {
			//	#if DEBUGreading
			//   Serial.print("pot value is : ");
			//   Serial.println(potValue);
			//   Serial.print("rampValue: ");
			//   Serial.println(rampOnValue);
			//	#endif //DEBUGreading
			//   rampOnValue--;
			//   analogWrite(stairRelay, rampOnValue);
			//   analogWrite(bathRelay, rampOnValue);
			//   if (rampOnValue <= (potValue / 3)) delay(37);
			//   else delay(14);
		 //  }
		 //  rampOnValue = potValue;
	  // }
	  // else if (justOn == 2) RampOnLight(stairRelay, false);
	  // else if(justOn == 3) RampOnLight(bathRelay, false);
	  // justOn = 0;

   //    //sensorReset();
   //    ledTimerAct = false;
   //    #if DEBUG 
   //    Serial.println("timer complete and reset"); 
   //    #endif // DEBUG
   //    } //end else
   // } //ends ledtimeract true
   // 
#if DEBUG  
    Serial.println("");
    #endif //debug
} //ends PIRsensorCheck







void PIRscan() {
  /*
  if (wasSleeped == true) {
    wasSleeped = false;
    delay(100);
  }
  */  
      #if DEBUG
	  Serial.println("PIR scan:");
      //Serial.println(millis());
      #endif
//	  if (justOn > 0) return;
     // if  ((alarmError0 || alarmError1 || alarmError2 == true) && ((millis() - 540) > alarmResetTime)) {  //reset alarms
		if ((alarmError0 || alarmError1 || alarmError2 || alarmError3 == true))  {
																										  //figure i add a auto reset function so i dont have to press the button myself on the rare occasions.  this will reset it after ~ 5min of no activity
      //just to recap, about 1 millis passes for every clock, even while the led is on. but not sure about when pressing a button, i think that is real time
         
          alarmError0 = false;  
          alarmError1 = false;
          alarmError2 = false;
          alarmError3 = false; 
          #if DEBUGerror
          Serial.print("alarms have been reset");
          #endif //debug
          }
      if (digitalRead(PIRtv) == HIGH && !alarmError0) {
        ledTimerAct = true;
		rampOnValue = 0;
		for (size_t i = 0; i < potValue; i++)
		{
			#if DEBUGreading
			Serial.print("pot value is : ");
			Serial.println(potValue);
			Serial.print("rampValue: ");
			Serial.println(rampOnValue);
			#endif //DEBUGreading
			rampOnValue++;
			analogWrite(stairRelay, rampOnValue);
			analogWrite(bathRelay, rampOnValue);
			if (rampOnValue <= (potValue / 4)) delay(RampSpeedSlow);
			else if (rampOnValue <= (potValue / 2)) delay(RampSpeedMed);
			else delay(RampSpeedFast);
		}
		


        alarmCountSen0++;   
        if (alarmCountSen0 == 1) { alarmTimeSen0 = millis(); }
        else if (alarmCountSen0 > 2) {
          if ((millis() - alarmTimeSen0) < alarmTimeLimit) {  //time divide by nap time
            //alarmBuzz = false;
            //alarmBuzzTime = 5000;
            alarmError0 = true;
            alarmResetTime = millis();
            #if DEBUGerror
            Serial.print("sensor 0 disabled");
            #endif //debug
          }
          else { alarmCountSen0 = 0; }
        } 
        #if DEBUG 
        Serial.println("tv hallway ON"); 
        #endif // DEBUG
		//justOn = 1;
		RecheckSensorstv:
		ledTimer = millis();

		delay(lightTime - 8000);
		while (millis() < (ledTimer + lightTime)) {
#if DEBUG
			Serial.println("inside the while!");
#endif
			if ((digitalRead(PIRtv) == HIGH && !alarmError0) || (digitalRead(PIRstair) == HIGH && !alarmError1) || (digitalRead(PIRbath) == HIGH && !alarmError2) || (digitalRead(PIRtele) == HIGH && !alarmError3)) {
#if DEBUG
				Serial.println("inside the detected scan!:");
#endif
				goto RecheckSensorstv;
			}
		} 

		for (size_t i = 0; i < potValue; i++)
		{
			#if DEBUGreading
			Serial.print("pot value is : ");
			Serial.println(potValue);
			Serial.print("rampValue: ");
			Serial.println(rampOnValue);
			#endif //DEBUGreading
			rampOnValue--;
			analogWrite(stairRelay, rampOnValue);
			analogWrite(bathRelay, rampOnValue);
			if (rampOnValue <= (potValue / 4)) delay(RampSpeedSlow);
			else if (rampOnValue <= (potValue / 2)) delay(RampSpeedMed);
			else delay(RampSpeedFast);
		}
		//rampOnValue = potValue;


		ledTimer = millis();
		ledTimerAct = false;
    
    delay(2000); //sensor readjust
      } //ends sensor 0 (tv)
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

      else if (digitalRead(PIRstair) == HIGH && !alarmError2) {
    
        ledTimerAct = true;
		RampOnLight(stairRelay, true);
		//digitalWrite(stairRelay, HIGH);
		ledTimer = millis();

      /*
      so theory is that once a sensor gets triggered. it would count up. and it also sets the time on te first time. 
      then after it gets triggered on its 3rd time. it will check how much time elasped between the first trigger and right now,
      and if it detects that it was triggered for its 3rd time in less than 3 or 4 minutes, then it will turn the alarm code on.
      the alarm code will then
      */
    
        alarmCountSen2++;  //count once it sensor triggers  
        if (alarmCountSen2 == 1) { alarmTimeSen2 = millis(); }  //for the first trigger, record the time
        else if (alarmCountSen2 == 3) {  //after being triggered on its 3rd time
          if ((millis() - alarmTimeSen2) < alarmTimeLimit) {    //check to see how much time elasped between the 4 triggers
           //if its the 4th trigger under 4 minutes (plus a little extra) then:    
            
            //alarmBuzz = true; //set buzzer on
            //alarmBuzzTime = millis();  //set buzz time
            alarmError2 = true;  //set stair error flag
            alarmResetTime = millis();
            #if DEBUGerror
            Serial.print("sensor pir stair disabled");
            #endif //debug
          }
          else { alarmCountSen2 = 0; }  //if its been more then 4 min (normal conditions): reset counter
        }
        #if DEBUG
        Serial.println("stair hallway ON"); 
        #endif // DEBUG 
		//justOn = 2;
		RecheckSensorsStair:
		ledTimer = millis();

		delay(lightTime - 8000);
		while (millis() < (ledTimer + lightTime)) {


/*

//			if ((digitalRead(PIRtv) == HIGH && !alarmError0) || (digitalRead(PIRstair) == HIGH && !alarmError1) || (digitalRead(PIRbath) == HIGH && !alarmError2) || (digitalRead(PIRtele) == HIGH && !alarmError3)) {
      if ((digitalRead(PIRstair) == HIGH) ) {

#if DEBUG
//Serial.println("slamr error is : ");
//Serial.print(digitalRead(PIRstair));
        Serial.println("inside the detected scan!:");
#endif
				goto RecheckSensorsStair;
			}
     */
     //litterally no idea why it keeps passing thru that section. even when the sensor is at 0 and alarm is 0 it still passes thru the if statement making a endless scan detection.
		}
		RampOnLight(stairRelay, false);
		ledTimerAct = false;
    delay(2000); //sensor readjust
      }  //close sensor 2 (stair) 
///////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if (digitalRead(PIRbath) == HIGH && !alarmError1) { 
		//else if (PIRbath == 100) { //this is fake to disable
		ledTimerAct = true;
		RampOnLight(bathRelay, true);
		ledTimer = millis();

		//digitalWrite(bathRelay, HIGH);
     
     ////////////////////
        alarmCountSen1++;  
        if (alarmCountSen1 == 1) { alarmTimeSen1 = millis(); }
        else if (alarmCountSen1 == 3) {
          if ((millis() - alarmTimeSen1) < alarmTimeLimit) {  //time divide by nap time
            //alarmBuzz = true;
            //alarmBuzzTime = millis();
            alarmError1 = true;
            alarmResetTime = millis();
            #if DEBUGerror
            Serial.print("sensor pir bath disabled");
            #endif //debug
          }
          else { alarmCountSen1 = 0; } 
        }
        #if DEBUG 
        Serial.println("bath hallway ON"); 
        #endif // DEBUG
		//justOn = 3;
		RecheckSensorsBath:
		ledTimer = millis();

		delay(lightTime - 8000);
		 while (millis() < (ledTimer + lightTime)) {
      #if DEBUG
      Serial.println("inside the while!");
#endif
			if ((digitalRead(PIRtv) == HIGH && !alarmError0) || (digitalRead(PIRstair) == HIGH && !alarmError1) || (digitalRead(PIRbath) == HIGH && !alarmError2) || (digitalRead(PIRtele) == HIGH && !alarmError3)) {
#if DEBUG
        Serial.println("inside the detected scan!:");
#endif
				goto RecheckSensorsBath;
			}
		}
		RampOnLight(bathRelay, false);
		ledTimerAct = false;
    delay(2000); //delay to allow time for sensor to readjust
      }  //close sensor 1 (bath)
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
   else if (digitalRead(PIRtele) == HIGH && !alarmError3) { 
    //else if (PIRtele == 100) {
    ledTimerAct = true;
    RampOnLight(stairRelay, true);
    ledTimer = millis();

    //digitalWrite(stairRelay, HIGH);
     
     ////////////////////
        alarmCountSen3++;  
        if (alarmCountSen3 == 1) { alarmTimeSen3 = millis(); }
        else if (alarmCountSen3 == 3) {
          if ((millis() - alarmTimeSen3) < alarmTimeLimit) {  //time divide by nap time
            //alarmBuzz = true;
            //alarmBuzzTime = millis();
            alarmError3 = true;
            alarmResetTime = millis();
            #if DEBUGerror
            Serial.print("sensor pir Tele disabled");
            #endif //debug
          }
          else { alarmCountSen3 = 0; } 
        }
        #if DEBUG 
        Serial.println("Tele hallway ON"); 
        #endif // DEBUG
    //justOn = 3;
    RecheckSensorsTele:
    ledTimer = millis();

    delay(lightTime - 8000);
     while (millis() < (ledTimer + lightTime)) {
      #if DEBUG
      Serial.println("inside the while!");
#endif
      if ((digitalRead(PIRtv) == HIGH && !alarmError0) || (digitalRead(PIRstair) == HIGH && !alarmError1) || (digitalRead(PIRbath) == HIGH && !alarmError2)|| (digitalRead(PIRtele) == HIGH && !alarmError3)) {
//      if ((digitalRead(PIRtv) == HIGH ) || (digitalRead(PIRstair) == HIGH ) || (digitalRead(PIRbath) == HIGH )|| (digitalRead(PIRtele) == HIGH )) {

#if DEBUG
        Serial.println("inside the detected scan!:");
#endif
        goto RecheckSensorsTele;
      }
    }
    RampOnLight(stairRelay, false);
    ledTimerAct = false;
    delay(2000); //delay to allow time for sensor to readjust
      }  //close sensor 3 (telephone-port)
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}  //end PIRscan


void ReadBrightnessPot() {
	potValue = analogRead(potPin);
	potValue = map(potValue, 0, 328, 0, 90);  //map(variable, from low high, to low high)
	//k very weird glitch. when the output map is anywhere between 100-200, the leds dont go back down with no debug
  //works fuine with debug tho
	#if DEBUGreading

	Serial.print("pot value is : ");
	Serial.println(potValue);
	#endif //DEBUGreading
}

void RampOnLight(int ledLocation,bool ONorOFF) {
	if (ONorOFF) rampOnValue = 0;
	else rampOnValue = potValue;
	for (size_t i = 0; i < potValue; i++)
	{
#if DEBUGreading
		Serial.print("pot value is : ");
		Serial.println(potValue);
		Serial.print("rampValue: ");
		Serial.println(rampOnValue);
#endif //DEBUGreading
		if (ONorOFF) rampOnValue++;
		else rampOnValue--;
		analogWrite(ledLocation, rampOnValue);
		if (rampOnValue <= (potValue / 4)) delay(RampSpeedSlow);
		else if (rampOnValue <= (potValue / 2)) delay(RampSpeedMed);
		else delay(RampSpeedFast);
	}
}


///////////////////////////////////////////////
/// Ping Sonic Sensor
///////////////////////////////////////////////

/*
//we dont need sensor reset i the middle of all our scripts anymore cause we're not using sonic sensors anymore
void sensorReset() {
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}


void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
/////////////////////////////////////////////////


/////////////////////////////////////////////


void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    #if DEBUG
    Serial.print("cm ");
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
    #endif //debug
    #if DEBUGreading
    Serial.print("CountSen0:");
    Serial.print(alarmCountSen0);
    #endif
    #if DEBUGerror
    Serial.print(" timeSen0:");
    Serial.print(millis() - alarmTimeSen0);
    #endif //debug error
    #if DEBUGreading
    Serial.print(" ledtime act ");
    Serial.print(ledTimerAct);
    Serial.print(" ledtime ");
    Serial.println(millis() - ledTimer);
    #endif // DEBUG
   ///////////////////////////////////////////////////////////

   if (!ledTimerAct) {
     
      
   //if ((i == 0) && cm[i] > 16 && cm[i] < 60 && !alarmError0) {  //tv sensor
     if (digitalRead(PIRtv) == HIGH && !alarmError0) {
     ledTimer = millis();
     ledTimerAct = true;
     digitalWrite(stairRelay, LOW);
     digitalWrite(bathRelay, LOW);
     ///////////////////
     alarmCountSen0++;   
      if (alarmCountSen0 == 1) { alarmTimeSen0 = millis(); }
      else if (alarmCountSen0 > 3) {
        if ((millis() - alarmTimeSen0) < alarmTimeLimit) {  //time divide by nap time
            
            //alarmBuzz = false;
            //alarmBuzzTime = 5000;
            alarmError0 = true;
            #if DEBUGerror
            Serial.print("sensor 0 disabled");
            #endif //debug
        }
        else { alarmCountSen0 = 0; }
      } 
     #if DEBUG 
     Serial.println("tv hallway ON"); 
     #endif // DEBUG
    } //ends sensor 0 (tv)
    
else if (digitalRead(PIRstair) == HIGH && !alarmError2) {
     //if  ((i == 0) && (cm[i] < 10) && cm[i] != 0 && !alarmError0)  { 
      ledTimer = millis();
      ledTimerAct = true;
      digitalWrite(stairRelay, LOW);

      
      //so theory is that once a sensor gets triggered. it would count up. and it also sets the time on te first time. 
      //then after it gets triggered on its 3rd time. it will check how much time elasped between the first trigger and right now,
      //and if it detects that it was triggered for its 3rd time in less than 3 or 4 minutes, then it will turn the alarm code on.
      //the alarm code will then
      
    
      alarmCountSen2++;  //count once it sensor triggers  
      if (alarmCountSen2 == 1) { alarmTimeSen2 = millis(); }  //for the first trigger, record the time
      else if (alarmCountSen2 == 4) {  //after being triggered on its 4th time
        if ((millis() - alarmTimeSen2) < alarmTimeLimit) {    //check to see how much time elasped between the 4 triggers
       //if its the 4th trigger under 4 minutes (plus a little extra) then:    
            
            //alarmBuzz = true; //set buzzer on
            //alarmBuzzTime = millis();  //set buzz time
            alarmError2 = true;  //set stair error flag
            #if DEBUGerror
            Serial.print("sensor pir stair disabled");
            #endif //debug
        }
        else { alarmCountSen2 = 0; }  //if its been more then 4 min (normal conditions): reset counter
      }
      #if DEBUG
      Serial.println("stair hallway ON"); 
      #endif // DEBUG 
      }  //close sensor 2 (stair) 
      
    
   // sonic sensor with pir: if  ((i == 0) && !alarmError1 && ( cm[i] > 4 || digitalRead(PIRbath) == HIGH)) { ////////////////////((alarmError != 2) || (cm[i] != 0 && alarmError == 2)))  {  
    //if bath sensor and less than distance and ( alarm is off or alarm is on but distance is not 0)
else if (digitalRead(PIRbath) == HIGH && !alarmError1) { 
     ledTimer = millis();
     ledTimerAct = true;
     digitalWrite(bathRelay, LOW);
    // digitalWrite(stairRelay, LOW);
     ////////////////////
     alarmCountSen1++;  
     if (alarmCountSen1 == 1) { alarmTimeSen1 = millis(); }
     else if (alarmCountSen1 == 4) {
        if ((millis() - alarmTimeSen1) < alarmTimeLimit) {  //time divide by nap time
            //alarmBuzz = true;
            //alarmBuzzTime = millis();
            alarmError1 = true;
            #if DEBUGerror
            Serial.print("sensor pir bath disabled");
            #endif //debug
        }
       else { alarmCountSen1 = 0; } 
     }
     #if DEBUG 
     Serial.println("bath hallway ON"); 
     #endif // DEBUG
    }  //close sensor 1 (bath)

   } //ends ledtimeract false
   
   if ((ledTimerAct) && ((millis() - ledTimer) > lightTime)) { //multiply number by 256 or whatever the napping sleep mode is set at to get about a minute
       digitalWrite(bathRelay, HIGH);
       digitalWrite(stairRelay, HIGH);
       //sensorReset();
       ledTimerAct = false;
       #if DEBUG 
       Serial.println("timer complete and reset"); 
       #endif // DEBUG
    } //ends ledtimeract true
    #if DEBUG  
  Serial.println();
  #endif //debug
  } //ends for sensor
} //ends onecycle


*/  //ping sensor comment out




///////////////////////////////////////////////
///Error Handling
///////////////////////////////////////////////
//buzzers won't work when sleeping. decided buzzer is not necessary when not working anyway because then you would know it wasnt working when walking by in the dark.
//measure time for once i trigger time wait a minute and look at reading for sensor 0. record that time. that will bre the on(1 minute milli seconds) time.
/*
void alarmErrorCheck() {
  if (alarmCountSen0 == 1) { alarmTimeSen0 = millis(); }  //for the first trigger, record the time
      else if (alarmCountSen0 == 4) {  //after being triggered on its 4th time
        if ((millis() - alarmTimeSen0) < 4000UL) {    //check to see how much time elasped between the 4 triggers
       //if its the 4th trigger under 4 minutes (plus a little extra) then:    
            alarmCountSen0++;
            alarmBuzz = true; //set buzzer on
            alarmBuzzTime = millis();  //set buzz time
            alarmError0 = true;  //set stair error flag
            #if DEBUGerror
            Serial.print("sensor 1 disabled");
            #endif //debug
        }
        else { alarmCountSen0 = 0; }  //if its been more then 4 min (normal conditions): reset counter
      }
  if (alarmCountSen1 == 1) { alarmTimeSen1 = millis(); }
      else if (alarmCountSen1 == 4) {
        if ((millis() - alarmTimeSen1) < 4000UL) {  //time divide by nap time
            alarmCountSen1++;
            alarmBuzz = true;
            alarmBuzzTime = millis();
            alarmError1 = true;
            #if DEBUGerror
            Serial.print("sensor 2 disabled");
            #endif //debug
        }
       else { alarmCountSen1 = 0; } 
      }
      if (alarmCountSen2 == 1) { alarmTimeSen2 = millis(); }
      else if (alarmCountSen2 == 4) {
        if ((millis() - alarmTimeSen2) < 4000UL) {  //time divide by nap time
            alarmCountSen2++;
            alarmBuzz = false;
            alarmBuzzTime = 5000;
            alarmError2 = true;
            #if DEBUGerror
            Serial.print("sensor 3 disabled");
            #endif //debug
        }
        else { alarmCountSen2 = 0; }
      }
}

*/
/*
void alarmErrorBuzz() {
  if (alarmError0 || alarmError1 || alarmError2) {  //if any error is on
     if (alarmBuzz == false && (millis() - alarmBuzzTime) > 100UL) {  //this is after the pause and will make the alarm sound here
        #if DEBUGerror
        Serial.print("alarm on now");
        #endif //debug
        //if alarm = 1 make different pitch
        digitalWrite(alarmPin, HIGH);
        
        alarmBuzz = true;
        alarmBuzzTime = millis();
     }
     else if (alarmBuzz == true) { //if the alarm is set on(currently making buzz)
      if ((millis() - alarmBuzzTime) > 100UL) { //once 1 minute passes, it will turn off the buzz
      #if DEBUGerror
      Serial.print("alarm off now");
      #endif //debug
       digitalWrite(alarmPin, LOW);
       alarmBuzz = false;
       alarmBuzzTime = millis();
      }
     }
  } //close alarmerror
}

*/

///need to make alarm errors seperate for eac`h one. not just one alarm error
/// figure out another way to notify error. sleep mode doesnt work during analog write




      
