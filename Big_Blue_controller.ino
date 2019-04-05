// include the library code:
#include <LiquidCrystal.h>
#include <DHT.h>
#include <INA226.h>
#include <Wire.h>

/* Pin List: (by "physical pin", aka, pin 36 below is also IDE pin 14, IDE analog pin A0, ADC7, PF7, and TDI)
 * 00  - xxxxxx     10  - Button 2       20  - xxxxxx     30  - LED SSR 3 40  - LCD  
 * 01  - DHT11_Data 11  - Button 3       21  - xxxxxx     31  - LED SSR 6 41  - LCD  
 * 02  - xxxxxx     12  - LED SSR 4      22  - xxxxxx     32  - LED SSR 1 42  - xxxxxx  
 * 03  - xxxxxx     13  - xxxxxx         23  - xxxxxx     33  - xxxxxx    43  - xxxxxx  
 * 04  - LCD_bl     14  - xxxxxx         24  - xxxxxx     34  - xxxxxx    44  - xxxxxx  
 * 05  - xxxxxx     15  - xxxxxx         25  - LCD_BL_En  35  - xxxxxx    45  - xxxxxx  
 * 06  - xxxxxx     16  - xxxxxx         26  - LED SSR 2  36  - LCD       46  - xxxxxx    
 * 07  - xxxxxx     17  - xxxxxx         27  - xxxxxx     37  - LCD       47  - xxxxxx  
 * 08  - xxxxxx     18  - I2C BUS Clock  28  - Door Sense 38  - LCD       48  - xxxxxx  
 * 09  - Button 1   19  - I2C BUS Data   29  - LED SSR 5  39  - LCD       49  - xxxxxx  
 *
 */ 

/* IN ORDER TO ADD A NEW STATE YOU MUST MODIFY THESE PLACES:
 **   Variables:
 ***    num_menu_states
 ***    menu_states
 ***    transition_select   <-- these can be tricky to do correctly
 ***    transition_up       <--/  dont just add the new state to
 ***    transition_down     <--/  the end of the list.
 **   Functions:
 ***    UpdateMenu(int state)
 ***    pressUP(int state)
 ***    pressDOWN(int state)
 ***    pressSELECT(int state)
 * PLUS whatever code you need to make that state work   
 */ 

// Variables associated with the overarching system---------//
// were a button pressed                                    //
volatile bool PCFlag = false;                               //
int numtoUpdate = 15;                                       //
int sysStep = 0;                                            //
int serialDelay = 30;                                       //
int serialCounter = 0;                                      //
                                                            //
//  status issues are arranged in most pressing to least    //
//    0b00000000 - all fine               "RUNNING"         //
//    0b00000001 - door open              "DOOR   "         //
//    0b00000010 - over Temp              "TEMP   "         //
//    0b00000100 - led issue              "LED!   "         //
//    0b00111000 - reserved for future use                  //
//    0b01000000 - run finished           "DONE   "         // 
//    0b10000000 - run not started yet    "IDLE   "         //
byte statusByte = 0b10000000;                               //
const char *statusOptions[9] = { "RUNNING", "DOOR   ", "TEMP   ","LED!   ","UNKNOWN","UNKNOWN","UNKNOWN", "DONE   ", "IDLE   "  };
char *statusString = NULL;                                  //
//----------------------------------------------------------//

// Variables and consts associated with the timer subsystem-------------//
const long MILLIINDAY = 86400000;                                       //
const long MILLIINHOUR = 3600000;                                       //
const long MILLIINMIN = 60000;                                          //
const long MILLIINSEC = 1000;                                           //
unsigned long millisStart = millis(); //time of run start               //
unsigned long millisElapse = millis();                                  //
char timeElapse[] = "10D12:51:11";                                      //
unsigned long millisSet = millis()+40000;                               //
char timeSet[] = "20D09:42:22";                                         //
unsigned long millisRemain = millis();                                  //
char timeRemain[] = "30D07:33:33";                                      //
                                                                        //
// These timers down here set how often things                          //
// get updated in the main loop.                                        //
// no need for timers since we're not fancy                             //
const long updatePeriod = 1000L;//ms                                    //
const long updateSubPeriod = updatePeriod/numtoUpdate;                  //
unsigned long updateTimer = millis()+updateSubPeriod;                   //
// we need one more lcd time-out timer                                  //
unsigned long millisScreenTimeOut = millis()+15000L;                    //
const int screenTimeOutPeriod = 15000;                                  //
//----------------------------------------------------------------------//


//Variables associated with the temp measurement subsystem--//
const int DHTPIN1 = 7;                                      //
DHT dht(DHTPIN1, DHT11);                                    //
int tempCurr = 35;                                          //
int tempLimit = 38;                                         //
//----------------------------------------------------------//


//Variables associated with the power measurement subsystem-//
const int numLEDS = 6;                                      //
INA226 ina0;                                                //
INA226 ina1;                                                //
INA226 ina2;                                                //
INA226 ina3;                                                //
INA226 ina4;                                                //
INA226 ina5;                                                //
//uint64_t inaptr[] = {&ina0,&ina1,&ina2,&ina3,&ina4,&ina5};  // 

//const int i2cAddr[] = {0x40,0x40,0x42,0x40,0x40,0x40};            // TEMPORARY
const int i2cAddr[] = {0x40,0x41,0x42,0x44,0x45,0x46};            // permanent
//                          //___|__A1_|__A0_|_             //
//int i2cAddr0 = 0x40;      //   | GND | GND |              //
//int i2cAddr1 = 0x41;      //   | GND |  VS |              //
//int i2cAddr2 = 0x42;      //   | GND | SDA |              //
//int i2cAddr3 = 0x44;      //   |  VS | GND |              //
//int i2cAddr4 = 0x45;      //   |  VS |  VS |              //
//int i2cAddr5 = 0x46;      //   |  VS | SDA |              //

int powTarget = 5;                                         //
int pinLevel[] = {5, 5, 5, 5, 5, 5};            // arduino claims all pins go 0-255 by default
//const int pinLED[] = {10,10,11,10,10,10}; // Temporary Dummy pins
const int pinLED[] = {10,9,11,6,13,5};                      // the pins used for the led control //                                                            //
int powLimit = 100; // Watts                                //
float powLED[] = {0,0,0,0,0,0};                   //
unsigned long enerLED[] = {0,0,0,0,0,0};                    //
float voltLED[] = {3.5,3.4,3.3,3.2,3.1,3.7};                //
float currLED[] = {3.1,3.1,3.1,3.1,3.1,3.1};                //
//----------------------------------------------------------//

// Variables associated with the LCD subsystem--------------//
// associate any needed LCD interface pin                   //
const int rs = A0, en = A1, d4 = A2, d5 = A3,               //
        d6 = A4, d7 = A5, bl = 4;                           // 
  //bl is for backlight PWM control, contr is PWM contrast  //
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                  //
int lcdbright = 50;                                         //
//int lcdcontr = 50;                                          // permanent
int buzzervol = 100;                                        //
                                                            //
byte degC[8] = {  //creates the deg C symbol                //
  0b11000,                                                  //
  0b11000,                                                  //
  0b00110,                                                  //
  0b01001,                                                  //
  0b01000,                                                  //
  0b01001,                                                  //
  0b00110,                                                  //
  0b00000,                                                  //
};                                                          //
byte invertS[8] = {  // inverted S symbol for start/stop    //
  0b10001,                                                  //
  0b01110,                                                  //
  0b01111,                                                  //
  0b10001,                                                  //
  0b11110,                                                  //
  0b01110,                                                  //
  0b10001,                                                  //
  0b00000,                                                  //
};                                                          //
//----------------------------------------------------------//

//--variable associating with button press detection----------------------------------------------------------//
int buttonID = 0;     // where i determing WHICH button was pressed                                           //
byte lastPinB = 0;    // need the last value of PINB register to determine if a RISING edge is found or not   //
byte PinB = 0;        // store the truncated current PINB values for PB1,2,3 only                             //
//volatile bool PCFlag = false; // tell the program that the PCINT happened                                   //
//------------------------------------------------------------------------------------------------------------//

// the DIRTIEST FSM, FAM, along with the functions at the bottom please dont look at them though--------------//
int current_menu_state = 0;                       //{m1,1a,1b,m2,m3,3a,3b,m4,m5,5a,5b,5c,5d,5e,5f,5g,5h,m6,m7,m8,m9,n1,n2,n3,s1,s2,n4,4a,n5,5a,n6,6a}
const int num_menu_states = 36;                   //{00,01,02,03,04,05,06,07,08,09,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}
//const int transition_select[num_menu_states] =  { 1, 0, 0, 3, 5, 6, 4, 7, 9,10,11,12,13,14,15,16, 8,17,18,19,20,21,22,23, 0, 0,27,26,29,28,31,30}; // on pressing "select" where should the current state transition
//const int transition_up[num_menu_states] =      {30, 2, 1, 0, 3, 5, 6, 4, 7, 9,10,11,12,13,14,15,16, 8,17,18,19,20,21,22, 0, 0,23,27,26,29,28,31}; // press up
//const int transition_down[num_menu_states] =    { 3, 2, 1, 4, 7, 5, 6, 8,17, 9,10,11,12,13,14,15,16,18,19,20,21,22,23,26, 0, 0,28,27,30,29, 0,31}; // press down
//slight cleanup of above using enums?
//                                               0              1             2             3              4               5          6            7               8                 9             10            11           12            13            14            15            16            17                18              19            20            21            22            23            24            25            26                 27                 28                 29                30                31             32                33              34            35             36 37 38 32       33       34 
enum menu_states                                {SCROLL_START,  START_ON,     START_OFF,    SCROLL_STATUS, SCROLL_TEMP,    TEMP_TENS, TEMP_ONES,   SCROLL_ELAPSED, SCROLL_SET_T,     SET_DAY_TENS, SET_DAY_ONES, SET_HR_TENS, SET_HR_ONES,  SET_MIN_TENS, SET_MIN_ONES, SET_SEC_TENS, SET_SEC_ONES, SCROLL_REMAINING, SCROLL_LED_1,   SCROLL_LED_2, SCROLL_LED_3, SCROLL_LED_4, SCROLL_LED_5, SCROLL_LED_6, IDLER,        SLEEPER,      SCROLL_LCD_BRIGHT, LCD_BRIGHT_FIVES,  SCROLL_LCD_CONTR,  LCD_CONTR_FIVES,  SCROLL_BUZZER,    BUZZER_TOGGLE, SCROLL_LED_POW,   LED_POW_HUNDOS, LED_POW_TENS, LED_POW_ONES}; //                             
const int transition_select[num_menu_states] =  {START_ON,      SCROLL_START, SCROLL_START, SCROLL_STATUS, TEMP_TENS,      TEMP_ONES, SCROLL_TEMP, SCROLL_ELAPSED, SET_DAY_TENS,     SET_DAY_ONES, SET_HR_TENS,  SET_HR_ONES, SET_MIN_TENS, SET_MIN_ONES, SET_SEC_TENS, SET_SEC_ONES, SCROLL_SET_T, SCROLL_REMAINING, SCROLL_LED_1,   SCROLL_LED_2, SCROLL_LED_3, SCROLL_LED_4, SCROLL_LED_5, SCROLL_LED_6, SCROLL_START, SCROLL_START, LCD_BRIGHT_FIVES,  SCROLL_LCD_BRIGHT, LCD_CONTR_FIVES,   SCROLL_LCD_CONTR, BUZZER_TOGGLE,    SCROLL_BUZZER, LED_POW_HUNDOS,   LED_POW_TENS,   LED_POW_ONES, SCROLL_LED_POW}; // on pressing "select" where should the current state transition
const int transition_up[num_menu_states] =      {SCROLL_LED_6,  START_OFF,    START_ON,     SCROLL_START,  SCROLL_STATUS,  TEMP_TENS, TEMP_ONES,   SCROLL_TEMP,    SCROLL_ELAPSED,   SET_DAY_TENS, SET_DAY_ONES, SET_HR_TENS, SET_HR_ONES,  SET_MIN_TENS, SET_MIN_ONES, SET_SEC_TENS, SET_SEC_ONES, SCROLL_SET_T,     SCROLL_LED_POW, SCROLL_LED_1, SCROLL_LED_2, SCROLL_LED_3, SCROLL_LED_4, SCROLL_LED_5, SCROLL_START, SCROLL_START, SCROLL_LED_6,      LCD_BRIGHT_FIVES,  SCROLL_LCD_BRIGHT, LCD_CONTR_FIVES,  SCROLL_LCD_CONTR, BUZZER_TOGGLE, SCROLL_REMAINING, LED_POW_HUNDOS, LED_POW_TENS, LED_POW_ONES}; // press up
const int transition_down[num_menu_states] =    {SCROLL_STATUS, START_OFF,    START_ON,     SCROLL_TEMP,   SCROLL_ELAPSED, TEMP_TENS, TEMP_ONES,   SCROLL_SET_T,   SCROLL_REMAINING, SET_DAY_TENS, SET_DAY_ONES, SET_HR_TENS, SET_HR_ONES,  SET_MIN_TENS, SET_MIN_ONES, SET_SEC_TENS, SET_SEC_ONES, SCROLL_LED_POW,   SCROLL_LED_2,   SCROLL_LED_3, SCROLL_LED_4, SCROLL_LED_5, SCROLL_LED_6, SCROLL_START, SCROLL_START, SCROLL_START, SCROLL_LCD_CONTR,  LCD_BRIGHT_FIVES,  SCROLL_BUZZER,     LCD_CONTR_FIVES,  SCROLL_START,     BUZZER_TOGGLE, SCROLL_LED_1,     LED_POW_HUNDOS, LED_POW_TENS, LED_POW_ONES}; // press down
const int idleStates[] =                        {SCROLL_START,  SCROLL_STATUS, SCROLL_TEMP, SCROLL_ELAPSED, SCROLL_SET_T,  SCROLL_REMAINING, SCROLL_LED_POW, SCROLL_LED_1, SCROLL_LED_2, SCROLL_LED_3, SCROLL_LED_4, SCROLL_LED_5, SCROLL_LED_6, SCROLL_LCD_BRIGHT, SCROLL_LCD_CONTR, SCROLL_BUZZER}; 
const int numIdles = sizeof(idleStates)/sizeof(idleStates[0]);                                                //                                                
int idleCounter = 0;
//------------------------------------------------------------------------------------------------------------//

void setup() {
  //----setup serial stuff--------------------//
  Serial.begin(115200);                       //
  //------------------------------------------//
  
  // set up the LCD---------------------------//
  lcd.begin(16, 2);                           //
  lcd.print("Hello, world!");                 //
  lcd.createChar(1,degC);                     //
  lcd.createChar(2,invertS);                  //
  lcd.display();                              //
  //setup bl and cont pins as analog outs     //
  pinMode(bl,OUTPUT);                         //
  //pinMode(contr, OUTPUT);                   //  contrast circuit didnt make it into design
  digitalWrite(bl, HIGH);                     //  digital control since i ran out of pins
                                              //  analogWrite(contr,contrastScaling(lcdcontr));
  // -----------------------------------------//

  //-set-up temp--------------------------------//
  tempCurr = int(dht.readTemperature())%100;    //
  //--------------------------------------------//

  //--setup button interrupts-------------------------------------------------------------//
  cli();    // disable interrupts                                                         //
  //setup pins as input and enable pullups                                                //
  DDRB  &= 0b11100001;  // set pins PB1, PB2, PB3, PB4 as input, leave rest as is         //
  PORTB |= 0b00011110;  // enable pullup on PB1, PB2, PB3, PB4, leave rest as is          //
  // setup pins as interrupts                                                             //
  PCICR  |= 0b00000001;   // enables interrupt register 0 (where PCINT7:0 live)           //
  PCMSK0 |= 0b00011110;   // allows PCINT1, PCINT2, PCINT3, PCINT4 to trigger interrupts  //
  sei();                                                                                  //
  lastPinB = int(((~PINB) & 0b0011110) >> 1);                                             //
  //--------------------------------------------------------------------------------------//
 
  //------------------setup LED Pins----------------------------------------------//
  int i=0;                                                                        //
  for(i=0; i<numLEDS; i++){                                                       //
    pinMode(pinLED[i], OUTPUT);                                                   //
    analogWrite(pinLED[i], pinLevel[i]);                                          //
  }                                                                               //
  //------------------------------------------------------------------------------//

  //------------------setup INA226's----------------------------------------------//
  //for (i=0;i<numLEDS;i++){                                                        //Test
  //  setupVPCsensor(inaptr[i], i2cAddr[i]);                                        //
  //}                                                                               //Test
  setupVPCsensor(&ina0, i2cAddr[0]);                                              //
  setupVPCsensor(&ina1, i2cAddr[1]);                                              //
  setupVPCsensor(&ina2, i2cAddr[2]);                                              //
  setupVPCsensor(&ina3, i2cAddr[3]);                                              //
  setupVPCsensor(&ina4, i2cAddr[4]);                                              //
  setupVPCsensor(&ina5, i2cAddr[5]);                                              //
  //------------------------------------------------------------------------------//
  
  // initialize everything else   ------------------------------------------------//
  FormatMillis(timeElapse, millisElapse );                                        //
  FormatMillis(timeSet, millisSet);                                               //
  FormatMillis(timeRemain, millisRemain);                                         //
  UpdateMenu(current_menu_state);                                                 //
  //------------------------------------------------------------------------------//

  
  Serial.println("starting up.....");          //
}

void loop() {

  if(millis()>updateTimer){
    UpdateSystem(sysStep);
    updateTimer += updateSubPeriod; 
    sysStep = (sysStep+1) % numtoUpdate;
  }
  // a button were pressed, handle that
  if(PCFlag){
    delay(8);                           // ghetto debouncing
    //check if the door is open FIRST
    if (PINB & 0b00010000){
      // SLAM them pins to 0, with      // pc7,pc6,pb7,pb6,pb5,pd7
      DDRB = DDRB & 0b00011111;
      DDRC = DDRC & 0b00111111;
      DDRD = DDRD & 0b01111111;
      // set that status byte 
      statusByte |= 0b00000001;
    }
    else { //check if it is closed (its the only pin change we trigger on both edges)
      if( !statusByte){
      DDRB = DDRB | 0b11100000;
      DDRC = DDRC | 0b11000000;
      DDRD = DDRD | 0b10000000;
      }
    }
    // ok it wasnt the door, what was it then?
    PinB = ((~PINB)&0b00001110)>>1;  //drop everything but PB1,2,3, and make button down = positive
    buttonID =int( (lastPinB^PinB)&PinB); // select out only NEWLY risen bits
    // reset for next press
    lastPinB=PinB;
    PCFlag = false;
    //millisScreenTimeOut = millis()+screenTimeOutPeriod; //temporary
    // now deal with actual button presses effects on state
    switch(buttonID)
    {
      case 1:{  // lets call this UP
        current_menu_state = pressUP(current_menu_state);
        break;}
      case 2:{  // lets call this DOWN
        current_menu_state = pressDOWN(current_menu_state);
        break;}
      case 4:{  // lets call this select
        current_menu_state = pressSELECT(current_menu_state);
        break;}
    }
    UpdateMenu(current_menu_state);
  }
  
}

ISR(PCINT0_vect){
  PCFlag = true;
  if (PINB & 0b00010000){
    // SLAM them pins to 0, test with pin 13, PC7, PC6, and PB
    DDRB = DDRB & 0b00011111;
    DDRC = DDRC & 0b00111111;
    DDRD = DDRD & 0b01111111;
  }
}

void setupVPCsensor(INA226 *inp, uint8_t address){
  inp->begin(address);
  inp->configure(INA226_AVERAGES_64, INA226_BUS_CONV_TIME_332US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
  inp->calibrate(0.01, 4.0); // Shunt resistor in Ohm, then max current in A
}

void readLED(INA226 *inp, int ledpos) // uses a few globals
{
  // todo
  powLED[ledpos]  = abs((inp->readBusPower()));
  voltLED[ledpos] = (inp->readBusVoltage());
  currLED[ledpos] = (inp->readShuntCurrent());
}

void printSerial( bool title)
{
  if(title){
    Serial.print("Time");
    Serial.print(", ");
    Serial.print("statusByte, BIN");
    Serial.print(", ");
    Serial.print("Start Time");
    Serial.print(", ");
    Serial.print("Elapsed Time");
    Serial.print(", ");
    Serial.print("Set Time");
    Serial.print(", ");
    Serial.print("Remaining Time");
    Serial.print(", ");
    Serial.print("current Temp");
    Serial.print(", ");
    Serial.print("limit Temp");
    Serial.print(", ");
    Serial.print("Power Target");
    Serial.print(", ");
    Serial.print("Power Limit");
    Serial.print(", ");
    Serial.print("LED1 power");
    Serial.print(", ");
    Serial.print("LED2 power");
    Serial.print(", ");
    Serial.print("LED3 power");
    Serial.print(", ");
    Serial.print("LED4 power");
    Serial.print(", ");
    Serial.print("LED5 power");
    Serial.print(", ");
    Serial.print("LED6 power");
    Serial.print(", ");
    Serial.print("LED1 Energy");
    Serial.print(", ");
    Serial.print("LED2 Energy");
    Serial.print(", ");
    Serial.print("LED3 Energy");
    Serial.print(", ");
    Serial.print("LED4 Energy");
    Serial.print(", ");
    Serial.print("LED5 Energy");
    Serial.print(", ");
    Serial.print("LED6 Energy");
    Serial.print(", ");
    Serial.print("LED1 Voltage");
    Serial.print(", ");
    Serial.print("LED2 Voltage");
    Serial.print(", ");
    Serial.print("LED3 Voltage");
    Serial.print(", ");
    Serial.print("LED4 Voltage");
    Serial.print(", ");
    Serial.print("LED5 Voltage");
    Serial.print(", ");
    Serial.print("LED6 Voltage");
    Serial.print(", ");
    Serial.print("LED1 Current");
    Serial.print(", ");
    Serial.print("LED2 Current");
    Serial.print(", ");
    Serial.print("LED3 Current");
    Serial.print(", ");
    Serial.print("LED4 Current");
    Serial.print(", ");
    Serial.print("LED5 Current");
    Serial.print(", ");
    Serial.print("LED6 Current");
    Serial.print(", ");
    Serial.println(" ");
    Serial.print("S");
    Serial.print(", ");
    Serial.print("binary");
    Serial.print(", ");
    Serial.print("ms");
    Serial.print(", ");
    Serial.print("ms");
    Serial.print(", ");
    Serial.print("ms");
    Serial.print(", ");
    Serial.print("ms");
    Serial.print(", ");
    Serial.print("C");
    Serial.print(", ");
    Serial.print("C");
    Serial.print(", ");
    Serial.print("W");
    Serial.print(", ");
    Serial.print("W");
    Serial.print(", ");
    Serial.print("W");
    Serial.print(", ");
    Serial.print("W");
    Serial.print(", ");
    Serial.print("W");
    Serial.print(", ");
    Serial.print("W");
    Serial.print(", ");
    Serial.print("W");
    Serial.print(", ");
    Serial.print("W");
    Serial.print(", ");
    Serial.print("J");
    Serial.print(", ");
    Serial.print("J");
    Serial.print(", ");
    Serial.print("J");
    Serial.print(", ");
    Serial.print("J");
    Serial.print(", ");
    Serial.print("J");
    Serial.print(", ");
    Serial.print("J");
    Serial.print(", ");
    Serial.print("V");
    Serial.print(", ");
    Serial.print("V");
    Serial.print(", ");
    Serial.print("V");
    Serial.print(", ");
    Serial.print("V");
    Serial.print(", ");
    Serial.print("V");
    Serial.print(", ");
    Serial.print("V");
    Serial.print(", ");
    Serial.print("A");
    Serial.print(", ");
    Serial.print("A");
    Serial.print(", ");
    Serial.print("A");
    Serial.print(", ");
    Serial.print("A");
    Serial.print(", ");
    Serial.print("A");
    Serial.print(", ");
    Serial.print("A");
    Serial.print(", ");
    Serial.println(" "); 
  }
  Serial.print(millis()/1000);
  Serial.print(", ");
  Serial.print(statusByte, BIN);
  Serial.print(", ");
  Serial.print(millisStart);
  Serial.print(", ");
  Serial.print(millisElapse);
  Serial.print(", ");
  Serial.print(millisSet);
  Serial.print(", ");
  Serial.print(millisRemain);
  Serial.print(", ");
  Serial.print(tempCurr);
  Serial.print(", ");
  Serial.print(tempLimit);
  Serial.print(", ");
  Serial.print(powTarget);
  Serial.print(", ");
  Serial.print(powLimit);
  Serial.print(", ");
  Serial.print(powLED[0]);
  Serial.print(", ");
  Serial.print(powLED[1]);
  Serial.print(", ");
  Serial.print(powLED[2]);
  Serial.print(", ");
  Serial.print(powLED[3]);
  Serial.print(", ");
  Serial.print(powLED[4]);
  Serial.print(", ");
  Serial.print(powLED[5]);
  Serial.print(", ");
  Serial.print(enerLED[0]);
  Serial.print(", ");
  Serial.print(enerLED[1]);
  Serial.print(", ");
  Serial.print(enerLED[2]);
  Serial.print(", ");
  Serial.print(enerLED[3]);
  Serial.print(", ");
  Serial.print(enerLED[4]);
  Serial.print(", ");
  Serial.print(enerLED[5]);
  Serial.print(", ");
  Serial.print(voltLED[0]);
  Serial.print(", ");
  Serial.print(voltLED[1]);
  Serial.print(", ");
  Serial.print(voltLED[2]);
  Serial.print(", ");
  Serial.print(voltLED[3]);
  Serial.print(", ");
  Serial.print(voltLED[4]);
  Serial.print(", ");
  Serial.print(voltLED[5]);
  Serial.print(", ");
  Serial.print(currLED[0]);
  Serial.print(", ");
  Serial.print(currLED[1]);
  Serial.print(", ");
  Serial.print(currLED[2]);
  Serial.print(", ");
  Serial.print(currLED[3]);
  Serial.print(", ");
  Serial.print(currLED[4]);
  Serial.print(", ");
  Serial.print(currLED[5]);
  Serial.print(", ");
  Serial.println(" ");
}
/*int backlightScaling( int percent){
  //Serial.println( (int)(5.11*(pow(10,1.699*((float)percent/100.0))-1))  );
  return          (int)(5.11*(pow(10,1.699*((float)percent/100.0))-1));}
int contrastScaling( int percent){
  //Serial.println( (int)((255/5.0)*(6.0*pow((float)percent/100.0-0.465, 3)+0.6))  );
  return (int)((255/5.0)*(6.0*pow((float)percent/100.0-0.465, 3)+0.6));}
*/

void FormatMillis(char buff[], unsigned long inputmilli) // uses no global variables
{
  //takes an input millis() result and modifies a char buffer of format "00D00:00:00" to display that millis()
  unsigned long seconds = (inputmilli % MILLIINMIN)/MILLIINSEC;
  unsigned long minutes = (inputmilli % MILLIINHOUR)/MILLIINMIN;  
  unsigned long hours = (inputmilli % MILLIINDAY)/MILLIINHOUR;  
  unsigned long days = (inputmilli / MILLIINDAY); 

  buff[0] = '0' + int(days/10);
  buff[1] = '0' + int(days%10);

  buff[3] = '0' + int(hours/10);
  buff[4] = '0' + int(hours%10);

  buff[6] = '0' + int(minutes/10);
  buff[7] = '0' + int(minutes%10);
  
  buff[9] = '0' + int(seconds/10);
  buff[10] = '0' + int(seconds%10);
}




void UpdateMenu(int state) // uses LOTS of globals
{
  lcd.noCursor();
  delay(5);
  lcd.clear();
  switch(state)
  {
    case SCROLL_START:{             // global variables: statusString, statusByte
      lcd.print("RUN: Start/Stop");
      if (((statusByte>>6)&0x03)){lcd.setCursor(11,0);}
      else                       {lcd.setCursor( 5,0);}
      lcd.write(2);
      lcd.setCursor(0,1);
      lcd.print("STATUS: ");
      lcd.print(statusString);
      lcd.setCursor(0,0);
      break;}
    case START_ON:{             // global variables: statusString, statusByte
      UpdateMenu(SCROLL_START);
      lcd.setCursor(5,0);
      break;}
    case START_OFF:{             // global variables: statusString, statusByte
      UpdateMenu(SCROLL_START);
      lcd.setCursor(11,0);
      break;}
    case SCROLL_STATUS:{             // global variables: statusString, tempCurr, tempLimit
      //row1
      lcd.print("STATUS: ");
      lcd.print(statusString);
      //row2
      lcd.setCursor(0,1);
      char buff[17]="      ";
      sprintf(buff, "TEMP:%2d  LIM:%2d ",tempCurr,tempLimit);
      lcd.print(buff);
      lcd.setCursor(7,1);
      lcd.write(1);
      lcd.setCursor(15,1);
      lcd.write(1);
      lcd.setCursor(0,0);
      break;}
    case SCROLL_TEMP:{             // global variables: tempCurr, tempLimit, elapTime
      //row1
      char buff[17]="      ";
      sprintf(buff, "TEMP:%2d  LIM:%2d ",tempCurr,tempLimit);
      lcd.print(buff);
      lcd.setCursor(7,0);
      lcd.write(1);
      lcd.setCursor(15,0);
      lcd.write(1);
      //row2
      lcd.setCursor(0,1);
      lcd.print("ELAP:");
      lcd.setCursor(5,1);
      lcd.print(timeElapse);
      //set blink location
      lcd.setCursor(0,0);
      break;}
    case TEMP_TENS:{             // global variables: tempCurr, tempLimit, elapTime
      UpdateMenu( SCROLL_TEMP);
      lcd.setCursor(13,0);
      break;}
    case TEMP_ONES:{             // global variables: tempCurr, tempLimit, elapTime
      UpdateMenu( SCROLL_TEMP);
      lcd.setCursor(14,0);
      break;}
    case SCROLL_ELAPSED:{             // global variables: timeElapse, timeSet
      //row1
      lcd.print("ELAP:");
      lcd.setCursor(5,0);
      lcd.print(timeElapse);
      //row2
      lcd.setCursor(0,1);
      lcd.print("SET :");
      lcd.setCursor(5,1);
      lcd.print(timeSet);
      //set blink location
      lcd.setCursor(0,0);
      break;}
    case SCROLL_SET_T:{             // global variables: timeSet, timeRemain
      //row1
      lcd.print("SET :");
      lcd.setCursor(5,0);
      lcd.print(timeSet);
      //row2
      lcd.setCursor(0,1);
      lcd.print("REM :");
      lcd.setCursor(5,1);
      lcd.print(timeRemain);
      //set blink location
      lcd.setCursor(0,0);
      break;}
    case SET_DAY_TENS:{             // global variables: timeSet, timeRemain
      UpdateMenu(SCROLL_SET_T);
      lcd.setCursor(5,0);
      break;}
    case SET_DAY_ONES:{             // global variables: timeSet, timeRemain
      UpdateMenu(SCROLL_SET_T);
      lcd.setCursor(6,0);
      break;}
    case SET_HR_TENS:{             // global variables: timeSet, timeRemain
      UpdateMenu(SCROLL_SET_T);
      lcd.setCursor(8,0);
      break;}
    case SET_HR_ONES:{             // global variables: timeSet, timeRemain
      UpdateMenu(SCROLL_SET_T);
      lcd.setCursor(9,0);
      break;}
    case SET_MIN_TENS:{             // global variables: timeSet, timeRemain
      UpdateMenu(SCROLL_SET_T);
      lcd.setCursor(11,0);
      break;}
    case SET_MIN_ONES:{             // global variables: timeSet, timeRemain
      UpdateMenu(SCROLL_SET_T);
      lcd.setCursor(12,0);
      break;}
    case SET_SEC_TENS:{             // global variables: timeSet, timeRemain
      UpdateMenu(SCROLL_SET_T);
      lcd.setCursor(14,0);
      break;}
    case SET_SEC_ONES:{             // global variables: timeSet, timeRemain
      UpdateMenu(SCROLL_SET_T);
      lcd.setCursor(15,0);
      break;}
    case SCROLL_REMAINING:{             // global variables: timeRemain, powTarget
      //row1
      char buff[17]="      ";
      lcd.print("REM :");
      lcd.setCursor(5,0);
      lcd.print(timeRemain);
      //row2
      lcd.setCursor(0,1);
      sprintf(buff, "Target Pow: %3dW", powTarget);
      lcd.print(buff);
      //set blink location
      lcd.setCursor(0,0);
      break;}
    case SCROLL_LED_1:{             // global variables: powLED1, enerLED[0]
      //row1
      char buff[17]="      ";
      sprintf(buff, "LED1: POW:%4dW", (int)powLED[0]);
      lcd.print(buff);
      //row2
      lcd.setCursor(0,1);
      sprintf(buff, " ENERGY:%6ldWh", (enerLED[0]*1000/(unsigned long)MILLIINHOUR));
      lcd.print(buff);
      //set blink location
      lcd.setCursor(0,0);
      break;}
    case SCROLL_LED_2:{             // global variables: powLED[1], enerLED[1]
      //row1
      char buff[17]="      ";
      sprintf(buff, "LED2: POW:%4dW", (int)powLED[1]);
      lcd.print(buff);
      //row2
      lcd.setCursor(0,1);
      sprintf(buff, " ENERGY:%6ldWh", (enerLED[1]*1000/(unsigned long)MILLIINHOUR));
      lcd.print(buff);
      //set blink location
      lcd.setCursor(0,0);
      break;}
    case SCROLL_LED_3:{             // global variables: powLED[2], enerLED[2]
      //row1
      char buff[17]="      ";
      sprintf(buff, "LED3: POW:%4dW", (int)powLED[2]);
      lcd.print(buff);
      //row2
      lcd.setCursor(0,1);
      sprintf(buff, " ENERGY:%6ldWh", (enerLED[2]*1000/(unsigned long)MILLIINHOUR));
      lcd.print(buff);
      //set blink location
      lcd.setCursor(0,0);
      break;}
    case SCROLL_LED_4:{             // global variables: powLED[3], enerLED[3]
      //row1
      char buff[17]="      ";
      sprintf(buff, "LED4: POW:%4dW", (int)powLED[3]);
      lcd.print(buff);
      //row2
      lcd.setCursor(0,1);
      sprintf(buff, " ENERGY:%6ldWh", (enerLED[3]*1000/(unsigned long)MILLIINHOUR));
      lcd.print(buff);
      //set blink location
      lcd.setCursor(0,0);
      break;}
    case SCROLL_LED_5:{             // global variables: powLED[4], enerLED[4]
      //row1
      char buff[17]="      ";
      sprintf(buff, "LED5: POW:%4dW", (int)powLED[4]);
      lcd.print(buff);
      //row2
      lcd.setCursor(0,1);
      sprintf(buff, " ENERGY:%6ldWh", (enerLED[4]*1000/(unsigned long)MILLIINHOUR));
      lcd.print(buff);
      //set blink location
      lcd.setCursor(0,0);
      break;}
    case SCROLL_LED_6:{             // global variables:  powLED[5], enerLED[5]
      //row1
      char buff[17]="      ";
      sprintf(buff, "LED6: POW:%4dW", (int)powLED[5]);
      lcd.print(buff);
      //row2
      lcd.setCursor(0,1);
      sprintf(buff, " ENERGY:%6ldWh", (enerLED[5]*1000/(unsigned long)MILLIINHOUR));
      lcd.print(buff);
      //set blink location
      lcd.setCursor(0,0);
      break;}
    case IDLER:{             // global variables: all of em PLUS the interrupt_flag for a button press
      UpdateMenu(idleStates[(int)(idleCounter/2)%numIdles]);
      idleCounter++;
      if (idleCounter > (3*numIdles)){
        idleCounter = 0;
        current_menu_state = SLEEPER;
      }
      /*const int delayTime = 2000;
      int i=0;
      millisScreenTimeOut = millis()+5*screenTimeOutPeriod;
      while(millisScreenTimeOut > millis()){
        long int startTime = millis();
        UpdateMenu(idleStates[i%numIdles]); // TEMPORARY
        i++; // TEMPORARY
        //UpdateMenu(idleStates[(i++)%numIdles]);  // TEMPORARY
        while((millis()-startTime)<delayTime && !PCFlag ){
          delay(10);
        }
        if(PCFlag){
          break;
        }
      }
      current_menu_state = SLEEPER;
      */
      break;}
    case SLEEPER:{             // global variables: none, hopefully just turn the display off
      //lcd.noDisplay();       // this doesnt save energy tho
      digitalWrite(bl,LOW);    // this does
      break;}
    case SCROLL_LCD_BRIGHT:{             // options 1) lcd brightness overall
      //row1
      char buff[17]="      ";
      sprintf(buff, "LCD Bright: %3d%%", lcdbright);
      lcd.print(buff);
      //row2
      lcd.setCursor(0,1);
      sprintf(buff, "LCD Contr : %3d%%", 100  );
      lcd.print(buff);
      //set blink location
      lcd.setCursor(0,0);
      break;}  
    case LCD_BRIGHT_FIVES:{             // lcd brightness incrementer (1a)
      UpdateMenu(SCROLL_LCD_BRIGHT);
      lcd.setCursor(15,0);
      break;}      
    case SCROLL_LCD_CONTR:{             //lcd contrast overall
      //row1
      char buff[17]="      ";
      sprintf(buff, "LCD Contr : %3d%%", 100);
      lcd.print(buff);
      //row2
      lcd.setCursor(0,1);
      sprintf(buff, "Buzzer:     %3d%%", buzzervol); // values are either 100 or 0 i think for now
      lcd.print(buff);
      //set blink location
      lcd.setCursor(0,0);
      break;} 
    case LCD_CONTR_FIVES:{             //   lcd contrast incrementer (2a)
      UpdateMenu(SCROLL_LCD_CONTR);
      lcd.setCursor(15,0);
      break;}      
    case SCROLL_BUZZER:{             // buzzer option overall ( on off?) (3)
      //row1
      char buff[17]="  ";
      sprintf(buff, "Buzzer : %3d%", buzzervol);
      lcd.print(buff);
      //row2
      lcd.setCursor(0,1);
      sprintf(buff, "                ");
      lcd.print(buff);
      //set blink location
      lcd.setCursor(0,0);
      break;}  
    case BUZZER_TOGGLE:{             // buzzer incrementer (3a)
      UpdateMenu(SCROLL_BUZZER);
      lcd.setCursor(15,0);
      break;}  
    case SCROLL_LED_POW:{             // Globals used powLED , powtarget
      //row1
      char buff[17]=" ";
      sprintf(buff, "Target Pow: %3dW", powTarget);
      lcd.print(buff);
      //row2
      lcd.setCursor(0,1);
      sprintf(buff, "LED1: POW:%4dW", (int)powLED[0]);
      lcd.print(buff);
      //set blink location
      lcd.setCursor(0,0);
    break;}
    case LED_POW_HUNDOS:{
      UpdateMenu(SCROLL_LED_POW);
      lcd.setCursor(12,0); // temporary
    break;}
    case LED_POW_TENS:{
      UpdateMenu(SCROLL_LED_POW);
      lcd.setCursor(13,0); // temporary
    break;}
    case LED_POW_ONES:{
      UpdateMenu(SCROLL_LED_POW);
      lcd.setCursor(14,0); // temporary
    break;}
  }
  lcd.cursor();
}


int pressUP(int state){ // global variables used in all: tempLimit, millisSet
  switch(state)
  {
    case SCROLL_START:{break;}
    case START_ON:{break;}
    case START_OFF:{break;}
    case SCROLL_STATUS:{break;}
    case SCROLL_TEMP:{break;}
    case TEMP_TENS:{             // global variables: tempLimit
      tempLimit = min(tempLimit+10, 90);
      // test negative values cases
      break;}
    case TEMP_ONES:{             // global variables: tempLimit
      tempLimit = min(tempLimit+1 , 90);
      // test negative values cases
      break;}
    case SCROLL_ELAPSED:{break;}
    case SCROLL_SET_T:{break;}
    case SET_DAY_TENS:{             // global variables: 
      millisSet = min((millisSet+10*MILLIINDAY) , (48*MILLIINDAY));
      FormatMillis(timeSet, millisSet);
      //
      break;}
    case SET_DAY_ONES:{             // global variables: 
      millisSet = min((millisSet+1*MILLIINDAY) , (48*MILLIINDAY));
      FormatMillis(timeSet, millisSet);
      //
      break;}
    case SET_HR_TENS:{             // global variables: 
      millisSet = min((millisSet+10*MILLIINHOUR) , (48*MILLIINDAY));
      FormatMillis(timeSet, millisSet);
      //
      break;}
    case SET_HR_ONES:{             // global variables:  
      millisSet = min((millisSet+1*MILLIINHOUR) , (48*MILLIINDAY));
      FormatMillis(timeSet, millisSet);
      //
      break;}
    case SET_MIN_TENS:{             // global variables: 
      millisSet = min((millisSet+10*MILLIINMIN) , (48*MILLIINDAY));
      FormatMillis(timeSet, millisSet);
      //
      break;}
    case SET_MIN_ONES:{             // global variables: 
      millisSet = min((millisSet+1*MILLIINMIN) , (48*MILLIINDAY));
      FormatMillis(timeSet, millisSet);
      //
      break;}
    case SET_SEC_TENS:{             // global variables:  
      millisSet = min((millisSet+10*MILLIINSEC) , (48*MILLIINDAY)); //(millisSet+10*MILLIINSEC) % (48*MILLIINDAY);
      FormatMillis(timeSet, millisSet);
      //
      break;}
    case SET_SEC_ONES:{             // global variables: 
      millisSet = min((millisSet+1*MILLIINSEC) , (48*MILLIINDAY));
      FormatMillis(timeSet, millisSet);
      //
      break;}
    case SCROLL_REMAINING:{break;}
    case SCROLL_LED_1:{break;}
    case SCROLL_LED_2:{break;}
    case SCROLL_LED_3:{break;}
    case SCROLL_LED_4:{break;}
    case SCROLL_LED_5:{break;}
    case SCROLL_LED_6:{break;}
    case IDLER:{break;}
    case SLEEPER:{
      digitalWrite(bl,HIGH);
      break;}
    case SCROLL_LCD_BRIGHT:{break;}      // options 1) lcd brightness overall
    case LCD_BRIGHT_FIVES:{             // lcd brightness incrementer (1a)
      //lcdbright = min(lcdbright+5,100);
      //analogWrite(bl, backlightScaling(lcdbright)); //
      break;}  
    case SCROLL_LCD_CONTR:{break;}      // 2) lcd contrast overall
    case LCD_CONTR_FIVES:{             //   lcd contrast incrementer (2a)
      //lcdcontr = min(lcdcontr+5,100);
      //analogWrite(contr, contrastScaling(lcdcontr)); // contrast didnt make it into design
      break;}      
    case SCROLL_BUZZER:{break;}      // buzzer option overall ( on off?) (3)
    case BUZZER_TOGGLE:{             // buzzer incrementer (3a)
      buzzervol = min(buzzervol+100,100);
      break;}
    case SCROLL_LED_POW:{break;}
    case LED_POW_HUNDOS:{
      powTarget = min(powTarget+100, 150);
    break;}
    case LED_POW_TENS:{
      powTarget = min(powTarget+10 , 150);
    break;}
    case LED_POW_ONES:{
      powTarget = min(powTarget+1  , 150);
    break;}      
  }
  return transition_up[state];
}


int pressDOWN(int state){ // global variables used in all: tempLimit, millisSet
  switch(state)
  {
    case SCROLL_START:{break;}
    case START_ON:{break;}
    case START_OFF:{break;}
    case SCROLL_STATUS:{break;}
    case SCROLL_TEMP:{break;}
    case TEMP_TENS:{             // global variables: tempLimit
      tempLimit = max(tempLimit-10,0);
      // test negative values cases
      break;}
    case TEMP_ONES:{             // global variables: tempLimit
      tempLimit = max(tempLimit-1, 0);
      // test negative values cases
      break;}
    case SCROLL_ELAPSED:{break;}
    case SCROLL_SET_T:{break;}
    case SET_DAY_TENS:{             // global variables: millisSet
      millisSet = max(0, (millisSet-10*MILLIINDAY));// % (48*MILLIINDAY);
      FormatMillis(timeSet, millisSet);
      break;}
    case SET_DAY_ONES:{             // global variables: millisSet
      millisSet = max(0, (millisSet-1*MILLIINDAY));// % (48*MILLIINDAY);
      FormatMillis(timeSet, millisSet);
      break;}
    case SET_HR_TENS:{             // global variables: millisSet
      millisSet = max(0, (millisSet-10*MILLIINHOUR));// % (48*MILLIINDAY);
      FormatMillis(timeSet, millisSet);
      break;}
    case SET_HR_ONES:{             // global variables: millisSet
      millisSet = max(0, (millisSet-1*MILLIINHOUR));// % (48*MILLIINDAY);
      FormatMillis(timeSet, millisSet);
      break;}
    case SET_MIN_TENS:{             // global variables: millisSet
      millisSet = max(0, (millisSet-10*MILLIINMIN));// % (48*MILLIINDAY);
      FormatMillis(timeSet, millisSet);
      break;}
    case SET_MIN_ONES:{             // global variables: millisSet
      millisSet = max(0, (millisSet-1*MILLIINMIN));  // % (48*MILLIINDAY);
      FormatMillis(timeSet, millisSet);
      break;}
    case SET_SEC_TENS:{             // global variables: millisSet
      millisSet = max(0, (millisSet-10*MILLIINSEC));         //(millisSet-10*MILLIINSEC) % (48*MILLIINDAY);
      FormatMillis(timeSet, millisSet);
      break;}
    case SET_SEC_ONES:{             // global variables: millisSet
      millisSet = max(0,(millisSet-1*MILLIINSEC));
      FormatMillis(timeSet, millisSet);
      break;}
    case SCROLL_REMAINING:{break;}
    case SCROLL_LED_1:{break;}
    case SCROLL_LED_2:{break;}
    case SCROLL_LED_3:{break;}
    case SCROLL_LED_4:{break;}
    case SCROLL_LED_5:{break;}
    case SCROLL_LED_6:{break;}
    case IDLER:{break;}
    case SLEEPER:{
      digitalWrite(bl,HIGH);
      break;}
    case SCROLL_LCD_BRIGHT:{break;}      // options 1) lcd brightness overall
    case LCD_BRIGHT_FIVES:{             // lcd brightness incrementer (1a)
      //lcdbright = max(lcdbright-5,0);
      //analogWrite(bl, backlightScaling(lcdbright)); //
      break;}  
    case SCROLL_LCD_CONTR:{break;}      // 2) lcd contrast overall
    case LCD_CONTR_FIVES:{             //   lcd contrast incrementer (2a)
      //lcdcontr = max(lcdcontr-5,0);
      //analogWrite(contr, contrastScaling(lcdcontr)); // lcd contrast circuit didnt end up making it in
      break;}      
    case SCROLL_BUZZER:{break;}      // buzzer option overall ( on off?) (3)
    case BUZZER_TOGGLE:{             // buzzer incrementer (3a)
      buzzervol = max(buzzervol-100,0);
      // buzzer didnt end up making it in
      break;}      
    case SCROLL_LED_POW:{break;}
    case LED_POW_HUNDOS:{
      powTarget = max(powTarget-100, 0);
    break;}
    case LED_POW_TENS:{
      powTarget = max(powTarget-10 , 0);
    break;}
    case LED_POW_ONES:{
      powTarget = max(powTarget-1  , 0);
    break;}
  }
  return transition_down[state];
}

int pressSELECT(int state){ // global variables used in all: transition_select
  switch(state)
  {
    case SCROLL_START:{             // global variables: 
      // Do nothing special
      break;}
    case START_ON:{             // global variables: 
      // Start the program ( a set of functions im not 100% sure on yet) OR just set the running flag and deal with the rest more openly?
      statusByte &= 0b01111111;
      printSerial( 1 );
      break;}
    case START_OFF:{             // global variables: 
      // STOP the program (see above)
      statusByte |= 0b10000000;
      break;}
    case SCROLL_STATUS:{break;}
    case SCROLL_TEMP:{break;}
    case TEMP_TENS:{break;}
    case TEMP_ONES:{break;}
    case SCROLL_ELAPSED:{break;}
    case SCROLL_SET_T:{break;}
    case SET_DAY_TENS:{break;}
    case SET_DAY_ONES:{break;}
    case SET_HR_TENS:{break;}
    case SET_HR_ONES:{break;}
    case SET_MIN_TENS:{break;}
    case SET_MIN_ONES:{break;}
    case SET_SEC_TENS:{break;}
    case SET_SEC_ONES:{break;}
    case SCROLL_REMAINING:{break;}
    case SCROLL_LED_1:{break;}
    case SCROLL_LED_2:{break;}
    case SCROLL_LED_3:{break;}
    case SCROLL_LED_4:{break;}
    case SCROLL_LED_5:{break;}
    case SCROLL_LED_6:{break;}
    case IDLER:{break;}
    case SLEEPER:{
      digitalWrite(bl,HIGH);
      break;}
    case SCROLL_LCD_BRIGHT:{break;}  // options 1) lcd brightness overall
    case LCD_BRIGHT_FIVES:{break;}  // lcd brightness incrementer (1a)
    case SCROLL_LCD_CONTR:{break;}  // 2) lcd contrast overall
    case LCD_CONTR_FIVES:{break;}  //   lcd contrast incrementer (2a)
    case SCROLL_BUZZER:{break;}  // buzzer option overall ( on off?) (3)
    case BUZZER_TOGGLE:{break;}  // buzzer incrementer (3a)
    case SCROLL_LED_POW:{break;}
    case LED_POW_HUNDOS:{break;}
    case LED_POW_TENS:{break;}
    case LED_POW_ONES:{break;}
  }
  return transition_select[state];
}

// this just breaks the normal control loop into parts so i can break up long functions 
// and avoid blocking the pcflag function for too long
// since while the thing is running it doesnt need to be too responsive, this is no problem
void UpdateSystem(int state) 
{
  // step 0 checks if anything has forced the LEDs to Stop, and if so, skips the next 5 steps
  // steps 1-5 update timers that are based on the LEDs being on (eg elapsed time or total power usage)
  // steps 6-10 check the various limits we have (eg door is open, over temp, over wattage)
  // step 14 updates the screen, i could do this more than once per loop through if i want the screen to seem more responsive
  switch(state)
  {
    case 0:{              // check to make sure nothing needs our attention
      statusString = statusOptions[0];
      if(statusByte){
        // 1) stop LEDs
        DDRB = DDRB & 0b00011111;
        DDRC = DDRC & 0b00111111;
        DDRD = DDRD & 0b01111111;
        // 2) figure out what kind of statusbyte
        int iS;
        for(iS = 0; iS <8; iS++){
          if((statusByte>>iS)&0x01){break;}
        }
        statusString = statusOptions[iS+1];
        sysStep = 5; // cheating a bit here to skip over the time based steps
      }
      break;}
    case 1:{              // INCREMENT elapsed timer
      millisElapse += updatePeriod;
      FormatMillis(timeElapse, millisElapse);
      break;}
    case 2:{              // DECREMENT remain timer
      millisRemain = millisSet-millisElapse;
      if(millisRemain > millisSet){
        millisRemain = 0;
      }
      FormatMillis(timeRemain, millisRemain);
      break;}
    case 3:{              // INCREMENT J readings
      int iP;
      for(iP=0;iP<numLEDS;iP++){
        enerLED[iP] += powLED[iP]*updatePeriod/MILLIINSEC; // awkwardly i only now realize ive been working in Joules
      }
      break;}
    case 4:{              // adjust led levels to match power very lazily though
      //pinLevel
      int iP;
      for(iP=0;iP<numLEDS;iP++){
        // the first min is to make sure at lo levels, the extrapolation doesnt go TOO nuts (since linear extrapolation is a lie anyway)
        // the second is to make sure that i dont go over the max output
        // the two maxes are just so that in case the current level is 0, i dont get errors, call it a bit of noise to get the algorithm going
        // the random number should help it get out of any weird spots, hopefully
        pinLevel[iP] = min( pinLevel[iP]*2+1,min(255,(int)((2*pinLevel[iP]+(float)powTarget*(max((float)pinLevel[iP],1)/max((float)powLED[iP],0.1)))/3)+random(-3,3))) ;
        analogWrite(pinLED[iP], pinLevel[iP]);
      }
      
      break;}     
    case 5:{break;}       // reserved for future
    case 6:{              // test Elapsed time
      if(millisElapse > millisSet){
        statusByte |= 0b01000000;
      } // has not less than check because i want to force a reset
      break;}
    case 7:{              // test Temps
      tempCurr = int(dht.readTemperature())%100;
      if(tempCurr>tempLimit){statusByte |= 0b00000010;}
      else                  {statusByte &= 0b11111101;}
      break;}
    case 8:{              // test DOOR
      if (PINB & 0b00010000){statusByte |= 0b00000001;}
      else                  {statusByte &= 0b11111110;}
      break;}
    case 9:{              // test LED Wattage
      int iP;
      for(iP=0;iP<numLEDS;iP++){
        if (powLED[iP] > powLimit){
          statusByte |= 0b00000100;
          // 1.a) make sure it doesnt happen again by lowering the power target and pinlevels
          int iP2;
          for(iP2=0;iP2<numLEDS;iP2++){
            pinLevel[iP2] = (int)(pinLevel[iP2]/2) ;        
          }
          powTarget = min(powTarget, powLimit-5); 
          break;
        }
        else {statusByte &= 0b11111011;}
      }
      break;}
    case 10:{             // test screen time out
      /*if ((current_menu_state != IDLER) && (millis()> millisScreenTimeOut)){
        current_menu_state = IDLER;
      } TEMPORARY */
      break;}
    case 11:{             // read the LED values
      //int iP;
      //for(iP=0;iP<numLEDS;iP++){ //TEST 
      //  readLED(iP);
      //}
      readLED(&ina0 , 0);
      readLED(&ina1 , 1);
      readLED(&ina2 , 2);
      readLED(&ina3 , 3);
      readLED(&ina4 , 4);
      readLED(&ina5 , 5);
      break;}
    case 12:{             // do serial communication of all the variables
      serialCounter = (serialCounter+1)%serialDelay;
      if ( !serialCounter){
      printSerial( 0 );
      }
      break;}
    case 13:{break;}
    case 14:{
      UpdateMenu(current_menu_state);
      break;}
    case 15:{break;}
  }
}



