/********************
Sept. 2014 ~ Oct 2016 Rui Azevedo - ruihfazevedo(@rrob@)gmail.com

menu with UTFT (tested on arduino due)
output: 3.2″ TFT LCD Module Display 240X320
input: Serial + Touch Panel
www.r-site.net

UTFT library from:
  http://www.rinkydinkelectronics.com/library.php?id=51
  http://www.rinkydinkelectronics.com/library.php?id=92

Note: I was unable to build for esp8266 - neu-rah
***/

  /***
    Advance to the next address, when at the end restart at the beginning.

    Larger AVR processors have larger EEPROM sizes, E.g:
    - Arduino Duemilanove: 512 B EEPROM storage.
    - Arduino Uno:         1 kB EEPROM storage.
    - Arduino Mega:        4 kB EEPROM storage.

    Rather than hard-coding the length, you should use the pre-provided length function.
    This will make your code portable to all AVR processors.
  ***/
 
#include <Arduino.h>
#include <menu.h>
#include <menuIO/utftOut.h>
//#include <menuIO/urtouchIn.h>
#include <TimerOne.h>
#include <ClickEncoder.h>
#include <menuIO/clickEncoderIn.h>
#include <menuIO/keyIn.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>
#include <menuIO/chainStream.h>
#include <plugin/userMenu.h>
#include <plugin/cancelField.h>
#include <plugin/barField.h>

#include <FastPID.h>              // Include PID Library     
#include <AccelStepper.h>
#include <EEPROM.h>

#include <Adafruit_ADS1X15.h>


using namespace Menu;

UTFT tft(SSD1289 ,38,39,40,41);
//extern uint8_t SmallFont[];
extern uint8_t SmallFont[];
extern uint8_t SevenSegmentFull[];
extern uint8_t DotMatrix_M_Slash[];
extern uint8_t BigFont[];
//extern uint8_t SevenSegNumFont[];

#define LEDPIN 13
#define PLASMA_INPUT_PIN A0
#define STEP_PIN 8      // Direction
#define DIR_PIN 9       // Step

AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

Adafruit_ADS1115 ads;

//Define Variables
double Input = 0;
float targetInput;
float gap;
float scale;
long threshold=74000;
long currentGap;
uint32_t oldDelay;
uint32_t arcStabilizeDelay;
long SetPoint = 6335;
long CalibrationOffset = 0;


//Specify the links and initial tuning parameters
float aggKp = 0.175, aggKi = 0.1, aggKd = 0.1;
float Kp = 0.075, Ki = 0.01, Kd = 0.01;
float Hz = 8;
int output_bits = 16;
bool output_signed = true;
bool alreadySetColor = false;

FastPID THCPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
////////////////////////////////////////////////////////////////////////////
long RPMD=1000;
int RPMvfd;
int minimo=10;
int maximo=80;
int automanual=0;
int hsensor=90;
int holgura=20;
int porcentaje=0;
int ledCtrl=LOW;

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

int Tiempo =0;
int tempInicioCiclo =0;
int temperaturaResMax =0;

int Tiempo_cal =0;
int tempInicioCiclo_cal =0;
int temperaturaResMax_cal =0;

int buff_time;

String target1;
String string_buff;

constexpr int dataSz=6; //cantidad de recetas, maximo 23...0-22
constexpr int nameSz=11;// longitud de nombre de receta

char char_buff[nameSz+1];
char idxchar[dataSz];

int id;
int id2;
int idx_update;
int number;
int test=0;

int tempAct;
int tempActual;
int temp_Recamara; 
int temp_Resistencia ; 
//int temperaturaResMax2=300;
//int temperaturaResMax3=200;

int temperaturaResMax2=240;
int temperaturaResMax3=240;

int TAPA_CERRADA=0; 
int CALENTADO=0;
int CALIBRAR=0;
int tiempo_minutos=0;
int tiempo_segundos=0;
int8_t offset1 =0;
int8_t offset2 =0;
int pass=1234;

int button1_State = 0, button2_State = 0, button3_State = 0, button4_State = 0; 
int prestate =0;

unsigned long lastMillis = 0;

bool iniciar = false;
///////////////////////////////////////////////////////////////////////////////////////
//movement
long steps_per_mm = 200;
float pos = 0;
float adjpos = 0;
long minPos = -(40 * steps_per_mm);
long maxPos = (40 * steps_per_mm);
long moveAmt = 0;
uint8_t output = 0;

// Encoder /////////////////////////////////////
#define encA 45
#define encB 47
//this encoder has a button here
#define encBtn 49


ClickEncoder clickEncoder(encA,encB,encBtn,2);
ClickEncoderStream encStream(clickEncoder,1);
MENU_INPUTS(in,&encStream);
void timerIsr() {clickEncoder.service();}




//some user data record type to be edited by the menu
struct Data {
  char name[nameSz+1]=  "<vacio>       ";

};

//THE DATA <-----------------------------------------
Data myTargets[dataSz];//our data
Data target;

//characters allowed on name field
const char* constMEM alphaNum MEMMODE=" 0123456789.ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz,\\|!\"#$%&/()=?~*^+-{}[]€";
const char* constMEM alphaNumMask[] MEMMODE={alphaNum};

char* constMEM hexDigit MEMMODE="0123456789";
char* constMEM hexNr[] MEMMODE={hexDigit,hexDigit,hexDigit,hexDigit};
char buf1[]="****";

  result datos2() {EEPROM.write(1, offset1);return proceed;}
  result datos3() {EEPROM.write(3, offset2);return proceed;}
 // result datos4() {return proceed;}
 // result datos5() {return proceed;}
  result dataSize(){EEPROM.write(0, test);return proceed;}
  result datUpdate(){/*update_data();*/ return proceed;} 
  result saveoffset(){ return proceed;}
  result callEvent() {
  Serial.print("call event: ");
//  lcd.clear();
  CALIBRAR=1;
  //load_data();
  //delay(1200);
  return proceed;
}


//a function to save the edited data record
result saveTarget(eventMask e,navNode& nav) {
  trace(MENU_DEBUG_OUT<<"saveTarget"<<endl);
  navNode& nn=nav.root->path[nav.root->level-1];
  idx_t n=nn.sel;//get selection of previous level
  myTargets[n]=target;
  id=n;
  //save_data();
  return quit;
}

MENU(subMenu1,"-Configurar",datUpdate,enterEvent,noStyle
   ,FIELD(Tiempo,"-SetPoint"," Volt",0,500,10,1,datos1,exitEvent,wrapStyle)
   ,FIELD(tempInicioCiclo,"-ArcDelay","msec",0,400,10,1,datos2,exitEvent,wrapStyle)
   ,FIELD(temperaturaResMax,"-Offset"," Volt",0,400,10,1,datos3,exitEvent,wrapStyle)
   ,EXIT("<Regresar")
  );

MENU(targetEdit,"Editar receta",doNothing,noEvent,wrapStyle
  ,EDIT("Name",target.name,alphaNumMask,doNothing,noEvent,noStyle)
  ,SUBMENU(subMenu1) 
  ,OP("-Salvar",saveTarget,enterEvent)
  ,EXIT("<Regresar")
);

//handling the user menu selection
//this will copy the selected recotd into the temp var
result targetEvent(eventMask e,navNode& nav);

  struct TargetMenu:UserMenu{
  using UserMenu::UserMenu;
  Used printItem(menuOut& out, int idx,int len) override {
   return len?out.printText(myTargets[idx].name,len):0;
  }
};

//build the user menu object, optionally giving a sub menu
#ifdef MENU_USERAM
  //for non-AVR devices or when MENU_USERAM is defined at compiler level
  //TargetMenu targetsMenu("Targets",dataSz,"<Back",targetEdit,targetEvent,enterEvent);
    TargetMenu targetsMenu("-Recetas",dataSz,targetEdit,targetEvent,enterEvent);//no exit option
#else
  //menu allocation compatible with AVR flash ---------------------------------
  constMEM char targetsMenuTitle[] MEMMODE="-Recetas";
  constMEM menuNodeShadowRaw targetsMenuInfoRaw MEMMODE={
    (callback)targetEvent,
    (systemStyles)(_menuData|_canNav),
    targetsMenuTitle,
    enterEvent,
    wrapStyle,
    dataSz,
    NULL
  };
  constMEM menuNodeShadow& targetsMenuInfo=*(menuNodeShadow*)&targetsMenuInfoRaw;
    TargetMenu targetsMenu(targetsMenuInfo,targetEdit,"<Regresar");//no exit option
#endif

 //customizing a prompt look!
  //by extending the prompt class
  class recetaPrompt:public prompt {
  public:
 unsigned int t=0;
  unsigned int last=0;
    recetaPrompt(constMEM promptShadow& p):prompt(p) {}
    Used printTo(navRoot &root,bool sel,menuOut& out, idx_t idx,idx_t len,idx_t) override {
      last=t;
     // return out.printRaw(myTargets[2].name,len);
      return out.printText((myTargets[test].name),len); 
   //   return len?out.printText(myTargets[idx].name,len):0;
    // return out.printRaw(F("special prompt!"),len);;
    }

   virtual bool changed(const navNode &nav,const menuOut& out,bool sub=true) {
    t=millis()/1000;
    return last!=t;
  }

//   virtual void clearChanged(const navNode &nav,const menuOut& out,bool sub) {
//    dirty = false;
//    t=millis()/1000;
//    if (last!=t) {
//      dirty = true;
//    }
//     }
  };

   MENU(subMenu,"-Opciones",showEvent,exitEvent,noStyle
   ,EDIT("-Pass",buf1,hexNr,datos1,exitEvent,wrapStyle)
   ,FIELD(offset1,"-Offset1","\xDF C",-200,200,1,1,datos2,exitEvent,wrapStyle)
   ,FIELD(offset2,"-Offset2","\xDF C",-200,200,1,1,datos3,exitEvent,wrapStyle)
   ,FIELD(Tiempo_cal,"-Tiempo"," sec",0,500,10,1,datos6,exitEvent,wrapStyle)
   ,FIELD(tempInicioCiclo_cal,"-Tempinic","\xDF C",0,400,10,1,datos4,exitEvent,wrapStyle)
   ,FIELD(temperaturaResMax_cal,"-TempRes","\xDF C",0,400,10,1,datos5,exitEvent,wrapStyle)
   ,OP("-Calibrar",callEvent,enterEvent)
   ,OP("-Salvar",saveoffset,enterEvent)
   ,EXIT("<Regresar")
  );

MENU(mainMenu,"Selec. Metales",doNothing,noEvent,wrapStyle
  ,FIELD(test,"Num. Metal","",0,5,1,0,dataSize,exitEvent,noStyle)
  ,altOP(recetaPrompt,"",processing,enterEvent)
  ,OBJ(targetsMenu)//attach the array edit menu on a macri build nenu
  ,SUBMENU(subMenu)
 // ,SUBMENU(subMenu2) 
  ,EXIT("<Regresar")
);

result targetEvent(eventMask e,navNode& nav) {
  trace(MENU_DEBUG_OUT<<"copy data to temp target:"<<(int)nav.target<<"\n");
  if(nav.target==&targetsMenu)//only if we are on targetsMenu
    target=myTargets[nav.sel];
    id2=nav.sel; //id entrada receta
  //you can store nav.sel for future reference
  return proceed;
}


// define menu colors --------------------------------------------------------
//  {{disabled normal,disabled selected},{enabled normal,enabled selected, enabled editing}}
//monochromatic color table

const colorDef<uint16_t> colors[6] MEMMODE={
  {{VGA_BLACK,VGA_BLACK},{VGA_BLACK,VGA_BLUE,VGA_BLUE}},//bgColor
  {{VGA_GRAY,VGA_GRAY},{VGA_WHITE,VGA_WHITE,VGA_RED}},//fgColor
  {{VGA_WHITE,VGA_BLACK},{VGA_YELLOW,VGA_YELLOW,VGA_RED}},//valColor
  {{VGA_WHITE,VGA_BLACK},{VGA_WHITE,VGA_YELLOW,VGA_YELLOW}},//unitColor
  {{VGA_WHITE,VGA_GRAY},{VGA_BLACK,VGA_BLUE,VGA_GRAY}},//cursorColor
  {{VGA_WHITE,VGA_YELLOW},{VGA_WHITE,VGA_BLACK,VGA_RED}},//titleColor
};

//PANELS(serial_panels,{0,0,40,10});//or use default
//serialOut outSerial(Serial);//,serial_panels);//the output device (just the serial port)

#define MAX_DEPTH 4

PANELS(gfx_panels,{0,0,16,12});
//PANELS(gfx_panels,{0,0,12,8},{13,0,12,8});
idx_t gfx_tops[MAX_DEPTH];
utftOut outGfx(tft,colors,gfx_tops,gfx_panels,16,16);//output device, latter set resolution from font measure

idx_t serialTops[MAX_DEPTH]={0};
serialOut outSerial(Serial,serialTops);
MENU_OUTLIST(out,&outGfx,&outSerial);
serialIn serial(Serial);
//extern navRoot nav;
//URTouch  uTouch( 6, 5, 4, 3, 2);
//menuUTouch touchPanel(uTouch,nav,outGfx);
//serialIn serial(Serial);
//
//MENU_INPUTS(in,&touchPanel,&serial);

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);
//
//result alert(menuOut& o,idleEvent e) {
//  if (e==idling) {
//    o.setColor(fgColor);
//    o.setCursor(0,0);
//    o.print("alert test");
//    o.setCursor(0,1);
//    o.print("press [select]");
//    o.setCursor(0,2);
//    o.print("to continue...");
//  }
//  return proceed;
//}
//
//result doAlert(eventMask e, prompt &item) {
//  nav.idleOn(alert);
//  return proceed;
//}

    //when menu is suspended
result idle(menuOut& o,idleEvent e) {
  if (e==idling) {
   drawHomeScreen();
  }
  return proceed;
}

  result datos4() {return proceed;}
  result datos5() {return proceed;}
  result datos6() {return proceed;}
  result processing(){CALIBRAR=0;/*load_data();*/nav.idleOn() ;return proceed;}

    result showEvent(eventMask e) {
     subMenu[1].enabled=disabledStatus;
     subMenu[2].enabled=disabledStatus;
     subMenu[6].enabled=disabledStatus;
     CALIBRAR=0;
     Serial.print("event: ");
     Serial.println(e);
     return proceed;
}



   result datos1() {
    
        String mypass(buf1);
        int password = mypass.toInt();
        if (password == pass) {
            strncpy(buf1,"****",4);
            subMenu[1].enabled=enabledStatus;
            subMenu[2].enabled=enabledStatus;
            subMenu[6].enabled=enabledStatus;
            mainMenu[2].enabled=enabledStatus;        
    }       return proceed;} 




void setup() {
  pinMode(LEDPIN,OUTPUT);
  pinMode(encBtn,INPUT_PULLUP);
  while(!Serial);
  Serial.begin(9600);
  Serial.println("menu 4.x UTFT + URTouch");Serial.flush();

  ads.setGain(GAIN_ONE);
  
  nav.idleTask=idle;//point a function to be used when menu is suspended
//  mainMenu[1].enabled=disabledStatus;
  nav.timeOut = 5;
  //nav.showTitle=false;
  nav.exit();

  tft.InitLCD();
  tft.setBrightness(4);
  tft.clrScr();

    for( number = 0; number < dataSz; number++) {      

         String texto = String(number) +"<vacio>       ";
         texto.toCharArray(char_buff, nameSz+1);
         strncpy(myTargets[number].name,char_buff,nameSz+1);
//       writeIntIntoEEPROM(356+(number*6), 0);EEPROM.commit();
       //  writeStringToEEPROM((number*(nameSz+1))+5,texto); 
//       EEPROM.commit();
         Serial.println(String(number) +"<vacio>       ");

//for( number = 356; number < 489; number++) { 
//      writeIntIntoEEPROM(number, 0);EEPROM.commit();
//  
//}
 
}

   for(idx_update = 0; idx_update < dataSz; idx_update++) { 

         string_buff = readStringFromEEPROM((idx_update*(nameSz+1))+5);
         string_buff.toCharArray(char_buff, nameSz+1);
         strncpy(myTargets[idx_update].name,char_buff,nameSz+1);
}

//  uTouch.InitTouch();
//  uTouch.setPrecision(PREC_MEDIUM);//LOW, MEDIUM, HI, EXTREME

  tft.setFont(BigFont);
  tft.setColor(0, 255, 0);
  tft.setBackColor(0, 0, 0);

  //outGfx.resX=tft.getFontXsize()+1;
  //outGfx.resY=tft.getFontYsize()+1;
  outGfx.println("Menu 4.x on UTFT");
  Timer1.initialize(500);
  Timer1.attachInterrupt(timerIsr);
  //Setup Stepper Driver
  stepper.setMaxSpeed(150000); //thru experimentation I found these values to work... Change for your setup.
  stepper.setAcceleration(20000);
  delay(1000);
  tft.clrScr();
}

void loop() {
  nav.poll();//this device only draws when needed
   if (nav.sleepTask)// If the menu system is not active, draw the main screen
      { 
         //sensorValue = analogRead(sensorPin);
         Input = map(analogRead(PLASMA_INPUT_PIN), 0, 1023, 0, 25000) + CalibrationOffset; //reads plasma arc voltage and convert to millivolt
         process(); //This is the main method of the application it calulates position and move steps if Input Voltage is over threshold.
         report();
      }
//  digitalWrite(LEDPIN, ledCtrl);
//  delay(100);//simulate a delay when other tasks are done
}


void process() //Calulates position and move steps
{
  oldDelay = micros();
  while (Input > (threshold + CalibrationOffset)) //Only move if cutting by checking for voltage above a threshold level
  {
    
    if (micros() - oldDelay >= arcStabilizeDelay) //wait for arc to stabilize tipically 100-300ms
    {
      Input = map(analogRead(PLASMA_INPUT_PIN), 0, 1023, 0, 25000) + CalibrationOffset; //get new plasma arc voltage and convert to millivolts

      currentGap = abs(SetPoint - Input); //distance away from setpoint
      if (currentGap < gap) {
        THCPID.setCoefficients(Kp, Ki, Kd, Hz); //we're close to setpoint, use conservative tuning parameters
      }
      else {
        THCPID.setCoefficients(aggKp, aggKi, aggKd, Hz); //we're far from setpoint, use aggressive tuning parameters
      }

      if (SetPoint > Input)
      {
        targetInput = Input - SetPoint + SetPoint;
        output = THCPID.step(SetPoint, targetInput);
        pos = pos + output;
//        Serial.print("sp>in: ");
//        Serial.println(output);
      }
      else
      {
        targetInput = SetPoint - Input + SetPoint;
        output = THCPID.step(SetPoint, targetInput);
        pos = pos - output;
//        Serial.print("sp<in: ");
//        Serial.println(output);
      }

      //Validate move is within range
      if (pos >= maxPos) {
        pos = maxPos;
      }
      if (pos <= minPos) {
        pos = minPos;
      }
        Serial.print("pos: ");
        Serial.println(pos);
      //do move
      stepper.moveTo(pos);
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }

      report(); //report plasma voltage and position
      //format();
    }
  }
  //after cut reset height
  pos = 0;
  //do move
  stepper.moveTo(pos);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

void report() {
  
    tft.setFont(SevenSegmentFull);
    tft.setColor(255, 0, 0);
    tft.setBackColor(0, 0, 0);
//  tft.printNumI(input,175, 85, 3,'0');
    tft.printNumF(Input/1000,3, 145, 85,'.', 0,' ');

    tft.setFont(DotMatrix_M_Slash);
    tft.setColor(255, 255, 0);
    tft.printNumF(pos/1000,3, 175, 52,'.', 7,' ');

    tft.setFont(BigFont);
}

void drawHomeScreen() {

  tft.setFont(SevenSegmentFull);
  tft.setColor(0, 255, 0);
  tft.setBackColor(0, 0, 0);
  tft.printNumI(SetPoint,175, 140, 3,'0');
   
  tft.setColor(100, 155, 203);
  tft.fillRoundRect (10, 10, 60, 36);
  tft.setColor(255, 255, 255);
  tft.drawRoundRect (10, 10, 60, 36);
  tft.drawRoundRect (10, 10, 60, 36);
  tft.setFont(BigFont);
  tft.setBackColor(100, 155, 203);
  tft.print("<-", 18, 15);
  tft.setBackColor(0, 0, 0);
  tft.setFont(BigFont);
  tft.print("THC Control", CENTER, 15);
  tft.print("Divider:", 10, 101);
  tft.print("Set point:", 10, 156);
  tft.print("Position:", 10, 55);
  tft.setColor(255, 0, 0);
  tft.drawLine(0,45,319,45); 
  tft.setColor(255, 0, 0);
  tft.drawLine(0,82,319,82); 
  tft.setColor(255, 0, 0);
  tft.drawLine(0,137,319,137); 
  tft.setColor(255, 255, 255);
  tft.drawRect(15, 198, 310, 228); 
}


void update_data()
{
  Tiempo = readIntFromEEPROM(356+(id2*2));
  tempInicioCiclo  = readIntFromEEPROM(401+(id2*2));
  temperaturaResMax  = readIntFromEEPROM(446+(id2*2));
}

void save_data()
{
for(int idx2 = 0; idx2 < dataSz; idx2++) {

     if (idx2==id){target1 = String(target.name);
      //EEPROM.write(idx2, buff_time);EEPROM.commit();
        writeStringToEEPROM((idx2*(nameSz+1))+5, target1);
     // Serial.println(idx2);}
     delay(10);// break;
}

}

 for(int dat1 = 0; dat1 < dataSz; dat1++) {
   if (dat1==id2){
   writeIntIntoEEPROM(356+(dat1*2), Tiempo);
   delay(10);}}

   for(int dat2 = 0; dat2 < dataSz; dat2++) {
   if (dat2==id2){
   writeIntIntoEEPROM(401+(dat2*2), tempInicioCiclo);
   delay(10);}}
   
   for(int dat3 = 0; dat3 < dataSz; dat3++) {
   if (dat3==id2){
   writeIntIntoEEPROM(446+(dat3*2), temperaturaResMax);
   delay(10);}}

}

 void writeIntIntoEEPROM(int address, int number)
{ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}
int readIntFromEEPROM(int address)
{
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

void writeStringToEEPROM(int addrOffset, const String &strToWrite)
{
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
}

String readStringFromEEPROM(int addrOffset)
{
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++)
  {
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\ 0'; // !!! NOTE !!! Remove the space between the slash "/" and "0" (I've added a space because otherwise there is a display bug)
  return String(data);
}
