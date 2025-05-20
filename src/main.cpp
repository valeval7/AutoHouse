#include <Arduino.h>
//#define SCREEN 
#ifdef SCREEN
  //The One with a Screen, GSM, LED Strip and IrMovement Detector
  #include <SPI.h>
  #include <TFT_eSPI.h>
  #include <HardwareSerial.h>
  #include <Adafruit_NeoPixel.h>
  #include <ESP32Servo.h>
  #define RX 26
  #define TX 27
  #define NEOPIN 16
  #define MOVEMENTSENSOR 39
  #define DOORSERVOPIN 13
  #define TOUCHDOORSWITCH 14
  #define THRESHOLD 30
  /*Screen Definitions*/
  #define SCREEN_WIDTH 320 // 320/(4)=80 -> S{0,80,160,240}, F(x)=x+5, F(S)={5,85,165,245}, W=75
  #define SCREEN_HEIGHT 240 // 240/(3)=80 
  #define FONT_SIZE 2
  #define FONT 4
#else
//#define GARAGE
  #ifdef GARAGE
    //NEMA, Ultrasonico, Beeper
    #include <AccelStepper.h>
    #include <Adafruit_NeoPixel.h>
    #define TRIGGERPIN 33
    #define ECHOPIN 25
    #define STEPPIN 27
    #define DIRPIN 15
    #define ENABLEMOTORPIN 26
    #define BUZZPIN 14
    #define IRPIN 36 //VP
    #define NEOPIXELPIN 32
  #else
    #define VENTILATOR
    #ifdef VENTILATOR
      //Hbridge, LED, TEmp, Gyro, Tachometer, Fan, SErvoCuna
    #include <Wire.h>
    #include <Adafruit_MPU6050.h>
    #include <Adafruit_Sensor.h>
    #include <Adafruit_BME280.h>
    #include <Adafruit_NeoPixel.h>
    #include <ESP32Servo.h>
    #define SCL 22
    #define SDA 21
    #define SERVOCUNAPIN 19
    #define TACHOPIN 18
    #define MOSFETPIN 32
    #define PWMPIN 5
    #define NEOPIXELPIN 23
    //Infrared Existence Sensor, Photoresistor, SErvoPersianas

    #define LIGHTPIN 36 //VN
    #define SERVOPERSIANASPIN 13
    #endif
  #endif
#endif

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

/*Garage a0:b7:65:2c:b6:6c */
/*Screen AC:15:18:E6:62:04 */
/*Roof 88:13:bf:68:b3:d4*/

#if defined SCREEN
typedef struct gateStruct {
  bool Open;
} gateStruct;

typedef struct fasnStruct {
  double Frequency;
  double Temperature;
} fasnStruct;

enum ScreenMode{
  NULLMODE=-1,
PASSWORDMODE,
MONITORMODE,
MANUALCONTROLMODE
};

TFT_eSPI tft = TFT_eSPI();
Adafruit_NeoPixel ws2812;

HardwareSerial GSMSerial(1);
Servo SeverDoor;


int Coordinates[12][4]={
  {5,5,75,75},
  {5,85,75,75},
  {5,165,75,75},

  {85,5,75,75},
  {85,85,75,75},
  {85,165,75,75},

  {165,5,75,75},
  {165,85,75,75},
  {165,165,75,75},
  
  {245,5,75,75},
  {245,85,75,75},
  {245,165,75,75}
};

uint8_t broadcastAddress[] = //{0x8C, 0x4F, 0x00, 0x28, 0x92, 0x64};//8C:4F:00:28:92:64 Microusb
{0x88, 0x13, 0xBF, 0x68, 0xB3, 0xD4};



gateStruct gateData{true};
fasnStruct FanData{0.0};

esp_now_peer_info_t peerInfo;

uint16_t xTouch, yTouch;
uint16_t PreviousXTouch, PreviousYTouch;
bool TouchNow, TouchPrevious, touchedBefore, GarageDoor, DetectedMovement;


String PasswordInsert="";

int PreviousTime;
enum ScreenMode mode;
void updateSerial();

bool inBoundingBox(int ah, int aw,int bh,int bw, int x, int y);

void PassWordScreen();
void MonitorScreen();
void ManualScreen();

void PassWordSetup();
void MonitorSetup();
void ManualSetup();

void fillPasswordMenu();
void fillControlMenu();
void fillMonitorMenu();

void OpenDoor();
void CloseDoor();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void MovementRiseInterrupt();

void poll();
void setup(){
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);
  
  /*GSMSerial.begin(9600, SERIAL_8N1, RX, TX);
  Serial.println("Initializing...");
  delay(1000);
  GSMSerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  GSMSerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  GSMSerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  GSMSerial.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();*/
  Serial.println("Trying to Start ESP-NOW");
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    delay(500);
    return;
  }
  
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  Serial.println("Registered Peer");
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
  Serial.println("Added Peer");
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Serial.println("Setup callbacks");

  tft.init();
  tft.setRotation(3);
  Serial.println("Initialized Screen");
  tft.fillScreen(0xF800);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  Serial.println("Filled With Red");
  ws2812=Adafruit_NeoPixel(8, NEOPIN, NEO_GRB + NEO_KHZ800);
  ws2812.begin();
  SeverDoor.attach(DOORSERVOPIN);
  pinMode(MOVEMENTSENSOR,INPUT);
  pinMode(TOUCHDOORSWITCH,INPUT);
  Serial.println("Input Modes");
  attachInterrupt(MOVEMENTSENSOR,MovementRiseInterrupt,RISING);
  Serial.println("Attached Rise Interrupt");
  mode=NULLMODE;
  GarageDoor=false;
}

void loop(){
  poll();
  if ((millis() - PreviousTime) > 2000){
    if(DetectedMovement){
      ws2812.fill(0xFFFF);
      DetectedMovement=false;
      Serial.println("Movement");
    }else{
      ws2812.fill(0x0);
      Serial.println("No Movement");
    }
      
    ws2812.show();
    PreviousTime=millis();
    Serial.println("Called Show");
  }
  switch(mode){
    case PASSWORDMODE:
      PassWordScreen();
    break;
    case MONITORMODE:
      MonitorScreen();
    break;
    case MANUALCONTROLMODE:
      ManualScreen();
    break;
    default:
    Serial.println("Default Transition");
    PassWordSetup();
    break;
  }
}
void PassWordScreen(){

  if(tft.getTouch(&xTouch,&yTouch)){

    int ii;
    for(ii=0;ii<12;ii++){
      if(inBoundingBox(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],xTouch,yTouch)){
        if(!touchedBefore){//NOT the same Collision as Previous Collision
          //Then We Do the Stuff tied to ii
          switch(ii){
            case 9:
            //Try the Password
              if(PasswordInsert.equals("159483726")){
                OpenDoor();
                Serial.println("Opened Door");
                MonitorSetup();
              }else{
                tft.fillScreen(0x1800);
                delay(100);
                tft.fillScreen(0xF800);
                delay(100);
                tft.fillScreen(0xFFFF);
                delay(100);
                tft.fillScreen(0xF800);
                delay(100);
                tft.fillScreen(0xFFFF);
                delay(100);
                tft.fillScreen(0xF800);
                PassWordSetup();
              }
              Serial.println("Tried Password");
            break;
            case 10:
            //Try the Delete
            if(PasswordInsert.length()!=0){
              PasswordInsert=PasswordInsert.substring(0,PasswordInsert.length()-1);
            }else{
              PasswordInsert.clear();
            }
              Serial.println("Removed Digit");
            break;
            case 11:
            //Try the Reset
              
              PasswordInsert.clear();
              Serial.println("Cleared Password");
            break;
            default:
            PasswordInsert.concat(ii+1);
            Serial.print("Number: ");
            Serial.println(ii+1);
          }

        }
        PreviousXTouch=xTouch;
        PreviousYTouch=yTouch;
        break;//end since we found the Correct Button Provided
      }
    }
    touchedBefore=true;
  }else{
    touchedBefore=false;
  }
  
}

void MonitorScreen(){
  //TODO: Add a Subroutine to Watch fan Speed/ON STATUS and Temperature Data
if(tft.getTouch(&xTouch,&yTouch)){
    int ii;
    for(ii=0;ii<12;ii++){
      if(inBoundingBox(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],xTouch,yTouch)){//Find Quadrant Collided
        if(!touchedBefore){//NOT the same Collision as Previous Collision
          //Then We Do the Stuff tied to ii
          switch(ii){
            case 9:
            //Try the Other Mode
            ManualSetup();         
            break;
            case 11:
            //Try the Locking Routine
              
            PassWordSetup();
            break;
            default:
            //No Operation;
            break;
          }

        }
        PreviousXTouch=xTouch;
        PreviousYTouch=yTouch;
        break;//end since we found the Correct Button Provided
      }
    }
    touchedBefore=true;
  }else{
    touchedBefore=false;
  }
}

void ManualScreen(){

  //TODO: Add a Subroutine to Control fan Speed/ON STATUS and Door controls
if(tft.getTouch(&xTouch,&yTouch)){
    int ii;
    for(ii=0;ii<12;ii++){
      if(inBoundingBox(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],xTouch,yTouch)){//Find Quadrant Collided
        if(!touchedBefore){//NOT the same Collision as Previous Collision
          //Then We Do the Stuff tied to ii
          switch(ii){
            case 0:
            //Turn Fan on/off
            
            break;
            case 1:
            //Map Higher Speed
            
            break;
            case 2:
            //Map Lower Speed
            
            break;
            case 9:
            //Try the Other Mode
            MonitorSetup();         
            break;
            case 11:
            case 10:
            case 8:

            //Try the Locking Routine
            PassWordSetup();
            break;
            default:
            //No Operation;
            break;
          }

        }
        PreviousXTouch=xTouch;
        PreviousYTouch=yTouch;
        break;//end since we found the Correct Button Provided
      }
    }
    touchedBefore=true;
  }else{
    touchedBefore=false;
  }
}

void PassWordSetup(){
  mode=PASSWORDMODE;
  Serial.println("Set Pass Mode");
  fillPasswordMenu();
  Serial.println("Filled Password Buttons");
  PasswordInsert.clear();
  Serial.println("Cleared Password");
  CloseDoor();
  Serial.println("Sent Close Door Signals");
}

void MonitorSetup(){
  tft.fillScreen(0xF800);
mode=MONITORMODE;
  fillMonitorMenu();
}
void ManualSetup(){
  tft.fillScreen(0xF800);
  mode=MANUALCONTROLMODE;
  fillControlMenu();
}


void OpenDoor(){
  if(GarageDoor){
    gateData.Open=true;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &gateData, sizeof(gateData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  }else{
    SeverDoor.write(10);
  }
}
void CloseDoor(){
    gateData.Open=false;
    
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &gateData, sizeof(gateData));
    
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    } 
    SeverDoor.write(170);
}
void poll(){
  TouchPrevious=TouchNow;
  TouchNow=touchRead(TOUCHDOORSWITCH)<THRESHOLD;
  if(TouchNow&&(TouchPrevious^TouchNow)){
    GarageDoor=!GarageDoor;
  }
}

void IRAM_ATTR MovementRiseInterrupt(){
  DetectedMovement=true;
}

void fillMonitorMenu(){
  int ii;
  ii=9;
  tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0x8811);
  
  
  ii=11;
  tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0x8811);
  tft.drawCentreString("LOCK",  (Coordinates[ii][0]+Coordinates[ii][2]/2), map((Coordinates[ii][1]+Coordinates[ii][3]/2),0,240,240,0), FONT);  
  ii=9;
  tft.drawCentreString("CONTROL",  (Coordinates[ii][0]+Coordinates[ii][2]/2), map((Coordinates[ii][1]+Coordinates[ii][3]/2),0,240,240,0), FONT);
}

void fillControlMenu(){
  int ii;
  ii=0;
  tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0x8800);
  
  ii=1;
  tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0xfdd7);
  
  ii=2;
  tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0xbff7);
  

  ii=9;
  tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0x8811);
  
  
  ii=11;
  tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0x8811);
  tft.drawCentreString("LOCK",  (Coordinates[ii][0]+Coordinates[ii][2]/2), map((Coordinates[ii][1]+Coordinates[ii][3]/2),0,240,240,0), FONT);  
  ii=0;
  tft.drawCentreString("ON/OFF",  (Coordinates[ii][0]+Coordinates[ii][2]/2), map((Coordinates[ii][1]+Coordinates[ii][3]/2),0,240,240,0), FONT);
  ii=1;
  tft.drawCentreString("LOWERSPEED",  (Coordinates[ii][0]+Coordinates[ii][2]/2), map((Coordinates[ii][1]+Coordinates[ii][3]/2),0,240,240,0), FONT);
  ii=2;
  tft.drawCentreString("ADDSPEED",  (Coordinates[ii][0]+Coordinates[ii][2]/2), map((Coordinates[ii][1]+Coordinates[ii][3]/2),0,240,240,0), FONT);
  ii=9;
  tft.drawCentreString("MONITOR",  (Coordinates[ii][0]+Coordinates[ii][2]/2), map((Coordinates[ii][1]+Coordinates[ii][3]/2),0,240,240,0), FONT);

}

void fillPasswordMenu(){
  Serial.println("Called Password Filler");
  for(int ii=0;ii<12;ii++){
    Serial.print("Drawing the ");
    Serial.print(ii);
    Serial.println(" Button");
    tft.fillRect(Coordinates[ii][0],Coordinates[ii][1],Coordinates[ii][2],Coordinates[ii][3],0xFFFF);
  }
  for(int ii=0;ii<12;ii++){
  if(ii<9){
      tft.drawNumber(ii+1,  (Coordinates[ii][0]+Coordinates[ii][2]/2), map((Coordinates[ii][1]+Coordinates[ii][3]/2),0,240,240,0), FONT);
    }else{
      switch(ii){
        case 9:
          tft.drawCentreString("Enter",  (Coordinates[ii][0]+Coordinates[ii][2]/2), map((Coordinates[ii][1]+Coordinates[ii][3]/2),0,240,240,0), FONT);
        break;
        case 10:
          tft.drawCentreString("Delete",  (Coordinates[ii][0]+Coordinates[ii][2]/2), map((Coordinates[ii][1]+Coordinates[ii][3]/2),0,240,240,0), FONT);
        break;
        case 11:
          tft.drawCentreString("Reset",  (Coordinates[ii][0]+Coordinates[ii][2]/2), map((Coordinates[ii][1]+Coordinates[ii][3]/2),0,240,240,0), FONT);  
        break;
        default:
        break;
      }
    }
  }
  Serial.println("Finished Internal Call");
}
bool inBoundingBox(int ah, int aw,int bh,int bw, int x, int y){
  return (ah<=x&&aw<=y&&(ah+bh)>=x&&(aw+bw)>=y);
}

void updateSerial(){
  while (Serial.available()){
    GSMSerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(GSMSerial.available()){
  Serial.write(GSMSerial.read());//Forward what Software Serial received to Serial Port
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&FanData, incomingData, sizeof(FanData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Float: ");
  Serial.println(FanData.Frequency);
  Serial.print("Float: ");
  Serial.println(FanData.Temperature);
}

#elif defined GARAGE

  typedef struct gateStruct {
    bool Open;
  } gateStruct;

  gateStruct GateData;
  AccelStepper StepperMotor(1,STEPPIN,DIRPIN);
  Adafruit_NeoPixel ws2812;

  int CurrDistance;
  int Timeout;
  int PreviousTime;
  bool BuzzState;
  double triggerRead();
  void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len);
  
  void IRRise();
  void IRFall();

  void setup(){
    Serial.begin(115200); // Starts the serial communication
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    
    // Once ESPNow is successfully Init, we will register for recv CallBack to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecieved));
    
    pinMode(TRIGGERPIN, OUTPUT); // Sets the trigPin as an Output
    pinMode(ECHOPIN, INPUT); // Sets the echoPin as an Input
    ws2812=Adafruit_NeoPixel(8, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);
    ws2812.begin();


    pinMode(BUZZPIN, OUTPUT); // Sets the trigPin as an Output
    StepperMotor.setAcceleration(100);
    StepperMotor.setMaxSpeed(1000);
    BuzzState=true;
    pinMode(IRPIN, INPUT); // Sets the echoPin as an Input
    attachInterrupt(IRPIN,IRRise,RISING);
    attachInterrupt(IRPIN,IRFall,FALLING);
  }
  void loop(){
    CurrDistance=triggerRead();
    triggerRead();
    Serial.print("Distance (CM): ");
    Serial.println(CurrDistance);
    if(CurrDistance<20&&CurrDistance>2){
      if ((millis() - PreviousTime) > 200){
        
        digitalWrite(BUZZPIN, BuzzState);
        Serial.print("Buzz:");
        Serial.println(BuzzState);
        
        BuzzState= !BuzzState;
        PreviousTime=millis();
      }
    }else {
      digitalWrite(BUZZPIN, LOW);
    }
    if(CurrDistance<15){
      StepperMotor.moveTo(0);
    }
    StepperMotor.run();
  }
  double triggerRead(){
    // Clears the trigPin
    digitalWrite(TRIGGERPIN, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(TRIGGERPIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGERPIN, LOW);
    // Reads the ECHOPIN, returns the sound wave travel time in microseconds
    // Calculating the distance
    return pulseIn(ECHOPIN, HIGH) * 0.034/ 2;//Divide the Echo due to being There and Back Distance(Ignores Dopple Effect)
    
    // Prints the distance on the Serial Monitor
  }

  
  void IRAM_ATTR IRRise(){
    ws2812.fill(0x0000);
    ws2812.show();
  }

  void IRAM_ATTR IRFall(){
    ws2812.fill(0xFFFF);
    ws2812.show();
  }
  void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&GateData, incomingData, sizeof(GateData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("x: ");
    Serial.println(GateData.Open);
    StepperMotor.moveTo(GateData.Open?100:0);
  }

#elif defined VENTILATOR
  uint8_t broadcastAddress[] = //{0x8C, 0x4F, 0x00, 0x28, 0x92, 0x64};//8C:4F:00:28:92:64 Microusb
  {0x88, 0x13, 0xBF, 0x68, 0xB3, 0xD4};
  esp_now_peer_info_t peerInfo;
  
  void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len);

  typedef struct fasnStruct {
    double Frequency;
    double Temperature;
  } fasnStruct;

  fasnStruct FanData;
  double Frequency;
  Servo SeverCuna,SeverCortina;
  Adafruit_BME280 bme;
  Adafruit_MPU6050 mpu;
  bool i2cGyro, i2ctemper, InterruptFlag;
  int PreviousTime;
  Adafruit_NeoPixel ws2812;
  volatile int Count;

int anguloActual = 140;
int incremento = 5;
const int minAngulo = 130;
const int maxAngulo = 150;
unsigned long intervaloMovimiento = 20;
unsigned long tiempoAnterior = 0;

float objetivoX = 9.0;
float objetivoY = 0.2;
float objetivoZ = 3.3;
float tolerancia = 0.2;

  void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len);
  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
  /*Interrupts*/
  void CountTacho();
  void LightFalls();
  void LightRise();
  void IRRise();
  void IRFall();
  /*Logic Code */
  void setup(){
  Serial.begin(115200); // Starts the serial communication
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    
    
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecieved));

    Wire.begin();
    ws2812=Adafruit_NeoPixel(8, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);
    ws2812.begin();
    SeverCuna.attach(SERVOCUNAPIN);
    SeverCortina.attach(SERVOPERSIANASPIN);
    i2cGyro=mpu.begin();
    if (!i2cGyro) {
      Serial.println("Sensor init failed");
    }else{
    Serial.println("Found a MPU-6050 sensor");
    }
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    i2ctemper=bme.begin(0x76);
    if (!i2ctemper) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }else{
      Serial.println("Found BME280");
    }
    pinMode(MOSFETPIN,OUTPUT);
    pinMode(LIGHTPIN,INPUT);
    
    pinMode(TACHOPIN, INPUT_PULLUP);
    
    attachInterrupt(LIGHTPIN,LightRise,RISING);
    //attachInterrupt(LIGHTPIN,LightFalls,FALLING);

  }

  void loop(){
    if(i2ctemper){
        //int temperature=bme.readTemperature();
      digitalWrite(MOSFETPIN,bme.readTemperature()>30);//Change Depending on High-Low Activation, or P-N Gate  
      FanData.Temperature=bme.readTemperature();
      Serial.println("Temperature Data");
      Serial.println(bme.readTemperature());
    }else{
      digitalWrite(MOSFETPIN,HIGH);
    }

    if(i2cGyro){
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        int y=map(a.acceleration.y,0,10,0,70);
        int x=map(a.acceleration.x,0,10,0,70);
        int z=map(a.acceleration.z,0,10,0,70);
        bool cercaX = abs(a.acceleration.x - objetivoX) <= tolerancia;
        bool cercaY = abs(a.acceleration.y - objetivoY) <= tolerancia;
        bool cercaZ = abs(a.acceleration.z - objetivoZ) <= tolerancia;
        
        if (cercaX && cercaY && cercaZ) {
          if (millis() - tiempoAnterior >= intervaloMovimiento) {
            anguloActual += incremento;
            if (anguloActual <= minAngulo || anguloActual >= maxAngulo) {
              incremento *= -1;
            }
            SeverCuna.write(anguloActual);
            tiempoAnterior = millis();
          }
          Serial.print("Movimiento automático. Ángulo: ");
          Serial.println(anguloActual);
        } else {
          Serial.println("Fuera del rango, servo quieto.");
        }
        SeverCuna.write(10+x+y+z);
      }else{
        SeverCuna.write(90);
      }
    
    if ((millis() - PreviousTime) > 2000){
      if(InterruptFlag){
        SeverCortina.write(170);
        ws2812.fill(0x000000);
        ws2812.show();
      }else{
        SeverCortina.write(90);
        ws2812.fill(0xFFFFFF);
        ws2812.show();
      }
      int StartTime=millis();
      //attachInterrupt(TACHOPIN,CountTacho,RISING);//use a PullUp to 3.3v or pullup Input
      
      StartTime=millis()-StartTime;
      FanData.Frequency=1000.0/StartTime;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &FanData, sizeof(FanData));
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
      InterruptFlag=false;
      PreviousTime=millis();
    }
  }

  void IRAM_ATTR CountTacho(){
    Count++;
    if(Count>=2){
      detachInterrupt(TACHOPIN);
      Count=-1;
    }
  }

  void IRAM_ATTR LightFalls(){
    InterruptFlag=false;
  }

  void IRAM_ATTR LightRise(){
    InterruptFlag=true;
  }

  void IRAM_ATTR IRRise(){
    SeverCortina.write(170);
    ws2812.fill(0x0000);
    ws2812.show();
  }

  void IRAM_ATTR IRFall(){
    ws2812.fill(0xFFFF);
    ws2812.show();
  }
  void OnDataRecieved(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&FanData, incomingData, sizeof(FanData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("x: ");
    Serial.println(FanData.Frequency);
    //digitalWrite(MOSFETPIN,FanData.Frequency);
  }

  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }

#endif
