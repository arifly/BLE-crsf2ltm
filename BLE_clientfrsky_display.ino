#include <esp_task_wdt.h>
//3 seconds WDT
#define WDT_TIMEOUT 20

/**
 * A BLE client example that is rich in capabilities.
 * There is a lot new capabilities implemented.
 * author unknown
 * updated by chegewara
 */
extern uint16_t telemetry_voltage;
extern uint32_t telemetry_lat;
extern uint32_t telemetry_lon;
extern float telemetry_speed;
extern float telemetry_course;
extern uint16_t telemetry_alt;
extern uint16_t telemetry_sats;
extern int telemetry_failed_cs;
extern uint16_t telemetry_att_pitch;
extern uint16_t telemetry_att_roll;
extern uint16_t telemetry_att_yaw;
extern uint16_t telemetry_current;
extern char* telemetry_fmode;


#include "BLEDevice.h"
//#include "BLEScan.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// The remote service we wish to connect to.
//static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID serviceUUID("0000fff0-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("0000fff6-0000-1000-8000-00805f9b34fb");

//0000fff0-000-1000-8000-00805f9b34fb
//01020304-

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static boolean bluetooth = false;
static boolean wlan = false;

//const int buttonPin = 25;    

//heltec
//#define SCL   4
//#define SDA   15

// wemos
#define SCL   4  
#define SDA   5


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// #define OLED_RESET     16 // Reset pin # (or -1 if sharing Arduino reset pin) -- heltec
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin) -- wemos
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define SERIAL2_TX 14
#define SERIAL2_RX 12
#define SERIAL2_BAUD 38400


//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

const int anzeigeupdate = 2000;
static unsigned long previousMillis = 0;  
int crosscounter = 0;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    //Serial.print("Notify callback for characteristic ");
    //Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    //Serial.print(" of data length ");
    //Serial.println(length);
//    Serial.print("data: ");
//    Serial.println((char*)pData);
    Serial2.write((char*)pData);
    //CRSF_Decode(pData, length);
    esp_task_wdt_reset();
    //Serial.println("csrf_exit");
    int old_y = display.getCursorY();
    display.setCursor(118,57);
    char crosselem[] = "/-\\|/-\\|";
    // for i 
    if ( crosscounter <  strlen(crosselem) ) {
      display.print(crosselem[crosscounter]);
      crosscounter++;
    } else { 
      crosscounter = 0;       
    }
    display.display();
    display.setCursor(0,old_y);
    return;
    
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    esp_task_wdt_reset();
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
    esp_task_wdt_reset();
    doScan = true;
    display.println("- connection lost");
    display.display();
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    //Serial.println(myDevice->getAddress().toString().c_str());
    ////display.print(F("Create Connection "));
    // //display.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println("- Created client");
    // //display.println("- Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println("Connected to server");
    display.println("- Connected to server");
    display.display();
    esp_task_wdt_reset();

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      esp_task_wdt_reset();
      display.println("Failed to find our service UUID");
      display.display();
      return false;
    }
    Serial.println("- Found our service");
    display.println("- Found our service");
    display.display();


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      display.println("Failed to find our characteristic UUID");
      display.display();
      esp_task_wdt_reset();
      return false;
    }
    Serial.println("- Found our characteristic");
    display.println("- Found our characte"); 
    display.println("   ristic");
    display.display();

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
      esp_task_wdt_reset();
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);
      esp_task_wdt_reset();
    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    ////display.print(F("BLE Dev found: "));
    ////display.println(advertisedDevice.toString().c_str());
    ////display.display();

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
      esp_task_wdt_reset();
    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

// -----------------------------------------------------------------

  //void 



void setup() {
  Serial.begin(115200);
  // rx, tx)
  Serial2.begin(SERIAL2_BAUD,SERIAL_8N1,SERIAL2_RX,SERIAL2_TX);
  // Serial2.begin(115200,SERIAL_8N1);
  // SDA, SCL
  //Wire.begin(4, 15);
  //Wire.begin(5, 17);
  Wire.begin(SDA, SCL);
  
   

//  pinMode(16, OUTPUT);
//  digitalWrite(16, HIGH); 
//  digitalWrite(16, LOW); 
  Serial.println("Starting BLE Client");
  //pinMode(buttonPin, INPUT);

  //U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);


  if(!display.begin(SSD1306_SWITCHCAPVCC , SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  } else {
    // Clear the buffer.
    display.clearDisplay();
    delay(1000); // Pause for 2 seconds
    display.setTextSize(1);             // Normal 1:1 pixel scale
    //display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0,0);             // Start at top-left corner
    display.println(F("Start BLE Client..."));
    display.display();
  }

  BLEDevice::init("");

  //------------------------------



  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
  display.println("- Scanning...");
  display.display();

  Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  
} // End of setup.


int i = 0;
int last = millis();

// This is the Arduino main loop function.
void loop() {
  //Serial.print(".");
  // resetting WDT every 2s, 5 times only
  if (millis() - last >= 2000 && i < 5) {
      esp_task_wdt_reset();
      last = millis();
  }
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
      display.println("- connected");
      display.display();
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
      //display.println("We have failed to connect to the server; there is nothin more we will do.");
      //display.display();
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
      
      String newValue = "Time since boot: " + String(millis()/1000);
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
     
      // Set the characteristic's value to be the array of bytes that is actually a string.
      pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());

//      if (millis() - previousMillis >= anzeigeupdate) {
//        previousMillis = millis();
//        display.clearDisplay();
//        //delay(100);
//        display.setCursor(0,0);
//        display.print("Volt: ");
//        display.print((uint16_t) telemetry_voltage);
////        display.print("Speed: ");
////        display.println((float) telemetry_speed);
//        display.print(" Current: ");
//        display.println((uint16_t) telemetry_current);
//        display.print("Lat: ");
//        display.println((uint16_t) telemetry_lat);
//        display.print("Lon: ");
//        display.println((uint16_t) telemetry_lon, 3);
//        display.print("Course: ");
//        display.println((float) telemetry_course);
//        display.print("Alt: ");
//        display.print((uint16_t) telemetry_alt, 0);
//        display.print(" Sats: ");
//        display.print((uint16_t) telemetry_sats);
//        display.print(" Err: ");
//        display.println((int) telemetry_failed_cs);
//        display.print("Pit: ");
//        display.print((uint16_t) telemetry_att_pitch);
//        display.print(" Rol: ");
//        display.print((uint16_t) telemetry_att_roll);        
//        display.print(" Yaw: ");
//        display.println((uint16_t) telemetry_att_yaw);                
//        display.print("FMode: ");
//        display.println((char*) telemetry_fmode);                
//        display.display();        
        
//    }

      
  }else if(doScan){
      Serial.println("Restart Scan");
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Restart Scan..");
      display.println("- Scanning...");
      display.display();
      BLEDevice::getScan()->start(5);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
  //Serial.print(".");
  doScan = true;
  delay(1000); // Delay a second between loops.
//  Serial.println("csrf_d1_e");
//  Serial.print("Volt: ");
//  Serial.println((uint16_t) telemetry_voltage);
//  Serial.print("Lat: ");
//  Serial.println((uint16_t) (telemetry_lat)/1E7, 7);
//  Serial.print("Lon: ");
//  Serial.println((uint16_t) (telemetry_lon)/1E7, 7);
//  Serial.print("Speed: ");
//  Serial.println((float) telemetry_speed);
//  Serial.print("Alt: ");
//  Serial.println((uint16_t) (telemetry_alt)/1E3, 0);
//  Serial.print("Sats: ");
//  Serial.println((uint16_t) telemetry_sats);
//
//  
//  Serial.print("Error: ");
//  Serial.println((int) telemetry_failed_cs);

  
} // End of loop
