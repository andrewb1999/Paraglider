/* This script allows to send  a Structured data transfer from a ESP8266 (ESP-12) / ESP32 or Moteino / Moteino MEGA / Arduino UNO
 *  to a ESP8266 (ESP-12) / ESP32 or a Moteino / Moteino MEGA / Arduino UNO
 *  Each 3 data packet a acknowledgement to a "ping" of the receiver mode may be expected
 *  Selection of different libraries is possible using the different #define parameters(see Options)
 *  In RFM Session mode the session option may be activated or not SESSION_KEY parameters
 * RFM69 High or Low power may also be selected   
 * RFM69 transceiver frequency may also be configured  
 *      
 */

/*** OPTIONS ***/
//#define RFM_SESSION         // If Session Key is used; to be removed for a normal RFM69 instance
                            // This parameter validates the RFM69_SessionKey  or RFM69X_SessionKey library according to the "X" parameter
#define X                   // This parameter validates one the libraries the basic one RFM69 or eXtended oneRFM69X

boolean IS_RFM69HW = true; // If High power RFM69HW is used
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
/***        ***/

#ifdef ESP32                // Allows automatic ESP32 processor recognition
 #include <WiFi.h>          // Required for ESP32
 #include <driver/adc.h>
 #include <pthread.h>
 #define X                  // eXtended Library Selector mandatory for ESP32
#endif

#ifdef ESP8266              // Allows automatic ESP32 processor recognition
  #include <ESP8266WiFi.h>  // Required for ESP8266
#endif  

#ifdef RFM_SESSION
  #ifdef X
    #include <RFM69X_SessionKey.h> // Enable session key support extension for RFM69 extended library
  #else
    #include <RFM69_SessionKey.h> // Enable session key support extension for RFM69 base library
  #endif 
#endif  
#ifdef X
  #include <RFM69X.h> //  eXteneded RFM69 base library  
#else
  #include <RFM69.h> //  RFM69 base library
#endif 

#define NODEID      1
#define NETWORKID   100
#define REMOTEID   2
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY   "ParagliderGroup4" //has to be same 16 characters/bytes on all nodes, not more not less!
#define SERIAL_BAUD 9600      // Serial connection speed

// Adapt configuration against the processor Type
#ifdef __AVR_ATmega1284P__
  #define LED         15        // Moteino MEGA have LED on D15
  #define RFM_SS      4         // Default Slave Select Moteino MEGA 
  #define RFM_INT     2         // One if a free GPIO supporting Interrupts
#endif
#ifdef __AVR_ATmega328P__
  #define LED         9         // Moteino has LEDs on D9
  #define RFM_SS      10        // Default Slave Select for Arduino UNO or Moteino
  #define RFM_INT     2         // One if a free GPIO supporting Interrupts
#endif
#ifdef ESP8266
  #define LED         2          // LED is on pin 2 for ESP-12          
//  #define RFM_SS      15         // Default GPIO15 for ESP8266 SS
//  #define RFM_INT     4          // One if a free GPIO supporting Interrupts
  #define RFM_SS       16         // Default GPIO15 changed to GPIO16 for ESP8266 SS
  #define RFM_INT  15         // One if a free GPIO supporting Interrupts (GPIO15)
#endif  
#ifdef ESP32
  #define LED         5          // LED is on pin 2 for ESP-32          
  #define RFM_SS      SS         // Default Slave Select PIN 5 for LOLIN 32
  #define RFM_INT     A16        // One if a free GPIO supporting Interrupts
#endif 
#ifdef __AVR_ATmega2560__
  #define LED         9          // LED is on pin 9 for ATMEGA2560          
  #define RFM_SS      53         // Default Slave Select 53 for ATMEG2560
  #define RFM_INT     19         // One if a free GPIO supporting Interrupts
#endif 

#ifdef __AVR_ATmega1284P__
byte RFM_INTNUM = 2; // digitalPinToInterrupt doesn't work for ATmeg1284p
#else
byte RFM_INTNUM = digitalPinToInterrupt(RFM_INT);  // Standard way to convert Interrupt pin in interrupt number  
#endif

int TRANSMITPERIOD = 200;        // Transmit a packet to gateway so often (in ms)
byte sendSize=0;
long int frameCnt = 0;  // frame Counter
long int ackSentCnt = 0;  // ACK Counter

#ifdef RFM_SESSION
  #ifdef X
       RFM69X_SessionKey radio(RFM_SS,RFM_INT,IS_RFM69HW,RFM_INTNUM); // Create Extended RFM69 Session Key radio instance
  #else
       RFM69_SessionKey radio(RFM_SS,RFM_INT,IS_RFM69HW,RFM_INTNUM);  // Create RFM69 Session Key radio instance
  #endif 
#else
  #ifdef X
       RFM69X radio(RFM_SS,RFM_INT,IS_RFM69HW,RFM_INTNUM);            // Create Extended RFM69radio instance
  #else
       RFM69 radio(RFM_SS,RFM_INT,IS_RFM69HW,RFM_INTNUM);             // Create RFM69 radio instance
  #endif
#endif  

/* STRUCTURED DATA TYPE METHOD FOR ESP DATA TYPE ALIGNEMENT */
struct __attribute__((__packed__)) Payload {                // Radio packet structure max 61 bytes or 57 id SessionKey is used
  float         yaw;   //Yaw Joystick
  float         pitch;   //Pitch Joystick
};
Payload theData;

// RFM69 parameters
bool SEND_PROMISCUOUS = false;       // Set to 'true' to sniff all packets on the same network
byte SEND_RTRY = 0;                  // Adjust the attempts of a Send With Reply is to be performed in case of timeout
unsigned long SEND_WAIT_WDG = 40;    // Adjust the Send With Reply wait time for which a acknowledge is expected (default is 40ms)
// RFM69 Session Key parameters
bool SESSION_KEY = true;             // Set usage of session mode (or not)
bool SESSION_3ACKS = false;          // Set 3 acks at the end of a session transfer (or not)
unsigned long SESSION_WAIT_WDG = 10;  // Adjust wait time of data reception in session mode is expected (default is 40ms)
#ifdef X
unsigned long SESSION_RSP_DLY = 400; // Adjust the delay time between a Session Request and a Session reply for slow node should be set to at least 400 while using RFM69X_Session 
#else
unsigned long SESSION_RSP_DLY = 200; // Adjust the delay time between a Session Request and a Session reply for slow node (default is 0us, maximum is 500us)  
#endif

void* __adc_manager(void *ptr) {
  while(true) {
    int x, y;
    x = adc1_get_raw(ADC1_CHANNEL_0);
    theData.yaw = (x+1.0)/(4096.0);
    y = adc1_get_raw(ADC1_CHANNEL_3);
    theData.pitch = (y+1.0)/(4096.0);
    delay(1);  
  }
}


void setup() {
  pthread_t adc_thread = 0;
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  delay(100);
  digitalWrite(9, LOW);
  delay(100);
  /*
  pinMode(LED, OUTPUT);     
#ifdef ESP8266
  digitalWrite(LED,HIGH);       // Inverted Logic for ESP8266
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) 
   digitalWrite(LED,LOW);
#endif 
*/

  Serial.begin(SERIAL_BAUD);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
  delay(10);
  if(!radio.initialize(FREQUENCY,NODEID,NETWORKID))
  {
    Serial.println ("\n****************************************************************");
    Serial.println (" WARNING: RFM Transceiver intialization failure: Set-up Halted  ");
    Serial.println ("****************************************************************"); 
    while (1); // Halt the process
  }
 
  if (IS_RFM69HW)  radio.setHighPower();      // Only for RFM69HW!
  radio.encrypt(ENCRYPTKEY);                  // Set encryption
  radio.promiscuous(SEND_PROMISCUOUS);        // Set promiscuous mode
#ifdef RFM_SESSION  
  radio.useSessionKey(SESSION_KEY);           // Set session mode 
  radio.sessionWaitTime(SESSION_WAIT_WDG);    // Set the Session Wait Watchdog 
  radio.useSession3Acks(SESSION_3ACKS);       // Set the Session 3 Acks option
  radio.sessionRespDelayTime(SESSION_RSP_DLY);// Set the slow node delay timer 
#endif  
  Serial.print ("\nBoard Type: "); 
#ifdef __AVR_ATmega1284P__
  Serial.println ("__AVR_ATmega1284P__");
#endif
#ifdef __AVR_ATmega328P__
  Serial.println ("__AVR_ATmega328P__");
#endif
#ifdef ESP8266
  Serial.println ("ESP8266");
#endif  
#ifdef ESP32
  Serial.println ("ESP32");
#endif 
#ifdef __AVR_ATmega2560__
  Serial.println ("__AVR_ATmega2560__");
#endif

#ifdef RFM_SESSION
  #ifdef X
        Serial.println ("Using RFM69X Extended Session mode");
  #else
        Serial.println ("Using RFM69 Session mode");
  #endif
#else    
  #ifdef X
        Serial.println ("Using RFM69 Extended mode");
  #else
        Serial.println ("Using RFM69 Basic mode");
  #endif  
#endif  
  
  
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  if(pthread_create(&adc_thread, NULL, __adc_manager, 0)) {
    Serial.println("Cannot start adc thread");  
  }
}

long lastPeriod = -1;
void loop() {
  //check for any received packets
  if (radio.receiveDone())
  {
    //Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    //for (byte i = 0; i < radio.DATALEN; i++)
      //Serial.print((char)radio.DATA[i]);
    //Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");

  /*
    if (radio.ACKRequested())
    {
      ackSentCnt++;
      radio.sendACK();
      Serial.print(" - ACK sent= "); 
      Serial.println (ackSentCnt);     
      delay(10);
    }
    */
    //Blink(LED,5);
    //Serial.println();
  }
  
  int currPeriod = millis()/TRANSMITPERIOD;
  if (currPeriod != lastPeriod)
  {
    //fill in the struct with new values
    //theData.yaw = 0.44;
    //theData.pitch = 0.44; //it's hot!
    
    //Serial.print("Sending struct (");
    //Serial.print(sizeof(theData));
    //Serial.print(" bytes) ");
    radio.send(REMOTEID, (const void*)(&theData), sizeof(theData));
    //Serial.print(" yaw=");
    //Serial.print(theData.yaw);
    //Serial.print(" pitch=");
    //Serial.print(theData.pitch);
    /*if ()
      Serial.print(" ok!");
    else Serial.print(" nothing...");
    Serial.println();
    */
   //Blink(LED,3);
    lastPeriod=currPeriod;
  }
}
void Blink(byte PIN, int DELAY_MS)
{
#ifdef ESP8266
  digitalWrite(PIN,LOW);
  delay(DELAY_MS);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) 
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS);  
#endif 
}
