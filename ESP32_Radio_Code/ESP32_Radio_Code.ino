#include <RFM69X.h>
#include <RFM69Xregisters.h>
#include <WiFi.h>
#include <RFM69X.h>

#define FREQUENCY RF69_915MHZ
#define NODEID      99
#define NETWORKID   100
#define REMOTEID   1
#define ENCRYPTKEY   "paragliderGroup4" //has to be same 16 characters/bytes on all nodes, not more not less!
#define SERIAL_BAUD 9600
#define LED         5          // LED is on pin 2 for ESP-32          
#define RFM_SS      SS         // Default Slave Select PIN 5 for LOLIN 32
#define RFM_INT     A16        // One if a free GPIO supporting Interrupts

boolean IS_RFM69HW = true;
byte RFM_INTNUM = digitalPinToInterrupt(RFM_INT);
int TRANSMITPERIOD = 200;        // Transmit a packet to gateway so often (in ms)
byte sendSize=0;
long int frameCnt = 0;  // frame Counter
long int ackSentCnt = 0;  // ACK Counter

RFM69X radio(RFM_SS,RFM_INT,IS_RFM69HW,RFM_INTNUM);

/* STRUCTURED DATA TYPE METHOD FOR ESP DATA TYPE ALIGNEMENT */
struct __attribute__((__packed__)) Payload {                // Radio packet structure max 61 bytes or 57 id SessionKey is used
  long int      nodeId; //store this nodeId
  long int      frameCnt;// frame counter
  unsigned long uptime; //uptime in ms
  float         temp;   //temperature maybe??
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
unsigned long SESSION_RSP_DLY = 400;

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
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
  Serial.print ("\nBoard Type: ");
  Serial.println ("ESP32");
  Serial.println ("Using RFM69 Extended mode");
  char buff[50];
  sprintf(buff, "\nTransmitting at 915 Mhz...");
  Serial.println(buff);
}

long lastPeriod = -1;
void loop() {
  //check for any received packets
  if (radio.receiveDone())
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");

    if (radio.ACKRequested())
    {
      ackSentCnt++;
      radio.sendACK();
      Serial.print(" - ACK sent= "); 
      Serial.println (ackSentCnt);     
      delay(10);
    }
    Serial.println();
  }
  
  int currPeriod = millis()/TRANSMITPERIOD;
  if (currPeriod != lastPeriod)
  {
    //fill in the struct with new values
    theData.nodeId = NODEID;
    theData.frameCnt = ++frameCnt;
    theData.uptime = millis();
    theData.temp = 91.23; //it's hot!
    
    Serial.print("Sending struct (");
    Serial.print(sizeof(theData));
    Serial.print(" bytes) ");
    Serial.print(" frame Count= ");
    Serial.print(theData.frameCnt);
    Serial.print(" uptime= ");
    Serial.print(theData.uptime);
    Serial.print(" .... ");
    if (radio.sendWithRetry(REMOTEID, (const void*)(&theData), sizeof(theData),SEND_RTRY,SEND_WAIT_WDG))
      Serial.print(" ok!");
    else Serial.print(" nothing...");
      Serial.println();
    lastPeriod=currPeriod;
  }
}
