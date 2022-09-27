/*
references used:
  russion atmega328p/oled version: https://github.com/fogbox/m365_display/blob/master/extend_speedometer/defines.h & https://electro.club/forum/elektrosamokatyi_xiaomi/displey_dlya_syaokata&page=8
  CamiAlfa BLE Protocoll: https://github.com/CamiAlfa/M365-BLE-PROTOCOL/blob/master/m365_register_map.h
  Paco Gorina NinebotMetrics: http://www.gorina.es/9BMetrics/
*/

//current version working on esp32 & esp8266
#define ESP32

#ifdef ESP32
  #include <WiFi.h>
  #include <WiFiUdp.h>
  #include <Update.h> //needed for arduinoOTA on esp32

//#include <endian.h>
#include <ArduinoOTA.h>

/* Modifications to ADAFRUIT SSD1306 Library 1.02for ESP32:
 *  Adafruit_SSD1306.h:35 - #elif defined(ESP8266) || defined(ESP32) || defined(ARDUINO_STM32_FEATHER)
 *  Adafruit_SSD1306.h: change line "void begin(uint8_t switchvcc = SSD1306_SWITCHCAPVCC, uint8_t i2caddr = SSD1306_I2C_ADDRESS, bool reset=true);"
      to: "void begin(uint8_t switchvcc = SSD1306_SWITCHCAPVCC, uint8_t i2caddr = SSD1306_I2C_ADDRESS, uint8_t sda =12, uint8_t scl=14, bool reset=true);"
 *  Adafruit_SSD1306.h: add new line with "uint8_t _sda, _scl;" before line that contains fastspiwrite
 *  Adafruit_SSD1306.cpp:27 - #if !defined(__ARM_ARCH) && !defined(ENERGIA) && !defined(ESP8266) && !defined(ESP32)
 *  Adafruit_SSD1306.cpp: change "void Adafruit_SSD1306::begin(uint8_t vccstate, uint8_t i2caddr, bool reset) {" to "void Adafruit_SSD1306::begin(uint8_t vccstate, uint8_t i2caddr, uint8_t sda, uint8_t scl, bool reset) {"
 *  Adafruit_SSD1306.cpp: in "void Adafruit_SSD1306::begin..." add "_sda = sda; _scl=scl;" as first line
 *  Adafruit_SSD1306.cpp: in "void Adafruit_SSD1306::begin..." change Wire.begin(); to Wire.begin(_sda,_scl,800000);
 *  nt8_t vccstate, uint8_t i2caddr, uint8_t sda, uint8_t scl, bool reset) add  Wire.setClock(800000); after Wire.begin;
 *  
 *  HACK for DUAL Display - 2 displays on 2 different i2c interfaces with 2 different displaybuffers:
 *  copy Adafruit_SSD1306 library directory and rename both to Adafruit_SSD1306_0 and Adafruit_SSD1306_1
 *  do the same with the .h/.cpp filenames -> append _0/_1
 *  change all appearances of Adafruit_SSD1306 to Adafruit_SSD1306_0 / Adafruit_SSD1306_1 in .h & .cpp
 *  change "Wire." to "MyWire0." / "MyWire1." in .cpp
 *  add TwoWire MyWire0 = TwoWire(0); / TwoWire MyWire1 = TwoWire(1); to .cpp after includes/before
 *  in library.properties file change library name, e.g. append "_0" and "_1"
 *  change the includes in this file (see display section below)
 */

#define swversion "18.07.11"

//functional modules
  //#define useoled1 //comment out to disable oled functionality
  //#define useoled2
  //#define usewlanclientmode //NOT IMPLEMENTED comment out to disable Wifi Client functionality
  //#define usewlanapmode //NOT IMPLEMENTED  comment out to disable Wifi Access Poiint functionality
  //#define usetelnetserver //comment out to disable telnet status/telemetrie server, this also disables RAW Server (so only mqtt might be left for leaving wifi activated)
  //#define userawserver //comment out to disable RAW Serial Data BUS Stream on Port 36524, NOT VERIFIED against wired-data-stream
  //#define usepacketserver //comment out to disable PACKET Decode on Port 36525
  //#define usemqtt //NOT IMPLEMENTED comment out to disable mqtt functionality

//DEBUG Settings
  #define debug_dump_states //dump state machines state
  //#define debug_dump_rawpackets //dump raw packets to Serial/Telnet
  #define debug_dump_packetdecode //dump infos from packet decoder

//M365 - serial
  /* ATTENZIONE - the Arduino Serial Library uses HardwareSerial from the arduino-esp32 core which uses "esp32-hal-uart.cpp" driver from esp-idf which runs on RTOS
  * ESP32 has 128Byte Hardware RX Buffer
  * esp32-hal-uart uses interrupts, but default config toggles the rx-event only after 112 received bytes
  * this leads to the problem -> on m365 we must listen to the packets sent by other nodes (bms/esc/ble modules) and send our stuff with the right timing
  * so long explanation, short solution:
  * in esp32-hal-uart.c, in void uartEnableInterrupt(uart_t* uart) change
  * uart->dev->conf1.rxfifo_full_thrhd = 112;
  * to
  *  uart->dev->conf1.rxfifo_full_thrhd = 1;
  */
  //#define UART2RX GPIO_NUM_5 //TTGO Test board
  //#define UART2TX GPIO_NUM_17 //TTGO Test board
  //#define UART2RXunused GPIO_NUM_23 //TTGO Test board; ESP32 does not support RX or TX only modes - so we remap the rx pin to a unused gpio during sending

  //custom pins see here: http://www.iotsharing.com/2017/05/how-to-use-serial-arduino-esp32-print-debug.html?m=1
  //enable rx/tx only  modes see: https://github.com/esp8266/Arduino/blob/master/cores/esp8266/HardwareSerial.h
  #ifdef ESP32
    #define DebugSerial Serial //Debuguart = default Serial Port/UART0
    HardwareSerial M365Serial(2);  // UART2for M365
    #define UART2RX GPIO_NUM_23 //Wemos board
    #define UART2TX GPIO_NUM_5 //Wemos board
    #define UART2RXunused GPIO_NUM_19 //TTGO Test board; ESP32 does not support RX or TX only modes - so we remap the rx pin to a unused gpio during sending
    #define M365SerialFull M365Serial.begin(115200,SERIAL_8N1, UART2RX, UART2TX);
    //#define Serial1RX M365Serial.begin(115200,SERIAL_8N1, UART2RX, -1)
    #define M365SerialTX M365Serial.begin(115200,SERIAL_8N1, UART2RXunused, UART2TX);


//M365 - serial receiver
  #define maxlen 256 //serial buffer size in bytes
  uint8_t crc1=0;
  uint8_t crc2=0;
  uint16_t crccalc=0;
  uint8_t sbuf[maxlen];
  uint8_t len=0;
  uint8_t readindex=0;
  uint8_t m365receiverstate = 0;
  uint8_t m365receiverstateold = 0;
  #define m365receiveroff 0 //Serial1 will not be read...
  #define m365receiverready 1 //reading bytes until we find 0x55
  #define m365receiverpacket1 2 //preamble 0x55 received, waiting for 0xAA
  #define m365receiverpacket2 3 //len preamble received waiting for LEN
  #define m365receiverpacket3 4 //payload this state is kept until LEN bytes received
  #define m365receiverpacket4 5 //checksum receiving checksum
  #define m365receiverpacket5 6 //received a packet, test for checksum
  #define m365receiverstorepacket 7 //packet received, checksum valid, store in array, jump back to receiverready for next packet, set newpacket flag

//M365 - packets
  uint8_t m365packetstate = 0;
  uint8_t m365packetstateold = 0;
  #define m365packetidle 0
  #define m365newpacket 1
  //offsets in sbuf
  #define i_address 0
  #define i_hz 1
  #define i_offset 2
  #define i_payloadstart 3
  #define address_ble 0x20 //actively sent data by BLE Module with gas/brake & mode values
  #define address_x1 0x21 //actively sent data by ?BLE? with status like led on/off, normal/ecomode...
  #define address_esc 0x23 
  #define address_bms_request 0x22 //used by apps to read data from bms
  #define address_bms 0x25 //data from bms sent with this address (only passive if requested via address_esc_request)

//M365 - Statistics
  uint16_t packets_rec=0;
  uint16_t packets_crcok=0;
  uint16_t packets_crcfail=0;
  uint16_t packets_rec_bms=0;
  uint16_t packets_rec_esc=0;
  uint16_t packets_rec_x1=0;
  uint16_t packets_rec_ble=0;
  uint16_t packets_rec_unhandled=0;

  uint16_t packetsperaddress[256];
  uint16_t requests_sent_bms=0;
  uint16_t requests_sent_esc=0;

  int16_t speed_min = 0;
  int16_t speed_max = 0;
  int16_t current_min = 0;
  int16_t current_max = 0;
  int32_t watt_min = 0;
  int32_t watt_max = 0;

  #define m365packettimeout  500 //timeout in ms after last received packet for showing connection-error
  unsigned long m365packettimestamp = 0;
  unsigned long m365packetlasttimestamp = 0;

  unsigned long duration_requestcycle=0;
  unsigned long timestamp_requeststart=0;


//M365 Device Buffers & Structs
  typedef struct {
    uint8_t u1[1];
    uint8_t throttle;
    uint8_t brake;
    //uint8_t u2[509];
  }__attribute__((packed))   blestruct;

  typedef struct {
    uint8_t mode; //offset 0x00 mode: 0-stall, 1-drive, 2-eco stall, 3-eco drive
    uint8_t battleds;  //offset 0x01 battery status 0 - min, 7(or 8...) - max
    uint8_t light;  //offset 0x02 0= off, 0x64 = on
    uint8_t beepaction;  //offset 0x03 "beepaction" ?
    //uint8_t u1[508];
  }__attribute__((packed))   x1struct;

  typedef struct {
    uint16_t u1[0x10]; //offset 0-0x1F
    char serial[14]; //offset 0x20-0x2D
    char pin[6]; //offset 0x2E-0x33
    uint8_t fwversion[2]; //offset 0x34-035,  e.g. 0x133 = 1.3.3
    uint16_t u2[10]; //offset 0x36-0x49
    uint16_t remainingdistance; //offset 0x4a-0x4B e.g. 123 = 1.23km
    uint16_t u3[20]; //offset 0x4C-0x73
    uint16_t ontime1; //offset 0x74-0x75 power on time in seconds
    uint16_t triptime; //offset 0x76-0x77 trip time in seconds
    uint16_t u4[2]; //offset 0x78-0x7C
    uint16_t frametemp1; //offset 0x7C-0x7D /10 = temp in °C
    uint16_t u5[54]; //offset 0x7e-0xe9
    uint16_t ecomode; //offset 0xEA-0xEB; on=1, off=0
    uint16_t u6[5]; //offset 0xec-0xf5
    uint16_t kers; //offset 0xf6-0xf7; 0 = weak, 1= medium, 2=strong
    uint16_t cruisemode; //offset 0xf8-0xf9, off 0, on 1
    uint16_t taillight; //offset 0xfa-0xfb, off 0, on 2
    uint16_t u7[50]; //offset  0xfc-0x15f
    uint16_t error; //offset 0x160-0x161
    uint16_t u8[3]; //offset 0x162-0x167
    uint16_t battpercent; //offset 0x168-0x169
    int16_t speed; //0x16A-0x16B /1000 in km/h, negative value = backwards...
    uint16_t averagespeed; //0x16C-0x16D /1000 in km/h?
    uint32_t totaldistance; //0x16e-0x171 /1000 in km
    uint16_t tripdistance; //0x172-0x173
    uint16_t ontime2; //offset 0x174-0x175 power on time 2 in seconds
    uint16_t frametemp2; //0x176-0x177 /10 = temp in °C
    uint16_t u10[68]; //offset 0x178-0x200 
  }__attribute__((packed))   escstruct;

  typedef struct {
    uint16_t u1[0x10]; //offset 0-0x1F
    char serial[14]; //offset 0x20-0x2D
    uint8_t fwversion[2]; //offset 0x2E-0x2f e.g. 0x133 = 1.3.3
    uint16_t totalcapacity; //offset 0x30-0x31 mAh
    uint16_t u2a[2]; //offset 0x32-0x35
    uint16_t cycles; //offset 0x36-0x37
    uint16_t chargingtimes; //offset 0x38-0x39
    uint16_t u2b[3]; //offset 0x3a-0x3f
    uint16_t proddate; //offset 0x40-0x41
        //fecha a la batt 7 MSB->año, siguientes 4bits->mes, 5 LSB ->dia ex:
      //b 0001_010=10, año 2010
      //        b 1_000= 8 agosto
      //            b  0_1111=15 
      //  0001_0101_0000_1111=0x150F 
    uint16_t u3[0x10]; //offset 0x42-0x61
    uint16_t remainingcapacity; //offset 0x62-0x63
    uint16_t remainingpercent; //offset 0x64-0x65
    int16_t current; //offset 0x66-67 - negative = charging; /100 = Ampere
    uint16_t voltage; //offset 0x68-69 /10 = Volt
    uint8_t temperature[2]; //offset 0x6A-0x6B -20 = °C
    uint16_t u4[5]; //offset 0x6C-0x75
    uint16_t health; //offset 0x76-0x77; 0-100, 60 schwellwert "kaputt"
    uint16_t u5[4]; //offset 0x78-0x7F
    uint16_t Cell1Voltage; //offset 0x80-0x81
    uint16_t Cell2Voltage; //offset 0x82-0x83
    uint16_t Cell3Voltage; //offset 0x84-0x85
    uint16_t Cell4Voltage; //offset 0x86-0x87
    uint16_t Cell5Voltage; //offset 0x88-0x89
    uint16_t Cell6Voltage; //offset 0x8A-0x8B
    uint16_t Cell7Voltage; //offset 0x8C-0x8D
    uint16_t Cell8Voltage; //offset 0x8E-0x8F
    uint16_t Cell9Voltage; //offset 0x90-0x91
    uint16_t Cell10Voltage; //offset 0x92-0x93
    uint16_t Cell11Voltage; //offset 0x94-0x95 not stock, custom bms with 12S battery, 0 if not connected
    uint16_t Cell12Voltage; //offset 0x96-0x97 not stock, custom bms with 12S battery, 0 if not connected
    //uint16_t u6[178]; //offset 0x98-0x
  }__attribute__((packed))  bmsstruct;

  uint8_t bledata[512];
  uint8_t x1data[512];
  uint8_t escdata[512];
  uint8_t bmsdata[512];

  blestruct* bleparsed = (blestruct*)bledata;
  x1struct* x1parsed = (x1struct*)x1data;
  escstruct* escparsed = (escstruct*)escdata;
  bmsstruct* bmsparsed = (bmsstruct*)bmsdata;

  bool newdata = false; //flag - we have update at least one byte in one of the data arrays
  bool senddata = false; //flag - we should send our data request _now_

//M365 - request stuff 
  /*
  request data from esc:   (Read 0x7x 2 words)
  PREAMBLE  LEN  Adr  HZ   Off  LEN  FIX1 FIX2 FIX30x
  0x55 0xaa 0x06 0x20 0x61 0x7c 0x02 0x02 0x28 0x27 CRC1 CRC2
  request data from bms: (Read Serial @ Offset 0x10, 0x10 words)
  PREAMBLE    LEN   Adr   HZ    Off   Len   CRC1  CRC2
  0x55  0xAA  0x03  0x22  0x01  0x10  0x12  0xB7  0xFF
  */

  //packet for bms requests and offsets
    uint8_t request_bms[9] = { 0x55,0xAA,0x03,0x22,0x01,0x10,0x3A,0xB7,0xFF};
    //uint8_t request_bms_serial[9] = { 0x55,0xAA,0x03,0x22,0x01,0x10,0x12,0xB7,0xFF};
    #define bms_request_offset 5
    #define bms_request_len 6
    #define bms_request_crcstart 2
    #define bms_request_crc1 7
    #define bms_request_crc2 8
  //packets for esc requests and offsets
    uint8_t request_esc[12] = { 0x55,0xAA,0x06,0x20,0x61,0x10,0xAC,0x02,0x28,0x27,0xB7,0xFF};
    //uint8_t request_esc_speed[12] = { 0x55,0xAA,0x06,0x20,0x61,0x7c,0x02,0x02,0x28,0x27,0xB7,0xFF};
    #define esc_request_offset 5 
    #define esc_request_len 6
    #define esc_request_throttle 8
    #define esc_request_brake 9
    #define esc_request_crcstart 2
    #define esc_request_crc1 10
    #define esc_request_crc2 11

  //request arrays  
    uint8_t requestindex = 0;
    #define requestmax 8
    uint8_t requests[requestmax][3]= {
        { address_esc, 0xB0, 0x20}, //error, battpercent,speed,averagespeed,totaldistance,tripdistance,ontime2,frametemp2
        { address_esc, 0x25, 0x02}, //remaining distance
        { address_esc, 0x3A, 0x0A}, //ontime, triptime, frametemp1
        { address_bms, 0x31, 0x0A}, //remaining cap & percent, current, voltage, temperature
        { address_bms, 0x40, 0x18}, //cell voltages (Cells 1-10 & 11-12 for 12S Batt/Custom BMS)
        { address_bms, 0x3B, 0x02}, //Battery Health
        { address_bms, 0x10, 0x22}, //serial,fwversion,totalcapacity,cycles,chargingtimes,productiondate
        { address_esc, 0x10, 0x16} //serial,pin,fwversion
    };

  //friendly names
    #define rq_esc_essentials 0x01
    #define rq_esc_remdist 0x02
    #define rq_esc_essentials2 0x04
    #define rq_bms_essentials 0x08
    #define rq_bms_cells 0x10
    #define rq_bms_health 0x20
    #define rq_bms_versioninfos 0x40
    #define rq_esc_versioninfos 0x80


//Misc
  #ifdef ESP32
    #define led GPIO_NUM_2
  #endif

  int ledontime = 200;
  int ledofftime = 10000;
  unsigned long ledcurrenttime = 100;
  uint8_t i;
  char tmp1[200];
  char tmp2[200];
  uint16_t tmpi;
  char chipid[7];
  char mac[12];
  unsigned int lastprogress=0;

  unsigned long duration_mainloop=0;
  unsigned long timestamp_mainloopstart=0;
  unsigned long duration_oled=0;
  unsigned long timestamp_oledstart=0;
  unsigned long duration_oled1draw=0;
  unsigned long timestamp_oled1draw=0;
  unsigned long duration_oled1i2c=0;
  unsigned long timestamp_oled1i2c=0;
  unsigned long duration_oled2draw=0;
  unsigned long timestamp_oled2draw=0;
  unsigned long duration_oled2i2c=0;
  unsigned long timestamp_oled2i2c=0;
  unsigned long duration_telnet=0;
  unsigned long timestamp_telnetstart=0;


  #define _max(a,b) ((a)>(b)?(a):(b))
  #define _min(a,b) ((a)<(b)?(a):(b))

void reset_statistics() {
  packets_rec=0;
  packets_crcok=0;
  packets_crcfail=0;
  for(uint16_t j = 0; j<=255; j++) {
    packetsperaddress[j]=0;  
  }
  for(uint16_t j = 0; j<=511; j++) {
    escdata[j]=0;  
    bmsdata[j]=0;  
    bledata[j]=0;  
    x1data[j]=0;  
  }
  speed_min = 0;
  speed_max = 0;
  current_min = 0;
  current_max = 0;
  watt_min = 0;
  watt_max = 0;
}

void start_m365() {
  M365SerialFull
  m365receiverstate = m365receiverready;
  m365packetstate=m365packetidle;
  reset_statistics();
}  //startm365

void m365_updatestats() {
  if (speed_min>escparsed->speed) { speed_min = escparsed->speed; }
  if (speed_max<escparsed->speed) { speed_max = escparsed->speed; }
  if (current_min>bmsparsed->current) { current_min = bmsparsed->current; }
  if (current_max<bmsparsed->current) { current_max = bmsparsed->current; }
  int32_t currentwatt = (int32_t)((float)(bmsparsed->voltage/100.0f)*(float)bmsparsed->current/100.0f);
  if (watt_min>currentwatt) { watt_min = currentwatt; }
  if (watt_max<currentwatt) { watt_max = currentwatt; }
}

void m365_handlerequests() {
  //find next subscribed index
    uint8_t startindex = requestindex;
    while (!(subscribedrequests&(1<<requestindex)))  {
      //DebugSerial.printf("skipping RQ index %d\r\n",requestindex+1);
      requestindex++;
      if (requestindex==requestmax) {
          //DebugSerial.println("rollover in rqloop1");
          requestindex=0; 
          duration_requestcycle=millis()-timestamp_requeststart;
          timestamp_requeststart=millis(); 
      }
      if (requestindex==startindex) { break; }
    }

  //request data for current index if we are interested in
  if (subscribedrequests&(1<<requestindex)) {
    //DebugSerial.printf("requesting RQ index %d\r\n",requestindex+1);
    if (requests[requestindex][0]==address_bms) {
      request_bms[bms_request_offset] = requests[requestindex][1];
      request_bms[bms_request_len] = requests[requestindex][2];
      crccalc = 0x00;
      for(uint8_t i=bms_request_crcstart;i<bms_request_crc1;i++) {
        crccalc=crccalc + request_bms[i];
      }
      crccalc = crccalc ^ 0xffff;
      request_bms[bms_request_crc1]=(uint8_t)(crccalc&0xff);
      request_bms[bms_request_crc2]=(uint8_t)((crccalc&0xff00)>>8);
      M365Serial.write((unsigned char*)&request_bms,9);
      requests_sent_bms++;
    } //if address address_bms
    if (requests[requestindex][0]==address_esc) {
      request_esc[esc_request_offset] = requests[requestindex][1];
      request_esc[esc_request_len] = requests[requestindex][2];
      request_esc[esc_request_throttle] = bleparsed->throttle;
      request_esc[esc_request_brake] = bleparsed->brake;
      crccalc = 0x00;
      for(uint8_t i=esc_request_crcstart;i<esc_request_crc1;i++) {
        crccalc=crccalc + request_esc[i];
      }
      crccalc = crccalc ^ 0xffff;
      request_esc[esc_request_crc1]=(uint8_t)(crccalc&0xff);
      request_esc[esc_request_crc2]=(uint8_t)((crccalc&0xff00)>>8);
      M365Serial.write((unsigned char*)&request_esc,12);
      requests_sent_esc++;
    } //if address address_esc
  } else {
    //DebugSerial.printf("skipping RQ index %d\r\n",requestindex+1);
  }
  //prepare next requestindex for next call
  requestindex++;
  if (requestindex==requestmax) { 
    //DebugSerial.println("rollover in rqloop2");
    requestindex=0; 
    duration_requestcycle=millis()-timestamp_requeststart;
    timestamp_requeststart=millis();
  }
  //DebugSerial.printf("---REQUEST-- %d\r\n",millis());
} //m365_handlerequests

void m365_handlepacket() {
 if (m365packetstate==m365newpacket) {
    m365packetlasttimestamp = millis()-m365packettimestamp;
    m365packettimestamp=millis();
    packetsperaddress[sbuf[i_address]]++;  
    

    #ifdef debug_dump_packetdecode
      sprintf(tmp1,"[%03d] PACKET Len %02X Addr %02X HZ %02X Offset %02X CRC %04X Payload: ",m365packetlasttimestamp,len,sbuf[i_address],sbuf[i_hz],sbuf[i_offset], crccalc);
      DebugSerial.print(tmp1);
      for(i = 0; i < len-3; i++){
        DebugSerial.printf("%02X ",sbuf[i_payloadstart+i]);
      }
      DebugSerial.println("");
      /*for(i = 0; i < MAX_SRV_CLIENTS; i++){
        if (serverClients[i] && serverClients[i].connected()){
          serverClients[i].write(tmp1,37);
          yield();
       }
      }*/
    #endif
    switch (sbuf[i_address]) {
      case address_bms:
          packets_rec_bms++;
          memcpy((void*)& bmsdata[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      case address_esc:
          packets_rec_esc++;
          memcpy((void*)& escdata[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      case address_ble:
          packets_rec_ble++;
          memcpy((void*)& bledata[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      case address_x1:
          packets_rec_x1++;
          memcpy((void*)& x1data[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      default:
          packets_rec_unhandled++;
        break;
    }
    newdata=true;
    m365packetstate=m365packetidle;
 } //if (m365packetstate==m365newpacket)
 else {
  if ((m365packettimestamp+m365packettimeout)>millis()) {
    //packet timeout
  }
 }
} //m365_handlepacket
  
void m365_receiver() { //recieves data until packet is complete
  uint8_t newbyte;
  if (M365Serial.available()) {
    newbyte = M365Serial.read();
#ifdef userawserver
     if (rawclient && rawclient.connected()) { rawclient.write(newbyte); }
#endif    
    switch(m365receiverstate) {
      case m365receiverready: //we are waiting for 1st byte of packet header 0x55
          if (newbyte==0x55) {
            m365receiverstate = m365receiverpacket1;
          }
        break;
      case m365receiverpacket1: //we are waiting for 2nd byte of packet header 0xAA
          if (newbyte==0xAA) {
            m365receiverstate = m365receiverpacket2;
            }
        break;
      case m365receiverpacket2: //we are waiting for packet length
          len = newbyte+1; //next byte will be 1st in sbuf, this is the packet-type1, len counted from 2nd byte in sbuf
          crccalc=newbyte;
          readindex=0;
          m365receiverstate = m365receiverpacket3;
        break;
      case m365receiverpacket3: //we are receiving the payload
          sbuf[readindex]=newbyte;
          readindex++;
          crccalc=crccalc+newbyte;
          if (readindex==len) {
            m365receiverstate = m365receiverpacket4;
          }
        break;
      case m365receiverpacket4: //we are waiting for 1st CRC byte
          crc1 = newbyte;
          m365receiverstate = m365receiverpacket5;
        break;
      case m365receiverpacket5: //we are waiting for 2nd CRC byte
          crc2 = newbyte;
          crccalc = crccalc ^ 0xffff;
          #ifdef usepacketserver
            if (packetclient && packetclient.connected()) { 
              sprintf(tmp1,"55 AA %02X ", len-1);
              sprintf(tmp2,"%02X %02X [CRCCalc:%04X]\r\n", crc1,crc2,crccalc);
              switch (sbuf[i_address]) {
                case address_bms:
                    packetclient.print(ansiRED);
                    packetclient.print("BMS: ");
                  break;
                case address_esc:
                    packetclient.print(ansiGRN);
                    packetclient.print("ESC: ");
                  break;
                case address_ble:
                    packetclient.print(ansiBLU);
                    packetclient.print("BLE: ");
                  break;
                case address_x1:
                    packetclient.print(ansiBLUF);
                    packetclient.print("X1 : ");
                  break;
                default:
                    packetclient.print(ansiREDF);
                    packetclient.print("???: ");
                  break;
                } //switch                else {
              packetclient.print(ansiEND);
              packetclient.print(tmp1);
              for(i = 0; i < len; i++){
                packetclient.printf("%02X ",sbuf[i]);
              } //for i...
              packetclient.print(tmp2);
            }
          #endif
          #ifdef debug_dump_rawpackets
            sprintf(tmp1,"Packet received: 55 AA %02X ", len-1);
            sprintf(tmp2,"CRC %02X %02X %04X\r\n", crc1,crc2,crccalc);
            DebugSerial.print(tmp1);
            for(i = 0; i < len; i++){
             DebugSerial.printf("%02X ",sbuf[i]);
            }
            DebugSerial.print(tmp2);
            /*
            for(i = 0; i < MAX_SRV_CLIENTS; i++){
              if (serverClients[i] && serverClients[i].connected()){
                serverClients[i].write(tmp1,26);
                for(i = 0; i < (len); i++){
                  telnetclient.printf("%02X ",sbuf[i]);
                  //serverClients[i].write(tmp1,3);
                }
                serverClients[i].write(tmp2, 12);
                yield();
             }
            }
            */
          #endif
          packets_rec++;
          if (crccalc==((uint16_t)(crc2<<8)+(uint16_t)crc1)) {
            m365packetstate = m365newpacket;
            packets_crcok++;
          } else {
            packets_crcfail++;
          }
        m365receiverstate = m365receiverready; //reset and wait for next packet
        if (sbuf[i_address]==0x20 && sbuf[i_hz]==0x65 && sbuf[i_offset]==0x00 ) {
          //senddata = true;
          m365_handlerequests();
          DebugSerial.printf("---REQUEST-- %d\r\n",millis());
        }
        break;
    } //switch
    //DebugSerial.printf("#S# %d\r\n",M365Serial.available());
  }//serial available
  
} //m365_receiver

#ifdef debug_dump_states //dump state machine states to Serial Port on change
  void print_states() {
    if (wlanstate!=wlanstateold) {
      DebugSerial.printf("### WLANSTATE %d -> %d\r\n",wlanstateold,wlanstate);
      wlanstateold=wlanstate;
    }
   
    if (m365receiverstate!=m365receiverstateold) {
      DebugSerial.printf("### M365RecState: %d -> %d\r\n",m365receiverstateold,m365receiverstate);
      m365receiverstateold=m365receiverstate;
    }
    if (m365packetstate!=m365packetstateold) {
      DebugSerial.printf("M365PacketState: %d -> %d\r\n",m365packetstateold,m365packetstate);
      m365packetstateold=m365packetstate;
    }    
  }
#endif

void setup() {
  pinMode(led,OUTPUT);
  digitalWrite(led,LOW);
  #ifdef ESP32
    DebugSerial.begin(115200);
  #endif

  DebugSerial.println(swversion);
  uint64_t cit;
  
  #ifdef ESP32
    cit = ESP.getEfuseMac();
    sprintf(chipid,"%02X%02X%02X",(uint8_t)(cit>>24),(uint8_t)(cit>>32),(uint8_t)(cit>>40));
    DebugSerial.println(chipid);
    sprintf(mac,"%02X%02X%02X%02X%02X%02X",(uint8_t)(cit),(uint8_t)(cit>>8),(uint8_t)(cit>>16),(uint8_t)(cit>>24),(uint8_t)(cit>>32),(uint8_t)(cit>>40));

  start_m365();
} //setup

void handle_led() {
  if (millis()>ledcurrenttime) {
    if (digitalRead(led)) {
      digitalWrite(led,LOW);
      ledcurrenttime = millis()+ledontime;
    } else {
      digitalWrite(led,HIGH);
      ledcurrenttime = millis()+ledofftime;
    }
  }
}

void loop() {
  timestamp_mainloopstart=micros();
 
  while (M365Serial.available()) { //decode anything we received as long as  there's data in the buffer
    m365_receiver(); 
    m365_handlepacket();
  }
  if (newdata) {
    m365_updatestats();
  }

  handle_led();

  #ifdef debug_dump_states
    print_states();
  #endif
  duration_mainloop=micros()-timestamp_mainloopstart;
}