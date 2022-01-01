
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>

int pinSensor = 21; //PINO DIGITAL UTILIZADO PELO SENSOR
bool sensor = digitalRead(pinSensor);
int verificacao=0;

#define ESP32_HELTEC_OLED_V2 1

#ifdef ESP32_HELTEC_OLED_V2
   #include "WiFi.h"
   #define MOSI 27
   #define MISO 19
   #define SCK 5
   #define SS 18
   #define RST 14
   #define DIO_0 26
   #define DIO_1 35
   #define DIO_2 34
#endif

#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else

# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif 


static const u1_t PROGMEM APPEUI[8]={ 0x3D, 0x5D, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]={ 0x47, 0xBC, 0xC7, 0x9D, 0x7F, 0x27, 0xEB, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0x10, 0xBF, 0x3F, 0x75, 0x91, 0x49, 0xA1, 0x50, 0x6C, 0x17, 0x2C, 0x65, 0x64, 0x41, 0x25, 0x56 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Projeto teste IoT";
uint8_t dados[3] = {0,0,0};
static osjob_t sendjob;

const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = SS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RST,
    .dio = {DIO_0, DIO_1, DIO_2},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
     
         dados[0] = sensor;         
         LMIC_setTxData2(1, dados, sizeof(dados), 0); //enviar o valor do vetor dado pela LoraWAN
        
        Serial.println(F("Packet queued"));
    }
  
}
void setup() {

  #ifdef ESP8266
    WiFi.mode(WIFI_OFF);  //desabilitar o WiFi do ESP8266 - caso não for usar para ter economia de energia...
#endif
#if defined(ESP32) || defined(ESP32_HELTEC_OLED_V1) || defined(ESP32_HELTEC_OLED_V2)
    btStop(); //desabilita BlueTooth - caso não for usar para ter economia de energia...
    WiFi.mode(WIFI_OFF); //desabilita WiFi - caso não for usar para ter economia de energia...
#endif
     Serial.begin(115200);  //deve-se modificar o Monitor Serial da IDE do Arduino para 115200bps !!!!
   //  Serial.begin(9600);    
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    //modo do pino do sensor, input pois ele tem um tratamento interno
  pinMode(pinSensor, INPUT);  
  
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    LMIC_selectSubBand(1);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
  
    //os_runloop_once();

   bool sensor = digitalRead(pinSensor);

  if (!sensor) {
   Serial.println("Veiculo detectado");
   verificacao = 1;
  
  } else {
    Serial.println("Veiculo não detectado");
    verificacao = 0;
  }

  Serial.println(verificacao);

  delay(1000);//delay 1 segundo

    
}


  
