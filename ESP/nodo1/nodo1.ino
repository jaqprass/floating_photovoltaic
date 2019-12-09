
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Wire.h>                // I2C library
#include <Adafruit_INA219.h>     // INA219 sensor library
#include <OneWire.h>             // Libraby neede for temperature sensor communication
#include <DallasTemperature.h>   // DS18B20 temperature sensor library
#include <DHT.h>                 // DHT11 sensor library

#define DHTPIN 12                // Set pin 12 to the DHT sensor
#define DHTTYPE DHT11            // DHT 11
#define RELE 23                  // Set digital pin 23 as relay drive
#define SENSOR_TEMP 13           // Set pin 13 to the temperatures sensors
#define QTD_SENSORS 3            // Amount of temperature sensors

DHT dht(DHTPIN, DHTTYPE);
Adafruit_INA219 ina219;
OneWire pin(SENSOR_TEMP);
DallasTemperature tempSensors(&pin);
DeviceAddress tempDeviceAddress;

// Struct to N temperature sensors 
typedef struct{
    DeviceAddress hSensorId;
    float         fTemp;
} stTempSensor;

// Temperature sensor array
stTempSensor sensors[] ={
    {{0}, 0},
    {{0}, 0},
    {{0}, 0}};

float fHumid = 0.0f, fTempEnv = 0.0f;        // Humidity and enviroment temperature from DHT11 sensor
float fBusVoltage = 0.0f;             // INA sensor voltage
float fCurrent = 0.0f, fIsc = 0.0f;          // INA sensor current
short countReads = 0;

uint8_t mydata[32] = {0};      // Data buffer to send
//static uint8_t mydata[] = "Hello, world!";

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0xAB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0xAB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed.
static const u4_t DEVADDR = 0xAB000007; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h, otherwise the linker will complain)
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations)
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, LMIC_UNUSED_PIN},
};

void onEvent (ev_t ev){
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
            break;
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
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
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
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// Setup all configuration. Run only once.
void setup(){
  Serial.begin(115200);
  delay(100);               // per sample code on RF_95 test
  Serial.println(F("Starting"));
  
  dht.begin();                 //Initializes DHT11 sensor
  pinMode(RELE, OUTPUT);       //Sets the rele pin how output
  digitalWrite(RELE, HIGH);    //Inicializes relay in turn off (NF)
  uint32_t currentFrequency; 
  ina219.begin();              //Initializes the INA219 sensor
  tempSensors.begin();         //Initializes the temperature sensors

  //Find temperature sensors address
  short i = 0;
  for(i=0; i<QTD_SENSORS; i++){
    if(tempSensors.getAddress(tempDeviceAddress, i))
      memcpy(sensors[i].hSensorId , tempDeviceAddress, sizeof(tempDeviceAddress));
  }
    
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
  #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
  #endif
  
  #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
  #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
  #endif
  
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
  
  // Start job
  do_send(&sendjob);
}

// Function that will be performed continuously
void loop(){

  // Every 1 minute reads the sensors
  if (countReads%12==0){
    readTempSensors();         // Reads the temperature sensors
    readSensorDHT();           // Reads the humidity and enviroment temperature of DHT11 sensor
    readSensorVoltCurrent();   // Reads voltage and current at load
  
    // APAGAR OS SERIAL.PRINTLN
    Serial.println("Temperature sensors:"); Serial.println(sensors[0].fTemp); Serial.println(sensors[1].fTemp); Serial.println(sensors[2].fTemp);
    Serial.println(String("Environment temperature: ") + fTempEnv + " ºC");
    Serial.println(String("Humidity: ") + fHumid + " %");
    Serial.println(String("Bus Voltage: ") + fBusVoltage + " V");
    Serial.println(String("Current: ") + fCurrent + " mA");
    Serial.println("");
  }

  // When pass 15 minutes reads the current current
  if (countReads == 180){
    //Switchs the load
    digitalWrite(RELE, LOW);   // Turn on the relay (NA)
    delay(30000);              // Wait 30 seconds
    readSensorIrradition();    // Reads short circuit current
    delay(5000);               // Wait 5 seconds
    digitalWrite(RELE, HIGH);  // Turn off the relay (NF)
  
    Serial.println(String("Irradiation: ") + fIsc + " W/m²");
    Serial.println("");
    float2Bytes();      // Converts and concatenates sensor data into a single buffer
    countReads=0;
  }

  float2Bytes();        // Converts and concatenates sensor data into a single buffer
  os_runloop_once();
  countReads++;
  
  delay(5000);          // Wait 5 seconds to read the sensors again 
  
//  Serial.print("VAI MONTAR O BUFFER DE DADOS:");
//  float2Bytes();            // Converts and concatenates sensor data into a single buffer 
//
//  Serial.println("BUFFER DE DADOS:");
//  Serial.println("temp 1:");
//  Serial.print(mydata[0], HEX); Serial.print(" "); Serial.print(mydata[1], HEX); Serial.print(" "); Serial.print(mydata[2], HEX); Serial.print(" "); Serial.print(mydata[3], HEX);
//  
//  Serial.println("");
//  bytes2Float();

}

// Temperature sensor function
void readTempSensors() {
  short iCurrSensor = 0;
  for(iCurrSensor=0; iCurrSensor<QTD_SENSORS; iCurrSensor++){
    tempSensors.requestTemperatures();                                                     // Enable the temperature read
    sensors[iCurrSensor].fTemp = tempSensors.getTempC(sensors[iCurrSensor].hSensorId);     //Reads the temperature
  }
}

// DHT11 humidity and environment temperature sensor function
void readSensorDHT(){
  fTempEnv = dht.readTemperature();     // Reads the environment temperature
  fHumid = dht.readHumidity();          // Reads the humidity
}

// Irradiation function
void readSensorIrradition(){
  short i = 0;  
  for(i=0;i<10;i++){                    // Reads the current a few times repeatedly
    fIsc = ina219.getCurrent_mA();     
  }
}

// Voltage and current function
void readSensorVoltCurrent(){
  short i = 0;
  for(i=0;i<10;i++){                    // Reads the current and voltage a few times repeatedly
    fBusVoltage = ina219.getBusVoltage_V();
    fCurrent = ina219.getCurrent_mA();
  }
}

void float2Bytes(){
  memcpy(mydata, (uint8_t*) (&sensors[0].fTemp), 4);
  memcpy(mydata+4, (uint8_t*) (&sensors[1].fTemp), 4);
  memcpy(mydata+8, (uint8_t*) (&sensors[2].fTemp), 4);
  memcpy(mydata+12, (uint8_t*) (&fTempEnv), 4);
  memcpy(mydata+16, (uint8_t*) (&fHumid), 4);
  memcpy(mydata+20, (uint8_t*) (&fIsc), 4);
  memcpy(mydata+24, (uint8_t*) (&fBusVoltage), 4);
  memcpy(mydata+28, (uint8_t*) (&fCurrent), 4);
}

//APAGAR! PARA TESTE
void bytes2Float(){
  float t1, t2, t3, ta, h, ir, v, c;
  memcpy(&t1, mydata, 4);
  memcpy(&t2, mydata+4, 4);
  memcpy(&t3, mydata+8, 4);
  memcpy(&ta, mydata+12, 4);
  memcpy(&h, mydata+16, 4);
  memcpy(&ir, mydata+20, 4);
  memcpy(&v, mydata+24, 4);
  memcpy(&c, mydata+28, 4);
  Serial.print("DADOS CONVETIDOS DE VOLTA PARA FLOAT:");
  Serial.print(t1); Serial.print(" "); Serial.print(t2); Serial.print(" "); Serial.print(t3); Serial.print(" "); Serial.print(ta); Serial.print(" ");
  Serial.print(h); Serial.print(" "); Serial.print(ir); Serial.print(" "); Serial.print(v); Serial.print(" "); Serial.println(c);
}
