/*--------------------------------------- INCLUDES --------------------------------------------- */
// Include library for Temp sensor incl. one wire bus
#include <DallasTemperature.h>
#include <OneWire.h>

// Include library for LORAWAN connection
#include <lmic.h>

// Include library for Cayenne Cloud which is used to prepare payloads in correct form 
#include <CayenneLPP.h>

// Include hardware abstraction layer for arduino environment
#include <hal/hal.h>

// Include library for HX711 (weightsensor)
#include <HX711.h>



/*---------------------------------------- DEFINES ---------------------------------------------- */
// Temp sensor
#define ONE_WIRE_BUS 5

// HX711 circuit wiring
#define LOADCELL_DOUT_PIN 3
#define LOADCELL_SCK_PIN 4

// Timer IC
#define TIMER_DONE A1

// Battery Meaurement
#define BATTERY_VOLTAGE A0


// HX711 Adjustment
//#define LOADCELL_DIVIDER 23.01458f
//#define LOADCELL_OFFSET -6826
#define LOADCELL_DIVIDER -22.7f
#define LOADCELL_OFFSET 156250



/*------------------------------------ GLOBAL VARIABLES ----------------------------------------- */
// Credentials which are necessary to connect to TTN (modify if you use other TTN account or device)
static const u1_t NWKSKEY[16] = { 0xB9, 0xCE, 0x75, 0x89, 0xB5, 0x91, 0x7B, 0xE9, 0xA4, 0xC6, 0xAB, 0x94, 0xE6, 0xBA, 0x8D, 0x83 };
static const u1_t APPSKEY[16] = { 0xB8, 0x54, 0x5D, 0x8F, 0xE1, 0x0A, 0x7C, 0x04, 0x50, 0x1D, 0xCD, 0x21, 0x97, 0x74, 0xF6, 0x0A };
static const u4_t DEVADDR = 0x26011FBC;

// Schedule TX messages over Lora within defined seconds
const unsigned TX_INTERVAL = 150; 

// Instantiate object for Temp sensor and one wire bus
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Instantiate object for HX711
HX711 loadcell;

// Instantiate Cayenne format object with maximum of 51 bytes lenght
CayenneLPP lpp(51);

// Instantiate sendjob object from LMIC library
static osjob_t sendjob;

// Declare struct for Dragino Shield pin mapping 
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};



/*-------------------------------------- PROTOTYPES ------------------------------------------- */




/*--------------------------------------- CALLBACKS -------------------------------------------- */
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }



/*--------------------------------------- FUNCTIONS -------------------------------------------- */

///////////////////////////////////////////////////////////////////////////////////////////////////
// Setup - will be processed once at the beginning
///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

    // Inititialize Timer IC Done pin and set it to LOW
    pinMode(TIMER_DONE, OUTPUT);
    digitalWrite(TIMER_DONE, LOW);
    
	  // Initialize serial interface with 9600 Baud-Rate
    Serial.begin(9600);
    Serial.println(F("Starting ..."));

    Serial.println(F("Initialize LMIC ..."));

    // Initialize LMIC lib
    os_init();
    
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters.
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF12,14);

    Serial.println(F("Initialize Temperature Sensor ..."));
        
    // Temp sensor begin
    sensors.begin();

    Serial.println(F("Initialize weight sensor ..."));
        
    // HX 711 begin
    loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); 
    loadcell.set_scale(LOADCELL_DIVIDER);                      // this value is obtained by calibrating the scale with known weights; see the README for details
    loadcell.set_offset(LOADCELL_OFFSET);// * LOADCELL_DIVIDER);

    //loadcell.tare();

    // Start job manually once; afterwards it will be started every defined seconds (see variable TX_INTERVAL )
    do_send(&sendjob);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// Loop will be processed after setup for infinite times
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {   

	// Let LMIC run in loop
  os_runloop_once(); 

  //Serial.println(loadcell.read_average(5));
  //Serial.println(loadcell.get_units(10), 2);

  // Battery Measurement
  /*int val=0;
  float voltage;
  val = analogRead(BATTERY_VOLTAGE);
  voltage = (float)val/1024.0*5.0;
  Serial.println(val);
  Serial.println(voltage);
  voltage = voltage * 3.2;
  
  Serial.println(voltage);*/

  // Get weight
  /*float weight;
  weight = loadcell.get_units(10);///1000.0;
  Serial.println(weight);
  
  delay(500);*/
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// onEvent is Handler which is called every time a event occurs in the LMIC stack 
///////////////////////////////////////////////////////////////////////////////////////////////////
void onEvent (ev_t ev) {
	
  // If the event was trigger by completing a sendjob 
  if (ev == EV_TXCOMPLETE) 
  {
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
	
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

      // Send Done Command to Timer ic
      digitalWrite(TIMER_DONE, HIGH);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// do_send is called manually or automatically by the LMIC OS every time a timer elapses TX_INTERVAL seconds
///////////////////////////////////////////////////////////////////////////////////////////////////
void do_send(osjob_t* j){
	  // Instantiate float variables for temp and humidity value
    float temperature;
    float weight;
    float voltage;

    // Get weight
    weight = loadcell.get_units(10)/1000.0;

    // Get battery voltage
    int val=0;
    val = analogRead(BATTERY_VOLTAGE);
    voltage = (float)val/1024.0*5.0;
    voltage = voltage * 3.2;

    // Send the command to get temperatures
    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);
    Serial.println(temperature);
   
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) 
	{
        Serial.println(F("OP_TXRXPEND, not sending"));
    } 
	else 
	{
        // Prepare upstream data transmission at the next possible time.
        lpp.reset();
        lpp.addTemperature(1, temperature);
        lpp.addAnalogInput(2, weight);
        lpp.addAnalogInput(3, voltage);
        //lpp.addRelativeHumidity(2, humidity);
        
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.println(F("Sending uplink packet..."));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}
