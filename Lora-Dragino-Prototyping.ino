//Temp.&Feuchte Sensor DHT11

/*--------------------------------------- INCLUDES --------------------------------------------- */
// Include library for Temp/Humidity sensor
#include <DHT_U.h>

// Include library for LORAWAN connection
#include <lmic.h>

// Include library for Cayenne Cloud which is used to prepare payloads in correct form 
#include <CayenneLPP.h>

// Include hardware abstraction layer for arduino environment
#include <hal/hal.h>



/*---------------------------------------- DEFINES ---------------------------------------------- */
// Temp/Humidity sensor
#define DHTPIN 5           // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT11   // Type DHT 11 is used as sensor



/*------------------------------------ GLOBAL VARIABLES ----------------------------------------- */
// Credentials which are necessary to connect to TTN (modify if you use other TTN account or device)
static const u1_t NWKSKEY[16] = { 0x76, 0x99, 0xE0, 0x63, 0x90, 0x0A, 0x93, 0x3F, 0x7D, 0x72, 0x8D, 0x7E, 0xE4, 0x5C, 0x5B, 0x07 };
static const u1_t APPSKEY[16] = { 0xA3, 0xD2, 0x85, 0xB8, 0x09, 0xBD, 0x80, 0xDE, 0xD4, 0x85, 0x47, 0x8E, 0xF2, 0xAA, 0x5B, 0xF7 };
static const u4_t DEVADDR = 0x260117B6 ;

// Schedule TX messages over Lora within defined seconds
const unsigned TX_INTERVAL = 150; 

// Delay which is minimal needed between sensor readings
uint32_t delayMS;

// Instantiate object for Temp/Humidity sensor class
DHT_Unified dht(DHTPIN, DHTTYPE); 

// Instantiate object for DHT sensor which is used to read out infos from library
sensor_t sensor;

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
	
	// Initialize serial interface with 9600 Baud-Rate
    Serial.begin(9600);
    Serial.println(F("Starting..."));
	
	// Start Temp/Humidity sensor
    dht.begin(); //DHT11 Sensor starten
    
	// Get all data of temperature sensor and send it over the serial interface
    dht.temperature().getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
    Serial.println(F("------------------------------------"));
	
	
    // Get all data of humidity sensor and send it over the serial interface
    dht.humidity().getSensor(&sensor);
    Serial.println(F("Humidity Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
    Serial.println(F("------------------------------------"));
    
	
    // Set delay between sensor readings based on sensor details.
    delayMS = sensor.min_delay / 1000;
  
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

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF12,14);

    // Start job manually once; afterwards it will be started every defined seconds (see variable TX_INTERVAL )
    do_send(&sendjob);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// Loop will be processed after setup for infinite times
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {   

	// Let LMIC run in loop
    os_runloop_once();      
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// onEvent is Handler which is called every time a event occurs in the LMIC stack 
///////////////////////////////////////////////////////////////////////////////////////////////////
void onEvent (ev_t ev) {
	
	// If the event was trigger by completing a sendjob 
    if (ev == EV_TXCOMPLETE) {
		
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
		
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// do_send is called manually or automatically by the LMIC OS every time a timer elapses TX_INTERVAL seconds
///////////////////////////////////////////////////////////////////////////////////////////////////
void do_send(osjob_t* j){
	// Instantiate float variables for temp and humidity value
    float temperature;
    float humidity;

    // Instantiate sensor event object
    sensors_event_t event;

    // Minimum delay between measurements.
	delay(delayMS);
  
  
	// Get temperature data 
	dht.temperature().getEvent(&event);
	if (isnan(event.temperature)) 
	{
		Serial.println(F("Error reading temperature!"));
	}
	else 
	{
		Serial.print(F("Temperature: "));
		Serial.print(event.temperature);
		Serial.println(F("°C"));
		temperature = event.temperature;
	}
	
	
	// Get humidity data
	dht.humidity().getEvent(&event);
	if (isnan(event.relative_humidity)) 
	{
		Serial.println(F("Error reading humidity!"));
	}
	else 
	{
		Serial.print(F("Humidity: "));
		Serial.print(event.relative_humidity);
		Serial.println(F("%"));
		humidity = event.relative_humidity;
	}
   
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
        lpp.addRelativeHumidity(2, humidity);
        
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        Serial.println(F("Sending uplink packet..."));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}
