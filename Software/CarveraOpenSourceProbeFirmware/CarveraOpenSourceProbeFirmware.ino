/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/



#include "NRF52840_HexToName.h"
#include <Arduino.h>
#include <bluefruit.h>


//#define StartBootMode//uncomment to start in bootmode
//#define ConstantPair//uncomment to start and stay in pair mode. Overwritten by StartBootMode
//#define SerialDebug // uncomment to use serial debugging.


#define PIN_VBAT (32)         // D32 battery voltage
#define PIN_VBAT_ENABLE (14)  // D14 LOW:read anable
#define PIN_HICHG (22)        // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG (23)          // D23 charge indicatore LOW:charge HIGH:no charge
#define PIN_BUTTON 9          //D2 normally closed button for probing.
#define PIN_WAKEUP 9
#define PIN_BOOTJUMPER 23
#define PIN_OTHERJUMPER 1
#define PIN_LASER01 8  //current sink pin for laser
#define PIN_LASER02 7  //current sink pin for laser

#define BATTERYPOWERCONVERSIONRATIO 1000 * 0.1856 * 3.6  //at some point calibrate this value to voltage readings from probe

// pin overrides, for David's setup. Shouldn't affect anything without command
// line parameters to define OVERRIDE_* variables
#ifdef OVERRIDE_LED_RED
#undef LED_RED
#define LED_RED OVERRIDE_LED_RED
#endif

#ifdef OVERRIDE_LED_GREEN
#undef LED_GREEN
#define LED_GREEN OVERRIDE_LED_GREEN
#endif

#ifdef OVERRIDE_LED_BLUE
#undef LED_BLUE
#define LED_BLUE OVERRIDE_LED_BLUE
#endif

#ifdef OVERRIDE_PIN_VBAT_ENABLE
#undef PIN_VBAT_ENABLE
#define PIN_VBAT_ENABLE OVERRIDE_PIN_VBAT_ENABLE
#endif

#ifdef OVERRIDE_PIN_BUTTON
#undef PIN_BUTTON
#define PIN_BUTTON OVERRIDE_PIN_BUTTON
#endif

#ifdef OVERRIDE_PIN_WAKEUP
#undef PIN_WAKEUP
#define PIN_WAKEUP OVERRIDE_PIN_WAKEUP
#endif

#ifdef OVERRIDE_PIN_HICHG
#undef PIN_HICHG
#define PIN_HICHG OVERRIDE_PIN_HICHG
#endif

#ifdef OVERRIDE_PIN_VBAT
#undef PIN_VBAT
#define PIN_VBAT OVERRIDE_PIN_VBAT
#endif

#ifdef OVERRIDE_PIN_LASER01
#undef PIN_LASER01
#define PIN_LASER01 OVERRIDE_PIN_LASER01
#endif

enum ProbeMode { UNDEF,
                 INIT,
                 IDLE,
                 SLEEP,
                 PAIR,
                 PROBE,
                 LASER,
                 UPDATE,
                 TEST };  //laser mode is wrapped inside pairing mode.
enum RadioMode { OFF,
                 SEND,
                 RECEIVE };

ProbeMode probe_mode_c = PROBE;

struct configurationVariablesStruct {

  ////////////////////////////configuration variables///////////////////////////////////////////////
  bool uninitialized = true;
  //button configuration variables
  int pollingRate = 100;                          //ms between tests
  unsigned long debounceDelay = 50;               // the debounce time; increase if the output flickers
  unsigned long buttonLongPressLength = 10000;    //how long you need to hold the button down before it enters pairing mode
  unsigned long buttonDoublePressTime = 1000;     //how long between button presses to register a double press
  unsigned long buttonHeartbeatUnpressed = 1000;  //how long between sending button released events in active probe mode

  //mode duration setup
  unsigned long pairingDelay = 10000;       //how long to wait in pairing mode before quitting back to probe mode
  unsigned long pairingLength = 20000;
  unsigned long laserDelay = 15000;         //how long to wait in laser mode before quitting back to probe mode
  unsigned long idleDelay = 20000;           //20000;          //how long to wait until the probe goes into idle mode
  unsigned long sleepingDelay = 10000;      //1000000; //how long until the probe goes into a deep sleep mode
  unsigned long idleHeartbeatDelay = 5000;  //how often to send heartbeat probe updates when in idle mode

  //communication configuration
  uint8_t pan[2] = { 0x22, 0x20 };
  uint8_t destination[2] = { 0xea, 0x0b };
  uint8_t source[2] = { 0xea, 0x0b };
  long ackInterval = 200;  // how long to wait for ACK packets
  int channel = 25;
  int channelRx = 25;
  bool send_unpressed_commands = false;
};
configurationVariablesStruct configurationVariables;

//////////////////////////////////setup variables////////////////////////////////////////////////////////////


//communication setup
uint8_t receivePkt[128] = {0x00};
uint8_t seq = 0;
//pairing wait
unsigned long previousMillis = 0;  // will store last time LED was updated need to consolodate with primary wait times
unsigned long currentMillis = 0;
// constants won't change:

//button setup variables
int button_state = LOW;     // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
bool single_press = true;
unsigned long blinkMillis = 0;  // will store last time LED was updated
int blinkState = 0;
unsigned long buttonLongPressMillis = 0;
unsigned long lastSwitchTime = 0;
unsigned long buttonHeartbeatUnpressedTime = 0;
unsigned long idleHeartbeatUnpressedTime = 0;

//laser setup variables
bool laserOn = false;

//batttery setup variables
int16_t vbatt = 0;


//flash storage setup
volatile uint32_t *flashConfig = (uint32_t *)0xfef00;  //rename to flash page. the second hex digit is the page number

//bluetooth BLE uarrt setup
BLEDfu bledfu;    // OTA DFU service
BLEDis bledis;    // device information
BLEUart bleuart;  // uart over ble
BLEBas blebas;    // battery

const byte blueart_numChars = 32;
char blueart_receivedChars[blueart_numChars];
char blueart_tempChars[blueart_numChars];  // temporary array for use when parsing

// variables to hold the parsed data
char blueart_returnCommand[blueart_numChars] = { 0 };
char blueart_returnVariable[blueart_numChars] = { 0 };
char blueart_returnValue[blueart_numChars] = { 0 };
uint8_t blueart_output_buf[64];

boolean blueart_newData = false;


//////////////////////config storage///////////////////////////////////

void eraseFlashPage(volatile uint32_t *addr, bool waitForFinish) {

  // block on NVMC ready
  //err_code =
  //sd_flash_page_erase(*addr);
  //Serial.println(err_code);

  while (!MMIO(NVMC_BASE, NVMC_READY)) {}

  // erase page, referencing the first byte in the page
  MMIO(NVMC_BASE, NVMC_CONFIG) = 2;  //enable flash erasure

  uint32_t startOfPage = ((uint32_t)addr) & 0xfffff000;
  MMIO(NVMC_BASE, NVMC_ERASEPAGE) = startOfPage;  // erase page
  MMIO(NVMC_BASE, NVMC_CONFIG) = 0;               //disable flash erasure

  if (waitForFinish) {
    // block on NVMC ready
    while (!MMIO(NVMC_BASE, NVMC_READY)) {}
  }
}

void writeFlash(volatile uint32_t *addr, uint32_t data, bool waitForFinish) {

  // block on NVMC readynext
  while (!MMIO(NVMC_BASE, NVMC_READYNEXT)) {}

  // write the word
  MMIO(NVMC_BASE, NVMC_CONFIG) = 1;  //enable flash writing
  *addr = data;
  MMIO(NVMC_BASE, NVMC_CONFIG) = 0;  //disable flash writing

  if (waitForFinish) {
    // block on NVMC ready
    while (!MMIO(NVMC_BASE, NVMC_READY)) {}
  }
}

void report_flash_vars_to_serial()  //for testing.
{
  for (int cnt = 0; cnt < sizeof(configurationVariables) / sizeof(int); cnt++)

  {
    Serial.println(*(flashConfig + cnt));
  }
}

void write_default_flash_vars() {

  //writeFlash(&flashConfig[0], configurationVariables, true); //todo. Alternatively when testing the probe, put into bluetooth mode and run a save command
}

void write_current_flash_vars() {

  Serial.println("vars at start: ");
  report_flash_vars_to_serial();


  eraseFlashPage(flashConfig, true);

  Serial.println("vars being written: ");

  // pointer to array of 16 bit values
  uint32_t *p;
  // take address of configurationVariables and assign to the pointer
  p = (uint32_t *)&configurationVariables;

  // loop thorugh the elements of the struct
  for (uint8_t cnt = 0; cnt < sizeof(configurationVariables) / sizeof(int); cnt++) {

    // p points to an address of an element in the array; *p gets you the value ofthat address
    writeFlash((flashConfig + cnt), *(p), true);
    Serial.println(*(p));
    p++;
  }

  Serial.println("vars at end: ");

  report_flash_vars_to_serial();
}

void read_current_flash_vars() {

  if (*(flashConfig) > 2) {
    bleuart.print("cannot read from flash");
    bleuart.print(*(flashConfig), HEX);
    return;
  }
  uint32_t *p;
  // take address of configurationVariables and assign to the pointer
  p = (uint32_t *)&configurationVariables;

  // loop thorugh the elements of the struct
  for (uint8_t cnt = 0; cnt < sizeof(configurationVariables) / sizeof(int); cnt++) {

    // p points to an address of an element in the array; *p gets you the value ofthat address
    *p = *(flashConfig + cnt);
    Serial.println(*(p));
    p++;
  }
}

void report_current_flash_vars_bleuart() {


  // loop thorugh the elements of the struct
  for (uint8_t cnt = 0; cnt < sizeof(configurationVariables) / sizeof(int); cnt++) {

    bleuart.print(*(flashConfig + cnt));
  }
}

///////////////////////radio//////////////////////////////////////////


void disable_radio() {
  if (MMIO(RADIO_BASE, RADIO_OFFSET_POWER)) {
    MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_DISABLED) = 0;
    MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_DISABLE) = 1;
    while (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_DISABLED) == 0) {} // wait to complete
  }
}

void set_radio_mode(RadioMode radio_mode) {

  disable_radio();
  switch (radio_mode) {
    case OFF:
      // turn off the radio, to conserve battery
      MMIO(RADIO_BASE, RADIO_OFFSET_POWER) = 0;

      // also turn off the high frequency clock, which is only needed for the radio, to save battery
      MMIO(CLOCK_BASE, CLOCK_OFFSET_TASKS_HFCLKSTOP) = 1;
      break;

    case SEND:
      MMIO(RADIO_BASE, RADIO_OFFSET_POWER) = 1; // turn on radio
      setup_radio_TXRX();
      MMIO(RADIO_BASE, RADIO_OFFSET_FREQUENCY) = configurationVariables.channel * 5 - 50;
      MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_TXEN) = 1;
      while (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_READY) == 0) {}  // wait to complete
      break;

    case RECEIVE:
      MMIO(RADIO_BASE, RADIO_OFFSET_POWER) = 1; // turn on radio
      setup_radio_TXRX();
      MMIO(RADIO_BASE, RADIO_OFFSET_FREQUENCY) = configurationVariables.channelRx * 5 - 50;
      MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_RXEN) = 1;
      while (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_READY) == 0) {}  // wait to complete
      break;

    default: break;
  }
}

void set_channel()  //needs to shut down and restart the radio between changing channels
{
  //to make this accessable in other areas, need to turn off radio, change channel, and then turn on radio
  //MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_DISABLE) = 0;
  //while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_DISABLED){}

  MMIO(RADIO_BASE, RADIO_OFFSET_FREQUENCY) = configurationVariables.channel * 5 - 50;  // 2475MHz = channel 25 page 321 of nRF52840_PS_v1.8
  //MMIO(RADIO_BASE, RADIO_OFFSET_FREQUENCY) = 75; // 2475MHz = channel 25
  //MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_DISABLE) = 0;
}

void setup_radio_TXRX()  // all the one time setup for the 802.15.4 radio
{
  // radio power on
  MMIO(RADIO_BASE, RADIO_OFFSET_POWER) = 1;
  //MMIO(RADIO_BASE, RADIO_OFFSET_TXPOWER) = 4;  //set radio power to 1-8
  // hf clock on
  MMIO(CLOCK_BASE, CLOCK_OFFSET_EVENTS_HFCLKSTARTED) = 0;         // clear event
  MMIO(CLOCK_BASE, CLOCK_OFFSET_TASKS_HFCLKSTART) = 1;            // enable
  while (!MMIO(CLOCK_BASE, CLOCK_OFFSET_EVENTS_HFCLKSTARTED)) {}  // wait for event
  MMIO(CLOCK_BASE, CLOCK_OFFSET_EVENTS_HFCLKSTARTED) = 0;         // clear event
    // misc radio configuration
  MMIO(RADIO_BASE, RADIO_OFFSET_CRCCNF) = 0x202;
  MMIO(RADIO_BASE, RADIO_OFFSET_CRCPOLY) = 0x11021;
  MMIO(RADIO_BASE, RADIO_OFFSET_CRCINIT) = 0;

  // disable radio shorts, radio interrupts, and PPI
  MMIO(RADIO_BASE, RADIO_OFFSET_SHORTS) = 0;
  MMIO(RADIO_BASE, RADIO_OFFSET_INTENCLR) = 0xffffffff;
  MMIO(0x4001F000, 0x500) = 0;  // PPI CHEN = 0

  //MMIO(RADIO_BASE, RADIO_OFFSET_CCACTRL) = 0x052D0000 | 2; // default thresholds; require energy level and carrier pattern to detect busy
  MMIO(RADIO_BASE, RADIO_OFFSET_CCACTRL) = 0x022D2D00;  // default thresholds; require energy level and carrier pattern to detect busy
  //set_channel();
  MMIO(RADIO_BASE, RADIO_OFFSET_MODE) = 15;  // ieee 802.15.4
  MMIO(RADIO_BASE, RADIO_OFFSET_PCNF0) = 8 | (2 << 24) | (1 << 26);
  MMIO(RADIO_BASE, RADIO_OFFSET_PCNF1) = 127;
  // MMIO(RADIO_BASE, 0x650) = 0x201; // MODECNF0
}

uint8_t batteryLow = 0;
uint8_t batteryHigh = 0;
void send_packet(uint8_t cmd = 0x01, bool read_battery = true, bool reset_seq = false) {
  uint8_t packet[15] = {};

  if (reset_seq) {
    seq = 0x00;
  }
  //packet lenght
  packet[0] = 14;
  //source addressing mode
  packet[1] = 0x61;
  packet[2] = 0x88;
  //sequence number
  packet[3] = seq++;
  //pan
  packet[4] = configurationVariables.pan[0];
  packet[5] = configurationVariables.pan[1];
  //destination
  if (cmd == 3) {
    // destination is broadcast address instead of paired address
    packet[6] = 0xff;
    packet[7] = 0xff;
  } else {
    packet[6] = configurationVariables.destination[0];
    packet[7] = configurationVariables.destination[1];
  }
  //source
  packet[8] = configurationVariables.source[0];
  packet[9] = configurationVariables.source[1];
  //data payload
  packet[10] = cmd;
  //battery status
  if (read_battery) {
    vbatt = analogRead(PIN_VBAT);  // convert to function later
    vbatt = BATTERYPOWERCONVERSIONRATIO * vbatt / 4096;  // conversion factor
    batteryLow = vbatt & 0xff;
    batteryHigh = vbatt >> 8;
  }
  if (cmd == 3) {
    packet[11] = configurationVariables.source[0];
    packet[12] = configurationVariables.source[1];
  } else {
    packet[11] = 0x6f; //batteryLow;
    packet[12] = 0x11; //batteryHigh;
  }

  _send_packet(packet);
}

void _send_packet(uint8_t pkt[127]) {
  // turn on radio, if necessary
  if (MMIO(RADIO_BASE, RADIO_OFFSET_POWER) == 0) {
    set_radio_mode(SEND);
  }

  // ensure state TXIDLE
  if (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_TXIDLE) {
    MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_TXEN) = 1;
  }
  while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_TXIDLE) { }

  // Send packet
  MMIO(RADIO_BASE, RADIO_OFFSET_PACKETPTR) = (uint32_t)&pkt[0];
  MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_START) = 1;

  //wait for complete
  while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_TXIDLE) {}
}

void send_ack() {
  uint8_t ack_pkt[6] = { 0x05, 0x02, 0x00 };
  ack_pkt[3] = receivePkt[3];
  _send_packet(ack_pkt);
}


int receive_packet() {  //return 0 for no packet, return 1 for a proper packet, return 2 for ack packet
  // turn on radio, if necessary
  if (MMIO(RADIO_BASE, RADIO_OFFSET_POWER) == 0) {
    set_radio_mode(RECEIVE);
  }

  // ensure state RXIDLE
  if (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_RXIDLE) {
    if (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) == RADIO_STATE_RX) {
      // cancel the current rx command, return to rxidle
      MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_STOP) = 1;
    } else {
      // rx rampup from disabled or tx
      MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_RXEN) = 1;
    }
  }
  while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_RXIDLE) { }

  // Receive packet
  for (int i = 0; i < 128; i++) {
    receivePkt[i] = 0xcc;
  }
  MMIO(RADIO_BASE, RADIO_OFFSET_PACKETPTR) = (uint32_t)&receivePkt[0];
  MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_END) = 0;
  MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCOK) = 0;
  MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCERROR) = 0;

  MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_START) = 1;
  previousMillis = millis();
  while (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_END) == 0) {  //wait until it receives something
    currentMillis = millis();
    if (currentMillis - previousMillis >= configurationVariables.ackInterval) {  //escape loop if taking too long
      return 0;
    }
  }

  //ack
  if (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCOK)) {  //check for CRC ok?
    if (receivePkt[0] == 0x5) {
      if (receivePkt[3] == seq - 1) {
        Serial.println("ack confirmed");
        return 2;
      } else {
        Serial.print("ack for unknown packet: ");
        Serial.print(receivePkt[3]);
        Serial.print(" (next seq = ");
        Serial.print(seq);
        Serial.println(")");
      }
    }
  }

  //packet
  if (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCOK)) {  //check for CRC ok?
    if (receivePkt[0] == 0xe && receivePkt[4] == 0x22 && receivePkt[5] == 0x20) {
      // length, PAN ok
      Serial.print(" length pan ok ");
      //Serial.print(receivePkt[10]);
      if (receivePkt[10] == 0x03) {
        Serial.println(" confirm packet ");
        configurationVariables.source[0] = receivePkt[11];
        configurationVariables.source[1] = receivePkt[12];
        configurationVariables.destination[0] = receivePkt[11];
        configurationVariables.destination[1] = receivePkt[12];
        return 1;
      }
    }
  }
  return 0;
}

///////////////////////blueart/////////////////////////
void setup_blueart() {

  disable_radio();
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  //Bluefruit.setTxPower(4);  // Check bluefruit.h for supported values
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("PMP");
  bledis.setModel("Carvera Open Source Touch Probe");
  Bluefruit.setName("PMP Open Source 3 Axis Probe v1");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();

  vbatt = analogRead(PIN_VBAT);
  vbatt = BATTERYPOWERCONVERSIONRATIO * vbatt / 4096;
  blebas.write(vbatt / 40);


  // Set up and start advertising
  startAdv();

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
}


void startAdv(void) {

  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html  
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle) {

  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {

  (void)conn_handle;
  (void)reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
}
void receive_blueart() {

  static boolean recvInProgress = false;
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (bleuart.available() > 0 && blueart_newData == false) {
    rc = bleuart.read();



    if (rc != endMarker) {
      blueart_receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= blueart_numChars) {
        ndx = blueart_numChars - 1;
      }

    } else {
      blueart_receivedChars[ndx] = '\0';  // terminate the string
      recvInProgress = false;
      ndx = 0;
      blueart_newData = true;
    }
  }
  if (blueart_newData == true) {
    strcpy(blueart_tempChars, blueart_receivedChars);
    blueart_parseData();
    blueart_parseInput();
    blueart_newData = false;
  }
}


void blueart_parseData() {  // split the data into its parts

  char *strtokIndx;  // this is used by strtok() as an index

  strtokIndx = strtok(blueart_tempChars, " ");  // get the first part - the string
  strcpy(blueart_returnCommand, strtokIndx);    // copy it to blueart_returnCommand

  strtokIndx = strtok(NULL, " ");              // get the first part - the string
  strcpy(blueart_returnVariable, strtokIndx);  // copy it to blueart_returnVariable

  strtokIndx = strtok(NULL, " ");           // get the first part - the string
  strcpy(blueart_returnValue, strtokIndx);  // copy it to blueart_returnValue
}

void blueart_get(bool report_all = false) {

  if (strcmp(blueart_returnVariable, "uninitialized") == 0 || report_all) {

    bleuart.printf("uninitialized = %d \r\n", configurationVariables.uninitialized);
  }
  if (strcmp(blueart_returnVariable, "pollingRate") == 0 || report_all) {

    bleuart.printf("pollingRate = %d \r\n", configurationVariables.pollingRate);
  }
  if (strcmp(blueart_returnVariable, "debounceDelay") == 0 || report_all) {

    bleuart.printf("debounceDelay = %d \r\n", configurationVariables.debounceDelay);
  }
  if (strcmp(blueart_returnVariable, "buttonLongPressLength") == 0 || report_all) {

    bleuart.printf("buttonLongPressLength = %d \r\n", configurationVariables.buttonLongPressLength);
  }
  if (strcmp(blueart_returnVariable, "buttonDoublePressTime") == 0 || report_all) {

    bleuart.printf("buttonDoublePressTime = %d \r\n", configurationVariables.buttonDoublePressTime);
  }
  if (strcmp(blueart_returnVariable, "buttonHeartbeatUnpressed") == 0 || report_all) {

    bleuart.printf("buttonHeartbeatUnpressed = %d \r\n", configurationVariables.buttonHeartbeatUnpressed);
  }
  if (strcmp(blueart_returnVariable, "pairingDelay") == 0 || report_all) {

    bleuart.printf("pairingDelay = %d \r\n", configurationVariables.pairingDelay);
  }
  if (strcmp(blueart_returnVariable, "laserDelay") == 0 || report_all) {

    bleuart.printf("laserDelay = %d \r\n", configurationVariables.laserDelay);
  }
  if (strcmp(blueart_returnVariable, "idleDelay") == 0 || report_all) {

    bleuart.printf("idleDelay = %d \r\n", configurationVariables.idleDelay);
  }
  if (strcmp(blueart_returnVariable, "idleHeartbeatDelay") == 0 || report_all) {

    bleuart.printf("idleHeartbeatDelay = %d \r\n", configurationVariables.idleHeartbeatDelay);
  }
  if (strcmp(blueart_returnVariable, "pan") == 0 || report_all) {

    uint16_t v_return = configurationVariables.pan[1] * 256 + configurationVariables.pan[0];
    bleuart.printf("pan = %04x\r\n", __builtin_bswap16(v_return));
  }
  if (strcmp(blueart_returnVariable, "destination") == 0 || report_all) {

    uint16_t v_return = configurationVariables.destination[1] * 256 + configurationVariables.destination[0];
    bleuart.printf("destination = %04x\r\n", __builtin_bswap16(v_return));
  }
  if (strcmp(blueart_returnVariable, "source") == 0 || report_all) {

    uint16_t v_return = configurationVariables.source[1] * 256 + configurationVariables.source[0];
    bleuart.printf("source = %04x\r\n", __builtin_bswap16(v_return));
  }
  if (strcmp(blueart_returnVariable, "ackInterval") == 0 || report_all) {

    bleuart.printf("ackInterval = %d \r\n", configurationVariables.ackInterval);
  }
  if (strcmp(blueart_returnVariable, "channel") == 0 || report_all) {

    bleuart.printf("channel = %d \r\n", configurationVariables.channel);
  }
  if (strcmp(blueart_returnVariable, "channelRx") == 0 || report_all) {

    bleuart.printf("channelRx = %d \r\n", configurationVariables.channelRx);
  } else {

    bleuart.print("error: variable does not exist");
  }
}

void blueart_set() {

  if (strcmp(blueart_returnVariable, "uninitialized") == 0) {

    int v_return = strtol(blueart_returnValue, NULL, 10);
    if (v_return >= 0) {

      configurationVariables.uninitialized = v_return;
      bleuart.printf("uninitialized = %d \r\n", configurationVariables.uninitialized);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid uninitialized.");
    }


  } else if (strcmp(blueart_returnVariable, "pollingRate") == 0) {

    int v_return = strtol(blueart_returnValue, NULL, 10);
    if (v_return > 0) {

      configurationVariables.pollingRate = v_return;
      bleuart.printf("pollingRate = %d \r\n", configurationVariables.pollingRate);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid pollingRate");
    }

  } else if (strcmp(blueart_returnVariable, "debounceDelay") == 0) {

    unsigned long v_return = strtoul(blueart_returnValue, NULL, 10);
    if (v_return > 0) {

      configurationVariables.debounceDelay = v_return;
      bleuart.printf("debounceDelay = %d \r\n", configurationVariables.debounceDelay);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid debounceDelay");
    }

  } else if (strcmp(blueart_returnVariable, "buttonLongPressLength") == 0) {

    unsigned long v_return = strtoul(blueart_returnValue, NULL, 10);
    if (v_return > 0) {

      configurationVariables.buttonLongPressLength = v_return;
      bleuart.printf("buttonLongPressLength = %d \r\n", configurationVariables.buttonLongPressLength);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid buttonLongPressLength");
    }

  } else if (strcmp(blueart_returnVariable, "buttonDoublePressTime") == 0) {

    unsigned long v_return = strtoul(blueart_returnValue, NULL, 10);
    if (v_return > 0) {

      configurationVariables.buttonDoublePressTime = v_return;
      bleuart.printf("buttonDoublePressTime = %d \r\n", configurationVariables.buttonDoublePressTime);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid buttonDoublePressTime");
    }

  } else if (strcmp(blueart_returnVariable, "buttonHeartbeatUnpressed") == 0) {

    unsigned long v_return = strtoul(blueart_returnValue, NULL, 10);
    if (v_return > 0) {

      configurationVariables.buttonHeartbeatUnpressed = v_return;
      bleuart.printf("buttonHeartbeatUnpressed = %d \r\n", configurationVariables.buttonHeartbeatUnpressed);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid buttonHeartbeatUnpressed");
    }

  } else if (strcmp(blueart_returnVariable, "pairingDelay") == 0) {

    unsigned long v_return = strtoul(blueart_returnValue, NULL, 10);
    if (v_return > 0) {

      configurationVariables.pairingDelay = v_return;
      bleuart.printf("pairingDelay = %d \r\n", configurationVariables.pairingDelay);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid pairingDelay");
    }

  } else if (strcmp(blueart_returnVariable, "laserDelay") == 0) {

    unsigned long v_return = strtoul(blueart_returnValue, NULL, 10);
    if (v_return > 0) {

      configurationVariables.laserDelay = v_return;
      bleuart.printf("laserDelay = %d \r\n", configurationVariables.laserDelay);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid laserDelay");
    }

  } else if (strcmp(blueart_returnVariable, "idleDelay") == 0) {

    unsigned long v_return = strtoul(blueart_returnValue, NULL, 10);
    if (v_return > 0) {

      configurationVariables.idleDelay = v_return;
      bleuart.printf("idleDelay = %d \r\n", configurationVariables.idleDelay);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid idleDelay");
    }

  } else if (strcmp(blueart_returnVariable, "sleepingDelay") == 0) {

    unsigned long v_return = strtoul(blueart_returnValue, NULL, 10);
    if (v_return > 0) {

      configurationVariables.sleepingDelay = v_return;
      bleuart.printf("sleepingDelay = %d \r\n", configurationVariables.sleepingDelay);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid sleepingDelay");
    }

  } else if (strcmp(blueart_returnVariable, "idleHeartbeatDelay") == 0) {

    unsigned long v_return = strtoul(blueart_returnValue, NULL, 10);
    if (v_return > 0) {

      configurationVariables.idleHeartbeatDelay = v_return;
      bleuart.printf("idleHeartbeatDelay = %d \r\n", configurationVariables.idleHeartbeatDelay);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid idleHeartbeatDelay");
    }

  } else if (strcmp(blueart_returnVariable, "pan") == 0)  //todo
  {
    uint16_t v_return = configurationVariables.pan[1] * 256 + configurationVariables.pan[0];
    bleuart.printf("pan = %04x\r\n", __builtin_bswap16(v_return));
    //set variable

  } else if (strcmp(blueart_returnVariable, "destination") == 0)  //todo
  {
    uint16_t v_return = configurationVariables.pan[1] * 256 + configurationVariables.pan[0];
    bleuart.printf("pan = %04x\r\n", __builtin_bswap16(v_return));

    //set variable

  } else if (strcmp(blueart_returnVariable, "source") == 0)  //todo
  {
    uint16_t v_return = configurationVariables.pan[1] * 256 + configurationVariables.pan[0];
    bleuart.printf("pan = %04x\r\n", __builtin_bswap16(v_return));

    //set variable

  } else if (strcmp(blueart_returnVariable, "ackInterval") == 0) {

    long v_return = strtol(blueart_returnValue, NULL, 10);
    if (v_return < 1000 && v_return >= 0) {

      configurationVariables.ackInterval = v_return;
      bleuart.printf("ackInterval = %d \r\n", configurationVariables.ackInterval);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid ackInterval");
    }

  } else if (strcmp(blueart_returnVariable, "channel") == 0) {

    int v_return = strtol(blueart_returnValue, NULL, 10);
    if (v_return < 27 && v_return > 10) {

      configurationVariables.channel = v_return;
      bleuart.printf("channel = %d \r\n", configurationVariables.channel);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid channel number.");
    }

  } else if (strcmp(blueart_returnVariable, "channelRx") == 0) {

    int v_return = strtol(blueart_returnValue, NULL, 10);
    if (v_return < 27 && v_return > 10) {

      configurationVariables.channelRx = v_return;
      bleuart.printf("channelRx = %d \r\n", configurationVariables.channelRx);
      bleuart.print("when finished setting variables, send a save command");

    } else {

      bleuart.print("error: invalid channel number.");
    }
  } else {

    bleuart.print("error: variable ");
    bleuart.print(blueart_returnVariable);
    bleuart.print("does not exist");
  }
}

void blueart_stop()  //right now this completely resets the device, which works but it would be nicer if it just shut off the service
{
  Bluefruit.Advertising.restartOnDisconnect(false);
  uint16_t connections = Bluefruit.connected();
  for (uint16_t conn = 0; conn < connections; conn++) {
    Bluefruit.disconnect(conn);
  }
  Bluefruit.Advertising.stop();
  //set_radio_mode(OFF);
  sd_softdevice_disable();
}

void blueart_parseInput() {

  if (strcmp(blueart_returnCommand, "exit") == 0)  // hangs somewhere when reenabling 802 radio
  {
    bleuart.print("received command: exit");
    //shut down blueart
    blueart_stop();
    //switch to probe mode //things fail here
    delay(1000);
    set_radio_mode(SEND);
    probe_mode_c = PROBE;
    probeCycle();


  } else if (strcmp(blueart_returnCommand, "get") == 0) {

    blueart_get();

  } else if (strcmp(blueart_returnCommand, "set") == 0) {

    bleuart.print("received command: set");
    blueart_set();

  } else if (strcmp(blueart_returnCommand, "report") == 0) {

    bleuart.print("received command: report");
    blueart_get(true);

  } else if (strcmp(blueart_returnCommand, "save") == 0)  // fails to restart BLE UART service on return
  {
    bleuart.print("received command: save");

    blueart_stop();
    write_current_flash_vars();
    delay(1000);

    //setup_blueart();
    //blueart_get(true);

  } else if (strcmp(blueart_returnCommand, "erase") == 0)  // fails to restart BLE UART service on return
  {
    bleuart.print("received command: erase");

    blueart_stop();
    eraseFlashPage(flashConfig, true);
    delay(1000);

    //setup_blueart();
    //blueart_get(true);

  } else if (strcmp(blueart_returnCommand, "clear") == 0) {

    bleuart.print("received command: clear");
    read_current_flash_vars();
    blueart_get(true);

  } else if (strcmp(blueart_returnCommand, "help") == 0)  // need to do a writeup of all commands and vars. Proably use report to get var names.
  {
    bleuart.print("received command: help");

  } else if (strcmp(blueart_returnCommand, "info") == 0)  // need to report serial number, firmware version, etc.
  {
    bleuart.print("received command: info");

  } else if (strcmp(blueart_returnCommand, "readFlash") == 0) {

    bleuart.print("received command: readFlash");
    report_current_flash_vars_bleuart();

  } else if (strcmp(blueart_returnCommand, "test") == 0)  // hangs somewhere when reenabling 802 radio
  {
    bleuart.print("received command: test.");
    //shut down blueart
    blueart_stop();
    //switch mode //things fail here
    delay(1000);
    set_radio_mode(SEND);
    probe_mode_c = TEST;
    testCycle();
  } else {

    bleuart.print("error: improper command");
  }
}
///////////////////////core////////////////////////////

void sendbuttonpress(int state) {
  set_radio_mode(SEND);
  if (state) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    Serial.println("button pressed");
    send_packet(0x01);

  } else {                          //todo add an option to send button not pressed over 802
    digitalWrite(LED_RED, LOW);     // turn the LED off by making the voltage HIGH
    digitalWrite(LED_GREEN, HIGH);  // turn the LED on (LOW is the voltage level)
    if (configurationVariables.send_unpressed_commands) {
      Serial.println("button released");
      send_packet(0x00);
    }
  }
}

void probeCycle() {
  currentMillis = millis();
  int reading = digitalRead(PIN_BUTTON);

  // Normal button press handling logic starts here
  if (reading == HIGH && button_state == LOW) {
    lastDebounceTime = currentMillis;
  }

  // Button hold detection
  if (reading == HIGH && button_state == HIGH) {
    if ((currentMillis - lastDebounceTime) > configurationVariables.buttonLongPressLength) {
      Serial.println("pairing");
      probe_mode_c = PAIR;
      button_state = LOW; // if still pressed when returning from pairing, treat it as a new press
      laserOn = false;  // Turn off laser when button is held
    }
  }

  // Button release and double press detection
  if (reading == LOW && button_state == HIGH) {
    if ((currentMillis - lastDebounceTime) > configurationVariables.debounceDelay) {

      // Handle double press detection
      if ((currentMillis - lastSwitchTime) < configurationVariables.buttonDoublePressTime) {
        laserOn = !laserOn;
        Serial.println("double press");
      } else {
        // First press is detected
        single_press = true;
        lastSwitchTime = currentMillis;  // Set the time for the first press
      }
    }
  }

  button_state = reading;

  // Send button press state
  sendbuttonpress(button_state);

  // Laser cycle handling
  if (laserOn) {
    digitalWrite(PIN_LASER01,LOW); //turn on lasers
    digitalWrite(PIN_LASER02,LOW);

    Serial.println("Im a frickin laser. Pew pew");
    if (currentMillis - blinkMillis >= 1000) {
      blinkMillis = currentMillis;
      blinkState = (blinkState == LOW) ? HIGH : LOW;
    }
    if ((currentMillis - lastDebounceTime) > configurationVariables.laserDelay) {
      laserOn = false;
      blinkState = HIGH;
    }
    digitalWrite(LED_BLUE, blinkState);
  } else {
    digitalWrite(PIN_LASER01,HIGH); //turn off lasers
    digitalWrite(PIN_LASER02,HIGH);
    digitalWrite(LED_BLUE, HIGH);
  }

  delay(configurationVariables.pollingRate);

  // Check for idle state
  testforIdle();
}

void pairCycle() {

  unsigned long currentMillis = millis();
  unsigned long pairing_prevMillis = currentMillis;
  int returnValue = 0;

  if (currentMillis - lastDebounceTime > configurationVariables.pairingLength) {

    lastDebounceTime = currentMillis;
    probe_mode_c = PROBE;
    return;
  }

  if (currentMillis - blinkMillis >= 1000) {
    // save the last time you blinked the LED
    blinkMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (blinkState == LOW) {
      blinkState = HIGH;
    } else {
      blinkState = LOW;
    }

    digitalWrite(LED_RED, blinkState);
    digitalWrite(LED_BLUE, blinkState);
  }

  Serial.println("starting Pairing");
  //send pairing packet 1
  set_radio_mode(RECEIVE);
  while (receive_packet() != 2) {

    if ((currentMillis - pairing_prevMillis) > 3000)  //&& (currentMillis - lastDebounceTime) < configurationVariables.pairingDelay)
    {
      Serial.println("Machine never sent first confirm packet");
      return;
    }
    set_radio_mode(SEND);
    send_packet(0x03);
    set_radio_mode(RECEIVE);
    currentMillis = millis();
  }
  //Serial.println("first packet confirmed, waiting for packet from machine");
  //wait for response packet
  while (receive_packet() != 1) {

    currentMillis = millis();
    if ((currentMillis - pairing_prevMillis) > 3000)  //&& (currentMillis - lastDebounceTime) < configurationVariables.pairingDelay)
    {
      Serial.println("pairing timout: waiting on machine response");
      return;
    }
  }
  Serial.print("received machine packet, sending second confirm");
  set_radio_mode(SEND);
  send_ack();

  // Reset sequence number; send status packet
  send_packet(0x06, /*new battery reading*/true, /*reset seq*/true);
  Serial.println("Pairing success");
  probe_mode_c = PROBE;
}

void laserCycle()  //this does nothing right now. It is here in case we want to implement a standalone laser mode without the probing functions
{
  Serial.println("Im a frickin laser. Pew pew");
  unsigned long currentMillis = millis();

  //sendpacket("pairing");
  if ((currentMillis - lastDebounceTime) > configurationVariables.laserDelay)  // || digitalRead(PIN_BUTTON))
  {
    // if pairing delay has been reached, go back to probe mode
    probe_mode_c = PROBE;
  }

  if (currentMillis - blinkMillis >= 1000) {
    // save the last time you blinked the LED
    blinkMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (blinkState == LOW) {
      blinkState = HIGH;
    } else {
      blinkState = LOW;
    }

    digitalWrite(LED_GREEN, blinkState);
    digitalWrite(LED_BLUE, blinkState);
  }
}

void updateCycle() {

  setup_blueart();
  while (1) {

    // Forward data from HW Serial to BLEUART
    while (Serial.available() && false) {

      // Delay to wait for enough input, since we have a limited transmission buffer
      delay(2);

      uint8_t buf[64];
      int count = Serial.readBytes(buf, sizeof(buf));
      bleuart.write(buf, count);
    }

    // Forward from BLEUART to HW Serial
    receive_blueart();
  }
}

void offCycle() {

  Serial.println("off");

  // turn off all high power peripherals
  digitalWrite(LED_RED, HIGH); // turn off LEDs
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(PIN_LASER01,HIGH); // turn off lasers
  digitalWrite(PIN_LASER02,HIGH);
  set_radio_mode(OFF);  // turn off radio and HF clock

  sd_power_system_off();  // this function puts the whole nRF52 to deep sleep (no Bluetooth).  If no sense pins are setup (or other hardware interrupts), the nrf52 will not wake up.

  // This is unreachable except for operation under debuggers; in normal execution the
  // chip is now in deep sleep, and execution will begin again on wake (gpio button)
  // from the top with a full reset
  //
  // in debuggers, code execution may continue here under emulated system off mode,
  // and will loop
}

void testforIdle() {

  // shutdown when time reaches sleepingDelay ms
  if ((currentMillis > configurationVariables.idleDelay + lastDebounceTime) && button_state == LOW && !laserOn) {
    Serial.println("going to sleep");
    probe_mode_c = IDLE;
  }
}

void idleCycle() {
  int beatsUntilSleep = max(1, ceil(configurationVariables.sleepingDelay / (float)configurationVariables.idleHeartbeatDelay));

  // turn off all high power peripherals
  digitalWrite(LED_RED, HIGH); // turn off LEDs
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(PIN_LASER01,HIGH); // turn off lasers
  digitalWrite(PIN_LASER02,HIGH);
  set_radio_mode(OFF);  // turn off radio and HF clock

  while (1) {
    if (digitalRead(PIN_BUTTON) || probe_mode_c == PROBE) {
      lastDebounceTime = millis();
      set_radio_mode(SEND);
      probe_mode_c = PROBE;
      return;
    }

    if (beatsUntilSleep <= 0) {
      // time for deep sleep, no more heartbeats
      probe_mode_c = SLEEP;
      return;
    }

    if (MMIO(RTC0_BASE, RTC_OFFSET_EVENTS_COMPARE0)) {
      // send a heartbeat
      set_radio_mode(SEND);
      Serial.println("idle heartbeat");
      send_packet(0x06);
      disable_radio();

      // set up for next heartbeat
      beatsUntilSleep--;
      MMIO(RTC0_BASE, RTC_OFFSET_TASKS_CLEAR) = 1;
      MMIO(RTC0_BASE, RTC_OFFSET_EVENTS_COMPARE0) = 0;
    } else {
      // sleep until next button event or heartbeat
      MMIO(RTC1_BASE, RTC_OFFSET_TASKS_STOP) = 1;  // Disable RTC1 to prevent Arduino tick functionality
      __set_BASEPRI(6 << (8 - __NVIC_PRIO_BITS));  // Set BASEPRI to mask RTC0 interrupt
      MMIO(RTC0_BASE, RTC_OFFSET_INTENSET) = 1 << 16;  // Enable RTC0 interrupt

      __WFE();  // Wait for event (CPU enters low-power state)

      // Disable RTC0 interrupt after wake-up
      MMIO(RTC0_BASE, RTC_OFFSET_INTENCLR) = 1 << 16;  // Disable RTC0 interrupt
      NVIC_ClearPendingIRQ(RTC0_IRQn);  // Clear RTC0 interrupt pending flag

      MMIO(RTC1_BASE, RTC_OFFSET_TASKS_START) = 1;  // Re-enable RTC1
    }
  }
}

void GPIO_Handler() {
  if (probe_mode_c == SLEEP || probe_mode_c == IDLE) {
    // wake up
    probe_mode_c = PROBE;
  }
}


void initHeartbeatTimer() {
  // set up RTC0 to track when it's time for the next heartbeat message
  // note: LF clock must already be running, but arduino sets this up for us

  // compute RTC0 config variables
  const int unscaledTicks = configurationVariables.idleHeartbeatDelay * 1e3 / RTC_TICK_US;
  const int prescale = unscaledTicks / (RTC_COUNTER_MAX);
  const int scaledTicks = unscaledTicks / (prescale + 1);

  // stop and set config variables
  MMIO(RTC0_BASE, RTC_OFFSET_TASKS_STOP) = 1;
  MMIO(RTC0_BASE, RTC_OFFSET_TASKS_CLEAR) = 1;
  MMIO(RTC0_BASE, RTC_OFFSET_EVENTS_COMPARE0) = 0;
  MMIO(RTC0_BASE, RTC_OFFSET_PRESCALER) = prescale;
  MMIO(RTC0_BASE, RTC_OFFSET_CC0) = scaledTicks;
  MMIO(RTC0_BASE, RTC_OFFSET_EVTEN) = 1 << 16;  // enable compare to CC0, disable others

  // set interrupt priority to avoid actually triggering the interrupt handler
  // (Arduino makes it very hard to define your own interrupt handlers, so instead
  // we use a BASEPRIO high enough that the interrupt never fires, leverage the
  // pending interrupt to trigger a wake from WFE, and simply clear the pending
  // interrupt to allow it to trigger again)
  NVIC_SetPriority(RTC0_IRQn, 7);
  __set_BASEPRI(6 << (8 - __NVIC_PRIO_BITS));
  SCB->SCR |= SCB_SCR_SEVONPEND_Msk;  // https://www.embedded.com/the-definitive-guide-to-arm-cortex-m0-m0-ultralow-power-designs
  NVIC_EnableIRQ(RTC0_IRQn);

  // start the RTC
  MMIO(RTC0_BASE, RTC_OFFSET_TASKS_START) = 1;
}

/////////////////////////////////tests///////////////////////////////////////////////////////////////////////////////////////////////////

void testCycle() {
}

/////////////////////////////////////Setup and main loop ////////////////////////////////////////////////////////////////////////////////

// the setup function runs once when you press reset or power the board
void setup() {
  //Bluefruit.begin();          // Sleep functions need the softdevice to be active.

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  pinMode(PIN_LASER01, OUTPUT);
  pinMode(PIN_LASER02, OUTPUT);

  //initialize digital pin for button as Input
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // Attach button interrupt for falling edge (button press)
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), GPIO_Handler, FALLING);

#ifdef SerialDebug
  Serial.begin(115200);
  delay(1000);
  //while(!Serial) delay(10); //remove for battery only operation
  Serial.println("nRF52840 Carvera Wireless Probe Core Features Example");
  Serial.println("--------------------------------\n");
#endif
  //setup battery
  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  pinMode(PIN_HICHG, OUTPUT);
  pinMode(PIN_CHG, INPUT);


  digitalWrite(PIN_VBAT_ENABLE, LOW);  // VBAT read enable
  digitalWrite(PIN_HICHG, LOW);        // charge current 100mA




  // initialise ADC wireing_analog_nRF52.c:73
  analogReference(AR_DEFAULT);  // default 0.6V*6=3.6V  wireing_analog_nRF52.c:73
  analogReadResolution(16);     // wireing_analog_nRF52.c:39

  //read_current_flash_vars();
  initHeartbeatTimer();
  //filesystem setup
  set_radio_mode(SEND);
  buttonHeartbeatUnpressedTime = currentMillis;


#ifdef ConstantPair
  probe_mode_c = PAIR;
#endif
#ifdef StartBootMode
  probe_mode_c = UPDATE;
#endif
}

// the loop function runs over and over again forever
void loop() {

  switch (probe_mode_c) {

    case UNDEF: break;

    case INIT: break;

    case IDLE: idleCycle(); break;

    case PAIR: pairCycle(); break;

    case PROBE: probeCycle(); break;

    case LASER: laserCycle(); break;

    case TEST: testCycle(); break;

    case UPDATE: updateCycle(); break;

    case SLEEP: offCycle(); break;

    default: break;
  }
}
