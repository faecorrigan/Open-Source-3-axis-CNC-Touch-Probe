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

unsigned long millLaserCommandDuration = 12500; // ms to keep laser on after mill command

// Any time a breaking change is made to how configuration variables are saved into
// flash, this version should be bumped. Ideally, read_current_flash_vars should also
// be updated to correctly read both old and new versions correctly
// To avoid confusion with uninitialized flash, never use version 0xFF. Or, for good measure, version 0
const uint8_t CONFIG_VERSION = 1;

struct configurationVariablesStruct {

  ////////////////////////////configuration variables///////////////////////////////////////////////
  uint8_t version = CONFIG_VERSION;
  //button configuration variables
  int pollingRate = 6;                          //ms between tests
  unsigned long debounceDelay = 50;               // the debounce time; increase if the output flickers
  unsigned long buttonLongPressLength = 9500;    //how long you need to hold the button down before it enters pairing mode
  unsigned long buttonDoublePressTime = 1000;     //how long between button presses to register a double press
  unsigned long buttonHeartbeatUnpressed = 1000;  //how long between sending button released events in active probe mode

  //mode duration setup
  unsigned long pairingDelay = 10000;       //how long to wait in pairing mode before quitting back to probe mode
  unsigned long pairingLength = 20000;
  unsigned long laserDelay = 15000;         //how long to wait in laser mode before quitting back to probe mode
  unsigned long idleDelay = 20000;           //how long to wait until the probe goes into idle mode
  unsigned long sleepingDelay = 1000 * 60 * 60 * 24 * 3;      //how long until the probe goes into a deep sleep mode
  unsigned long idleHeartbeatDelay = 1000 * (60 * 5 + 10);  //how often to send heartbeat probe updates when in idle mode

  //communication configuration
  uint8_t pan[2] = { 0x22, 0x20 };
  uint8_t destination[2] = { 0xea, 0x0b };
  uint8_t source[2] = { 0xea, 0x0b };
  long ackInterval = 200;  // how long to wait for ACK packets
  int channel = 25;
  int channelRx = 25;
  bool send_unpressed_commands = false;
  uint8_t seq = 0;
};
configurationVariablesStruct configurationVariables;

//////////////////////////////////setup variables////////////////////////////////////////////////////////////


//communication setup
uint8_t receivePkt[128] = {0x00};
//pairing wait
unsigned long previousMillis = 0;  // will store last time LED was updated need to consolodate with primary wait times
unsigned long currentMillis = 0;
// constants won't change:

//button setup variables
int button_state = LOW;     // the current reading from the input pin
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long lastLaserCommandTime = 0;  // the last time the mill sent a laser on command
unsigned long lastButtonSendTime = 0;  // the last time the probe sent a button press packet to the mill
unsigned long blinkMillis = 0;  // will store last time LED was updated
int blinkState = 0;
unsigned long buttonLongPressMillis = 0;
unsigned long lastSwitchTime = 0;
unsigned long idleHeartbeatUnpressedTime = 0;

//laser setup variables
bool laserOn = false;

//batttery setup variables
int16_t vbatt = 0;


//flash storage setup
volatile uint32_t *configFlashPage = (uint32_t *)0xfef00; // note: must point to the start of a flash page (multiple of 0x100)

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

void writeFlash(volatile uint32_t *addr, uint32_t data) {
  // block on NVMC readynext
  while (!MMIO(NVMC_BASE, NVMC_READYNEXT)) {}

  MMIO(NVMC_BASE, NVMC_CONFIG) = 1;  //enable flash writing
  *addr = data;
  MMIO(NVMC_BASE, NVMC_CONFIG) = 0;  //disable flash writing

  // block on NVMC ready
  while (!MMIO(NVMC_BASE, NVMC_READY)) {}
}

void report_flash_vars_to_serial()  //for testing.
{
  for (int cnt = 0; cnt < sizeof(configurationVariables) / sizeof(int); cnt++)

  {
    Serial.println(*(configFlashPage + cnt));
  }
}

void write_default_flash_vars() {

  //writeFlash(&configFlashPage[0], configurationVariables, true); //todo. Alternatively when testing the probe, put into bluetooth mode and run a save command
}

// returns true if any change was made
// does not rewrite the page if there are no changes to make
bool write_current_flash_vars() {
  bool changed = false;
  for (int i = 0; i < sizeof(configurationVariables) / sizeof(char); i++) {
    if (((char *)configFlashPage)[i] != ((char *)&configurationVariables)[i]) {
      changed = true;
      break;
    }
  }

  if (changed) {
    Serial.println("flash vars at start: ");
    report_flash_vars_to_serial();

    Serial.println("vars being written...");
    eraseFlashPage(configFlashPage, true);

    int count = (sizeof(configurationVariables) + sizeof(uint32_t) - 1)/ sizeof(uint32_t);
    uint32_t *src = (uint32_t *)&configurationVariables;
    for (int i=0; i < count; i++) {
      writeFlash(&configFlashPage[i], src[i]);
    }

    Serial.println("vars at end: ");
    report_flash_vars_to_serial();
  }

  return changed;
}

void read_current_flash_vars() {
  configurationVariablesStruct *tmp = (configurationVariablesStruct *)configFlashPage;
  if (tmp->version == CONFIG_VERSION) {
    configurationVariables = *tmp;
  }
}

void report_current_flash_vars_bleuart() {


  // loop thorugh the elements of the struct
  for (uint8_t cnt = 0; cnt < sizeof(configurationVariables) / sizeof(int); cnt++) {

    bleuart.print(*(configFlashPage + cnt));
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
      while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_TXIDLE) {} // wait to complete
      break;

    case RECEIVE:
      MMIO(RADIO_BASE, RADIO_OFFSET_POWER) = 1; // turn on radio
      setup_radio_TXRX();
      MMIO(RADIO_BASE, RADIO_OFFSET_FREQUENCY) = configurationVariables.channelRx * 5 - 50;
      MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_RXEN) = 1;
      while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_RXIDLE) {} // wait to complete
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

// returns true if it got an ack
uint8_t batteryLow = 0;
uint8_t batteryHigh = 0;
bool send_packet(uint8_t cmd) {
  // construct the packet
  uint8_t packet[15] = {};

  //packet length
  packet[0] = 14;
  //source addressing mode
  packet[1] = 0x61;
  packet[2] = 0x88;
  //sequence number
  packet[3] = configurationVariables.seq;
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
  bool read_battery = cmd == 0x06;
  if (read_battery) {
    pinMode(PIN_VBAT_ENABLE, OUTPUT);
    digitalWrite(PIN_VBAT_ENABLE, LOW);  // VBAT read enable
    vbatt = analogRead(PIN_VBAT);  // convert to function later
    pinMode(PIN_VBAT_ENABLE, INPUT);
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

  // actually send the packet
  _send_packet(packet);

  // wait for an ack
  uint32_t ackStart = micros();
  uint32_t ackTimeout = 1300; // I clocked one ack at 977us, and added some buffer to that
  receive_async();
  while (micros() - ackStart < ackTimeout) {
    if (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCOK) == 1) {
      if (receivePkt[0] == 0x5) {
        volatile uint32_t diff = micros() - ackStart;
        if (receivePkt[3] == configurationVariables.seq) {
          // got ack -- increment seq and return
          configurationVariables.seq++;
          return 1;
        }
      }
    }
    if (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_RX) {
      receive_async();
    }
  }

  // timed out with no ack -- return without incrementing seq
  return 0;
}

void _send_packet(uint8_t pkt[127]) {
  // turn on radio, if necessary
  if (MMIO(RADIO_BASE, RADIO_OFFSET_POWER) == 0) {
    set_radio_mode(SEND);
  }

  // ensure state TXIDLE
  while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) == RADIO_STATE_TX) {
    MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_STOP) = 1;
  }
  while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) == RADIO_STATE_RX) {
    MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_STOP) = 1;
  }
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

void clear_receive_packet() {
  for (int i = 0; i < 128; i++) {
    receivePkt[i] = 0x0;
  }
}

// performs necessary actions to put the radio in rx mode and start listening for
// packets. This process is blocking, but actually waiting for a packet is async
void receive_async() {
  // turn on radio, if necessary
  if (MMIO(RADIO_BASE, RADIO_OFFSET_POWER) == 0) {
    set_radio_mode(RECEIVE);
  }

  // ensure state RXIDLE or RX
  if (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_RX) {
    if (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_RXIDLE) {
      // rx rampup from disabled or tx
      MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_RXEN) = 1;
      while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_RXIDLE) { }
    }
    // state is now RXIDLE; prepare for and start receive

    // Receive packet
    clear_receive_packet();
    MMIO(RADIO_BASE, RADIO_OFFSET_PACKETPTR) = (uint32_t)&receivePkt[0];
    MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_END) = 0;
    MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCOK) = 0;
    MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCERROR) = 0;

    MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_START) = 1;
  }
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

    bleuart.printf("uninitialized = %d \r\n", configurationVariables.version != CONFIG_VERSION);
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

      configurationVariables.version = v_return;
      bleuart.printf("uninitialized = %d \r\n", configurationVariables.version != CONFIG_VERSION);
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
    probeCycle(true);


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
    eraseFlashPage(configFlashPage, true);
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
  if (state) {
    Serial.println("button pressed");
    send_packet(0x01);

  } else {                          //todo add an option to send button not pressed over 802
    if (configurationVariables.send_unpressed_commands) {
      Serial.println("button released");
      send_packet(0x00);
    }
  }
}

void probeCycle(bool firstTime) {
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
      lastLaserCommandTime = 0;
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
        lastSwitchTime = currentMillis;  // Set the time for the first press
      }
    }
  }

  button_state = reading;

  digitalWrite(LED_RED, button_state);
  digitalWrite(LED_GREEN, !button_state);

  // Send button press state
  if (lastButtonSendTime == 0 || currentMillis - lastButtonSendTime >= configurationVariables.pollingRate) {
    sendbuttonpress(button_state);
    lastButtonSendTime = currentMillis;
  }

  // Send heartbeat if it has been long enough since the last one
  // (or none have been sent since power on)
  if (firstTime || MMIO(RTC0_BASE, RTC_OFFSET_EVENTS_COMPARE0)) {
    sendHeartbeat();
  }

  // Check if we have received a mill laser command
  if (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCOK) == 1) {
    // received a valid radio transmission
    if (receivePkt[0] == 0xe && receivePkt[4] == 0x22 && receivePkt[5] == 0x20) {
      // packet has the expected length and correct PAN
      if (receivePkt[6] == configurationVariables.source[0] && receivePkt[7] == configurationVariables.source[1]) {
        // packet is addressed to this probe
        if (receivePkt[10] == 0x02) {
          // packet is a laser on command
          lastLaserCommandTime = currentMillis;
          MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCOK) = 0;
          clear_receive_packet();
        }
      }
    }
  }

  // If not already listening, listen for mill commands
  receive_async();

  // Laser cycle handling
  if (laserOn || (lastLaserCommandTime > 0 && currentMillis <= lastLaserCommandTime + millLaserCommandDuration)) {
    digitalWrite(PIN_LASER01,LOW); //turn on lasers
    digitalWrite(PIN_LASER02,LOW);

    if (currentMillis - blinkMillis >= 1000) {
      blinkMillis = currentMillis;
      blinkState = (blinkState == LOW) ? HIGH : LOW;
      Serial.println("Im a frickin laser. Pew pew");
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

  // Check for idle state
  unsigned long idleTime = lastDebounceTime;
  if (lastLaserCommandTime > 0) {
    idleTime = max(idleTime, lastLaserCommandTime + millLaserCommandDuration);
  }
  idleTime += configurationVariables.idleDelay;
  if (currentMillis > idleTime && button_state == LOW && !laserOn) {
    Serial.println("going to sleep");
    probe_mode_c = IDLE;
  }
}

bool pairingShouldExit() {
  // check if the button is released
  if(digitalRead(PIN_BUTTON) == LOW) {
    lastDebounceTime = millis();
    button_state = LOW; // indicates the probe was last noticed untriggered
    probe_mode_c = PROBE;
    return true;
  }
  return false;
}

void pairCycle() {
  unsigned long currentMillis = millis();

  if (pairingShouldExit()) {
    return;
  }

  delay(1000);

  // blink leds
  blinkState = !blinkState;
  digitalWrite(LED_RED, blinkState);
  digitalWrite(LED_BLUE, blinkState);

  Serial.println("Pairing...");

  //send pairing packet 1 until the machine sends back an ack
  bool gotAck = false;
  const int maxRepeat = 10;
  for (int sentCount = 0; !gotAck && sentCount < maxRepeat; sentCount++) {
    uint32_t sendTime = micros();
    gotAck = send_packet(0x03);
    if (!gotAck && sentCount + 1 < maxRepeat) {
      // delay before sending next packet
      while (micros() - sendTime < 1025) { // approximate value, measured
        if (pairingShouldExit()) {
          return;
        }
      }
    }
  }
  if (!gotAck) {
    return; // restart pairing process
  }

  bool pairingSucceeded = false;
  uint32_t startReceiveTime = millis();
  receive_async();
  while (millis() - startReceiveTime < 2000) {
    // listen for a reply from the mill and send an ack
    // do not exit early because there is no guarantee the mill hears the ack!!
    // the mill needs our ack so it also knows pairing succeeded

    if (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCOK) == 1) {
      // got a valid packet -- process it
      if (receivePkt[0] == 0xe && receivePkt[4] == 0x22 && receivePkt[5] == 0x20 && receivePkt[10] == 0x03) {
        // length, PAN ok
        configurationVariables.source[0] = receivePkt[11];
        configurationVariables.source[1] = receivePkt[12];
        configurationVariables.destination[0] = receivePkt[11];
        configurationVariables.destination[1] = receivePkt[12];
        pairingSucceeded = true;
        send_ack();
      }
    }

    if (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_RX) {
      receive_async();
    }
  }

  if (pairingSucceeded) {
    // Reset sequence number, save pairing info, and send a fresh status packet
    Serial.println("Pairing success");
    configurationVariables.seq = 0;
    write_current_flash_vars();
    sendHeartbeat();

    // configure state to enter the probe active cycle
    lastDebounceTime = millis();
    button_state = LOW; // indicates the probe was last noticed untriggered
    probe_mode_c = PROBE;
  }
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

  // save configuration variables to flash
  write_current_flash_vars();

  sd_power_system_off();  // this function puts the whole nRF52 to deep sleep (no Bluetooth).  If no sense pins are setup (or other hardware interrupts), the nrf52 will not wake up.

  // This is unreachable except for operation under debuggers; in normal execution the
  // chip is now in deep sleep, and execution will begin again on wake (gpio button)
  // from the top with a full reset
  //
  // in debuggers, code execution may continue here under emulated system off mode,
  // and will loop
}

// sends a heartbeat and sets up the RTC for the next one
void sendHeartbeat() {
  send_packet(0x06);
  MMIO(RTC0_BASE, RTC_OFFSET_EVENTS_COMPARE0) = 0;
  MMIO(RTC0_BASE, RTC_OFFSET_TASKS_CLEAR) = 1;
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
      sendHeartbeat();
      beatsUntilSleep--;
      disable_radio();
    } else {
      // sleep until next button event or heartbeat
      MMIO(RTC1_BASE, RTC_OFFSET_TASKS_STOP) = 1;  // Disable RTC1 to prevent Arduino tick functionality
      const auto oldBasepri = __get_BASEPRI();
      __set_BASEPRI(6 << (8 - __NVIC_PRIO_BITS));  // Set BASEPRI to mask RTC0 interrupt
      MMIO(RTC0_BASE, RTC_OFFSET_INTENSET) = 1 << 16;  // Enable RTC0 interrupt

      __WFE();  // Wait for event (CPU enters low-power state)

      // Disable RTC0 interrupt after wake-up to prevent it from being handled
      // (arduino implements the RTC0 handler with a trap, and we need to avoid
      // getting execution stuck in that trap)
      MMIO(RTC0_BASE, RTC_OFFSET_INTENCLR) = 1 << 16;  // Disable RTC0 interrupt
      NVIC_ClearPendingIRQ(RTC0_IRQn);  // Clear RTC0 interrupt pending flag
      __set_BASEPRI(oldBasepri);  // Restore BASEPRI

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
  // initialize with the comparator triggered to generate a heartbeat immediately after power on
  MMIO(RTC0_BASE, RTC_OFFSET_EVENTS_COMPARE0) = 1;
  MMIO(RTC0_BASE, RTC_OFFSET_PRESCALER) = prescale;
  MMIO(RTC0_BASE, RTC_OFFSET_CC0) = scaledTicks;
  MMIO(RTC0_BASE, RTC_OFFSET_EVTEN) = 1 << 16;  // enable compare to CC0, disable others

  // set interrupt priority to avoid actually triggering the interrupt handler
  // (Arduino makes it very hard to define your own interrupt handlers, so instead
  // we use a BASEPRIO high enough that the interrupt never fires, leverage the
  // pending interrupt to trigger a wake from WFE, and simply clear the pending
  // interrupt to allow it to trigger again)
  NVIC_SetPriority(RTC0_IRQn, 7);
  //__set_BASEPRI(6 << (8 - __NVIC_PRIO_BITS)); // only do this when sleeping
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
  pinMode(PIN_BUTTON, INPUT);
  button_state = LOW; // indicates the probe was last noticed untriggered
  lastButtonSendTime = 0;

  // Attach button interrupt for rising edge (button press)
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), GPIO_Handler, RISING);

#ifdef SerialDebug
  Serial.begin(115200);
  delay(1000);
  //while(!Serial) delay(10); //remove for battery only operation
  Serial.println("nRF52840 Carvera Wireless Probe Core Features Example");
  Serial.println("--------------------------------\n");
#endif
  //setup battery
  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, INPUT); // low power mode when not in use
  pinMode(PIN_HICHG, OUTPUT);
  pinMode(PIN_CHG, INPUT);

  digitalWrite(PIN_HICHG, LOW);        // charge current 100mA

  // initialise ADC wiring_analog_nRF52.c:73
  analogReference(AR_DEFAULT);  // default 0.6V*6=3.6V  wiring_analog_nRF52.c:73
  analogReadResolution(16);     // wiring_analog_nRF52.c:39

  read_current_flash_vars();
  initHeartbeatTimer();

  // init radio and hf clock. FWIW I haven't checked this, but I think the primary
  // purpose of this line is to make the radio power (default on) match the HF clock
  // (default off), as expected in the rest of the code. This line happens to do that,
  // as well as (unimportantly) preparing the radio to send
  set_radio_mode(SEND);

#ifdef ConstantPair
  probe_mode_c = PAIR;
#endif
#ifdef StartBootMode
  probe_mode_c = UPDATE;
#endif
}

// the loop function runs over and over again forever
ProbeMode lastProbeMode = SLEEP;
void loop() {

  switch (probe_mode_c) {

    case UNDEF: break;

    case INIT: break;

    case IDLE: idleCycle(); break;

    case PAIR: pairCycle(); break;

    case PROBE: probeCycle(probe_mode_c != lastProbeMode); break;

    case LASER: laserCycle(); break;

    case TEST: testCycle(); break;

    case UPDATE: updateCycle(); break;

    case SLEEP: offCycle(); break;

    default: break;
  }

  lastProbeMode = probe_mode_c;
}
