/*
  
*/
#include "NRF52840_HexToName.h"
#include <Arduino.h>
//#include <Adafruit_LittleFS.h>
//#include <InternalFileSystem.h>
#include <bluefruit.h>


//using namespace Adafruit_LittleFS_Namespace;

#define PIN_VBAT        (32)  // D32 battery voltage
#define PIN_VBAT_ENABLE (14)  // D14 LOW:read anable
#define PIN_HICHG       (22)  // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG         (23)  // D23 charge indicatore LOW:charge HIGH:no charge
#define PIN_BUTTON      2     //D2 normally closed button for probing. 
#define PIN_WAKEUP      2
#define PIN_BOOTJUMPER  23
#define PIN_OTHERJUMPER 1
//#define StartBootMode //uncomment to start in bootmode

enum ProbeMode { UNDEF, INIT, IDLE, SLEEP, PAIR, PROBE, LASER, UPDATE, TEST}; //laser mode is wrapped inside pairing mode. 
enum RadioMode  {OFF,SEND,RECIEVE};

ProbeMode probe_mode_c = PROBE;
bool uninitialized = true;
//button configuration variables
int polling_rate = 100; //ms between tests
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
unsigned long buttonLongPressLength = 10000; //how long you need to hold the button down before it enters pairing mode
unsigned long buttonDoublePressTime = 500; //how long between button presses to register a double press
unsigned long buttonHeartbeatUnpressed = 1000; //how long between sending button released events in active probe mode

//mode duration setup
unsigned long pairingDelay = 15000; //how long to wait in pairing mode before quitting back to probe mode
unsigned long laserDelay = 15000; //how long to wait in laser mode before quitting back to probe mode
unsigned long idle_Delay = 20000; //how long to wait until the probe goes into idle mode
unsigned long sleeping_delay = 100000000;//1000000; //how long until the probe goes into a deep sleep mode
unsigned long idle_heartbeat_delay = 5000;//how often to send heartbeat probe updates when in idle mode 

//communication configuration
uint8_t pan[2] = {0x22,0x20};
uint8_t destination[2] = {0xea,0x0b};
uint8_t source[2] = {0xea,0x0b};
const long ack_interval = 200;  // how long to wait for ACK packets
int channel = 25;
int channel_rx = 25;


//////////////////////////////////////////////////////////////////////////////////////////////

//communication setup
uint8_t packet[127] = PACKET_PAIR;
uint8_t seq = 1; // should probably sort this out at some point
uint8_t ack_pkt[127] = {0x05, 0x02, 0x00, 0x02};
//pairing wait
unsigned long previousMillis = 0;  // will store last time LED was updated need to consolodate with primary wait times
unsigned long currentMillis;
// constants won't change:

//button setup variables
int button_state;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
bool button_held = false;
bool single_press = false;
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

void disable_radio(){
  
  //NRF_RADIO->SHORTS = 0;
  MMIO(RADIO_BASE, RADIO_OFFSET_SHORTS) = 0;
  MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_DISABLED) = 0;
  MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_DISABLE) = 1; //

  while (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_DISABLED) == 0)
  {
      // Do nothing.
  }
  MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_DISABLED) = 0;
}

void set_radio_mode(RadioMode radio_mode)
{
  disable_radio();
  switch (radio_mode)
  {
    case OFF: 
      MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_DISABLED) = 0; //turn off events on the radio. 0 is off, 1 allows events
      MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_DISABLE) = 1; // 
      while (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_DISABLED) == 0) {} // wait to complete
    break;

    case SEND:
      MMIO(RADIO_BASE, RADIO_OFFSET_FREQUENCY) = channel*5-50;
      MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_DISABLED) = 0; //turn off events on the radio. 0 is off, 1 allows events
      MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_DISABLE) = 1; // turn off tasks
      while (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_DISABLED) == 0) {} // wait to complete
      setup_radio_TXRX();
      MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_TXEN) = 1;
      while (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_READY) == 0) {} // wait to complete
      break;

    case RECIEVE: 
      MMIO(RADIO_BASE, RADIO_OFFSET_FREQUENCY) = channel_rx*5-50;
      MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_DISABLED) = 0; //turn off events on the radio. 0 is off, 1 allows events
      MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_DISABLE) = 1; // turn off tasks
      while (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_DISABLED) == 0) {} // wait to complete
      setup_radio_TXRX();
      MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_RXEN) = 1;
      while (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_READY) == 0) {} // wait to complete
      break;

    default: break;
  }
}

void set_channel() //needs to shut down and restart the radio between changing channels
{
  //to make this accessable in other areas, need to turn off radio, change channel, and then turn on radio
  //MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_DISABLE) = 0;
  //while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_DISABLED){}
  
  MMIO(RADIO_BASE, RADIO_OFFSET_FREQUENCY) = channel*5-50; // 2475MHz = channel 25 page 321 of nRF52840_PS_v1.8
  //MMIO(RADIO_BASE, RADIO_OFFSET_FREQUENCY) = 75; // 2475MHz = channel 25
  //MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_DISABLE) = 0;


}

void setup_radio_TXRX() // all the one time setup for the 802.15.4 radio
{
  // radio power on
	MMIO(RADIO_BASE, RADIO_OFFSET_POWER) = 1;
  MMIO(RADIO_BASE, RADIO_OFFSET_TXPOWER) = 2; //set radio power to 1-8
  // hf clock on
	MMIO(CLOCK_BASE, CLOCK_OFFSET_EVENTS_HFCLKSTARTED) = 0; // clear event
	MMIO(CLOCK_BASE, CLOCK_OFFSET_TASKS_HFCLKSTART) = 1; // enable
	while (!MMIO(CLOCK_BASE, CLOCK_OFFSET_EVENTS_HFCLKSTARTED)) {} // wait for event
	MMIO(CLOCK_BASE, CLOCK_OFFSET_EVENTS_HFCLKSTARTED) = 0; // clear event
  // misc radio configuration
	MMIO(RADIO_BASE, RADIO_OFFSET_CRCCNF) = 0x202;
	MMIO(RADIO_BASE, RADIO_OFFSET_CRCPOLY) = 0x11021;
	MMIO(RADIO_BASE, RADIO_OFFSET_CRCINIT) = 0;

	// disable radio shorts, radio interrupts, and PPI
	MMIO(RADIO_BASE, RADIO_OFFSET_SHORTS) = 0;
	MMIO(RADIO_BASE, RADIO_OFFSET_INTENCLR) = 0xffffffff;
	MMIO(0x4001F000, 0x500) = 0; // PPI CHEN = 0

	//MMIO(RADIO_BASE, RADIO_OFFSET_CCACTRL) = 0x052D0000 | 2; // default thresholds; require energy level and carrier pattern to detect busy
	MMIO(RADIO_BASE, RADIO_OFFSET_CCACTRL) = 0x022D2D00; // default thresholds; require energy level and carrier pattern to detect busy
  //set_channel();
  MMIO(RADIO_BASE, RADIO_OFFSET_MODE) = 15; // ieee 802.15.4
  MMIO(RADIO_BASE, RADIO_OFFSET_PCNF0) = 8 | (2 << 24) | (1 << 26);
	MMIO(RADIO_BASE, RADIO_OFFSET_PCNF1) = 127;
	// MMIO(RADIO_BASE, 0x650) = 0x201; // MODECNF0

}

void build_packet(uint8_t status = 0x01, uint8_t batteryB = 0x4c, uint8_t batteryA = 0x10, bool reset_seq = false)
{
  if (reset_seq){
    seq = 0x00;
  }
  //packet lenght
  packet[0]=14;  
  //source addressing mode
  packet[1] = 0x61;
  packet[2] = 0x88;
  //sequence number
  packet[3] = seq++;
  //pan
  packet[4] = pan[0];
  packet[5] = pan[1];
  //destination
  packet[6] = destination[0];
  packet[7] = destination[1];
  //source
  packet[8] = source[0];
  packet[9] = source[1];
  //data payload
  if (status == 3){
    packet[10] = status;
    //destination
    packet[6] = 0xff;
    packet[7] = 0xff;
  }
  else{
    packet[10] = status;
  }
  //battery status
  packet[11] = batteryB;
  packet[12] = batteryA;


}

void send_packet(uint8_t pkt[127]) {
	// block waiting for a stable radio state
	uint32_t state = -1;


	// if the state is not rx idle, perform rx ramp up and wait
	if (state == RADIO_STATE_RXIDLE) {
		MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_STOP) = 1;
	}
	state = MMIO(RADIO_BASE, RADIO_OFFSET_STATE);
	if (state != RADIO_STATE_TXIDLE) { 
		MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_RXEN) = 0;
    MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_TXEN) = 1;
	}
	while (state != RADIO_STATE_TXIDLE) {
		state = MMIO(RADIO_BASE, RADIO_OFFSET_STATE);
	}


	// Send packet
	volatile uint32_t *radio_state = &MMIO(RADIO_BASE, RADIO_OFFSET_STATE);
	
	//uint8_t pkt[127] = datapacket;
	MMIO(RADIO_BASE, RADIO_OFFSET_PACKETPTR) = (uint32_t)&pkt[0];
	//MMIO(RADIO_BASE, RADIO_OFFSET_SHORTS) = (0
	//		//| 1 << 1  /*end -> disable */
	//		| 1 << 11 /*rxready -> cca start*/
	//		| 1 << 12 /*cca idle -> txen*/
	//		| 1 << 13 /*cca_busy -> disable*/
	//		//| 1 << 17 /*cca idle -> stop*/
	//		| 1 << 18 /*txready -> start*/
	//		);
	//MMIO(RADIO_BASE, RADIO_OFFSET_SHORTS) = (0
	//      | 1 << 19  /*rxready -> start*/
	//      | 1 << 1   /*end -> disable*/
	//    );
	uint32_t last_state = *radio_state;
	uint32_t states[30] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	int i = 0;
	states[i++] = last_state;

	MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_TXEN) = 1;
	MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_START) = 1;
	
  while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_TXIDLE) {} //wait for complete
 

}

void send_ack(){
  	// block waiting for a stable radio state
	uint32_t state = -1;


	// if the state is not rx idle, perform rx ramp up and wait
	if (state == RADIO_STATE_RXIDLE) {
		MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_STOP) = 1;
	}
	state = MMIO(RADIO_BASE, RADIO_OFFSET_STATE);
	if (state != RADIO_STATE_TXIDLE) { 
		MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_RXEN) = 0;
    MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_TXEN) = 1;
	}
	while (state != RADIO_STATE_TXIDLE) {
		state = MMIO(RADIO_BASE, RADIO_OFFSET_STATE);
	}


	// Send packet
	volatile uint32_t *radio_state = &MMIO(RADIO_BASE, RADIO_OFFSET_STATE);
	
	//uint8_t pkt[127] = datapacket;
	MMIO(RADIO_BASE, RADIO_OFFSET_PACKETPTR) = (uint32_t)&ack_pkt[0];
	//MMIO(RADIO_BASE, RADIO_OFFSET_SHORTS) = (0
	//		//| 1 << 1  /*end -> disable */
	//		| 1 << 11 /*rxready -> cca start*/
	//		| 1 << 12 /*cca idle -> txen*/
	//		| 1 << 13 /*cca_busy -> disable*/
	//		//| 1 << 17 /*cca idle -> stop*/
	//		| 1 << 18 /*txready -> start*/
	//		);
	//MMIO(RADIO_BASE, RADIO_OFFSET_SHORTS) = (0
	//      | 1 << 19  /*rxready -> start*/
	//      | 1 << 1   /*end -> disable*/
	//    );
	uint32_t last_state = *radio_state;
	uint32_t states[30] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	int i = 0;
	states[i++] = last_state;

	MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_TXEN) = 1;
	MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_START) = 1;
	
  while (MMIO(RADIO_BASE, RADIO_OFFSET_STATE) != RADIO_STATE_TXIDLE) {} //wait for complete
 
}

int receive_test() { //return 0 for no packet, return 1 for a proper packet, return 2 for ack packet
	// radio power on
	MMIO(RADIO_BASE, RADIO_OFFSET_POWER) = 1;

	// hf clock on
	MMIO(CLOCK_BASE, CLOCK_OFFSET_EVENTS_HFCLKSTARTED) = 0; // clear event
	MMIO(CLOCK_BASE, CLOCK_OFFSET_TASKS_HFCLKSTART) = 1; // enable
	while (!MMIO(CLOCK_BASE, CLOCK_OFFSET_EVENTS_HFCLKSTARTED)) {} // wait for event
	MMIO(CLOCK_BASE, CLOCK_OFFSET_EVENTS_HFCLKSTARTED) = 0; // clear event

	// misc radio configuration
	MMIO(RADIO_BASE, RADIO_OFFSET_CRCCNF) = 0x202;
	MMIO(RADIO_BASE, RADIO_OFFSET_CRCPOLY) = 0x11021;
	MMIO(RADIO_BASE, RADIO_OFFSET_CRCINIT) = 0;

	//MMIO(RADIO_BASE, RADIO_OFFSET_CCACTRL) = 0x052D0000 | 2; // default thresholds; require energy level and carrier pattern to detect busy
	MMIO(RADIO_BASE, RADIO_OFFSET_CCACTRL) = 0x022D2D00; // default thresholds; require energy level and carrier pattern to detect busy

	MMIO(RADIO_BASE, RADIO_OFFSET_MODE) = 15; // ieee 802.15.4
	MMIO(RADIO_BASE, RADIO_OFFSET_FREQUENCY) = 75; // 2475MHz = channel 25
	MMIO(RADIO_BASE, RADIO_OFFSET_PCNF0) = 8 | (2 << 24)| (1 << 26);
	MMIO(RADIO_BASE, RADIO_OFFSET_PCNF1) = 127;
	// MMIO(RADIO_BASE, 0x650) = 0x201; // MODECNF0

	// block waiting for a stable radio state
	uint32_t state = -1;
	while (state != RADIO_STATE_DISABLED && state != RADIO_STATE_RXIDLE && state != RADIO_STATE_TXIDLE && state != RADIO_STATE_RX) {
    //Serial.print("state trap 01");
		
    state = MMIO(RADIO_BASE, RADIO_OFFSET_STATE);
	}

	// if the state is not rx idle, perform rx ramp up and wait
	if (state == RADIO_STATE_RX) {
    
		MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_STOP) = 1;
	}
	state = MMIO(RADIO_BASE, RADIO_OFFSET_STATE);
	if (state != RADIO_STATE_RXIDLE) {
		MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_RXEN) = 1;
	}
	while (state != RADIO_STATE_RXIDLE) {
    //Serial.print("state trap 02");
		state = MMIO(RADIO_BASE, RADIO_OFFSET_STATE);
	}

	// Receive packet
	volatile uint32_t *radio_state = &MMIO(RADIO_BASE, RADIO_OFFSET_STATE);
	uint8_t pkt[127];
  for (int i=0; i<127;i++) {
    pkt[i] = 0xcc;
  }
	MMIO(RADIO_BASE, RADIO_OFFSET_PACKETPTR) = (uint32_t)&pkt[0];
  MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_END) = 0;
  MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCOK) = 0;
  MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCERROR) = 0;

	uint32_t last_state = *radio_state;
	uint32_t states[30] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	int i = 0;
	states[i++] = last_state;
  i = i%30;
	MMIO(RADIO_BASE, RADIO_OFFSET_TASKS_START) = 1;
  previousMillis = millis();
  while(MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_END) == 0) { //wait until it recieves something
    currentMillis = millis();
    if (currentMillis - previousMillis >= ack_interval) { //escape loop if taking too long
      return 0;
    }
    
  }

  
  
  //ack
  if (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCOK)) { //check for CRC ok?
    if (pkt[0]==0x5){//} && pkt[4]==0x22 && pkt[5]==0x20) {
      // length, PAN ok
      //Serial.print(" packet ack ");
      return 2;
    }
  }

  //packet
  if (MMIO(RADIO_BASE, RADIO_OFFSET_EVENTS_CRCOK)) { //check for CRC ok?
    if (pkt[0]==0xe && pkt[4]==0x22 && pkt[5]==0x20) {
      // length, PAN ok
      Serial.print(" length pan ok ");
      //Serial.print(pkt[10]);
      ack_pkt[4] = pkt[3];
      
      if (pkt[10] == 0x03){
        Serial.println(" confirm packet ");
        source[0] = pkt[11];
        source[1] = pkt[12];
        destination[0] = pkt[11];
        destination[1] = pkt[12];
        return 1;
      } 
    }
    
  }

  for (int j = 0; j<pkt[0]; j++){
    ack_pkt[j] = pkt[j];
    Serial.print(" ");
    Serial.print(ack_pkt[j]);
  }
  Serial.println();
  
	//while(1) {}
  return 0;
}

/*
void gotoSleep(unsigned long time) // depreciated
{
  // shutdown when time reaches SLEEPING_DELAY ms
  if ((time>sleeping_delay + lastDebounceTime))
  {
    // to reduce power consumption when sleeping, turn off all your LEDs (and other power hungry devices)
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);  
    digitalWrite(LED_BLUE, HIGH);                       

    // setup your wake-up pins.
    pinMode(PIN_WAKEUP,  INPUT_PULLUP_SENSE);    // this pin (WAKE_LOW_PIN) is pulled up and wakes up the feather when externally connected to ground.
    //pinMode(WAKE_HIGH_PIN, INPUT_PULLDOWN_SENSE);  // this pin (WAKE_HIGH_PIN) is pulled down and wakes up the feather when externally connected to 3.3v.
 
    // power down nrf52.
    
    sd_power_system_off();                              // this function puts the whole nRF52 to deep sleep (no Bluetooth).  If no sense pins are setup (or other hardware interrupts), the nrf52 will not wake up.
  } 
}
*/

/*
void writeConfig()
{
  InternalFS.begin();
  file.open(FILENAME, FILE_O_READ);


  //String config_contents;
  //config_contents = "reset:";// + String(uninitialized) + ",pan:" + String(pan) + ",channel:" + String(com_channel)   + ",src_short_addr:" + String(src_short_addr)   + ",dst_short_addr:" + String(dst_short_addr)  + ",polling_rate:" + String(polling_rate);


  // file existed
  if ( file )
  {
    Serial.println(FILENAME " file exists");
    
    uint32_t readlen;
    char buffer[64] = { 0 };
    readlen = file.read(buffer, sizeof(buffer));

    buffer[readlen] = 0;
    Serial.println(buffer);
    file.close();
  }else
  {
    Serial.print("Open " FILENAME " file to write ... ");

    if( file.open(FILENAME, FILE_O_WRITE) )
    {
      Serial.println("OK");
      
      file.write(CONTENTS, strlen(CONTENTS));
      file.close();
    }else
    {
      Serial.println("Failed!");
    }
  }
  InternalFS.end();
}

void readConfig()
{
  InternalFS.begin();
  file.open(FILENAME, FILE_O_READ);


  //String config_contents = "reset:" + String(uninitialized) + ",pan:" + String(pan);
  //+ ",channel:" + String(com_channel)
  //+ ",src_short_addr:" + String(src_short_addr)
  //+ ",dst_short_addr:" + String(dst_short_addr)
  //+ ",polling_rate:" + String(polling_rate);



  // file existed
  if ( file )
  {
    Serial.println(FILENAME " file exists");
    
    uint32_t readlen;
    char buffer[64] = { 0 };
    readlen = file.read(buffer, sizeof(buffer));

    buffer[readlen] = 0;
    Serial.println(buffer);
    file.close();
  }else
  {
    Serial.print("Open " FILENAME " file to write ... ");

    if( file.open(FILENAME, FILE_O_WRITE) )
    {
      Serial.println("OK");
      
      file.write(CONTENTS, strlen(CONTENTS));
      file.close();
    }else
    {
      Serial.println("Failed!");
    }
  }
  InternalFS.end();
}
*/


void sendbutonpress(int state)
{
  vbatt = analogRead(PIN_VBAT);
  vbatt = 1000*0.1856 * 3.6 * vbatt / 4096;
  String out_str;
  out_str = String(vbatt, HEX) + "    " + String(vbatt) + "    " + String(digitalRead(PIN_CHG)) ; //2.961 for 12 bit
  Serial.println(out_str);

  set_radio_mode(SEND);
  if (state){
    digitalWrite(LED_RED, HIGH); 
    digitalWrite(LED_GREEN, LOW); 
    Serial.println("button pressed");
    build_packet(0x01, vbatt & 0xff, vbatt >> 8);
    send_packet(packet);
  }
  else{
    digitalWrite(LED_RED, LOW);   // turn the LED off by making the voltage HIGH
    digitalWrite(LED_GREEN, HIGH);  // turn the LED on (LOW is the voltage level)
    Serial.println("button released");
    build_packet(0x00, vbatt & 0xff, vbatt >> 8);
    send_packet(packet);
    
  }
}

void probeCycle()
{

  currentMillis = millis();

  int reading = ! digitalRead(PIN_BUTTON);
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:


  if (reading == HIGH && button_state == LOW)
  {
    lastDebounceTime = currentMillis; //onTime - lastDebounceTime
  }

  //held
  if (reading == HIGH && button_state == HIGH)
  {
    if ((currentMillis-lastDebounceTime)>buttonLongPressLength)
    {
      Serial.println("pairing");
      probe_mode_c = PAIR;
      button_held = true;
      // shut the laser off
      laserOn = false;
    }
  }

  //released
  if (reading == LOW && button_state == HIGH)
  {
    if ((currentMillis-lastDebounceTime) > debounceDelay && !button_held)
    {
      //button has been released test for double press
      if ((currentMillis - lastSwitchTime) >= buttonDoublePressTime) {
        single_press = 1;
        lastSwitchTime = currentMillis;
        
      } else if ((currentMillis - lastSwitchTime) < buttonDoublePressTime) {
        laserOn = !laserOn;
        Serial.println("double press");
        single_press = 0;
        lastSwitchTime = currentMillis;
      }  
    }
  }

  button_state = reading;

  if ((single_press == true) && ((currentMillis -lastDebounceTime) > (buttonDoublePressTime)))
  {
    single_press = false;
    Serial.println("single Press");
  }


  if (button_state == LOW && ((currentMillis - buttonHeartbeatUnpressedTime) < buttonHeartbeatUnpressed))
  {
    //only send button presses as low on a certain interval
    buttonHeartbeatUnpressedTime = currentMillis;
  }
  else
  {
    sendbutonpress(button_state);
      //read battery 
    

  
  }

  //laser cycle
  if(laserOn)
  {
    Serial.println("Im a frickin laser. Pew pew");
    

    if (currentMillis - blinkMillis >= 1000) {
      // save the last time you blinked the LED
      blinkMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      if (blinkState == LOW) {
        blinkState = HIGH;
      } else {
        blinkState = LOW;
      }
     
    }
    if ((currentMillis - lastDebounceTime) > laserDelay)
    {
      // shut the laser off
      laserOn = false;
      blinkState = HIGH;
    }
      
    digitalWrite(LED_BLUE, blinkState);
  } else
  {
    digitalWrite(LED_BLUE, HIGH);
  }

  delay(polling_rate);   


  //test for idle
  testforIdle();

}

void pairCycle()
{
  unsigned long currentMillis = millis();

  set_radio_mode(SEND);
  build_packet(0x03, 0xea, 0x0b);
  send_packet(packet);
  set_radio_mode(RECIEVE);
  int result = receive_test();
  if (result == 2){ //ack recieved
    //Serial.println("ack recieved"); 

    delay_ns(100);   
  }

  if (result == 1){ //test for pairing confirmation from machine
    Serial.println("pair success");
    //delay(4);
    set_radio_mode(SEND);
    send_ack();
    delay(2000);
    build_packet(0x06, 0xea, 0x0b, true); //send confirm packet to machine
    set_radio_mode(RECIEVE);
    while (receive_test() != 2 && (currentMillis - lastDebounceTime) > pairingDelay)
    {
      Serial.print("trap");
      set_radio_mode(SEND);
      build_packet(0x06, 0xea, 0x0b);
      send_packet(packet);
      delay(4000);
      currentMillis = millis();
    }
    Serial.println("pairing complete"); 
    probe_mode_c = PROBE;
  
  }
  result = receive_test();
  if (result == 1){ //test for pairing confirmation from machine
    Serial.println("pair success");
    //delay(4);
    set_radio_mode(SEND);
    send_ack();
    delay(2000);



    build_packet(0x06, 0xea, 0x0b, true); //send confirm packet to machine
    set_radio_mode(RECIEVE);
    //send packet to channel 26???
    while (receive_test() != 2 && (currentMillis - lastDebounceTime) > pairingDelay) 
    {
      Serial.print("trap");
      set_radio_mode(SEND);
      build_packet(0x06, 0xea, 0x0b);
      send_packet(packet);
      delay(4000);
      currentMillis = millis();
    }
    Serial.println("pairing complete"); 
    probe_mode_c = PROBE;
  }
  delay(50);

  if ((currentMillis - lastDebounceTime) > pairingDelay)// || digitalRead(PIN_BUTTON)) 
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

    digitalWrite(LED_RED, blinkState); 
    digitalWrite(LED_GREEN, blinkState); 
  }






}

void laserCycle() //this does nothing right now. It is here in case we want to implement a standalone laser mode without the probing functions
{
  Serial.println("Im a frickin laser. Pew pew");
  unsigned long currentMillis = millis();
  
  //sendpacket("pairing");
  if ((currentMillis - lastDebounceTime) > laserDelay)// || digitalRead(PIN_BUTTON)) 
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

    digitalWrite(LED_BLUE, blinkState); 
    digitalWrite(LED_GREEN, blinkState); 
  }
}

void updateCycle()
{
}

void testCycle()
{
  bool pair = true;
  //Serial.print("n");
  if (pair)//!digitalRead(2))
  {
    set_radio_mode(SEND);
    build_packet(0x03, 0xea, 0x0b);
    send_packet(packet);
    set_radio_mode(RECIEVE);
    int result = receive_test();
    if (result == 2){ //ack recieved
      //Serial.println("ack recieved"); 

      delay_ns(100);   
    }

    if (result == 1){ //test for pairing confirmation from machine
      Serial.println("pair success");
      //delay(4);
      set_radio_mode(SEND);
      send_ack();
      delay(2000);
      build_packet(0x06, 0xea, 0x0b, true); //send confirm packet to machine
      set_radio_mode(RECIEVE);
      while (receive_test() != 2)
      {
        set_radio_mode(SEND);
        build_packet(0x06, 0xea, 0x0b);
        send_packet(packet);
        delay(400000);
      }
      Serial.println("pairing complete"); 
      while(1);
    
    }
    result = receive_test();
    if (result == 1){ //test for pairing confirmation from machine
      Serial.println("pair success");
      //delay(4);
      set_radio_mode(SEND);
      send_ack();
      delay(2000);



      build_packet(0x06, 0xea, 0x0b, true); //send confirm packet to machine
      set_radio_mode(RECIEVE);
      //send packet to channel 26???
      while (receive_test() != 2)
      {
        set_radio_mode(SEND);
        build_packet(0x06, 0xea, 0x0b);
        send_packet(packet);
        delay(400000);
      }
      Serial.println("pairing complete"); 
      while(1);
    }
    delay(50);

    

  }else if (!pair){
    //digitalWrite(LED_RED, LOW);
    
    
    build_packet(0x01, 0x99, 0x11);
    send_packet(packet);
    //receive_test();
    delay(6);
  }

}

void offCycle()
{
  Serial.println("off");
  Bluefruit.begin(); 
  // setup your wake-up pins.
  pinMode(PIN_WAKEUP,  INPUT_PULLUP_SENSE);    // this pin (WAKE_LOW_PIN) is pulled up and wakes up the feather when externally connected to ground.
  //pinMode(WAKE_HIGH_PIN, INPUT_PULLDOWN_SENSE);  // this pin (WAKE_HIGH_PIN) is pulled down and wakes up the feather when externally connected to 3.3v.

  // power down nrf52.
  
  sd_power_system_off();                              // this function puts the whole nRF52 to deep sleep (no Bluetooth).  If no sense pins are setup (or other hardware interrupts), the nrf52 will not wake up.
}

void testforIdle()
{
// shutdown when time reaches SLEEPING_DELAY ms
  if ((currentMillis>idle_Delay + lastDebounceTime))
  {
    // to reduce power consumption when sleeping, turn off all your LEDs (and other power hungry devices)
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);  
    digitalWrite(LED_BLUE, HIGH);                       

    probe_mode_c = IDLE;

  } 
}

void idleCycle()
{
  idleHeartbeatUnpressedTime = currentMillis;
  Serial.println("idle");
  while(digitalRead(PIN_BUTTON)){
    
    currentMillis = millis();
    
    if ((currentMillis>sleeping_delay + lastDebounceTime))
    {
      // to reduce power consumption when sleeping, turn off all your LEDs (and other power hungry devices)
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, HIGH);  
      digitalWrite(LED_BLUE, HIGH);                       
      set_radio_mode(OFF); //shut radio off
      probe_mode_c = SLEEP;
      return;
    } 
    if ((currentMillis - idleHeartbeatUnpressedTime >idle_heartbeat_delay)) {
      //send heartbeat to machine
      idleHeartbeatUnpressedTime = currentMillis;

      vbatt = analogRead(PIN_VBAT); //convert to function later
      vbatt = 1000*0.1856 * 3.6 * vbatt / 4096; //2.961 for 12 bit. Exact value to be determined via voltage logging later. 
      String out_str;
      out_str = String(vbatt, HEX) + "    " + String(vbatt) + "    " + String(digitalRead(PIN_CHG)) ; 
      Serial.println(out_str);
      build_packet(0x00, vbatt & 0xff, vbatt >> 8);


      Serial.println("idle heartbeat");
      set_radio_mode(SEND);
      build_packet(0x00, 0x99, 0x11);
      send_packet(packet);
      disable_radio();
    }



  }
  lastDebounceTime = millis();
  probe_mode_c = PROBE;

  
}

// the setup function runs once when you press reset or power the board
void setup() {
  //Bluefruit.begin();          // Sleep functions need the softdevice to be active.

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  //initialize digital pin for button as Input
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  

  Serial.begin(115200);
  delay(1000);
  //while(!Serial) delay(10);
  Serial.println("nRF52840 Carvera Wireless Probe Core Features Example");
  Serial.println("--------------------------------\n");

  //setup battery
  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  pinMode(PIN_HICHG, OUTPUT);
  pinMode(PIN_CHG, INPUT);
  

  digitalWrite(PIN_VBAT_ENABLE, LOW); // VBAT read enable
  digitalWrite(PIN_HICHG, LOW);       // charge current 100mA
  
  // initialise ADC wireing_analog_nRF52.c:73
  analogReference(AR_DEFAULT);        // default 0.6V*6=3.6V  wireing_analog_nRF52.c:73
  analogReadResolution(16);           // wireing_analog_nRF52.c:39

  //filesystem setup
  set_radio_mode(SEND);

  //readConfig();
}

// the loop function runs over and over again forever
void loop() {
  
  switch (probe_mode_c)
  {
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
