#include "NRF52840_HexToName.h"
#include <stdint.h>
#include <Arduino.h>
#include <bluefruit.h>


uint8_t packet[127] = PACKET_PAIR;
uint8_t seq = 1; // should probably sort this out at some point

uint8_t pan[2] = {0x22,0x20};
uint8_t destination[2] = {0xea,0x0b};
uint8_t source[2] = {0xea,0x0b};
int channel = 25;
int channel_rx = 25;

enum RadioMode  {OFF,SEND,RECIEVE};
uint8_t ack_pkt[127] = {0x05, 0x02, 0x00, 0x02};

//pairing wait
unsigned long previousMillis = 0;  // will store last time LED was updated
unsigned long currentMillis;
// constants won't change:
const long ack_interval = 200;  // how long to wait for ACK packets


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

//{14, 0x61, 0x88, seq++, 0x22, 0x20, 0xff, 0xff, 0x4c, 0x10, 3, 0x4c, 0x10}
//{14, 0x61,0x88,seq++, PANB,PANA, destAddrB,destAddrA,srcAddB,SrcAddA,dataA,batB,BatA}
//dataA: 0 probe not hit, 1 probe hit, 3 pairing, 
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

void setup(){
  Serial.begin(115200);
  while(!Serial) delay(10); // remove for battery operation
  Serial.println("nRF52840 Carvera Wireless Probe Core Features Example");
  Serial.println("--------------------------------\n");
  pinMode(LED_RED, OUTPUT);
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);


  
  set_radio_mode(SEND);
  build_packet(0x03, 0xea, 0x0b);
  send_packet(packet);
  send_ack();
  set_radio_mode(RECIEVE);
  

  uint8_t packet_out[127] = PACKET_PROBE;


}

void loop(){
  uint8_t packet_out[127] = PACKET_PAIR;
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
  

  //send_packet();
  //delay(5);
  //digitalWrite(LED_RED, HIGH);
  //send_packet();
  //delay(5);



}