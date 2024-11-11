// Flash memory
// 1024kb, from 0x0 to 0x100000. Code is written to the bottom of that space, so perhaps I'll write data at the top
// organized into 4kb pages
// Erase one page at a time, resetting it to all ones
// Write one word at a time, only clearing bits. Max of 2 writes before erasing and rewriting is required
// 41us to write a word
// 85ms to erase a page
// Registers:
// Base: 0x4001E000 NVMC NVMC Non-volatile memory controller
// READY 0x400 Ready flag: 0=ongoing write/erase operation; 1=ready
// READYNEXT 0x408 Ready flag 0=cannot write now; 1=can enqueue write
// CONFIG 0x504 Configuration register 0=read only, 1=write enabled 2=erase enabled
// ERASEPAGE 0x508 Register for erasing a page in code area: write address of first word in the page
#define NVMC_BASE 0x4001E000
#define NVMC_READY 0x400
#define NVMC_READYNEXT 0x408
#define NVMC_CONFIG 0x504
#define NVMC_ERASEPAGE 0x508



// 0x40000000 CLOCK CLOCK Clock control
// offsets: write 1 to start tasks
// TASKS_HFCLKSTART 0x000 Start HFXO crystal oscillator
// TASKS_HFCLKSTOP 0x004 Stop HFXO crystal oscillator
// TASKS_LFCLKSTART 0x008 Start LFCLK
// TASKS_LFCLKSTOP 0x00C Stop LFCLK
// LFCKSRC 0x518
//   0 -> LFRC oscillator
// LFRCMODE 0x5B4 LFRC mode configuration. Write 0 for normal, 1 for ultra low power (lower precision)
#define CLOCK_BASE 0x40000000 
#define CLOCK_OFFSET_TASKS_HFCLKSTART 0x000 
#define CLOCK_OFFSET_TASKS_HFCLKSTOP 0x004 
#define CLOCK_OFFSET_TASKS_LFCLKSTART 0x008 
#define CLOCK_OFFSET_TASKS_LFCLKSTOP 0x00C 
#define CLOCK_OFFSET_EVENTS_HFCLKSTARTED 0x100 
#define CLOCK_OFFSET_LFCKSRC 0x518
#define CLOCK_OFFSET_LFRCMODE 0x5B4 

// HF Clocks
// HCLK64M: 64 MHz CPU clock
// PCLK1M: 1 MHz peripheral clock
// PCLK16M: 16 MHz peripheral clock
// PCLK32M: 32 MHz peripheral clock
// The HFXO must be running to use the RADIO. Note that when HF clock starts it will be HFINT (internal) until the external HFXO is started

// LF Clocks
// Always switched off when in system off mode
// 32.768 kHz RC oscillator (LFRC)
// 32.768 kHz crystal oscillator (LFXO)
// 32.768 kHz synthesized from HFCLK (LFSYNT)

// Power Management
// ION_RAMOFF_RTC System ON, no RAM retention, wake on RTC (running from LFRC clock) 1.50 µA
// ION_RAMOFF_EVENTSystem ON, no RAM retention, wake on any event 0.97 µA
// 5.3.3 System OFF mode = deepest sleep, can be woken by the DETECT signal, optionally from the GPIO peripheral
//    SYSTEMOFF register page 79
// 0x40000000 POWER POWER Power control
// offsets:
// SYSTEMOFF 0x500 System OFF register -- write 1 to turn system off

// Peripherals:
// There is a direct relationship between peripheral ID and base address. For example, a peripheral with base address 0x40000000 is assigned ID=0, a peripheral with base address 0x40001000 is assigned ID=1, and a peripheral with base address 0x4001F000 is assigned ID=31.
// Configure, then write the enable register
// An event register is only cleared when firmware writes 0 to it.
// A peripheral only occupies one interrupt, and the interrupt number follows the peripheral ID. For example, the peripheral with ID=4 is connected to interrupt number 4 in the nested vectored interrupt controller (NVIC).

// GPIO
// When the correct level is detected on a configured pin, the sense mechanism will set the DETECT signal high ... Default behavior, defined by the DETECTMODE register, combines all DETECT signals from the pins in the GPIO port into one common DETECT signal and routes it through the system to be utilized by other peripherals.
// 0x50000000 GPIO GPIO General purpose input and output
// 0x50000000 GPIO P0 General purpose input and output, port P0.00 to P0.31 implemented
// 0x50000300 GPIO P1 General purpose input and output, port P1.00 to P1.15 implemented
// PIN_CNF[n] 0x700 + 4*n Configuration of GPIO pins
//   bit 0: 0 = input, 1 = output
//   bit 1: 0 = connect input buffer (required to use for input), 1 = disconnect (lower power)
//   bit 2,3: 0 = no pull, 1 = pull down, 3 = pull up
//   bit 8,9,10: drive conf. 0 = standard. Other options for high drive or disconnect
//   bit 16,17: pin sense. 0 = disabled, 2 = enabled for high, 3 = enabled for low
// OUTSET 0x508 Set individual bits in GPIO port
// OUTCLR 0x50C Clear individual bits in GPIO port
// IN 0x510 Read GPIO port
// The GPIO DETECT signal will not wake the system up again if the system is put into System ON IDLE while the DETECT signal is high. Clear all DETECT sources before entering sleep.
// Setting the system to System OFF while DETECT is high will cause a wakeup from System OFF reset.
// This feature is always enabled even if the peripheral itself appears to be IDLE, meaning no clocks or other power intensive infrastructure have to be requested to keep this feature enabled. This feature can therefore be used to wake up the CPU from a WFI or WFE type sleep in System ON when all peripherals and the CPU are idle, meaning the lowest power consumption in System ON mode.
#define GPIO_BASE 0x50000000 
#define GPIO_OFFSET_PIN_CNF0 0x700
#define GPIO_OFFSET_OUTSET  0x508
#define GPIO_OFFSET_OUTCLR  0x50C
#define GPIO_OFFSET_IN  0x510

// Radio
// 250 kbps IEEE 802.15.4 mode
// For MODE = Ieee802154_250Kbit the PREAMBLE is 4 bytes long and set to all zeros.
// Radio packets are stored in memory inside instances of a radio packet data structure as illustrated in In- RAM representation of radio packet - S0, LENGTH and S1 are optional on page 312. The PREAMBLE, ADDRESS, CI, TERM1, TERM2 and CRC fields are omitted in this data structure.
// Addresses have logical indices 0..7. The air address represented by each is composed of a base address and a prefix. 0 has BASE0 and the rest have BASE1. 0-3 have PREFIX0.AP[0-3] and 4-7 have PREFIX1.AP[4-7]. Outside of the BASE and PREFIX registers, always use the logical addresses
// Radio states: disabled -(txen)-> txru=ramping up -> txidle <-(start)-> tx -(disable)-> txdisable -> disabled
// Radio states: disabled -(rxen)-> rxru=ramping up -> rxidle <-(start)-> rx -(disable)-> rxdisable -> disabled
//   I think it is also possible to call txen or rxen directly from the rx/tx idle states
//   For tx the carrier keeps running in txidle, so you should disable
//   Shortcuts can be used to automatically run through this state machine instead of manually triggering the steps
// 6.20.12.1 The 802.15.4 packet structure: 5 bytes = 160us preamble and SFD, 1 byte = 32us length, payload, optional crc
//   Payload: 4 zeros
//   SFD: Start of Frame Delimeter = 0xA7 by default, programmable via SFD register
//   PACKETPTR points to the length (followed by payload)
//   length should maybe sometimes include the CRC??? when receiving, I think?
// 802.15.4 MAC: FCF word, Seq byte, PAN, dst addr, src PAN, src addr, security, payload, crc word
//   FCF bits:
//     0,1,2: type
//     3: security
//     4: pend
//     5: ACK
//     6: comp
//     7-9: reserved
//     10-11: dst A mode
//     12-13: frame version
//     14-15: src A mode
//   radio has assisted operation modes that disect this information
// Channel 25    center MHz 2475     FREQUENCY setting 75
// When a complete frame is received the CRCSTATUS register will be updated accordingly and the EVENTS_CRCOK or EVENTS_CRCERROR generated. When the CRC module is enabled it will not write the two last octets (CRC)
// Code to set up the CRC module with the correct polynomial
//     /* 16-bit CRC with ITU-T polynomial with 0 as start condition*/
//     write_reg(NRFRADIO_REG(CRCCNF), 0x202);
//     write_reg(NRFRADIO_REG(CRCPOLY), 0x11021);
//     write_reg(NRFRADIO_REG(CRCINIT), 0);
// The ENDIANESS subregister must be set to little-endian since the FCS field is transmitted leftmost bit first.
// 802.15.4 transmit sequence: disabled -> rxru -> rxidle -> rx (clear channel assessment) -> txru -> txidle -> tx -> txidle -> txdisable -> disabled. See section 6.20.12.6 for details about events and shortcuts
// inter-frame spacing (IFS) is eitehr 12 or 40 symbols. 12 for small frames. In assisted mode this is taken care of automatically
//   see diagram: 32 symbols before an ACK, then IFS before the next packet
// Registers
// Base: 0x40001000 RADIO RADIO 2.4 GHz radio
// TASKS_TXEN 0x000 Enable RADIO in TX mode
// TASKS_RXEN 0x004 Enable RADIO in RX mode
// TASKS_START 0x008 Start RADIO
// TASKS_STOP 0x00C Stop RADIO
// TASKS_DISABLE 0x010 Disable RADIO
// TASKS_CCASTART 0x02C Start the clear channel assessment used in IEEE 802.15.4 mode
// TASKS_CCASTOP 0x030 Stop the clear channel assessment
// EVENTS_READY 0x100 RADIO has ramped up and is ready to be started
// EVENTS_PAYLOAD 0x108 Packet payload sent or received
// EVENTS_END 0x10C Packet sent or received
// EVENTS_DISABLED 0x110 RADIO has been disabled
// EVENTS_CRCOK 0x130 Packet received with CRC ok
// EVENTS_CRCERROR 0x134 Packet received with CRC error
// EVENTS_FRAMESTART 0x138 IEEE 802.15.4 length field received
// EVENTS_CCAIDLE 0x144 Wireless medium in idle - clear to send
// EVENTS_CCABUSY 0x148 Wireless medium busy - do not send
// EVENTS_CCASTOPPED 0x14C The CCA has stopped
// SHORTS 0x200 Shortcuts between local events and tasks
//   bit 0: shortcut ready -> start (0 = disable, 1 = enable)
//   bit 1: end -> disable
//   bit 11: rxready -> cca_start
//   bit 12: cca_idle -> txen
//   bit 13: cca_busy -> disable
//   bit 18: txready -> start
//   bit 19: rxready -> start
//   bit 20: phyend -> disable
//   ... see 6.20.14.37 SHORTS
// INTENSET 0x304 Enable interrupt. See 6.20.14.38 for which bit is which event
// INTENCLR 0x308 Disable interrupt. See 6.20.14.39 for which bit is which event
// PACKETPTR 0x504 Packet pointer
// FREQUENCY 0x508 Frequency. Value 2400 + freq(MHz). Bit 8 reserved for changing 2400 to something else
// TXPOWER 0x50C Output power. Default 0x0 = 0dBm
// MODE 0x510 Data rate and modulation. 15 = ieee 802.15.4
// CRCCNF 0x534 CRC configuration. bits 0-1 = 2 and bits 8-9 = 2 for 802.15.4
// CRCPOLY 0x538 CRC polynomial
// CRCINIT 0x53C CRC initial value
// TIFS 0x544 Interframe spacing in µs
// STATE 0x550 Current radio state
//   0 Disabled
//   1 RxRu         9 TxRu
//   2 RxIdle      10 TxIdle
//   3 Rx          11 Tx
//   4 RxDisable   12 TxDisable
// PCNF0 0x514 packet configuration for DMA
//   bits 0-3: size of length packet in bits (8)
//        8: length of S0 field in bytes (0)
//        16-19: length of S1 in bits (0)
//        20: include S1 always (1) or only if non-zero length (0)
//        22-23: length of code indicator (0)
//        24-25: length of preamble: 32 bits for ieee 802.15.4 = (2)
//        26: length includes crc (1) or does not (0)
//        29-30: length of term field (0)
// PCNF1 0x518 packet configuration for DMA
//   bits 0-7: maxlen of payload
//        8-15: statlen = additional length to receive/send after payload, before CRC (0)
//        16-18: balen = base address length not including prefix = 0??
//        24: endianness, little = 0
//        25: whitening, disable = 0
// POWER 0xFFC Peripheral power control: write 0 = off, 1 = on
#define RADIO_BASE 0x40001000 
#define RADIO_OFFSET_TASKS_TXEN 0X000 
#define RADIO_OFFSET_TASKS_RXEN 0x004
#define RADIO_OFFSET_TASKS_START 0x008
#define RADIO_OFFSET_TASKS_STOP 0x00C
#define RADIO_OFFSET_TASKS_DISABLE 0x010
#define RADIO_OFFSET_TASKS_CCASTART 0x02C
#define RADIO_OFFSET_TASKS_CCASTOP 0x030
#define RADIO_OFFSET_EVENTS_READY 0x100
#define RADIO_OFFSET_EVENTS_PAYLOAD 0x108
#define RADIO_OFFSET_EVENTS_END 0x10C
#define RADIO_OFFSET_EVENTS_DISABLED 0x110
#define RADIO_OFFSET_EVENTS_CRCOK 0x130
#define RADIO_OFFSET_EVENTS_CRCERROR 0x134
#define RADIO_OFFSET_EVENTS_FRAMESTART 0x138
#define RADIO_OFFSET_EVENTS_CCAIDLE 0x144
#define RADIO_OFFSET_EVENTS_CCABUSY 0x148
#define RADIO_OFFSET_EVENTS_CCASTOPPED 0x14C
#define RADIO_OFFSET_SHORTS 0x200
#define RADIO_OFFSET_INTENSET 0x304
#define RADIO_OFFSET_INTENCLR 0x308
#define RADIO_OFFSET_PACKETPTR 0x504
#define RADIO_OFFSET_FREQUENCY 0x508
#define RADIO_OFFSET_TXPOWER 0x50C
#define RADIO_OFFSET_MODE 0x510
#define RADIO_OFFSET_CRCCNF 0x534
#define RADIO_OFFSET_CRCPOLY 0x538
#define RADIO_OFFSET_CRCINIT 0x53C
#define RADIO_OFFSET_CCACTRL 0x66C  // default 0x052D0000, mode in low 3 bits
#define RADIO_OFFSET_TIFS 0x544
#define RADIO_OFFSET_STATE 0x550
#define RADIO_OFFSET_POWER 0xFFC
#define RADIO_OFFSET_PCNF0 0x514
#define RADIO_OFFSET_PCNF1 0x518

#define RADIO_STATE_DISABLED 0
#define RADIO_STATE_RXIDLE   2
#define RADIO_STATE_RX       3
#define RADIO_STATE_TXIDLE   10

// RTC
// Real Time Counter (RTC) -- used for sleeping on the slow clock
// 24 bit counter, 12 bit 1/(X+1) prescale on the low frequency clock (32.768kHz = 30.517us)
// The software has to explicitly start LFCLK before using the RTC.
// prescale resets each time you run the RTC
// Usage: (init: event/interupt enable) -> STOP -> CLEAR -> set prescale, set counter compare -> START -> STOP
// 0x4000B000 RTC RTC0 Real-time counter 0. CC[0..2] implemented, CC[3] not implemented
// 0x40011000 RTC RTC1 Real-time counter 1. CC[0..3] implemented
// 0x40024000 RTC RTC2 Real-time counter 2. CC[0..3] implemented
// TASKS_START 0x000 Start RTC COUNTER
// TASKS_STOP 0x004 Stop RTC COUNTER
// TASKS_CLEAR 0x008 Clear RTC COUNTER
// EVENTS_COMPARE[0] 0x140 Compare event on CC[0] match. 0=not generated, 1=generated
// EVENTS_COMPARE[1] 0x144 Compare event on CC[1] match. 0=not generated, 1=generated
// INTENSET 0x304 Enable interrupt. bit 16 for compare 0, 17 for 1
// INTENCLR 0x308 Disable interrupt. bit 16 for compare 0, 17 for 1
// EVTEN 0x340 Enable or disable event routing. bit 16 for compare 0, 17 for 1
// EVTENSET 0x344 Enable event routing. bit 16 for compare 0, 17 for 1
// EVTENCLR 0x348 Disable event routing. bit 16 for compare 0, 17 for 1
// PRESCALER 0x508 12 bit prescaler for COUNTER frequency (32768/(PRESCALER+1)). Must be written when RTC is stopped. Resets to 0
// CC[0] 0x540 Compare register 0
// CC[1] 0x544 Compare register 1
#define RTC0_BASE 0x4000B000 
#define RTC1_BASE 0x40011000 
#define RTC2_BASE 0x40024000 
#define RTC_OFFSET_TASKS_START 0x000
#define RTC_OFFSET_TASKS_STOP 0x004
#define RTC_OFFSET_TASKS_CLEAR 0x008
#define RTC_OFFSET_EVENTS_COMPARE0 0x140
#define RTC_OFFSET_EVENTS_COMPARE1 0x144
#define RTC_OFFSET_INTENSET 0x304
#define RTC_OFFSET_INTENCLR 0x308
#define RTC_OFFSET_EVTEN 0x340
#define RTC_OFFSET_EVTENSET 0x344
#define RTC_OFFSET_EVTENCLR 0x348
#define RTC_OFFSET_PRESCALER 0x508
#define RTC_OFFSET_CC0 0x540
#define RTC_OFFSET_CC1 0x544
#define RTC_TICK_US 30.517
#define RADIO_STATE_TXIDLE 10
#define RTC_COUNTER_MAX 0xFFFFFF

#define MMIO(base, offset) (*((volatile uint32_t*)(base + offset)))

#define PACKET_PROBE {14, 0x61, 0x88, 1, 0x22, 0x20, 0x4c, 0x10, 0x4d, 0x10, 1, 0xee, 0xee/*0x0d, 0x0b*/}
#define PACKET_PAIR {14, 0x61, 0x88, 1, 0x22, 0x20, 0xff, 0xff, 0x4d, 0x10, 3, 0x4c, 0x10/*0x0d, 0x0b*/} //4c 10 is this probes address
