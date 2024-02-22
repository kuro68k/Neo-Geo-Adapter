/*
 * NeoGeoAdapter.c
 *
 * Created: 09/02/2024 15:48:40
 * Author : kuro68k
 */ 

#include <stdbool.h>
#include <avr/io.h>
#include <avr/fuse.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "hw.h"

volatile uint8_t af_counter_AT = 0;
volatile uint8_t ps_ack_detected_SIG = 0;
bool ps_ack_missed = false;

/******************************************************************************
* Autofire counter overflow interrupt handler
*/
ISR(TCA0_OVF_vect)
{
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
	af_counter_AT++;
}

/******************************************************************************
* Playstation ACK signal detector interrupt, triggered on falling edge
*/
ISR(PORTB_PORT_vect)
{
	ps_ack_detected_SIG = 1;
}

/******************************************************************************
* Set up hardware after reset
*/
static void hw_init(void)
{
	SLPCTRL.CTRLA = 0;	// disable sleep
	
	// set up main clock
	CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;
	CLKCTRL.MCLKCTRLB = 0;
	CLKCTRL.OSCHFCTRLA = CLKCTRL_FRQSEL_24M_gc;
	CLKCTRL.MCLKLOCK = CLKCTRL_LOCKEN_bm;
	
	// set up ports
	PORTA.OUT = 0;
	PORTA.DIR = 0;
	
	PORTB.OUT = PS_ATT_PIN_bm;
	PORTB.DIR = PIN2_bm | PIN3_bm | PS_ATT_PIN_bm;
	PORTB.PIN0CTRL |= PORT_PULLUPEN_bm;		// AF D2
	PORTB.PIN1CTRL |= PORT_PULLUPEN_bm;		// AF D1
	PORTB.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;	// PS ACK

	PORTC.OUT =	PIN0_bm | PIN2_bm;			// SPI pins start high
	PORTC.DIR = PIN0_bm | PIN2_bm | PIN3_bm;
	PORTC.PIN1CTRL |= PORT_PULLUPEN_bm;		// PS DAT / MISO
	PORTC.PIN4CTRL |= PORT_PULLUPEN_bm;		// MODE2
	PORTC.PIN5CTRL |= PORT_PULLUPEN_bm;		// MODE1
	PORTC.PIN6CTRL |= PORT_PULLUPEN_bm;		// AUTO2
	PORTC.PIN7CTRL |= PORT_PULLUPEN_bm;		// AUTO2

	PORTD.OUT = SAT_TL_bm | SAT_TR_bm | SAT_TH_bm;
	PORTD.DIR = SAT_TL_bm | SAT_TR_bm | SAT_TH_bm;
	PORTD.PIN0CTRL |= PORT_PULLUPEN_bm;		// D1
	PORTD.PIN1CTRL |= PORT_PULLUPEN_bm;		// D0
	PORTD.PIN5CTRL |= PORT_PULLUPEN_bm;		// D3
	PORTD.PIN6CTRL |= PORT_PULLUPEN_bm;		// D2
	PORTD.PIN7CTRL |= PORT_PULLUPEN_bm;		// AF C2
	
	PORTE.OUT = 0;
	PORTE.DIR = 0;
	PORTE.PIN0CTRL |= PORT_PULLUPEN_bm;		// AF C1
	PORTE.PIN1CTRL |= PORT_PULLUPEN_bm;		// AF B2
	PORTE.PIN2CTRL |= PORT_PULLUPEN_bm;		// AF B1
	PORTE.PIN3CTRL |= PORT_PULLUPEN_bm;		// AF A2
	
	PORTF.OUT = 0;
	PORTF.DIR = 0;
	PORTF.PIN0CTRL |= PORT_PULLUPEN_bm;		// AF A1
	PORTF.PIN3CTRL |= PORT_PULLUPEN_bm;		// NEO NC2
	PORTF.PIN4CTRL |= PORT_PULLUPEN_bm;		// NEO NC1
	
	// autofire timers
	TCA0.SINGLE.CTRLB = 0;
	TCA0.SINGLE.CTRLC = 0;
	TCA0.SINGLE.CTRLD = 0;
	TCA0.SINGLE.EVCTRL = 0;
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
	TCA0.SINGLE.PER = 0x61A7;							// 15Hz
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
	//TCA1.SINGLE.CTRLB = 0;
	//TCA1.SINGLE.CTRLC = 0;
	//TCA1.SINGLE.CTRLD = 0;
	//TCA1.SINGLE.EVCTRL = 0;
	//TCA1.SINGLE.INTCTRL = 0;
	//TCA1.SINGLE.PER = 0x493D;							// 5Hz
	//TCA1.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm;
	
	// SPI
	SPI.INTCTRL = 0;
	SPI.CTRLB = SPI_SSD_bm | SPI_MODE_0_gc;
	SPI.CTRLA = SPI_DORD_bm | SPI_MASTER_bm |			// LSB first
	SPI_PRESC_DIV128_gc | SPI_ENABLE_bm;	// ~180kHz
}

/******************************************************************************
* Send a command and receive a response from the Playstation. Checks for
* the ACK signal.
*/
inline static uint8_t ps_rxtx(uint8_t tx_byte)
{
	ps_ack_detected_SIG = 0;
	SPI.DATA = PS_CMD_ADDR;
	while (!(SPI.INTFLAGS & SPI_IF_bm));
	_delay_us(15);
	if (!ps_ack_detected_SIG)
		ps_ack_missed = true;
	return SPI.DATA;
}

/******************************************************************************
* Main entry point
*/
int main(void)
{
	NVMCTRL.CTRLB = NVMCTRL_APPDATAWP_bm | NVMCTRL_APPCODEWP_bm;	// prevent accident writes to flash
	hw_init();
	sei();
	
	// main loop
	for(;;)
	{
		ps_ack_missed = false;
		
		// start Saturn comms
		SAT_PORT.OUTSET = SAT_TH_bm | SAT_TL_bm | SAT_TR_bm;
		// start PS comms
		while (!(SPI.INTFLAGS & SPI_IF_bm));
		PS_ACK_PORT.OUTCLR = PS_ATT_PIN_bm;
		_delay_us(20);									// match PS timing
		
		// Saturn stage 1
		uint8_t sat_status1 = SAT_PORT.IN;
		SAT_PORT.OUTCLR = SAT_TH_bm;
		// PS address controller
		ps_rxtx(PS_CMD_ADDR);
		
		// Saturn stage 2
		uint8_t sat_status2 = SAT_PORT.IN;
		SAT_PORT.OUTCLR = SAT_TR_bm;
		// PS read command
		ps_rxtx(PS_CMD_READ);
		
		// Saturn stage 3
		uint8_t sat_status3 = SAT_PORT.IN;
		SAT_PORT.OUTSET = SAT_TH_bm | SAT_TR_bm;
		// PS unused (?) byte
		ps_rxtx(0x00);

		// Saturn stage 4
		uint8_t sat_status4 = SAT_PORT.IN;
		SAT_PORT.OUTSET = SAT_TL_bm;
		// PS first digital status byte
		uint8_t ps_status1 = ps_rxtx(0x00);

		// PS second digital status byte
		uint8_t ps_status2 = ps_rxtx(0x00);

		// TODO: analogue sticks


		// switches
		uint8_t map = 0;
		map |= (SW_MODE1_PORT.IN & SW_MODE1_PIN_bm) ? (1<<0) : 0;
		map |= (SW_MODE2_PORT.IN & SW_MODE2_PIN_bm) ? (1<<1) : 0;
		uint8_t af_mode = 0;
		map |= (SW_AUTO1_PORT.IN & SW_AUTO1_PIN_bm) ? (1<<0) : 0;
		map |= (SW_AUTO2_PORT.IN & SW_AUTO2_PIN_bm) ? (1<<1) : 0;

		// autofire
		uint8_t af_count = af_counter_AT;
		uint8_t af_a = 0;
		uint8_t af_ap = 0;
		uint8_t af_b = 0;
		uint8_t af_bp = 0;
		uint8_t af_c = 0;
		uint8_t af_cp = 0;
		uint8_t af_d = 0;
		uint8_t af_dp = 0;
		switch (af_mode)
		{
			case AF_MODE_DUAL:
			default:
				if (!(SW_AF_A1_PORT.IN & SW_AF_A1_PIN_bm))	af_ap = (af_count & 0b01) ? 0xFF : 0;
				if (!(SW_AF_A2_PORT.IN & SW_AF_A2_PIN_bm))	af_ap = ((af_count & 0b10) >> 1) ? 0xFF : 0;
				if (!(SW_AF_B1_PORT.IN & SW_AF_B1_PIN_bm))	af_bp = (af_count & 0b01) ? 0xFF : 0;
				if (!(SW_AF_B2_PORT.IN & SW_AF_B2_PIN_bm))	af_bp = ((af_count & 0b10) >> 1) ? 0xFF : 0;
				if (!(SW_AF_C1_PORT.IN & SW_AF_C1_PIN_bm))	af_cp = (af_count & 0b01) ? 0xFF : 0;
				if (!(SW_AF_C2_PORT.IN & SW_AF_C2_PIN_bm))	af_cp = ((af_count & 0b10) >> 1) ? 0xFF : 0;
				if (!(SW_AF_D1_PORT.IN & SW_AF_D1_PIN_bm))	af_dp = (af_count & 0b01) ? 0xFF : 0;
				if (!(SW_AF_D2_PORT.IN & SW_AF_D2_PIN_bm))	af_dp = ((af_count & 0b10) >> 1) ? 0xFF : 0;
				af_a = af_b = af_c = af_d = 0xFF;
				break;

			case AF_MODE_ALL:
				if (!(SW_AF_A1_PORT.IN & SW_AF_A1_PIN_bm))	af_ap = (af_count & 0b01) ? 0xFF : 0;
				if (!(SW_AF_A2_PORT.IN & SW_AF_A2_PIN_bm))	af_ap = ((af_count & 0b10) >> 1) ? 0xFF : 0;
				if (!(SW_AF_B1_PORT.IN & SW_AF_B1_PIN_bm))	af_bp = (af_count & 0b01) ? 0xFF : 0;
				if (!(SW_AF_B2_PORT.IN & SW_AF_B2_PIN_bm))	af_bp = ((af_count & 0b10) >> 1) ? 0xFF : 0;
				if (!(SW_AF_C1_PORT.IN & SW_AF_C1_PIN_bm))	af_cp = (af_count & 0b01) ? 0xFF : 0;
				if (!(SW_AF_C2_PORT.IN & SW_AF_C2_PIN_bm))	af_cp = ((af_count & 0b10) >> 1) ? 0xFF : 0;
				if (!(SW_AF_D1_PORT.IN & SW_AF_D1_PIN_bm))	af_dp = (af_count & 0b01) ? 0xFF : 0;
				if (!(SW_AF_D2_PORT.IN & SW_AF_D2_PIN_bm))	af_dp = ((af_count & 0b10) >> 1) ? 0xFF : 0;
				af_a = af_ap;
				af_b = af_bp;
				af_c = af_cp;
				af_d = af_dp;
				break;

			case AF_MODE_HOLD:
				af_a = af_b = af_c = af_d = 0xFF;
				af_ap = af_bp = af_cp = af_dp = 0xFF;
				break;
		}
/*
		if (!(SW_AF_A1_PORT.IN & SW_AF_A1_PIN_bm))	af_a = (af_count & 0b01) ? 0xFF : 0;
		if (!(SW_AF_A2_PORT.IN & SW_AF_A2_PIN_bm))	af_a = ((af_count & 0b10) >> 1) ? 0xFF : 0;
		if (!(SW_AF_B1_PORT.IN & SW_AF_B1_PIN_bm))	af_b = (af_count & 0b01) ? 0xFF : 0;
		if (!(SW_AF_B2_PORT.IN & SW_AF_B2_PIN_bm))	af_b = ((af_count & 0b10) >> 1) ? 0xFF : 0;
		if (!(SW_AF_C1_PORT.IN & SW_AF_C1_PIN_bm))	af_c = (af_count & 0b01) ? 0xFF : 0;
		if (!(SW_AF_C2_PORT.IN & SW_AF_C2_PIN_bm))	af_c = ((af_count & 0b10) >> 1) ? 0xFF : 0;
		if (!(SW_AF_D1_PORT.IN & SW_AF_D1_PIN_bm))	af_d = (af_count & 0b01) ? 0xFF : 0;
		if (!(SW_AF_D2_PORT.IN & SW_AF_D2_PIN_bm))	af_d = ((af_count & 0b10) >> 1) ? 0xFF : 0;
*/

		uint8_t neo_port1 = 0;
		uint8_t neo_port2 = 0;

		// common to all modes
		neo_port1 |= (sat_status2 & SAT_D0_bm) ? NEO_UP_PIN_bm : 0;
		neo_port1 |= (sat_status2 & SAT_D1_bm) ? NEO_DOWN_PIN_bm : 0;
		neo_port1 |= (sat_status2 & SAT_D2_bm) ? NEO_LEFT_PIN_bm : 0;
		neo_port1 |= (sat_status2 & SAT_D3_bm) ? NEO_RIGHT_PIN_bm : 0;
		neo_port1 |= (ps_status1 & PS_S1_UP_bm) ? NEO_UP_PIN_bm : 0;
		neo_port1 |= (ps_status1 & PS_S1_RIGHT_bm) ? NEO_RIGHT_PIN_bm : 0;
		neo_port1 |= (ps_status1 & PS_S1_DOWN_bm) ? NEO_DOWN_PIN_bm : 0;
		neo_port1 |= (ps_status1 & PS_S1_LEFT_bm) ? NEO_LEFT_PIN_bm : 0;
		
		// Saturn		D0	D1	D2	D3
		// Status 1		-	-	-	L
		// Status 2		Up	Dn	Lf	Rt
		// Status 3		Z	Y	X	R
		// Status 4		B	C	A	Start
	
		switch(map)
		{
			case MAP_PAD:
			default:
			
			// Saturn
			if (sat_status4 & SAT_D3_bm)	// Start
			{
				neo_port2 |= (sat_status1 & SAT_D3_bm) ? NEO_START_PIN_bm : 0;		// L = Start
				sat_status1 &= ~SAT_D3_bm;
				neo_port2 |= (sat_status3 & SAT_D3_bm) ? NEO_SELECT_PIN_bm : 0;		// R = Select
				sat_status3 &= ~SAT_D3_bm;
			}
			else
			{
				neo_port1 |= (sat_status1 & SAT_D3_bm) ? NEO_B_PIN_bm & af_b:  0;	// L = B'
				neo_port2 |= (sat_status3 & SAT_D3_bm) ? NEO_D2_PIN_bm & af_d : 0;	// R = D'
			}

			neo_port1 |= (sat_status3 & SAT_D0_bm) ? NEO_C_PIN_bm & af_c : 0;	// Z = C'
			neo_port1 |= (sat_status3 & SAT_D1_bm) ? NEO_D1_PIN_bm  : 0;		// Y = D
			neo_port2 |= (sat_status3 & SAT_D1_bm) ? NEO_D2_PIN_bm  : 0;		// Y = D
			neo_port1 |= (sat_status3 & SAT_D2_bm) ? NEO_C_PIN_bm : 0;			// X = C
			
			neo_port1 |= (sat_status4 & SAT_D0_bm) ? NEO_B_PIN_bm : 0;			// B = B
			neo_port1 |= (sat_status4 & SAT_D1_bm) ? NEO_A_PIN_bm & af_a : 0;	// C = A'
			neo_port1 |= (sat_status4 & SAT_D2_bm) ? NEO_A_PIN_bm : 0;			// A = A

			// Playstation
			if (!ps_ack_missed)
			{
				neo_port2 |= (ps_status1 & PS_S1_SELECT_bm) ? NEO_SELECT_PIN_bm : 0;
				//neo_port2 |= (ps_status1 & PS_S1_L3_bm) ? NEO_SELECT_PIN_bm : 0;
				//neo_port2 |= (ps_status1 & PS_S1_R3_bm) ? NEO_SELECT_PIN_bm : 0;
				neo_port2 |= (ps_status1 & PS_S1_START_bm) ? NEO_START_PIN_bm : 0;
			
				neo_port1 |= (ps_status2 & PS_S2_L2_bm) ? NEO_C_PIN_bm & af_c : 0;
				neo_port1 |= (ps_status2 & PS_S2_R2_bm) ? NEO_D1_PIN_bm & af_d : 0;
				neo_port2 |= (ps_status2 & PS_S2_R2_bm) ? NEO_D2_PIN_bm & af_d : 0;
				neo_port1 |= (ps_status2 & PS_S2_L1_bm) ? NEO_A_PIN_bm & af_a : 0;
				neo_port1 |= (ps_status2 & PS_S2_R1_bm) ? NEO_B_PIN_bm & af_b : 0;
				neo_port1 |= (ps_status2 & PS_S2_TRIANGLE_bm) ? NEO_D1_PIN_bm : 0;
				neo_port2 |= (ps_status2 & PS_S2_TRIANGLE_bm) ? NEO_D2_PIN_bm : 0;
				neo_port1 |= (ps_status2 & PS_S2_CIRCLE_bm) ? NEO_B_PIN_bm : 0;
				neo_port1 |= (ps_status2 & PS_S2_CROSS_bm) ? NEO_A_PIN_bm : 0;
				neo_port1 |= (ps_status2 & PS_S2_SQUARE_bm) ? NEO_C_PIN_bm : 0;
			}

			break;
		}

		neo_port1 |= (ps_status2 & PS_S2_L2_bm) ? NEO_C_PIN_bm & af_c : 0;
		neo_port1 |= (ps_status2 & PS_S2_R2_bm) ? NEO_D1_PIN_bm & af_d : 0;
		neo_port2 |= (ps_status2 & PS_S2_R2_bm) ? NEO_D2_PIN_bm & af_d : 0;
		neo_port1 |= (ps_status2 & PS_S2_L1_bm) ? NEO_A_PIN_bm & af_a : 0;
		neo_port1 |= (ps_status2 & PS_S2_R1_bm) ? NEO_B_PIN_bm & af_b : 0;

		if (af_mode == AF_MODE_HOLD)
		{
			if (!(SW_AF_A1_PORT.IN & SW_AF_A1_PIN_bm))	neo_port1 |= (af_count & 0b01) ? NEO_A_PIN_bm : 0;
			if (!(SW_AF_A2_PORT.IN & SW_AF_A2_PIN_bm))	neo_port1 |= NEO_A_PIN_bm;
			if (!(SW_AF_B1_PORT.IN & SW_AF_B1_PIN_bm))	neo_port1 |= (af_count & 0b01) ? NEO_B_PIN_bm : 0;
			if (!(SW_AF_B2_PORT.IN & SW_AF_B2_PIN_bm))	neo_port1 |= NEO_B_PIN_bm;
			if (!(SW_AF_C1_PORT.IN & SW_AF_C1_PIN_bm))	neo_port1 |= (af_count & 0b01) ? NEO_C_PIN_bm : 0;
			if (!(SW_AF_C2_PORT.IN & SW_AF_C2_PIN_bm))	neo_port1 |= NEO_C_PIN_bm;
			if (!(SW_AF_D1_PORT.IN & SW_AF_D1_PIN_bm))	neo_port1 |= (af_count & 0b01) ? NEO_D1_PIN_bm : 0;
			if (!(SW_AF_D2_PORT.IN & SW_AF_D2_PIN_bm))	neo_port1 |= NEO_D1_PIN_bm;
			if (!(SW_AF_D1_PORT.IN & SW_AF_D1_PIN_bm))	neo_port2 |= (af_count & 0b01) ? NEO_D2_PIN_bm : 0;
			if (!(SW_AF_D2_PORT.IN & SW_AF_D2_PIN_bm))	neo_port2 |= NEO_D2_PIN_bm;
		}

		NEO_PORT1.DIR = neo_port1;
		NEO_PORT2.DIR = neo_port2;
	}
}

FUSES = {
	.WDTCFG = WINDOW_OFF_gc | PERIOD_512CLK_gc,		// TODO: lower to minimum
	.BODCFG = LVL_BODLEVEL3_gc | SAMPFREQ_128Hz_gc |
			  ACTIVE_ENABLE_gc | SLEEP_ENABLE_gc,
	.OSCCFG = CLKSEL_OSCHF_gc,
	.SYSCFG0 = RSTPINCFG_RST_gc | FUSE_EESAVE,
	.SYSCFG1 = SUT_64MS_gc,
	.CODESIZE = FUSE_CODESIZE_DEFAULT,				// whole of flash is BOOT section
	.BOOTSIZE = FUSE_BOOTSIZE_DEFAULT,
};
