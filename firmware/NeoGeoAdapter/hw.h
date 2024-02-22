/*
 * hw.h
 *
 * Created: 12/02/2024 11:08:38
 *  Author: kuro68k
 */ 


#ifndef HW_H_
#define HW_H_

// Playstation
#define SPI					SPI1	// PORTC
#define PS_ACK_PORT			PORTB
#define PS_ACK_PIN_bm		PIN4_bm
#define PS_ATT_PORT			PORTB
#define PS_ATT_PIN_bm		PIN5_bm

#define PS_CMD_ADDR			0x01
#define PS_CMD_READ			0x42

#define PS_S1_SELECT_bm		(1<<0)
#define PS_S1_L3_bm			(1<<1)	// left thumb stick pressed down
#define PS_S1_R3_bm			(1<<2)	// right thumb stick pressed down
#define PS_S1_START_bm		(1<<3)
#define PS_S1_UP_bm			(1<<4)
#define PS_S1_RIGHT_bm		(1<<5)
#define PS_S1_DOWN_bm		(1<<6)
#define PS_S1_LEFT_bm		(1<<7)

#define PS_S2_L2_bm			(1<<0)
#define PS_S2_R2_bm			(1<<1)
#define PS_S2_L1_bm			(1<<2)
#define PS_S2_R1_bm			(1<<3)
#define PS_S2_TRIANGLE_bm	(1<<4)
#define PS_S2_CIRCLE_bm		(1<<5)
#define PS_S2_CROSS_bm		(1<<6)
#define PS_S2_SQUARE_bm		(1<<7)


// Saturn
#define SAT_PORT			PORTD
#define	SAT_D1_bm			PIN0_bm
#define	SAT_D0_bm			PIN1_bm
#define	SAT_TH_bm			PIN2_bm
#define	SAT_TR_bm			PIN3_bm
#define	SAT_TL_bm			PIN4_bm
#define	SAT_D3_bm			PIN5_bm
#define	SAT_D2_bm			PIN6_bm


// Neo Geo
// port 1
#define NEO_PORT1			PORTA
#define NEO_UP_PORT			PORTA
#define NEO_UP_PIN_bm		PIN0_bm
#define NEO_DOWN_PORT		PORTA
#define NEO_DOWN_PIN_bm		PIN1_bm
#define NEO_LEFT_PORT		PORTA
#define NEO_LEFT_PIN_bm		PIN2_bm
#define NEO_RIGHT_PORT		PORTA
#define NEO_RIGHT_PIN_bm	PIN3_bm
#define NEO_A_PORT			PORTA
#define NEO_A_PIN_bm		PIN4_bm
#define NEO_B_PORT			PORTA
#define NEO_B_PIN_bm		PIN5_bm
#define NEO_C_PORT			PORTA
#define NEO_C_PIN_bm		PIN6_bm
#define NEO_D1_PORT			PORTA
#define NEO_D1_PIN_bm		PIN7_bm
// port 2
#define NEO_PORT2			PORTF
#define NEO_START_PORT		PORTF
#define NEO_START_PIN_bm	PIN1_bm
#define NEO_SELECT_PORT		PORTF
#define NEO_SELECT_PIN_bm	PIN2_bm
#define NEO_NC2_PORT		PORTF
#define NEO_NC2_PIN_bm		PIN3_bm
#define NEO_NC1_PORT		PORTF
#define NEO_NC1_PIN_bm		PIN4_bm
#define NEO_D2_PORT			PORTF
#define NEO_D2_PIN_bm		PIN5_bm


// switches
#define SW_AF_A1_PORT		PORTF
#define SW_AF_A1_PIN_bm		PIN0_bm
#define SW_AF_A2_PORT		PORTE
#define SW_AF_A2_PIN_bm		PIN3_bm

#define SW_AF_B1_PORT		PORTE
#define SW_AF_B1_PIN_bm		PIN2_bm
#define SW_AF_B2_PORT		PORTE
#define SW_AF_B2_PIN_bm		PIN1_bm

#define SW_AF_C1_PORT		PORTE
#define SW_AF_C1_PIN_bm		PIN0_bm
#define SW_AF_C2_PORT		PORTD
#define SW_AF_C2_PIN_bm		PIN7_bm

#define SW_AF_D1_PORT		PORTB
#define SW_AF_D1_PIN_bm		PIN1_bm
#define SW_AF_D2_PORT		PORTB
#define SW_AF_D2_PIN_bm		PIN0_bm

#define SW_MODE1_PORT		PORTC
#define SW_MODE1_PIN_bm		PIN5_bm
#define SW_MODE2_PORT		PORTC
#define SW_MODE2_PIN_bm		PIN4_bm
#define MAP_PAD				0b00
#define MAP_STICK1			0b01
#define MAP_STICK2			0b10

#define SW_AUTO1_PORT		PORTC
#define SW_AUTO1_PIN_bm		PIN7_bm
#define SW_AUTO2_PORT		PORTC
#define SW_AUTO2_PIN_bm		PIN6_bm
#define AF_MODE_DUAL		0b00
#define AF_MODE_ALL			0b01
#define AF_MODE_HOLD		0b10


#endif /* HW_H_ */