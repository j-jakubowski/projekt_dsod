/* _____STANDARD INCLUDES____________________________________________________ */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

/* _____PROJECT INCLUDES_____________________________________________________ */
#include "UART.h"

/**
 *  @name Size of transmit and receive buffer
 *
 *  @note Code will be optimized if the size is a power of two
 *        (2,4,8,16,32,64,128 or 256).
 */
//@{
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE 128
#endif

#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE 64
#endif
//@}


/* _____MACROS_______________________________________________________________ */
//! Macro to convert the specified BAUD rate to a 16-bit UBBR register value


#define UART_UBBR_VALUE(Baud) (DIV(F_CPU, (16ul*Baud))-1)

#define GET_NEXT_BUFF_IDX(u8BufferIdx, BUFFER_SIZE) \
        { \
            uint8_t u8Temp = u8BufferIdx; \
            u8Temp++; \
            /* See if buffer size is a power of two */ \
            if((BUFFER_SIZE&(BUFFER_SIZE-1)) == 0) \
            { \
                /* Use masking to optimize pointer wrapping to index 0 */ \
                u8Temp &= (BUFFER_SIZE-1); \
            } \
            else \
            { \
                /* Wrap index to 0 if it has exceeded buffer boundary */ \
                if(u8Temp == (uint8_t)BUFFER_SIZE) u8Temp = 0; \
            } \
            u8BufferIdx = u8Temp; \
        }


/* _____LOCAL VARIABLES______________________________________________________ */
/// Receive ring (circular) buffer
static uint8_t UART_u8RxBuffer[UART_RX_BUFFER_SIZE];
static uint8_t UART_u8RxOut;
static volatile uint8_t UART_u8RxIn;

/// Transmit ring (circular) buffer
static uint8_t UART_u8TxBuffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t UART_u8TxOut;
static uint8_t UART_u8TxIn;

/// Flag that is used by interrupt handler to indicate that transmission is finished (transmit buffer empty)
static bool  UART_bTxFinishedFlag;

/// UART Rx Buffer Overflow
static void (* UART_RxBuffOvf)(void);

/// UART Tx Buffer Overflow
static void (* UART_TxBuffOvf)(void);

//! UART Rx Data Error
static void (* UART_RxDataErr)(UART_Error_t eError);

/* _____PRIVATE FUNCTIONS____________________________________________________ */
//! Received data interrupt handler
ISR(USART0_RX_vect) {
	uint8_t ucsra = UCSR0A;
	uint8_t data  = UDR0;
	uint8_t index = UART_u8RxIn;

	// Calculate next pointer position
	GET_NEXT_BUFF_IDX(index, UART_RX_BUFFER_SIZE);

	// Make sure there is space available in buffer.
	if(index == UART_u8RxOut) {
		if (UART_RxBuffOvf != NULL)
			UART_RxBuffOvf();

		// Receive buffer is full, discard received data
		return;
	}

	// Accept data only if there were no Framing, Data Overrun or Parity Error(s)
	if (ucsra & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) {
		if (UART_RxDataErr != NULL) {
			if (ucsra & (1 << FE0))
				UART_RxDataErr(UART_FRAMING_ERROR);

			if (ucsra & (1 << DOR0))
				UART_RxDataErr(UART_DATA_OVERRUN_ERROR);

			if (ucsra & (1 << UPE0))
				UART_RxDataErr(UART_PARITY_ERROR);
		}

		// Received data had an error, discard received data
		return;
	}

	// Add data to ring buffer
	UART_u8RxBuffer[UART_u8RxIn] = data;

	// Advance pointer
	UART_u8RxIn = index;
}

//! Transmit data register empty interrupt handler
ISR(USART0_UDRE_vect)
{
	// See if there is more data to be sent
	if(UART_u8TxOut == UART_u8TxIn)
	{
		// Disable transmit data register empty interrupt
		BIT_SET_LO(UCSR0B, UDRIE);

		// Enable transmit complete interrupt
		BIT_SET_HI(UCSR0B, TXCIE);

		return;
	}

	// Clear flag to indicate that transmission is busy
	UART_bTxFinishedFlag = FALSE;

	// Send data
	UDR0 = UART_u8TxBuffer[UART_u8TxOut];

	// Advance pointer
	GET_NEXT_BUFF_IDX(UART_u8TxOut, UART_TX_BUFFER_SIZE);
}

//! Transmit complete interrupt handler
ISR(USART0_TX_vect) {
	// Set flag to indicate that transmission has finished
	UART_bTxFinishedFlag = TRUE;

	// Disable interrupt
	BIT_SET_LO(UCSR0B, TXCIE);
}

/* _____PUBLIC FUNCTIONS_____________________________________________________ */

void UART_vInit(uint32_t          baud,
		uint8_t           data_bits,
		UART_Parity_t parity,
		uint8_t           stop_bits) {
	uint8_t ucsrc = 0x00;

	// Initialise variables
	UART_u8RxIn    = 0;
	UART_u8RxOut   = 0;
	UART_u8TxIn    = 0;
	UART_u8TxOut   = 0;
	UART_bTxFinishedFlag = TRUE;

	UART_RxBuffOvf = NULL;
	UART_TxBuffOvf = NULL;
	UART_RxDataErr = NULL;

	switch(parity) {
	case UART_ODD_PARITY :
		// Odd parity
		ucsrc |= (1<<UPM1) | (1<<UPM0);
		break;
	case UART_EVEN_PARITY :
		// Even parity
		ucsrc |= (1<<UPM1) | (0<<UPM0);
		break;
	case UART_NO_PARITY :
		// Fall through...
	default:
		// No parity
		ucsrc |= (0<<UPM1) | (0<<UPM0);
		break;
	}

	switch(data_bits) {
	case 5:
		// 5 data bits
		ucsrc |= (0<<UCSZ2) | (0<<UCSZ1) | (0<<UCSZ0);
		break;
	case 6:
		// 6 data bits
		ucsrc |= (0<<UCSZ2) | (0<<UCSZ1) | (1<<UCSZ0);
		break;
	case 7:
		// 7 data bits
		ucsrc |= (0<<UCSZ2) | (1<<UCSZ1) | (0<<UCSZ0);
		break;
	case 8:
		// Fall through...
	default:
		// 8 data bits
		ucsrc |= (0<<UCSZ2) | (1<<UCSZ1) | (1<<UCSZ0);
		break;
	}

	UCSR0C = ucsrc;

	UART_vChangeBaud(baud);
}


void UART_vSetRxBuffOvfHandler(void (*UART_RxBuffOvf_func)(void)) {
	UART_RxBuffOvf = UART_RxBuffOvf_func;
}

void UART_vSetTxBuffOvfHandler(void (*UART_TxBuffOvf_func)(void)) {
	UART_TxBuffOvf = UART_TxBuffOvf_func;
}

void UART_vSetRxDataErrHandler(void (*UART_RxDataErr_func)(UART_Error_t eError)) {
	UART_RxDataErr = UART_RxDataErr_func;
}

void UART_vChangeBaud(uint32_t baud) {
	uint16_t  ubrr;
	ldiv_t div;

	// Disable UART
	UCSR0B = 0;

	// Calculate new 16-bit UBBR register value
	baud <<= 4;
	div    = ldiv(F_CPU, baud);
	ubrr   = (uint16_t)div.quot;
	baud >>= 1;
	if((uint32_t)(div.rem) < baud) {
		ubrr--;
	}
	// Set BAUD rate by initalising 16-bit UBBR register
	UBRR0H = U16_HI(ubrr);
	UBRR0L = U16_LO(ubrr);

	// Enable RxD/TxD and receive interupt
	UCSR0B = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
}

/**
 *  See if there is received data in the receive buffer.
 *
 *  @retval TRUE    There is received data in the receive buffer
 *  @retval FALSE   The receive buffer is empty
 */
bool UART_bRxBufferEmpty(void) {
	// See if there is data in the buffer
	if(UART_u8RxOut == UART_u8RxIn) {
		return FALSE;
	}
	else {
		return TRUE;
	}
}


/**
 *  See if received byte is available and store it in specified location.
 *
 *  @param[out] pu8Data Pointer to location where data byte must be stored
 *
 *  @retval TRUE  Received byte is stored in specified location
 *  @retval FALSE No received data available (receive buffer empty)
 */
bool UART_bGetRxByte(uint8_t* pu8Data) {
	// See if there is data in the buffer
	if(UART_u8RxOut == UART_u8RxIn) {
		return FALSE;
	}
	// Fetch data
	*pu8Data = UART_u8RxBuffer[UART_u8RxOut];

	// Advance pointer
	GET_NEXT_BUFF_IDX(UART_u8RxOut,UART_RX_BUFFER_SIZE);

	return TRUE;
}

/**
 *  Copy received data from ring buffer into specified buffer.
 *
 *  @param[out] pu8Buffer          Buffer to copy received data into
 *  @param[in] u8MaximumBufferSize Maximum number of received bytes to copy into buffer
 *
 *  @return uint8 Number of received bytes copied into buffer
 */
uint8_t UART_u8GetRxData(uint8_t* pu8Buffer, uint8_t u8MaximumBufferSize) {
	uint8_t u8DataReceived = 0;

	while(u8MaximumBufferSize) {
		// See if there is data in the buffer
		if(UART_u8RxOut == UART_u8RxIn) {
			break;
		}

		// Fetch data
		*pu8Buffer++ = UART_u8RxBuffer[UART_u8RxOut];
		// Advance pointer
		GET_NEXT_BUFF_IDX(UART_u8RxOut,UART_RX_BUFFER_SIZE);
		// Next byte
		u8DataReceived++;
		u8MaximumBufferSize--;
	}
	return u8DataReceived;
}

uint8_t UART_u8NoOfRxBytes(void) {
	if (UART_u8RxIn >= UART_u8RxOut) {
		return (UART_u8RxIn - UART_u8RxOut);
	} else {
		return ((UART_RX_BUFFER_SIZE - UART_u8RxOut) + UART_u8RxIn);
	}
}

void UART_vFlushRxData(void) {
	UART_u8RxOut = UART_u8RxIn = 0;
}

uint8_t UART_u8NoOfTxBytes(void) {
	if (UART_u8TxIn >= UART_u8TxOut) {
		return (UART_u8TxIn - UART_u8TxOut);
	} else {
		return ((UART_TX_BUFFER_SIZE - UART_u8TxOut) + UART_u8TxIn);
	}
}

bool UART_bTxBufferFull(void) {
	uint8_t index = UART_u8TxIn;

	// Calculate next pointer position
	GET_NEXT_BUFF_IDX(index, UART_TX_BUFFER_SIZE);

	if (index == UART_u8TxOut) {
		return TRUE;
	}
	else {
		return FALSE;
	}
}

bool UART_bTxBufferEmpty(void) {
	if(UART_u8TxOut == UART_u8TxIn) {
		return TRUE;
	}
	else {
		return FALSE;
	}
}

bool UART_bTxFinished(void) {
	if(UART_u8TxOut == UART_u8TxIn) {
		return FALSE;
	}

	return UART_bTxFinishedFlag;
}

bool UART_bTxByte(uint8_t u8Data) {
	uint8_t index = UART_u8TxIn;

	// Calculate next pointer position
	GET_NEXT_BUFF_IDX(index, UART_TX_BUFFER_SIZE);

	// Make sure there is space available in buffer
	if (index == UART_u8TxOut) {
		if (UART_TxBuffOvf != NULL)
			UART_TxBuffOvf();

		return FALSE;
	}

	// Insert data into buffer
	UART_u8TxBuffer[UART_u8TxIn] = u8Data;

	// Advance pointer
	UART_u8TxIn = index;

	// Make sure transmit process is started by enabling interrupt
	BIT_SET_HI(UCSR0B, UDRIE);
//	if((UCSRB&(1<<UDRIE))==1) PORTB|=(1<<0);

	return TRUE;
}

extern uint8_t UART_u8TxData(const uint8_t* pu8Data, uint8_t u8NoBytesToSend) {
	uint8_t bytes_buffered = 0;
	uint8_t index;

	while (u8NoBytesToSend) {
		// Calculate next pointer position
		index = UART_u8TxIn;
		GET_NEXT_BUFF_IDX(index, UART_TX_BUFFER_SIZE);

		// Make sure there is space available in buffer
		if (index == UART_u8TxOut) {
			if (UART_TxBuffOvf != NULL)
				UART_TxBuffOvf();

			break;
		}

		// Insert data into buffer
		UART_u8TxBuffer[UART_u8TxIn] = *pu8Data++;

		// Advance pointer
		UART_u8TxIn = index;

		// Next byte
		bytes_buffered++;
		u8NoBytesToSend--;
	}

	// Make sure transmit process is started by enabling interrupt
	BIT_SET_HI(UCSR0B, UDRIE);

	return bytes_buffered;
}

/**
 *  @}
 */
