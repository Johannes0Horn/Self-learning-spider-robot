#include <avr/io.h>
#include "usart.h"


void USART_init( unsigned int ubrr){
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_transmit_char( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}
void USART_print (char *s)
{
    while (*s)
    {   /* so lange *s != '\0' also ungleich dem "String-Endezeichen(Terminator)" */
        USART_transmit_char(*s);
        s++;
    }
}
void USART_newline()
{
    char* s="\n\r";
    while (*s)
    {  
        USART_transmit_char(*s);
        s++;
    }
}
void USART_println (char *s)
{
    while (*s)
    {   /* so lange *s != '\0' also ungleich dem "String-Endezeichen(Terminator)" */
        USART_transmit_char(*s);
        s++;
    }
    
	(void) USART_newline() ;
    
}

char USART_getc(void)
{
    // wait for data
    while (!(UCSR0A & (1 << RXC0)))
        ;
    // return data
    return UDR0;
}

void USART_getLine(char *buf, int n)
{
    int bufIdx = 0;
    char c;

    // while received character is not carriage return
    // and end of buffer has not been reached
    do
    {
        // receive character
        c = USART_getc();

        // store character in buffer
        buf[bufIdx++] = c;
    } while ((bufIdx < n) && (c != '\r'));

    // ensure buffer is null terminated
    buf[bufIdx] = 0;
}
