void USART_init( unsigned int ubrr);
void USART_print (char *s);
void USART_println (char *s);
char USART_getc(void);
void USART_getLine(char *buf, int n);