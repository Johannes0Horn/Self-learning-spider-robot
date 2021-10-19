#include <avr/io.h>
#include "adc.h"



//////////////////////////////////////////
// Analog zu Digital Wandler:
//////////////////////////////////////////
void ADC_Init(void)
{
  ADMUX = (1<<REFS0); // Versorgungsspannung AVcc als Referenz wählen
  // oder interne Referenzspannung als Referenz für den ADC:
  // ADMUX = (1<<REFS1) | (1<<REFS0);
	//keine Ahnung was besser ist ...

  /* Bit ADFR ("free running") in ADCSRA steht beim Einschalten
     schon auf 0, also single conversion -> ADC macht nur eine conversation
	   ADFR=1 wäre free running aktiviert
	   ADCSRA ist das ADC Control and Status Register A
	   und da drin müssen verschiedene Bits gesetzt werden.*/
  ADCSRA = (1<<ADPS1) | (1<<ADPS0);     // Frequenzvorteiler
	//ADEN=ADC Enable
  ADCSRA |= (1<<ADEN); // Den Analog-Digital Wandler aktivieren
    /* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
     also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */

  ADCSRA |= (1<<ADSC);                  // eine ADC-Wandlung
  while (ADCSRA & (1<<ADSC) ) {         // auf Abschluss der Konvertierung warten
  }
  /* ADCW muss einmal gelesen werden, sonst wird Ergebnis der nächsten
     Wandlung nicht übernommen. */
  (void) ADCW;
}

/* ADC Einzelmessung */
uint16_t ADC_Read( uint8_t channel )
{
  // Kanal waehlen, ohne andere Bits zu beeinflußen
	// ADMUX = ADC Multiplexer Select Register
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
  ADCSRA |= (1<<ADSC);//ADSC ADC Start Conversion / eine Wandlung "single conversion"
  while (ADCSRA & (1<<ADSC) ) {   // auf Abschluss der Konvertierung warten
  }
  return ADCW;  // ist das selbe wie ADCL und ADCH nacheinander auslesen  // ADC auslesen und zurückgeben
}

/* ADC Mehrfachmessung mit Mittelwertbbildung */
/* beachte: Wertebereich der Summenvariablen */
uint16_t ADC_Read_Avg( uint8_t channel, uint8_t nsamples )
{
  uint32_t sum = 0;

  for (uint8_t i = 0; i < nsamples; ++i ) {
    sum += ADC_Read( channel );
  }

  return (uint16_t)( sum / nsamples );
}
