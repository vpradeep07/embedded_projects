#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

class Foo {

};


int main (void)
{
  DDRC |= _BV(DDC0);
  DDRC |= _BV(DDC1);
  DDRC |= _BV(DDC2);
  DDRC |= _BV(DDC3);

  while(1)
  {
    PORTC = 0x0C;
    _delay_ms(1000);
    PORTC = 0x03;
    _delay_ms(1000);
  }
}
