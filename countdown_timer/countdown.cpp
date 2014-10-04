#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

struct PinMapping {
  volatile uint8_t* ddr;     // Data direction port
  volatile uint8_t* port;    // data port
  uint8_t  bitmask; // bit
};

struct DisplayState {
  bool numbers[4][8];
};

class Display {

 public:
  Display(const PinMapping* digit_mapping,
          const PinMapping* segment_mapping)
  {
    for (int i = 0; i < 4; i++)
      digit_mapping_[i] = digit_mapping[i];

    for (int i = 0; i < 8; i++)
      segment_mapping_[i] = segment_mapping[i];

    for (int i=0; i < 4; i++)
      *(digit_mapping_[i].ddr) |= digit_mapping_[i].bitmask;

    for (int i=0; i < 8; i++)
      *(segment_mapping_[i].ddr) |= segment_mapping_[i].bitmask;
  }

  void SetDisplayState(const DisplayState& display_state) {
    display_state_ = display_state;
  }

  void Scan(int sleep_cycles) {
    *(digit_mapping_[0].port) |= ~digit_mapping_[0].bitmask;
    *(digit_mapping_[1].port) &=  digit_mapping_[1].bitmask;
    *(digit_mapping_[2].port) &=  digit_mapping_[2].bitmask;
    *(digit_mapping_[3].port) &=  digit_mapping_[3].bitmask;

    for (int i=0; i < 8; i++) {
      if (display_state_.numbers[0][i])
        *(segment_mapping_[i].port) &= ~segment_mapping_[i].bitmask;
      else
        *(segment_mapping_[i].port) |= segment_mapping_[i].bitmask;
    }
  }

 private:
  PinMapping digit_mapping_[4];
  PinMapping segment_mapping_[8];
  DisplayState display_state_;
};

int main (void) {
  PinMapping digit_mapping[4] =
  {
    { &DDRC, &PORTC, _BV(DDC2) }, // DIG1
    { &DDRC, &PORTC, _BV(DDC3) }, // DIG2
    { &DDRD, &PORTD, _BV(DDD3) }, // DIG3
    { &DDRD, &PORTD, _BV(DDD4) } // DIG4
  };

  PinMapping segment_mapping[8] =
  {
    { &DDRB, &PORTB, _BV(DDB0) }, // A
    { &DDRC, &PORTC, _BV(DDC0) }, // B
    { &DDRD, &PORTD, _BV(DDB6) }, // C
    { &DDRC, &PORTC, _BV(DDB1) }, // D
    { &DDRB, &PORTB, _BV(DDB7) }, // E
    { &DDRD, &PORTD, _BV(DDD7) }, // F
    { &DDRB, &PORTB, _BV(DDB6) }  // DP
  };

  Display display(digit_mapping, segment_mapping);

  DisplayState state = {
    {
    {false, false, false, false, false, false, false, false},
    {false, false, false, false, false, false, false, false},
    {false, false, false, false, false, false, false, false},
    {false, false, false, false, false, false, false, false}
    }
  };

//  DDRC |= _BV(DDC0);
//  DDRC |= _BV(DDC1);
//  DDRC |= _BV(DDC2);
//  DDRC |= _BV(DDC3);

//  *(&DDRC) |= _BV(DDC0);
//  *(&DDRC) |= _BV(DDC1);
//  *(&DDRC) |= _BV(DDC2);
//  *(&DDRC) |= _BV(DDC3);


//  while(1)
//  {
//    PORTC = 0x0C;
//    _delay_ms(1000);
//    PORTC = 0x03;
//    _delay_ms(1000);
//  };

  while(1)
  {
    // Crawl along display
    for (int filled_segments=0; filled_segments < 8; filled_segments++) {
      // Fill in up to i
      for (int i=0; i < 4; i++)
        for (int j=0; j < 8; j++)
          state.numbers[i][j] = (bool) (i*8+j < filled_segments);
      display.SetDisplayState(state);
      display.Scan(1);
      _delay_ms(1000);
    }
  }
}
