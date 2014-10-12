#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

struct PinMapping {
  volatile uint8_t* ddr;     // Data direction port
  volatile uint8_t* port;    // data port
  volatile uint8_t* pin;     // input
  uint8_t  bitmask; // bit
};

struct DisplayState {
  bool numbers[4][8];
  bool colon;
};

void SetupIOPin(const PinMapping& pin) {
  *(pin.ddr) |= pin.bitmask;
}

void ConfigureAsInput(const PinMapping& pin, bool pullup) {
  *(pin.ddr) &= ~pin.bitmask;
  if (pullup)
    *(pin.port) |= pin.bitmask;
  else
    *(pin.port) &= ~pin.bitmask;
}

bool GetInputPinState(const PinMapping& pin) {
  if (*pin.pin & pin.bitmask)
    return true;
  else
    return false;
}

void SetPin(const PinMapping& pin, bool on) {
  if (on)
    *(pin.port) |=  pin.bitmask;
  else
    *(pin.port) &= ~pin.bitmask;
}

class Display {

 public:
  Display(const PinMapping* digit_mapping,
          const PinMapping* segment_mapping,
          const PinMapping& colon_high,
          const PinMapping& colon_low)
  {
    for (int i = 0; i < 4; i++)
      digit_mapping_[i] = digit_mapping[i];

    for (int i = 0; i < 8; i++)
      segment_mapping_[i] = segment_mapping[i];

    colon_high_ = colon_high;
    colon_low_  = colon_low;

    for (int i=0; i < 4; i++)
      *(digit_mapping_[i].ddr) |= digit_mapping_[i].bitmask;

    for (int i=0; i < 8; i++)
      *(segment_mapping_[i].ddr) |= segment_mapping_[i].bitmask;

    *colon_high_.ddr |= colon_high_.bitmask;
    *colon_low_.ddr |= colon_low_.bitmask;
  }

  void SetDisplayState(const DisplayState& display_state) {
    display_state_ = display_state;
  }

  void Scan(int sleep_us) {
    for (int i = 0; i < 4; i++) {
      // Turn off all digits
      *(digit_mapping_[0].port) &= ~digit_mapping_[0].bitmask;
      *(digit_mapping_[1].port) &= ~digit_mapping_[1].bitmask;
      *(digit_mapping_[2].port) &= ~digit_mapping_[2].bitmask;
      *(digit_mapping_[3].port) &= ~digit_mapping_[3].bitmask;

      // Turn off colon
      *colon_high_.port &= ~colon_high_.bitmask;

      // Set each of the segments that we care about
      for (int s=0; s < 8; s++) {
        if (display_state_.numbers[i][s])
          *(segment_mapping_[s].port) &= ~segment_mapping_[s].bitmask;
        else
          *(segment_mapping_[s].port) |=  segment_mapping_[s].bitmask;
      }

      // Turn on the digit that we're on
      *(digit_mapping_[i].port) |= digit_mapping_[i].bitmask;
      _delay_us(sleep_us/5);
    }

    // Deal with Colon
    // Turn off digits
    *(digit_mapping_[0].port) &= ~digit_mapping_[0].bitmask;
    *(digit_mapping_[1].port) &= ~digit_mapping_[1].bitmask;
    *(digit_mapping_[2].port) &= ~digit_mapping_[2].bitmask;
    *(digit_mapping_[3].port) &= ~digit_mapping_[3].bitmask;

    // Turn off colon
    *colon_high_.port &= ~colon_high_.bitmask;

    if (display_state_.colon) {
      *colon_high_.port |=  colon_high_.bitmask;
      *colon_low_.port  &= ~colon_low_.bitmask;
    }
    else {
      *colon_high_.port &= ~colon_high_.bitmask;
      *colon_low_.port  |=  colon_low_.bitmask;
    }
    _delay_us(sleep_us/5);
    *colon_high_.port &= ~colon_high_.bitmask;
    *colon_low_.port  |=  colon_low_.bitmask;
  }

 private:
  PinMapping digit_mapping_[4];
  PinMapping segment_mapping_[8];
  PinMapping colon_high_;
  PinMapping colon_low_;
  DisplayState display_state_;
};


static const bool digit_segments[10][7] =
{ { 1, 1, 1, 1, 1, 1, 0}, // '0'
  { 0, 1, 1, 0, 0, 0, 0}, // '1'
  { 1, 1, 0, 1, 1, 0, 1}, // '2'
  { 1, 1, 1, 1, 0, 0, 1}, // '3'
  { 0, 1, 1, 0, 0, 1, 1}, // '4'
  { 1, 0, 1, 1, 0, 1, 1}, // '5'
  { 0, 0, 1, 1, 1, 1, 1}, // '6'
  { 1, 1, 1, 0, 0, 0, 0}, // '7'
  { 1, 1, 1, 1, 1, 1, 1}, // '8'
  { 1, 1, 1, 0, 0, 1, 1}};// '9'

void CharMapping(const unsigned char letter, bool s[8]) {
  if (letter >= '0' || letter <= '9') {
    for (int i = 0; i < 7; i++) {
      s[i] = digit_segments[letter - '0'][i];
    }
  }
  s[7] = false; // Turn of decimal pt
}

int main (void) {
  // Enable PullUps
  MCUCR &= ~_BV(PUD);

  PinMapping digit_mapping[4] =
  {
    { &DDRC, &PORTC, &PINC, _BV(DDC2) }, // DIG1
    { &DDRC, &PORTC, &PINC, _BV(DDC3) }, // DIG2
    { &DDRD, &PORTD, &PIND, _BV(DDD3) }, // DIG3
    { &DDRD, &PORTD, &PIND, _BV(DDD4) }  // DIG4
  };

  PinMapping segment_mapping[8] =
  {
    { &DDRB, &PORTB, &PINB, _BV(DDB0) }, // A
    { &DDRC, &PORTC, &PINC, _BV(DDC0) }, // B
    { &DDRD, &PORTD, &PIND, _BV(DDD6) }, // C
    { &DDRC, &PORTC, &PINC, _BV(DDB1) }, // D
    { &DDRB, &PORTB, &PINB, _BV(DDB7) }, // E
    { &DDRD, &PORTD, &PIND, _BV(DDD7) }, // F
    { &DDRD, &PORTD, &PIND, _BV(DDD5) }, // G
    { &DDRB, &PORTB, &PINB, _BV(DDB6) }  // DP
  };

  PinMapping colon_high = { &DDRD, &PORTD, &PIND, _BV(DDD2) };
  PinMapping colon_low  = { &DDRD, &PORTD, &PIND, _BV(DDD6) };

  PinMapping speaker_low = {&DDRB, &PORTB, &PINB, _BV(DDB2) };
  //PinMapping speaker_low = {&DDRD, &PORTD, _BV(DDD0) };

  SetupIOPin(speaker_low);
  SetPin(speaker_low, true);

  PinMapping blue_wire   = {&DDRC, &PORTC, &PINC, _BV(DDC4) };
  PinMapping green_wire  = {&DDRC, &PORTC, &PINC, _BV(DDC5) };

  ConfigureAsInput(blue_wire, true);
  ConfigureAsInput(green_wire, true);

  Display display(digit_mapping, segment_mapping, colon_high, colon_low);

//  DisplayState state;
//  for (int i=0; i < 4; i++)
//    for (int j=0; j < 8; j++)
//      state.numbers[i][j] = false;
//  state.numbers[0][1] = true;

  DisplayState state =
  {
    {
    {false, true, true, true, true, true, true, true},
    //{true, true, true, true, true, true, true, true},
    //{true, true, true, true, true, true, true, true},
    //{true, true, true, true, true, true, true, true}
    {false, true, false, false, false, false, false, false},
    {false, false, false, false, false, false, false, false},
    {false, false, false, false, false, false, false, false}
    }
  };

  // CharMapping('0', state.numbers[0]);
  // CharMapping('1', state.numbers[1]);
  // CharMapping('2', state.numbers[2]);
  // CharMapping('3', state.numbers[3]);

  const int initial_count = 3600;
  int count = initial_count-1;
  while(1) {
    int remainder = count;
    int digit;
    digit = remainder / 600;
    CharMapping(digit + '0', state.numbers[0]);
    remainder = remainder - digit * 600;

    digit = remainder / 60;
    CharMapping(digit + '0', state.numbers[1]);
    remainder = remainder - digit * 60;

    digit = remainder / 10;
    CharMapping(digit + '0', state.numbers[2]);
    remainder = remainder - digit * 10;

    CharMapping(remainder + '0', state.numbers[3]);

    state.colon = true;

    display.SetDisplayState(state);

    // Blue Wire: Causes faster countdown
    // Green wire: Causes timer to go back to starting number
    // Yellow wire: Does nothing
    bool blue_wire_state = GetInputPinState(blue_wire);
    bool green_wire_state = GetInputPinState(green_wire);

    int cur_cycles;
    int speaker_cycles;
    if (green_wire_state) {
      cur_cycles = 25;
      speaker_cycles = 100;
    }
    else {
      if (blue_wire_state) {
        cur_cycles = 200;
        speaker_cycles = 400;
      }
      else {
        cur_cycles = 900;
        speaker_cycles = 400;
      }
    }

    for (int i=0; i < cur_cycles; i++) {
      display.Scan(1000);
    }

    if (count < initial_count) {
      bool speaker_state = true;
      for (int i=0; i < speaker_cycles; i++) {
        SetPin(speaker_low, speaker_state);
        speaker_state = !speaker_state;
        _delay_us(125);
      }
    }
    SetPin(speaker_low, true);

    if (green_wire_state) {
      if (count < initial_count) {
        count++;
      }
    }
    else if (count > 0)
      count--;
  }
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
//    for (int filled_segments=0; filled_segments < 8; filled_segments++) {
//      // Fill in up to i
//      for (int i=0; i < 4; i++)
//        for (int j=0; j < 8; j++)
//          state.numbers[i][j] = (bool) (i*8+j < filled_segments);
//      display.SetDisplayState(state);
//      display.Scan(1);
//      _delay_ms(200);
//    }
  }
}
