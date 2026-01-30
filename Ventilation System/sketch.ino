#include "wiring_private.h"

// Variables for DHT22(the sensor for measuring the temperature)
volatile int16_t t16bits = 0; //contains the temperature

// Variables for the Display
#define DISPRATE 20
#define TEMPRATE 100
#define BLINKINGRATE 300
#define IDLE_TIMEOUT 5000

// 7-segment codes(gfedcba)
const uint8_t PROGMEM ds7[12] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b00000000, // empty
  0b01000000  // minus
};

volatile uint8_t disps[3];
volatile uint8_t currentDigit = 0;
volatile uint8_t dispcnt = DISPRATE;
volatile uint8_t tempcnt = TEMPRATE;
volatile uint16_t blinkcnt = BLINKINGRATE;
volatile uint16_t displayTimeoutCnt = IDLE_TIMEOUT;


// Variables for buttons and LEDs
#define LEDRATE 20
volatile uint16_t ledcnt = LEDRATE;
volatile bool inEditMode = false;
volatile int8_t* editTarget = nullptr;

bool redNowPressed  = false;
bool blueNowPressed = false;

uint16_t redPressedDuration  = 0;
uint16_t bluePressedDuration = 0;

volatile int8_t tMin = 0;
volatile int8_t tMax = 25;


// DHT22 - beginning of the code
volatile bool lastDht22Read;
volatile bool dataReady = false; //Check if the temperature is ready

ISR(TIMER1_COMPA_vect) {
  static uint16_t dht22Steps = 0;
  static uint64_t rawData = 0;
  static uint16_t startReadingStep = 0;
  static uint8_t bitIndex = 0;


  bool dhtRead = PINC & (1 << PC0);

  if (dht22Steps == 0) {
    DDRC  |= (1 << PC0);      // Set PC0 as output
    PORTC &= ~(1 << PC0);     // Pull LOW to start the signal
  }

  if (dht22Steps == 200) {
    PORTC |= (1 << PC0);      // Pull HIGH
    DDRC  &= ~(1 << PC0);     // Set PC0 as input
    PORTC |= (1 << PC0);      // Enable pull-up
  }

  if (dht22Steps >= 224 && bitIndex < 40) {
    if (dhtRead && !lastDht22Read) {
      startReadingStep = dht22Steps;
    }

    if (!dhtRead && lastDht22Read && startReadingStep != 0) {
      rawData <<= 1;

      if ((dht22Steps - startReadingStep) > 4) {
        rawData |= 1;
      }

      bitIndex++;
      startReadingStep = 0;
    }

    if (bitIndex == 40) {
      t16bits = (rawData >> 8) & 0xFFFF;
      dataReady = true;
    }
  }

  dht22Steps++;
  if (dht22Steps >= 1000) {
    dht22Steps = 0;
    bitIndex = 0;
    rawData = 0;
  }

  lastDht22Read = dhtRead;
}

// Get the temperature in degrees Celsius
int8_t getTemperature() {

  int16_t raw = (t16bits & 0x7FFF); // unsigned temperature
  return (int8_t) ((t16bits >> 15) ? -(raw + 5) / 10 : (t16bits + 5) / 10); //We round the temperature to the nearest whole number
}
// DHT22 - end of code


// LED code
void updateLedLogic(int8_t currentTemperature, int8_t tMin, int8_t tMax) {
  uint8_t pwmOutputValue;

  if (currentTemperature < tMin) {
    pwmOutputValue = 0;

  } else if (currentTemperature > tMax) {
    pwmOutputValue = 255;

  } else {
    if (tMin >= tMax) {
      // Handles cases where tMin might be incorrectly set to be equal to or greater than tMax.
      // If the current temperature is equal to tMin (and tMax), set the default duty cycle to 25%.
      pwmOutputValue = 64; // 25% of 255
    } else {
   /*** Calculate the corresponding PWM value by linearly interpolating the current temperature,
        over the range [tMin, tMax], mapping the result between 25% (64) and 99% (252) of the duty cycle. ***/
      pwmOutputValue = 64 + ((uint16_t)((currentTemperature - tMin) * 188)) / (tMax - tMin);
    }
  }

  if (!pwmOutputValue) {       //The value is 0
    TCCR2A &= ~((1 << COM2A1) | (1 << COM2A0));
    PORTB  &= ~(1 << PB3);
  } else {
    TCCR2A = (TCCR2A & ~((1 << COM2A1) | (1 << COM2A0))) | (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (TCCR2B & ~((1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20))) | (1 << CS22);
    OCR2A  = pwmOutputValue;
  }
}
// End of LED code


// Timer0 Logic
ISR(TIMER0_COMPA_vect) {
  //Measuring keypress time
  if (redNowPressed)  redPressedDuration++;
  if (blueNowPressed) bluePressedDuration++;

  // Exit edit mode after 5 seconds
  if (inEditMode && --displayTimeoutCnt == 0) {
    inEditMode  = false;
    editTarget  = nullptr;
  }

  // Logic for blinking in edit mode
  if (inEditMode && --blinkcnt == 0) {
    blinkcnt = BLINKINGRATE;
    static bool blinkVisible = true;
    blinkVisible = !blinkVisible;

    if (blinkVisible) {
      display(*editTarget);
    } else {
      disps[0] = 10; disps[1] = 10; disps[2] = 10;
    }
  }

  // Idle mode
  if (--tempcnt == 0 && !inEditMode && dataReady) {
    tempcnt = TEMPRATE;
    dataReady = false;
    display(getTemperature());
  }

  // When is it time to update your 7-segment display
  if (--dispcnt == 0) {
    dispcnt = DISPRATE;

    // Turn off all digits
    PORTB |= (1 << PB1) | (1 << PB2) | (1 << PB4);

    // 7-segment pattern from PROGMEM
    uint8_t segs = pgm_read_byte(ds7 + disps[currentDigit]);

    // Writes the segments all at once
    // a-f: PD2-PD7, g: PB0

    // PD2-PD7 to 0, then set from segs<0:5>
    PORTD = (PORTD & 0x03)          // maintains PD0, PD1
            | ((segs & 0x3F) << 2); // bit 0-5 → PD2-PD7

    // PB0 to 0, then set by segs<6>
    if (segs & 0x40) PORTB |=  (1 << PB0);
    else PORTB &= ~(1 << PB0);

    // Enable only the current digit
    // pin digits 9→PB1 10→PB2 12→PB4
    // set the selected one LOW
    PORTB &= ~(1 << (1 << currentDigit));

    // Advance to the next digit
    currentDigit = (currentDigit + 1) % 3;
  }

  // When is it time to upgrade your LED
  if (--ledcnt == 0) {
    ledcnt = LEDRATE;
    updateLedLogic(getTemperature(), tMin, tMax);
  }
}
// Logic for the display
void display(int8_t temp) {
  uint8_t absTemp = (temp > 0) ? temp : -temp;

  if (absTemp < 10 && temp >= 0) {
    disps[0] = 10;
    disps[1] = 10;
    disps[2] = absTemp;
  } else if (temp < 0 && absTemp < 10) {
    disps[0] = 10;
    disps[1] = 11;
    disps[2] = absTemp;
  } else {
    disps[0] = (temp < 0) ? 11 : 10;
    disps[1] = absTemp / 10;
    disps[2] = absTemp % 10;
  }
}

// Interrupts for buttons
ISR(PCINT1_vect) {
  redNowPressed  = (PINC & (1 << PC1)) == 0;
  blueNowPressed = (PINC & (1 << PC2)) == 0;

  if (!redNowPressed) {
    if (redPressedDuration >= 30 && redPressedDuration < 2000) {
      if (!inEditMode) {
        inEditMode = true;
        editTarget = &tMax;
      } else {
        (*editTarget) += 1;
      }
      displayTimeoutCnt = IDLE_TIMEOUT;
      display(*editTarget);
    }
    redPressedDuration = 0;
  }

  if (!blueNowPressed) {
    if (bluePressedDuration >= 2000 && !inEditMode) {
      inEditMode = true;
      editTarget = &tMin;
      displayTimeoutCnt = IDLE_TIMEOUT;
      display(*editTarget);
    } else if (bluePressedDuration >= 30 && inEditMode) {
      (*editTarget) -= 1;
      displayTimeoutCnt = IDLE_TIMEOUT;
      display(*editTarget);
    }
    bluePressedDuration = 0;
  }
}

void setup() {
  // Setup for the display
  DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
  DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB4);
  PORTB |= (1 << PB1) | (1 << PB2) | (1 << PB4);
  // Set Timer0 to CTC mode
  sbi(TCCR0A, WGM01);  // WGM01 = 1
  cbi(TCCR0A, WGM00);  // WGM00 = 0
  cbi(TCCR0B, WGM02);  // Part of the mode selection

  // Set the compare match value to 1 ms
  OCR0A = 249;

  // Set the prescaler to 64
  sbi(TCCR0B, CS01);
  sbi(TCCR0B, CS00);
  cbi(TCCR0B, CS02);

  //Enable Compare Match A
  sbi(TIMSK0, OCIE0A);

  // Setup for the DHT22 sensor
  TCCR1A = 0;
  TCCR1B = (1 << WGM12);
  OCR1A  = 19;
  TCCR1B |= (1 << CS11);     // Prescaler = 8
  sbi(TIMSK1, OCIE1A);       // Enable Timer1 Match A interrupt appears

  // Setup for the buttons
  DDRC &= ~((1 << PC1) | (1 << PC2));
  PORTC |= (1 << PC1) | (1 << PC2);

  PCICR  |= (1 << PCIE1);                     // Enable interrupts on PCINT state change
  PCMSK1 |= (1 << PCINT9) | (1 << PCINT10);   // Enable interrupts on pins PC1 and PC2

  // Setup for the LED
  DDRB |= (1 << PB3);
  PORTB &= ~(1 << PB3);
}

void loop() {
}
