/*
Stereo LFO for my tremolo pedal.
these are used to drive the VCAs for the tremolo

controls:
- bypass toggle on pin 8, low to trigger
- rotary encoder on pins 2 and 3 to scroll through modes, two clicks to scroll sad!
- speed pot on A0
- depth pot on A1 unused

outputs:
- LFO signals on digital pins 5 and 6.
- rgb out on pins 9, 10, 11. pull low to turn on the leds

Make sure you put a steep lo pass at like ~200Hz on the LFO outs otherwise
you get the PWM whine in the signal, should've used an actual DAC but whatever

skittlemittle 2012
*/

#include <math.h>
#include <Rotary.h>  // https://github.com/brianlow/Rotary

#define NUMMODES 2
#define DEBOUNCE_DELAY 50

const int LFO1 = 5;
const int LFO2 = 6;
const int bypass_button = 8;
const int speed = A0;
//const int depth = A1;

// status led stuff
const int status_pins[] = {9, 10, 11}; //rgb
const int status_colors[NUMMODES][3] = {
  {250, 0, 10}, //"pink1"
  {0, 249, 30}, //"green"
};

/*---- state ---- */
volatile int wave = 0; // which wave are we on
// ugly !
bool s_bypass_button = true;
bool s_last_bypass = false;
unsigned long last_debounce_time = 0;

// A = 2, B = 3
Rotary r_mode = Rotary(2, 3);

void setup()
{
  r_mode.begin();
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();

  pinMode(LFO1, OUTPUT);
  pinMode(LFO2, OUTPUT);
  for (const int p : status_pins)
    pinMode(p, OUTPUT);

  pinMode(bypass_button, INPUT_PULLUP);

  displayWave(wave);
  Serial.begin(9600);
}

/////////////////////////////////////////////////////////////////////////////////////////
// WAVES
/////////////////////////////////////////////////////////////////////////////////////////
// square wave in phase
void toggle()
{
  digitalWrite(LFO1, HIGH);
  digitalWrite(LFO2, HIGH);
  delay(map(analogRead(speed), 0, 1024, 1, 200));
  digitalWrite(LFO1, LOW);
  digitalWrite(LFO2, LOW);
  delay(map(analogRead(speed), 0, 1024, 1, 200));
}

// square wave LFO 1 and 2 pi out of phase
void toggleHarmonic()
{
  digitalWrite(LFO1, HIGH);
  digitalWrite(LFO2, LOW);
  delay(map(analogRead(speed), 0, 1024, 1, 200));
  digitalWrite(LFO1, LOW);
  digitalWrite(LFO2, HIGH);
  delay(map(analogRead(speed), 0, 1024, 1, 200));
}

/////////////////////////////////////////////////////////////////////////////////////////
// UTILS
/////////////////////////////////////////////////////////////////////////////////////////

// cycle throught the waves
void nextwave(const int step) {
  if (wave + step < 0) 
    wave = NUMMODES - 1;
  else
    wave = (wave + step) % NUMMODES;
}

/**
 checks if a button has been pressed and updates passed in state information,
 assumes button is wired as pulldown.
 @param old_state: pointer to old button state
 @param current_state: pointer to current button state
 @param pin: pin the button is connected to
 @returns: true if the button has been pressed, false otherwise
 checkButton(&old, &current, pin);
*/
bool checkButton(bool *old_state, bool *current_state, const int pin)
{
  const bool reading = digitalRead(pin);
  bool ret = false;

  if (reading != *old_state) {
    last_debounce_time = millis();
  }

  if ((millis() - last_debounce_time) > DEBOUNCE_DELAY) {
    if (reading != *current_state) {
      *current_state = reading;
      // return true if state toggles to ON
      if (*current_state)
        ret = true;
    }
  }
  *old_state = reading;
  return ret;
}

/**
 set the mode led color
*/
void displayWave(int wave)
{
  wave = wave % NUMMODES; // we will NOT be crashing at runtime :)
  const int* colors = status_colors[wave];
  for (int i = 0; i < 3; i++) {
    analogWrite(status_pins[i], HIGH);
    delay(1);
    analogWrite(status_pins[i], 255 - colors[i]);
    delay(10);
  }
  delay(10);
}
void ledoff()
{
  for (int i = 0; i < 3; i++)
    digitalWrite(status_pins[i], HIGH);
}

// array of the wave functions
const void (*waves[NUMMODES])() = {toggle, toggleHarmonic};

bool bypass = true;
void loop()
{
  if (checkButton(&s_last_bypass, &s_bypass_button, bypass_button)) {
    bypass = !bypass;
  }

  if (!bypass) {
    (*waves[wave])();
    displayWave(wave);
  } else {
    ledoff();
  }
}


ISR(PCINT2_vect) {
  unsigned char result = r_mode.process();
  if (result == DIR_NONE) {}
  else if (result == DIR_CW) {
//   Serial.println("C");
    nextwave(1);
  }
  else if (result == DIR_CCW) {
//    Serial.println("CC");
    nextwave(-1);
  }
}
