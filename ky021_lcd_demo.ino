//
// Example code for sensor KY021
// More info on http://tkkrlab.nl/wiki/Arduino_KY-021_Mini_magnetic_reed_modules
//
//    Gerkon module KY-021
//    Arduino GND --> Module pin -
//    Arduino +5V --> Module PLUS (middle pin)
//    Arduino Digital pin 3 --> Module S

#define LCDbacklight() pinMode(10, INPUT) // turn on backlight
#define LCDnoBacklight() pinMode(10, OUTPUT) // turn off backlight

#include <EEPROM.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#include <FrequencyTimer2.h>
#define TIMER_PERIOD 32768

namespace _heartPipeline {
  void timer2_interrupt();
};

namespace Pins {
  const byte REED = 3,
             LIGHT_CONTROL = 2,
             LIGHT_LEVEL = 15,
             ADD_GROUND = 17,
             ADD_GROUND2 = 16,
             ADD_VCC = 18,
             PULSE_IN = 19,
             EEPROM_RADIUS_ADDR = 0;
             // PULSE = 3;  // analog
};

namespace SensorValues {
  int ky021 = 0;
  int ky021_prev = 0;
  int lcd_buttons = 0;
  int light_level = 0;
};

namespace State {
  int reed_cycles = 0;
  byte light_on = 0;
  byte lcd_backlight = 0;
  byte lcd_backlight_can_change = 1;
  byte left_key_served = 0;
  byte up_key_served = 0;
  byte down_key_served = 0;
  byte right_key_served = 0;
  byte bpm = 60;
  byte radius = 10;
  float speed_ = 0;
};

#define select_is_pressed() (SensorValues::lcd_buttons < 650 \
                             && SensorValues::lcd_buttons > 600)
#define left_is_pressed() (SensorValues::lcd_buttons < 420 \
                           && SensorValues::lcd_buttons > 400)
#define up_is_pressed() (SensorValues::lcd_buttons < 110 \
                         && SensorValues::lcd_buttons > 90)
#define down_is_pressed() (SensorValues::lcd_buttons < 265 \
                           && SensorValues::lcd_buttons > 245)
#define right_is_pressed() (SensorValues::lcd_buttons < 20)
                           
void select_button_handler();
// void left_button_handler();
void light_sensor_handler(int light_level);
// void interruptSetup();
void speedometer_handler();
void radius_handler();
void bike_light_mode_handler();

void setup ()
{
  lcd.begin(16, 2);
  pinMode(Pins::REED, INPUT);  // output interface as defined Reed sensor
  pinMode(Pins::LIGHT_LEVEL, INPUT);
  pinMode(Pins::PULSE_IN, INPUT);
  
  pinMode(Pins::LIGHT_CONTROL, OUTPUT);
  digitalWrite(Pins::LIGHT_CONTROL, LOW);

  pinMode(Pins::ADD_GROUND, OUTPUT);
  digitalWrite(Pins::ADD_GROUND, LOW);
  pinMode(Pins::ADD_GROUND2, OUTPUT);
  digitalWrite(Pins::ADD_GROUND2, LOW);
  pinMode(Pins::ADD_VCC, OUTPUT);
  digitalWrite(Pins::ADD_VCC, HIGH);

  State::radius = EEPROM.read(Pins::EEPROM_RADIUS_ADDR);
  FrequencyTimer2::setPeriod(TIMER_PERIOD /* microseconds */);
  FrequencyTimer2::setOnOverflow(_heartPipeline::timer2_interrupt);
  
  Serial.begin(9600);
}

void loop ()
{
  SensorValues::ky021_prev = SensorValues::ky021;
  SensorValues::lcd_buttons = analogRead(0);
  SensorValues::ky021 = digitalRead(Pins::REED);
  SensorValues::light_level = analogRead(Pins::LIGHT_LEVEL);

  lcd.setCursor(0, 0);
  lcd.print("R=");
  lcd.print(State::radius); 
  lcd.print("cm ");
  lcd.print(SensorValues::light_level / 10);
  lcd.print(" ");
  lcd.print(State::reed_cycles);
  lcd.print(" ");
  lcd.setCursor(0, 1);
  lcd.print("BPM ");
  lcd.print(State::bpm);
  lcd.print(State::bpm < 100? "  " : " ");
  lcd.setCursor(8, 1);
  lcd.print(State::speed_);
  lcd.print(" ");
  lcd.setCursor(13, 1);
  lcd.print("KMH");

  select_button_handler();
  light_sensor_handler(SensorValues::light_level);
  speedometer_handler();
  radius_handler();
  bike_light_mode_handler();
}

void bike_light_mode_handler()
{
  if (!State::right_key_served && State::light_on && right_is_pressed())
  {
    delay(100); if (!right_is_pressed()) return;
    digitalWrite(Pins::LIGHT_CONTROL, LOW);
    delay(150);
    digitalWrite(Pins::LIGHT_CONTROL, HIGH);
    State::right_key_served = 1;
  }
  if (!right_is_pressed())
    State::right_key_served = 0;
}


void radius_handler()
{
  if (up_is_pressed() && !State::up_key_served)
  {
    State::radius += 1;
    State::up_key_served = 1;
    EEPROM.write(Pins::EEPROM_RADIUS_ADDR, State::radius);
  }
  if (!up_is_pressed())
    State::up_key_served = 0;

  if (down_is_pressed() && !State::down_key_served)
  {
    State::radius -= 1;
    State::down_key_served = 1;
    EEPROM.write(Pins::EEPROM_RADIUS_ADDR, State::radius);
  }
  if (!down_is_pressed())
    State::down_key_served = 0;
}

byte _i_switch = 0;
byte _ready = 0;
long long int _speedometer_switches[5];
void speedometer_handler() 
{
  if (SensorValues::ky021 == LOW 
      && SensorValues::ky021_prev != SensorValues::ky021)
  {
    State::reed_cycles++;
    _speedometer_switches[_i_switch] = millis();
    _i_switch++;

    if (_i_switch == 5)
    {
      _ready = 1;
      _i_switch = 0;
    }
    if (_ready)
    {
      // Calculate speed.
      int total_delta = 0;
      for (int i = 1; i < 5; i++) total_delta += (
          _speedometer_switches[i] - _speedometer_switches[i-1]
      );
      total_delta /= 4;      
      State::speed_ = 36.0*2.0*3.14*State::radius / total_delta;
      _ready = 0;
    }
  }
  if (!_ready && millis() - _speedometer_switches[max(_i_switch-1, 0)] > 20000)
  {
    _ready = 0;
    _i_switch = 0;
    State::speed_ = 0;
  }
}


void select_button_handler()
{
  if (select_is_pressed() && State::lcd_backlight && State::lcd_backlight_can_change)
  { 
    LCDnoBacklight(); 
    State::lcd_backlight ^= 1; 
    State::lcd_backlight_can_change = 0; 
  }
  if (select_is_pressed() && !State::lcd_backlight && State::lcd_backlight_can_change)
  { 
    LCDbacklight(); 
    State::lcd_backlight ^= 1; 
    State::lcd_backlight_can_change = 0;
  }
  if (SensorValues::lcd_buttons > 1000) 
    State::lcd_backlight_can_change = 1;
}


void light_sensor_handler(int light_level)
{ 
  if (light_level > 700)
    if (!State::light_on) 
    {
      delay(1000); if (analogRead(Pins::LIGHT_LEVEL) < 700) return;
      digitalWrite(Pins::LIGHT_CONTROL, HIGH);  delay(150);
      digitalWrite(Pins::LIGHT_CONTROL, LOW);   delay(150);
      digitalWrite(Pins::LIGHT_CONTROL, HIGH);  delay(150);
      digitalWrite(Pins::LIGHT_CONTROL, LOW);   delay(150);
      digitalWrite(Pins::LIGHT_CONTROL, HIGH);
      State::light_on = 1;
    }
    
  if (light_level < 600)
    if (State::light_on) 
    {
      delay(500); if (analogRead(Pins::LIGHT_LEVEL) > 600) return;
      digitalWrite(Pins::LIGHT_CONTROL, LOW);
      State::light_on = 0;
    }
}

namespace _heartPipeline {
  const float heart_beat2[41] = {
    -11.39,-12.39,-11.39,-8.390,-5.390,-2.390,-0.390,1.6097,
    2.6097,2.6097,1.6097,-0.390,-1.390,-3.390,-4.390,-5.390,
    -7.390,-8.390,-7.390,-6.390,-5.390,-4.390,-4.390,-5.390,
    -3.390,0.6097,7.6097,12.609,18.609,22.609,24.609,24.609,
    19.609,14.609,8.6097,-0.390,-3.390,-7.390,-10.39,-10.39,
    -11.39};
  const float heart_sq2 = 4297.756;

  const float heart_beat1[41] = {
    -9.634,-10.63,-8.634,-8.634,-6.634,-3.634,-2.634,-0.634,
    1.3658,1.3658,1.3658,0.3658,-0.634,-1.634,-2.634,-2.634,
    -1.634,0.3658,5.3658,10.365,15.365,18.365,20.365,18.365,
    14.365,12.365,6.3658,2.3658,-3.634,-6.634,-8.634,-10.63,
    -10.63,-9.634,-8.634,-5.634,-2.634,-1.634,-0.634,0.3658,
    0.3658};
  const float heart_sq1 = 2880;
  
  int data[41];
  float data_norm[41];
  int j = 0;
  
  
  float mean(int* data, int len)
  {
    int sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return float(sum)/len;
  }
  
  void vadd(int* in, int len, int num, float* out)
  {
    for(int i = 0; i < len; i++)
      out[i] = in[i] + num;
  }
  
  float correlate(int* data, const float* beat, const float beat_sq, int len)
  {
    float m = mean(data, len);
    vadd(data, len, -m, data_norm);
    
    float numerator = 0;
    for (int i = 0; i < len; i++)
      numerator += data_norm[i] * beat[i];
  
    float denominator = 0;
    for (int i = 0; i < len; i++)
      denominator += data_norm[i] * data_norm[i];
    denominator *= beat_sq;
    denominator = pow(denominator, .5);
  
    return numerator / denominator;
  }
  
  float diff_mean(long long int* data, int len)
  {
    int sum = 0;
    for (int i = 1; i < len; i++)
      sum += data[i] - data[i-1];
    return (float)sum / (len-1);
  }
  
  float corr = 0, corr_pr = 0;
  const int BEATS_LEN = 20;
  long long int beats[BEATS_LEN];
  
  void timer2_interrupt()  // timer0 overflow interrupt
  {
    for (int i = 1; i < 41; i++)
        data[i-1] = data[i];
    data[40] = analogRead(Pins::PULSE_IN);

    corr_pr = corr;
    corr = correlate(data, heart_beat1, heart_sq1, 41);
    if (corr_pr < 0.75 && corr >= 0.75)
    {  
        for (int i = 1; i < BEATS_LEN; i++) 
            beats[i-1] = beats[i];
        beats[BEATS_LEN-1] = millis();
        State::bpm = max(0, 60000 / diff_mean(beats, BEATS_LEN));
        State::bpm = min(State::bpm, 250);
    }
  }
}
