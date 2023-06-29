#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2);
#define PWM_out 10 // pwm signal is taken out from digital pin `10 of arduino



////////////symbols to display in screen////////////////////


uint8_t Battery[8]  = {0x0E, 0x1B, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F};
uint8_t Panel[8]  = {0x1F, 0x15, 0x1F, 0x15, 0x1F, 0x15, 0x1F, 0x00};
uint8_t Pwm[8]  = {0x1D, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x17};
uint8_t Flash[8]  = {0x01, 0x02, 0x04, 0x1F, 0x1F, 0x02, 0x04, 0x08};
byte customChar1[8] = { 0b00100, 0b01110, 0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100};

////////////////////battery Constants///////////////////////////////////////////////
#define bulk_voltage_max 14.5
#define bulk_voltage_min 13
#define absorption_voltage 14.7
#define float_voltage_max 13
#define battery_min_voltage 10
#define solar_min_voltage 19
#define charging_current 2.0
#define absorption_max_current 2.0
#define absorption_min_current 0.1
#define float_voltage_min 13.2
#define float_voltage 13.4
#define float_max_current 0.12
#define LCD_refresh_rate 1000
byte BULK = 0;        //Give values to each mode
byte ABSORPTION = 1;
byte FLOAT = 2;
byte mode = 0;        //We start with mode 0 BULK
String mode_str = "BULK";


//////pin definition ////////////////////////////////////
#define v_in_pin A6
#define i_in_pin A7
#define v_out_pin A2
#define i_out_pin A3



float error = 0, Setpoint = 20;
float pwm_value = 0;
float input_voltage, battery_voltage, input_current, output_current, input_power, output_power, ofset_voltage = 2.52, current_factor = 0.185;
float power_previous = 0, voltage_previous = 0;
unsigned long int pre;


//-------------------------------------setup---------------------------------------//
void setup() {
  // put your setup code here, to run once:
  pinMode(PWM_out, OUTPUT);           //Set pins as OUTPUTS
  digitalWrite(PWM_out, LOW);
  TCCR1B = TCCR1B & B11111000 | B00000001; //timer 1 PWM frequency of 31372.55 Hz
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, Battery);
  lcd.createChar(1, Panel);
  lcd.createChar(2, Pwm);
  lcd.createChar(3, Flash);
  lcd.createChar(4, customChar1);
  pre = millis();
  //  delay(100);
}

//----------------------------------main loop------------------------------------------

void loop() {

  input_voltage =   get_voltage(v_in_pin, 10);
  battery_voltage =  get_voltage(v_out_pin, 10); 
  output_current =  get_current(i_out_pin, 10);
  output_power = battery_voltage * output_current;
  
  int pwm_percentage = map(pwm_value, 0, 255, 0, 100);


  //---------------------------------------print value on lcd -----------------------------
  if ((millis() - pre) > 1000)
  {

    lcd.clear();
    pre = millis();
    lcd.setCursor(0, 0);              //Column 0 row 0
    lcd.write(1);                     //Panel icon
    lcd.print(" ");                   //Empty space
    lcd.print(input_voltage, 2);      //Soalr voltage
    lcd.print("V");                   //Volts
    lcd.print(" ");                //Empty spaces
    lcd.write(0);                     //Battery icon
    lcd.print(" ");                   //Empty space
    lcd.print(battery_voltage, 2);        //Battery voltsge
    lcd.print("V");                   //Volts

    lcd.setCursor(0, 1);              //Column 0 row 1
    lcd.write(4);
    lcd.print(" ");
    lcd.print(input_current, 2);      //Solar current
    lcd.print("A");

    lcd.setCursor(12, 1);

    lcd.write(4);//Empty spaces
    lcd.print(" ");
    lcd.print(output_current, 2);      //charging  current
    lcd.print("A");//Ampers

    lcd.setCursor(0, 2);
    lcd.print("P"); //Column 2 row 1
    lcd.print(" ");
    lcd.print(input_power, 2);      //Solar current
    lcd.print("W");

    lcd.setCursor(12, 2);
    lcd.print("P"); //Column 2 row 1
    lcd.print(" "); //Empty spaces
    lcd.print(output_power, 2);      //charging  current
    lcd.print("W");//Ampers
    //Watts
//    lcd.setCursor(0, 3);
//    lcd.print("PWM ");           //Print PWM
//    lcd.print(pwm_percentage);        //PWM value
//    lcd.print("%");                   //Percentage

//    lcd.setCursor(12, 3);               //Column 0 row 3
//    lcd.print(mode_str);/

  }
  error=battery_voltage-Setpoint;
  if(error<0)
  {
    pwm_value++;
  }
  else if(error>0){
    pwm_value--;
  }
  else
  {
    pwm_value=pwm_value;
  }
  pwm_value=constrain(pwm_value, 10, 220);
  analogWrite(PWM_out,pwm_value);


}


//------------------------voltage measurement function-------------------------------
float get_voltage(int pin, int n_samples)
{
  float voltage = 0;
  for (int i = 0; i < n_samples; i++)
  {
    //    Serial.println( analogRead(pin));
    voltage += ((analogRead(pin)) * (5.0 / 1024.0) * 23.0);//-4
    delay(2);
  }
  voltage = voltage / n_samples;
  if (voltage < 0) {
    voltage = 0;
  }
  return (voltage);
}



//------------------------------current measurement function-------------------------------
float get_current(int pin , int n_samples)
{
  float Sensor_voltage;
  float current = 0;
  for (int i = 0; i < n_samples; i++)
  {
    Sensor_voltage = ((analogRead(pin)) * (5.0 / 1024.0));//4.55
    Serial.println(Sensor_voltage);
    current = current + (Sensor_voltage - ofset_voltage) / current_factor;
    delay(2);
  }
  current = current / n_samples;
  //  Serial.print(" load current : ");
  //  Serial.print(current);
  return (current);
}


//-----------------------to clear lcd by line---------------------
void clearLCDLine(int line)
{
  lcd.setCursor(0, line);
  for (int n = 0; n < 20; n++) // 20 indicates symbols in line. For 2x16 LCD write - 16
  {
    lcd.print(" ");
  }
}
