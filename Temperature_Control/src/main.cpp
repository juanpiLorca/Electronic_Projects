#include <Arduino.h>
#include <LiquidCrystal.h>
#include <stdio.h>
#include <string.h>

#define RS 17 // P7.4
#define EN 15 // P3.0
#define D4 14 // P3.1
#define D5 13 // P2.6
#define D6 12 // P2.3
#define D7 11 // P8.1

// in charge of sending screen commands & sending data to be shown: 
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7); 

void lcd_display(float T, 
                 float P, 
                 float Kp, 
                 float Ki, 
                 int dec)
{
  // Temperature: 
  lcd.setCursor(2,0);
  lcd.print(T,dec);
  // Power dissipated
  lcd.setCursor(11,0);
  lcd.print(P,dec);
  // Kp:
  lcd.setCursor(3,1);
  lcd.print(Kp,dec); 
  // Ki: 
  lcd.setCursor(12,1);
  lcd.print(Ki,dec);
}

// Funcion que entrega un 0 si la señal es menor al 0.25, 1 si es entre 0.25 y 0.75 y 2 si es mayor a 0.75
int compare(float a){
  int value = 0;
  if (a < 0.25 * 3.3){
    value = 0;
  }
  else if (0.25 * 3.3 < a  and  a < 0.75 * 3.3){
    value = 1;
  }
  else{
    value = 2;
  }
  return value;
}

// Control functions: 

// 1.- duty control
// buttons: 
int buttonPin1 = 0;
int buttonPin2 = 0;  
// leds: 
const int ledPin1 = P1_0;     
const int ledPin2 = P4_7;   
// PWM unitary increase: 
int pwmCounter = 0; 
const int upDuty = 1; 
// flag = 0: Manual control 
// flag = 1: Automatic control 
int flag = 0; 

int manual(int buttonState, int pwmCounter){ 
  if (buttonState == LOW) {
    pwmCounter = pwmCounter + upDuty; 
    if (pwmCounter == 255){
      pwmCounter = 0; 
    }
  }
  return pwmCounter; 
}

// 2.- Bounding manipulate variable (actuators)
float bound(float u, float u_min, float u_max) {
    if (u < u_min) { u = u_min; }
    if (u > u_max) { u = u_max; }
    return u;
}

// control initialization variables: 
float error    = 0; float intError = 0; float errorOld = 0; 
float control  = 0; float Vs       = 0; int dutyCycle  = 0;
// % duty & current: 
float Current; float duty; float Power; 
// Voltage DC motor; 
float Vr = 12.0; 
// step size for integration: sampling time 
float Ts = 0.1; // 0.1 = 100 [ms] 
float current_time, elapsed_time, time_diff; 
// Reference: 
float Tref = 50.0; 
// PI controller gains: 
float Kp, Ki; 
// sensor PIN: 
#define sensorValue 23 // P2.6
// output PIN: 
#define CONTROL_PIN 40 // P2.5

// UART PC control: 
float UartPC(){
  if (Serial.available() > 0){
    String tune = Serial.readStringUntil('='); 
    // Temperature reference update: Tref 
    if (tune == "Tref"|| tune == "tref"){
      Tref = Serial.parseFloat(); 
    }
    // Proportional gain update: Kp
    if (tune == "Kp"|| tune == "kp"){
      Kp = Serial.parseFloat(); 
    }
    // Integral gain update: Ki
    if (tune == "Ki"|| tune == "ki"){
      Ki = Serial.parseFloat(); 
    }
  }
}


// Temperature samples: 
float Temp1 = 0; 
float Temp2 = 0; 
float Temp3 = 0; 
float Temp4 = 0; 
float Temp5 = 0; 
float Temp6 = 0; 
float Temp7 = 0; 
float Temp8 = 0; 
float Temp9 = 0; 


void setup() 
{
  // push 1:
  pinMode(PUSH1, INPUT_PULLUP); 
  // push 2: 
  pinMode(PUSH2, INPUT_PULLUP);
   // led 1: 
  pinMode(ledPin1, OUTPUT);
  // led 2: 
  pinMode(ledPin2, OUTPUT);

  // LCD set-up: 16 columns & 2 rows
  lcd.begin(16,2);  
  lcd.clear();

  // LCD set up: 
  // Temperature: 
  lcd.setCursor(0,0);
  lcd.print("T:");
  // Power dissipated
  lcd.setCursor(9,0);
  lcd.print("P:");
  // Kp:
  lcd.setCursor(0,1);
  lcd.print("Kp:");
  // Ki: 
  lcd.setCursor(9,1);
  lcd.print("Ki:");

  // Control set-up: 
  // Proportional gain:
  Kp = 25; 
  // Integral gain:
  Ki = 0.01; 
  // Default output (PWM rate ~2kHz):
  pinMode(CONTROL_PIN, OUTPUT); 
  analogFrequency(2000); 
  
  // Voltage measurement:
  Serial.begin(9600);
}

void loop() 
{
  // 1.- set current time: 
  current_time = millis(); 

  // 2.- read temperature sensor: 
  float digitalVt = analogRead(sensorValue); 
  float Vt = digitalVt * ((3.3 - 0.0)/4095); 
  float T = Vt * 50.997 - 44.28; 
  Temp1 = T; 
  Temp2 = Temp1; 
  Temp3 = Temp2; 
  Temp4 = Temp3; 
  Temp5 = Temp4; 
  Temp6 = Temp5; 
  Temp7 = Temp6; 
  Temp8 = Temp7; 
  Temp9 = Temp8; 
  float Temp = (Temp1 + Temp2 + Temp3 + Temp4 + Temp5 + Temp6 + Temp7 + Temp8 + Temp9) / 10; 

  // 3.- PC command; 
  UartPC(); 

  buttonPin1 = digitalRead(PUSH1); 
  if (buttonPin1 == LOW){
    dutyCycle = 0;
    if (flag == 0){
      flag = 1; 
    } 
    else if (flag == 1){
      flag = 0; 
    } 
  }

  if (flag == 0){
    digitalWrite(ledPin1, LOW); 
    error = Tref - Temp;
    // 4.- PI control: 
    intError += error * Ts;
    // bound intError to [-1...1]
    intError = bound(intError, -25, 25);
    control += Kp*(error - errorOld) + Ki*intError;
    errorOld = error;
    // bound control signal:
    control = bound(control, 0, 1000);
    dutyCycle = (255*control)/1000;
    //dutyCycle=round(control); 
    // control PWM ~2.0 kHz
    analogWrite(CONTROL_PIN, dutyCycle); 
    // current mapping: 
    Current = (0.362/255)*dutyCycle - 0.0804; 
    // power computing: 
    Power = Vr * Current; 
    elapsed_time = (millis() - current_time)/1000;
    time_diff = Ts - elapsed_time; 
  } 

  else if (flag == 1){
    buttonPin2 = digitalRead(PUSH2); 
    digitalWrite(ledPin1, HIGH); 
    dutyCycle = manual(buttonPin2, dutyCycle); 
    // control
    analogWrite(CONTROL_PIN, dutyCycle); 
    // current mapping: 
    Current = (0.362/255)*dutyCycle - 0.0804; 
    // power computing: 
    Power = Vr * Current;
  }

  delay(time_diff); 

  // Display: 
  lcd_display(Temp, Power, Kp, Ki, 1); 

  Serial.print("temperature = "); Serial.print(Temp); Serial.print("°C");
  Serial.print("\tTref = "); Serial.print(Tref); Serial.print("°C");
  Serial.print("\tKp = "); Serial.print(Kp); 
  Serial.print("\tKi = "); Serial.print(Ki); 
  Serial.print("\tcontrol = "); Serial.print(control); 
  Serial.print("\tduty = "); Serial.print(dutyCycle);
  Serial.print("\n");
}


/******** Timer Interruptions ********/
/*
#include <msp430.h>

int main(void)
{
    // Stop the watchdog timer
    WDTCTL = WDTPW + WDTHOLD;

    // Configure Timer_A
    TA0CCTL0 = CCIE;            // Enable Timer A interrupt
    TA0CCR0 = 1000;             // Set the timer period (adjust as needed)
    TA0CTL = TASSEL_2 + MC_1;   // Use the SMCLK as the clock source and set the mode to up mode

    // Enable global interrupts
    __enable_interrupt();

    // Your main code here

    while(1)
    {
        // Your main loop code
    }
}

// Timer A interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
    // Your timer interrupt code here
}

*/
/****************/