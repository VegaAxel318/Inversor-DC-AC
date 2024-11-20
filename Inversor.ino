#include <avr/io.h>
#include <avr/interrupt.h>

// Look up tables with 200 entries each, normalised to have max value of 1600 which is the period of the PWM loaded into register ICR1.
long lookUp1[] = {
50 ,100 ,151 ,201 ,250 ,300 ,349 ,398 ,446 ,494 ,
542 ,589 ,635 ,681 ,726 ,771 ,814 ,857 ,899 ,940 ,
981 ,1020 ,1058 ,1095 ,1131 ,1166 ,1200 ,1233 ,1264 ,1294 ,
1323 ,1351 ,1377 ,1402 ,1426 ,1448 ,1468 ,1488 ,1505 ,1522 ,
1536 ,1550 ,1561 ,1572 ,1580 ,1587 ,1593 ,1597 ,1599 ,1600 ,
1599 ,1597 ,1593 ,1587 ,1580 ,1572 ,1561 ,1550 ,1536 ,1522 ,
1505 ,1488 ,1468 ,1448 ,1426 ,1402 ,1377 ,1351 ,1323 ,1294 ,
1264 ,1233 ,1200 ,1166 ,1131 ,1095 ,1058 ,1020 ,981 ,940 ,
899 ,857 ,814 ,771 ,726 ,681 ,635 ,589 ,542 ,494 ,
446 ,398 ,349 ,300 ,250 ,201 ,151 ,100 ,50 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0};

long lookUp2[] = {
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
50 ,100 ,151 ,201 ,250 ,300 ,349 ,398 ,446 ,494 ,
542 ,589 ,635 ,681 ,726 ,771 ,814 ,857 ,899 ,940 ,
981 ,1020 ,1058 ,1095 ,1131 ,1166 ,1200 ,1233 ,1264 ,1294 ,
1323 ,1351 ,1377 ,1402 ,1426 ,1448 ,1468 ,1488 ,1505 ,1522 ,
1536 ,1550 ,1561 ,1572 ,1580 ,1587 ,1593 ,1597 ,1599 ,1600 ,
1599 ,1597 ,1593 ,1587 ,1580 ,1572 ,1561 ,1550 ,1536 ,1522 ,
1505 ,1488 ,1468 ,1448 ,1426 ,1402 ,1377 ,1351 ,1323 ,1294 ,
1264 ,1233 ,1200 ,1166 ,1131 ,1095 ,1058 ,1020 ,981 ,940 ,
899 ,857 ,814 ,771 ,726 ,681 ,635 ,589 ,542 ,494 ,
446 ,398 ,349 ,300 ,250 ,201 ,151 ,100 ,50 ,0};

// Parámetros para el control PID
float Kp = 3.0;
float Ki = 0.0;
float Kd = 0.10;

float integral = 0;
float lastError = 0;

//Variables de filtro de media móvil
const int numReadings = 30;
float readings[numReadings];
int readIndex = 0;
float total;

// Variables de control
const float desiredVoltage = 600; // Voltaje objetivo (ajuste necesario según el ADC)
const int deadTime = 10;        // Tiempo muerto en cuentas del temporizador
float feedbackVoltage;            // Valor de voltaje leído del ADC
float outputPWM;


void setup(){
  Serial.begin(9600);
    // Register initilisation, see datasheet for more detail.
    TCCR1A = 0b10100010;
       /*10 clear on match, set at BOTTOM for compA.
         10 clear on match, set at BOTTOM for compB.
         00
         10 WGM1 1:0 for waveform 15.
       */
    TCCR1B = 0b00011001;
       /*000
         11 WGM1 3:2 for waveform 15.
         001 no prescale on the counter.
       */
    TIMSK1 = 0b00000001;
       /*0000000
         1 TOV1 Flag interrupt enable. 
       */
    ICR1   = 1333;     // Period for 16MHz crystal, for a switching frequency of 100KHz for 200 subdevisions per 50Hz sin wave cycle.
    sei();             // Enable global interrupts.
    DDRB = 0b00000110; // Set PB1 and PB2 as outputs.
    pinMode(13,OUTPUT);
}

void loop(){
  // Leer el voltaje de salida
    feedbackVoltage = lecturaFiltrada(A1);
    Serial.print(feedbackVoltage);
    Serial.print(",");
    // Calcular el error y actualizar el PID
    float error = desiredVoltage - feedbackVoltage;
    Serial.println(error);
    integral += error;

    if(integral>1800) integral=1800;
    if(integral< 500) integral =500;

    float derivative = error - lastError;
    outputPWM = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Limitar la salida del PID al rango permitido
    if (outputPWM > ICR1) outputPWM = ICR1;
    if (outputPWM < 400) outputPWM = 400;
    
    // Actualizar el último error
    lastError = error;
}

ISR(TIMER1_OVF_vect){
    static int num;

    long dutyCycleA = (lookUp1[num] * outputPWM) / 1333;  // Ajuste según el PID
    long dutyCycleB = (lookUp2[num] * outputPWM) / 1333;  // Ajuste según el PID con tiempo muerto

    // Limitar dutyCycleA y dutyCycleB a valores entre 0 e ICR1
    if (dutyCycleA < 0) dutyCycleA = 0;                // Evitar valores negativos en dutyCycleA
    if (dutyCycleA > ICR1) dutyCycleA = ICR1;          // Limitar a valor máximo permitido
    
    if (dutyCycleB < 0) dutyCycleB = 0;                // Evitar valores negativos en dutyCycleB
    if (dutyCycleB > ICR1) dutyCycleB = ICR1;          // Limitar a valor máximo permitido

    // Asignar valores ajustados a OCR1A y OCR1B
    OCR1A = dutyCycleA;
    OCR1B = dutyCycleB;
    
    if(++num >= 200){ // Pre-increment num then check it's below 200.
       num = 0;       // Reset num.
     }
}

float lecturaFiltrada(int pin) {
  float average;
  ///Aplicacion de filtro de media movil
  // restar ultima lectura
  total = total - readings[readIndex];

  readings[readIndex] = analogRead(pin);
  // sumar nueva lectura:
  total = total + readings[readIndex];

  // incrementar indice
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  // calcular promedio:
  average = total / numReadings;
  return average;
}
