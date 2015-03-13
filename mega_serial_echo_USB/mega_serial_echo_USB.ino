#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define isrPin 5
#define sendPin 8
#define receivePin 7
#define triggerPin 12

const byte mask = B11111000;
int prescale = 1;

int n;
int nIn;
int nISR;
int v1;
int v_out;

int inByte;
int fresh;
bool send_ser;

void setup()
{
  Serial.begin(115200);

  Serial.print("mega serial1 echo test USB");
  Serial.print("\n");

  //Serial1.begin(115200);

  send_ser = false;


  pinMode(isrPin, OUTPUT);
  pinMode(sendPin, OUTPUT);
  pinMode(receivePin, OUTPUT);
  pinMode(triggerPin, OUTPUT);

  digitalWrite(isrPin, LOW);
  digitalWrite(receivePin, LOW);
  digitalWrite(triggerPin, LOW);
  digitalWrite(sendPin, LOW);

  // turn on pullup resistors
  //digitalWrite(encoderPinA, HIGH);
  //digitalWrite(encoderPinB, HIGH);


  //TCCR0A = _BV(COM0A1) | _BV(COM0B1) | 
  //          _BV(WGM00); 
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B

  // set compare match register to desired timer count:
  //OCR1A = 15624;
  //OCR1A = 155;//100 Hz
  //OCR1A = 100;//150ish - seems to work
  //OCR1A = 77;//200 Hz <-- seems very borderline (might be 184 Hz)
  OCR1A = 30;//500 Hz
  //OCR1A = 15;//1000 Hz
  //OCR1A = 7;//2000 Hz


  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);

  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
 
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);


  TCCR0B = _BV(CS00);
  TCCR0B = (TCCR0B & mask) | prescale;

  sei();
}

unsigned char getsecondbyte(int input){
    unsigned char output;
    output = (unsigned char)(input >> 8);
    return output;
}

 

int reassemblebytes(unsigned char msb, unsigned char lsb){
    int output;
    output = (int)(msb << 8);
    output += lsb;
    return output;
}

int readtwobytes(void){
    unsigned char msb, lsb;
    int output;
    int iter = 0;
    while (Serial.available() <2){
      iter++;
      if (iter > 1e5){
	break;
      }
    }
    msb = Serial.read();
    lsb = Serial.read();
    output = reassemblebytes(msb, lsb);
    return output;
}

void SendTwoByteInt(int intin){
    unsigned char lsb, msb;
    lsb = (unsigned char)intin;
    msb = getsecondbyte(intin);
    Serial.write(msb);
    Serial.write(lsb);
}



void loop()
{
  if (Serial.available() > 0) {
    digitalWrite(readPin, HIGH);
    inByte = Serial.read();
    if (inByte == 1){
      //main control case
      //send_ser = true;
      nIn = readtwobytes();
      v1 = readtwobytes();
    }
    else if (inByte == 2){
      //start new test
      send_ser = true;
      nISR = -1;
    }
    else if (inByte == 3){
      send_ser = false;
      v1 = 0;
    }
    digitalWrite(readPin, LOW);
  }
  
  if (fresh > 0){
    fresh = 0;
    if (send_ser){
      digitalWrite(sendPin, HIGH);
      //send_ser = false;
      SendTwoByteInt(nISR);
      SendTwoByteInt(v_out);
      Serial.write(10);
      digitalWrite(sendPin, LOW);
    }
    if (nISR > 500){
      send_ser = false;
      v1 = 0;
    }
  }
}


ISR(TIMER1_COMPA_vect)
{     
  digitalWrite(isrPin, HIGH);
  nISR++;
  //analogWrite(pwmA, v1);
  v_out = v1*v1;
  fresh = 1;
  digitalWrite(isrPin, LOW);
}
