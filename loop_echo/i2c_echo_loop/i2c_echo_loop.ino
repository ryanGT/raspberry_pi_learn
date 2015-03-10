#include "Wire.h"

#define SLAVE_ADDRESS 0x04
#define bufferlen 20
//expect i2c data packets that contain 6 bytes
//with the newline character in the last position
//i.e. index 5
#define packetlen 6
#define nlindex 5

int number = 0;
int state = 0;

const byte mask = B11111000;
int prescale = 1;

int n;
int nIn;
int nISR;
int v1;
int v_out;

int case_in;
int fresh_in;
int fresh_out;
bool send_ser;

int inbuffer [bufferlen];
int in_x = 0;

int outbuffer [bufferlen];
int out_x = 0;

void setup() {
    send_ser = false;
    fresh_in = 0;
    //---------------------------
    //Timer ISR stuff
    //TCCR0A = _BV(COM0A1) | _BV(COM0B1) | 
    //          _BV(WGM00); 
    cli();          // disable global interrupts
    TCCR1A = 0;     // set entire TCCR1A register to 0
    TCCR1B = 0;     // same for TCCR1B

    // set compare match register to desired timer count:
    //OCR1A = 15624;
    //OCR1A = 155;//100 Hz
    //OCR1A = 100;//150ish - seems to work
    OCR1A = 77;//200 Hz <-- seems very borderline (might be 184 Hz)
    //OCR1A = 30;//500 Hz
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
    //----------------------
    pinMode(13, OUTPUT);
    Serial.begin(115200);         // start serial for output
    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    sei();
    Serial.println("i2c krauss buffer v0.1");
}

void read_i2c_buffer() {
  //the last data byte must be set to something other than zero
  //to mark transmission as complete
  while (inbuffer[nlindex] == 0){
    delayMicroseconds(10);
  }
  case_in = inbuffer[0];
  if (case_in  == 1){
    //read nIn and v1 as part of a normal time step
    nIn = reassemblebytes(inbuffer[1], inbuffer[2]);
    v1 = reassemblebytes(inbuffer[3], inbuffer[4]);
  }
  else if (case_in == 2){
    //start new test
    nISR = 0;
  }
  
  //reset inbuffer as clear and ready for new data
  inbuffer[nlindex] = 0;
  in_x = 0;
}

void write_i2c_case1(){
  Wire.write(1);
  SendTwoByteInt(nISR);
  SendTwoByteInt(v_out);
  Wire.write(10);
}

void loop() {
    int i;

    if (fresh_in > 0){
        //new data has arrived
	read_i2c_buffer();
        fresh_in = 0;
        Serial.print("data received: ");
        for (i=0; i<bufferlen; i++){
	    Serial.println(inbuffer[i]);
        }
	fresh_out = 1;
    }

    if (fresh_out > 0){
      write_i2c_case1();
      fresh_out = 0;
    }
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
    Wire.write(msb);
    Wire.write(lsb);
}



// callback for received data
void receiveData(int byteCount){
  //int x = 0;

    while(Wire.available()) {
        inbuffer[in_x] = Wire.read();
        in_x++;
	fresh_in = 1;
        //Serial.print("data received: ");
        //Serial.println(number);

        /* if (number == 1){ */

        /*     if (state == 0){ */
        /*         digitalWrite(13, HIGH); // set the LED on */
        /*         state = 1; */
        /*     } */
        /*     else{ */
        /*         digitalWrite(13, LOW); // set the LED off */
        /*         state = 0; */
        /*     } */
        /*  } */
    }
}

// callback for sending data
void sendData(){
    Wire.write(number);
}

ISR(TIMER1_COMPA_vect)
{     
  nISR++;
  //analogWrite(pwmA, v1);
  v_out = v1*v1;
  //fresh = 1;
}
