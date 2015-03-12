#include "Wire.h"

#define SLAVE_ADDRESS 0x04
#define bufferlen 10
//expect i2c data packets that contain 6 bytes
//with the newline character in the last position
//i.e. index 5
#define receivelen 6
#define sendlen 6
#define nlindex 5

#define receivePin 7
#define sendPin 8

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

uint8_t inbuffer [bufferlen];
int in_x = 0;
int bytesin;

uint8_t outbuffer [sendlen];
int out_x = 0;
int out_byte;
int out_ready = 0;

void setup() {
    send_ser = false;
    fresh_in = 0;
    number = 7;
    //---------------------------
    //Timer ISR stuff
    //TCCR0A = _BV(COM0A1) | _BV(COM0B1) | 
    //          _BV(WGM00); 
    cli();          // disable global interrupts
    TCCR1A = 0;     // set entire TCCR1A register to 0
    TCCR1B = 0;     // same for TCCR1B

    // set compare match register to desired timer count:
    OCR1A = 15624;
    //OCR1A = 155;//100 Hz
    //OCR1A = 100;//150ish - seems to work
    //OCR1A = 77;//200 Hz <-- seems very borderline (might be 184 Hz)
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
    pinMode(sendPin, OUTPUT);
    pinMode(receivePin, OUTPUT);

    digitalWrite(receivePin, LOW);
    digitalWrite(sendPin, LOW);

    Serial.begin(115200);         // start serial for output
    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    sei();
    Serial.println("i2c krauss buffer v0.2.3.1 3/12/15");
}

void read_i2c_buffer() {
  //the last data byte must be set to something other than zero
  //to mark transmission as complete
  while (inbuffer[nlindex] == 0){
    delayMicroseconds(3);
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
  unsigned char nlsb, nmsb, vlsb, vmsb;
  out_ready = 0;
  outbuffer[0] = 0;

  nlsb = (unsigned char)nISR;
  nmsb = getsecondbyte(nISR);
  outbuffer[1] = nmsb;
  outbuffer[2] = nlsb;

  vlsb = (unsigned char)v_out;
  vmsb = getsecondbyte(v_out);
  outbuffer[3] = vmsb;
  outbuffer[4] = vlsb;

  outbuffer[5] = 10;
  outbuffer[0] = 1;
  out_ready = 1;
  out_x = 0;
}

void loop() {
    int i;

    delay(100);

    if (fresh_in > 0){
        //new data has arrived
        //digitalWrite(receivePin, HIGH);
        //pause i2c transmission from Arduino:
        //turn_off_transmit();
	//read_i2c_buffer();
        fresh_in = 0;
	/* Serial.print("bytesin: "); */
	/* Serial.println(bytesin); */

        /* Serial.print("data received: "); */
        /* for (i=0; i<bufferlen; i++){ */
	/*     Serial.println(inbuffer[i]); */
        /* } */
	//digitalWrite(receivePin, LOW);
    }

    /* if (fresh_out > 0){ */
    /*   write_i2c_case1(); */
    /*   fresh_out = 0; */
    /* } */
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


void turn_off_transmit(){
  out_x = 0;
  outbuffer[0] = 0;
  out_ready = 0;
}

// callback for received data
void receiveData(int byteCount){
  //int x = 0;
    //Serial.print("byte count=");
    //Serial.println(byteCount);

    //Serial.print("in_x=");
    //Serial.println(in_x);

    //Serial.print("data received: ");

    digitalWrite(receivePin, HIGH);
    bytesin = byteCount;
    in_x = 0;

    while(Wire.available()) {
        inbuffer[in_x] = Wire.read();
        //Serial.println(inbuffer[in_x]);
        in_x++;
    }
    
    fresh_in = 1;

    digitalWrite(receivePin, LOW);

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

// callback for sending data
void sendData(){
    unsigned char nlsb, nmsb, vlsb, vmsb;

    digitalWrite(sendPin, HIGH);
    nlsb = (unsigned char)nISR;
    //nmsb = getsecondbyte(nISR);
    outbuffer[0] = nlsb;

    vlsb = (unsigned char)v_out;
    vmsb = getsecondbyte(v_out);
    outbuffer[1] = vmsb;
    outbuffer[2] = vlsb;

    outbuffer[3] = 5;
    outbuffer[4] = 10;
    
    outbuffer[5] = 0;

    Wire.write(outbuffer, sendlen);

    digitalWrite(sendPin, LOW);
}

ISR(TIMER1_COMPA_vect)
{     
  nISR++;
  //analogWrite(pwmA, v1);
  //v_out = v1*v1;
  //fresh_out = 1;
  //out_ready = 0;
}
