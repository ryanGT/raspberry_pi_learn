
#include <SPI.h>

volatile boolean process_it;
byte c;

void setup (void)
{
  Serial.begin (115200);   // debugging

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);

  pinMode(6, OUTPUT);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  
  // get ready for an interrupt 
  process_it = false;

  // now turn on interrupts
  SPI.attachInterrupt();

}  // end of setup


// SPI interrupt routine
ISR (SPI_STC_vect)
{
  c = SPDR;  // grab byte from SPI Data Register
  
  digitalWrite(6,HIGH);
  delay(1000);
  digitalWrite(6,LOW);
  // add to buffer if room
  process_it = true;
}  // end of interrupt routine SPI_STC_vect

// main loop - wait for flag set in interrupt routine
void loop (void)
{
  if (process_it)
  {
    //digitalWrite(SS, LOW);
    SPI.transfer (c);
    //digitalWrite(SS, HIGH);
    process_it = false;

  }  // end of flag set

}  // end of loop

