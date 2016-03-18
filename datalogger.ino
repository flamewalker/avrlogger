/**
 *  I2C interface to SPI for CTC Ecologic EXT
**/
#include <Wire.h>

#define LED_SYNC 8
#define LED_SAMPLE 9

#define ARRAY_SIZE 0xDC

// State machine declarations for I2C
enum I2CState
{
  I2C_IDLE,
  I2C_RESPONSE,
  I2C_SAMPLE,
  I2C_REQUEST,
  I2C_COMMAND
};

volatile uint8_t i2c_state = I2C_IDLE;

// Command buffer for I2C ISR.
uint8_t i2c_nextcmd[2] = { 0xDE, 0x01 };

// State machine declarations for SPI
enum SPIState
{
  SPI_IDLE,
  SPI_COMMAND,
  SPI_COMMAND_BYTE,
  SPI_ADRESS,
  SPI_DUMP
};

volatile uint8_t spi_state = SPI_IDLE;

// Command buffer for SPI ISR
volatile uint8_t spi_cmd = 0xFE;

// Array init for storing logged values
volatile uint8_t datalog[ARRAY_SIZE];

volatile uint8_t count = 0;

// Flag for performing sample
volatile boolean perform_sample = false;

// Flag for sampling complete
volatile boolean sample_done = false;

// Flag for contact with CTC Ecologic EXT
volatile boolean sync = false;

/*
 I2C-Write from Master
*/
void onWireReceive(int numBytes)
{
  if (!sync) {
    SPDR = 0x01;
    sync = true;
  }

  switch (i2c_state) {
    case I2C_IDLE:
      // We expect a single byte.
      if (numBytes != 1 || Wire.read() != 0xFE)
        break;
      // Check if this is a command or a register request.
      if (i2c_nextcmd[0] > 0xDB)
        i2c_state = I2C_COMMAND;
      else
        i2c_state = I2C_REQUEST;
      break;

    case I2C_RESPONSE:
      // Expected address and data bytes.
      // First byte should match what we sent.
      if (numBytes != 2 || Wire.read() != i2c_nextcmd[0])
      {
        i2c_state = I2C_IDLE;
        i2c_nextcmd[0] = 0xFF;
        break;
      }
      if (perform_sample) {
        datalog[count++] = Wire.read();
        if (count < ARRAY_SIZE)
          i2c_nextcmd[0] = count;
        else
        {
          i2c_nextcmd[0] = 0xFF;
          perform_sample = false;
          sample_done = true;
          SPDR = 0xFE;
        }
      }
      else {
        datalog[0] = Wire.read();    // FIX ME!!! NOT correct in this form
        i2c_nextcmd[0] = 0xFF;
      }
      i2c_state = I2C_IDLE;
      break;

    case I2C_SAMPLE:
      // We expect a single byte.
      if (numBytes != 1 || Wire.read() != 0xFE)
        break;
      perform_sample = true;         // Start a new sample sequence
      sample_done = false;
      count = 0;
      i2c_nextcmd[0] = count;
      i2c_state = I2C_REQUEST;
      break;

    default:
      i2c_state = I2C_IDLE;
      i2c_nextcmd[0] = 0xFF;
  }
} // end of I2C-Write from Master

/*
 I2C-Read from Master
*/
void onWireRequest()
{
  switch (i2c_state) {
    case I2C_REQUEST:                  // register request
      Wire.write(i2c_nextcmd, 1);
      i2c_state = I2C_RESPONSE;
      break;

    case I2C_COMMAND:
      // No-op/ping is a single byte.
      if (i2c_nextcmd[0] == 0xFF)
        Wire.write(i2c_nextcmd, 1);
      else
        Wire.write(i2c_nextcmd, 2);

      i2c_state = I2C_IDLE;
      i2c_nextcmd[0] = 0xFF;
      break;

    default:
      i2c_state = I2C_IDLE;
      i2c_nextcmd[0] = 0xFF;
  }
} // end of I2C-Read from Master

/*
 SPI ISR
*/
ISR (SPI_STC_vect)
{
  // Collect byte sent from MASTER
  uint8_t spi_in = SPDR;
  // Preload response to MASTER
  uint8_t spi_out = spi_in;

  // set status report output
  if (!sync)
    spi_out = 0;                       // Not in sync with I2C master yet
  else {
    if (!sample_done)
      spi_out = 1;                     // In sync with I2C master, no new sample available

    switch (spi_state) {
      case SPI_IDLE:
        switch (spi_in) {
          case 0xFE:                   // No-op / PING to I2C master
            break;

          case 0xFF:                   // Start sample sequence from I2C master
            i2c_state = I2C_SAMPLE;
            break;

          case 0x02:                   // Start command sequence to I2C master
            spi_state = SPI_COMMAND;
            break;

          case 0x03:                   // Start adress request sequence from I2C master
            spi_state = SPI_ADRESS;
            break;

          case 0x04:                   // Start transferring array
            if (!sample_done)
              break;
            spi_state = SPI_DUMP;
            break;
        }
        break;

      case SPI_COMMAND:                // First byte of command sequence
        spi_cmd = spi_in;
        spi_state = SPI_COMMAND_BYTE;
        break;

      case SPI_COMMAND_BYTE:           // Second byte of command sequence
        i2c_nextcmd[0] = spi_cmd;
        i2c_nextcmd[1] = spi_in;
        spi_state = SPI_IDLE;
        break;

      case SPI_ADRESS:                 // Byte of adress request
        spi_out = datalog[spi_in];
        spi_state = SPI_IDLE;
        break;

      case SPI_DUMP:                   // Transfer whole array in sequence
        spi_out = datalog[spi_in];
        if (spi_in == 0xDB) {
          sample_done = false;
          spi_state = SPI_IDLE;
        }
        break;
    }
  }
  // Next byte to send to MASTER
  SPDR = spi_out;
} // end of SPI ISR

void setup()
{
  // init sync LED
  pinMode(LED_SYNC, OUTPUT);

  // init sample_done LED
  pinMode(LED_SAMPLE, OUTPUT);

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);

  // enable SPI in slave mode
  SPCR |= _BV(SPE);

  // clear the registers
  uint8_t clr = SPSR;
  clr = SPDR;
  SPDR = 0x00;

  // now turn on interrupts
  SPCR |= _BV(SPIE);

  // enable I2C in slave mode
  Wire.begin(0x5C);               // slave address
  Wire.onReceive(onWireReceive);  // ISR for I2C-Write
  Wire.onRequest(onWireRequest);  // iSR for I2C-Read

} // end of setup

void loop()
{
  // set the sync LED with the state of the variable:
  digitalWrite(LED_SYNC, sync);

  // set the sample_done LED with the state of the variable:
  digitalWrite(LED_SAMPLE, sample_done);
} // end of loop
