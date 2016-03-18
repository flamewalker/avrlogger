/**
 *  I2C interface to SPI for CTC Ecologic EXT
 *  ver 0.8.7
**/

#include <Wire.h>
#include <EEPROM.h>

#define SETTINGS 0x68
#define SYSTIME 0x75
#define HISTORICAL 0x8B
#define CURRENT 0xAC

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

// Command buffer for I2C ISR
uint8_t i2c_nextcmd[2] = { 0xDE, 0x01 };  // Preload with command to signal Modem = OK , SMS = 1

// State machine declarations for SPI
enum SPIState
{
  SPI_IDLE,
  SPI_COMMAND,
  SPI_COMMAND_BYTE,
  SPI_DUMP
};

volatile uint8_t spi_state = SPI_IDLE;

// Command buffer for SPI ISR
volatile uint8_t spi_cmd = 0xFE;

// Pointer init, later use point to arrays for store of values
volatile uint8_t *datalog;
volatile uint8_t *templog;

volatile uint8_t ARRAY_SIZE;

volatile uint8_t count = 0;

// Flags for sampling complete / available for transfer
volatile boolean sample_done = false;
volatile boolean new_sample_available = false;
volatile uint8_t sample_send = 0;

// Flags for contact with CTC Ecologic EXT
volatile boolean first_sync = false;
volatile boolean sync = false;

/*
 I2C-Write from Master
*/
static void onWireReceive(int numBytes)
{
  if (!sync) {      // If we got this far, we're in sync!
    SPDR = 0x01;    // Preload with sync message
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
        count = 0;
        break;
      }
      templog[i2c_nextcmd[0]] = Wire.read();
      if (++count < ARRAY_SIZE)
        i2c_nextcmd[0] = count;
      else
      {
        i2c_nextcmd[0] = 0xFF;
        sample_done = true;
        count = 0;
        SPDR = 0xFE;
      }
      i2c_state = I2C_IDLE;
      break;

    case I2C_SAMPLE:
      // We expect a single byte.
      if (numBytes != 1 || Wire.read() != 0xFE)
        break;
      // Start a new sample sequence
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
static void onWireRequest()
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
    if (!new_sample_available)
      spi_out = 1;                     // In sync with I2C master, no new sample available

    switch (spi_state) {
      case SPI_IDLE:
        switch (spi_in) {
          case 0xFE:                   // No-op / PING to I2C master
            break;

          case 0xFF:                   // Start sample sequence from I2C master
            //            i2c_state = I2C_SAMPLE;
            break;

          case 0x02:                   // Start command sequence to I2C master
            spi_state = SPI_COMMAND;
            spi_out = 2;
            break;

          case 0x04:                   // Start transferring array
            if (!new_sample_available)
              break;
            spi_state = SPI_DUMP;
            spi_out = 4;
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

      case SPI_DUMP:                   // Transfer the blocks which has changed
        if (spi_in == 0xFE)
        {
          spi_out = sample_send;
          break;
        }
        spi_out = datalog[spi_in];
        if (spi_in == 0xAD)           // Changed to 0xAD in ver 0.8.4, only send up to CURRENT
        {
          sample_send = 0;
          new_sample_available = false;
          spi_state = SPI_IDLE;
        }
        break;
    }
  }
  // Next byte to send to MASTER
  SPDR = spi_out;
} // end of SPI ISR

void checkforchange(uint8_t x_start , uint8_t x_stop , uint8_t block)
{
  // Check for change in the specified address and set according block, return true if change has occured
  for (uint8_t x = x_start ; x <= x_stop ; x++)
    if (datalog[x] != templog[x])
    {
      datalog[x] = templog[x];
      sample_send |= block;
      new_sample_available = true;
    }
} // end of checkforchange

void setup()
{
  // Check if EEPROM has the right value for array declaration, dirty fix until later
  EEPROM.update(0, 0xDC);   // 0xDC == Highest available address of registers in CTC
  ARRAY_SIZE = EEPROM.read(0);

  // Declare arrays to store samples in, no error checking... I KNOW...
  datalog = static_cast<uint8_t*>(calloc(ARRAY_SIZE, sizeof(uint8_t)));
  templog = static_cast<uint8_t*>(calloc(ARRAY_SIZE, sizeof(uint8_t)));

  // Port B0 and B1 output, sync and sample LED
  DDRB |= (1 << DDB0) | (1 << DDB1);

  // Set MISO output
  DDRB |= (1 << DDB4);

  // Interrupt enabled, spi enabled, msb 1st, slave, clk low when idle,
  // sample on leading edge of clk
  SPCR = (1 << SPIE) | (1 << SPE);

  // clear the registers
  uint8_t clr = SPSR;
  clr = SPDR;
  SPDR = 0x00;

  // enable I2C in slave mode
  Wire.begin(0x5C);               // slave address
  Wire.onReceive(onWireReceive);  // ISR for I2C-Write
  Wire.onRequest(onWireRequest);  // iSR for I2C-Read

} // end of setup

void loop()
{
  // Set the sync and sample_done LED with the state of the variable:
  if (sync)
  {
    PORTB |= (1 << PORTB0);
    if (!first_sync && i2c_state == I2C_IDLE)   // Check if this is First contact, then enable autologging
    {
      i2c_state = I2C_SAMPLE;   // Start the auto-sampling!
      first_sync = true;
    }
  }
  else
  {
    PORTB &= ~(1 << PORTB0);
    first_sync = false;
  }
  // Start checking the status of the newly taken sample versus the last sent
  if (sample_done && i2c_state == I2C_IDLE)
  {
    // Check for change in SYSTIME, normal every minute
    checkforchange(0x73 , SYSTIME , B00000001);

    // Check for change in CURRENT, normal every change of temp
    checkforchange(0x8C , CURRENT , B00000010);

    // Check for change in HISTORICAL, normal every hour and week
    checkforchange(0x76 , HISTORICAL , B00000100);

    // Check for change in SETTINGS, hardly ever any changes
    checkforchange(0x00 , SETTINGS , B00001000);

    i2c_state = I2C_SAMPLE;   // Go for another auto sample!
  }
  if (new_sample_available)
    PORTB |= (1 << PORTB1);
  else
    PORTB &= ~(1 << PORTB1);
} // end of loop
