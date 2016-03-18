/**
 *  I2C interface to SPI for CTC Ecologic EXT
 *  ver 0.8.9
**/

#include <Wire.h>
#include <EEPROM.h>

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
  SPI_DUMP,
  SPI_RECEIVE
};

volatile uint8_t spi_state = SPI_IDLE;

// Command buffer for SPI ISR
volatile uint8_t spi_cmd = 0xFE;

// Pointer init, later use point to arrays for store of values
volatile uint8_t *datalog;
volatile uint8_t *templog;
volatile uint8_t *sample_template;

// Global dirty variables for register areas
volatile uint8_t ARRAY_SIZE;
volatile uint8_t SETTINGS;
volatile uint8_t SYSTIME;
volatile uint8_t HISTORICAL;
volatile uint8_t CURRENT;
volatile uint8_t ALARMS;
volatile uint8_t LAST_24H;
volatile uint8_t STATUS;

// Counters...
volatile uint8_t count = 0;
volatile uint8_t eeprom_count = 0;

// Flags for sampling complete / available for transfer
volatile boolean sample_done = false;
volatile boolean new_sample_available = false;
volatile uint8_t sample_send = 0;

// Flags for contact with CTC Ecologic EXT
volatile boolean first_sync = false;
volatile boolean i2c_sync = false;
volatile boolean spi_sync = true;

/*
 I2C-Write from Master
*/
static void onWireReceive(int numBytes)
{
  if (!i2c_sync)      // If we got this far, we're in sync!
    i2c_sync = true;

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
      templog[count++] = Wire.read();  // Right about here, would be the place to insert check if sample is newer than before
      if (count < ARRAY_SIZE)
        i2c_nextcmd[0] = sample_template[count];
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
      i2c_nextcmd[0] = sample_template[count];
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

  if (!spi_sync)      // If we got this far, we're in sync!
    spi_sync = true;

  // set status report output
  if (spi_in == 0xF0)                    // Receive sample_template from SPI Master
  {
    spi_state = SPI_RECEIVE;
    eeprom_count = 0;
  }
  else
  {
    switch (spi_state)
    {
      case SPI_IDLE:
        if (!i2c_sync)                       // Not in sync with I2C master yet
        {
          spi_out = 0x00;
          break;
        }
        switch (spi_in)
        {
          case 0xFE:                     // No-op / PING to I2C master
            break;

          case 0xFF:                    // Start sample sequence from I2C master
            //            i2c_state = I2C_SAMPLE;
            break;

          case 0xF1:                    // Start command sequence to I2C master
            spi_state = SPI_COMMAND;
            spi_out = 0xF1;
            break;

          case 0xF2:                    // Start transferring array
            if (!new_sample_available)  // In sync with I2C master, no new sample available
            {
              spi_out = 0x01;
              break;
            }
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

      case SPI_DUMP:                   // Transfer the blocks which has changed
        if (spi_in == 0xFE)
        {
          spi_out = sample_send;
          break;
        }
        if (spi_in == 0xAD)           // Changed to 0xAD in ver 0.8.4, only send up to CURRENT
        {
          sample_send = 0;
          new_sample_available = false;
          spi_state = SPI_IDLE;
          break;
        }
        spi_out = datalog[spi_in];
        break;

      case SPI_RECEIVE:
        EEPROM.update(eeprom_count++, spi_in);
        if (eeprom_count > (ARRAY_SIZE + 8))
        {
          init_arrays();
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
  for (uint8_t x = x_start ; x < x_stop ; x++)
    if (datalog[x] != templog[x])
    {
      datalog[x] = templog[x];
      sample_send |= block;
      new_sample_available = true;
    }
} // end of checkforchange

void init_arrays()
{
  // Set different register areas from template stored in EEPROM
  ARRAY_SIZE = EEPROM.read(0);
  SETTINGS = EEPROM.read(1);
  SYSTIME = EEPROM.read(2);
  HISTORICAL = EEPROM.read(3);
  CURRENT = EEPROM.read(4);
  ALARMS = EEPROM.read(5);
  LAST_24H = EEPROM.read(6);
  STATUS = EEPROM.read(7);

  // Declare arrays to store samples in, no error checking... I KNOW...
  datalog = static_cast<uint8_t*>(calloc(ARRAY_SIZE, sizeof(uint8_t)));
  templog = static_cast<uint8_t*>(calloc(ARRAY_SIZE, sizeof(uint8_t)));
  sample_template = static_cast<uint8_t*>(calloc(ARRAY_SIZE, sizeof(uint8_t)));

  // Populate sample_template with values stored in EEPROM
  for (uint8_t x = 0; x < ARRAY_SIZE  ; x++)
    sample_template[x] = EEPROM.read(8 + x);
} // end of init_arrays

void setup()
{
  // Initialize arrays with values stored in EEPROM
  init_arrays();

  // Port B0 and B1 output, sync and sample LED
  DDRB |= (1 << DDB0) | (1 << DDB1);

  // Set MISO output
  DDRB |= (1 << DDB4);

  // Interrupt enabled, SPI enabled, MSB first, slave, CLK low when idle, sample on leading edge of CLK
  SPCR = (1 << SPIE) | (1 << SPE);

  // Clear the registers
  uint8_t clr = SPSR;
  clr = SPDR;
  SPDR = 0x00;

  // Enable I2C in slave mode
  Wire.begin(0x5C);               // Slave address
  Wire.onReceive(onWireReceive);  // ISR for I2C-Write
  Wire.onRequest(onWireRequest);  // ISR for I2C-Read

} // end of setup

void loop()
{
  // Set the sync and sample_done LED with the state of the variable:
  if (i2c_sync && spi_sync)
  {
    PORTB |= (1 << PORTB0);
    if (!first_sync && i2c_state == I2C_IDLE)   // Check if this is First contact, then enable autologging, do not alert the Federation...
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
    checkforchange(SETTINGS , SYSTIME , B00000001);

    // Check for change in CURRENT, normal every change of temp
    checkforchange(HISTORICAL , CURRENT , B00000010);

    // Check for change in HISTORICAL, normal every hour and week
    checkforchange(SYSTIME , HISTORICAL , B00000100);

    // Check for change in SETTINGS, hardly ever any changes
    checkforchange(0 , SETTINGS , B00001000);

    // Check for change in ALARMS, hardly ever any changes
    checkforchange(CURRENT , ALARMS , B00010000);

    // Check for change in LAST_24H, hardly ever any changes
    checkforchange(ALARMS , LAST_24H , B00100000);

    // Check for change in STATUS, hardly ever any changes
    checkforchange(LAST_24H , STATUS , B01000000);

    i2c_state = I2C_SAMPLE;   // Go for another auto sample!
  }
  if (new_sample_available)
    PORTB |= (1 << PORTB1);
  else
    PORTB &= ~(1 << PORTB1);
} // end of loop
