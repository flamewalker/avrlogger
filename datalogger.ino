/**
    I2C interface to SPI for CTC Ecologic EXT
    ver 0.9.2
**/

#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>

// Max size of data array
#define ARRAY_SIZE 0xAF

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Array to hold device address
DeviceAddress tempDeviceAddress;

// Variables for handling sampling from OneWire sensors
float temperature = 0.0;
int8_t temperaturearray[20];
unsigned long lastTempRequest = 0;
unsigned int delayInMillis = 750;

// State machine declarations for I2C
enum I2CState
{
  I2C_IDLE,
  I2C_RESPONSE,
  I2C_REQUEST,
  I2C_COMMAND,
  I2C_DEBUG
};

static volatile I2CState i2c_state = I2C_IDLE;

// Command buffer for I2C ISR
static uint8_t i2c_nextcmd[2] = { 0xDE, 0x01 };    // Preload with command to signal Modem = OK , SMS = 1  For some reason this cannot be volatile, problem with Wire library
static volatile uint8_t *save_i2ccmd;

// State machine declarations for SPI
enum SPIState
{
  SPI_IDLE,
  SPI_COMMAND,
  SPI_COMMAND_BYTE,
  SPI_DUMP
};

static volatile SPIState spi_state = SPI_IDLE;

// Command buffer for SPI ISR
static volatile uint8_t spi_cmd[2] = { 0xFF, 0x00 };

// Pointer init, later use point to arrays for store of values
volatile uint8_t *datalog = NULL;
volatile uint8_t *templog = NULL;

// Debug vars
static volatile uint8_t *test = NULL;
static volatile uint8_t test2 = 0;
static volatile uint8_t test3 = 0;
static volatile uint8_t slask_id1 = 0;
static volatile uint8_t slask_id2 = 0;
static volatile uint8_t slask_re1 = 0;
static volatile uint8_t slask_re2 = 0;
static volatile I2CState state_dbg_wr = I2C_DEBUG;
static volatile I2CState state_dbg_re = I2C_DEBUG;

// Counter...
static volatile uint8_t count = 0;

// Flags for sampling complete / available for transfer
static volatile boolean sample_done = false;
static volatile boolean new_sample_available = false;
static volatile uint8_t sample_send = B01111111;
static volatile boolean command_pending = false;
static volatile boolean sample_pending = false;
static volatile boolean start_sampling = false;

// Flags for contact with CTC Ecologic EXT
static volatile boolean first_sync = false;
static volatile boolean sync = false;

/*
  I2C-Write from Master
*/
static void onWireReceive(int numBytes)
{
  if (!sync)                  // If we got this far, we're in sync!
    sync = true;

  switch (i2c_state) {
    case I2C_IDLE:
      // We expect a single byte.
      slask_id1 = Wire.read();
      if (numBytes != 1 || slask_id1 != 0xFE)
      {

        test2 |= 1;

        if (numBytes == 2)
        {
          test2 |= 2;
          slask_id2 = Wire.read();
        }
        break;
      }
      if (start_sampling)
      {
        count = 0;
        i2c_nextcmd[0] = count;
        start_sampling = false;
      }
      if (command_pending)
      {
        save_i2ccmd[0] = i2c_nextcmd[0];
        save_i2ccmd[1] = i2c_nextcmd[1];
        i2c_nextcmd[0] = spi_cmd[0];
        i2c_nextcmd[1] = spi_cmd[1];
      }
      // Check if this is a command or a register request.
      if (i2c_nextcmd[0] > 0xDB)
        i2c_state = I2C_COMMAND;
      else
        i2c_state = I2C_REQUEST;
      break;

    case I2C_RESPONSE:
      // Expected address and data bytes.
      // First byte should match what we sent.
      slask_re1 = Wire.read();
      if (numBytes != 2 || slask_re1 != i2c_nextcmd[0])
      {
        if (numBytes == 2)
        {
          test2 |= 4;
          slask_re2 = Wire.read();
          i2c_state = I2C_IDLE;
          break;
        }
        else
        {
          if (numBytes == 1 && slask_re1 == 0xFE)           // Somehow the master is in I2C_IDLE mode
          {
            test2 |= 8;
            i2c_state = I2C_REQUEST;                        // Better fix it by doing the request again
            break;
          }
        }
        i2c_state = I2C_IDLE;
        break;
      }
      slask_re2 = Wire.read();

      if (slask_re2 == i2c_nextcmd[0])
        test2 |= 16;

      templog[count] = slask_re2;
      sample_pending = true;
      count++;
      if (count < ARRAY_SIZE)
        i2c_nextcmd[0] = count;
      else
      {
        i2c_nextcmd[0] = 0xFF;
        sample_done = true;
        count = 0;
      }
      i2c_state = I2C_IDLE;
      break;

    default:
      test2 |= 64;
      state_dbg_wr = i2c_state;
      if (numBytes != 1 || Wire.read() != 0xFE)
      {
        i2c_state = I2C_IDLE;
        break;
      }
      // Check if this is a command or a register request.
      if (i2c_nextcmd[0] > 0xDB)
        i2c_state = I2C_COMMAND;
      else
        i2c_state = I2C_REQUEST;
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
      {
        Wire.write(i2c_nextcmd, 2);
        i2c_nextcmd[0] = save_i2ccmd[0];
        i2c_nextcmd[1] = save_i2ccmd[1];
        command_pending = false;
      }
      i2c_state = I2C_IDLE;
      break;

    default:
      test2 |= 128;
      state_dbg_re = i2c_state;
      Wire.write("0xFF", 1);
      i2c_state = I2C_IDLE;
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
    spi_out = 0x00;                       // Not in sync with I2C master yet
  else {
    if (!new_sample_available)
      spi_out = 0x01;                     // In sync with I2C master, no new sample available

    switch (spi_state) {
      case SPI_IDLE:
        switch (spi_in) {
          case 0xFF:                   // NO-OP / PING
            break;

          case 0xF1:                   // Start command sequence to I2C master
            spi_state = SPI_COMMAND;
            spi_out = 0xF1;
            break;

          case 0xF0:                   // Start transferring array
            if (!new_sample_available)
              break;
            spi_state = SPI_DUMP;
            spi_out = 0xF0;
            break;

          case 0xA0:
            spi_out = i2c_state;
            break;

          case 0xA1:
            test = i2c_nextcmd;
            spi_out = *test;
            break;

          case 0xA2:
            test = i2c_nextcmd;
            test++;
            spi_out = *test;
            break;

          case 0xA3:
            spi_out = test2;
            break;

          case 0xA4:
            spi_out = test3;
            break;

          case 0xA5:
            spi_out = sample_done;
            break;

          case 0xA6:
            spi_out = sample_pending;
            break;

          case 0xA7:
            spi_out = start_sampling;
            break;

          case 0xA8:
            spi_out = slask_id1;
            break;

          case 0xA9:
            spi_out = slask_id2;
            break;

          case 0xAA:
            spi_out = slask_re1;
            break;

          case 0xAB:
            spi_out = slask_re2;
            break;

          case 0xAC:
            spi_out = count;
            break;

          case 0xAD:
            spi_out = state_dbg_wr;
            break;

          case 0xAE:
            spi_out = state_dbg_re;
            break;
        }
        break;

      case SPI_COMMAND:                // First byte of command sequence
        spi_cmd[0] = spi_in;
        spi_out = spi_in;
        if (command_pending)           // Signal that we already have a unhandled command waiting
          spi_out = 0xFF;
        spi_state = SPI_COMMAND_BYTE;
        break;

      case SPI_COMMAND_BYTE:           // Second byte of command sequence
        spi_cmd[1] = spi_in;
        spi_out = spi_in;
        if (command_pending)           // Signal that we already have a unhandled command waiting
          spi_out = 0xFF;
        command_pending = true;
        spi_state = SPI_IDLE;
        break;

      case SPI_DUMP:                   // Routine for transferring data
        if (spi_in == 0xFF)            // NO-OP / PING
          break;
        if (spi_in == 0xFE)            // Signal which blocks has changed
        {
          spi_out = sample_send;
          break;
        }
        if (spi_in >= 0xAD)           // Changed to 0xAD in ver 0.8.4, only send up to CURRENT
        {
          if (spi_in >= 0xB0 && spi_in <= 0xBF)
          {
            spi_out = temperaturearray[spi_in - 0xB0];
          }
          else
          {
            sample_send = 0;
            new_sample_available = false;
            spi_state = SPI_IDLE;
            break;
          }
        }
        else
        {
          spi_out = datalog[spi_in];
          if (spi_out == spi_in)
          {
            test2 |= 32;
            test3++;
          }
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
  // Declare arrays to store samples in, no error checking... I KNOW...
  datalog = static_cast<uint8_t*>(calloc(ARRAY_SIZE, sizeof(uint8_t)));
  templog = static_cast<uint8_t*>(calloc(ARRAY_SIZE, sizeof(uint8_t)));
  save_i2ccmd = static_cast<uint8_t*>(calloc(2, sizeof(uint8_t)));

  // Initialize OneWire sensors
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, 12);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  lastTempRequest = millis();

  // Port D7 output, sample_ready signal
  DDRD |= (1 << DDD7);

  // Set MISO output
  DDRB |= (1 << DDB4);

  // Interrupt enabled, SPI enabled, MSB first, Slave, CLK low when idle,
  // Sample on leading edge of CLK
  SPCR = (1 << SPIE) | (1 << SPE);

  // Clear the registers
  uint8_t clr = SPSR;
  clr = SPDR;
  SPDR = 0x00;

  // Enable I2C in slave mode
  Wire.begin(0x5C);               // slave address
  Wire.onReceive(onWireReceive);  // ISR for I2C-Write
  Wire.onRequest(onWireRequest);  // ISR for I2C-Read

} // end of setup

void loop()
{
  // Set the sync and sample_done LED with the state of the variable:
  if (sync)
  {
    if (!first_sync && i2c_state == I2C_IDLE)   // Check if this is First contact, then enable autologging, do not alert the Federation...
    {
      start_sampling = true;                    // Start the auto-sampling!
      first_sync = true;
    }
  }
  else
    first_sync = false;

  if (millis() - lastTempRequest >= delayInMillis)
  {
    temperature = sensors.getTempCByIndex(0);
    if (temperature != DEVICE_DISCONNECTED_C)
    {
      temperaturearray[0] = (int)temperature;
      temperaturearray[1] = (int)round(temperature * 100.0) - (temperaturearray[0] * 100);
    }
    sensors.requestTemperatures();
    lastTempRequest = millis();
  }

  // Start checking the status of the newly taken sample versus the last sent
  if (sample_done && i2c_state == I2C_IDLE)
  {
    // Check for change in SYSTIME, normal every minute
    checkforchange(0x73 , 0x75 , B00000001);

    // Check for change in CURRENT, normal every change of temp
    checkforchange(0x8C , 0x99 , B00000010);
    checkforchange(0x9B , 0xA0 , B00000010);
    checkforchange(0xA8 , 0xAC , B00000010);

    // Check for change in HISTORICAL, normal every hour and week
    checkforchange(0x76 , 0x7E , B00000100);
    checkforchange(0x80 , 0x84 , B00000100);
    checkforchange(0x87 , 0x8B , B00000100);

    // Check for change in SETTINGS, hardly ever any changes
    checkforchange(0x00 , 0x68 , B00001000);

    // Check for change in ALARMS, almost never ever any changes
    checkforchange(0xA1 , 0xA4 , B00010000);

    // Check for change in LAST_24H,  normal change once every hour
    checkforchange(0x85 , 0x86 , B00100000);
    checkforchange(0x7F , 0x7F , B00100000);

    // Check for change in STATUS, normal change every minute
    checkforchange(0x9A , 0x9A , B01000000);
    checkforchange(0xA5 , 0xA7 , B01000000);

    sample_done = false;
    sample_pending = false;
    start_sampling = true;       // Go for another auto sample!
  }
  if (new_sample_available)
    PORTD |= (1 << PORTD7);      // Set the interrupt line HIGH
  else
    PORTD &= ~(1 << PORTD7);
} // end of loop
