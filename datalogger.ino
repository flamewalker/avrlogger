/**
    I2C interface to SPI for CTC Ecologic EXT
    ver 0.9.1
**/

#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>

#define ARRAY_SIZE 0xDC

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
int temperaturearray[20];
unsigned long lastTempRequest = 0;
int delayInMillis = 750;

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
volatile uint8_t save_i2cstate = I2C_IDLE;

// Command buffer for I2C ISR
uint8_t i2c_nextcmd[2] = { 0xDE, 0x01 };  // Preload with command to signal Modem = OK , SMS = 1
uint8_t save_i2ccmd[2] = { 0xFF, 0x00 };

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

// Counter...
volatile uint8_t count = 0;

// Flags for sampling complete / available for transfer
volatile boolean sample_done = false;
volatile boolean new_sample_available = false;
volatile uint8_t sample_send = B01111111;

// Flags for contact with CTC Ecologic EXT
volatile boolean first_sync = false;
volatile boolean sync = false;
volatile boolean first_sample = true;

/*
  I2C-Write from Master
*/
void onWireReceive(int numBytes)
{
  if (!sync)                  // If we got this far, we're in sync!
    sync = true;

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
      }
      i2c_state = I2C_IDLE;
      break;

    case I2C_SAMPLE:
      // We expect a single byte.
      if (numBytes != 1 || Wire.read() != 0xFE)
        break;
      // Start a new sample sequence
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
      {
        Wire.write(i2c_nextcmd, 1);
        i2c_state = I2C_IDLE;
      }
      else
      {
        Wire.write(i2c_nextcmd, 2);
        i2c_state = save_i2cstate;
        i2c_nextcmd[0] = save_i2ccmd[0];
        i2c_nextcmd[1] = save_i2ccmd[1];
        save_i2cstate = I2C_IDLE;
        save_i2ccmd[0] = 0xFF;
        save_i2ccmd[1] = 0x00;
      }
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
        }
        break;

      case SPI_COMMAND:                // First byte of command sequence
        spi_cmd = spi_in;
        spi_state = SPI_COMMAND_BYTE;
        spi_out = spi_in;
        break;

      case SPI_COMMAND_BYTE:           // Second byte of command sequence
        save_i2ccmd[0] = i2c_nextcmd[0];
        save_i2ccmd[1] = i2c_nextcmd[1];
        save_i2cstate = i2c_state;
        i2c_nextcmd[0] = spi_cmd;
        i2c_nextcmd[1] = spi_in;
        i2c_state = I2C_IDLE;
        spi_state = SPI_IDLE;
        spi_out = spi_in;
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
          spi_out = datalog[spi_in];
        break;
    }
  }
  // Next byte to send to MASTER
  SPDR = spi_out;
} // end of SPI ISR

void checkforchange(uint8_t x_start , uint8_t x_stop , uint8_t block)
{
  uint8_t diff = 0;
  // Check for change in the specified address and set according block, return true if change has occured
  for (uint8_t x = x_start ; x <= x_stop ; x++)
    if (datalog[x] != templog[x])
    {
      diff = abs(datalog[x] - templog[x]);
      if (diff < 10 || first_sample)
      {
        datalog[x] = templog[x];
        sample_send |= block;
        new_sample_available = true;
      }
    }
} // end of checkforchange

void setup()
{
  // Declare arrays to store samples in, no error checking... I KNOW...
  datalog = static_cast<uint8_t*>(calloc(ARRAY_SIZE, sizeof(uint8_t)));
  templog = static_cast<uint8_t*>(calloc(ARRAY_SIZE, sizeof(uint8_t)));

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
  Wire.onRequest(onWireRequest);  // ISR for I2C-Read

} // end of setup

void loop()
{
  // Set the sync and sample_done LED with the state of the variable:
  if (sync)
  {
    if (!first_sync && i2c_state == I2C_IDLE)   // Check if this is First contact, then enable autologging, do not alert the Federation...
    {
      i2c_state = I2C_SAMPLE;   // Start the auto-sampling!
      first_sync = true;
    }
  }
  else
  {
    first_sync = false;
  }

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
    first_sample = false;
    i2c_state = I2C_SAMPLE;   // Go for another auto sample!
  }
  if (new_sample_available)
    PORTD |= (1 << PORTD7);
  else
    PORTD &= ~(1 << PORTD7);
} // end of loop
