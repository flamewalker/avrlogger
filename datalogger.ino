/**
    I2C interface to SPI for CTC Ecologic EXT
    ver 0.9.4
**/

#include <DallasTemperature.h>
#include <OneWire.h>

// Defs for making a median search routine
#define MED_SORT(a,b) { if ((a)>(b)) MED_SWAP((a),(b)); }
#define MED_SWAP(a,b) { float tmp=(a);(a)=(b);(b)=tmp; }

// TWI buffer length
#define TWI_BUFFER_LENGTH 4

// Array sizes
#define ARRAY_SIZE 0xB0 + NUM_SENSORS * 2
#define SAMPLE_SIZE 0xAD

// Number of OneWire sensors
#define NUM_SENSORS 1

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
float mediantemp = 0.0;
uint8_t num_samp = 0;
float temparray[NUM_SENSORS][3];
unsigned long lastTempRequest = 0;
unsigned int delayInMillis = 750;

// Command buffer for I2C ISR
static volatile uint8_t i2c_nextcmd[2] = { 0xDE, 0x01 };    // Preload with command to signal Modem = OK , SMS = 1
static volatile uint8_t save_i2ccmd[2];

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

// Init the arrays for storage
static volatile uint8_t datalog[ARRAY_SIZE];
static volatile uint8_t templog[ARRAY_SIZE];

// Debug vars
static volatile uint8_t test1, test2, test3, test4, test5, test6, test7, test8, slask_tx1, slask_tx2, slask_rx1, slask_rx2 = 0;

// Counter...
static volatile uint8_t count = 0;

// Flags for sampling complete / available for transfer
static volatile boolean sample_done = false;
static volatile boolean new_sample_available = false;
static volatile uint8_t sample_send = B11111111;
static volatile boolean command_pending = false;
static volatile boolean start_sampling = false;

// Flags for contact with CTC Ecologic EXT
static volatile boolean first_sync = false;
static volatile boolean sync = false;

// TWI vars
static volatile uint8_t twi_txBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_txBufferIndex;
static volatile uint8_t twi_txBufferLength;

static volatile uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_rxBufferIndex;

/*
  TWI ISR
*/
ISR (TWI_vect)
{
  switch (TWSR & ((1 << TWS7) | (1 << TWS6) | (1 << TWS5) | (1 << TWS4) | (1 << TWS3)))
  {
    // Status Codes for SLAVE RECEIVER mode
    case 0x60:          // Own SLA+W has been received; ACK has been returned
    case 0x68:          // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
    case 0x70:          // General call address has been received; ACK has been returned
    case 0x78:          // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
      // Reset rx buffer index
      twi_rxBufferIndex = 0;
      // ACK will be returned
      TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
      break;

    case 0x80:          // Previously addressed with own SLA+W; data has been received; ACK has been returned
    case 0x90:          // Previously addressed with general call; data has been received; ACK has been returned
      // Check if there is room in the buffer
      if (twi_rxBufferIndex < TWI_BUFFER_LENGTH)
      {
        // Put received byte in buffer
        twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
        // ACK will be returned
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
      }
      else
      {
        // No room, NOT ACK will be returned
        test2 |= 2;   // Debug
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
      }
      break;

    case 0x88:          // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
    case 0x98:          // Previously addressed with general call; data has been received; NOT ACK has been returned
      test2 |= 4;   // Debug
      // NOT ACK will be returned
      TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
      break;

    case 0xA0:          // A STOP condition or repeated START condition has been received while still addressed as Slave
      // Release bus, ACK will be returned
      TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
      // Add a NULL char after data, if possible
      if (twi_rxBufferIndex < TWI_BUFFER_LENGTH)
        twi_rxBuffer[twi_rxBufferIndex] = '\0';

      // Handle the data we got in the buffer ------>
      if (twi_rxBufferIndex == 1 && twi_rxBuffer[0] == 0xFE)
      {
        if (!sync)                  // If we got this far, we're in sync!
          sync = true;
        if (start_sampling)         // Check flag to see if we should start the sampling process
        {
          i2c_nextcmd[0] = 0;       // Always start from the first address
          count = 0;                // In case the counter somehow has changed, reset it
          start_sampling = false;
        }
        if (command_pending)
        {
          save_i2ccmd[0] = i2c_nextcmd[0];    // Store whatever we were doing and issue command instead
          save_i2ccmd[1] = i2c_nextcmd[1];
          i2c_nextcmd[0] = spi_cmd[0];
          i2c_nextcmd[1] = spi_cmd[1];
        }
      }
      else if (twi_rxBufferIndex == 2 && twi_rxBuffer[0] == i2c_nextcmd[0])
      {
        templog[count++] = twi_rxBuffer[1];
        if (count < SAMPLE_SIZE)
          i2c_nextcmd[0] = count;
        else
        {
          i2c_nextcmd[0] = 0xFF;  // Load with NO-OP/PING command
          count = 0;              // Reset counter ASAP to prevent out of bounds array addressing
          sample_done = true;     // Set flag to indicate we have finished the sampling process
        }
      }
      twi_rxBufferIndex = 0;
      break;

    // Status Codes for SLAVE TRANSMITTER mode
    case 0xA8:          // Own SLA+R has been received; ACK has been returned
    case 0xB0:          // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
      twi_txBufferIndex = 0;
      twi_txBufferLength = 0;

      // Prepare data to send ------->

      // No-op/ping is a single byte.
      if (i2c_nextcmd[0] == 0xFF || i2c_nextcmd[0] < 0xDC)
      {
        twi_txBufferLength = 1;
        twi_txBuffer[0] = i2c_nextcmd[0];
      }
      else
      {
        twi_txBufferLength = 2;
        twi_txBuffer[0] = i2c_nextcmd[0];
        twi_txBuffer[1] = i2c_nextcmd[1];
        i2c_nextcmd[0] = save_i2ccmd[0];
        i2c_nextcmd[1] = save_i2ccmd[1];
        command_pending = false;
      }
      if (twi_txBufferLength == 0)
      {
        test2 |= 8;   // Debug
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0xFF;
      }
    // Load TWDR with first byte to send in next step

    case 0xB8:          // Data byte in TWDR has been transmitted; ACK has been received
      // Load TWDR with byte to send from buffer
      TWDR = twi_txBuffer[twi_txBufferIndex++];
      // Check if there is more bytes to send
      if (twi_txBufferIndex < twi_txBufferLength)
        // ACK will be returned if there is more to send
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
      else
        // NOT ACK will be returned
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
      break;

    case 0xC0:          // Data byte in TWDR has been transmitted; NOT ACK has been received
    case 0xC8:          // Last data byte in TWDR has been transmitted (TWEA = "0"); ACK has been received
      // ACK will be returned
      TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
      break;

    // TWI Errors
    case 0xF8:          // No relevant state information available; TWINT = "0"
      test2 |= 32;    // Debug
      break;

    case 0x00:          // Bus Error due to an illegal START or STOP condition
      test1++;                        // Debug
      test2 |= 64;                    // Debug
      slask_rx1 = twi_rxBuffer[0];    // Debug
      slask_rx2 = twi_rxBuffer[1];    // Debug
      slask_tx1 = twi_txBuffer[0];    // Debug
      slask_tx2 = twi_txBuffer[1];    // Debug
      // Release bus and reset TWI hardware
      TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWSTO);
      break;

    default:
      test2 |= 128;                   // Debug
  }
}  // end of TWI ISR

/*
  SPI ISR
*/
ISR (SPI_STC_vect)
{
  // Set status report output
  if (!sync)
    SPDR = 0x00;                       // Not in sync with I2C master yet
  else
  {
    switch (spi_state)
    {
      case SPI_IDLE:
        switch (SPDR)
        {
          case 0xFF:                   // NO-OP / PING
            if (!new_sample_available)
              SPDR = 0x01;                     // In sync with I2C master, no new sample available
            else
              SPDR = 0xFF;
            break;

          case 0xF1:                   // Start command sequence to I2C master
            spi_state = SPI_COMMAND;
            SPDR = 0xF1;
            break;

          case 0xF0:                   // Start transferring array
            if (!new_sample_available)
              SPDR = 0x01;
            else
            {
              spi_state = SPI_DUMP;
              SPDR = 0xF0;
            }
            break;

          case 0xF2:                  //  Release bus and reset TWI hardware
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWSTO);
            SPDR = 0xF2;
            break;

          case 0xA0:
            SPDR = test1;
            break;

          case 0xA1:
            SPDR = test2;
            break;

          case 0xA2:
            SPDR = test3;
            break;

          case 0xA3:
            SPDR = count;
            break;

          case 0xA4:
            SPDR = twi_rxBuffer[0];
            break;

          case 0xA5:
            SPDR = twi_rxBuffer[1];
            break;

          case 0xA6:
            SPDR = twi_txBuffer[0];
            break;

          case 0xA7:
            SPDR = twi_txBuffer[1];
            break;

          case 0xA8:
            SPDR = slask_rx1;
            break;

          case 0xA9:
            SPDR = slask_rx2;
            break;

          case 0xAA:
            SPDR = slask_tx1;
            break;

          case 0xAB:
            SPDR = slask_tx2;
            break;

          case 0xAF:
            test1 = 0;
            test2 = 0;
            test3 = 0;
            test4 = 0;
            slask_tx1 = 0;
            slask_tx2 = 0;
            slask_rx1 = 0;
            slask_rx2 = 0;
            SPDR = 0xAF;
            break;
        }
        break;

      case SPI_COMMAND:                // First byte of command sequence
        spi_cmd[0] = SPDR;
        if (command_pending)           // Signal that we already have a unhandled command waiting
          SPDR = 0xFF;
        spi_state = SPI_COMMAND_BYTE;
        break;

      case SPI_COMMAND_BYTE:           // Second byte of command sequence
        spi_cmd[1] = SPDR;
        if (command_pending)           // Signal that we already have a unhandled command waiting
          SPDR = 0xFF;
        command_pending = true;
        spi_state = SPI_IDLE;
        break;

      case SPI_DUMP:                   // Routine for transferring data
        if (SPDR < ARRAY_SIZE)         // Check that we're within the array bounds
        {
          SPDR = datalog[SPDR];
          break;
        }
        else
        {
          if (SPDR == 0xFF)            // NO-OP / PING
          {
            SPDR = 0xFF;
            break;
          }
          if (SPDR == 0xFE)            // Signal which blocks has changed
          {
            SPDR = sample_send;
            break;
          }
          if (SPDR == 0xF0)
          {
            new_sample_available = false;
            sample_send = 0;          // Reset now that we've sent everything
            spi_state = SPI_IDLE;
            SPDR = test4;
            test4 = 0;
            break;
          }
        }
        break;
    }
  }
} // end of SPI ISR

float median3(float * f)
{
  MED_SORT(f[0], f[1]) ; MED_SORT(f[1], f[2]) ; MED_SORT(f[0], f[1]) ;
  return (f[1]) ;
}

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
  // Initialize OneWire sensors
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, 12);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  // Initialize ports
  // Set Port B1, B0 output
  DDRB |= (1 << DDB1) | (1 << DDB0);

  // Set Port C0, C1, C2, C3 analog input
  DDRC |= (0 << DDC3) | (0 << DDC2) | (0 << DDC1) | (0 << DDC0);
  DIDR0 |= (1 << ADC3D) | (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D);

  // Set Port D7, D6 output, sample_ready signal
  DDRD |= (1 << DDD7) | (1 << DDD6);

  // Initialize all the interfaces
  // SPI slave interface to Raspberry Pi
  // Set Port B4 output, SPI MISO
  DDRB |= (1 << DDB4);

  // Interrupt enabled, SPI enabled, MSB first, Slave, CLK low when idle, Sample on leading edge of CLK (SPI Mode 0)
  SPCR = (1 << SPIE) | (1 << SPE);

  // Clear the registers
  uint8_t clr = SPSR;
  clr = SPDR;
  SPDR = 0x00;

  // Initialize USART0 as SPI Master interface to digital potentiometer
  UBRR0 = 0;

  // Set Port D4 output, Port D5 output: SPI CLK, SS
  DDRD |= (1 << DDD4) | (1 << DDD5);

  // Enable USART0 SPI Master Mode, MSB first, CLK low when idle, Sample on trailing edge of CLK (Mode 1)
  UCSR0C = (1 << UMSEL01) | (1 << UMSEL00) | (1 << UCPHA0) | (0 << UCPOL0);

  // Enable USART0 SPI Master Mode receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  // Set baud rate to 4Mbps
  UBRR0 = 1;

  // Clear the registers
  clr = UCSR0A;
  clr = UDR0;

  // TWI slave interface to CTC EcoLogic EXT
  // Enable internal pullup for TWI pins (Could be removed when real pullups have been fitted)
  PORTC |= (1 << PC4) | (1 << PC5);

  // Set the slave address
  TWAR = 0x5C << 1;

  // Enable TWI module, acks and interrupt
  TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA);

  delay(750);                                     // We have to guarantee that the array contains a valid temperature before we start the main program
  if (millis() - lastTempRequest >= delayInMillis)
  {
    for (uint8_t sensor = 0; sensor < NUM_SENSORS; sensor++)
    {
      temperature = sensors.getTempCByIndex(sensor);
      if (temperature != DEVICE_DISCONNECTED_C)
      {
        temparray[sensor][num_samp++] = temperature;
        if (num_samp > 2)
        {
          mediantemp = median3(temparray[sensor]);
          num_samp = 0;
          datalog[0xB0 + sensor * 2] = (int)mediantemp;
          datalog[0xB1 + sensor * 2] = (int)round(mediantemp * 100.0) - (int)mediantemp * 100;
        }
      }
    }
    sensors.requestTemperatures();
    lastTempRequest = millis();
  }
} // end of setup

void loop()
{
  // Check if we are in sync with the CTC master and if we can start the sampling
  if (sync && !first_sync)
  {
    start_sampling = true;                    // Start the auto-sampling!
    first_sync = true;
  }

  if (millis() - lastTempRequest >= delayInMillis)
  {
    for (uint8_t sensor = 0; sensor < NUM_SENSORS; sensor++)
    {
      temperature = sensors.getTempCByIndex(sensor);
      if (temperature != DEVICE_DISCONNECTED_C)
      {
        temparray[sensor][num_samp++] = temperature;
        if (num_samp > 2)
        {
          mediantemp = median3(temparray[sensor]);
          num_samp = 0;
          templog[0xB0 + sensor * 2] = (int)mediantemp;
          templog[0xB1 + sensor * 2] = (int)round(mediantemp * 100.0) - (int)mediantemp * 100;
        }
      }
    }
    sensors.requestTemperatures();
    lastTempRequest = millis();
  }

  // Start checking the status of the newly taken sample versus the last sent
  if (sample_done)
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

    // Check for change in ONEWIRE, normal every change of temp
    checkforchange(0xB0 , 0xB1 , B10000000);

    sample_done = false;
    start_sampling = true;      // Go for another auto sample!
    test4++;                    // Increment the sample counter
  }
  if (new_sample_available)
    PORTD |= (1 << PORTD7);      // Set the interrupt line HIGH
  else
    PORTD &= ~(1 << PORTD7);
} // end of loop
