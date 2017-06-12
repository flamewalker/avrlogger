/**
    I2C interface to SPI for CTC Ecologic EXT
    ver 1.2.151
**/

#define VER_MAJOR 1
#define VER_MINOR 2
#define VER_BUILD 151

/*
#include <DallasTemperature.h>
#include <OneWire.h>
#include <EEPROM.h>

// Defs for making a median search routine
#define MED_SORT(a,b) { if ((a)>(b)) MED_SWAP((a),(b)); }
#define MED_SWAP(a,b) { float tmp=(a);(a)=(b);(b)=tmp; }

#define NUM_MEDIAN 0    // Number of medians, if any
*/

// Number of connected OneWire sensors
#define NUM_SENSORS 10

// Basic array sizes, just need to add the number of connected OneWire sensors
#define ARRAY_SIZE 0xB0 + NUM_SENSORS * 2
#define SAMPLE_SIZE 0xAD

// TWI buffer length
#define TWI_BUFFER_LENGTH 4

/*
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Array(s) to hold the adress of the connected devices
DeviceAddress tempsensor[] = {0x28, 0x2E, 0xE8, 0x1D, 0x07, 0x00, 0x00, 0x80,    // Sensor0
                              0x28, 0x67, 0x3A, 0x1E, 0x07, 0x00, 0x00, 0x36,    // Sensor1
                              0x28, 0x6F, 0xD4, 0x28, 0x07, 0x00, 0x00, 0xAE,    // Sensor2
                              0x28, 0xAC, 0x32, 0x1D, 0x07, 0x00, 0x00, 0xE7,    // Sensor3
                              0x28, 0x0D, 0x73, 0x1D, 0x07, 0x00, 0x00, 0xA8,    // Sensor4
                              0x28, 0xB7, 0x3B, 0x1D, 0x07, 0x00, 0x00, 0xB7,    // Sensor5
                              0x28, 0x2E, 0x68, 0x1D, 0x07, 0x00, 0x00, 0x4B,    // Sensor6
                              0x28, 0x20, 0x40, 0x1D, 0x07, 0x00, 0x00, 0x9E,    // Sensor7
                              0x28, 0x41, 0x2B, 0x29, 0x07, 0x00, 0x00, 0x4D,    // Sensor8
                              0x28, 0x99, 0x6D, 0x1C, 0x07, 0x00, 0x00, 0x94     // Sensor9
                             };

float sensor_calibration[NUM_SENSORS];

// A method for converting float to array of uint8_t and back
union Convert
{
  uint8_t buf[4];
  float number;
};

static volatile union Convert convert;
*/

// Look-up table for controlling digipot to simulate 22K NTC between 26-98C
const uint8_t temp[] = {16,   33,  52,  77,  94,
                        109, 124, 145, 165, 178, 196, 213, 224, 239, 249,
                        5,    17,  26,  35,  43,  55,  62,  71,  77,  84,
                        91,   97, 107, 113, 119, 125, 130, 136, 141, 147,
                        154, 159, 164, 169, 173, 178, 182, 186, 189, 193,
                        196, 199, 203, 206, 208, 212, 216, 219, 221, 222,
                        226, 228, 229, 231, 233, 236, 237, 240, 242, 244,
                        245, 247, 248, 250, 251, 251, 252, 254
                       };

// Command buffer for DigiPot
static volatile uint8_t digi_cmd[2] = {0, 0};

// Variable to hold actual temp of hot water
static volatile uint8_t dhw_ctc = 75;

/*
// Variables for handling sampling from OneWire sensors
static float temperature, mediantemp = 0.0;
//static float medtmp[NUM_SENSORS][NUM_MEDIAN];
static float owtemp[NUM_SENSORS][4];
static float tempfiltered[NUM_SENSORS][4];
static uint32_t lastTempRequest = 0;
static uint16_t delayInMillis = 2000;
*/

/*
// State machine declarations for SPI
enum SPIState
{
  SPI_IDLE,
  SPI_DUMP
};

static volatile SPIState spi_state = SPI_IDLE;
*/

// Buffer and variables for SPI -> TWI command transfer
static volatile uint8_t spi_cmd[2] = { 0xDE, 0x64 };
static volatile uint8_t save_twi_tx;
static volatile boolean command_pending = true;

// Initialize the storage arrays
static volatile uint8_t datalog[ARRAY_SIZE];
static volatile uint8_t templog[ARRAY_SIZE];

// Debug vars
static volatile uint8_t test1, test2, test3, test4, test6, test7, test8, slask_tx1, slask_tx2, slask_rx1, slask_rx2 = 0;
static volatile uint16_t test5 = 0;

// Counter...
static volatile uint8_t count = 0;

// Flags for sampling complete / available for transfer
static volatile boolean twi_sample_done = false;
static volatile boolean new_twi_sample, new_ow_sample = false;
static volatile uint8_t twi_sample_send = B01111111;
static volatile uint8_t ow_sample_send = B00000000;
static volatile boolean start_sampling = true;

// Flags for contact with CTC Ecologic EXT
static volatile boolean sync = false;

// TWI vars
static volatile uint8_t twi_txBuffer[2];
static volatile uint8_t twi_txBufferIndex = 0;
static volatile uint8_t twi_txBufferLength = 0;

static volatile uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_rxBufferIndex = 0;

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
      // Reset RX buffer index
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

      if (twi_rxBufferIndex == 1 && twi_rxBuffer[0] == 0xFE)      // Received question from CTC Master
      {
        sync = true;                // If we got this far, we're in sync!

        if (start_sampling)         // Check flag to see if we should start the sampling process
        {
          twi_txBuffer[0] = 0;      // Always start from the first address
          count = 0;                // In case the counter somehow has changed, reset it
          start_sampling = false;
        }
      }
      else if (twi_rxBufferIndex == 2 && twi_rxBuffer[0] == twi_txBuffer[0])    // Received answer to previous address request from CTC Master
      {
        templog[count++] = twi_rxBuffer[1];
        if (count < SAMPLE_SIZE)
          twi_txBuffer[0] = count;
        else
        {
          twi_txBuffer[0] = 0xFF;   // Load with NO-OP/PING command
          count = 0;                // Reset counter ASAP to prevent out of bounds array addressing
          twi_sample_done = true;   // Set flag to indicate we have finished the sampling process
        }
      }
      break;

    // Status Codes for SLAVE TRANSMITTER mode
    case 0xA8:          // Own SLA+R has been received; ACK has been returned
    case 0xB0:          // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
      twi_txBufferIndex = 0;
      twi_txBufferLength = 0;

      if (command_pending)
      {
        save_twi_tx = twi_txBuffer[0];
        twi_txBuffer[0] = spi_cmd[0];
        twi_txBuffer[1] = spi_cmd[1];
      }

      // NO-OP/PING is a single byte
      if (twi_txBuffer[0] == 0xFF || twi_txBuffer[0] < 0xDC)
        twi_txBufferLength = 1;
      else
        twi_txBufferLength = 2;
    // Load TWDR with first byte to send in next step

    case 0xB8:          // Data byte in TWDR has been transmitted; ACK has been received
      // Load TWDR with byte to send from buffer
      TWDR = twi_txBuffer[twi_txBufferIndex++];
      // Check if there is more bytes to send
      if (twi_txBufferIndex < twi_txBufferLength)
        // ACK will be returned if there is more to send
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
      else
      {
        // NOT ACK will be returned
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
        // Restore the buffer after command sending
        if (command_pending)
        {
          twi_txBuffer[0] = save_twi_tx;
          command_pending = false;
        }
      }
      break;

    case 0xC0:          // Data byte in TWDR has been transmitted; NOT ACK has been received
    case 0xC8:          // Last data byte in TWDR has been transmitted (TWEA = "0"); ACK has been received
      // ACK will be returned
      TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
      break;

    // TWI Errors
    case 0xF8:          // No relevant state information available; TWINT = "0"
      test2 |= 32;      // Debug
      break;

    case 0x00:          // Bus Error due to an illegal START or STOP condition
      test1++;                        // Debug
      test2 |= 64;                    // Debug
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
  switch (SPDR)
  {
    case 0xFF:                        // NO-OP/PING
      if (!sync)                      // Not in sync with TWI Master
        SPDR = 0x00;
      else if (!new_twi_sample && !new_ow_sample)
        SPDR = 0x01;                  // In sync with TWI Master, no new sample available
      break;

    case 0xF0:                        // Array transfer command
      if (!sync)                      // Not in sync with TWI Master
        SPDR = 0x00;
      else if (!new_twi_sample && !new_ow_sample) // In sync with TWI Master, no new sample available
        SPDR = 0x01;
      else
      {
        SPDR = 0xFA;
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        if (new_ow_sample)
        {
          SPDR = ow_sample_send;
          while (!(SPSR & (1 << SPIF))); // Wait for next byte from Master
          SPDR = 0xFD;                   // Access SPDR to clear SPIF
          if (!new_twi_sample)
            PORTD &= ~(1 << PORTD7);     // Set the Interrupt signal LOW
        }
        else if (new_twi_sample)
        {
          SPDR = twi_sample_send;
          while (!(SPSR & (1 << SPIF))); // Wait for next byte from Master
          SPDR = 0xFD;                   // Access SPDR to clear SPIF
          if (!new_ow_sample)
            PORTD &= ~(1 << PORTD7);     // Set the Interrupt signal LOW
        }

        while (!(SPSR & (1 << SPIF))); // Wait for next byte from Master

        while (SPDR < ARRAY_SIZE)
        {
          while (SPDR < ARRAY_SIZE)          // Continue as long we get requests within the array size
          {
            SPDR = datalog[SPDR];
            while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
          }

          while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        }

        if (SPDR == 0xF0)
        {
          SPDR = (test5 >> 8);             // Load with number of ow_samples we've collected since last time, MSByte
          while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
          SPDR = (test5 & 0xFF);           // Load with number of ow_samples we've collected since last time, LSByte
          while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
          SPDR = 0xFF;                     // Access SPDR to clear SPIF
          test5 = 0;
          new_ow_sample = false;
          ow_sample_send = 0;              // Reset now that we've sent everything
          break;
        }
        if (SPDR == 0xF1)
        {
          SPDR = test4;                    // Load with number of twi_samples we've collected since last time
          while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
          SPDR = 0xFF;                     // Access SPDR to clear SPIF
          test4 = 0;
          new_twi_sample = false;
          twi_sample_send = 0;             // Reset now that we've sent everything
          break;
        }
      }
      break;

    case 0xF1:                        // Start command transfer to TWI Master
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      if (command_pending)            // Signal that we already have a unhandled command waiting
        SPDR = 0xFF;
      else
        spi_cmd[0] = SPDR;            // First byte of command sequence
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      if (command_pending)            // Signal that we already have a unhandled command waiting
        SPDR = 0xFF;
      else
        spi_cmd[1] = SPDR;            // Second byte of command sequence
      command_pending = true;
      break;

    case 0xF2:                        // Start DigiPot temp setting sequence
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      set_ctc_temp(SPDR);             // Receive the temperature and set it
      SPDR = dhw_ctc;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xFF;
      break;

    case 0xF3:                                // Start command sequence to send cmd to digipot
      while (!(SPSR & (1 << SPIF)));          // Wait for next byte from Master
      digi_cmd[0] = SPDR;                     // Receive cmd+adress byte for digipot
      while (!(SPSR & (1 << SPIF)));          // Wait for next byte from Master
      digi_cmd[1] = SPDR;                     // Receive data byte for digipot
      SPDR = xfer(digi_cmd[0], digi_cmd[1]);  // Transfer to digipot and receive answer
      while (!(SPSR & (1 << SPIF)));          // Wait for next byte from Master
      SPDR = 0xFF;
      break;

    case 0xF4:                        // Send version number
      SPDR = VER_MAJOR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = VER_MINOR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = VER_BUILD;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xFF;                    // Access SPDR to clear SPIF
      break;

    case 0xF5:                        // Fetch ALL debug variables
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = test1;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = test2;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = test3;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = count;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = twi_rxBuffer[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = twi_rxBuffer[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = twi_txBuffer[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = twi_txBuffer[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = slask_rx1;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = slask_rx2;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = slask_tx1;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = slask_tx2;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xFF;                    // Access SPDR to clear SPIF
      break;

    case 0xF6:                        // Release bus and reset TWI hardware
      TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWSTO);
      if (!sync)                      // Not in sync with TWI Master
        SPDR = 0x00;
      else if (!new_twi_sample && !new_ow_sample)
        SPDR = 0x01;                  // In sync with TWI Master, no new sample available
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xFF;                    // Access SPDR to clear SPIF
      break;
/*
    case 0xF7:                        // Program sensor_calibration
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.buf[0] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.buf[1] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.buf[2] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.buf[3] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      sensor_calibration[SPDR] = convert.number;
      EEPROM.put((SPDR * sizeof(float)), sensor_calibration[SPDR]);
      break;

    case 0xF8:                        // Read sensor_calibration
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.number = sensor_calibration[SPDR];
      SPDR = convert.buf[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.buf[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.buf[2];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.buf[3];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xFF;                    // Access SPDR to clear SPIF
      break;
*/
    case 0xA0:
      SPDR = test1;                   // Number of TWI bus errors due to illegal START or STOP condition
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xFF;                    // Access SPDR to clear SPIF
      break;

    case 0xA1:
      SPDR = test2;                   // Bitflags of TWI error conditions
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xFF;                    // Access SPDR to clear SPIF
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
      slask_tx1 = 0;
      slask_tx2 = 0;
      slask_rx1 = 0;
      slask_rx2 = 0;
      SPDR = 0xAF;
      break;
  }
} // end of SPI ISR

#if NUM_MEDIAN == 3
static float median3(float *f)
{
  MED_SORT(f[0], f[1]) ; MED_SORT(f[1], f[2]) ; MED_SORT(f[0], f[1]);
  return (f[1]) ;
}
#endif
#if NUM_MEDIAN == 5
static float median5(float *f)
{
  MED_SORT(f[0], f[1]) ; MED_SORT(f[3], f[4]) ; MED_SORT(f[0], f[3]);
  MED_SORT(f[1], f[4]) ; MED_SORT(f[1], f[2]) ; MED_SORT(f[2], f[3]);
  MED_SORT(f[1], f[2]);
  return (f[2]);
}
#endif
#if NUM_MEDIAN == 6
static float median6(float *f)
{
  MED_SORT(f[1], f[2]); MED_SORT(f[3], f[4]);
  MED_SORT(f[0], f[1]); MED_SORT(f[2], f[3]); MED_SORT(f[4], f[5]);
  MED_SORT(f[1], f[2]); MED_SORT(f[3], f[4]);
  MED_SORT(f[0], f[1]); MED_SORT(f[2], f[3]); MED_SORT(f[4], f[5]);
  MED_SORT(f[1], f[2]); MED_SORT(f[3], f[4]);
  return ( f[2] + f[3] ) * 0.5;
  /* MED_SORT(f[2], f[3]) results in lower median in f[2] and upper median in f[3] */
}
#endif

static boolean checkforchange(uint8_t x_start , uint8_t x_stop)
{
  boolean new_sample_available = false;
  // Check for change in the specified address and set according block, return true if change has occured
  for (uint8_t x = x_start ; x <= x_stop ; x++)
    if (datalog[x] != templog[x])
    {
      datalog[x] = templog[x];
      new_sample_available = true;
    }
  return new_sample_available;
} // end of checkforchange

static void set_ctc_temp(uint8_t t)
{
  int8_t dir = 0;
  if (t > dhw_ctc)                // Correct for hysteres in CTC circuits
    dir = 1;
  else if (t < dhw_ctc)
    dir = -1;

  if (t >= 99)                    // Check upper constraint
  {
    dhw_ctc = 99;
    xfer(152, 129);               // Send top scale command to RDAC1 & RDAC2
  }
  else if (t <= 25)               // Check lower constraint
  {
    dhw_ctc = 25;
    xfer(152, 1);                 // Send bottom scale command to RDAC1 & RDAC2
  }
  else if (t > 40)
  {
    dhw_ctc = t;
    xfer(32, temp[t - 26] + dir); // Load input registers
    xfer(33, 255);
    xfer(104, 0);                 // Transfer input registers to RDACs
  }
  else
  {
    dhw_ctc = t;
    xfer(32, 0);                  // Load input registers
    xfer(33, temp[t - 26] + dir);
    xfer(104, 0);                 // Transfer input registers to RDACs
  }
}

static uint8_t xfer(uint8_t data1, uint8_t data2)
{
  PORTD &= ~(1 << PORTD5);            // SYNC LOW
  while (!( UCSR0A & (1 << UDRE0)) );
  UDR0 = data1;                       // First byte
  UDR0 = data2;                       // Second byte
  while (!(UCSR0A & ( 1 << RXC0)) );
  uint8_t clr = UDR0;                 // Receive first byte
  while (!(UCSR0A & ( 1 << RXC0)) );
  clr = UDR0;                         // Receive second byte
  PORTD |= (1 << PORTD5);             // SYNC HIGH
  PORTD &= ~(1 << PORTD5);
  while (!( UCSR0A & (1 << UDRE0)));
  UDR0 = 0;
  UDR0 = 0;
  while (!(UCSR0A & ( 1 << RXC0)) );
  clr = UDR0;
  while (!(UCSR0A & ( 1 << RXC0)) );
  clr = UDR0;
  PORTD |= (1 << PORTD5);
  return clr;
}

void setup()
{
  // Initialize ports
  // Disable all pull-ups
  MCUCR |= (1 << PUD);

/*
  // Initialize sensor calibration values from EEPROM
  for (uint8_t x = 0; x < NUM_SENSORS; x++)
    EEPROM.get((x * sizeof(float)), sensor_calibration[x]);

  // Initialize OneWire sensors
  sensors.begin();
  sensors.setResolution(12);                  // We want all sensors at 12 bit
  sensors.requestTemperatures();              // We have to guarantee that the array contains a valid temperature before we start the main program
  for (uint8_t x = 0; x < NUM_SENSORS; x++)
  {
    temperature = sensors.getTempC(tempsensor[x]);
    if (temperature != DEVICE_DISCONNECTED_C)
    {
      // Apply sensor calibration
      temperature = temperature + sensor_calibration[x];

      mediantemp = lrintf(temperature * 10.0) * 0.1;                        // Round to nearest, one decimal
      templog[0xB0 + x * 2] = mediantemp;                                   // Split into integer and
      templog[0xB1 + x * 2] = mediantemp * 100 - (uint8_t)mediantemp * 100; // two decimals
      datalog[0xB0 + x * 2] = templog[0xB0 + x * 2];
      datalog[0xB1 + x * 2] = templog[0xB1 + x * 2];

      // Init the filter
      owtemp[x][0] = temperature;
      owtemp[x][1] = temperature;
      owtemp[x][2] = temperature;
      owtemp[x][3] = temperature;
      tempfiltered[x][0] = temperature;
      tempfiltered[x][1] = temperature;
      tempfiltered[x][2] = temperature;
      tempfiltered[x][3] = temperature;
    }
    else
    {
      templog[0xB0 + x * 2] = 0;
      templog[0xB1 + x * 2] = 0;
      datalog[0xB0 + x * 2] = templog[0xB0 + x * 2];
      datalog[0xB1 + x * 2] = templog[0xB1 + x * 2];
    }
  }
  sensors.setWaitForConversion(false);        // Now that we have a starting value in the array we don't have to wait for conversions anymore
  sensors.requestTemperatures();
  lastTempRequest = millis();
*/

  // Initialize ports
  // Set Port B1, B0 output
  DDRB |= (1 << DDB1) | (1 << DDB0);

  // Set Port C0, C1, C2, C3 analog input
  DDRC &= ~((1 << DDC3) | (1 << DDC2) | (1 << DDC1) | (1 << DDC0));
  DIDR0 |= (1 << ADC3D) | (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D);

  // Set Port D7, D6 output, sample_ready signal
  DDRD |= (1 << DDD7) | (1 << DDD6);

  // Initialize all the interfaces
  // SPI slave interface to Raspberry Pi
  // Set Port B4 output, SPI MISO
  DDRB |= (1 << DDB4);

/*
  // Interrupt enabled, SPI enabled, MSB first, Slave, CLK low when idle, Sample on leading edge of CLK (SPI Mode 0)
  SPCR = (1 << SPIE) | (1 << SPE);
*/
  // Interrupt enabled, SPI enabled, MSB first, Slave, CLK high when idle, Sample on leading edge of CLK (SPI Mode 2)
  SPCR = (1 << SPIE) | (1 << SPE) | (1 << CPOL);

  // Clear the registers
  uint8_t clr = SPSR;
  clr = SPDR;
  SPDR = 0x00;

  // Initialize USART0 as SPI Master interface to digital potentiometer
  UBRR0 = 0;

  // Set Port D4 output, Port D5 output: SPI CLK, SS
  DDRD |= (1 << DDD4) | (1 << DDD5);

/*
  // Enable USART0 SPI Master Mode, MSB first, CLK low when idle, Sample on trailing edge of CLK (SPI Mode 1)
  UCSR0C = (1 << UMSEL01) | (1 << UMSEL00) | (1 << UCPHA0);
*/
  // Enable USART0 SPI Master Mode, MSB first, CLK high when idle, Sample on leading edge of CLK (SPI Mode 2)
  UCSR0C = (1 << UMSEL01) | (1 << UMSEL00) | (1 << UCPOL0);

  // Enable USART0 SPI Master Mode receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  // Set baud rate to 4Mbps
  UBRR0 = 1;

  // Clear the registers
  clr = UCSR0A;
  clr = UDR0;

  // TWI slave interface to CTC EcoLogic EXT
  // Set the slave address
  TWAR = 0x5C << 1;

  // Enable TWI module, acks and interrupt
  TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA);

  // Command DigiPot RESET (command number 14)
  xfer(176, 0);

} // end of setup

void loop()
{
/*
  if (millis() - lastTempRequest >= delayInMillis)
  {
    for (uint8_t x = 0; x < NUM_SENSORS; x++)
    {
      temperature = sensors.getTempC(tempsensor[x]);
      if (temperature != DEVICE_DISCONNECTED_C)
      {
        // Apply sensor calibration
        temperature = temperature + sensor_calibration[x];

        // Input for filter
        owtemp[x][0] = owtemp[x][1];
        owtemp[x][1] = owtemp[x][2];
        owtemp[x][2] = owtemp[x][3];
        owtemp[x][3] = temperature;

        // Butterworth filter with cutoff frequency 0.01*sample frequency (FS=1.33Hz)
        tempfiltered[x][0] = tempfiltered[x][1];
        tempfiltered[x][1] = tempfiltered[x][2];
        tempfiltered[x][2] = tempfiltered[x][3];

        tempfiltered[x][3] = (owtemp[x][0] + owtemp[x][3] + 3 * (owtemp[x][1] + owtemp[x][2])) / 3.430944333e+04 + (0.8818931306 * tempfiltered[x][0]) + (-2.7564831952 * tempfiltered[x][1]) + (2.8743568927 * tempfiltered[x][2]);

/*
		// Fill the median temporary storage
        medtmp[x][0] = lrintf(tempfiltered[x][0] * 10.0) * 0.1;
        medtmp[x][1] = lrintf(tempfiltered[x][1] * 10.0) * 0.1;
        medtmp[x][2] = lrintf(tempfiltered[x][2] * 10.0) * 0.1;
        medtmp[x][3] = lrintf(tempfiltered[x][3] * 10.0) * 0.1;
        medtmp[x][4] = lrintf(tempfiltered[x][4] * 10.0) * 0.1;
        medtmp[x][5] = lrintf(tempfiltered[x][5] * 10.0) * 0.1;

        mediantemp = lrintf(median6(medtmp[x]) * 10.0) * 0.1;
//        mediantemp = median6(medtmp[x]);
*/
/*
        mediantemp = lrintf(tempfiltered[x][3] * 10.0) * 0.1;
        templog[0xB0 + x * 2] = mediantemp;
        templog[0xB1 + x * 2] = mediantemp * 100 - (uint8_t)mediantemp * 100;
      }
      else
      {
        templog[0xB0 + x * 2] = 0;
        templog[0xB1 + x * 2] = 0;
      }
    }
    sensors.requestTemperatures();
    lastTempRequest = millis();
    test5++;

    // Check for change in ONEWIRE, normal every change of temp (could be ~750ms)
    if (sync)
      if (checkforchange(0xB0 , 0xAF + NUM_SENSORS * 2))
        ow_sample_send = B10000000;

    if (ow_sample_send != 0)
    {
      PORTD &= ~(1 << PORTD7);    // Set the Interrupt signal LOW
      new_ow_sample = true;
      PORTD |= (1 << PORTD7);     // Set the Interrupt signal HIGH
    }
  }
*/

  // Start checking the status of the newly taken sample versus the last sent
  if (twi_sample_done)
  {
    // Check for change in SYSTIME, normal every minute
    if (checkforchange(0x73 , 0x75))
      twi_sample_send |= B00000001;

    // Check for change in CURRENT, normal every change of temp
    if (checkforchange(0x8C , 0x99))
      twi_sample_send |= B00000010;
    if (checkforchange(0x9B , 0xA0))
      twi_sample_send |= B00000010;
    if (checkforchange(0xA8 , 0xAD))
      twi_sample_send |= B00000010;

    // Check for change in HISTORICAL, normal every hour and week
    if (checkforchange(0x76 , 0x7E))
      twi_sample_send |= B00000100;
    if (checkforchange(0x80 , 0x84))
      twi_sample_send |= B00000100;
    if (checkforchange(0x87 , 0x8B))
      twi_sample_send |= B00000100;

    // Check for change in SETTINGS, hardly ever any changes
    if (checkforchange(0x00 , 0x68))
      twi_sample_send |= B00001000;

    // Check for change in ALARMS, almost never ever any changes
    if (checkforchange(0xA1 , 0xA4))
      twi_sample_send |= B00010000;

    // Check for change in LAST_24H,  normal change once every hour
    if (checkforchange(0x85 , 0x86))
      twi_sample_send |= B00100000;
    if (checkforchange(0x7F , 0x7F))
      twi_sample_send |= B00100000;

    // Check for change in STATUS, normal change every minute
    if (checkforchange(0x9A , 0x9A))
      twi_sample_send |= B01000000;
    if (checkforchange(0xA5 , 0xA7))
      twi_sample_send |= B01000000;

    twi_sample_done = false;
    start_sampling = true;      // Go for another auto sample!
    test4++;                    // Increment the sample counter

    if (twi_sample_send != 0)
    {
      PORTD &= ~(1 << PORTD7);    // Set the Interrupt signal LOW
      new_twi_sample = true;
      PORTD |= (1 << PORTD7);     // Set the Interrupt signal HIGH
    }
  }
} // end of loop
