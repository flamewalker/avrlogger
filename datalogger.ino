/**
    I2C interface to SPI for CTC Ecologic EXT
    ver 1.4.0
**/

#define VER_MAJOR 1
#define VER_MINOR 4
#define VER_BUILD 0

#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

// Defs for making a median search routine
#define MED_SORT(a,b) { if ((a)>(b)) MED_SWAP((a),(b)); }
#define MED_SWAP(a,b) { float tmp=(a);(a)=(b);(b)=tmp; }

#define NUM_MEDIAN 0    // Number of medians, if any

// Number of connected OneWire sensors
#define NUM_SENSORS 11

// Basic array sizes, just need to add the number of connected OneWire sensors
#define ARRAY_SIZE 0xCD
#define SAMPLE_SIZE 0xAD

// TWI buffer length
#define TWI_BUFFER_LENGTH 6

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
                              0x28, 0x99, 0x6D, 0x1C, 0x07, 0x00, 0x00, 0x94,    // Sensor9
                              0x3B, 0x3F, 0xA8, 0x5D, 0x06, 0xD8, 0x4C, 0x39     // Sensor10  (Thermocouple type K, via MAX31850K
                             };

float sensor_calibration[NUM_SENSORS];

// A method for converting numbers for transfer with SPI
union Convert
{
  uint8_t nr_8[4];
  uint16_t nr_16;
  uint32_t nr_32;
  float number;
};

static volatile union Convert convert;

// Look-up table for controlling digipot to simulate 22K NTC between 26-98C
const uint8_t temp[] PROGMEM = {16,   33,  52,  77,  94,
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
static uint8_t check_dhw = 75;

// Variables for handling sampling from OneWire sensors
static float temperature, mediantemp = 0.0;
//static float medtmp[NUM_SENSORS][NUM_MEDIAN];
static float owtemp[NUM_SENSORS][4];
static float tempfiltered[NUM_SENSORS][4];
static uint32_t lastTempRequest = 0;
static uint32_t time_now = 0;
static uint32_t lastCheck = 0;
static uint16_t delayInMillis = 750;
static uint16_t checkDelay = 1000;

// Variables for handling reading of the ADC
const float InternalReferenceVoltage = 1.100;  // Actually not measured... yet
const float ReferenceResistor = 1270.0;        // Actually not measured... yet
static volatile float AnalogReferenceVoltage = 3.300;   // Ideally it should be this... but we measure it later
static volatile uint16_t rawADC = 0;
static volatile uint32_t adjusted_ADC = 0;
static volatile uint32_t oversampled_ADC = 0;
const uint8_t nr_extra_bits = 2;                           // Number of additional bits over 10
const uint16_t nr_oversamples = 1 << (nr_extra_bits * 2);  // Number of oversamples needed
const uint32_t adc_divisor = 1024UL << nr_extra_bits;      // Divisor to be used
const float lsb_adjust = (1 + nr_extra_bits) * 0.5;        // Adjustment since we cant reach max value
static volatile uint16_t sample_counter = 0;
static volatile float voltage_in = 0.0;
static volatile float solar_resistor = 0.0;
static volatile float solar_temp = 0.0;
static volatile boolean adcDone = false;
static float tank1_lower = 0.0;
static float tank1_upper = 0.0;
static float wood_burner_in = 0.0;
static float wood_burner_out = 0.0;
static int16_t wood_burner_smoke = 0;
static float adctemp[4];
static float adcfiltered[4];
static uint32_t adc_lastCheck = 0;
static uint16_t adc_checkDelay = 2000;

// Variables to control relays
static volatile boolean solar_pump_on = false;
static volatile boolean laddomat_on = false;

// Buffer and variables for SPI -> TWI command transfer
static volatile uint8_t spi_cmd[2] = { 0xDE, 0x64 };
static volatile uint8_t save_twi_tx;
static volatile boolean command_pending = true;

// Initialize the storage arrays
static volatile uint8_t datalog[ARRAY_SIZE];
static volatile uint8_t templog[ARRAY_SIZE];

// Debug vars
static volatile uint32_t test1 = 0;
static volatile uint8_t test2 = 0;
static volatile uint8_t test3 = 0;
static volatile uint32_t test4 = 0;
static volatile uint32_t test5 = 0;

// Counter...
static volatile uint8_t count = 0;

// Flags for sampling complete / available for transfer
static volatile boolean twi_sample_done = false;
static volatile boolean new_sample = false;
static volatile uint16_t sample_send = 0;
static volatile boolean start_sampling = true;
static volatile boolean ok_sample_ow = false;
static uint32_t twi_lastCheck = 0;
static uint16_t twi_sampletime = 0;
static volatile uint16_t twi_accum_time = 0;
static volatile uint16_t twi_min_time = 3000;
static volatile uint16_t twi_max_time = 2000;

// Flags for contact with CTC Ecologic EXT
static volatile boolean sync = false;
static boolean first_run = true;

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
      SPDR = 0xDF;
      if (!sync)                      // Not in sync with TWI Master
        SPDR = 0x00;
      else if (!new_sample)
        SPDR = 0x01;                  // In sync with TWI Master, no new sample available
      break;

    case 0xF0:                        // Array transfer command
      if (!sync)                      // Not in sync with TWI Master
        SPDR = 0x00;
      else if (!new_sample)
        SPDR = 0x01;
      else
      {
        PORTD &= ~(1 << PORTD7);     // Set the Interrupt signal LOW
        SPDR = 0xFA;
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        convert.nr_16 = sample_send;
        SPDR = convert.nr_8[0];
        while (!(SPSR & (1 << SPIF))); // Wait for next byte from Master
        SPDR = convert.nr_8[1];
        while (!(SPSR & (1 << SPIF))); // Wait for next byte from Master
        SPDR = 0xFD;                   // Access SPDR to clear SPIF

        while (!(SPSR & (1 << SPIF)));     // Wait for next byte from Master

        while (SPDR < ARRAY_SIZE)
        {
          while (SPDR < ARRAY_SIZE)        // Continue as long we get requests within the array size
          {
            SPDR = datalog[SPDR];
            while (!(SPSR & (1 << SPIF))); // Wait for next byte from Master
          }
          while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        }
        new_sample = false;
        sample_send = 0;                   // Reset now that we have sent everything

        // Reset the upper bits of system status flags
        templog[0xCC] &= ~240;
        datalog[0xCC] = templog[0xCC];

        // Send test5
        convert.nr_32 = test5;
        SPDR = convert.nr_8[0];          // Load with number of ow_samples we've collected since last time, MSByte
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        SPDR = convert.nr_8[1];          // Load with number of ow_samples we've collected since last time, LSByte
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        SPDR = convert.nr_8[2];          // Load with number of ow_samples we've collected since last time, LSByte
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        SPDR = convert.nr_8[3];          // Load with number of ow_samples we've collected since last time, LSByte
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master

        // Send test4
        convert.nr_32 = test4;
        SPDR = convert.nr_8[0];          // Load with number of twi_samples we've collected since last time
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        SPDR = convert.nr_8[1];          // Load with number of twi_samples we've collected since last time
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        SPDR = convert.nr_8[2];          // Load with number of twi_samples we've collected since last time
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        SPDR = convert.nr_8[3];          // Load with number of twi_samples we've collected since last time
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master

        // Send twi timing vars
        convert.nr_16 = twi_min_time;
        SPDR = convert.nr_8[0];
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
        SPDR = convert.nr_8[1];
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
        convert.nr_16 = twi_max_time;
        SPDR = convert.nr_8[0];
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
        SPDR = convert.nr_8[1];
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
        convert.nr_16 = twi_accum_time;
        SPDR = convert.nr_8[0];
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
        SPDR = convert.nr_8[1];
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

        // Send test2
        SPDR = test2;                   // Bitflags of TWI error conditions
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

        // If test2 contains error bit6 then Send test1
        if (test2 & 64)
        {
          convert.nr_32 = test1;          // Number of TWI bus errors due to illegal START or STOP condition
          SPDR = convert.nr_8[0];
          while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
          SPDR = convert.nr_8[1];
          while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
          SPDR = convert.nr_8[2];
          while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
          SPDR = convert.nr_8[3];
          while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
        }

        // Reset debug vars
        test2 = 0;
        test3 = 0;
        twi_rxBuffer[2] = 0;
        twi_rxBuffer[3] = 0;

        SPDR = 0xD0;                    // Access SPDR to clear SPIF
      }
      break;

    case 0xF1:                        // Start command transfer to TWI Master
      SPDR = 0xD1;
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
      SPDR = 0xD2;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      set_ctc_temp(SPDR);             // Receive the temperature and set it
      SPDR = dhw_ctc;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xD2;
      break;

    case 0xF3:                                // Start command sequence to send cmd to digipot
      SPDR = 0xD3;
      while (!(SPSR & (1 << SPIF)));          // Wait for next byte from Master
      digi_cmd[0] = SPDR;                     // Receive cmd+adress byte for digipot
      while (!(SPSR & (1 << SPIF)));          // Wait for next byte from Master
      digi_cmd[1] = SPDR;                     // Receive data byte for digipot
      SPDR = xfer(digi_cmd[0], digi_cmd[1]);  // Transfer to digipot and receive answer
      while (!(SPSR & (1 << SPIF)));          // Wait for next byte from Master
      SPDR = 0xD3;
      break;

    case 0xF4:                        // Send version number
      SPDR = VER_MAJOR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = VER_MINOR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = VER_BUILD;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = templog[0xCC];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xD4;                    // Access SPDR to clear SPIF
      break;

    case 0xF5:                        // Fetch ALL debug variables
      SPDR = 0xD5;
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
      SPDR = twi_rxBuffer[2];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = twi_rxBuffer[3];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
/*
      SPDR = twi_rxBuffer[4];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = twi_rxBuffer[5];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
*/
      SPDR = 0xD5;                    // Access SPDR to clear SPIF
      break;

    case 0xF6:                        // Release bus and reset TWI hardware
      SPDR = 0xD6;
      TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWSTO);
      if (!sync)                      // Not in sync with TWI Master
        SPDR = 0x00;
      else if (!new_sample)
        SPDR = 0x01;                  // In sync with TWI Master, no new sample available
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xD6;                    // Access SPDR to clear SPIF
      break;

    case 0xF7:                        // Program sensor_calibration
      SPDR = 0xD7;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[0] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[1] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[2] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[3] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      sensor_calibration[SPDR] = convert.number;
      EEPROM.put((SPDR * sizeof(float)), sensor_calibration[SPDR]);
      break;

    case 0xF8:                        // Read sensor_calibration
      SPDR = 0xD8;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.number = sensor_calibration[SPDR];
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[2];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[3];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xD8;                    // Access SPDR to clear SPIF
      break;

    case 0xF9:                        // Collect debug variables
      convert.nr_16 = twi_min_time;
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_16 = twi_max_time;
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_16 = twi_accum_time;
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xD9;                    // Access SPDR to clear SPIF
      break;

    case 0xFA:
      solar_pump_on = true;
      SPDR = solar_pump_on;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xDA;                    // Access SPDR to clear SPIF
      break;

    case 0xFB:
      solar_pump_on = false;
      SPDR = solar_pump_on;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xDB;                    // Access SPDR to clear SPIF
      break;

    case 0xFC:
      laddomat_on = true;
      SPDR = laddomat_on;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xDC;                    // Access SPDR to clear SPIF
      break;

    case 0xFD:
      laddomat_on = false;
      SPDR = laddomat_on;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xDD;                    // Access SPDR to clear SPIF
      break;

    case 0xA0:
      convert.nr_32 = test1;          // Number of TWI bus errors due to illegal START or STOP condition
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[2];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[3];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xB0;                    // Access SPDR to clear SPIF
      break;

    case 0xA1:
      // Send test5
      convert.nr_32 = test5;
      SPDR = convert.nr_8[0];          // Load with number of ow_samples we've collected since last time, MSByte
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[1];          // Load with number of ow_samples we've collected since last time, LSByte
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[2];          // Load with number of ow_samples we've collected since last time, LSByte
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[3];          // Load with number of ow_samples we've collected since last time, LSByte
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master

      // Send test4
      convert.nr_32 = test4;
      SPDR = convert.nr_8[0];          // Load with number of twi_samples we've collected since last time
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[1];          // Load with number of twi_samples we've collected since last time
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[2];          // Load with number of twi_samples we've collected since last time
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[3];          // Load with number of twi_samples we've collected since last time
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master

      // Send test2
      SPDR = test2;                   // Bitflags of TWI error conditions
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xB1;                    // Access SPDR to clear SPIF
      break;

    case 0xA2:
      twi_min_time = 3000;
      twi_max_time = 2000;
      twi_accum_time = 0;
      SPDR = 0xB2;
      break;

    case 0xAF:
      test2 = 0;
      test3 = 0;
      twi_rxBuffer[2] = 0;
      twi_rxBuffer[3] = 0;
      SPDR = 0xBF;
      break;

    default:
      SPDR = 0xAA;
      break;
  }
} // end of SPI ISR

/*
  ADC ISR
*/
ISR (ADC_vect)
{
  oversampled_ADC += ADC;

  if (++sample_counter >= nr_oversamples)
  {
    ADCSRA &= ~((1 << ADATE) | (1 << ADIE));
    adcDone = true;
    adjusted_ADC = (oversampled_ADC >> nr_extra_bits);
    oversampled_ADC = 0;
    sample_counter = 0;
  }
} // end of ADC ISR

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
    xfer(32, pgm_read_byte_near(&temp[t - 26]) + dir); // Load input registers
    xfer(33, 255);
    xfer(104, 0);                 // Transfer input registers to RDACs
  }
  else
  {
    dhw_ctc = t;
    xfer(32, 0);                  // Load input registers
    xfer(33, pgm_read_byte_near(&temp[t - 26]) + dir);
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

static void measureAVcc()
{
  // Save the ADC state and MUX
  uint8_t saveADCSRA = ADCSRA;
  uint8_t saveADMUX = ADMUX;

  // Disable interrupt and auto trigger
  ADCSRA &= ~((1 << ADIE) | (1 << ADATE));

  // Clear and select AVcc as reference and Bandgap voltage as source
  ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);

  // Set prescaler to CLK/128 = 125kHz
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  // Wait for the ADC to finish any previous conversion
  while ((ADCSRA & (1 << ADSC)));

  // Wait for Vbg to stabilize
  delayMicroseconds(350);

  // Clear the flag and start a new conversion
  ADCSRA |= (1 << ADSC) | (1 << ADIF);

  while (!(ADCSRA & (1 << ADIF)));       // Wait for the ADC to finish conversion
  rawADC = ADC;

  // Calculate the value of AVcc
  AnalogReferenceVoltage = InternalReferenceVoltage / (rawADC + 0.5) * 1024.0;

  // Reset the ADC state
  ADMUX = saveADMUX;
  ADCSRA = saveADCSRA;
}

void setup()
{
  // Check the reset cause
  templog[0xCC] = (MCUSR << 4);
  MCUSR = 0;

  // Initialize WDT
  wdt_enable(WDTO_8S);

  // Initialize ports
  // Disable all pull-ups
  MCUCR |= (1 << PUD);

  // Set Port B1, B0 output, solarpump relay
  DDRB |= (1 << DDB1) | (1 << DDB0);

  // Set Port C0, C1, C2, C3 analog input, disable digital buffers to save power
  DDRC &= ~((1 << DDC3) | (1 << DDC2) | (1 << DDC1) | (1 << DDC0));
  DIDR0 |= (1 << ADC3D) | (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D);

  // Set Port D7, D6 output, sample_ready signal and laddomat relay
  DDRD |= (1 << DDD7) | (1 << DDD6);

  // Initialize ADC
  // Clear ADMUX and set Vref to AVcc, ADC0 selected as source
  ADMUX = (1 << REFS0);

  // ADC enable, start first conversion, prescaler CLK/32 = 500kHz
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS0);

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
    if (temperature != DEVICE_DISCONNECTED_C && !isnan(temperature))
    {
      // Apply sensor calibration
      temperature = temperature + sensor_calibration[x];

      if (x != 10)
      {
        mediantemp = lrintf(temperature * 10.0) * 0.1;                        // Round to nearest, one decimal
        templog[0xB0 + x * 2] = mediantemp;                                   // Split into integer and
        templog[0xB1 + x * 2] = mediantemp * 100 - (uint8_t)mediantemp * 100; // two decimals
        datalog[0xB0 + x * 2] = templog[0xB0 + x * 2];
        datalog[0xB1 + x * 2] = templog[0xB1 + x * 2];
      }
      else
      {
        convert.nr_16 = (int16_t)lrintf(temperature);
        templog[0xB0 + x * 2] = convert.nr_8[0];                              // Split into high and
        templog[0xB1 + x * 2] = convert.nr_8[1];                              // low part of int
        datalog[0xB0 + x * 2] = templog[0xB0 + x * 2];
        datalog[0xB1 + x * 2] = templog[0xB1 + x * 2];
      }

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
/*    else
    {
      templog[0xB0 + x * 2] = 0;
      templog[0xB1 + x * 2] = 0;
      datalog[0xB0 + x * 2] = templog[0xB0 + x * 2];
      datalog[0xB1 + x * 2] = templog[0xB1 + x * 2];
    }
*/
  }
  sensors.setWaitForConversion(false);        // Now that we have a starting value in the array we don't have to wait for conversions anymore
  sensors.requestTemperatures();
  time_now = millis();
  lastTempRequest = time_now;

  tank1_lower = lrintf(tempfiltered[0][3] * 10.0) * 0.1;
  tank1_upper = lrintf(tempfiltered[1][3] * 10.0) * 0.1;
  check_dhw = lrintf(tempfiltered[3][3]);
  wood_burner_in = lrintf(tempfiltered[6][3] * 10.0) * 0.1;
  wood_burner_out = lrintf(tempfiltered[7][3] * 10.0) * 0.1;
  wood_burner_smoke = (int16_t)lrintf(tempfiltered[10][3]);

  // Initialize all the interfaces
  // SPI slave interface to Raspberry Pi
  // Set Port B4 output, SPI MISO
  DDRB |= (1 << DDB4);

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

  // Wait for the ADC to finish the first conversion
  while ((ADCSRA & (1 << ADSC)));

  solar_resistor = ReferenceResistor / ((1024.0 / (ADC + 0.5)) - 1.0);
  solar_temp = (solar_resistor - 1000.0) / 3.75;

  templog[0xCA] = solar_temp;
  templog[0xCB] = solar_temp * 100 - (uint8_t)solar_temp * 100;
  datalog[0xCA] = templog[0xCA];
  datalog[0xCB] = templog[0xCB];

  // Input for filter
  adctemp[0] = solar_temp;
  adctemp[1] = solar_temp;
  adctemp[2] = solar_temp;
  adctemp[3] = solar_temp;

  // Butterworth filter with cutoff frequency 0.01*sample frequency (FS=1.33Hz)
  adcfiltered[0] = solar_temp;
  adcfiltered[1] = solar_temp;
  adcfiltered[2] = solar_temp;
  adcfiltered[3] = solar_temp;

  // Get a updated value of AVcc
  measureAVcc();

  // Initialize the timekeeping vars
  time_now = millis();
  lastTempRequest = time_now;
  adc_lastCheck = time_now;
  lastCheck = time_now;
  twi_lastCheck = time_now;
} // end of setup

void loop()
{
  time_now = millis();

  // Start checking the status of the newly taken sample versus the last sent
  if (twi_sample_done)
  {
    ok_sample_ow = true;

    twi_sampletime = time_now - twi_lastCheck;
    if (twi_sampletime > 0)
      twi_accum_time = (twi_accum_time + twi_sampletime) * 0.5;
    else
      twi_accum_time = twi_sampletime;

    if (twi_sampletime < twi_min_time)
      twi_min_time = twi_sampletime;

    if (twi_sampletime > twi_max_time)
      twi_max_time = twi_sampletime;

    // Check for change in SYSTIME, normal every minute
    if (checkforchange(0x73 , 0x75))
      sample_send |= 1;

    // Check for change in CURRENT, normal every change of temp
    if (checkforchange(0x8C , 0x99))
      sample_send |= 2;
    if (checkforchange(0x9B , 0xA0))
      sample_send |= 2;
    if (checkforchange(0xA8 , 0xAD))
      sample_send |= 2;

    // Check for change in HISTORICAL, normal every hour and week
    if (checkforchange(0x76 , 0x7E))
      sample_send |= 4;
    if (checkforchange(0x80 , 0x84))
      sample_send |= 4;
    if (checkforchange(0x87 , 0x8B))
      sample_send |= 4;

    // Check for change in SETTINGS, hardly ever any changes
    if (checkforchange(0x00 , 0x68))
      sample_send |= 8;

    // Check for change in ALARMS, almost never ever any changes
    if (checkforchange(0xA1 , 0xA4))
      sample_send |= 16;

    // Check for change in LAST_24H,  normal change once every hour
    if (checkforchange(0x85 , 0x86))
      sample_send |= 32;
    if (checkforchange(0x7F , 0x7F))
      sample_send |= 32;

    // Check for change in STATUS, normal change every minute
    if (checkforchange(0x9A , 0x9A))
      sample_send |= 64;
    if (checkforchange(0xA5 , 0xA7))
      sample_send |= 64;

    twi_sample_done = false;
    twi_lastCheck = millis();
    start_sampling = true;      // Go for another auto sample!
    test4++;                    // Increment the sample counter

    if (sample_send & 127)
      first_run = false;
  }

  if (ok_sample_ow && (time_now - lastTempRequest) >= delayInMillis)
  {
    for (uint8_t x = 0; x < NUM_SENSORS; x++)
    {
      temperature = sensors.getTempC(tempsensor[x]);
      if (temperature != DEVICE_DISCONNECTED_C && !isnan(temperature))
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
        if (x != 10)
        {
          mediantemp = lrintf(tempfiltered[x][3] * 10.0) * 0.1;
          templog[0xB0 + x * 2] = mediantemp;
          templog[0xB1 + x * 2] = mediantemp * 100 - (uint8_t)mediantemp * 100;
        }
        else
        {
          convert.nr_16 = (int16_t)lrintf(tempfiltered[x][3]);
          templog[0xB0 + x * 2] = convert.nr_8[0];                                      // Split into high and
          templog[0xB1 + x * 2] = convert.nr_8[1];                                      // low part of int
        }
      }
/*      else
      {
        templog[0xB0 + x * 2] = 0;
        templog[0xB1 + x * 2] = 0;
      }
*/
    }
    sensors.requestTemperatures();
    lastTempRequest = time_now;
    test5++;
    ok_sample_ow = false;

    // Check for change in ONEWIRE, normal every change of temp (could be ~750ms)
    if (sync)
      if (checkforchange(0xB0 , 0xAF + NUM_SENSORS * 2))
        sample_send |= 128;

    tank1_lower = lrintf(tempfiltered[0][3] * 10.0) * 0.1;
    tank1_upper = lrintf(tempfiltered[1][3] * 10.0) * 0.1;
    check_dhw = lrintf(tempfiltered[3][3]);
    wood_burner_in = lrintf(tempfiltered[6][3] * 10.0) * 0.1;
    wood_burner_out = lrintf(tempfiltered[7][3] * 10.0) * 0.1;
    wood_burner_smoke = (int16_t)lrintf(tempfiltered[10][3]);
  }

  if (adcDone)
  {
    adcDone = false;
    voltage_in = AnalogReferenceVoltage * (adjusted_ADC + lsb_adjust) / adc_divisor;
    solar_resistor = ReferenceResistor / ((adc_divisor / (adjusted_ADC + lsb_adjust)) - 1.0);
    solar_temp = (solar_resistor - 1000.0) / 3.75;

    // Input for filter
    adctemp[0] = adctemp[1];
    adctemp[1] = adctemp[2];
    adctemp[2] = adctemp[3];
    adctemp[3] = solar_temp;

    // Butterworth filter with cutoff frequency 0.01*sample frequency (FS=1.33Hz)
    adcfiltered[0] = adcfiltered[1];
    adcfiltered[1] = adcfiltered[2];
    adcfiltered[2] = adcfiltered[3];

    adcfiltered[3] = (adctemp[0] + adctemp[3] + 3 * (adctemp[1] + adctemp[2])) / 3.430944333e+04 + (0.8818931306 * adcfiltered[0]) + (-2.7564831952 * adcfiltered[1]) + (2.8743568927 * adcfiltered[2]);

    solar_temp = lrintf(adcfiltered[3] * 10.0) * 0.1;     // Round to one decimal
  }

  if ((time_now - adc_lastCheck) >= adc_checkDelay)
  {
    adc_lastCheck = time_now;
    templog[0xCA] = solar_temp;
    templog[0xCB] = solar_temp * 100 - (uint8_t)solar_temp * 100;
  }

  if ((time_now - lastCheck) >= checkDelay)
  {
    lastCheck = time_now;

    if ((solar_temp - tank1_lower) <= 4.0)
    {
      solar_pump_on = false;
      templog[0xCA] = solar_temp;
      templog[0xCB] = solar_temp * 100 - (uint8_t)solar_temp * 100;
      templog[0xCC] &= ~(1 << 0);
    }

    if ((solar_temp - tank1_lower) >= 10.0)
    {
      solar_pump_on = true;
      templog[0xCA] = solar_temp;
      templog[0xCB] = solar_temp * 100 - (uint8_t)solar_temp * 100;
      templog[0xCC] |= (1 << 0);
    }

    if (wood_burner_smoke <= 100)
    {
      laddomat_on = false;
      templog[0xCC] &= ~(1 << 1);
    }

    if (wood_burner_smoke > 100 & wood_burner_out > 70.0)
    {
      laddomat_on = true;
      templog[0xCC] |= (1 << 1);
    }

    if (check_dhw != dhw_ctc)
      set_ctc_temp(check_dhw);

    if (!first_run)
      if (checkforchange(0xCA, 0xCC))
        sample_send |= 256;

    if (laddomat_on)
      PORTD |= (1 << PORTD6);     // Activate the laddomat relay
    else
      PORTD &= ~(1 << PORTD6);    // De-Activate the laddomat relay

    if (solar_pump_on)
      PORTB |= (1 << PORTB1);     // Activate the solarpump relay
    else
      PORTB &= ~(1 << PORTB1);    // De-Activate the solarpump relay

    // Reset the watchdog timer
    wdt_reset();

    // Start the ADC oversampling
    ADCSRA |= (1 << ADSC) | (1 << ADATE) | (1 << ADIE);

    if (sample_send != 0)
    {
      new_sample = true;
      PORTD |= (1 << PORTD7);     // Set the Interrupt signal HIGH
    }
  }
} // end of loop
