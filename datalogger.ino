/**
    I2C interface to SPI for CTC Ecologic EXT
    ver 2.0.2
**/

#define VER_MAJOR 2
#define VER_MINOR 0
#define VER_BUILD 2

#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <EEPROM.h>

// Defs for making a median search routine
#define MED_SORT(a,b) { if ((a)>(b)) MED_SWAP((a),(b)); }
#define MED_SWAP(a,b) { float tmp=(a);(a)=(b);(b)=tmp; }

#define NUM_MEDIAN 0    // Number of medians, if any

// Number of connected OneWire sensors
#define NUM_SENSORS 11

// Basic array sizes, just need to add the number of connected OneWire sensors
#define ARRAY_SIZE 0xD0
#define SAMPLE_SIZE 0xAD

// TWI buffer length
#define TWI_BUFFER_LENGTH 6

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define DEVICE_ERROR -127

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

typedef uint8_t DeviceAddress[8];

// Array to hold the adress of the connected devices
const DeviceAddress tempsensor[] PROGMEM = {0x28, 0x2E, 0xE8, 0x1D, 0x07, 0x00, 0x00, 0x80,    // Sensor0  =  Tank 1 Nedre
                                            0x28, 0x67, 0x3A, 0x1E, 0x07, 0x00, 0x00, 0x36,    // Sensor1  =  Tank 1 Övre
                                            0x28, 0x6F, 0xD4, 0x28, 0x07, 0x00, 0x00, 0xAE,    // Sensor2  =  Tank 1 Element
                                            0x28, 0xAC, 0x32, 0x1D, 0x07, 0x00, 0x00, 0xE7,    // Sensor3  =  Tank 1 Varmvatten
                                            0x28, 0x0D, 0x73, 0x1D, 0x07, 0x00, 0x00, 0xA8,    // Sensor4  =  Tank 2 Nedre
                                            0x28, 0x41, 0x2B, 0x29, 0x07, 0x00, 0x00, 0x4D,    // Sensor5  =  Tank 2 Övre
                                            0x28, 0x2E, 0x68, 0x1D, 0x07, 0x00, 0x00, 0x4B,    // Sensor6  =  Panna IN
                                            0x28, 0x20, 0x40, 0x1D, 0x07, 0x00, 0x00, 0x9E,    // Sensor7  =  Panna UT
                                            0x28, 0xB7, 0x3B, 0x1D, 0x07, 0x00, 0x00, 0xB7,    // Sensor8  =  Tidigare Sensor5 (Trasig)
                                            0x28, 0x99, 0x6D, 0x1C, 0x07, 0x00, 0x00, 0x94,    // Sensor9  =  Ej inkopplad
                                            0x3B, 0xFE, 0x20, 0x18, 0x00, 0x00, 0x00, 0x1F     // Sensor10 =  Rökgastemperatur  (Thermocouple type K, via MAX31850K)
                                           };
// Array to hold error status and error counter, 255 = Disconnected sensor
uint8_t sensor_status[] = {0,       // Sensor0
                           0,       // Sensor1
                           0,       // Sensor2
                           0,       // Sensor3
                           0,       // Sensor4
                           0,       // Sensor5
                           0,       // Sensor6
                           0,       // Sensor7
                           255,     // Sensor8
                           255,     // Sensor9
                           0        // Sensor10
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
const uint8_t PROGMEM temp[] = {16,   33,  52,  77,  94,
                                109, 124, 145, 165, 178, 196, 213, 224, 239, 249,
                                5,    17,  26,  35,  43,  55,  62,  71,  77,  84,
                                91,   97, 107, 113, 119, 125, 130, 136, 141, 147,
                                154, 159, 164, 169, 173, 178, 182, 186, 189, 193,
                                196, 199, 203, 206, 208, 212, 216, 219, 221, 222,
                                226, 228, 229, 231, 233, 236, 237, 240, 242, 244,
                                245, 247, 248, 250, 251, 251, 252, 254
                               };


// Variable to hold actual temp of hot water
static volatile uint8_t dhw_ctc = 75;
static uint8_t check_dhw = 75;

// Variables for handling sampling from OneWire sensors
//static float medtmp[NUM_SENSORS][NUM_MEDIAN];
static float owtemp[NUM_SENSORS][4];
static float tempfiltered[NUM_SENSORS][4];
static uint32_t lastTempRequest = 0;
static uint32_t lastCheck = 0;
const uint16_t delayInMillis = 750;
const uint16_t checkDelay = 1000;

// Variables for handling reading of the ADC
//const float InternalReferenceVoltage = 1.100;  // Actually not measured... yet
const float ReferenceResistor = 1270.0;        // Actually not measured... yet
//static volatile float AnalogReferenceVoltage = 3.300;   // Ideally it should be this... but we measure it later
static volatile uint16_t rawADC = 0;
static volatile uint32_t adjusted_ADC = 0;
static volatile uint32_t oversampled_ADC = 0;
const uint8_t nr_extra_bits = 2;                           // Number of additional bits over 10
const uint16_t nr_oversamples = 1 << (nr_extra_bits * 2);  // Number of oversamples needed
const uint32_t adc_divisor = 1024UL << nr_extra_bits;      // Divisor to be used
const float lsb_adjust = (1 + nr_extra_bits) * 0.5;        // Adjustment since we cant reach max value
static volatile uint16_t sample_counter = 0;
// static volatile float voltage_in = 0.0;
static volatile float solar_temp = 0.0;
static float solar_on_temp = 10.0;
static float solar_off_temp = 4.0;
static volatile boolean adcDone = false;
static float tank1_lower = 0.0;
static float tank1_upper = 0.0;
static float wood_burner_in = 0.0;
static float wood_burner_out = 0.0;
static int16_t wood_burner_smoke = 0;
static float adctemp[4];
static float adcfiltered[4];
static uint32_t adc_lastCheck = 0;
const uint16_t adc_checkDelay = 2000;

// Variables to control relays
static volatile boolean solar_pump_on = false;
static volatile boolean solar_pump_force_on = false;
static volatile boolean solar_pump_force_off = false;
static volatile boolean laddomat_on = false;

// Variables for handling self-circulation in solar panels
static volatile boolean anti_circulation = false;
static uint32_t anti_circ_timer = 0;
const uint16_t anti_circ_delay = 10000;

// Buffer and variables for SPI -> TWI command transfer
static volatile uint8_t spi_cmd[2] = { 0xDE, 0x64 };
static volatile uint8_t save_twi_tx;
static volatile boolean command_pending = true;

// Initialize the storage arrays
static volatile uint8_t datalog[ARRAY_SIZE];
static volatile uint8_t templog[SAMPLE_SIZE];

// Error condition variables
static volatile uint8_t twi_error_code = 0;
static volatile uint32_t twi_sample_counter = 0;
static volatile uint32_t twi_bus_error_counter = 0;
static volatile uint16_t sensor_error_code = 0;
static volatile uint32_t ow_sample_counter = 0;
static volatile uint8_t adc_error_code = 0;

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
//static boolean first_run = true;

// TWI vars
static volatile uint8_t twi_txBuffer[2];
static volatile uint8_t twi_txBufferIndex = 0;
static volatile uint8_t twi_txBufferLength = 0;

static volatile uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_rxBufferIndex = 0;

// Watchdog reset check variable
static volatile uint8_t mcu_reset = 0;

// Temp counter
static volatile uint8_t xx = 0;

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
        twi_error_code |= 2;   // Debug
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
      }
      break;

    case 0x88:          // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
    case 0x98:          // Previously addressed with general call; data has been received; NOT ACK has been returned
      twi_error_code |= 4;   // Debug
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
      twi_error_code |= 32;      // Debug
      break;

    case 0x00:          // Bus Error due to an illegal START or STOP condition
      twi_bus_error_counter++;                        // Debug
      twi_error_code |= 64;                    // Debug
      // Release bus and reset TWI hardware
      TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWSTO);
      break;

    default:
      twi_error_code |= 128;                   // Debug
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
      else if (!new_sample && sample_send == 0)
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
        datalog[0xCE] &= ~240;

        // Send ow_sample_counter
        convert.nr_32 = ow_sample_counter;
        SPDR = convert.nr_8[0];          // Load with number of ow_samples we've collected since last time, MSByte
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        SPDR = convert.nr_8[1];          // Load with number of ow_samples we've collected since last time, LSByte
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        SPDR = convert.nr_8[2];          // Load with number of ow_samples we've collected since last time, LSByte
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
        SPDR = convert.nr_8[3];          // Load with number of ow_samples we've collected since last time, LSByte
        while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master

        // Send twi_sample_counter
        convert.nr_32 = twi_sample_counter;
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

        // Send twi_error_code
        SPDR = twi_error_code;                   // Bitflags of TWI error conditions
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
        // If twi_error_code contains error bit6 then Send twi_bus_error_counter
        if (twi_error_code & 64)
        {
          convert.nr_32 = twi_bus_error_counter;          // Number of TWI bus errors due to illegal START or STOP condition
          SPDR = convert.nr_8[0];
          while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
          SPDR = convert.nr_8[1];
          while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
          SPDR = convert.nr_8[2];
          while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
          SPDR = convert.nr_8[3];
          while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
        }
        twi_error_code = 0;

        // Send sensor_error_code
        convert.nr_16 = sensor_error_code;
        SPDR = convert.nr_8[0];
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
        SPDR = convert.nr_8[1];
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
        sensor_error_code = 0;

        // Send adc_error_code
        SPDR = adc_error_code;
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
        adc_error_code = 0;

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

    case 0xF3:                                // Start command sequence to send cmd to digipot
      static volatile uint8_t digi_cmd[2] = {0, 0};           // Command buffer for DigiPot
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
      SPDR = mcu_reset;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.number = solar_on_temp;
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[2];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[3];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.number = solar_off_temp;
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[2];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[3];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xD4;                    // Access SPDR to clear SPIF
      break;

    case 0xF5:                        // Fetch ALL debug variables
      SPDR = 0xD5;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

      // Send TWI_bus_error_counter
      convert.nr_32 = twi_bus_error_counter;	// Number of TWI bus errors due to illegal START or STOP condition
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[2];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[3];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

      // Send TWI_error_code
      SPDR = twi_error_code;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

      // Send sensor_error_code
      convert.nr_16 = sensor_error_code;
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

      // Send count
      SPDR = count;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

      // Send TWI buffers
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
      twi_rxBuffer[2] = 0;
      twi_rxBuffer[3] = 0;

      // Send some boolean variables
      SPDR = start_sampling;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = twi_sample_done;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = ok_sample_ow;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xD5;                    // Access SPDR to clear SPIF
      break;

    case 0xF6:                        // Release bus and reset TWI hardware
      SPDR = 0xD6;
      TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWSTO);
      start_sampling = true;
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

    case 0xA6:                        // Read sensor_value
      SPDR = 0xB8;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.number = tempfiltered[SPDR][3];
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[2];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[3];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xB8;                    // Access SPDR to clear SPIF
      break;

    case 0xA7:
      xx = 0;
      SPDR = sensor_status[xx];
      while (xx++ < 11)
      {
        while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master   
        SPDR = sensor_status[xx];
      }
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

    case 0xFA:                        // Toggle the state of solar_pump_force_on
      solar_pump_force_on = !solar_pump_force_on;
      if (solar_pump_force_off && solar_pump_force_on)  // Not both true at the same time
        solar_pump_force_on = false;
      SPDR = solar_pump_force_on;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xDA;                    // Access SPDR to clear SPIF
      break;

    case 0xFB:                        // Toggle the state of solar_pump_force_off
      solar_pump_force_off = !solar_pump_force_off;
      if (solar_pump_force_on && solar_pump_force_off)  // Not both true at the same time
        solar_pump_force_off = false;
      SPDR = solar_pump_force_off;
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

    case 0xFE:
      anti_circulation = true;
      SPDR = anti_circulation;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xDE;                    // Access SPDR to clear SPIF
      break;

    case 0xA0:
      convert.nr_32 = twi_bus_error_counter;          // Number of TWI bus errors due to illegal START or STOP condition
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
      // Send ow_sample_counter
      convert.nr_32 = ow_sample_counter;
      SPDR = convert.nr_8[0];          // Load with number of ow_samples we've collected since last time, MSByte
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[1];          // Load with number of ow_samples we've collected since last time, LSByte
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[2];          // Load with number of ow_samples we've collected since last time, LSByte
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[3];          // Load with number of ow_samples we've collected since last time, LSByte
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master

      // Send twi_sample_counter
      convert.nr_32 = twi_sample_counter;
      SPDR = convert.nr_8[0];          // Load with number of twi_samples we've collected since last time
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[1];          // Load with number of twi_samples we've collected since last time
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[2];          // Load with number of twi_samples we've collected since last time
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master
      SPDR = convert.nr_8[3];          // Load with number of twi_samples we've collected since last time
      while (!(SPSR & (1 << SPIF)));   // Wait for next byte from Master

      // Send twi_error_code
      SPDR = twi_error_code;                   // Bitflags of TWI error conditions
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = 0xB1;                    // Access SPDR to clear SPIF
      break;

    case 0xA2:
      twi_min_time = 3000;
      twi_max_time = 2000;
      twi_accum_time = 0;
      SPDR = 0xB2;
      break;

    case 0xA3:                        // Program solar_on_temp
      SPDR = 0xB3;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[0] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[1] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[2] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[3] = SPDR;
      solar_on_temp = convert.number;
      EEPROM.put(256, solar_on_temp);
      break;

    case 0xA4:                        // Program solar_off_temp
      SPDR = 0xB4;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[0] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[1] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[2] = SPDR;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      convert.nr_8[3] = SPDR;
      solar_off_temp = convert.number;
      EEPROM.put((256 + sizeof(float)), solar_off_temp);
      break;

    case 0xA5:                        // Fetch ALL debug variables
      SPDR = 0xB5;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

      // Send adjusted_ADC
      convert.nr_32 = adjusted_ADC;// The oversampled value of ADC
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[2];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[3];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

      // Send system_status
      SPDR = datalog[0xCE];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

      // Send solar_temp
      convert.number = solar_temp;     // The actual solar_temp
      SPDR = convert.nr_8[0];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[1];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[2];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master
      SPDR = convert.nr_8[3];
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

      // Send solar_pump_on
      SPDR = solar_pump_on;
      while (!(SPSR & (1 << SPIF)));  // Wait for next byte from Master

      SPDR = 0xD5;                    // Access SPDR to clear SPIF
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

/*
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
*/

float getTemp(const DeviceAddress deviceaddress)
{
  oneWire.reset();
  oneWire.select(deviceaddress);
  oneWire.write(0xBE);
  uint8_t scratch[9];
  for (uint8_t i = 0; i < 9; i++)
    scratch[i] = oneWire.read();
  if (oneWire.crc8(scratch, 8) == scratch[8])
  {
    int16_t raw = (int16_t)(scratch[1] << 8) | (int16_t)scratch[0];
    switch (deviceaddress[0])
    {
      case 0x28:
        if (raw >= -880 && raw <= 2000)		// Equal to -55 and +125 temperature
          return (float)raw * 0.0625;
        break;
      case 0x3B:
        if (!(scratch[0] & 1))
        {
          if (raw >= -4000 && raw <= 25600)	// Equal to -250 and +1600 temperature
            return (float)raw * 0.0625;
        }
        else
        {
          sensor_error_code |= ((scratch[2] & 7) << 11);
        }
        break;
    }
  }
  return DEVICE_ERROR;
}

void requestTemp()
{
  oneWire.reset();
  oneWire.skip();
  oneWire.write(0x44);
}

void setup()
{
  // Signal that we've started
  sensor_error_code |= (1 << 15);

  // Check the reset cause
  mcu_reset = MCUSR;
  datalog[0xCE] = (mcu_reset << 4);
  sample_send |= 256;
  MCUSR = 0;

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

  // Request a temperature conversion from OneWire sensors
  requestTemp();

  // Initialize ADC
  // Clear ADMUX and set Vref to AVcc, ADC0 selected as source
  ADMUX = (1 << REFS0);

  // ADC enable, start first conversion, prescaler CLK/32 = 500kHz
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS0);

  // Initialize all the interfaces
  // TWI slave interface to CTC EcoLogic EXT
  // Set the slave address
  TWAR = 0x5C << 1;

  // Enable TWI module, acks and interrupt
  TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA);

  // SPI slave interface to Raspberry Pi
  // Set Port B4 output, SPI MISO
  DDRB |= (1 << DDB4);

  // Interrupt enabled, SPI enabled, MSB first, Slave, CLK high when idle, Sample on leading edge of CLK (SPI Mode 0)
//  SPCR = (1 << SPIE) | (1 << SPE) | (1 << CPOL);
  SPCR = (1 << SPIE) | (1 << SPE);

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

  // Command DigiPot RESET (command number 14)
  xfer(176, 0);

  // Initialize sensor calibration values from EEPROM
  for (uint8_t x = 0; x < NUM_SENSORS; x++)
  {
    EEPROM.get((x * sizeof(float)), sensor_calibration[x]);
    if ((sensor_calibration[x] < -2.0) || (sensor_calibration[x] > 2.0))	// Sanity check in case the EEPROM has been corrupted or is not programmed
    {
      sensor_calibration[x] = 0;
      EEPROM.put((x * sizeof(float)), sensor_calibration[x]);
    }
  }

  // Initialize Solar Panel On/Off - values
  float stored_temp = 0.0;					// Temporary variable
  EEPROM.get(256, stored_temp);
  if ((stored_temp > 0.0) && (stored_temp > solar_off_temp))	// Sanity check in case the EEPROM has been corrupted or is not programmed
    solar_on_temp = stored_temp;
  else
    EEPROM.put(256, solar_on_temp);
  stored_temp = 0.0;
  EEPROM.get((256 + sizeof(float)), stored_temp);
  if ((stored_temp > 0.0) && (stored_temp < solar_on_temp))	// Sanity check in case the EEPROM has been corrupted or is not programmed
    solar_off_temp = stored_temp;
  else
    EEPROM.put((256 + sizeof(float)), solar_off_temp);

  // Wait for the ADC to finish the first conversion
  while ((ADCSRA & (1 << ADSC)));
  float solar_resistor = ReferenceResistor / ((1024.0 / (ADC + 0.5)) - 1.0);	// Only one sample used
  float solar_raw = (solar_resistor - 1000.0) / 3.75;

  // Input for filter
  adctemp[0] = solar_raw;
  adctemp[1] = solar_raw;
  adctemp[2] = solar_raw;
  adctemp[3] = solar_raw;

  // Butterworth filter with cutoff frequency 0.01*sample frequency (FS=1.33Hz)
  adcfiltered[0] = solar_raw;
  adcfiltered[1] = solar_raw;
  adcfiltered[2] = solar_raw;

  adcfiltered[3] = (adctemp[0] + adctemp[3] + 3 * (adctemp[1] + adctemp[2])) / 3.430944333e+04 + (0.8818931306 * adcfiltered[0]) + (-2.7564831952 * adcfiltered[1]) + (2.8743568927 * adcfiltered[2]);

  solar_temp = lrintf(adcfiltered[3] * 10.0) * 0.1;     // Round to one decimal

  convert.number = solar_temp;
  datalog[0xCA] = convert.nr_8[0];
  datalog[0xCB] = convert.nr_8[1];
  datalog[0xCC] = convert.nr_8[2];
  datalog[0xCD] = convert.nr_8[3];
  sample_send |= 256;

  // Wait for sensors to finish temperature conversion
//  while (!oneWire.read_bit());

  // Initialize OneWire sensors
  DeviceAddress devicetemp;
  float temperature = 0.0;
  for (uint8_t x = 0; x < NUM_SENSORS; x++)
  {
    memcpy_P (&devicetemp, tempsensor[x], sizeof (DeviceAddress));
    temperature = getTemp(devicetemp);
    if (temperature != DEVICE_ERROR)
    {
      // Apply sensor calibration
      temperature = temperature + sensor_calibration[x];

      // Init the filter
      owtemp[x][0] = temperature;
      owtemp[x][1] = temperature;
      owtemp[x][2] = temperature;
      owtemp[x][3] = temperature;
      tempfiltered[x][0] = temperature;
      tempfiltered[x][1] = temperature;
      tempfiltered[x][2] = temperature;

      tempfiltered[x][3] = (owtemp[x][0] + owtemp[x][3] + 3 * (owtemp[x][1] + owtemp[x][2])) / 3.430944333e+04 + (0.8818931306 * tempfiltered[x][0]) + (-2.7564831952 * tempfiltered[x][1]) + (2.8743568927 * tempfiltered[x][2]);

      if (x != 10)
      {
        temperature = lrintf(tempfiltered[x][3] * 10.0) * 0.1;
        convert.nr_8[0] = temperature;						// Use this global var to save some room
        convert.nr_8[1] = temperature * 100 - (uint8_t)temperature * 100;	// Use this global var to save some room
      }
      else
      {
        convert.nr_16 = (int16_t)lrintf(tempfiltered[x][3]);
      }
      datalog[0xB0 + x * 2] = convert.nr_8[0];
      datalog[0xB1 + x * 2] = convert.nr_8[1];
      sample_send |= 128;
    }
    else
    // Result was DEVICE_ERROR
    {
      if (sensor_status[x] != 255)        // Is the sensor already disconnected?
      {
        sensor_status[x] == 1;         // Since this is setup, this is the first error
      }
    }
  }
  requestTemp();

  tank1_lower = lrintf(tempfiltered[0][3] * 10.0) * 0.1;
  tank1_upper = lrintf(tempfiltered[1][3] * 10.0) * 0.1;
  check_dhw = lrintf(tempfiltered[3][3]);
  wood_burner_in = lrintf(tempfiltered[6][3] * 10.0) * 0.1;
  wood_burner_out = lrintf(tempfiltered[7][3] * 10.0) * 0.1;
  wood_burner_smoke = (int16_t)lrintf(tempfiltered[10][3]);

  // Initialize the timekeeping vars
  lastTempRequest = millis();
  adc_lastCheck = lastTempRequest;
  lastCheck = lastTempRequest;
  twi_lastCheck = lastTempRequest;

  // Wait until first TWI sample has been collected
  while (!twi_sample_done);

  // Initialize WDT with 4 seconds timeout
  wdt_enable(WDTO_4S);

  // Get a updated value of AVcc
  //measureAVcc();
} // end of setup

void loop()
{
  uint32_t time_now = millis();

  // Handle self-ciculation in solar panels
  // Check if it is time to shut off anti-circ
  if ((datalog[0xCF] & 256) && ((time_now - anti_circ_timer) >= anti_circ_delay))
  {
    datalog[0xCF] &= ~(1 << 0);
    solar_pump_force_on = false;
    sample_send |= 256;
  }
  // Check if we have been commanded to anti-circ
  if (anti_circulation && !solar_pump_force_off)
  {
    datalog[0xCF] |= (1 << 0);
    anti_circ_timer = millis();
    anti_circulation = false;
    solar_pump_force_on = true;
    sample_send |= 256;
  }
  
  // Start checking the status of the newly taken sample versus the last sent
  if (twi_sample_done)
  {
    // Insert current DHW value into templog
    templog[0x8C] = check_dhw;

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
    twi_sample_counter++;       // Increment the sample counter

//    if (sample_send & 127)
//      first_run = false;

  }

  if (ok_sample_ow && (time_now - lastTempRequest) >= delayInMillis)
  {
    DeviceAddress devicetemp;
    float temperature = 0.0;
    for (uint8_t x = 0; x < NUM_SENSORS; x++)
    {
      memcpy_P (&devicetemp, tempsensor[x], sizeof (DeviceAddress));
      temperature = getTemp(devicetemp);
      if (temperature != DEVICE_ERROR)
      {
        if (sensor_status[x] >= 254)     // Has sensor been connected again?
        {
          if ((temperature == 85.0) && (sensor_status[x] != 254))  // Perhaps this is the first read of scratchpad?
            sensor_status[x] = 254;
          else
          {
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
        }

        // Apply sensor calibration
        temperature = temperature + sensor_calibration[x];

        // Input for filter
        owtemp[x][0] = owtemp[x][1];
        owtemp[x][1] = owtemp[x][2];
        owtemp[x][2] = owtemp[x][3];
        if (x != 10)
        {
          if (temperature < (owtemp[x][3]+10.0) && temperature > (owtemp[x][3]-10.0))
          {
            owtemp[x][3] = temperature;
            sensor_status[x] = 0; // Reset the error counter
          }
          else
            sensor_error_code |= (1 << x) | (1 << 14);
        }
        else
        {
          if (temperature < (owtemp[x][3]+75.0) && temperature > (owtemp[x][3]-75.0))
          {
            owtemp[x][3] = temperature;
            sensor_status[x] = 0; // Reset the error counter
          }
          else
            sensor_error_code |= (1 << x) | (1 << 14);
        }

        // Butterworth filter with cutoff frequency 0.01*sample frequency (FS=1.33Hz)
        tempfiltered[x][0] = tempfiltered[x][1];
        tempfiltered[x][1] = tempfiltered[x][2];
        tempfiltered[x][2] = tempfiltered[x][3];

        tempfiltered[x][3] = (owtemp[x][0] + owtemp[x][3] + 3 * (owtemp[x][1] + owtemp[x][2])) / 3.430944333e+04 + (0.8818931306 * tempfiltered[x][0]) + (-2.7564831952 * tempfiltered[x][1]) + (2.8743568927 * tempfiltered[x][2]);

        // Check if we have a overflow condition
        if (x != 10)
        {
          if (tempfiltered[x][3] < -55.0 || tempfiltered[x][3] > 125.0)         // Sanity check
          {
            tempfiltered[x][3] = temperature;          // Somewhat hacky
            datalog[0xCF] |= (1 << 1);
            sample_send |= 256;
          }
          else
          {
            if ((datalog[0xCF] & 2))
            {
              datalog[0xCF] &= ~(1 << 1);
              sample_send |= 256;
            }
          }
          temperature = lrintf(tempfiltered[x][3] * 10.0) * 0.1;
          convert.nr_8[0] = temperature;					// Use this global var to save some room
          convert.nr_8[1] = temperature * 100 - (uint8_t)temperature * 100;	// Use this global var to save some room
        }
        else
        {
          if (tempfiltered[x][3] < -250.0 || tempfiltered[x][3] > 1600.0)         // Sanity check
          {
            tempfiltered[x][3] = temperature;          // Somewhat hacky
            datalog[0xCF] |= (1 << 1);
            sample_send |= 256;
          }
          else
          {
            if ((datalog[0xCF] & 2))
            {
              datalog[0xCF] &= ~(1 << 1);
              sample_send |= 256;
            }
          }
          convert.nr_16 = (int16_t)lrintf(tempfiltered[x][3]);
        }
        // See if there has been a change since last sample?
        if ((datalog[0xB0 + x * 2] != convert.nr_8[0]) || (datalog[0xB1 + x * 2] != convert.nr_8[1]))
        {
          datalog[0xB0 + x * 2] = convert.nr_8[0];
          datalog[0xB1 + x * 2] = convert.nr_8[1];
          sample_send |= 128;
        }
      }
      else
      // Result was DEVICE_ERROR
      {
        if (sensor_status[x] != 255)        // Is the sensor already disconnected?
        {
          sensor_status[x]++;               // Increase error counter
          if (sensor_status[x] >= 2)        // 2 or more errors in a row?
          {
            sensor_error_code |= (1 << x);
            if (sensor_status[x] >= 10)     // 10 errors in a row?
            {
              sensor_status[x] = 255;       // Regard sensor as disconnected
              // Clear the filter
              owtemp[x][0] = 0;
              owtemp[x][1] = 0;
              owtemp[x][2] = 0;
              owtemp[x][3] = 0;
              tempfiltered[x][0] = 0;
              tempfiltered[x][1] = 0;
              tempfiltered[x][2] = 0;
              tempfiltered[x][3] = 0;
              // Clear the datalog
              datalog[0xB0 + x * 2] = 0;
              datalog[0xB1 + x * 2] = 0;
            }
          }
        }
      }
    }

    requestTemp();
    lastTempRequest = millis();
    ow_sample_counter++;
    ok_sample_ow = false;

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
    // voltage_in = AnalogReferenceVoltage * (adjusted_ADC + lsb_adjust) / adc_divisor;
    float solar_resistor = ReferenceResistor / ((adc_divisor / (adjusted_ADC + lsb_adjust)) - 1.0);
    float solar_raw = (solar_resistor - 1000.0) / 3.75;

    if (solar_raw >= -55.0 && solar_raw <= 200.0)
    {
      // Input for filter
      adctemp[0] = adctemp[1];
      adctemp[1] = adctemp[2];
      adctemp[2] = adctemp[3];
      if (solar_raw < (adctemp[3]+10.0) && solar_raw > (adctemp[3]-10.0))
        adctemp[3] = solar_raw;
      else
        adc_error_code |= (1 << 0) | (1 << 1);

      // Butterworth filter with cutoff frequency 0.01*sample frequency (FS=1.33Hz)
      adcfiltered[0] = adcfiltered[1];
      adcfiltered[1] = adcfiltered[2];
      adcfiltered[2] = adcfiltered[3];

      adcfiltered[3] = (adctemp[0] + adctemp[3] + 3 * (adctemp[1] + adctemp[2])) / 3.430944333e+04 + (0.8818931306 * adcfiltered[0]) + (-2.7564831952 * adcfiltered[1]) + (2.8743568927 * adcfiltered[2]);

      if (adcfiltered[3] < -55.0 || adcfiltered[3] > 200.0)         // Sanity check
      {
        adcfiltered[3] = solar_raw;          // Somewhat hacky
        datalog[0xCF] |= (1 << 2);
        sample_send |= 256;
      }
      else
      {
        if ((datalog[0xCF] & 4))
        {
          datalog[0xCF] &= ~(1 << 2);
          sample_send |= 256;
        }
      }
    }
    else
    {
      if (solar_raw > 200.0)	// Could indicate an open circuit
        adc_error_code |= (1 << 0) | (1 << 2);
      if (solar_raw < -55.0)    // Could indicate a short circuit
        adc_error_code |= (1 << 0) | (1 << 3);
    }
  }

  if ((time_now - adc_lastCheck) >= adc_checkDelay)
  {
    adc_lastCheck = millis();
    if (adcfiltered[3] >= -55.0 && adcfiltered[3] <= 200.0)         // Sanity check
    {
      convert.number = lrintf(adcfiltered[3] * 10.0) * 0.1;     // Re-use global var and round to one decimal
      if (solar_temp != convert.number)
      {
        solar_temp = convert.number;
        datalog[0xCA] = convert.nr_8[0];
        datalog[0xCB] = convert.nr_8[1];
        datalog[0xCC] = convert.nr_8[2];
        datalog[0xCD] = convert.nr_8[3];
        sample_send |= 256;
      }
    }
  }

  if ((time_now - lastCheck) >= checkDelay)
  {
    lastCheck = millis();

    if ((solar_pump_on && (solar_temp <= (solar_off_temp + tank1_lower)) && !solar_pump_force_on) || solar_pump_force_off)
    {
      solar_pump_on = false;
      convert.number = solar_temp;
      datalog[0xCA] = convert.nr_8[0];
      datalog[0xCB] = convert.nr_8[1];
      datalog[0xCC] = convert.nr_8[2];
      datalog[0xCD] = convert.nr_8[3];
      datalog[0xCE] &= ~(1 << 0);
      sample_send |= 256;
    }

    if ((!solar_pump_on && (solar_temp >= (solar_on_temp + tank1_lower)) && !solar_pump_force_off) || solar_pump_force_on)
    {
      solar_pump_on = true;
      convert.number = solar_temp;
      datalog[0xCA] = convert.nr_8[0];
      datalog[0xCB] = convert.nr_8[1];
      datalog[0xCC] = convert.nr_8[2];
      datalog[0xCD] = convert.nr_8[3];
      datalog[0xCE] |= (1 << 0);
      sample_send |= 256;
    }

    if (laddomat_on && (wood_burner_smoke <= 100))
    {
      laddomat_on = false;
      datalog[0xCE] &= ~(1 << 1);
      sample_send |= 256;
    }

    if (!laddomat_on && ((wood_burner_smoke > 100) && (wood_burner_out > 70.0)))
    {
      laddomat_on = true;
      datalog[0xCE] |= (1 << 1);
      sample_send |= 256;
    }

    if (check_dhw != dhw_ctc && check_dhw > 24 && check_dhw < 100)
      set_ctc_temp(check_dhw);

    if (solar_pump_force_on)
    {
      if (!(datalog[0xCE] & 4))
      {
        datalog[0xCE] |= (1 << 2);
        sample_send |= 256;
      }
    }
    else
    {
      if ((datalog[0xCE] & 4))
      {
        datalog[0xCE] &= ~(1 << 2);
        sample_send |= 256;
      }
    }
    if (solar_pump_force_off)
    {
      if (!(datalog[0xCE] & 8))
      {
        datalog[0xCE] |= (1 << 3);
        sample_send |= 256;
      }
    }
    else
    {
      if ((datalog[0xCE] & 8))
      {
        datalog[0xCE] &= ~(1 << 3);
        sample_send |= 256;
      }
    }
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
      interrupts();               // Should NOT need this, unless something is VERY wrong...
    }
  }
} // end of loop
