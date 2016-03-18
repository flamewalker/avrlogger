/**
 * Copyright (c) 2012 Tgaz
 * Published under the MIT license.
 *
 * Very simple test program to emulate a CTC SMS controller.
 *
 * Accepts commands over serial port:
 *
 *  * No-op/ping is 0xFF, and 0xFF is returned.
 *  * Commands are two bytes, one byte returned as ack (first byte received.)
 *  * Register requests are one byte, two bytes returned (address and value.)
 *
 * See http://vpwiki.org/index.php/EcoLogic_modemprotokoll for more information.
**/
#include <Wire.h>


enum CtcState
{
  CTC_IDLE,
  CTC_REQUEST,
  CTC_COMMAND,
  CTC_RESPONSE,
};

static uint8_t state = CTC_IDLE;

// Command buffer for ISR.
static uint8_t nextCmd[2] = { 0xFF, 0xFF };

// Reply buffer for ISR.
static uint8_t reply = 0xFF;

const int antal = 0xDC;

static uint8_t datalog[antal][2];

int ledState = LOW;

/**
 * The master (display card) sent us data.
**/
static void onWireReceive(int numBytes)
{
  // if the LED is off turn it on and vice-versa:
  if (ledState == LOW)
    ledState = HIGH;
  else
    ledState = LOW;

  // set the LED with the ledState of the variable:
  digitalWrite(LED_BUILTIN, ledState);

  switch (state) {
    case CTC_IDLE:
      // We expect a single byte.
      if (numBytes != 1 || Wire.read() != 0xFE)
        break;

      // Check if this is a command or a register request.
      if (nextCmd[0] > 0xDB)
        state = CTC_COMMAND;
      else
        state = CTC_REQUEST;

      break;

    case CTC_RESPONSE:
      // Expected address and data bytes.
      // First byte should match what we sent.
      if (numBytes != 2 || Wire.read() != nextCmd[0]) {
        reply = 0xFF;
        state = CTC_IDLE;
        nextCmd[0] = 0xFF;
        break;
      }

      reply = Wire.read();
      state = CTC_IDLE;
      nextCmd[0] = 0xFF;
      break;

    default:
      state = CTC_IDLE;
      nextCmd[0] = 0xFF;
  }

  //  digitalWrite(LED_BUILTIN, LOW);
}

/**
 * The master requests data from us.
**/
static void onWireRequest()
{
  // if the LED is off turn it on and vice-versa:
  if (ledState == LOW)
    ledState = HIGH;
  else
    ledState = LOW;

  // set the LED with the ledState of the variable:
  digitalWrite(LED_BUILTIN, ledState);

  switch (state) {
    case CTC_REQUEST:
      Wire.write(nextCmd, 1);
      state = CTC_RESPONSE;
      break;

    case CTC_COMMAND:
      // No-op/ping is a single byte.
      if (nextCmd[0] == 0xFF)
        Wire.write(nextCmd, 1);
      else
        Wire.write(nextCmd, 2);

      state = CTC_IDLE;
      nextCmd[0] = 0xFF;
      break;

    default:
      state = CTC_IDLE;
      nextCmd[0] = 0xFF;
  }

  //  digitalWrite(LED_BUILTIN, LOW);
}

/**
 * Request data from the display card.
 *
 * Blocks until data is returned. This can take up to a minute,
 * since we have to wait for the display card to poll.
**/
static uint8_t SampleRegister()
{
  int x = 0;

  while (x < antal)
  {
    if (state != CTC_IDLE)
      return 0xFF;
    nextCmd[0] = x;
    while (state != CTC_IDLE || nextCmd[0] != 0xFF)
      delay(0);
    datalog[x][0] = x;
    datalog[x++][1] = reply;
  }
  return 0x00;
}

static void PrintResult()
{
  Serial.print("Utetemp: ");
  Serial.println(datalog[0x8E][1] - 40, DEC);
  Serial.print("Rumstemp: ");
  Serial.print(datalog[0x8F][1], DEC);
  Serial.print(".");
  Serial.println(datalog[0x90][1], DEC);
  Serial.print("Framledning: ");
  Serial.print(datalog[0x8D][1], DEC);
  Serial.print("(");
  Serial.print(datalog[0xA9][1], DEC);
  Serial.println(")");
  Serial.print("VP framledning: ");
  Serial.print(datalog[0x9C][1], DEC);
  Serial.print("(");
  Serial.print(datalog[0xAB][1], DEC);
  Serial.println(")");
  Serial.print("VP retur: ");
  Serial.print(datalog[0x91][1], DEC);
  Serial.print("(");
  Serial.println(")");
  Serial.print("Brine in/ut: ");
  Serial.print(datalog[0x95][1] - 40, DEC);
  Serial.print("/");
  Serial.println(datalog[0x94][1] - 40, DEC);
  Serial.print("Hetgas: ");
  Serial.println(datalog[0x96][1], DEC);
  Serial.print("Ack.tank: ");
  Serial.print(datalog[0x92][1], DEC);
  Serial.print("(");
  Serial.print(datalog[0xff][1], DEC);
  Serial.println(")");
  Serial.println();
}

/**
 * Request data from the display card.
 *
 * Blocks until command sent. This can take up to a minute,
 * since we have to wait for the display card to poll.
**/
static void ctcSendCommand(uint8_t cmd, uint8_t arg)
{
  if (state != CTC_IDLE)
    return;

  nextCmd[0] = cmd;
  nextCmd[1] = arg;

  while (state != CTC_IDLE || nextCmd[0] != 0xFF)
    delay(0);
}

void setup()
{
  Serial.begin(230400);
  Wire.begin(0x5C);
  Wire.onReceive(onWireReceive);
  Wire.onRequest(onWireRequest);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Datalogger");
}

void loop()
{
  /*
  SampleRegister();
  PrintResult();
  */
}

void serialEvent()
{
  uint8_t cmd = Serial.read();

  if (cmd == 0xFF) {
    Serial.write(cmd);
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;
    // set the LED with the ledState of the variable:
    digitalWrite(LED_BUILTIN, ledState);

  } else if (cmd > 0xDB) {
    while (!Serial.available())
      continue;

    ctcSendCommand(cmd, Serial.read());

    Serial.write(cmd);
  } else {
    SampleRegister();
        for (int y = 0; y < antal; y++) {
          Serial.print(datalog[y][0], HEX);
          Serial.print("=");
          Serial.println(datalog[y][1], DEC);
        }
  }
}
