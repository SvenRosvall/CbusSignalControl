// Modified by Sven Rosvall (M3777)

/*
  Copyright (C) Sven Rosvall 2021 (sven@rosvall.ie)
  
  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

/*
      3rd party libraries needed for compilation: (not for binary-only distributions)

      SignalControl -- library to control signals
      CBUS        -- Library to manage CBUS events on the bus.
      CBUS2515    -- Library to support CBUS communication over CAN bus using a MCP2515/25625 controller.
      ACAN2515    -- library to support the MCP2515/25625 CAN controller IC
      Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
*/
///////////////////////////////////////////////////////////////////////////////////
// Pin Use map UNO:
// Digital pin 2 ()       Interupt CAN
// Digital pin 3 (PWM)    Signal RED
// Digital pin 4 ()       Not Used
// Digital pin 5 (PWM)    Signal YELLOW
// Digital pin 6 (PWM)    Signal GREEN
// Digital pin 7 ()       Not Used
// Digital pin 8 ()       Not Used
// Digital pin 9 (PWM)    Signal 2nd YELLOW
// Digital pin 10 (PWM)   CS   CAN
// Digital pin 11 (PWM,MOSI) SI CAN   ICSP-4
// Digital pin 12 (MISO)  SO    CAN   ICSP-1
// Digital pin 13 (SCK)   Sck   CAN   ICSP-3

// Digital / Analog pin 0     Not Used
// Digital / Analog pin 1     Not Used
// Digital / Analog pin 2     Not Used
// Digital / Analog pin 3     Yellow FLiM LED
// Digital / Analog pin 4     Green SLiM LED
// Digital / Analog pin 5     CBUS Switch
//////////////////////////////////////////////////////////////////////////


// 3rd party libraries
#include <Streaming.h>

// CBUS library header files
#include <CBUS2515.h>               // CAN controller and CBUS class
#include <CBUSswitch.h>             // pushbutton switch
#include <CBUSLED.h>                // CBUS LEDs
#include <CBUSconfig.h>             // module configuration
#include <CBUSParams.h>             // CBUS parameters
#include <cbusdefs.h>               // MERG CBUS constants

// SignalControl header files
#include <Signal4Aspect.h>
#include <DistanceWithCondition.h>
#include <BlockDistanceInput.h>
#include <SlowLight.h>
#include <SettableInput.h>

#include <ProcessSerialInput.h>

// module name, max 7 characters.
unsigned char mname[] = "ASIGBLK";

// constants
const byte VER_MAJ = 0;                  // code major version
const char VER_MIN = '0';                // code minor version
const byte VER_BETA = 1;                 // code beta sub-version
const byte MODULE_ID = 204;              // CBUS module type

const unsigned long CAN_OSC_FREQ = 8000000;     // Oscillator frequency on the CAN2515 board

const byte LED_GRN = A4;                  // CBUS green SLiM LED pin
const byte LED_YLW = A3;                  // CBUS yellow FLiM LED pin
const byte SWITCH0 = A5;                  // CBUS push button switch pin

// CBUS pins
const byte CAN_INT_PIN = 2;  // Only pin 2 and 3 support interrupts
const byte CAN_CS_PIN = 10;
const byte CAN_SI_PIN = 11;  // Cannot be changed
const byte CAN_SO_PIN = 12;  // Cannot be changed
const byte CAN_SCK_PIN = 13;  // Cannot be changed

// CBUS objects
CBUS2515 cbus;                      // CBUS object
CBUSConfig config;                  // configuration object
CBUSLED ledGrn, ledYlw;             // LED objects
CBUSSwitch pb_switch;               // switch object

// CANASIGNAL objects
const byte SIGNAL_RED_PIN = 3;
const byte SIGNAL_YELLOW_PIN = 5;
const byte SIGNAL_GREEN_PIN = 6;
const byte SIGNAL_YELLOW_2_PIN = 9;

// EV 1 - Control signal. OFF -> start timer.
const int NUM_EVS = 1;

// forward function declarations
void eventhandler(byte index, byte opc);

//
/// setup CBUS - runs once at power on from setup()
//
void setupCBUS()
{
  // set config layout parameters
  config.EE_NVS_START = 10;
  config.EE_NUM_NVS = 10;
  config.EE_EVENTS_START = 50;
  config.EE_MAX_EVENTS = 64;
  config.EE_NUM_EVS = NUM_EVS;
  config.EE_BYTES_PER_EVENT = (config.EE_NUM_EVS + 4);

  // initialise and load configuration
  config.setEEPROMtype(EEPROM_INTERNAL);
  config.begin();

  Serial << F("> mode = ") << ((config.FLiM) ? "FLiM" : "SLiM") << F(", CANID = ") << config.CANID;
  Serial << F(", NN = ") << config.nodeNum << endl;

  // show code version and copyright notice
  printConfig();

  // set module parameters
  CBUSParams params(config);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setModuleId(MODULE_ID);
  params.setFlags(PF_FLiM | PF_COMBI);

  // assign to CBUS
  cbus.setParams(params.getParams());
  cbus.setName(mname);

  // initialise CBUS switch and assign to CBUS
  pb_switch.setPin(SWITCH0, LOW);
  pb_switch.run();
  cbus.setSwitch(pb_switch);

  // module reset - if switch is depressed at startup and module is in SLiM mode
  if (pb_switch.isPressed() && !config.FLiM) {
    Serial << F("> switch was pressed at startup in SLiM mode") << endl;
    config.resetModule(ledGrn, ledYlw, pb_switch);
  }

  // set CBUS LED pins and assign to CBUS
  ledGrn.setPin(LED_GRN);
  ledYlw.setPin(LED_YLW);
  cbus.setLEDs(ledGrn, ledYlw);

  // register our CBUS event handler, to receive event messages of learned events
  cbus.setEventHandler(eventhandler);

  // set CBUS LEDs to indicate mode
  cbus.indicateMode(config.FLiM);

  // configure and start CAN bus and CBUS message processing
  cbus.setNumBuffers(2);         // more buffers = more memory used, fewer = less
  cbus.setOscFreq(CAN_OSC_FREQ);   // select the crystal frequency of the CAN module
  cbus.setPins(CAN_CS_PIN, CAN_INT_PIN); // select pins for CAN bus CE and interrupt connections
  cbus.begin();
}

SettableInput block1;
SettableInput block2;
SettableInput block3;
BlockDistanceInput blockDistanceInput(block1, block2);
SettableInput pointInput;
DistanceWithCondition distanceWithCondition(blockDistanceInput, pointInput);
SlowLight greenLight(SIGNAL_GREEN_PIN);
SlowLight redLight(SIGNAL_RED_PIN);
SlowLight yellowLight1(SIGNAL_YELLOW_PIN);
SlowLight yellowLight2(SIGNAL_YELLOW_2_PIN);
Signal4Aspect signal(distanceWithCondition, greenLight, redLight, yellowLight1, yellowLight2);

void setupASignal()
{
  // Signal object is initialized by constructors.
}

void setup()
{
  Serial.begin (115200);
  Serial << endl << endl << F("> ** CBUS Arduino Signal - Timer triggered by event ** ") << __FILE__ << endl;

  setupCBUS();
  setupASignal();

  Serial << F("> ready") << endl << endl;
}

void loop()
{
  // do CBUS message, switch and LED processing
  cbus.process();

  // process console commands
  processSerialInput(cbus, config, printConfig);

  blockDistanceInput.update();
  signal.update();

  // Serial << "Time: " << millis() << endl;
}

//
/// user-defined event processing function
/// called from the CBUS library when a learned event is received
/// it receives the event table index and the CAN frame
//
void eventhandler(byte index, CANFrame *msg)
{
  // as an example, control an LED

  Serial << F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]) << endl;

  // read the value of the first event variable (EV) associated with this learned event
  byte ev1val = config.getEventEVval(index, 1);
  Serial << F("> EV1 = ") << ev1val << endl;

  switch (msg->data[0]) 
  {
  case OPC_ACON:
  case OPC_ACOF:
    switch (ev1val)
    {
      case 1:
        block1.set(msg->data[0] == OPC_ACON);
        break;
      case 2:
        block2.set(msg->data[0] == OPC_ACON);
        break;
      case 3:
        block3.set(msg->data[0] == OPC_ACON);
        break;
      case 10:
        Serial << "point changed" << endl;
        pointInput.set(msg->data[0] == OPC_ACOF);
        break;
    }
    break;
  }
}

//
/// print code version config details and copyright notice
//
void printConfig(void)
{
  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Sven Rosvall 2021") << endl;
  return;
}
