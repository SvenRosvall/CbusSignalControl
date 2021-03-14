#include <CBUS2515.h>               // CAN controller and CBUS class
#include <CBUSconfig.h>             // module configuration

//
/// command interpreter for serial console input
//
void processSerialInput(CBUS2515 & cbus, CBUSConfig & config, void (*printConfig)(void))
{
  byte uev = 0;
  char msgstr[32];

  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {

      case 'n':
        // node config
        printConfig();

        // node identity
        Serial << F("> CBUS node configuration") << endl;
        Serial << F("> mode = ") << (config.FLiM ? "FLiM" : "SLiM") << F(", CANID = ") << config.CANID << F(", node number = ") << config.nodeNum << endl;
        Serial << endl;
        break;

      case 'e':
        // EEPROM learned event data table
        Serial << F("> stored events ") << endl;
        Serial << F("  max events = ") << config.EE_MAX_EVENTS << F(" EVs per event = ") << config.EE_NUM_EVS << F(" bytes per event = ") << config.EE_BYTES_PER_EVENT << endl;

        for (byte j = 0; j < config.EE_MAX_EVENTS; j++) {
          if (config.getEvTableEntry(j) != 0) {
            ++uev;
          }
        }

        Serial << F("  stored events = ") << uev << F(", free = ") << (config.EE_MAX_EVENTS - uev) << endl;
        Serial << F("  using ") << (uev * config.EE_BYTES_PER_EVENT) << F(" of ") << (config.EE_MAX_EVENTS * config.EE_BYTES_PER_EVENT) << F(" bytes") << endl << endl;

        Serial << F("  Ev#  |  NNhi |  NNlo |  ENhi |  ENlo | ");

        for (byte j = 0; j < (config.EE_NUM_EVS); j++) {
          sprintf(msgstr, "EV%03d | ", j + 1);
          Serial << msgstr;
        }

        Serial << F("Hash |") << endl;

        Serial << F(" --------------------------------------------------------------") << endl;

        // for each event data line
        for (byte j = 0; j < config.EE_MAX_EVENTS; j++) {
          if (config.getEvTableEntry(j) != 0) {
            sprintf(msgstr, "  %03d  | ", j);
            Serial << msgstr;

            // for each data byte of this event
            for (byte e = 0; e < (config.EE_NUM_EVS + 4); e++) {
              sprintf(msgstr, " 0x%02hx | ", config.readEEPROM(config.EE_EVENTS_START + (j * config.EE_BYTES_PER_EVENT) + e));
              Serial << msgstr;
            }

            sprintf(msgstr, "%4d |", config.getEvTableEntry(j));
            Serial << msgstr << endl;
          }
        }

        Serial << endl;

        break;

      // NVs
      case 'v':
        // note NVs number from 1, not 0
        Serial << "> Node variables" << endl;
        Serial << F("   NV   Val") << endl;
        Serial << F("  --------------------") << endl;

        for (byte j = 1; j <= config.EE_NUM_NVS; j++) {
          byte v = config.readNV(j);
          sprintf(msgstr, " - %02d : %3hd | 0x%02hx", j, v, v);
          Serial << msgstr << endl;
        }

        Serial << endl << endl;

        break;

      // CAN bus status
      case 'c':
        cbus.printStatus();
        break;

      case 'h':
        // event hash table
        config.printEvHashTable(false);
        break;

      case 'y':
        // reset CAN bus and CBUS message processing
        cbus.reset();
        break;

      case '*':
        // reboot
        config.reboot();
        break;

      case 'm':
        // free memory
        Serial << F("> free SRAM = ") << config.freeSRAM() << F(" bytes") << endl;
        break;

      case 'r':
        // renegotiate
        cbus.renegotiate();
        break;

      case 'z':
        // Reset module, clear EEPROM
        static bool ResetRq = false;
        static unsigned long ResWaitTime;
        if (!ResetRq) {
          // start timeout timer
          Serial << F(">Reset & EEPROM wipe requested. Press 'z' again within 2 secs to confirm") << endl;
          ResWaitTime = millis();
          ResetRq = true;
        }
        else {
          // This is a confirmed request
          // 2 sec timeout
          if (ResetRq && ((millis() - ResWaitTime) > 2000)) {
            Serial << F(">timeout expired, reset not performed") << endl;
            ResetRq = false;
          }
          else {
            //Request confirmed within timeout
            Serial << F(">RESETTING AND WIPING EEPROM") << endl;
            // TODO: Update CBUSConfig library to the latest where this method is available without arguments.
            //config.resetModule();
            ResetRq = false;
          }
        }
        break;

      default:
        // Serial << F("> unknown command ") << c << endl;
        break;
    }
  }
}
