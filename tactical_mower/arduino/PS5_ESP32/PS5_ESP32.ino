// Dateipfad Buttons:   C:\Users\TobiasLüttin\AppData\Local\Arduino15\packages\esp32-bluepad32\hardware\esp32\4.1.0\tools\sdk\esp32c3\include\bluepad32\include\controller

#include <Bluepad32.h>  //board ESP32_BluePad32 installieren
                        //siehe https://www.youtube.com/watch?v=EEViXFoSzww
                        //und "ESP32 Dev Module" board in "esp32_bluepad32" auswählen
// #include <SoftwareSerial.h>


// CRC16-CCITT (0x1021, Startwert 0xFFFF) für Strings
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;  // Initialwert

  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;  // Byte ins obere CRC-Byte xor'en
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;    // Polynom anwenden
      } else {
        crc <<= 1;                    // Nur schieben
      }
    }
  }
  return crc;
}

// Komfort-Funktion für Arduino-String
uint16_t crc16_ccitt(const String& s) {
  return crc16_ccitt((const uint8_t*)s.c_str(), s.length());
}


#define myRX 18
#define myTX 15
#define Licht 25
#define Charging 33
#define Sirene 26
#define GPS_Status 5
#define GPS_Save 2
#define Lidar 32
#define Forward 27

unsigned long lastSerialTime = 0;
const unsigned long serialInterval = 200;

// Failsafe-Herzschlag: solange KEIN Controller L1 hält (oder gar keiner
// verbunden ist), synthetisiert loop() im selben idx=...-Format wie
// dumpGamepad() eine Null-Zeile auf Serial (USB-CDC zur Jetson).
// Damit kann joy_controller_node.py den L1-Release auch dann erkennen,
// wenn Bluepad32 mangels neuer HID-Reports nicht mehr processGamepad() aufruft.
const unsigned long failsafeInterval = 20;   // 50 Hz
unsigned long lastFailsafeTime = 0;

unsigned long lastVibrationTime = 0;
const unsigned long vibrationInterval = 30000;  // 30 Sekunden
const unsigned long vibrationDuration = 1000;   // 1 Sekunden
const int batteryLimit = 20;                    // ab 20% beginnt der Controller zu vibrieren

int mapRX, mapRX_alt = 0;
int mapY, mapY_alt = 0;
float Ti = 1.0;  // Zeitkonstante des Tiefpassfilters, gibt an wieviel vom neuen Wert genommen werden soll
int right;
int left;
bool GPS_ON = false;

const int SERIAL_BAUD_CUSTOM = 19200;
// --- Deadman Switch Flags ---
volatile bool g_deadmanPressed = false;   // true solange L1 gehalten wird
volatile bool g_deadmanPrev    = false;   // für Flankenerkennung (Loslassen)

// Edge-Detection pro Taste – verhindert Wiederholungen bei gehaltenem Knopf
static bool g_aWasDown = false;
static bool g_bWasDown = false;
static bool g_xWasDown = false;
static bool g_yWasDown = false;

// --- Button-States für UART ---
static bool g_miscSelectState   = false;  // aktueller Status SELECT
static bool g_lichtToggleState  = false;  // logischer Licht-Status (A)
static bool g_gpsToggleState    = false;  // logischer GPS-Status  (X)


// Skaliert einen Wert von einem Bereich auf einen anderen
float mapRange(float value, float in_min, float in_max, float out_min, float out_max) {
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// SoftwareSerial Serial1(myRX, myTX);

// ========== UART-Empfang vom Jetson ==========
// Separate buffers: one per port so partial messages never interleave.
String incomingUartBuffer = "";
String incomingUsbBuffer  = "";

// Shared command parser — called for both Serial1 (UART1 hw pins) and
// Serial (USB-CDC).  Each caller passes its own buffer so the two
// streams stay independent.  Debug output always goes to Serial (USB)
// which is fine: TX and RX are separate hardware FIFOs, and the Jetson
// joy parser only matches "idx=…" lines so our prints are harmless.
void processCommandFromStream(Stream& port, String& buffer) {
  if (!port.available()) return;

  char c = port.read();

  if (c == '\n') {
    buffer.trim();

    if (buffer.length() > 0) {
      Serial.print("UART RX: ");
      Serial.println(buffer);

      int lastComma = buffer.lastIndexOf(',');

      if (lastComma > 0) {
        String payload = buffer.substring(0, lastComma);
        String crcStr  = buffer.substring(lastComma + 1);

        uint16_t receivedCRC   = strtol(crcStr.c_str(), NULL, 16);
        uint16_t calculatedCRC = crc16_ccitt(payload);

        if (receivedCRC == calculatedCRC) {
          Serial.println("CRC OK - Verarbeite Befehl");

          int values[8] = {0};
          int idx = 0;
          int startPos = 0;

          for (int i = 0; i <= (int)payload.length() && idx < 8; i++) {
            if (i == (int)payload.length() || payload[i] == ',') {
              values[idx] = payload.substring(startPos, i).toInt();
              idx++;
              startPos = i + 1;
            }
          }

          // values[5] = licht, values[7] = charging
          if (values[5] == 1) {
            digitalWrite(Licht, HIGH);
            g_lichtToggleState = true;
            Serial.println("Jetson: Licht EIN");
          } else if (values[5] == 0) {
            digitalWrite(Licht, LOW);
            g_lichtToggleState = false;
            Serial.println("Jetson: Licht AUS");
          }

          if (values[7] == 1) {
            digitalWrite(Charging, HIGH);
            Serial.println("Jetson: CHARGING ON");
          } else {
            digitalWrite(Charging, LOW);
            Serial.println("Jetson: CHARGING OFF");
          }

        } else {
          Serial.print("CRC FEHLER! Erwartet: ");
          Serial.print(calculatedCRC, HEX);
          Serial.print(", Empfangen: ");
          Serial.println(receivedCRC, HEX);
        }
      } else {
        Serial.println("Ungültiges Format (kein Komma gefunden)");
      }
    }

    buffer = "";

  } else {
    buffer += c;

    if (buffer.length() > 100) {
      Serial.println("WARNUNG: UART Buffer overflow - wird geleert");
      buffer = "";
    }
  }
}

// Legacy UART1 path (hw pins 18/15 @ 19200) — kept for backwards compat.
void processIncomingUartCommand() {
  processCommandFromStream(Serial1, incomingUartBuffer);
}
// ========== ENDE UART-Empfang ==========


// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }

  // Beim Disconnect Null-Daten mit CRC senden
  String zeroMsg = buildZeroMessage();
  Serial1.println(zeroMsg);
  Serial.println(zeroMsg);
}

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );


  // Werte mappen
  if (ctl->axisRX() > 50) {
    mapRX = mapRange(ctl->axisRX(), 50, 500, 0, 100);
  }

  if (ctl->axisY() > 50) {
    mapY = mapRange(ctl->axisY(), 50, 500, 0, 100);
  }

  if (ctl->axisRX() < -50) {
    mapRX = mapRange(ctl->axisRX(), -50, -500, -0, -100);
  }

  if (ctl->axisY() < -50) {
    mapY = mapRange(ctl->axisY(), -50, -500, -0, -100);
  }

  if (ctl->axisRX() < 50 && ctl->axisRX() > -50) {
    mapRX = 0;
  }

  if (ctl->axisY() < 50 && ctl->axisY() > -50) {
    mapY = 0;
  }


  right = mapRange(ctl->throttle(), 0, 1023, 0, 100);

  left = mapRange(ctl->brake(), 0, 1023, 0, 100);


  Serial.print("brake: ");
  // Serial.println(ctl->brake());
  Serial.println(left);

  mapRX = constrain(mapRX, -100, 100);
  mapY = constrain(mapY * -1, -100, 100);

  // Tiefpassfilter für Steuersignale, cu 2025067
  // filter nur aktivieren wenn neuer Wert kleiner als alter Wert, also beim Bremsen
  // evlt. auch nur wenn neuer Wert = 0;
  // Filter evtl. nur für Vor/Rück-Bewegung notwendig
  // if (mapRX < mapRX_alt)
  //   mapRX = Ti*mapRX + (1-Ti)* mapRX_alt;
  //   mapRX_alt = mapRX;
  // else
  //   mapRX_alt = 0;
  // endif

  if (mapY < mapY_alt) {
    mapY = Ti * mapY + (1 - Ti) * mapY_alt;
  }

  mapY_alt = mapY;


  float values[] = {
    mapRX,
    mapY,
    right,
    left,
    g_miscSelectState ? 1.0f : 0.0f,   // SELECT-Status
    g_lichtToggleState ? 1.0f : 0.0f,  // Licht (A)
    g_gpsToggleState ? 1.0f : 0.0f     // GPS (X)
  };

  int arraySize = sizeof(values) / sizeof(values[0]);

  String message2 = "";
  for (int i = 0; i < arraySize; i++) {
    message2 += values[i];
    if (i < arraySize - 1) {
      message2 += ',';  // Trennzeichen zwischen den Werten
    }
  }

    // --- Deadman-gesteuertes Senden ---
    // Halte Intervall ein, damit Bus nicht geflutet wir
    // Neue Nachricht mit CRC bauen
    String message = buildDataMessage(values, arraySize);

    if (millis() - lastSerialTime > serialInterval) {
      if (g_deadmanPressed) {
        // L1 gehalten -> aktuelle Steuerdaten senden
        Serial1.println(message);   // nur noch Daten + CRC
        Serial.println(message);
      } else {
        // L1 NICHT gehalten -> durchgehend Nullen (Failsafe)
        // String zeroMsg = buildZeroMessage();
        // Serial1.println(zeroMsg);
        // Serial.println(zeroMsg);
      }
      lastSerialTime = millis();
    }

    if (!g_deadmanPressed && g_deadmanPrev) {
      // gerade losgelassen -> Null-Daten senden
      String zeroMsg = buildZeroMessage();
      Serial1.println(zeroMsg);
      Serial.println(zeroMsg);
    }
}

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...

  // Akku-Stand auslesen
  uint8_t rawBattery = ctl->battery();
  float batteryPercent = (rawBattery / 255.0f) * 100.0f;

  Serial.print("Battery: ");
  Serial.print(batteryPercent, 1);  // eine Nachkommastelle
  Serial.println("%");

  // Deadman Status für diese Schleife aktualisieren
  g_deadmanPressed = ctl->l1();

  if (batteryPercent < 20) {
    unsigned long currentTime = millis();

    if (currentTime - lastVibrationTime >= vibrationInterval) {
      ctl->playDualRumble(0 /* delayedStartMs */, vibrationDuration /* durationMs */,
                          0xC8 /* weakMagnitude */, 0xC8 /* strongMagnitude */);
      lastVibrationTime = currentTime;
    }
  }

    /* Select Taste Logik:*/
  g_miscSelectState = ctl->miscSelect();

  /* X Taste*/
  bool aNow = ctl->a();
  if (aNow && !g_aWasDown) {
    // Licht-Status toggeln: wird per UART gesendet UND per GPIO ausgegeben
    g_lichtToggleState = !g_lichtToggleState;

    digitalWrite(Licht, g_lichtToggleState ? HIGH : LOW);
    Serial.println(g_lichtToggleState
                   ? "Licht eingeschaltet (HIGH, UART-Flag = 1)"
                   : "Licht ausgeschaltet (LOW, UART-Flag = 0)");
  }
  g_aWasDown = aNow;


  // --- X (GPS toggeln) nur auf Flanke, ohne delay ---
  g_gpsToggleState = ctl->x();


  // Another way to query controller data is by getting the buttons() function.
  // See how the different "dump*" functions dump the Controller info.
  dumpGamepad(ctl);

  // Flankenstatus für nächsten Loop speichern
  g_deadmanPrev = g_deadmanPressed;

}


// True, wenn irgendein verbundener Controller gerade L1 (Deadman) hält.
static bool anyDeadmanHeld() {
  for (auto c : myControllers) {
    if (c && c->isConnected() && c->isGamepad() && c->l1()) {
      return true;
    }
  }
  return false;
}

// Druckt eine synthetische Null-Zeile im exakt selben Format wie
// dumpGamepad(), damit der Jetson-Parser (ESP_JOY_LINE) sie matched.
// Nur Throttle: 50 Hz, und nur wenn L1 NICHT gehalten wird — sonst
// soll dumpGamepad() weiter die echten Werte streamen.
void emitFailsafeIdxLine() {
  if (millis() - lastFailsafeTime < failsafeInterval) return;
  if (anyDeadmanHeld()) return;
  Serial.printf(
    "idx=0, dpad: 0x00, buttons: 0x0000, axis L:    0,    0, axis R:    0,    0, "
    "brake:    0, throttle:    0, misc: 0x00, gyro x:     0 y:     0 z:     0, "
    "accel x:     0 y:     0 z:     0\n"
  );
  lastFailsafeTime = millis();
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Baut die CSV-Nutzdaten aus einem Werte-Array + CRC16-CCITT am Ende
String buildDataMessage(const float* values, int count) {
  String payload;

  // Nutzdaten als kommagetrennte ganze Zahlen aufbauen
  for (int i = 0; i < count; i++) {
    payload += String((int)values[i]);   // bei Bedarf auf int runden
    if (i < count - 1) {
      payload += ',';
    }
  }

  // CRC16 über die reinen Nutzdaten berechnen
  uint16_t crc = crc16_ccitt(payload);

  // CRC als letztes Feld (hex oder dezimal, hier hex)
  payload += ',';
  payload += String(crc, HEX);      // z.B. "3F2A"

  return payload;                   // Beispiel: "0,10,0,0,0,1,0,3F2A"
}

// Null-Nachricht in gleicher Struktur (für Deadman/offline)
String buildZeroMessage() {
  // 7 Felder wie im values-Array
  String payload = "0,0,0,0,0,0,0";
  uint16_t crc = crc16_ccitt(payload);
  payload += ',';
  payload += String(crc, HEX);
  return payload;
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Serial1.begin();
  Serial1.begin(SERIAL_BAUD_CUSTOM, SERIAL_8N1, myRX, myTX);

    // Licht AUS beim Start
  digitalWrite(Licht, LOW);      // Pegel vorgeben
  pinMode(Licht, OUTPUT);        // dann als Ausgang schalten

  // Charging AUS beim Start
  digitalWrite(Charging, LOW);
  pinMode(Charging, OUTPUT);

  // Sirene AUS beim Start
  digitalWrite(Sirene, LOW);
  pinMode(Sirene, OUTPUT);

  // GPS Status AUS beim Start
  digitalWrite(GPS_Status, LOW);
  pinMode(GPS_Status, OUTPUT);

  // GPS Save AUS beim Start
  digitalWrite(GPS_Save, LOW);
  pinMode(GPS_Save, OUTPUT);

  // Lidar AUS beim Start
  digitalWrite(Lidar, LOW);
  pinMode(Lidar, OUTPUT);

  // Forward AUS beim Start
  digitalWrite(Forward, LOW);
  pinMode(Forward, OUTPUT);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

  // Failsafe-Herzschlag: Null-Zeile bei losem L1 / fehlendem Controller.
  emitFailsafeIdxLine();

  // UART-Empfang vom Jetson: check both Serial1 (hw UART1) and Serial (USB-CDC)
  processIncomingUartCommand();
  processCommandFromStream(Serial, incomingUsbBuffer);

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  // delay(150);
}