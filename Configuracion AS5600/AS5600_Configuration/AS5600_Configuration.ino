#include <Wire.h>
#include <ctype.h>

#define AS5600_ADDR 0x36

// Registros
#define ZMCO_REG 0x00
#define ZPOS_REG_MSB 0x01
#define ZPOS_REG_LSB 0x02
#define MANG_REG_MSB 0x05
#define MANG_REG_LSB 0x06
#define CONF_REG_MSB 0x07
#define CONF_REG_LSB 0x08
#define STATUS_REG 0x0B
#define RAW_ANGLE_MSB 0x0C
#define RAW_ANGLE_LSB 0x0D
#define BURN_REG 0xFF

// Comandos de BURN según la hoja de datos
#define BURN_SETTING_CMD 0x40
#define BURN_ANGLE_CMD 0x80

#define PIN_INPUT 4

// --- Helpers I2C robustos ---

// Devuelve 0xFF en caso de error
uint8_t readReg8(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  int tx = Wire.endTransmission();
  if (tx != 0) {
    Serial.print("I2C write error (readReg8), code: ");
    Serial.println(tx);
    return 0xFF;
  }
  delay(5);
  int n = Wire.requestFrom((int)AS5600_ADDR, 1);
  if (n != 1) {
    Serial.print("I2C: expected 1 byte, got ");
    Serial.println(n);
    return 0xFF;
  }
  return Wire.read();
}

// Devuelve 0xFFFF en caso de error
uint16_t readReg16(uint8_t msbReg) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(msbReg);
  int tx = Wire.endTransmission();
  if (tx != 0) {
    Serial.print("I2C write error (readReg16), code: ");
    Serial.println(tx);
    return 0xFFFF;
  }
  delay(5);
  int n = Wire.requestFrom((int)AS5600_ADDR, 2);
  if (n != 2) {
    Serial.print("I2C: expected 2 bytes, got ");
    Serial.println(n);
    return 0xFFFF;
  }
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return ((uint16_t)hi << 8) | lo;
}

void writeReg8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  Wire.write(value);
  int tx = Wire.endTransmission();
  if (tx != 0) {
    Serial.print("I2C write error (writeReg8), code: ");
    Serial.println(tx);
  }
  delay(5);
}

void writeReg16(uint8_t msbReg, uint16_t value) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(msbReg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  int tx = Wire.endTransmission();
  if (tx != 0) {
    Serial.print("I2C write error (writeReg16), code: ");
    Serial.println(tx);
  }
  delay(5);
}

// --- waitForYes con timeout opcional (ms). timeoutMs = 0 => bloquear indefinidamente ---
bool waitForYes(const char* msg, unsigned long timeoutMs = 0) {
  Serial.print(msg);
  Serial.print(" (Y/N): ");

  unsigned long start = millis();
  while (true) {
    if (Serial.available() > 0) {
      // Ignorar espacios y saltos de línea iniciales
      while (Serial.available() && isspace((unsigned char)Serial.peek())) Serial.read();

      if (Serial.available() == 0) continue;

      int r = Serial.read();
      if (r < 0) continue;
      char ch = (char)r;
      ch = toupper((unsigned char)ch);

      // Vaciar resto de la línea para evitar lecturas posteriores inesperadas
      while (Serial.available()) {
        int rr = Serial.read();
        if (rr == '\n') break;
      }

      if (ch == 'Y' || ch == 'S') {
        Serial.println(ch);
        return true;
      } else if (ch == 'N') {
        Serial.println(ch);
        return false;
      } else {
        Serial.println();
        Serial.println("Respuesta no válida. Ingrese Y (sí) o N (no):");
      }
    }

    // Timeout
    if (timeoutMs > 0 && (millis() - start) >= timeoutMs) {
      Serial.println();
      Serial.println("Tiempo de espera agotado.");
      return false; // asumir NO si expira
    }

    delay(10); // cede CPU
    yield();   // especialmente útil en ESP32
  }
}

void setup() {
  Serial.begin(115200);
  // Si en tu ESP32 usas pines SDA/SCL no por defecto, usa Wire.begin(SDA, SCL);
  Wire.begin();
  pinMode(PIN_INPUT, INPUT);
  delay(100);

  Serial.println("\n=== Configuración AS5600 paso a paso ===");
  Serial.println("ATENCIÓN: Las operaciones BURN son irreversibles. Proceda con cuidado.");

  // Pregunta inicial con timeout de 15s (ejemplo). Cambia a 0 para esperar indefinidamente.
  if (!waitForYes("¿Desea configurar el AS5600?", 15000UL)) {
    Serial.println("Se ha elegido no configurar el AS5600 (o timeout). Saltando al loop principal.");
    Serial.println("\n=== Configuración OMITIDA ===");
    return;  // termina setup -> va al loop
  }

  uint8_t zmco = 0;

  // Paso 2
  if (waitForYes("2) Leer STATUS_REG")) {
    delay(100);
    uint8_t status = readReg8(STATUS_REG);
    if (status == 0xFF) {
      Serial.println("Error leyendo STATUS_REG (I2C).");
    } else {
      bool md = (status & (1 << 5));
      Serial.print("   MD (magnet detect): ");
      Serial.println(md ? "OK" : "NO DETECTADO");
      while (!md) {
        Serial.println("   Coloca un imán adecuado y confirma nuevamente.");
        while (!waitForYes("   ¿Seguir?")) {
          Serial.println("   Esperando confirmación para continuar...");
          delay(500);
          yield();
        }
        status = readReg8(STATUS_REG);
        if (status == 0xFF) {
          Serial.println("Error leyendo STATUS_REG (I2C). Abortando paso.");
          break;
        }
        md = (status & (1 << 5));
        Serial.print("   MD (magnet detect): ");
        Serial.println(md ? "OK" : "NO DETECTADO");
      }
    }
  } else {
    Serial.println("   Paso 2 saltado.");
  }

  // Paso 3
  if (waitForYes("3) Leer ZMCO (veces BURN_ANGLE)")) {
    delay(100);
    uint8_t r = readReg8(ZMCO_REG);
    if (r == 0xFF) {
      Serial.println("Error leyendo ZMCO (I2C).");
    } else {
      zmco = r & 0x03;
      Serial.print("   ZMCO = ");
      Serial.println(zmco);
    }
  } else {
    Serial.println("   Paso 3 saltado.");
  }

  // Pasos iniciales si zmco == 0
  if (zmco == 0) {
    if (waitForYes("4) Leer CONF, aplicar ajustes y reescribir")) {
      delay(100);
      uint16_t conf = readReg16(CONF_REG_MSB);
      if (conf == 0xFFFF) {
        Serial.println("Error leyendo CONF (I2C).");
      } else {
        conf &= ~(1 << 7);
        conf &= ~(0b111 << 4);
        conf = (conf & ~(0b11 << 2)) | (0b11 << 2);
        conf &= ~(0b11);
        writeReg16(CONF_REG_MSB, conf);
        Serial.println("   CONF reescrito");
      }

      if (waitForYes("5) Fijar MANG a 0x0FFF")) {
        delay(100);
        writeReg16(MANG_REG_MSB, 0x0FFF);
        Serial.println("   MANG escrito");
      } else {
        Serial.println("   Paso 5 saltado.");
      }

      if (waitForYes("6) BURN_SETTING (0x40)")) {
        writeReg8(BURN_REG, BURN_SETTING_CMD);
        delay(100);
        Serial.println("   BURN_SETTING enviado");
      } else {
        Serial.println("   Paso 6 saltado.");
      }

      if (waitForYes("7) Verificar: enviar 0x01,0x11,0x10 a BURN")) {
        Wire.beginTransmission(AS5600_ADDR);
        Wire.write(BURN_REG);
        Wire.write(0x01);
        Wire.write(0x11);
        Wire.write(0x10);
        int tx = Wire.endTransmission();
        if (tx != 0) {
          Serial.print("I2C: error al enviar secuencia BURN, code ");
          Serial.println(tx);
        } else {
          Serial.println("   Secuencia enviada");
        }
      } else {
        Serial.println("   Paso 7 saltado.");
      }
    } else {
      Serial.println("   Pasos 4-7 saltados.");
    }
  } else {
    Serial.println("No se puede configurar nuevamente el AS5600 (ZMCO != 0).");
  }

  // Paso 8
  if (waitForYes("8) Leer MANG y CONF")) {
    delay(100);
    uint16_t mang_check = readReg16(MANG_REG_MSB);
    uint16_t conf_check = readReg16(CONF_REG_MSB);
    if (mang_check == 0xFFFF || conf_check == 0xFFFF) {
      Serial.println("Error leyendo MANG/CONF (I2C).");
    } else {
      Serial.print("   MANG = 0x");
      Serial.println(mang_check, HEX);
      Serial.print("   CONF = 0x");
      Serial.println(conf_check, HEX);
    }
  } else {
    Serial.println("   Paso 8 saltado.");
  }

  // Pasos 9-12 si ZMCO < 3
  if (zmco < 3) {
    if (waitForYes("9) Leer RAW_ANGLE y fijar ZPOS")) {
      delay(100);
      uint16_t raw = readReg16(RAW_ANGLE_MSB);
      if (raw == 0xFFFF) {
        Serial.println("Error leyendo RAW_ANGLE (I2C).");
      } else {
        raw &= 0x0FFF;
        writeReg16(ZPOS_REG_MSB, raw);
        Serial.print("   ZPOS fijado a 0x");
        Serial.println(raw, HEX);
      }

      if (waitForYes("10) BURN_ANGLE (0x80)")) {
        writeReg8(BURN_REG, BURN_ANGLE_CMD);
        Serial.println("   BURN_ANGLE enviado");
      } else {
        Serial.println("   Paso 10 saltado.");
      }

      if (waitForYes("11) Verificar: enviar 0x01,0x11,0x10 a BURN")) {
        Wire.beginTransmission(AS5600_ADDR);
        Wire.write(BURN_REG);
        Wire.write(0x01);
        Wire.write(0x11);
        Wire.write(0x10);
        int tx = Wire.endTransmission();
        if (tx != 0) {
          Serial.print("I2C: error al enviar secuencia BURN, code ");
          Serial.println(tx);
        } else {
          Serial.println("   Secuencia enviada");
        }
        delay(100);
      } else {
        Serial.println("   Paso 11 saltado.");
      }

      if (waitForYes("12) Leer ZPOS para comprobar")) {
        uint16_t zpos_check = readReg16(ZPOS_REG_MSB);
        if (zpos_check == 0xFFFF) {
          Serial.println("Error leyendo ZPOS (I2C).");
        } else {
          Serial.print("   ZPOS = 0x");
          Serial.println(zpos_check, HEX);
        }
      } else {
        Serial.println("   Paso 12 saltado.");
      }
    } else {
      Serial.println("   Pasos 9-12 saltados.");
    }

    if (waitForYes("13) Leer ZMCO final")) {
      delay(100);
      uint8_t zmco_final = readReg8(ZMCO_REG);
      if (zmco_final == 0xFF) {
        Serial.println("Error leyendo ZMCO final (I2C).");
      } else {
        zmco_final &= 0x03;
        Serial.print("   ZMCO final = ");
        Serial.println(zmco_final);
      }
    } else {
      Serial.println("   Paso 13 saltado.");
    }

  } else {
    Serial.println("Tampoco se puede volver a configurar el ángulo 0 (ZMCO >= 3).");
  }

  Serial.println("\n=== Configuración COMPLETA ===");
}

void loop() {
  uint16_t raw_angle = readReg16(RAW_ANGLE_MSB);
  if (raw_angle == 0xFFFF) {
    Serial.println("Error leyendo RAW_ANGLE (I2C).");
    delay(500);
    return;
  }
  raw_angle &= 0x0FFF;

  uint16_t adc_angle = analogRead(PIN_INPUT);
  float digitalValue = (raw_angle / 4095.0f) * 360.0f;
  float analogValue = (adc_angle / 4095.0f) * 360.0f;

  Serial.print("Lectura Digital: ");
  Serial.println(raw_angle);
  Serial.print("Lectura Analógica: ");
  Serial.println(adc_angle);
  Serial.print("Angulo Digital: ");
  Serial.println(digitalValue, 2);
  Serial.print("Angulo Analógico: ");
  Serial.println(analogValue, 2);
  delay(1000);
}
