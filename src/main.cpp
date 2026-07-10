#include <Arduino.h>

void setup()
{
  Serial.begin(115200);

  unsigned long start = millis();
  while (!Serial && millis() - start < 5000)
  {
    delay(10);
  }

  delay(1000);

  Serial.println();
  Serial.println("===== ESP32-S3 Serial Test =====");

  Serial.print("Chip model: ");
  Serial.println(ESP.getChipModel());

  Serial.print("CPU freq MHz: ");
  Serial.println(ESP.getCpuFreqMHz());

  Serial.print("Flash size: ");
  Serial.println(ESP.getFlashChipSize());

  Serial.print("PSRAM found: ");
  Serial.println(psramFound() ? "YES" : "NO");

  Serial.print("PSRAM size: ");
  Serial.println(ESP.getPsramSize());
}

void loop()
{
  Serial.println("running...");
  delay(1000);

    Serial.print("Chip model: ");
  Serial.println(ESP.getChipModel());

  Serial.print("CPU freq MHz: ");
  Serial.println(ESP.getCpuFreqMHz());

  Serial.print("Flash size: ");
  Serial.println(ESP.getFlashChipSize());

  Serial.print("PSRAM found: ");
  Serial.println(psramFound() ? "YES" : "NO");

  Serial.print("PSRAM size: ");
  Serial.println(ESP.getPsramSize());
}