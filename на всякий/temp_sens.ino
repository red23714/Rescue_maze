void init_temp() {
  therm_l.begin(0x5A);
  therm_l.setUnit(TEMP_C);

  therm_r.begin(0x5B);
  therm_r.setUnit(TEMP_C);
}

bool is_temp() {
  if (therm_l.read() || therm_r.read()) {
    if (therm_l.object() >= TEMP || therm_r.object() >= TEMP) return true;
  } else return false;
}

void debug_temp() {
  if (therm_l.read())  // On success, read() will return 1, on fail 0.
  {
    Serial.print("Object_l: " + String(therm_l.object(), 2));
    Serial.print("C");
    Serial.print("   Ambient_l: " + String(therm_l.ambient(), 2));
    Serial.print("C");
    Serial.println(" <-----> ");
  }
  if (therm_r.read())  // On success, read() will return 1, on fail 0.
  {
    Serial.print("Object_r: " + String(therm_r.object(), 2));
    Serial.print("C");
    Serial.print("   Ambient_r: " + String(therm_r.ambient(), 2));
    Serial.print("C");
    Serial.println(" <-----> ");
  }
}