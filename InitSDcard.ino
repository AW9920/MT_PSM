void InitSDcard(void) {
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(CS_SD)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) {};
  }
  Serial.println("card initialized.");
}