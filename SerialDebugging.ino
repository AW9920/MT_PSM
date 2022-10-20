void SerialDebugging(char mode) {
  int val;

  if (mode == 'LS') {
    val = 0;
  } else if (mode == 'DC_ENC') {
    val = 1;
  } else if (mode == 'S_ENC') {
    val = 2;
  }

  switch (val) {
    case 0:
      Serial.print("Limit switch 1:\t");
      Serial.print(digitalRead(LS1_NC));
      Serial.print('\t');
      Serial.println(digitalRead(LS1_NO));

      Serial.print("Limit switch 2:\t");
      Serial.print(digitalRead(LS2_NC));
      Serial.print('\t');
      Serial.println(digitalRead(LS2_NO));

      Serial.print("Limit switch 3:\t");
      Serial.print(digitalRead(LS3_NC));
      Serial.print('\t');
      Serial.println(digitalRead(LS3_NO));
      delay(200);
      break;

    case 1:
      if (counter1 != temp1) {
        Serial.println(counter1);
        temp1 = counter1;
      }
      if (counter2 != temp2) {
        Serial.println(counter2);
        temp2 = counter2;
      }
      if (counter3 != temp3) {
        Serial.println(counter3);
        temp3 = counter3;
      }
      break;

    case 2:
      break;
  }
}