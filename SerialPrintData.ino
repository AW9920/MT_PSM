void SerialPrintData(int type) {
  //0 ->  "SamplingTime"
  //1 ->  "Joint Axis 1, 2 and 3 raw sensor value"
  //2 ->  "Joint Axis 1, 2 and 3 joint value"
  //3 ->  "Target position callback"
  //4 ->
  //5 ->  "Record step response of PSM"

  float ax1_angle, ax2_angle, ax3_angle;

  switch (type) {
    case 0:
      // samplingTime = millis() - currentTime;
      // //Output sampling Time
      // Serial.print("Sampling Time:");
      // Serial.println(samplingTime);
      break;

    case 1:
      Serial.print(counter1);
      Serial.print('\t');
      Serial.print(counter2);
      Serial.print('\t');
      Serial.println(counter3);
      break;

    case 2:
      ax1_angle = Ax1toAngle(counter1);
      ax2_angle = Ax2toAngle(counter2);
      ax3_angle = Ax3toAngle(counter3);
      Serial.print(ax1_angle);
      Serial.print('\t');
      Serial.print(ax2_angle);
      Serial.print('\t');
      Serial.println(ax3_angle);
      break;

    case 3:
      Serial.print(target_pos1);
      Serial.print('\t');
      Serial.print(target_pos2);
      Serial.print('\t');
      Serial.println(target_pos3);
      break;

    case 4:
      ax1_angle = Ax1toAngle(counter1);
      ax2_angle = Ax2toAngle(counter2);
      ax3_angle = Ax3toAngle(counter3);
      Serial.print(ax1_angle);
      Serial.print('\t');
      Serial.print(ax2_angle);
      Serial.print('\t');
      Serial.print(ax3_angle);
      Serial.print('\t');
      Serial.print(control_values[0]);
      Serial.print('\t');
      Serial.print(control_values[1]);
      Serial.print('\t');
      Serial.println(control_values[2]);
      break;

    case 5:
      ax1_angle = Ax1toAngle(counter1);
      ax2_angle = Ax2toAngle(counter2);
      ax3_angle = Ax3toAngle(counter3);
      if (true) {
        Serial.print(*target_pos[0]);
        Serial.print('\t');
        Serial.print(*target_pos[1]);
        Serial.print('\t');
        Serial.print(*target_pos[2]);
        Serial.print('\t');
        Serial.print(ax1_angle);
        Serial.print('\t');
        Serial.print(ax2_angle);
        Serial.print('\t');
        Serial.println(ax3_angle);
      }
      break;

    case 6:
      getDT();
      Serial.println(dt,6);
      break;
  }
}