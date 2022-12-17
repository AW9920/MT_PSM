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
      Serial.println(dt, 6);
      break;

    case 1:
      Serial.print(Enc1.read());
      Serial.print('\t');
      Serial.print(Enc2.read());
      Serial.print('\t');
      Serial.println(Enc3.read());
      break;

    case 2:
      ax1_angle = Ax1toAngle(Enc1.read());
      ax2_angle = Ax2toAngle(Enc2.read());
      ax3_angle = Ax3toAngle(Enc3.read());
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
      ax1_angle = Ax1toAngle(Enc1.read());
      ax2_angle = Ax2toAngle(Enc2.read());
      ax3_angle = Ax3toAngle(Enc3.read());
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
      ax1_angle = Ax1toAngle(Enc1.read());
      ax2_angle = Ax2toAngle(Enc2.read());
      ax3_angle = Ax3toAngle(Enc3.read());
      if (startRec) {
        Serial.print(*target_pos[0], 4);
        Serial.print('\t');
        Serial.print(*target_pos[1], 4);
        Serial.print('\t');
        Serial.print(*target_pos[2], 4);
        Serial.print('\t');
        Serial.print(ax1_angle, 4);
        Serial.print('\t');
        Serial.print(ax2_angle, 4);
        Serial.print('\t');
        Serial.println(ax3_angle, 4);
      }
      break;

    case 6:
      Serial.println(dt, 6);
      break;

    case 7:
      Serial.print(pre_rate_filter[0], 4);
      Serial.print('\t');
      Serial.print(pre_rate_filter[1], 4);
      Serial.print('\t');
      Serial.println(pre_rate_filter[2], 4);
      break;

    case 8:
      Serial.print(prev_e[0], 4);
      Serial.print('\t');
      Serial.print(prev_e[1], 4);
      Serial.print('\t');
      Serial.println(prev_e[2], 4);
      break;

    case 9:
      Serial.print(prev_e[0], 4);
      Serial.print('\t');
      Serial.print(prev_e[1], 4);
      Serial.print('\t');
      Serial.print(prev_e[2], 4);
      Serial.print('\t');
      Serial.print(control_values[0], 4);
      Serial.print('\t');
      Serial.print(control_values[1], 4);
      Serial.print('\t');
      Serial.println(control_values[2], 4);
      break;

    case 10:
      Serial.print(integral[0], 4);
      Serial.print('\t');
      Serial.print(integral[1], 4);
      Serial.print('\t');
      Serial.print(integral[2], 4);
      Serial.print('\t');
      Serial.print(clamp_I[0]);
      Serial.print('\t');
      Serial.print(clamp_I[1]);
      Serial.print('\t');
      Serial.println(clamp_I[2]);

      break;

    case 11:
      Serial.print(prev_e[0], 4);
      Serial.print('\t');
      Serial.print(prev_e[1], 4);
      Serial.print('\t');
      Serial.print(prev_e[2], 4);
      Serial.print('\t');
      Serial.print(control_values[0], 4);
      Serial.print('\t');
      Serial.print(control_values[1], 4);
      Serial.print('\t');
      Serial.println(control_values[2], 4);
      break;

    case 12:
      ax1_angle = Ax1toAngle(Enc1.read());
      ax2_angle = Ax2toAngle(Enc2.read());
      ax3_angle = Ax3toAngle(Enc3.read());
      if (startRec) {
        Serial.print(*target_pos[0], 4);
        Serial.print('\t');
        Serial.print(*target_pos[1], 4);
        Serial.print('\t');
        Serial.print(*target_pos[2], 4);
        Serial.print('\t');
        Serial.print(ax1_angle, 4);
        Serial.print('\t');
        Serial.print(ax2_angle, 4);
        Serial.print('\t');
        Serial.print(ax3_angle, 4);
        Serial.print('\t');
        Serial.print(prev_e[0], 4);
        Serial.print('\t');
        Serial.print(prev_e[1], 4);
        Serial.print('\t');
        Serial.print(prev_e[2], 4);
        Serial.print('\t');
        Serial.print(m_speed[0]);
        Serial.print('\t');
        Serial.print(m_speed[1]);
        Serial.print('\t');
        Serial.println(m_speed[2]);
      }
      break;

      case 13:
        Serial.print(m_speed[0]);
        Serial.print('\t');
        Serial.print(m_speed[1]);
        Serial.print('\t');
        Serial.println(m_speed[2]);
      break;
  }
}