/* Thes function is a debugging tool. It has multiple modes that allow to look at a number of predefined values
Input arguments:
  -type: Determines what values are looked at.

Below is a list which case prints which data
  //0 ->  "SamplingTime"
  //1 ->  "Joint Axis 1, 2 and 3 raw sensor value"
  //2 ->  "Joint Axis 1, 2 and 3 joint value"
  //3 ->  "Target position callback"
  //4 ->  "Joint values of Axis 1, 2 and 3 plus corresponding control vlaues"
  //5 ->  "Record step response of PSM" (desired and actual encoder values of Ax 1, 2 and 3)
  //6 ->  "Sampling Time (can be deleted)"
  //7 ->  "Filtered error rate de/dt of previous iteration"
  //8 ->  "Error values e of the previous iteration"
  //9 ->  "Error values e of the previous iteration plus resulting control values u"
  //10 ->  "integrated error plus the values resulting from active clamping"
  //11 ->  "Error values e of the previous iteration plus resulting control values u (Same as 9; can be deleted)"
  //12 ->  "Display of desired and current position data, error and resulting control values"
  //13 ->  "Control values u converted into Duty Cycle (ranging form -255 to +255)"
  //14 ->  "Display of desired and current position data, error and resulting control values. Plus the x, y and z coordinate of the ToolTip"
  //15 ->  "Computed desired position of servo values. Control the attached daVinci surgical tool"
*/

void SerialPrintData(int type) {
  float ax1_angle, ax2_angle, ax3_angle;
  switch (type) {
    case 0:
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

    case 14:
      ax1_angle = Ax1toAngle(Enc1.read());
      ax2_angle = Ax2toAngle(Enc2.read());
      ax3_angle = Ax3toAngle(Enc3.read());
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
      Serial.print(m_speed[2]);
      Serial.print('\t');
      Serial.print(x, 2);
      Serial.print('\t');
      Serial.print(y, 2);
      Serial.print('\t');
      Serial.println(z, 2);
      break;

    case 15:
      Serial.print(*target_pos[3], 2);
      Serial.print('\t');
      Serial.print(*target_pos[4], 2);
      Serial.print('\t');
      Serial.print(*target_pos[5], 2);
      Serial.print('\t');
      Serial.print(*target_pos[6], 2);
      Serial.print("\t");
      Serial.print(servo_val[0]);
      Serial.print("\t");
      Serial.print(servo_val[1]);
      Serial.print("\t");
      Serial.print(servo_val[2]);
      Serial.print("\t");
      Serial.println(servo_val[3]);
      break;
  }
}