void PIDupdate(float* target, int index, String mode) {
  //Function variables
  float current;
  float kp, ki, kd;
  int PID_select;
  float u;
  float Umag;
  double rate;
  int sign_e;
  int sign_u;

  // Select mode of controller ("P","PI","PD" or "PID")
  if (mode == "P") {
    PID_select = 0;
  } else if (mode == "PI") {
    PID_select = 1;
  } else if (mode == "PD") {
    PID_select = 2;
  } else if (mode == "PID") {
    PID_select = 3;
  } else {
    PID_select = 0;
  }  //DEFAULT

  //Choose parameters based on index
  switch (index) {
    case 0:
      current = Ax1toAngle(Enc1.read());
      kp = 35, ki = 1, kd = 5;  //5
      Umag = 15.0;
      break;
    case 1:
      current = Ax2toAngle(Enc2.read());
      kp = 25, ki = 1, kd = 3;  //40
      Umag = 15.0;
      break;
    case 2:
      current = Ax3toAngle(Enc3.read());
      kp = 45, ki = 1, kd = 3.8;  //3.8
      Umag = 7.4;
      break;
    default:
      kp = 0, ki = 0, kd = 0;
      Umag = 1;
      break;
  }

  //---------------------Controller---------------
  float e = *target - current;
  //Implement LP filter for rate
  rate = 0.956 * e_yn1[index] + 0.0245 * rate_e[index] + 0.0245 * prev_e[index];
  //Update LP filter values
  e_yn1[index] = rate;
  prev_e[index] = e;

  //PID controlelr state machine
  switch (PID_select) {
    case 0:
      u = kp * e;
      break;

    case 1:
      if (!clamp_I) {
        // Integrator
        integral[index] += e * dt;
        u = kp * e + ki * integral[index];
      } else if (clamp_I) {
        u = kp * e;
      }
      break;

    case 2:
      rate = (e - prev_e[index]) / dt;
      rate_e[index] = rate;
      u = kp * e + kd * rate;
      break;

    case 3:
      // Derivative
      rate = (e - prev_e[index]) / dt;
      rate_e[index] = rate;
      if (!clamp_I) {
        // Integrator
        integral[index] += e * dt;
        u = kp * e + ki * integral[index] + kd * rate;
      } else if (clamp_I) {
        u = kp * e + kd * rate;
      }
      break;
    default:
      u = 0;
      break;
  }

  // Check if clamping is requried
  control_values[index] = u;
  sat_control_values[index] = constrain(u, -255, 255);
  if (u < 0) {
    sign_u = -1;
  } else {
    sign_u = 1;
  }
  if (e < 0) {
    sign_e = -1;
  } else {
    sign_e = 1;
  }
  if ((sign_u == sign_e) && (control_values[index] != sat_control_values[index])) {
    clamp_I = true;
  }

  //Determine direction
  int dir = -1;
  if (e < 0) {
    dir = 1;
  }

  //Determine speed
  int speed = (int)fabs(u);
  if (speed > 255) {
    speed = 255;
  }

  motor[index].setSpeed(dir * speed);
}