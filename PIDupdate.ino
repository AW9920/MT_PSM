void PIDupdate(float* target, int index, String mode) {
  //Function variables
  float current;
  float kp, ki, kd;
  int PID_select;
  float u;

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
      current = Ax1toAngle(counter1);
      kp = 10, ki = 1, kd = 0.3;
      break;
    case 1:
      current = Ax2toAngle(counter2);
      kp = 10, ki = 2, kd = 0.025;
      break;
    case 2:
      current = Ax3toAngle(counter3);
      kp = 10, ki = 2, kd = 0.025;
      break;
    default:
      kp = 0, ki = 0, kd = 0;
      break;
  }

  //---------------------Controller---------------
  getDT();
  float e = *target - current;
  integral[index] += integral[index] * dt;
  rate_e[index] = (e - prev_e[index]) / dt;
  switch (PID_select) {
    case 0:
      u = kp * e;
      break;

    case 1:
      u = kp * e + ki * integral[index];
      break;

    case 2:
      u = kp * e + kd * rate_e[index];
      break;

    case 3:
      u = kp * e + ki * integral[index] + kd * rate_e[index];
      break;
    default:
      u = 0;
      break;
  }
  prev_e[index] = e;
  control_values[index] = u;

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