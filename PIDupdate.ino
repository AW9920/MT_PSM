void PIDupdate(float* target, int index) {
  //Function variables
  float current;
  float kp, ki, kd;

  //Choose parameters based on index
  switch (index) {
    case 0:
      current = Ax1toAngle(counter1);
      kp = 1, ki = 0, kd = 0;
      break;
    case 1:
      current = Ax2toAngle(counter2);
      kp = 1, ki = 0, kd = 0;
      break;
    case 2:
      current = Ax3toAngle(counter3);
      kp = 1, ki = 0, kd = 0;
      break;
    default:
      kp = 0, ki = 0, kd = 0;
      break;
  }

  float e = *target - current;
  float u = kp * e;

  //Determine direction
  int dir = 1;
  if (e < 0) {
    dir = -1;
  }

  //Determine speed
  int speed = (int)fabs(u);
  if (speed > 255) {
    speed = 255;
  }

  motor[index].setSpeed(dir * speed);
}