/*PIDupdate --> Geneartion of control values for axes 1-3;
Update routine that is called once per cycle to update the control values of each axis based on the target value
which is handed over.
Input Arguments:
  -target: Desired Position (is received over UART)
  -index: incrementing value that indicates the axis of interest.
  -Mode: Allows to select between 'P', 'PI', 'PD' and 'PID' controller
In addition the function features integrator clamping to avoid effects of integrator windup.
*/
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
  float clamp_Lim_up;
  float clamp_Lim_low;
  int dir;
  int speed;

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
      kp = 35, ki = 50, kd = 5;  //5
      clamp_Lim_up = 32;
      clamp_Lim_low = -32;
      Umag = 15.0;
      break;
    case 1:
      current = Ax2toAngle(Enc2.read());
      kp = 60, ki = 110, kd = 5;  //40
      Umag = 15.0;
      clamp_Lim_up = 40;
      clamp_Lim_low = -40;
      break;
    case 2:
      current = Ax3toAngle(Enc3.read());
      kp = 45, ki = 800, kd = 3.8;  //3.8
      Umag = 7.4;
      clamp_Lim_up = 7;
      clamp_Lim_low = -7;
      break;
    default:
      kp = 0, ki = 0, kd = 0;
      Umag = 1;
      break;
  }

  //---------------------Controller---------------
  float e = *target - current;
  //----Derivative
  rate = (e - prev_e[index]) / dt;
  rate_e[index] = rate;
  //----LP-filter
  rate = 0.956 * pre_rate_filter[index] + 0.0245 * rate_e[index] + 0.0245 * prev_rate_e[index];

  //PID controlelr state machine
  switch (PID_select) {
    case 0:  //P-mode
      u = kp * e;
      break;

    case 1:  //PI-mode
      if (!clamp_I[index]) {
        // Integrator
        integral[index] += e * dt;
        u = kp * e + ki * integral[index];
      } else if (clamp_I[index]) {
        u = kp * e;
      }
      break;

    case 2:  //PD-mode
      u = kp * e + kd * rate;
      break;

    case 3:  //PID-mode (To-Do: NOT WORKING)
      if (!clamp_I[index]) {
        // Integrator
        integral[index] += e * dt;
        u = kp * e + ki * integral[index] + kd * rate;
      } else if (clamp_I[index]) {
        u = kp * e + kd * rate;
      }
      break;
  }

  //Update LP filter values for rate
  pre_rate_filter[index] = rate;
  prev_rate_e[index] = rate_e[index];
  //Update previous error
  prev_e[index] = e;

  // Check if clamping is requried
  control_values[index] = u;
  sat_control_values[index] = constrain(u, clamp_Lim_low, clamp_Lim_up);  //Dont clamp at physical limit of motor; Prevents remaining Windup!
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
    clamp_I[index] = true;
  } else {
    clamp_I[index] = false;
  }

  //Determine direction
  dir = -1;
  if (u < 0) {
    dir = 1;
  }
  //Determine speed
  speed = (int)fabs(u);
  if (speed > 255) {
    speed = 255;
  }

  if((speed < 8) && (index == 2)){
    speed = 8;
  }
  
  m_speed[index] = dir * speed; //For Evaluation

  motor[index].setSpeed(dir * speed);
}