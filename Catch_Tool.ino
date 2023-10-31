void Catch_Tool(void) {
  //Extend Prismatic Joint
  *target_pos[2] = 100;
  long extend_start = millis();
  long duration = 1000;
  while (Ax3toAngle(Enc3.read()) < (0.95 * *target_pos[2]) && (millis() - extend_start) <= duration) {
    PIDupdate(target_pos[2], 2, "P");
  }
  motor3.setSpeed(0);
  //------------------Catch Surgical Instrument------------------------
  //---------(Cab only be done when setup; otherwise damage)-----------
  for (int i = 0; i < 4; i++) {
    servos[i].write(0);
  }
  delay(1000);
  for (int i = 0; i < 4; i++) {
    servos[i].write(180);
  }
  delay(1000);
  //Snapp back too offset
  for (int i = 0; i < 4; i++) {
    servos[i].write(servo_off[i]);
  }
  delay(1000);
}