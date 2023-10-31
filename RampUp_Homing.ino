void RampUp_Homing(void) {
  // -----------------------------Ramp up to Zero-Position----------------------------------
  long RampUp_Duration = 4000;  //Duration of Ramping Up to initial target position. Time in ms (milliseconds)
  long RampUp_StartTime;        //Reference time when ramping up process is starte
  long RampUp_time;             //Current time within the rampup process (t0-->Start of rampup)

  //------------Set Target Position (snappshot of MTM)-------------
  while (!Serial3.available()) {
    //Wait for data from MTM
  }
  recvWithStartEndMarkers();
  while (newData == true) {
    receive_time = millis();
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    newData = false;
  }
  //--------------Set Controller mode for RampUp--------------------
  c_mode = "P";

  //----------------Get Snapshot of current position----------------
  float curr_pos1 = Ax1toAngle(Enc1.read());
  float curr_pos2 = Ax2toAngle(Enc2.read());
  float curr_pos3 = Ax3toAngle(Enc3.read());

  //-----------Slope of the approximated linear function-------------
  float k_1;
  float k_2;
  float k_3;
  k_1 = (*target_pos[0] - curr_pos1) / RampUp_Duration;
  k_2 = (*target_pos[1] - curr_pos2) / RampUp_Duration;
  k_3 = (*target_pos[2] - curr_pos3) / RampUp_Duration;

  Serial.println(*target_pos[0] - curr_pos1);
  Serial.println(*target_pos[1] - curr_pos2);
  Serial.println(*target_pos[2] - curr_pos3);

  float end_pos1;
  float end_pos2;
  float end_pos3;
  end_pos1 = *target_pos[0];
  end_pos2 = *target_pos[1];
  end_pos3 = *target_pos[2];

  //------------------Enter the RampUp loop------------------------
  RampUp_StartTime = millis();
  while ((millis() - RampUp_StartTime) <= RampUp_Duration) {
    RampUp_time = millis() - RampUp_StartTime;

    //RampUp Function for Axis 1 - 3
    *target_pos[0] = k_1 * (float)RampUp_time + curr_pos1;
    *target_pos[1] = k_2 * (float)RampUp_time + curr_pos2;
    *target_pos[2] = k_3 * (float)RampUp_time + curr_pos3;

    *target_pos[0] = constrain(*target_pos[0], curr_pos1, end_pos1);
    *target_pos[1] = constrain(*target_pos[1], curr_pos2, end_pos2);
    *target_pos[2] = constrain(*target_pos[2], curr_pos3, end_pos3);

    /* Serial.print("target Axis1\t");
    Serial.println(*target_pos[0]);
    Serial.print("target Axis2\t");
    Serial.println(*target_pos[1]);
    Serial.print("target Axis3t\t");
    Serial.println(*target_pos[2]);*/

    //To-Do: The controller should be optimized for this rampup motion
    for (int i = 0; i < 3; i++) {
      PIDupdate(target_pos[i], i, c_mode);
    }
  }
  Serial.print("Final target Axis1\t");
  Serial.println(*target_pos[0]);
  Serial.print("Final target Axis2\t");
  Serial.println(*target_pos[1]);
  Serial.print("Final target Axis3\t");
  Serial.println(*target_pos[2]);
  
}