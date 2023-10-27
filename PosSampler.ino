/*
This function is implemented with the purpose to generate step responses for system identification
It features a number of modes.
Input Arguments:
  -mode: Select the type of simulated target values, which includes:
      -Step: step input
      -Sine: sine signal of target value
      -Ramp: No code implemented to simulate a ramp (delete or implement)
      -MTM: this is the MTM mode, Signal is generated with MTM and sampled form SD card. Not useable anymore
The individual Signals can be changed in frequency and magnitude.
*/

void PosSampler(String mode) {
  //Function variables
  int Sampler_select;
  double speed = 0.5;  //Hz
  double y;
  double mag = 2;
  double t;
  bool t_set = false;

  if (mode == "Step") {
    Sampler_select = 0;
  } else if (mode == "Sine") {
    startRec = true;
    Sampler_select = 1;
  } else if (mode == "Ramp") {
    Sampler_select = 2;
  } else if (mode == "MTM") {
    Sampler_select = 3;
  } else {
    Sampler_select = 0;
  }  //DEFAULT

  switch (Sampler_select) {
    case 0:
      if ((millis() - stepResponsetimer) >= 3000 && (millis() - stepResponsetimer) < 6000) {
        startRec = true;
        c_mode = "PID";
        *target_pos[0] = 1, *target_pos[1] = 1, *target_pos[2] = 81;
        //Serial.println('Stage 1');
      } else if ((millis() - stepResponsetimer) >= 6000 && (millis() - stepResponsetimer) < 9000) {
        *target_pos[0] = -2, *target_pos[1] = -2, *target_pos[2] = 80;
        //Serial.println('Stage 2');
      } else if ((millis() - stepResponsetimer) >= 9000 && (millis() - stepResponsetimer) < 12000) {
        *target_pos[0] = 0, *target_pos[1] = 0, *target_pos[2] = 70;
        //Serial.println('Stage 3');
      } else {
        *target_pos[0] = 0, *target_pos[1] = 0, *target_pos[2] = 80;
        //Serial.println('Stage 4');
      }
      break;

    case 1:
      if ((millis() - stepResponsetimer) < 3000) {
        sampler_setup_flag = true;
      } else if ((millis() - stepResponsetimer) >= 3000 && (millis() - stepResponsetimer) < 8000) {
        sampler_setup_flag = false;
        c_mode = "PID";
        if (first_sample) {
          sampler_start_time = millis();
          t = 0.0;
          first_sample = false;
        } else {
          t = (millis() - sampler_start_time) / 1e3;
        }
        y = mag * sin(2 * PI * speed * t);
        for (int i = 0; i < (sizeof(target_pos) / sizeof(target_pos[0])); i++) {
          if (i == 2) { y = 80 + y; }
          *target_pos[i] = y;
        }
      }

    case 2:
      break;

    case 3:
      if ((millis() - stepResponsetimer) < 2000) {
        startRec = true;
        if (setupFlag) {
          if (dataFile) {  //Get starting position
            // read from the file until there's nothing else in it:
            if (dataFile.available()) {
              memset(line, '\0', sizeof(line));
              dataFile.readBytesUntil('\n', line, sizeof(line) - 1);
              char* strtokIndx;  // this is used by strtok() as an index

              strtokIndx = strtok(line, ",");     // get the first part - the string
              *target_pos[0] = atof(strtokIndx);  // convert this part to a float

              strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
              *target_pos[1] = atof(strtokIndx);

              strtokIndx = strtok(NULL, ",");
              *target_pos[2] = atof(strtokIndx);
            }
          }
          setupFlag = false;
        }
        SD_Timer = millis();
      }
      else if(((millis() - stepResponsetimer) > 2000) && ((millis() - stepResponsetimer) < 4000)){c_mode = "PID";} 
      else if (((millis() - stepResponsetimer) > 4000)) {//((millis() - SD_Timer) >= SD_sampT)
        c_mode = "PID";
        if (dataFile) {
          // read from the file until there's nothing else in it:
          if (dataFile.available()) {
            memset(line, '\0', sizeof(line));
            dataFile.readBytesUntil('\n', line, sizeof(line) - 1);
            char* strtokIndx;  // this is used by strtok() as an index

            strtokIndx = strtok(line, ",");     // get the first part - the string
            *target_pos[0] = atof(strtokIndx);  // convert this part to a float

            strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
            *target_pos[1] = atof(strtokIndx);

            strtokIndx = strtok(NULL, ",");
            *target_pos[2] = atof(strtokIndx);
          } else {
            // close the file:
            dataFile.close();
          }
        }
        SD_Timer = millis();
      }
      break;
  }
}