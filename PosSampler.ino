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
    Sampler_select = 1;
  } else if (mode == "Ramp") {
    Sampler_select = 2;
  } else {
    Sampler_select = 0;
  }  //DEFAULT

  switch (Sampler_select) {
    case 0:
      startRec = true;
      if ((millis() - stepResponsetimer) >= 3000 && (millis() - stepResponsetimer) < 6000) {
        *target_pos[0] = 2, *target_pos[1] = 2, *target_pos[2] = 100;
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
        startRec = true;
      } else if ((millis() - stepResponsetimer) >= 3000 && (millis() - stepResponsetimer) < 8000) {
        if (first_sample) {
          sampler_start_time = millis();
          t = 0.0;
          first_sample = false;
        } else {
          t = (millis() - sampler_start_time)/1e3;
        }
        y = mag * sin(2 * PI * speed * t);
        Serial.println(t);
        for (int i = 0; i < (sizeof(target_pos) / sizeof(target_pos[0])); i++) {
          if (i == 2) { y = 80 + y; }
          *target_pos[i] = y;
        }
      } else {
        for (int i = 0; i < (sizeof(target_pos) / sizeof(target_pos[0])); i++) {
          if (i != 2) {
            *target_pos[i] = 0;
          } else {
            *target_pos[i] = 80;
          }
        }
      }
      break;
    default:
      break;
  }
}