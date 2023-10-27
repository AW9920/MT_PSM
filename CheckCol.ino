/*This function should check if the limites are reached during runtime. It is not used for this iteration of the PSM*/
void CheckCol(void){
  if (digitalRead(LS1_NC) == HIGH && digitalRead(LS1_NO) == LOW) {
    motor[0].setSpeed(0);
    motor[1].setSpeed(0);
    motor[2].setSpeed(0);
    while (true) {};
  }
  if (digitalRead(LS2_NC) == HIGH && digitalRead(LS2_NO) == LOW) {
    motor[0].setSpeed(0);
    motor[1].setSpeed(0);
    motor[2].setSpeed(0);
    while (true) {};
  }
  if (digitalRead(LS3_NC) == HIGH && digitalRead(LS3_NO) == LOW) {
    motor[0].setSpeed(0);
    motor[1].setSpeed(0);
    motor[2].setSpeed(0);
    while (true) {};
  }
}