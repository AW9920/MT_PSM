/*
Allows to set the PWM frequency of the generated control signal.

*/
void setPWMfrequency(void){
  float PWM_freq = 14500.0;
  analogWriteFrequency(DC1_PWM,PWM_freq);
  analogWriteFrequency(DC2_PWM,PWM_freq);
  analogWriteFrequency(DC3_PWM,PWM_freq);
}