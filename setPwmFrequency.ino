void setPWMfrequency(void){
  float PWM_freq = 18500.0;
  analogWriteFrequency(DC1_PWM,PWM_freq);
  analogWriteFrequency(DC2_PWM,PWM_freq);
  analogWriteFrequency(DC3_PWM,PWM_freq);
}