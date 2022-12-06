void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 4 || pin == 5 || pin == 6) {
    switch (divisor) {
      case 1: mode = 0b00000001; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 4) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else if (pin == 5) {
      TCCR3B = TCCR3B & 0b11111000 | mode;
    } else if (pin == 6) {
      TCCR4B = TCCR4B & 0b11111000 | mode;
    }
  }
}