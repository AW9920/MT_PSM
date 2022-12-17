void LPFilter_DT(void) {
  // Filters the signal of delta time with cut off frequency of 4Hz

  // Function variables
  double dt_yn;

  // Filter routine
  if (!first) {
    dt_yn = 0.956 * dt_yn1 + 0.0245 * dt + 0.0245 * dt_xn1;  //Cut-off frequency 4Hz
    dt_yn1 = dt_yn;
    dt_xn1 = dt;
    dt = dt_yn;
  } else {
    dt_yn1 = dt;
    dt_xn1 = dt;
    first = false;
  }
  return;
}