/* This function was used in the first revision of the PSM to record the system behaviour on an SD card for later analyzation.
The function served no purpose in the 2nd version of the PSM.
However it may be reinstated to record and process system data whenever required.
*/
void SaveData2SD(String data) {
  // Function variables
  int index = 0;
  //Update joint values
  q[0] = Ax1toAngle(Enc1.read());
  q[1] = Ax2toAngle(Enc2.read());
  q[2] = Ax3toAngle(Enc3.read());

  //Pack data string
  for (int i = 0; i < (sizeof(target_pos) / sizeof(target_pos[0]) + sizeof(target_pos) / sizeof(target_pos[0])); i++) {
    index = i - 3;
    if (i <= 2) {
      data += String(q[i]);
      data += ",";
    } else if (i > 2 && i < 5) {
      data += String(*target_pos[index]);
      data += ",";
    } else {
      data += String(*target_pos[index]);
    }
  }
  // Write data to SD card
  if (dataFile) {
    dataFile.println(data);
  }
}