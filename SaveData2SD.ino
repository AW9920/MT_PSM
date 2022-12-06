void SaveData2SD(String data) {
  // Function variables
  int index = 0;
  //Update joint values
  q[0] = Ax1toAngle(counter1);
  q[1] = Ax2toAngle(counter2);
  q[2] = Ax3toAngle(counter3);

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