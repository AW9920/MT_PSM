void parseData(void) {  // split the data into its parts

  char* strtokIndx;  // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");  // get the first part - the string
  target_pos1 = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
  target_pos2 = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  target_pos3 = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  target_pos4 = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  target_pos5 = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  target_pos6 = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  target_pos7 = atof(strtokIndx);
}