
void setup ()
{
  Serial.begin(9600);
  //mySoftwareSerial.begin(9600);

  // уровень звука
  DFPlayer_send_cmd(0x06, 0, 20);
  delay(100);
  DFPlayer_send_cmd(0x03, 0, 2);
}  
  
void loop() 
{
}


// функция отправки команды в DFPLayer MP3
void DFPlayer_send_cmd(byte cmd, byte prm1, byte prm2) 
{
  // вычисляем контрольную сумму (2 байта)
  int16_t checksum = -(0xFF + 0x06 + cmd + 0x00 + prm1 + prm2);

  // получаем строку команды
  byte cmd_line[10] = {0x7E, 0xFF, 0x06, cmd, 0x00, prm1, prm2, checksum >> 8, checksum & 0xFF, 0xEF};

  // отправляем в модуль
  for (byte i=0; i<10; i++)
    Serial.write(cmd_line[i]);
}
