/* DF Player mini command discovery (Modified for Particle world by @FiDel - Feb 16, 2016)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This program is meant to discover all the possibilities of the command structure of the DFPlayer mini.
No special libraries are needed.
It's very easy to understand and can be the basis for your own mp3 player sketch.
Note: Commands are not always correctly described in the manual. I tried to fix it, but there is still a lot to do. The commands recoverd so far are listed below.

Use of sketch: Enter three (separated) decimal numbers in the Serial Monitor with no end of line character.
First number : Command
Second number: First (High Byte) parameter
Third number : Second (Low Byte) parameter
E.g.: 3,0,1 will play the first track on the TF card

Very important for 5V Arduinos: Use serial 1K resistors or a level shifter between module RX and TX and Arduino to suppress noise
Connect Sound module board RX to Arduino pin 11 (via 1K resistor)
Connect Sound module board TX to Arduino pin 10 (via 1K resistor)
Connect Sound module board Vcc to Arduino Vin when powered via USB (preferably 3.0) else use seperate 5V power supply
Connect Sound module board GND to Arduino GND

General DF Player mini command structure (only byte 3, 5 and 6 to be entered in the serial monitor):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Byte Function Value
==== ================ ====
(0) Start Byte 0x7E
(1) Version Info 0xFF (don't know why it's called Version Info)
(2) Number of bytes 0x06 (Always 6 bytes)
(3) Command 0x__
(4) Command feedback 0x__ If enabled returns info with command 0x41 [0x01: info, 0x00: no info]
(5) Parameter 1 [DH] 0x__
(6) Parameter 2 [DL] 0x__
(7) Checksum high 0x__ See explanation below. Is calculated in function: execute_CMD
(8) Checksum low 0x__ See explanation below. Is calculated in function: execute_CMD
(9) End command 0xEF

Checksum calculation.
~~~~~~~~~~~~~~~~~~~~
Checksum = -Sum(byte(1..6)) (2 bytes, notice minus sign!)

Commands without returned parameters (*=Confirmed command ?=Unknown, not clear or not validated)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
CMD CMD
HEX Dec Function Description Parameters(2 x 8 bit)
==== === =================================== ========================================================================
0x01 1 Next * [DH]=X, [DL]=X Next file in current folder.Loops when last file played
0x02 2 Previous * [DH]=X, [DL]=X Previous file in current folder.Loops when last file played
0x03 3 Specify track(NUM) * [DH]=highByte(NUM), [DL]=lowByte(NUM)
1~2999 Playing order is order in which the numbers are stored.
Filename and foldername are arbitrary, but when named starting with
an increasing number and in one folder, files are played in
that order and with correct track number.
e.g. 0001-Joe Jackson.mp3...0348-Lets dance.mp3)
0x04 4 Increase volume * [DH]=X, [DL]=X Increase volume by 1
0x05 5 Decrease volume * [DH]=X, [DL]=X Decrease volume by 1
0x06 6 Specify volume * [DH]=X, [DL]= Volume (0-0x30) Default=0x30
0x07 7 Specify Equalizer * [DH]=X, [DL]= EQ(0/1/2/3/4/5) [Normal/Pop/Rock/Jazz/Classic/Base]
0x08 8 Specify repeat(NUM) * [DH]=highByte(NUM), [DL]=lowByte(NUM).Repeat the specified track number
0x09 9 Specify playback source (Datasheet) ? [DH]=X, [DL]= (0/1/2/3/4)Unknown. Seems to be overrided by automatic detection
(Datasheet: U/TF/AUX/SLEEP/FLASH)
0x0A 10 Enter into standby – low power loss * [DH]=X, [DL]=X Works, but no command found yet to end standby
(insert TF-card again will end standby mode)
0x0B 11 Normal working (Datasheet) ? Unknown. No error code, but no function found
0x0C 12 Reset module * [DH]=X, [DL]=X Resets all (Track = 0x01, Volume = 0x30)
Will return 0x3F initialization parameter (0x02 for TF-card)
"Clap" sound after excecuting command (no solution found)
0x0D 13 Play * [DH]=X, [DL]=X Play current selected track
0x0E 14 Pause * [DH]=X, [DL]=X Pause track
0x0F 15 Specify folder and file to playback * [DH]=Folder, [DL]=File
Important: Folders must be named 01~99, files must be named 001~255
0x10 16 Volume adjust set (Datasheet) ? Unknown. No error code. Does not change the volume gain.
0x11 17 Loop play * [DH]=X, [DL]=(0x01:play, 0x00:stop play)
Loop play all the tracks. Start at track 1.
0x12 18 Play mp3 file [NUM] in mp3 folder * [DH]=highByte(NUM), [DL]=lowByte(NUM)
Play mp3 file in folder named mp3 in your TF-card. File format exact
4-digit number (0001~2999) e.g. 0235.mp3
0x13 19 Unknown ? Unknown: Returns error code 0x07
0x14 20 Unknown ? Unknown: Returns error code 0x06
0x15 21 Unknown ? Unknown: Returns no error code, but no function found 
0x16 22 Stop * [DH]=X, [DL]=X, Stop playing current track
0x17 23 Loop Folder 01 * [DH]=x, [DL]=1~255, Loops all tracks in folder named "01"
0x18 24 Random play * [DH]=X, [DL]=X Random all tracks, always starts at track 1
0x19 25 Single loop * [DH]=0, [DL]=0 Loops the track that is playing
0x1A 26 Pause * [DH]=X, [DL]=(0x01:pause, 0x00:stop pause)

Commands with returned parameters (*=Confirmed command ?=Unknown, not clear or not validated)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
CMD CMD
HEX Dec Function Description Parameters(2 x 8 bit)
==== === =================================== ===========================================================================
0x3A 58 Medium inserted * [DH]=0, [DL]=(1:U-disk, 2:TF-card)
0x3B 59 Medium ejected * [DH]=0, [DL]=(1:U-disk, 2:TF-card)
0x3C 60 Finished track on U-disk * [DH]=highByte(NUM), [DL]=lowByte(NUM)
Not validated. Returns track number when song is finished on U-Disk
0x3D 61 Finished track on TF-card * [DH]=highByte(NUM), [DL]=lowByte(NUM)
Returns track number when song is finished on TF
0x3E 62 Finished track on Flash * [DH]=highByte(NUM), [DL]=lowByte(NUM)
Not validated. Returns track number when song is finished on Flash
0x3F 63 Initialization parameters * [DH]=0, [DL]= 0 ~ 0x0F. Returned code when Reset (0x12) is used.
(each bit represent one device of the low-four bits)
See Datasheet. 0x02 is TF-card. Error 0x01 when no medium is inserted.
0x40 64 Error ? [DH]=0, [DL]= 0~7 Error code(Returned codes not yet analyzed)
0x41 65 Reply ? [DH]=0, [DL]= 0~? Return code when command feedback is high
0x00 no Error (Other returned code not known)
0x42 66 The current status * [DH] = Device number [DL] = 0 no play, 1 play
0x43 67 The current volume * [DH]=0, [DL]= Volume (0-30)
0x44 68 The current EQ * [DH]=0, [DL]= EQ(0/1/2/3/4/5) [Normal/Pop/Rock/Jazz/Classic/Base]
0x45 69 The current playback mode * [DH]=0, [DL]= (0x00: no CMD 0x08 used, 0x02: CMD 0x08 used, not usefull)
0x46 70 The current software version * [DH]=0, [DL]= Software version. (My version is 5)
0x47 71 The total number of U-disk files * [DH]=highByte(NUM), [DL]=lowByte(NUM). Not validated
0x48 72 The total number of TF-card files * [DH]=highByte(NUM), [DL]=lowByte(NUM)
0x49 73 The total number of flash files * [DH]=highByte(NUM), [DL]=lowByte(NUM). Not validated
0x4A 74 Keep on (Datasheet) ? Unknown. No returned parameter
0x4B 75 The current track of U-Disk * [DH]=highByte(NUM), [DL]=lowByte(NUM), Current track on all media
0x4C 76 The current track of TF card * [DH]=highByte(NUM), [DL]=lowByte(NUM), Current track on all media
0x4D 77 The current track of Flash * [DH]=highByte(NUM), [DL]=lowByte(NUM), Current track on all media
0x4E 78 Folder "01" [DH]=x, [DL]=1 * [DH]=0, [DL]=(NUM) Change to first track in folder "01"
Returns number of files in folder "01"
0x4F 79 The total number of folders * [DH]=0, [DL]=(NUM), Total number of folders, including root directory

Additional info can be found on DFRobot site, but is not very reliable
Additional info:http://www.dfrobot.com/index.php?route=product/product&product_id=1121

Ype Brada 2015-04-06
*/
    
# define Start_Byte 0x7E
# define Version_Byte 0xFF
# define Command_Length 0x06
# define End_Byte 0xEF
# define Acknowledge 0x00 //Returns info with command 0x41 [0x01: info, 0x00: no info]

#include "SoftwareSerial.h"

SoftwareSerial mySoftwareSerial(10, 11);

void setup ()
{
 Serial.begin(9600);
 mySoftwareSerial.begin(9600);

 Serial.println("Started");
 
 //execute_CMD(0x3F, 0, 0); // Send request for initialization parameters

 //while (mySoftwareSerial.available()<10) // Wait until initialization parameters are received (10 bytes)
 delay(30); // Pretty long delays between succesive commands needed (not always the same)

 // Initialize sound to very low volume. Adapt according used speaker and wanted volume
 execute_CMD(0x06, 0, 5); // Set the volume (0x00~0x30)
 //DFPlayer_send_cmd(6, 0, 11);
 Serial.println("Ready");
}


void loop ()
{
 if (Serial.available())
 {
  // Input in the Serial monitor: Command and the two parameters in decimal numbers (NOT HEX)
  // E.g. 3,0,1 (or 3 0 1 or 3;0;1) will play first track on the TF-card
  byte Command = Serial.parseInt();
  byte Parameter1 = Serial.parseInt();
  byte Parameter2 = Serial.parseInt();
  
  // Write the input at the screen
  Serial.print("Command : 0x");if (Command < 16) Serial.print("0"); Serial.print(Command, HEX);
  Serial.print("("); Serial.print(Command, DEC);
  Serial.print("); Parameter: 0x");if (Parameter1 < 16) Serial.print("0");Serial.print(Parameter1, HEX);
  Serial.print("("); Serial.print(Parameter1, DEC);
  Serial.print("), 0x");if (Parameter2 < 16) Serial.print("0");Serial.print(Parameter2, HEX);
  Serial.print("("); Serial.print(Parameter2, DEC);Serial.println(")");

  // Excecute the entered command and parameters
  execute_CMD(Command, Parameter1, Parameter2);
 }

 if (mySoftwareSerial.available()>=10) // There is at least 1 returned message (10 bytes each)
 {
  // Read the returned code
  byte Returned[10];
  for (byte k=0; k<10; k++)
  Returned[k] = mySoftwareSerial.read();

  // Wtite the returned code to the screen
  Serial.print("Returned: 0x"); if (Returned[3] < 16) Serial.print("0"); Serial.print(Returned[3],HEX);
  Serial.print("("); Serial.print(Returned[3], DEC);
  Serial.print("); Parameter: 0x"); if (Returned[5] < 16) Serial.print("0"); Serial.print(Returned[5],HEX);
  Serial.print("("); mySoftwareSerial.print(Returned[5], DEC);
  Serial.print("), 0x"); if (Returned[6] < 16) Serial.print("0"); Serial.print(Returned[6],HEX);
  Serial.print("("); Serial.print(Returned[6], DEC); Serial.println(")");
 }
}



void execute_CMD(byte CMD, byte Par1, byte Par2) // Excecute the command and parameters
{
 // Calculate the checksum (2 bytes)
 int16_t checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + Par1 + Par2);

 // Build the command line
 byte Command_line[10] = { Start_Byte, Version_Byte, Command_Length, CMD, Acknowledge, Par1, Par2, checksum >> 8, checksum & 0xFF, End_Byte};

 //Send the command line to the module
 for (byte k=0; k<10; k++)
 {
  mySoftwareSerial.write( Command_line[k]);
 }
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
    mySoftwareSerial.write(cmd_line[i]);
}
