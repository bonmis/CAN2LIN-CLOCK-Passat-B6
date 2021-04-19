#********************************************
#* Author: (c) bonmis
#* contact: bonmis@go2.pl
#********************************************

#include <SPI.h>
#include "mcp_can.h"
int can_hour;
int can_min;
int can_sec;
int can_read;

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);  // Set CS pin

#define CAN_delay 1  // definicja co ile ma się wykonywać program CAN
#define LIN_delay 1 // definicja co ile ma się przesuwać wskazówka zegara (program CLOCK)

unsigned long now = 0;
unsigned long last;
unsigned long next_change_can;
unsigned long next_change_lin;

#define linspeed  19200
unsigned long Tbit = 1000000 / linspeed;
#define breakfield  13
#define breakdelimiter  1
#define responsedelay 8
int responsespace = responsedelay * Tbit;
#define numbers  8
#define interbytedelay  0
int interbytespace = interbytedelay * Tbit;
byte linb, sync, idbyte, PID, checksum;
byte message[numbers], sending[numbers];

void setup() 
{
  next_change_can = millis() + CAN_delay;
  next_change_lin = millis() + LIN_delay;
  
  //Serial.begin(115200, SERIAL_8N1); // port debug
  while (CAN_OK != CAN.begin(CAN_100KBPS))              // init CAN BUS : baudrate = 100k
  {
    //Serial.println("CAN BUS Shield init fail");
    //Serial.println(" Init CAN BUS Shield again");
    delay(500);
  }
  //Serial.println("CAN BUS Shield init OK");  

  Lininit();
  delay(1000);
  pinMode(9, OUTPUT); //CS dla LIN-a
  digitalWrite(9, HIGH); //LIN_CS Wlaczenie
}

byte illu = 0x00;
byte godzina = 0x00;
byte minuta = 0x00;
byte sekunda = 0x00;

bool isEvenMinute;
uint32_t bckl = 0x0D0000;
uint32_t hByte = 0x000000;
uint32_t mByte = 0x000000;
uint32_t sByte = 0x000000;

uint32_t calculateHour(int hour) 
{
  byte uHour = hour*0x04;
  byte hLow = uHour >> 4;
  byte hHigh = uHour << 4;
  uint32_t ret = (hHigh*0x10000)+(hLow*0x100);
  return ret;
}

uint16_t calculateMinute(int minute) 
{
  uint16_t ret;
  if(minute % 2) 
  {
    isEvenMinute = false;
    if(minute <= 31) 
    {
      int minT = (minute-1)/2;
      ret = minT*0x1000;
    } 
    else 
    {
      int minT = (minute-35)/2;
      ret = (minT*0x1000)+0x0001;
    }
  }
  else 
  {
    isEvenMinute = true;
    if(minute <= 30) 
    {
      int minT = minute/2;
      ret = minT*0x1000;
    } 
    else
    {
      int minT = (minute-32)/2;
      ret = (minT*0x1000)+0x0001;
    }
  }
  return ret;
}

uint8_t calculateSecond(int second) 
{
  uint8_t ret;
  if(isEvenMinute == false) 
  {
    ret = (second*0x02)+0x78;
  } 
  else 
  {
    ret = second*0x02;
  }
  return ret;
}

void loop() 
{
  last = now;
  now = millis();

  if( now >= next_change_can || now < last ) // sprawdzamy czy należy wykonać program CAN
  {
    next_change_can = now + CAN_delay;
    unsigned char len = 0;
    unsigned char buf_can[8];
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
      CAN.readMsgBuf(&len, buf_can);    // read data,  len: data length, buf: data buf
      unsigned int canId = CAN.getCanId();
      if(canId == 0x470) 
      {
        can_read = 1;
          illu = (buf_can[2]); //przepisanie wartości z CAN do zmiennej wysyłanej przez LIN 
      }
      if(canId == 0x65D) 
      {
        can_read = 1;
        bitWrite(can_hour,4,bitRead(buf_can[6],0)); 
        bitWrite(can_hour,3,bitRead(buf_can[5],7)); 
        bitWrite(can_hour,2,bitRead(buf_can[5],6)); 
        bitWrite(can_hour,1,bitRead(buf_can[5],5)); 
        bitWrite(can_hour,0,bitRead(buf_can[5],4)); 

        bitWrite(can_min,5,bitRead(buf_can[6],6)); 
        bitWrite(can_min,4,bitRead(buf_can[6],5)); 
        bitWrite(can_min,3,bitRead(buf_can[6],4)); 
        bitWrite(can_min,2,bitRead(buf_can[6],3)); 
        bitWrite(can_min,1,bitRead(buf_can[6],2)); 
        bitWrite(can_min,0,bitRead(buf_can[6],1)); 

        bitWrite(can_sec,5,bitRead(buf_can[7],4)); 
        bitWrite(can_sec,4,bitRead(buf_can[7],3)); 
        bitWrite(can_sec,3,bitRead(buf_can[7],2)); 
        bitWrite(can_sec,2,bitRead(buf_can[7],1)); 
        bitWrite(can_sec,1,bitRead(buf_can[7],0)); 
        bitWrite(can_sec,0,bitRead(buf_can[6],7));

        godzina = (buf_can[5]);
        minuta = (buf_can[6]);
        sekunda = (buf_can[7]);
      }
    }
  }

  if( now >= next_change_lin || now < last ) // sprawdzamy czy należy wykonać program zegara
  {
    next_change_lin = now + LIN_delay;
    if (can_read == 0)
    {
        
    }
    else
    {
      if (godzina == 0x00 && minuta == 0x00 && sekunda == 0xC0)
      {
        
      }
      else
      {
        LinSend();
        can_read = 0;
      }
    }

  }
}

void Lininit() 
{
  Serial.begin(linspeed, SERIAL_8N1);
  pinMode(0, INPUT_PULLUP);
}

void LinWritePid(byte PID) 
{
  Serial.end();
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
  delayMicroseconds(breakfield * Tbit);               //after Frame Error Tbit to Sync Field
  digitalWrite(1, HIGH);
  delayMicroseconds(breakdelimiter * Tbit);
  Serial.begin(linspeed, SERIAL_8N1);
  Serial.write(0x55);
  Serial.write(PID | PIDCRC(PID) << 6 ); //
  delayMicroseconds(responsespace);
}

void LinWriteDat(byte* ms) 
{
  for (int i = 0; i < 8; i++) 
  {
    Serial.write(ms[i]);
    if (interbytespace == 0) 
    {
      delayMicroseconds(1);
    } 
    else 
    {
      delayMicroseconds(interbytespace);
    }
  }
}

void LinWriteCrc(byte id, byte* msg) 
{
  uint16_t sum = id;
  for (int i = 0; i < numbers; i++) 
  {
    sum += msg[i];
  }
  while (sum >> 8)
  sum = (sum & 255) + (sum >> 8);
  Serial.write(~sum);
}

void LinWakeUp() 
{
  Serial.begin(linspeed, SERIAL_8N1);
  Serial.write(0x80);
}

void LinGoSleep() 
{
  byte sleep[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  LinWritePid(0x3C);
  LinWriteDat(sleep);
  LinWriteCrc(0x3C, 0x00);
}

void LinSend()
{
        LinWakeUp();
  
        //Przeliczamy poszczególne składowe na bajty...
        hByte=calculateHour(can_hour);
        mByte=calculateMinute(can_min);
        sByte=calculateSecond(can_sec);
        //... i je składamy do kupy
        uint32_t sendByte = hByte + mByte + sByte + bckl;

        //Rozbijamy wyliczoną wartość na poszczególne bajty...
        uint32_t buf[3];
        buf[0] = (byte) (sendByte >> 16) & 0xFF;
        buf[1] = (byte) (sendByte >> 8) & 0xFF;
        buf[2] = (byte) sendByte & 0xFF;

        //... i je wysyłamy
        byte msg1[8] = {illu, buf[0], buf[1], buf[2], 0xFF, 0xFF, 0xFF, 0xFF};
        LinWritePid(0x73); // Break, Sync, ID wiadomości
        LinWriteDat(msg1); // Treść wiadomości
        LinWriteCrc(0x73, msg1); // Wyliczanie i wysyłanie CRC
    
    delayMicroseconds(250);

}

int PIDCRC(int PID) 
{
  int P0 = ((PID >> 0) + (PID >> 1) + (PID >> 2) + (PID >> 4)) & 1;
  int P1 = ~((PID >> 1) + (PID >> 3) + (PID >> 4) + (PID >> 5)) & 1;
  return (P0 | (P1 << 1));
}
