#include <Wire.h>

#define LED_PIN 4

#define BME280_ADDRESS 0x76

// GPIO pin for I2C
#define I2C_SCL_PIN 14
#define I2C_SDA_PIN 16

// software I2C
class I2C {
private:
  void delay()
  {
  }

  // 初期化
  void init_i2c()
  {
    pinMode(I2C_SCL_PIN, INPUT_PULLUP); 
    pinMode(I2C_SDA_PIN, INPUT_PULLUP); 
  }

  void i2c_cl_0()
  {
    pinMode(I2C_SCL_PIN, OUTPUT); 
    digitalWrite(I2C_SCL_PIN, LOW);
  }

  void i2c_cl_1()
  {
    digitalWrite(I2C_SCL_PIN, HIGH);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP); 
  }

  void i2c_da_0()
  {
    pinMode(I2C_SDA_PIN, OUTPUT); 
    digitalWrite(I2C_SDA_PIN, LOW);
  }

  void i2c_da_1()
  {
    digitalWrite(I2C_SDA_PIN, HIGH);
    pinMode(I2C_SDA_PIN, INPUT_PULLUP); 
  }

  int i2c_get_da()
  {
    i2c_da_1();
    return digitalRead(I2C_SDA_PIN) ? 1 : 0;
  }

  // スタートコンディション
  void i2c_start()
  {
    i2c_da_0(); // SDA=0
    delay();
    i2c_cl_0(); // SCL=0
    delay();
  }

  // ストップコンディション
  void i2c_stop()
  {
    i2c_cl_1(); // SCL=1
    delay();
    i2c_da_1(); // SDA=1
    delay();
  }

  // リピーテッドスタートコンディション
  void i2c_repeat()
  {
    i2c_cl_1(); // SCL=1
    delay();
    i2c_da_0(); // SDA=0
    delay();
    i2c_cl_0(); // SCL=0
    delay();
  }

  // 1バイト送信
  bool i2c_write(int c)
  {
    int i;
    bool nack;

    delay();

    // 8ビット送信
    for (i = 0; i < 8; i++) {
      if (c & 0x80) {
        i2c_da_1(); // SCL=1
      } else {
        i2c_da_0(); // SCL=0
      }
      c <<= 1;
      delay();
      i2c_cl_1(); // SCL=1
      delay();
      i2c_cl_0(); // SCL=0
      delay();
    }

    i2c_da_1(); // SDA=1
    delay();

    i2c_cl_1(); // SCL=1
    delay();
    // NACKビットを受信
    nack = i2c_get_da();
    i2c_cl_0(); // SCL=0

    return nack;
  }

  // 1バイト受信
  int i2c_read(bool nack)
  {
    int i, c;

    i2c_da_1(); // SDA=1
    delay();

    c = 0;

    for (i = 0; i < 8; i++) {
      i2c_cl_1(); // SCL=1
      delay();
      c <<= 1;
      if (i2c_get_da()) { // SDAから1ビット受信
        c |= 1;
      }
      i2c_cl_0(); // SCL=0
      delay();
    }

    // NACKビットを送信
    if (nack) {
      i2c_da_1(); // SDA=1
    } else {
      i2c_da_0(); // SDA=0
    }
    delay();
    i2c_cl_1(); // SCL=1
    delay();
    i2c_cl_0(); // SCL=0
    delay();

    return c;
  }

  int address; // I2Cデバイスアドレス

public:
  I2C(int address)
    : address(address)
  {
    init_i2c();
  }

  // デバイスのレジスタに書き込む
  virtual void write(int reg, int data)
  {
    i2c_start();                   // スタート
    i2c_write(address << 1);       // デバイスアドレスを送信
    i2c_write(reg);                // レジスタ番号を送信
    i2c_write(data);               // データを送信
    i2c_stop();                    // ストップ
  }

  // デバイスのレジスタを読み取る
  virtual int read(int reg)
  {
    int data;
    i2c_start();                   // スタート
    i2c_write(address << 1);       // デバイスアドレスを送信
    i2c_write(reg);                // レジスタ番号を送信
    i2c_repeat();                  // リピーテッドスタートコンディション
    i2c_write((address << 1) | 1); // デバイスアドレスを送信（読み取りモード）
    data = i2c_read(true);         // データを受信
    i2c_stop();                    // 受信
    return data;
  }
};


I2C *wire;


unsigned long int hum_raw,temp_raw,pres_raw;
signed long int t_fine;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;

void readTrim()
{
  uint8_t data[32];

  for (int i = 0; i < 24; i++) {
    data[i] = wire->read(0x88 + i);
  }

  data[24] = wire->read(0xa1);

  for (int i = 0; i < 7; i++) {
    data[i + 25] = wire->read(0xe1 + i);
  }

  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11]<< 8) | data[10];
  dig_P4 = (data[13]<< 8) | data[12];
  dig_P5 = (data[15]<< 8) | data[14];
  dig_P6 = (data[17]<< 8) | data[16];
  dig_P7 = (data[19]<< 8) | data[18];
  dig_P8 = (data[21]<< 8) | data[20];
  dig_P9 = (data[23]<< 8) | data[22];
  dig_H1 = data[24];
  dig_H2 = (data[26]<< 8) | data[25];
  dig_H3 = data[27];
  dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
  dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
  dig_H6 = data[31];
}

void readData()
{
  int i = 0;
  uint32_t data[8];
  for (int i = 0; i < 8; i++) {
    data[i] = wire->read(0xf7 + i);
  }
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
  hum_raw  = (data[6] << 8) | data[7];
}


signed long int calibration_T(signed long int adc_T)
{
  signed long int var1, var2, T;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

unsigned long int calibration_P(signed long int adc_P)
{
  signed long int var1, var2;
  unsigned long int P;
  var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
  var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
  if (var1 == 0) {
    return 0;
  }
  P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
  if (P < 0x80000000) {
    P = (P << 1) / ((unsigned long int) var1);
  } else {
    P = (P / (unsigned long int)var1) * 2;
  }
  var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
  var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
  P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
  return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
  signed long int v_x1;

  v_x1 = (t_fine - ((signed long int)76800));
  v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
    ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
    (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
    ((signed long int) dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (unsigned long int)(v_x1 >> 12);
}






void setup()
{
  wire = new I2C(BME280_ADDRESS);
  
  uint8_t osrs_t = 1;             //Temperature oversampling x 1
  uint8_t osrs_p = 1;             //Pressure oversampling x 1
  uint8_t osrs_h = 1;             //Humidity oversampling x 1
  uint8_t mode = 3;               //Normal mode
  uint8_t t_sb = 5;               //Tstandby 1000ms
  uint8_t filter = 0;             //Filter off 
  uint8_t spi3w_en = 0;           //3-wire SPI Disable
  
  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg  = osrs_h;
  
  Serial.begin(115200);
  Wire.begin();
  
  wire->write(0xf2, ctrl_hum_reg);
  wire->write(0xf4, ctrl_meas_reg);
  wire->write(0xf5, config_reg);
  readTrim();                    //

  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  
  double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
  signed long int temp_cal;
  unsigned long int press_cal,hum_cal;
  
  readData();
  
  temp_cal = calibration_T(temp_raw);
  press_cal = calibration_P(pres_raw);
  hum_cal = calibration_H(hum_raw);
  temp_act = (double)temp_cal / 100.0;
  press_act = (double)press_cal / 100.0;
  hum_act = (double)hum_cal / 1024.0;
  Serial.print("TEMP : ");
  Serial.print(temp_act);
  Serial.print(" DegC  PRESS : ");
  Serial.print(press_act);
  Serial.print(" hPa  HUM : ");
  Serial.print(hum_act);
  Serial.println(" %");    
}
