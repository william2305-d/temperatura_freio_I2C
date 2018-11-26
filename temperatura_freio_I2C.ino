#include <i2cmaster.h>
#include <SPI.h>
#include "mcp_can.h"

#define PESO 100 // no stm vai ser necessário multiplicar o valor por 0,01

int direccionSensor1 = 0x50<<1;   // Dirección del sensor 1 
int direccionSensor2 = 0x55<<1;   // Dirección del sensor 2

float celcius1 = 0;   // Guardara temperatura 1 en Celcius
float celcius2 = 0;   // Guardara temperatura 2 en Celcius

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

//ID para comunicação com a CAN (0x18100000 >> 19) == 0x302
const unsigned long int PRIVATE_ID = 0x18100000;

unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
uint8_t stmp[2] = {0, 0xAD};

void setup()
{
  Serial.begin(115200);                       // Comunicación serial en 9600bps.
  i2c_init();                               // Inicializar  i2c bus.
  PORTC = (1 << PORTC4) | (1 << PORTC5);    // Pullups.

/*Init CAN Bus*/

    while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_8MHz))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}

uint8_t data[6] = {0, 0, 0, 0, 0, 0};

void buildDataFrame(){
  int temp1 = celcius1 * PESO;
  int temp2 = celcius2 * PESO;
  data[5] = temp2 & 0xff;
  data[4] = (temp2>>8) & 0xff;
  data[3] = 0x24;
  data[2] = temp1 & 0xff;
  data[1] = (temp1>>8) & 0xff;
  data[0] = 0x23;
}

void loop()
{
  int i;
  celcius1 = temperaturaCelcius(direccionSensor1);   //Lee la temperatura para cada sensor y
  celcius2 = temperaturaCelcius(direccionSensor2);   //lo guarda en la respectiva variable

  buildDataFrame();

  Serial.print("Valor separado: ");
  for(i = 0; i< 6; i++){
    Serial.print(data[i]);
    Serial.print(" ");  
  }          
  Serial.println();   

  CAN.sendMsgBuf(0x18100000, 1, 6, data);    
  /*fahrenheit1 = (celcius1*1.8) + 32;     // Sidesean convertir en Fahrenheit 
  fahrenheit2 = (celcius2*1.8) + 32;*/

  Serial.print("Sensor 1: ");    
  Serial.print(celcius1);       //Imprimir valor 1         
  Serial.print("ºC");           
  Serial.println();             //nueva linea

  Serial.print("Sensor 2: ");
  Serial.print(celcius2);       //Imprimir valor 2
  Serial.print("ºC");
  Serial.println();
  Serial.println();
  
  delay(200);                         // Espera 200ms para tomar otro par de lecturas
}

//Función con comentarios originales de http://wiki.wiring.co
float temperaturaCelcius(int address) {
  int dev = address;
  int data_low = 0;
  int data_high = 0;
  int pec = 0;
  unsigned char retorno;

  retorno = i2c_readAck();
  Serial.println(retorno);
  if((i2c_start(dev+I2C_WRITE)) == 1){
    Serial.println("Dispositivo não encontrado");
    return 0;
    Serial.println("nao encontrou");
  }
  else{
    Serial.println("encontrou");
    // Write
    i2c_start_wait(dev+I2C_WRITE);
    i2c_write(0x07);

    // Read
    i2c_rep_start(dev+I2C_READ);
    data_low = i2c_readAck();       // Read 1 byte and then send ack.
    data_high = i2c_readAck();      // Read 1 byte and then send ack.
    pec = i2c_readNak();
    i2c_stop();

    // This converts high and low bytes together and processes temperature, 
    // MSB is a error bit and is ignored for temps.
    double tempFactor = 0.02;       // 0.02 degrees per LSB (measurement 
                                  // resolution of the MLX90614).
    double tempData = 0x0000;       // Zero out the data
    int frac;                       // Data past the decimal point

    // This masks off the error bit of the high byte, then moves it left 
    // 8 bits and adds the low byte.
    tempData = (double)(((data_high & 0x007F) << 8) + data_low);
    tempData = (tempData * tempFactor)-0.01;
    float celcius = tempData - 273.15;
  
    // Returns temperature un Celcius.
    return celcius;
  }
}
