#include <i2cmaster.h>

int direccionSensor1 = 0x50<<1;   // Dirección del sensor 1 
int direccionSensor2 = 0x55<<1;   // Dirección del sensor 2

float celcius1 = 0;   // Guardara temperatura 1 en Celcius
float celcius2 = 0;   // Guardara temperatura 2 en Celcius

void setup()
{
  Serial.begin(9600);                       // Comunicación serial en 9600bps.
  i2c_init();                               // Inicializar  i2c bus.
  PORTC = (1 << PORTC4) | (1 << PORTC5);    // Pullups.
}

void loop()
{
  celcius1 = temperaturaCelcius(direccionSensor1);   //Lee la temperatura para cada sensor y
  celcius2 = temperaturaCelcius(direccionSensor2);   //lo guarda en la respectiva variable
  
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

  delay(1000);                         // Espera un segundo para tomar otro par de lecturas
}

//Función con comentarios originales de http://wiki.wiring.co
float temperaturaCelcius(int address) {
  int dev = address;
  int data_low = 0;
  int data_high = 0;
  int pec = 0;

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
