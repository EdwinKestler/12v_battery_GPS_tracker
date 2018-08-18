#include <Arduino.h>
#include <TimeLib.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP085.h>

SoftwareSerial SerialGPS = SoftwareSerial(D5, D7);
TinyGPS gps;
unsigned long fix;
float latitud, longitud;

const int offset = -6;
time_t prevDisplay = 0;

#define gps_interval 60000
unsigned long gps_time;
//--//--

//--batt
volatile int AnalogInput = A0;
int raw = 0;
float V, Volts = 0.0;

#define Interval_Bat 15000
unsigned long TimeNow;
//--batt

//
Adafruit_BMP085 bmp;
#define intervalo_bmp 30000
unsigned long tiempo_bmp;
int temperature_bmp;
int altitude_bmp;
//

#include <Wire.h>
#define diferencia 5
#define intervalo 15000
String eje_de_movimiento;

byte Version[3];
int8_t x_data;
int8_t y_data;
int8_t z_data;

unsigned long tiempo_recorr;

int8_t x_old;
int8_t y_old;
int8_t z_old;
//-----------------------------------------------------------------
void setup() 
{
    pinMode (AnalogInput, INPUT);

    Serial.begin(9600);
    
    while (!Serial) ;

    SerialGPS.begin(9600);
    
    if (!bmp.begin()) 
    {
        Serial.println("BMP180 sensor not found");
        while (1){
        }
    }
    
    Wire.begin(D2,D1);    //Declare pins
    Wire.beginTransmission(0x0A); // address of the accelerometer 
     // low pass filter, range settings 
    Wire.write(0x20); 
    Wire.write(0x05); 
    Wire.endTransmission();
}
 
void AccelerometerInit() 
{ 
   Wire.beginTransmission(0x0A); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(0x04); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(0x0A,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    Version[0] = Wire.read(); // receive a byte as characte
  }  
  x_data=(int8_t)Version[0]>>2;
 
  Wire.beginTransmission(0x0A); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(0x06); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(0x0A,1);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    Version[1] = Wire.read(); // receive a byte as characte
  }  
  y_data=(int8_t)Version[1]>>2;
  
  Wire.beginTransmission(0x0A); // address of the accelerometer 
  // reset the accelerometer 
  Wire.write(0x08); // Y data
  Wire.endTransmission(); 
  Wire.requestFrom(0x0A,1);    // request 6 bytes from slave device #2
   while(Wire.available())    // slave may send less than requested
  { 
    Version[2] = Wire.read(); // receive a byte as characte
  }  
   z_data=(int8_t)Version[2]>>2; 
}
 
void fecha_gps()
{
    while (SerialGPS.available())
    {
        if (gps.encode(SerialGPS.read()))
        {
            unsigned long age;
            int Year;
            byte Month, Day, Hour, Minute, Second;
            gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
            if (age < 500)
            {
                // set the Time to the latest GPS reading
                setTime(Hour, Minute, Second, Day, Month, Year);
                adjustTime(offset * SECS_PER_HOUR);
            }
            gps.f_get_position(&latitud,&longitud,&fix);
        }
    }
}

void printDigits(int digits) 
{
    Serial.print(":");
    if(digits < 10)
    {
        Serial.print('0');
    }
    Serial.print(digits);
}

void digitalClockDisplay()
{
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(" ");
    Serial.print(month());
    Serial.print(" ");
    Serial.print(year()); 
    Serial.println(); 
}

bool Check_on_movemnet( String & Movimiento_en_eje){
    //Muestra valores (informacion) cuando el sensor se mueva
    Movimiento_en_eje = "";
    
    if(x_data >= x_old + diferencia)
    {
        x_old = x_data;
        Movimiento_en_eje = "X";
        return  true;
        //Serial.print("X=");
        //Serial.print(x_data);
        //Serial.println(" ");
    }
    if(y_data >= y_old + diferencia)
    {
        y_old = y_data;
        Movimiento_en_eje = "Y";
        return  true;
        //Serial.print("Y=");
        //Serial.print(y_data);
        //Serial.println(" ");
    }    
    if(z_data >= z_old + diferencia)
    {
        z_old = z_data;
        Movimiento_en_eje = "Z";
        return  true;
        //Serial.print("Z=");   
        //Serial.print(z_data);
        //Serial.println(" ");
    }    
    return false;
}

void loop() 
{
    if(millis() > TimeNow + Interval_Bat)
    {
        TimeNow = millis();
        raw = analogRead(AnalogInput);
        V = raw / 1023.0;
        Volts = V * 12.59;
        Serial.print(raw);
        Serial.print(":");
        Serial.println(Volts);
    }
    if (millis()> tiempo_bmp + intervalo_bmp)
    {
        tiempo_bmp = millis();
        temperature_bmp = bmp.readTemperature();
        altitude_bmp = bmp.readAltitude(101500);
        Serial.print("Temperature = ");
        Serial.print(temperature_bmp);
        Serial.println(" *C");
        Serial.print("Altitude = ");
        Serial.print(altitude_bmp);
        Serial.println(" meters");
        Serial.println();
    }
    AccelerometerInit();

    if( Check_on_movemnet(eje_de_movimiento) != false ){
        Serial.println("movimiento en : ");
        Serial.println(eje_de_movimiento);
    }

    //reinicia los valores, para que despues de cierto tiempo, los vuelva a mostrar, ya que sin esto, se necesitaria mas movimiento del sensor para poder mostrar nuevamente los valores
    if (millis()> tiempo_recorr + intervalo)
    {
        tiempo_recorr = millis();
        x_old = x_data;
        y_old = y_data;
        z_old = z_data;
    }
   
    fecha_gps();

    if(millis() >gps_time + gps_interval)
    {
        gps_time = millis();
        Serial.println("Ejecutando_gps");
        Serial.print("latitud: ");
        Serial.print(latitud);
        Serial.print(" longitud: ");
        Serial.print(longitud);
        Serial.print(" fix: ");
        Serial.println(fix);
        
        if (timeStatus()!= timeNotSet)
        {
            if (now() != prevDisplay)
            {
                prevDisplay = now();
                digitalClockDisplay();
            }
        }
    }
}