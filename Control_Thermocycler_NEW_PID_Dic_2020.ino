#include <Wire.h>  //Librería I2C de Arduino
//#include <SoftwareSerial.h>
#define addresslux 0x10 //Dirección I2C de 7bit del Sensor VEML6030
#define addresstemp 0x5b  //Direccion I2C de sensor de temperatura MLX90615 cabezal 1: 0x5b, cabezal 2: 0x5f
#include <PID_v1.h>
#include <avr/wdt.h>
String aux="";            //variable funcion formato
int cont=0,  contLED=0;
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
int datastring = 0;
//SoftwareSerial mySerial(16, 17); // RX, TX (A4, A5)
//=======Variables de configuracion de los ciclos======================//
  int tprev=20;       //Tiempo en seg de desnaturalizacion inicial
  int cont60=0, contci=0, contci2=0, cont95=0;        //Contadores No mover
  int dur60=15;       //Tiempo en seg de duracion de 60°C
  int dur95=15;       //Tiempo en seg de duracion de 95°C
  int intert=10, tdesga=100;       //Tiempo en seg de duracion intermedia
  int ciclo_pausa=0, numero_ciclos=40, Numero_ciclos=40, prevtemp=95, annetemp=60, desnutemp=95, intertemp=75, desgatemp=30;
  int lectciclos=0;       //Numero que define cada cuantos ciclos se realiza una lectura

  int xt, zt;   //Definimos variables enteras
  float PIDOUT, sensor1, sensor2, sensor3, sensor4, yt;   //Variable de punto flotante
  int TempFAN, TempFAN0, TempFAN1, TempFAN2, TempFAN3, TempFAN4, TempFAN5, TempFAN6, contventil=0;
  double xl1, zl1, xl2, zl2, xl3, zl3, xl4, zl4;
//////////////////////////////////////////////////////////
//==============CONFIGURACIONES======================/////
//////////////////////////////////////////////////////////
////////////////PID///////////////////////
double kp=8, ki=0.0001, kd=8000, set_temp, PIDinput, PIDoutput;
//PID myPID(&PIDinput, &PIDoutput, &set_temp,6,0.2,4, DIRECT);
PID myPID(&PIDinput, &PIDoutput, &set_temp,6,0.15,6,P_ON_E, DIRECT);
//PID myPID(&PIDinput, &PIDoutput, &set_temp,6,0.2,4, DIRECT);
/////////////////////////////////////////
void setup() {
  wdt_disable(); // Desactivar el watchdog mientras se configura, para que no se resetee
  Serial.begin(9600); // para mostrar las lecturas en puerto serie
  Wire.begin(); // iniciamos bus I2C
  inputString.reserve(200);
set_temp=30;      //temperatura deseada, cambiar en cualquier momento con una interrupcion de puerto serial.


  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,100);
pinMode(2, OUTPUT);  //D2 activación foto sensor 1
pinMode(3, OUTPUT);   //D3 activación foto sensor 2
pinMode(4, OUTPUT);   //D4 activación foto sensor 3
pinMode(5, OUTPUT);   //D5 activación foto sensor 4
pinMode(10, OUTPUT);   //D10 control led de excitación
pinMode(12,OUTPUT);    //Vacuum PUMP
pinMode(11, OUTPUT);  //D11 control peltier
pinMode(9, OUTPUT);  //D21 control peltier
pinMode(13, OUTPUT);  //LED
pinMode(6, OUTPUT);  //Salida de control del ventilador
//pinMode(A6, INPUT);   //Entrada sensor disipador


digitalWrite(2, HIGH);              //sensores se desactivan con un 1
digitalWrite(3, HIGH);
digitalWrite(4, HIGH);
digitalWrite(5, HIGH);
digitalWrite(10, LOW);
digitalWrite(12, LOW);
digitalWrite(11, HIGH);
digitalWrite(9, HIGH);
digitalWrite(A1, LOW);

// Config Sensores #1, 2, 3, 4

 // Escritura de los datos configuracion de todos los foto sensores (dos registros por eje).
 digitalWrite(5, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x41);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(5, HIGH);              //sensores se desactivan con un 1
delay(10);
 digitalWrite(3, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x41);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(3, HIGH);              //sensores se desactivan con un 1
delay(10);
 digitalWrite(2, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x41);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(2, HIGH);              //sensores se desactivan con un 1
delay(10);
 digitalWrite(4, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x41);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(4, HIGH);              //sensores se desactivan con un 1
//////////////////////////PID///////////////////////////
//   Input = readtemp();
//   Setpoint = 60;

////////////////////////////////////////////////////////
  // set the data rate for the SoftwareSerial port
  //mySerial.begin(9600);
 // mySerial.println("Hello, world?");


delay (1000);                       // tiempo para que el sensor obtenga lectura
wdt_enable(WDTO_500MS); // Configurar watchdog a 500 mili segundos
}
//////////////////////////////////////////////////////////
//==============FUNCIONES============================/////
//////////////////////////////////////////////////////////
//==================================================
//Lectura puero serie
//===============================================


void serialEvent() {
    while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == 'S') {
       stringComplete = true;
      }
      
  }
}

//=========================================
// Control de temperatura.
//=========================================

double controltemp(){
double RTemp, PIDout, OUT, OUT2;

//CONTROL 0-100%              ///////PID///////////////
PIDinput = readtemp();         //Leer temperatura del sensor

myPID.Compute();
PIDout = PIDoutput;



//Conversion de valores 0-100 a salidas del peltier

if (PIDout <=50){             //El PID requiere Enfriar
  OUT = (50-PIDout)*5.1;
  OUT = 255-OUT;
  analogWrite(11,OUT);
  analogWrite(9,255);
}
if (PIDout >=51){           //El PID requiere Calentar
  OUT2 = PIDout-51;
  OUT2 = OUT2*5.1;
  OUT2 = 255-OUT2;
  analogWrite(9,OUT2);
  analogWrite(11,255);

}

return PIDout;
}
  




//=============================================================================
// Lectura de los datos de temperatura sensor (dos registros por eje).
//===========================================================================
float readtemp(){
  int xl, zl, xt, zt;
  float yt;
  Wire.beginTransmission(addresstemp);   //Trasmitir comando de lectura
  Wire.write(0x27);                  //Lectura de RAM en localidad 0x07 (temp de obj)
  Wire.endTransmission(0);           //finaliza transmision del comando sin cerrar comunicacion (0)


  Wire.requestFrom(addresstemp, 2);     //Captura de los datos de RAM
  if (Wire.available() >= 2) {      //Datos son de dos bytes
    xt = Wire.read();                  // Byte menos significativo en x
    zt = Wire.read();                  // Byte mas significativo en z
  }



//calculo de la temperatura
yt = xt + (zt * 256);  //sumatoria de los bytes en y
yt = (yt * 0.02) - 273.15; //escala de temperatura por fabricante

return yt;      //retorno de la temperatura
}
///======================================
///=====FORMATO STRING===================
///======================================
String FORMATO(float x){
  x*=100;
  int y=(int)x;
  String v=(String)y; 
  
  switch(v.length()){
    case 1:
      aux=v;
      v="00000"+aux;
      break;
    case 2:
      aux=v;
      v="0000"+aux;
      break;
    case 3:
      aux=v;
      v="000"+aux;
      break;
     case 4:
      aux=v;
      v="00"+aux;
      break;
     case 5:
      aux=v;
      v="0"+aux;
      break;
    }
return v;
 }



//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//=================PROGRAMA PRINCIPAL===========================//
//////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
void loop() {
//===================Lectura del vacio=============================//
int vaciom=(analogRead(A7)-1024)*(-0.1);


//===================Control del vantilador========================//
analogWrite(6,10);                //salida de control del ventilador
//TempFAN = analogRead(A6);
TempFAN5 = (5.0 * analogRead(A6)*100.0)/1023.0;
//float millivolts = (TempFAN / 1023.0) * 5000;
//  float celsius = millivolts / 10;
contventil=contventil+1;
if (contventil==5) contventil=0;
if (contventil==0) TempFAN0=TempFAN5;
if (contventil==1) TempFAN1=TempFAN5;
if (contventil==2) TempFAN2=TempFAN5;
if (contventil==3) TempFAN3=TempFAN5;
if (contventil==4) TempFAN4=TempFAN5;

TempFAN=(TempFAN0+TempFAN1+TempFAN2+TempFAN3+TempFAN4)/5;

if(TempFAN > 50){
  TempFAN6=((TempFAN-50)*25);
  if(TempFAN6>=255) TempFAN6=255;
  analogWrite(6,TempFAN6);
}


//========Control de temperatura PID============
PIDOUT = controltemp ();              // Control de temperatura
//=========================================


/////////////==========Control de los ciclos====================/////////////////

///=========Comando INICIO-PAUSA===============
//if(digitalRead(A1)==1) ciclo_pausa=1;     //Para pausar los ciclos se debe de poner ciclo_pausa=0
//else ciclo_pausa=0;                       //Para reanudar desde el punto de pausa ciclo_pausa=1

                                            
//======Comando STOP====================
//if(digitalRead(A1)==1){
//  contci2=0;
//  numero_ciclos=Numero_ciclos;                          //Para reiniciar los ciclos poner: contci2=0 y reinicir el contador de ciclos
//}


                                            
contci=contci+1;                           //CONTADOR PARA LOS CICLOS DE TEMPERATURA
if(contci==10){
  contci=0;
  //contci2=contci2+1;                      // Inclemento cada segundo
  if(set_temp!=29)contci2=contci2+1;
}

if(ciclo_pausa==1){
  digitalWrite(12,HIGH);                                        //BOMBA de VACÍO
  if(contci2<=tdesga) set_temp=desgatemp;
  if(contci2>tdesga) set_temp=prevtemp;
  if(contci2>tprev+tdesga)  set_temp=annetemp;
  if(contci2==tdesga+tprev+dur60+intert-2) cont=-10;                                  //Encender led despues de alineación
  if(contci2>tdesga+tprev+dur60) set_temp=intertemp;//set_temp=desnutemp;
  if(contci2>tdesga+tprev+dur60+intert) set_temp=desnutemp;
  if(contci2>tdesga+tprev+dur60+intert+dur95){
    contci2=tdesga+tprev+1;
    numero_ciclos=numero_ciclos-1;
    if(numero_ciclos==0){
      ciclo_pausa=0;
      contci2=0; 
       
    }
  }
}
else {
  set_temp=29;
  digitalWrite(12,LOW);
}

  


//==============control de lecturas y led de excitación=============
                              //CONTADOR PARA MANIPULAR LECTURA DE DATOS
if(contci==9) cont=cont+10;
cont=cont+1;
if(cont==1){

  analogWrite(10, 100);
  digitalWrite(13,HIGH);                  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// Activacion sensor 1
 digitalWrite(5, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x40);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(5, HIGH);              //sensores se desactivan con un 1
delay(1);
// Activacion sensor 2
 digitalWrite(3, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x40);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(3, HIGH);              //sensores se desactivan con un 1
delay(1);
// Activacion sensor 3
 digitalWrite(2, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x40);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(2, HIGH);              //sensores se desactivan con un 1
delay(1);
// Activacion sensor 4
 digitalWrite(4, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x40);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(4, HIGH);              //sensores se desactivan con un 1
delay(100);
}
if(cont==3){                                        //Lectura de sensores
// desactivacion sensor 1
 digitalWrite(5, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x41);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(5, HIGH);              //sensores se desactivan con un 1
delay(1);
// desactivacion sensor 2
 digitalWrite(3, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x41);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(3, HIGH);              //sensores se desactivan con un 1
delay(1);
// desactivacion sensor 3
 digitalWrite(2, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x41);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(2, HIGH);              //sensores se desactivan con un 1
delay(1);
// desactivacion sensor 4
 digitalWrite(4, LOW);
 Wire.beginTransmission(addresslux);   //Trasmitir comando de lectura
 Wire.write(0x00);                  //Acceso localidad de configuracion 0x00 (direccion i2c)
 Wire.write(0x41);                  //LSB       *(08)         
 Wire.write(0x00);                  //MSB       *(C0)
 //Wire.write(0xE6);                  //PEC calcular PEG en crccalc.com con 00 08 c0 separados por enters
 Wire.endTransmission(1);           //finaliza transmision del comando y cerrar comunicacion (1)
digitalWrite(4, HIGH);              //sensores se desactivan con un 1

  analogWrite(10, 00);
  digitalWrite(13,LOW);                  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


}


if(cont==4){ 
//******************************************************************************************************
//Lectura sensor #1
digitalWrite(5, LOW);                   //Activación sensor
Wire.beginTransmission(addresslux);     //Trasmitir comando de lectura
Wire.write(0x04);                       //Lectura de RAM en localidad 0x04 (ALS)
Wire.endTransmission(0);                //finaliza transmision del comando sin cerrar comunicacion (0)
Wire.requestFrom(addresslux, 2);        //Captura de los datos de RAM
  if (Wire.available() >= 2) {          //Datos son de dos bytes
    xl1 = Wire.read();                   // Byte menos significativo en x
    zl1 = Wire.read();                   // Byte mas significativo en z
  }
// calculo de intensidades en lux SEGUN FABRICANTE
sensor1=xl1+(zl1*256);                    //sumatoria de los bytes en y
sensor1=(sensor1*0.0036);               //escala de temperatura por fabricante
sensor1=sensor1*15;                      //prueba
//sensor1=(sensor1*1.4511)-8.7645;                //Ajuste de calibracion*****************************************
digitalWrite(5, HIGH);                  // desactiva sensor 1

delay(1);
//Sensor #2
digitalWrite(3, LOW);//Activación sensor
Wire.beginTransmission(addresslux);     //Trasmitir comando de lectura
Wire.write(0x04);                       //Lectura de RAM en localidad 0x04 (ALS)
Wire.endTransmission(0);                //finaliza transmision del comando sin cerrar comunicacion (0)
Wire.requestFrom(addresslux, 2);        //Captura de los datos de RAM
  if (Wire.available() >= 2) {          //Datos son de dos bytes
    xl2 = Wire.read();                   // Byte menos significativo en x
    zl2 = Wire.read();                   // Byte mas significativo en z
  }
// calculo de intensidades en lux
sensor2=xl2+(zl2*256);                    //sumatoria de los bytes en y
sensor2=(sensor2*0.0036);               //escala de temperatura por fabricante
sensor2=sensor2*15;                      //prueba
//sensor2=(sensor2*2.654)-8.6787;                //Ajuste de calibracion*****************************************
digitalWrite(3, HIGH);                  // desactiva sensor 2
delay(1);
//Sensor #3
digitalWrite(2, LOW);//Activación sensor
Wire.beginTransmission(addresslux);     //Trasmitir comando de lectura
Wire.write(0x04);                       //Lectura de RAM en localidad 0x04 (ALS)
Wire.endTransmission(0);                //finaliza transmision del comando sin cerrar comunicacion (0)
Wire.requestFrom(addresslux, 2);        //Captura de los datos de RAM
  if (Wire.available() >= 2) {          //Datos son de dos bytes
    xl3 = Wire.read();                   // Byte menos significativo en x
    zl3 = Wire.read();                   // Byte mas significativo en z
  }
// calculo de intensidades en lux
sensor3=xl3+(zl3*256);        //sumatoria de los bytes en y
sensor3=(sensor3*0.0036);  //escala de temperatura por fabricante
sensor3=sensor3*15;          //Prueba
//sensor3=(sensor3*1.9313)-14.021;                //Ajuste de calibracion*****************************************
digitalWrite(2, HIGH); // desactiva sensor 2
delay(1);
//Sensor #4
digitalWrite(4, LOW);
Wire.beginTransmission(addresslux);     //Trasmitir comando de lectura
Wire.write(0x04);                       //Lectura de RAM en localidad 0x04 (ALS)
Wire.endTransmission(0);                //finaliza transmision del comando sin cerrar comunicacion (0)
Wire.requestFrom(addresslux, 2);        //Captura de los datos de RAM
  if (Wire.available() >= 2) {          //Datos son de dos bytes
    xl4 = Wire.read();                   // Byte menos significativo en x
    zl4 = Wire.read();                   // Byte mas significativo en z
  }
// calculo de intensidades en lux
sensor4=xl4+(zl4*256);                    //sumatoria de los bytes en y
sensor4=(sensor4*0.0036);               //escala de temperatura por fabricante
sensor4=sensor4*15;                      //Prueba
//sensor4=(sensor4*1.2019)-9.2909;                //Ajuste de calibracion*****************************************
digitalWrite(4, HIGH);                  // desactiva sensor 2
}

///********************************************************************************

if(cont>=6) cont=5;                     // tiempo entre lecturas 1:100ms (5min)

//=========================Lectura de temperatura======================
yt=readtemp();

//========CARGA DATOS COMO CADENA DE CARACTERES===============================
//sensor1=10;
//sensor2=20;
//sensor3=30;
//sensor4=40;

String sup, inf;
int supint=0, infint=0;
//String Variables="A100100100100030040050060070080090100";
String Datos="A"+FORMATO(sensor1)+FORMATO(sensor2)+FORMATO(sensor3)+FORMATO(sensor4); //Carga datos de sensores
String Variables="A"+FORMATO(set_temp)+FORMATO(yt)+FORMATO(numero_ciclos)+FORMATO(vaciom)+FORMATO(sensor1)+FORMATO(sensor2)+FORMATO(sensor3)+FORMATO(sensor4); //Carga datos de sensores
//=========================================================================
//=Lectura puerto serie=====
//=================================
  if (stringComplete) {
    sup=inputString.charAt(8);    //Carga valores del string en sup e inf
    inf=inputString.charAt(9);
    inputString.setCharAt(8,'0'); //Valores a cero
    inputString.setCharAt(9,'0');

    if (inputString == "leerdat:00S"){//<<<<<<<<<<<<<<<<<<<<<<COMANDO PARA LEER INTENSIDADES
      Serial.println(Datos);
      supint=sup.toInt();
      infint=inf.toInt();
        if(supint==1) cont=0;      //RESET DE CONTADOR
    }
    if (inputString == "setcicl:00S"){//<<<<<<<<<<<<<<<<<<<<<<COMANDO PARA CONTROL DE CICLOS
      supint=sup.toInt();
      infint=inf.toInt();

        if (supint==1){
        contci2=0;
        numero_ciclos=Numero_ciclos;                          //Para reiniciar los ciclos poner: contci2=0 y reinicir el contador de ciclos
        } 
        ciclo_pausa=infint;
      
    }
    if (inputString == "leervar:00S"){//<<<<<<<<<<<<<<<<<<<<<<<COMANDO PARA LEER VARIABLES
      Serial.println(Variables);
    }
    if (inputString == "cnfcicl:00S"){
      supint=sup.toInt();
      infint=inf.toInt();  
      supint=(supint*10)+infint;
      Numero_ciclos=supint;
      numero_ciclos=Numero_ciclos;
      Serial.println(Numero_ciclos);    
    }

    if (inputString == "cnftini:00S"){
      supint=sup.toInt();
      infint=inf.toInt();  
      supint=(supint*10)+infint;
      tprev=supint;
      Serial.println(tprev);    
      tprev=tprev*10;
    }

    if (inputString == "cnfcini:00S"){
      supint=sup.toInt();
      infint=inf.toInt();  
      supint=(supint*10)+infint;
      prevtemp=supint;
      Serial.println(prevtemp);    
    }    

    if (inputString == "cnfTdes:00S"){
      supint=sup.toInt();
      infint=inf.toInt();  
      supint=(supint*10)+infint;
      dur95=supint;
      Serial.println(dur95);    
    }

    if (inputString == "cnfTali:00S"){
      supint=sup.toInt();
      infint=inf.toInt();  
      supint=(supint*10)+infint;
      dur60=supint;
      Serial.println(dur60);    
    }
    if (inputString == "cnfTemD:00S"){
      supint=sup.toInt();
      infint=inf.toInt();  
      supint=(supint*10)+infint;
      desnutemp=supint;
      Serial.println(desnutemp);    
    }   
    if (inputString == "cnfTemA:00S"){
      supint=sup.toInt();
      infint=inf.toInt();  
      supint=(supint*10)+infint;
      annetemp=supint;
      Serial.println(annetemp);    
    }  

    if (inputString == "cnfTemI:00S"){
      supint=sup.toInt();
      infint=inf.toInt();  
      supint=(supint*10)+infint;
      intertemp=supint;
      Serial.println(intertemp);    
    }
      if (inputString == "cnfTint:00S"){
      supint=sup.toInt();
      infint=inf.toInt();  
      supint=(supint*10)+infint;
      intert=supint;
      Serial.println(intert);    
    }
        if (inputString == "cnfTemV:00S"){
      supint=sup.toInt();
      infint=inf.toInt();  
      supint=(supint*10)+infint;
      desgatemp=supint;
      Serial.println(desgatemp);    
    }
      if (inputString == "cnfTvac:00S"){
      supint=sup.toInt();
      infint=inf.toInt();  
      supint=(supint*10)+infint;
      tdesga=supint;
      Serial.println(tdesga);
      tdesga=tdesga*10;    
    }
    inputString = "";
    stringComplete = false;
  }
delay(100);                  //Delay principal
wdt_reset(); // Actualizar el watchdog para que no produzca un reinicio
//if (mySerial.available()) {
//      myserialEvent();
//   }  

}
