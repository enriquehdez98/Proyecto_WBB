#include <SPI.h>  //Librería para comunicación con SD
#include "SD.h"   //Librería de SD
#include <Wire.h>   // Librería de I2C para RTC y OLED
#include <Adafruit_GFX.h>   //Librería general para pantallas
#include <Adafruit_SSD1306.h> //Librería para OLED
#include "RTClib.h"   //Librería para RTC


File myFile; //Crea instancia para direccionar datos a SD

#define sclk    17      // TX pin SCLK ADS1222 1 y 2, entrada de reloj de serie -D2
#define Dout1   32      //32 pin Dout ADS1222 1, DRDY/DOUT drdy-> nivel bajo datos a leer listos, salida de datos por flaco positivo de slck, MSB primero -D3
#define Dout2   4       // 4pin Dout ADS1222 2, DRDY/DOUT drdy-> nivel bajo datos a leer listos, salida de datos por flaco positivo de slck, MSB primero -D4
#define mux     16      // RX pin MUX ADS1222 1 y 2, selección de la entrada analógica 0->AINP1+ AINN1- 1->AINP2+ AINN2 -D5
#define Buzz    15//Pin de conexión al Buzzer (también es el del LED de la ESP32), en la tarjeta es el D2,PIN4
//Para la uSD se tienen los siguientes pines: CS(D5), SCK(D18), MOSI(D23), MISO(D19)

#define PULSADOR_wii 25    // Para Botón de WBB
#define LED_wii      33    // Para LED
#define ON_wii       2     // Enciende WBB

unsigned long sensor [4] = {0, 0, 0, 0}, sensor_cal[4] = {0, 0, 0, 0};
float calib [4] = {0.0, 0.0, 0.0, 0.0};  //Array para datos de calibración
float mV[4] ={0.0, 0.0, 0.0, 0.0};
unsigned long timeElapsed;  //Variable para conteo de tiempo de ejecución
unsigned long timeInit;  //Variable para conteo de tiempo inicial 
unsigned long timeFin;  //Variable para conteo de tiempo final
  uint32_t segs;
  int contador=0;



float sum_sensors=0.0;  //Suma de sensores para umbral
const float umbral=20.0;  //Umbral de suma de sensores
int flagUmbral=0;  //Bandera de que no se pasa el umbralLED_wii
int peaks=0;  //Contador de picos de umbral

char filenameOA[] = "/12345678_0205_OA.txt"; // DíaMesAño_HoraMin(01012022_0205) Archivo AO
char filenameOC[] = "/12345678_0205_OC.txt"; // DíaMesAño_HoraMin(01012022_0205) Archivo OC
char filenameOAF[] = "/12345678_0205_OAF.txt"; // DíaMesAño_HoraMin(01012022_0205) Archivo AO
char filenameOCF[] = "/12345678_0205_OCF.txt"; // DíaMesAño_HoraMin(01012022_0205) Archivo OC
char test;

double ch[4] ={124.0, 112.0, 936.0, 289.0}; 
int flagCali=0;

//************Declara vriables para interrupción************************
volatile int FlagInt=0;;  //Indicador de que hubo unterrupción
hw_timer_t * timer = NULL;  //Creamos una variable para renombrar el timer.
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; //Variable para sincronizar var entre int y loop
//▓▓▓▓▓▓▓▓▓  Rutina de interrupción *▓▓▓▓▓▓▓▓▓▓▓▓▓*
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);  //Bloquea proceso
  FlagInt = 1; //Activa bandera de interrupción
  contador++; //Incrementa el contador de muestras
  portEXIT_CRITICAL_ISR(&timerMux);   //Desbloquea proceso
}
//**********************************************************************

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Dom", "Lun", "Mar", "Mie", "Jue", "Vie", "Sab"};
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET    -1  // Reset pin # (or -1 if sharing reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// **********Inicializa ADS1222*****************************************
void iniADS1222(void) {
  byte x, n;
  digitalWrite(mux, HIGH);
  for (n = 0; n < 3; n++) {         // realizamos 3 lecturas completas
    delay (10);
    for (x = 0; x < 24; x++) {      // 24 pulsos-> 24 bits
      digitalWrite(sclk, HIGH);     // HIGH pulse, algo más de 33us
      delayMicroseconds(30);
      digitalWrite(sclk, LOW);      // LOW pulse, algo más de 33us
      delayMicroseconds(30);
    }
  }
  delay (10);
  while (digitalRead(Dout1) + digitalRead(Dout2)) {}  // esperamos datos preparados
  for (x = 0; x < 26; x++) {                          // auto-calibrado 26 pulsos-> 24bits + 2  On calibración
    digitalWrite(sclk, HIGH);
    delayMicroseconds(5);
    digitalWrite(sclk, LOW);
    delayMicroseconds(5);
  }
#ifdef debug
  Serial.println("Auto-calibrado..");
#endif
  while (digitalRead(Dout1) + digitalRead(Dout2)) {}     // esperamos fin calibracion
#ifdef debug
  Serial.println("Inicializando");
#endif
}
//***********************************************************************************
// ADS1222 leemos (se leen los 20 bits MSB, solo son efectivos los 20 bits de más peso)
void read_ads1222(void) {
    byte x, n = 0, z = 2;
    bool canal = 0;
  digitalWrite(mux, canal);                      // selecionamos el canal 0
  delayMicroseconds(8);  //tenia 8
  do {
    delayMicroseconds(2); // tenía 2
  } while (digitalRead(Dout1) + digitalRead(Dout2));   // esperamos datos listos, Dout1 y Dout2

  for (x = 23; x >= 4; x--) {               // del bit 23 al 3
    digitalWrite(sclk, HIGH);
    digitalRead(Dout1) ? bitWrite(sensor[n], x, 1) : bitWrite(sensor[n], x, 0); // algo más de 16us, leemos 0 y 1
    digitalRead(Dout2) ? bitWrite(sensor[z], x, 1) : bitWrite(sensor[z], x, 0); // algo más de 16us, leemos 2 y 3
    digitalWrite(sclk, LOW);
    delayMicroseconds(2);  //tenía 30
  }
  for (x = 0; x < 5; x++) {               // realizamos 5 pulsos, bits del 3 al 0 + pulso 25 -> forzamos Dout1 y Dout2 a 1
    digitalWrite(sclk, HIGH);
    delayMicroseconds(2);  //tenía 30
    digitalWrite(sclk, LOW);
    delayMicroseconds(2);    //tenía 30
  }
// Lee canal 1
    n = 1;    z = 3;     canal = 1;
  digitalWrite(mux, canal);                      // selecionamos el canal 1
  delayMicroseconds(2);  //tenia 8
  do {
    delayMicroseconds(2); // tenía 2
  } while (digitalRead(Dout1) + digitalRead(Dout2));   // esperamos datos listos, Dout1 y Dout2

  for (x = 23; x >= 4; x--) {               // del bit 23 al 3
    digitalWrite(sclk, HIGH);
    digitalRead(Dout1) ? bitWrite(sensor[n], x, 1) : bitWrite(sensor[n], x, 0); // algo más de 16us, leemos 0 y 1
    digitalRead(Dout2) ? bitWrite(sensor[z], x, 1) : bitWrite(sensor[z], x, 0); // algo más de 16us, leemos 2 y 3
    digitalWrite(sclk, LOW);
    delayMicroseconds(2);  //tenía 30
  }
  for (x = 0; x < 5; x++) {               // realizamos 5 pulsos, bits del 3 al 0 + pulso 25 -> forzamos Dout1 y Dout2 a 1
    digitalWrite(sclk, HIGH);
    delayMicroseconds(2);  //tenía 30
    digitalWrite(sclk, LOW);
    delayMicroseconds(2);    //tenía 30
  }
// Revisa si no tiene valores negativos y convierte a mV
    for (int j=0;j<=3;j++){   //Si el bit 23 es 1, entonces es un valor negativo y convierte
    if (bitRead(sensor[j],23)){
      mV[j]=(double)(sensor[j]-16777216.0)/8388.607*3.3;
      }
      else{mV[j]=(double) sensor[j]/8388.607*3.3;}
    }
/////////////////////////////////
    for (int j=0;j<=3;j++){
      if (flagCali!=2){flagCali=0;}// Si esta calibrando flag será 0, posterior a calibrar flag será 2
      if (abs(mV[j]-ch[j])<30 | flagCali==2){ //Si mV[j] es parecido a valor normal de ch[j], actualiza el valor de ch[j] 
         if ((flagCali==2) & ((abs(mV[j]-calib[j])<=800) & (mV[j]-calib[j])>=-3)){  //Verficia si esta calibrando, 2 == Si esta calibrada la WBB  //(flagCali==2) & ((abs(mV[j]-calib[j])<=800) & (mV[j]-calib[j])>=-3)
            ch[j]=mV[j];
            }
          if (flagCali==0) { //No esta calibrada la WBB
            flagCali=1; //Activa bandera para saber que no es un pico
            ch[j]=mV[j];
            }
        }
      if((abs(mV[j]-ch[j]))>30 & flagCali==0){ //Si existe un pico, asigna dato anterior durante la calibración
          mV[j]=ch[j];
        }
      if((abs(mV[j]-calib[j])>=800 | mV[j]-calib[j]<=-1)& flagCali==2){        
          mV[j]=ch[j];
      }
    }
 }
 
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
 //Función para tono en Buzzer. Cuando se llame requiere el Pin donde saldrá el pulso y
 //la frequencia deseada para el pulso
void tone(byte pin, int freq) {
  ledcSetup(0, 2000, 8); // setup beeper
  ledcAttachPin(pin, 0); // attach beeper
  ledcWriteTone(0, freq); // play tone
}


////////////////////////////////////////
float cop_vec[1200];

//////////////////////////enrique///////////////////////////////////
void Filtrar_datos_SD(char rut_file[], char rut_newfile[]){
  ////////////////////////////////////////////////////////////////////////////////////////////
  // Titulo: Función para leer, filtrar y guardar datos en una SD                           //
  // Este función recibe:                                                                   //
  // -rut_file:Ruta del archivo txt con datos crudos de las trayectorias del copx y copy.   //
  // -rut_newfile:Nueva ruta de un achivo txt, donde se almacenarán las trayectorias        //
  // del copx y copy filtradas mediante un filtro IRR digital butterworth orden 8           //
  // con fm=50 y fc=7hz.                                                                    //
  ////////////////////////////////////////////////////////////////////////////////////////////
  
 // ----------Inicialización de variables inicio --------------------
  File myFile;
  File myFileFiltrado;
  myFile = SD.open(rut_file);//abrimos  el archivo
  myFileFiltrado = SD.open (rut_newfile,FILE_WRITE); 
  //myFile = SD.open("/prueba.txt");//abrimos  el archivo
  //myFileFiltrado = SD.open ("/pruebaF.txt",FILE_WRITE);  
  int flagColumna  = 0;
  String cadena;
  float window_x[8]={0,0,0,0,0,0,0,0};
  float cop_x[8]={0,0,0,0,0,0,0,0};
  float window_y[8]={0,0,0,0,0,0,0,0};
  float cop_y[8]={0,0,0,0,0,0,0,0};
  int cont_8=0;
  int flag_filtro=0;
  float  a[8]={1, -3.06190312916490, 4.61551695155407,  -4.15585580305572, 2.37815653704039,  -0.853179924410146,  0.176453731249952, -0.0161309543925350};
  float b[8]={0.000648886006415002,  0.00454220204490501, 0.0136266061347150,  0.0227110102245251,  0.0227110102245251,  0.0136266061347150,  0.00454220204490501, 0.000648886006415002};
  float cop[2]={0,0};
  int cont_n=0;
// ----------Inicialización de variables fin --------------------

// --------- Bloque algoritmo ejecutable inicio -----------------------
while(myFile.available()){ //Permite recorrer todos los caracteres del archivo
   char caracter = myFile.read(); //Lee caracter por caracter
        if(flagColumna==5){ // Si se detecta dato en la columna 5 
          if (caracter!=44){
            cadena=cadena+caracter; // Si es un signo - + ó un numero se almacena en una cadena
            }else{   
            cop[0]=cadena.toFloat(); // Se convierte la cadena de texto en flotante
            cont_n=cont_n+1;
            cadena.clear();
            if (cont_8<7){
              cont_8=cont_8+1;
              window_x[cont_8-1]=cop[0];
              }else{
                window_x[cont_8]=cop[0];
                for(int i=0;i<=7;i++){
                  cop_x[cont_8]=(cop_x[cont_8]+(b[i]*window_x[7-i])-(a[i]*cop_x[7-i]));
                  }
                 Serial.print(cop[0]); 
                 Serial.print("  ");
                 Serial.print(cop_x[cont_8]);
                 Serial.print("  ");
                 myFileFiltrado.print(cop_x[cont_8]);
                 myFileFiltrado.print(",");
                for (int i=0;i<=6;i++){
                  cop_x[i]=cop_x[i+1];
                  window_x[i]=window_x[i+1];
                  }
                 cont_8=7;  
                }
              }
          }

         if (flagColumna==6){
          if (caracter!=44){ 
           cadena=cadena+caracter;
            if (myFile.position()==myFile.size()){
                cop[1]=cadena.toFloat();
//                cop_vec[cont_n]=cop[1];
                cadena.clear();
                /////////////////////////
                for(int i=0;i<=7;i++){
                  cop_y[cont_8]=(cop_y[cont_8]+(b[i]*window_y[7-i])- (a[i]*cop_y[7-i]));
                  }
                 myFileFiltrado.print(cop_y[cont_8]); 
                //////////////////////////////
              }
          }else{
            cop[1]=cadena.toFloat();
            cadena.clear();     
           ////////////////////////
              if (cont_8<7){
              cont_8=cont_8+1;  
              window_y[cont_8-1]=cop[1];
              }else{
                window_y[cont_8]=cop[1];
                for(int i=0;i<=7;i++){
                  cop_y[cont_8]=(cop_y[cont_8]+(b[i]*window_y[7-i])- (a[i]*cop_y[7-i]));
                  }
                 Serial.print(cop[1]);
                 Serial.print(" ");  
                 Serial.println(cop_y[cont_8]);
                 myFileFiltrado.println(cop_y[cont_8]);
                for (int i=0;i<=6;i++){
                  cop_y[i]=cop_y[i+1];
                  window_y[i]=window_y[i+1];
                  }  
                }
           ///////////////////////////
            }  
        }
        if (flagColumna>6) { 
          flagColumna=1;
        }
   if (caracter==44){
      flagColumna=flagColumna+1;
   }     
  }
   myFileFiltrado.close(); 
   myFile.close();
  }
  // --------- Bloque algoritmo ejecutable fin --------------------------


//////////////////////////enrique///////////////////////////////////

//█████████████████████████████████████████████████████
//████████████████ SETUP █████████████████████████████████

void setup()
{
  //******************Inicia Comunicación serial*****************
  Serial.begin(115200);
  if (!SD.begin(5)) {   //Inicializa SD
    Serial.println("Fallo conexion serial!");
    while (1);
  }
  //***********************Inicia RTC****************************+
  if (! rtc.begin()) {
    Serial.println("Fallo en RTC");
    while (1);
  }
  //*********************Inicia OLED****************************
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { Serial.println(F("NO PUDO INICIAR OLED"));
    for (;;); // Don't proceed, loop forever
  }

  //█████████****Inicia Timer para interrupción*****█████████████████*********
  timer = timerBegin(0, 80, true); //Inicializa timer 0, preescaler 80, conteo incremental (1us/incremento)
  timerAttachInterrupt(timer, &onTimer, true); //Usando timer0 nombramos rutina "onTimer" que se activará en edge
  timerAlarmWrite(timer, 20000, true); //Interrumpe cada 20mS (50Hz) se interrumpirá, y recarga el contador
  timerAlarmEnable(timer); //Habilitamos el timer para iniciar las interrupciones
  //*********************************************************

// rtc.adjust(DateTime(__DATE__, __TIME__));  //Solo se usa la primera vez para configurar el RTC con el time de la PC

//*********************██*ENCIENDE WBBB*██****************************************+
  byte ciclo = 0;
  pinMode(Buzz, OUTPUT);    //Indica que el pin del Buzzer será tipo salida
  pinMode(ON_wii, OUTPUT);
  digitalWrite(ON_wii, HIGH);      //Enciende WBB PCB
  pinMode(mux, OUTPUT);
  digitalWrite(mux, LOW);
  pinMode(sclk, OUTPUT);
  digitalWrite(sclk, LOW);
  pinMode(LED_wii, OUTPUT);
  digitalWrite(LED_wii, HIGH);
  pinMode(PULSADOR_wii, INPUT_PULLUP);  //Activa Pull-up para boton
  pinMode(Dout1, INPUT);
  pinMode(Dout2, INPUT);

  iniADS1222();  //Inicia ADCs
  tone(Buzz,10000);  //Manda señal a Buzzer a 1kHz, para indicar que ya se prendió la WBB
  delay(500);   //Espera 200ms (Este ratardo igual es necesario para estabilizar los ADCs

//************************███CALIBRA███***********************************
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("CALIBRANDO SENSORES");
  display.display();
  tone(Buzz,0); //Apaga buzzer
int cont=0;
int muestras=200;
Serial.print("Inicia Calibración");
while (cont<muestras){
if (FlagInt==1){
    read_ads1222();   //Lee datos de sensores
    for (int j=0;j<=3;j++){  //FOR para leer datos e ir sumando valores
    calib[j]+=mV[j]; //va sumando el valor de cada lectura para los 4 sensores en mV   
    }
    Serial.print(mV[0]);
    Serial.print(" ");
    Serial.print(mV[1]);
    Serial.print(" ");
    Serial.print(mV[2]);
    Serial.print(" ");
    Serial.println(mV[3]);
    cont++;   //Incrementa contador
   FlagInt = 0; //Borra bandera de interrupción para indicar que ya hizo la tarea
  }  //Cierra ciclo de comparación si hay interrupción
}  //Cierra while de número de muestras
    for (int j=0;j<=3;j++){  //Calcula el promedio de las muestras
    calib[j]=calib[j]/muestras;}
//    display.clearDisplay();  //Borra OLED
//    display.setCursor(0, 0);  //Pocisiona cursor para mensaje
//    display.print("Calibrado");  //Indica que termino de obtener valor de calibración
//    display.display();  //Envía a pantalla
 ch[0]=0;
 ch[1]=0;
 ch[2]=0;
 ch[3]=0;
 Serial.println("Termina Calibración");
//********TERMINA CALIBRACIÓN*****************************
//  DateTime now = rtc.now();   //Obtiene datos para nombrar archivo
//  sprintf (filename, "/%04d%02d%02d_%02d%02d.txt", now.year(), now.month(), now.day(), now.hour(), now.minute());
//  myFile = SD.open(filename, FILE_WRITE);
  //  myFile = SD.open("/test.txt", FILE_WRITE);
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Suba a WBB");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
//  delay(5000); //Tiempo para ver mensaje
//  DateTime now = rtc.now();
//  segs=now.unixtime();
}


//██████████████████████/LOOP//██████████████████████████████████
//████████████████████████████████████████████████████████████
void loop(){
//▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//Chequea si el peso es superior a un umbral
while (flagUmbral==0){   //Mientras no sobrepase el umbral
if (FlagInt==1){  //Si hay una interrupción, lee datos
//    timeInit=micros();
    flagCali=2; // Si ya esta calibrada manda un flag = 2
    read_ads1222();   //Lee datos de sensor 2 y 4
    for(int j=0;j<=3;j++){  //Para cada canal
    mV[j]=mV[j]-calib[j]; //Resta el offset de calibración al valor leído 
//    timeFin=micros();   //Revisa que tiempo pasó antes de la última interrupción
//    timeElapsed=timeFin-timeInit; 
//    Serial.print(timeElapsed);
//    Serial.print(" ");    
    if (j<3){Serial.print(mV[j]);  //Manda datos para ver nivel
       Serial.print(" ");}
    if (j==3){Serial.println(mV[j]);}
    }
    //Cierra ciclo de eliminación de offset
    sum_sensors=abs(mV[0])+abs(mV[1])+abs(mV[2])+abs(mV[3]); //suma valores sensores
    if(sum_sensors>umbral){
      peaks++; //Incrementa dcontador de picos
      if (peaks>100){  //Si ya pasó 1 segundo con umbral arriba
      flagUmbral=1;  //Activa bandera de que ya pasó el umbral
      peaks=0;    //Reinicia contador de picos
        }  
     } 
   FlagInt = 0; //Borra bandera de interrupción para indicar que ya hizo la tarea
}
// Mientras no haya interrupción, muestra la hora en la pantalla OLED
  DateTime now = rtc.now();
  display.setTextSize(2);  //Determina tamaño de texto
  display.setCursor(77, 20); //Determina pocisión inicial de texto
    if (now.second()<=9){
      display.print("0");
        display.println(now.second(), DEC);  //Segundos
        }
    else{
      display.setTextColor(WHITE, BLACK);  //Borra ultimo valor
      display.println(now.second(), DEC);  //Segundos
    }
  display.setCursor(25, 20);
  display.println(":");
  display.setCursor(65, 20);
  display.println(":");
  display.setCursor(40, 20);
  display.setTextColor(WHITE, BLACK);  //Borra ultimo valor
   if (now.minute()<10){
   display.print("0");}
  display.setTextColor(WHITE, BLACK);  //Borra ultimo valor
  display.println(now.minute(), DEC);  //Minutos
  display.setCursor(0, 20);
  display.setTextColor(WHITE, BLACK);  //Borra ultimo valor
  display.println(now.hour(), DEC);  //Horas
  display.display();


}
//░░░░░░░░░░Hay alguien sobre la plataforma, inicia grabación ░░░░░░░░░░░░░░░░░░░░░░░░
if(flagUmbral==1){   //Si ya detectó que alguien subió a la plataforma
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Creando");  //Mensaje para indicar que el sistema está listo
  display.setCursor(0, 20);
  display.print("archivo");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
  tone(Buzz,5000); //enciende Buzzer
  DateTime now = rtc.now();   //Obtiene datos para nombrar archivo
  sprintf (filenameOA, "/%02d%02d%04d_%02d%02d_OA.txt", now.day(), now.month(), now.year(), now.hour(), now.minute());
  sprintf (filenameOAF, "/%02d%02d%04d_%02d%02d_OAF.txt", now.day(), now.month(), now.year(), now.hour(), now.minute());
  sprintf (filenameOC, "/%02d%02d%04d_%02d%02d_OC.txt", now.day(), now.month(), now.year(), now.hour(), now.minute());
  sprintf (filenameOCF, "/%02d%02d%04d_%02d%02d_OCF.txt", now.day(), now.month(), now.year(), now.hour(), now.minute());
  while (myFile==0){
  myFile = SD.open(filenameOA, FILE_WRITE); //Crea archivo para ojos cerrados
  delay(500);  //Tiempo para ver mensaje
  
    }
 
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Inicia");  //Mensaje para indicar que el sistema está listo
  display.setCursor(0, 20);
  display.print("grabacion");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
  delay(2000);  //Espera dos segundos antes de grabar para estabilizar un poco
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Grabando");  //Mensaje para indicar que el sistema está listo
  display.setCursor(0, 20);
  display.print("Etapa OA");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
  tone(Buzz,0); //Apaga Buzzer
  
  contador=0;    //Iniciazaliza contador de muestras 
//  rtc.now();   //Lee tiempo antes de iniciar
//  segs=now.unixtime();
while (contador<1500){  //1500 muestras para 30 s @50Hz muestreo
 if (FlagInt==1){  //Checa si hubo interrupción
  timeFin=micros();   //Revisa que tiempo pasó antes de la última interrupción
  timeElapsed=timeFin-timeInit;   //Calcula el tiempo qyue pasó entre esta interrupción y la anterior
    Serial.println(timeElapsed);
    
    read_ads1222();   //Lee datos de sensores, tarda aproximadamente 13.5ms en leer
    for(int j=0;j<=3;j++){  //Para cada canal resta offset
      mV[j]=mV[j]-calib[j];} 
    if (myFile) { //Guarda datos en microSD
      myFile.print(contador);
      myFile.print(",");
      myFile.print(mV[0]);
      myFile.print(",");
      myFile.print(mV[1]);
      myFile.print(",");
      myFile.print(mV[2]);
      myFile.print(",");
      myFile.print(mV[3]);
      myFile.print(",");
      myFile.print( (433/2)* (( (mV[2]+mV[3])-(mV[0]+mV[1]) ) / (mV[0]+mV[1]+mV[2]+mV[3])) );//CoPx/X=433mm
      myFile.print(",");
      myFile.println( (238/2)* (( (mV[2]+mV[0])-(mV[3]+mV[1]) ) / (mV[0]+mV[1]+mV[2]+mV[3])) );//CoPy/Y=238mm
      }
      else {Serial.println("Error de archivo");}  //Manda mensaje de error si no hay archivo
//      Serial.println(timeElapsed);  //Manda dato de tiempo de lectura
//    for (int j=0;j<=3;j++){  //Manda datos a serial para control
//      if (j<3){Serial.print(mV[j]);
//               Serial.print(" ");}
//      if (j==3){Serial.println(mV[j]);}
//      }

    FlagInt=0;  //Borra bandera de interrupción para esperar una nueva
     timeInit=micros();  //Actualiza valor de conteo inicial    
     } 
}
  myFile.close();  //Cierra archivo
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Cierre");  //Mensaje para indicar que el sistema está listo
  display.setCursor(0, 20);
  display.print(" los ojos");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
  delay(1000); //Espera 1 segundo a que cierre ojos y estabilice señal
//█████████████████ Inicia grabación ojos cerrados█████████████████
  tone(Buzz,5000);  //Enciende buzzer
  myFile = SD.open(filenameOC, FILE_WRITE); //Crea archivo para ojos cerrados
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Grabando");  //Mensaje para indicar que el sistema está listo
  display.setCursor(0, 20);
  display.print("Etapa OC");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
  delay(2000); // Para buzzer y estabilizar señal
  tone(Buzz,0); //enciende Buzzer
 contador=0;    //Iniciazaliza contador de muestras 
while (contador<1500){  //1500 muestras para 3o s @50Hz muestreo
if (FlagInt==1){  //Checa si hubo interrupción 
    read_ads1222();   //Lee datos de sensores
    for(int j=0;j<=3;j++){  //Para cada canal resta offset
      mV[j]=mV[j]-calib[j];} 
    if (myFile) { //Guarda datos en microSD
      myFile.print(contador);
      myFile.print(",");
      myFile.print(mV[0]);
      myFile.print(",");
      myFile.print(mV[1]);
      myFile.print(",");
      myFile.print(mV[2]);
      myFile.print(",");
      myFile.print(mV[3]);
      myFile.print(",");
      myFile.print( (433/2)* (( (mV[2]+mV[3])-(mV[0]+mV[1]) ) / (mV[0]+mV[1]+mV[2]+mV[3])) );//CoPx/X=433mm
      myFile.print(",");
      myFile.println( (238/2)* (( (mV[2]+mV[0])-(mV[3]+mV[1]) ) / (mV[0]+mV[1]+mV[2]+mV[3])) );//CoPy/Y=238mm
      
    }
      else {Serial.println("Error de archivo");}  //Manda mensaje de error si no hay archivo
//    for (int j=0;j<=3;j++){  //Manda datos a serial para control
//      if (j<3){Serial.print(mV[j]);
//               Serial.print(" ");}
//      if (j==3){Serial.println(mV[j]);}
//      }
      FlagInt=0;  //Borra bandera de interrupción para esperar una nueva
//      contador++; //Incrementa contador de muestras 
     } 
}
  myFile.close();  //Cierra archivo
  // flagUmbral=0;   //Borra bandera de uso, para detener ciclos
  for (int i=0;i<=5;i++){
}  //Cierra ciclo de grabación, ahora vuelve a esperar a que se suban a la plataforma

while (flagUmbral==1){   //Mientras no sobrepase el umbral
    display.clearDisplay();  //Borra OLED
    display.setCursor(0, 0);
    display.print("PRUEBA");  //Mensaje para indicar que el sistema está listo
    display.setCursor(0, 20);
    display.print("FINALIZADA");  //Mensaje para indicar que el sistema está listo
    display.display();  //Envía a pantalla
    tone(Buzz,6000); // Buzzer
    delay(500); 
    tone(Buzz,0); //Apaga Buzzer
    delay(500); 
if (FlagInt==1){  //Si hay una interrupción, lee datos
    flagCali=2; // Si ya esta calibrada manda un flag = 2
    read_ads1222();   //Lee datos de sensor 2 y 4
    for(int j=0;j<=3;j++){  //Para cada canal
    mV[j]=mV[j]-calib[j]; //Resta el offset de calibración al valor leído 
    if (j<3){Serial.print(mV[j]);  //Manda datos para ver nivel
       Serial.print(" ");}
    if (j==3){Serial.println(mV[j]);}
    } //Cierra ciclo de eliminación de offset
    sum_sensors=abs(mV[0])+abs(mV[1])+abs(mV[2])+abs(mV[3]); //suma valores sensores
    if(sum_sensors<umbral){
      flagUmbral=0;  //Activa bandera de que ya pasó el umbral
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Suba a WBB");  //Mensaje para indicar que el sistema está listo
      display.display();  //Envía a pantalla
        }  
     } 
   FlagInt = 0; //Borra bandera de interrupción para indicar que ya hizo la tarea

}
}
  ///////////////////////////////////Filtrado/////////////////////////////////////////
  Serial.println("Filtrado OA");
  Filtrar_datos_SD(filenameOA,filenameOAF);
  Serial.println("Filtrado OC");
  Filtrar_datos_SD(filenameOC,filenameOCF);
   ////////////////////////////////////////////////////////////////////////////////////

   

}
