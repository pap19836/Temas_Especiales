//***************************************************************************************************************************************
/* Librería para el uso de la pantalla ILI9341 en modo 8 bits
 * Basado en el código de martinayotte - https://www.stm32duino.com/viewtopic.php?t=637
 * Adaptación, migración y creación de nuevas funciones: Pablo Mazariegos y José Morales
 * Con ayuda de: José Guerra
 * Modificaciones y adaptación: Diego Morales
 * IE3027: Electrónica Digital 2 - 2021
 */
//***************************************************************************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <TM4C123GH6PM.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
//#include <SPI.h>
//#include <SD.h>
//File myFile;
#include "bitmaps.h"
#include "font.h"
#include "lcd_registers.h"

#define LCD_RST PD_0
#define LCD_CS PD_1
#define LCD_RS PD_2
#define LCD_WR PD_3
#define LCD_RD PE_1
int DPINS[] = {PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7};  
//***************************************************************************************************************************************
// Functions Prototypes
//***************************************************************************************************************************************
void LCD_Init(void);
void LCD_CMD(uint8_t cmd);
void LCD_DATA(uint8_t data);
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void LCD_Clear(unsigned int c);
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void LCD_Print(String text, int x, int y, int fontSize, int color, int background);

void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]);
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[],int columns, int index, char flip, char offset);
void Muerto(void);
void Menu(void);

//extern uint8_t runner[];
extern uint8_t background[];
//extern uint8_t runnerBit[];
//extern uint8_t bossBit[];
//extern uint8_t cloudBit[];
//extern uint8_t routeBit[];
//extern uint8_t rockBit[];
struct Sprite1 { // estructura para sprites
  int x; // posicion x
  int y; // posicion y
  int width; // ancho de bitmap
  int height; // altura de bitmap
  int columns; // columna sprite sheet
  int index; // indice sprite sheet
  int flip; // voltear imagen
  int offset; // desfase
}runner, cloud, boss, route, rock;


int subir, bajar, jefe, contador, inicio;
int pin = PE_3;
unsigned long previousMillis = 0;  
const long interval = 42;
int rock_flag, kick_flag = 0;
bool colision;
int player;
char buffer[1];

void setup() {
  SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
  Serial.begin(9600);     //Este es el jugador
  Serial2.begin(9600);    //Este es el jefe
  //Serial3.begin(9600);
  GPIOPadConfigSet(GPIO_PORTB_BASE, 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
  Serial.println("Start");
  LCD_Init();
  LCD_Clear(0x00);
  pinMode(PUSH1, INPUT_PULLUP);
  pinMode(pin, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP);
  pinMode(BLUE_LED, OUTPUT);
  //SPI.setModule(0);                                       //Inicialización del módulo SPI0
  //pinMode(PA_3, OUTPUT);
  //SD.begin(PA_3);
//  FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
//
//    
//  LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]);
  

  runner.x = 20;
  runner.y = 200;
  runner.width = 28;
  runner.height = 39;
  runner.columns = 8;
  runner.index = 0;
  runner.flip = 0;
  runner.offset = 0;

  boss.x = 150;
  boss.y = 120;
  boss.width = 64;
  boss.height = 34;
  boss.columns = 1;
  boss.index = 0;
  boss.flip = 0;
  boss.offset = 0;

  cloud.x = 320;
  cloud.y = 40;
  cloud.width = 36;
  cloud.height = 15;
  cloud.columns = 1;
  cloud.index = 0;
  cloud.flip = 0;
  cloud.offset = 0;

  route.x = 0;
  route.y = 120;
  route.width = 320;
  route.height = 120;
  route.columns = 5;
  route.index = 0;
  route.flip = 0;
  route.offset = 0;
//
  rock.x = 150;
  rock.y = 100;
  rock.width = 32;
  rock.height = 32;
  rock.columns = 1;
  rock.index = 0;
  rock.flip = 0;
  rock.offset = 0;

  Serial.write("probando");
  
 inicio = 0;
 contador=0;
}
//***************************************************************************************************************************************
// Loop
//***************************************************************************************************************************************
void loop() {
    unsigned long currentMillis = millis();
  if(contador==25){
    Ganador();
  }
  if(inicio==0){
    Menu();
  }
  if(inicio==1){
    LCD_Bitmap(0, 0, 320, 240, background);
    //FillRect(0, 0, 320, 240, 0x0000);
    contador = 0;
    inicio = 2;
  }
  // actualización de frame cada 42ms = 24fps
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    while(Serial2.available()){
      jefe = Serial2.read();
    }
    
    while(Serial.available() > 0){
      player = Serial.read();
    }
    if (player == 0) {
      digitalWrite(BLUE_LED, 1);
      Serial.println("asawhe");
    }
    else{
      digitalWrite(BLUE_LED, 0);
    }
    //************************************************************************************************************************************
    //Configuracion y acciones del BOSS
    //*************************************************************************************************************************************
    if (jefe == 0x31) { // modificación de atributos de sprite
      boss.x += 4;
      if(rock_flag!=2){
        rock.x +=4;
      }
      boss.index++;
      boss.index %= 4;
      boss.flip = 0;
      subir = 1;
      jefe = 0;
    }
    if (jefe == 0x32) {
      boss.x -= 4;
      if(rock_flag!=2){
        rock.x -=4;
      }
      boss.index++;
      boss.index %= 4;
      boss.flip = 0;
      bajar = 1;
      jefe = 0;
    }
    if(boss.x >= 320){
      boss.x = 319;
    }
    else if(boss.x <= 0){
      boss.x = 1;
    }
    if (subir==1){ // dependiendo de la dirección, se colorea resto del sprite del frame anterior
      FillRect(boss.x- boss.width/2, boss.y, boss.width, boss.height, 0x0000);
      subir = 0;
    }
    else if(bajar==1){
      FillRect(boss.x + boss.width/2, boss.y, boss.width, boss.height, 0x0000);
      bajar = 0;
    }
    LCD_Sprite(boss.x, boss.y, boss.width, boss.height, bossBit, boss.columns, boss.index, boss.flip, boss.offset);
    //************************************************************************************************************************************
    //Configuracion y acciones del runner
    //*************************************************************************************************************************************
    runner.index++;
    runner.index %=9;
    
    LCD_Sprite(runner.x, runner.y, runner.width, runner.height, runnerBit, runner.columns, runner.index, runner.flip, runner.offset);
    if(player > 0x30){
      //FillRect(runner.x, runner.y, runner.width, runner.height, 0xffff);
      LCD_Bitmap(runner.x, runner.y, runner.width, runner.height, runner_kick);
      kick_flag = 1;
    }
    else{
      //FillRect(runner.x, runner.y, runner.width, runner.height, 0xffff);
      LCD_Sprite(runner.x, runner.y, runner.width, runner.height, runnerBit, runner.columns, runner.index, runner.flip, runner.offset);
      kick_flag = 0;
    }
    //************************************************************************************************************************************
    //Configuracion y acciones de la roca
    //*************************************************************************************************************************************
    
    if((jefe == 0x33)&&(rock_flag==0)){
      digitalWrite(BLUE_LED, LOW);
      rock_flag = 1;
      jefe = 0;
    }
    if(rock_flag==1){
      LCD_Bitmap(rock.x, rock.y, rock.width, rock.height, rockBit);
      rock.y+=4;
      FillRect(rock.x, rock.y-rock.height/10, rock.width, rock.height, 0xffff);
      if(rock.y==180){
        FillRect(0, 120, 320, 40, 0x0000);
      }
    }
    if(rock.y==208){
        rock_flag = 2;
      }
    if(rock_flag==2){
      LCD_Bitmap(rock.x, rock.y, rock.width, rock.height, rockBit);
      rock.x-=5;
      FillRect(rock.x + rock.width, rock.y, rock.width, rock.height, 0xffff);
    }
    if((rock.x <= runner.x)&&(rock_flag==2)){
      rock_flag = 0;
      digitalWrite(BLUE_LED, HIGH);
      rock.x = boss.x;
      rock.y = boss.y;
      FillRect(runner.x, runner.y, runner.width+rock.width/2, runner.height, 0xffff);
    }
    //************************************************************************************************************************************
    //Configuracion y acciones de la nube
    //*************************************************************************************************************************************
    cloud.x-=2;
    if((cloud.x+cloud.width)==0){
      cloud.x = 320;
    }
    LCD_Sprite(cloud.x, cloud.y, cloud.width, cloud.height, cloudBit, cloud.columns, cloud.index, cloud.flip, cloud.offset);
    FillRect(cloud.x + cloud.width, cloud.y, cloud.width, cloud.height, 0x0000);

    //***************************************************************************************************************************************
    //Colision
    //***************************************************************************************************************************************
    colision = Collision(runner.x, runner.y, runner.width, runner.height, rock.x, rock.y, rock.width, rock.height);
    if((colision)&&(kick_flag==0)){
      Muerto();
    }
    else if((colision)&&(kick_flag==1)){
      contador++;
      kick_flag=0;
    }
    
  }
}


//***************************************************************************************************************************************
// Función para inicializar LCD
//***************************************************************************************************************************************
void LCD_Init(void) {
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RD, OUTPUT);
  for (uint8_t i = 0; i < 8; i++){
    pinMode(DPINS[i], OUTPUT);
  }
  //****************************************
  // Secuencia de Inicialización
  //****************************************
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_RD, HIGH);
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  delay(20);
  digitalWrite(LCD_RST, HIGH);
  delay(150);
  digitalWrite(LCD_CS, LOW);
  //****************************************
  LCD_CMD(0xE9);  // SETPANELRELATED
  LCD_DATA(0x20);
  //****************************************
  LCD_CMD(0x11); // Exit Sleep SLEEP OUT (SLPOUT)
  delay(100);
  //****************************************
  LCD_CMD(0xD1);    // (SETVCOM)
  LCD_DATA(0x00);
  LCD_DATA(0x71);
  LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0xD0);   // (SETPOWER) 
  LCD_DATA(0x07);
  LCD_DATA(0x01);
  LCD_DATA(0x08);
  //****************************************
  LCD_CMD(0x36);  // (MEMORYACCESS)
  LCD_DATA(0x40|0x80|0x20|0x08); // LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0x3A); // Set_pixel_format (PIXELFORMAT)
  LCD_DATA(0x05); // color setings, 05h - 16bit pixel, 11h - 3bit pixel
  //****************************************
  LCD_CMD(0xC1);    // (POWERCONTROL2)
  LCD_DATA(0x10);
  LCD_DATA(0x10);
  LCD_DATA(0x02);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC0); // Set Default Gamma (POWERCONTROL1)
  LCD_DATA(0x00);
  LCD_DATA(0x35);
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC5); // Set Frame Rate (VCOMCONTROL1)
  LCD_DATA(0x04); // 72Hz
  //****************************************
  LCD_CMD(0xD2); // Power Settings  (SETPWRNORMAL)
  LCD_DATA(0x01);
  LCD_DATA(0x44);
  //****************************************
  LCD_CMD(0xC8); //Set Gamma  (GAMMASET)
  LCD_DATA(0x04);
  LCD_DATA(0x67);
  LCD_DATA(0x35);
  LCD_DATA(0x04);
  LCD_DATA(0x08);
  LCD_DATA(0x06);
  LCD_DATA(0x24);
  LCD_DATA(0x01);
  LCD_DATA(0x37);
  LCD_DATA(0x40);
  LCD_DATA(0x03);
  LCD_DATA(0x10);
  LCD_DATA(0x08);
  LCD_DATA(0x80);
  LCD_DATA(0x00);
  //****************************************
  LCD_CMD(0x2A); // Set_column_address 320px (CASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x3F);
  //****************************************
  LCD_CMD(0x2B); // Set_page_address 480px (PASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0xE0);
//  LCD_DATA(0x8F);
  LCD_CMD(0x29); //display on 
  LCD_CMD(0x2C); //display on

  LCD_CMD(ILI9341_INVOFF); //Invert Off
  delay(120);
  LCD_CMD(ILI9341_SLPOUT);    //Exit Sleep
  delay(120);
  LCD_CMD(ILI9341_DISPON);    //Display on
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para enviar comandos a la LCD - parámetro (comando)
//***************************************************************************************************************************************
void LCD_CMD(uint8_t cmd) {
  digitalWrite(LCD_RS, LOW);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = cmd;
  digitalWrite(LCD_WR, HIGH);
}
//***************************************************************************************************************************************
// Función para enviar datos a la LCD - parámetro (dato)
//***************************************************************************************************************************************
void LCD_DATA(uint8_t data) {
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = data;
  digitalWrite(LCD_WR, HIGH);
}
//***************************************************************************************************************************************
// Función para definir rango de direcciones de memoria con las cuales se trabajara (se define una ventana)
//***************************************************************************************************************************************
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
  LCD_CMD(0x2a); // Set_column_address 4 parameters
  LCD_DATA(x1 >> 8);
  LCD_DATA(x1);   
  LCD_DATA(x2 >> 8);
  LCD_DATA(x2);   
  LCD_CMD(0x2b); // Set_page_address 4 parameters
  LCD_DATA(y1 >> 8);
  LCD_DATA(y1);   
  LCD_DATA(y2 >> 8);
  LCD_DATA(y2);   
  LCD_CMD(0x2c); // Write_memory_start
}
//***************************************************************************************************************************************
// Función para borrar la pantalla - parámetros (color)
//***************************************************************************************************************************************
void LCD_Clear(unsigned int c){  
  unsigned int x, y;
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);   
  SetWindows(0, 0, 319, 239); // 479, 319);
  for (x = 0; x < 320; x++)
    for (y = 0; y < 240; y++) {
      LCD_DATA(c >> 8); 
      LCD_DATA(c); 
    }
  digitalWrite(LCD_CS, HIGH);
} 
//***************************************************************************************************************************************
// Función para dibujar una línea horizontal - parámetros ( coordenada x, cordenada y, longitud, color)
//*************************************************************************************************************************************** 
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {  
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + x;
  SetWindows(x, y, l, y);
  j = l;// * 2;
  for (i = 0; i < l; i++) {
      LCD_DATA(c >> 8); 
      LCD_DATA(c); 
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una línea vertical - parámetros ( coordenada x, cordenada y, longitud, color)
//*************************************************************************************************************************************** 
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {  
  unsigned int i,j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + y;
  SetWindows(x, y, x, l);
  j = l; //* 2;
  for (i = 1; i <= j; i++) {
    LCD_DATA(c >> 8); 
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);  
}
//***************************************************************************************************************************************
// Función para dibujar un rectángulo - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  H_line(x  , y  , w, c);
  H_line(x  , y+h, w, c);
  V_line(x  , y  , h, c);
  V_line(x+w, y  , h, c);
}
//***************************************************************************************************************************************
// Función para dibujar un rectángulo relleno - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW); 
  
  unsigned int x2, y2;
  x2 = x+w;
  y2 = y+h;
  SetWindows(x, y, x2-1, y2-1);
  unsigned int k = w*h*2-1;
  unsigned int i, j;
  for (int i = 0; i < w; i++) {
    for (int j = 0; j < h; j++) {
      LCD_DATA(c >> 8);
      LCD_DATA(c); 
      k = k - 2;
     } 
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar texto - parámetros ( texto, coordenada x, cordenada y, color, background) 
//***************************************************************************************************************************************
void LCD_Print(String text, int x, int y, int fontSize, int color, int background) {
  int fontXSize ;
  int fontYSize ;
  
  if(fontSize == 1){
    fontXSize = fontXSizeSmal ;
    fontYSize = fontYSizeSmal ;
  }
  if(fontSize == 2){
    fontXSize = fontXSizeBig ;
    fontYSize = fontYSizeBig ;
  }
  
  char charInput ;
  int cLength = text.length();
  Serial.println(cLength,DEC);
  int charDec ;
  int c ;
  int charHex ;
  char char_array[cLength+1];
  text.toCharArray(char_array, cLength+1) ;
  for (int i = 0; i < cLength ; i++) {
    charInput = char_array[i];
    Serial.println(char_array[i]);
    charDec = int(charInput);
    digitalWrite(LCD_CS, LOW);
    SetWindows(x + (i * fontXSize), y, x + (i * fontXSize) + fontXSize - 1, y + fontYSize );
    long charHex1 ;
    for ( int n = 0 ; n < fontYSize ; n++ ) {
      if (fontSize == 1){
        charHex1 = pgm_read_word_near(smallFont + ((charDec - 32) * fontYSize) + n);
      }
      if (fontSize == 2){
        charHex1 = pgm_read_word_near(bigFont + ((charDec - 32) * fontYSize) + n);
      }
      for (int t = 1; t < fontXSize + 1 ; t++) {
        if (( charHex1 & (1 << (fontXSize - t))) > 0 ) {
          c = color ;
        } else {
          c = background ;
        }
        LCD_DATA(c >> 8);
        LCD_DATA(c);
      }
    }
    digitalWrite(LCD_CS, HIGH);
  }
}
//***************************************************************************************************************************************
// Función para dibujar una imagen a partir de un arreglo de colores (Bitmap) Formato (Color 16bit R 5bits G 6bits B 5bits)
//***************************************************************************************************************************************
void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]){  
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW); 
  
  unsigned int x2, y2;
  x2 = x+width;
  y2 = y+height;
  SetWindows(x, y, x2-1, y2-1);
  unsigned int k = 0;
  unsigned int i, j;

  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      LCD_DATA(bitmap[k]);
      LCD_DATA(bitmap[k+1]);
      //LCD_DATA(bitmap[k]);    
      k = k + 2;
     } 
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una imagen sprite - los parámetros columns = número de imagenes en el sprite, index = cual desplegar, flip = darle vuelta
//***************************************************************************************************************************************
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[],int columns, int index, char flip, char offset){
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW); 

  unsigned int x2, y2;
  x2 =   x+width;
  y2=    y+height;
  SetWindows(x, y, x2-1, y2-1);
  int k = 0;
  int ancho = ((width*columns));
  if(flip){
    for (int j = 0; j < height; j++){
        k = (j*(ancho) + index*width -1 - offset)*2;
        k = k+width*2;
       for (int i = 0; i < width; i++){
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k+1]);
        k = k - 2;
       } 
    }
  }
  else{
     for (int j = 0; j < height; j++){
      k = (j*(ancho) + index*width + 1 + offset)*2;
     for (int i = 0; i < width; i++){
      LCD_DATA(bitmap[k]);
      LCD_DATA(bitmap[k+1]);
      k = k + 2;
     } 
    }
  }
  digitalWrite(LCD_CS, HIGH);
}
bool Collision(int x1, int y1, int w1, int h1, int x2, int y2, int w2, int h2){
  return (x1 < x2 + w2) && (x1+ w1 > x2) && (y1 < y2 + h2) && (y1 + h1 > y2);
}

void Menu(){
  //mapeo_SD("menu.txt");
  FillRect(0, 0, 320, 240, 0x0000);
  String text1 = "BIT.TRIP RUNNER";
  LCD_Print(text1, 40, 110, 2, 0xffff, 0x0000);
  delay(2000);
  //mapeo_SD("menu.txt");
  FillRect(0, 0, 320, 240, 0x0000);
  text1 = "PRESIONE P1 PARA";
  LCD_Print(text1, 40, 90, 2, 0xffff, 0x0000);
  text1 = "NUEVA PARTIDA";
  LCD_Print(text1, 50, 110, 2, 0xffff, 0x0000);
  jefe = Serial2.read();
  player = Serial.read();
  while(player < 49){
    while(Serial.available() > 0){
      player = Serial.read();
    }
    if(player > 48){
      digitalWrite(BLUE_LED, 1);
    }
    else{
      digitalWrite(BLUE_LED,0);
    }
    //player = digitalRead(PUSH1);
    if(player > 48){
      inicio = 1;
      contador = 0;
    }
  }
//  myFile = SD.open("test.txt", FILE_READ);
//  Serial.write(myFile.read());
//  myFile.close();
  return;
}


void Muerto(){
  colision = 0;
  FillRect(50,50,220,140,0x0000);
//  sprintf(buffer, "%d", contador);
//  String text2 = buffer;
  String text2 = String(contador/5);
  String text1 = "PUNTOS";
  LCD_Print(text2, 60, 60, 2, 0xffff, 0x0000);
  LCD_Print(text1, 110, 60, 2, 0xffff, 0x0000);
  text1 = "PRESIONE P1";
  LCD_Print(text1, 60, 80, 2, 0xffff, 0x0000);
  text1 = "PARA SEGUIR";
  LCD_Print(text1, 60, 120, 2, 0xffff, 0x0000);
  text1 = "P2 PARA MENU";
  LCD_Print(text1, 60, 140, 2, 0xffff, 0x0000);
  while(inicio==2){
    jefe = Serial2.read();
    player = Serial.read();
    if(jefe == 0x31){
      inicio = 1;
    }
    if(player==0x31){
      inicio = 1;
      FillRect(0,0,320,240,0x0000);
    }
  }
  contador=0;
//  myFile = SD.open("test.txt", FILE_WRITE);
//  sprintf(buffer, "%d", contador/7);
//  myFile.println(buffer);
//  myFile.close();
  kick_flag = 0;
  rock_flag = 0;
  rock.x = boss.x;
  rock.y = 100;
  return;
}

void Ganador(){
  LCD_Bitmap(0, 0, 320, 240, background);
  String text1="GANADOR!!!";
  LCD_Print(text1,60,40,2,0xffff,0x0000);
  text1 = "PRESIONE P1";
  LCD_Print(text1, 60, 80, 2, 0xffff, 0x0000);
  text1 = "PARA SEGUIR";
  LCD_Print(text1, 60, 120, 2, 0xffff, 0x0000);
  player=Serial.read();
  while(player!=0x31){
    player=Serial.read();
  }
  if(player==0x31);
  inicio=1;
  contador=0;
}
int ascii2hex(int a) {
  switch (a) {
    case (48):      //caso 0
      return 0;
    case (49):      //caso 1
      return 1;
    case (50):      //caso 2
      return 2;
    case (51):      //caso 3
      return 3;
    case (52):      //caso 4
      return 4;
    case (53):      //caso 5
      return 5;
    case (54):      //caso 6
      return 6;
    case (55):      //caso 7
      return 7;
    case (56):      //caso 8
      return 8;
    case (57):      //caso 9
      return 9;
    case (97):      //caso A
      return 10;
    case (98):      //caso B
      return 11;
    case (99):      //caso C
      return 12;
    case (100):     //caso D
      return 13;
    case (101):     //caso E
      return 14;
    case (102):     //caso F
      return 15;
  }
}
//-------FUNCION PARA MOSTRAR LAS IMAGENES DESDE SD
//void mapeo_SD(char doc[]) {
//  myFile = SD.open(doc, FILE_READ);   //se toma el archivo de la imagen 
//  int hex1 = 0;                       //declaracion de variable 1 para valor hex
//  int val1 = 0;                       
//  int val2 = 0;
//  int mapear = 0;
//  int vertical = 0;
//  unsigned char maps[640];            //se crea arreglo vacio para almacenar el mapeo
//
//  if (myFile) {
//    while (myFile.available() ) {     //se leen datos mientras este disponible
//      mapear = 0;
//      while (mapear < 640) {          //se limita el rango
//        hex1 = myFile.read();         //se lee el archivo con la imagen
//        if (hex1 == 120) {
//          val1 = myFile.read();       //se lee el primer valor hexadecimal del bitmap
//          val2 = myFile.read();       //se lee el segundo valor hexadecimal del bitmap
//          val1 = ascii2hex(val1);     //se mapea el primer valor hexadecimal 
//          val2 = ascii2hex(val2);     //se mapea el segundo valor hexadecimal 
//          maps[mapear] = val1 * 16 + val2;  //se colona en el arreglo nuevo
//          mapear++;                         //se cambia de posicion
//        }
//      }
//      LCD_Bitmap(0, vertical, 320, 1, maps);
//      vertical++;
//    }
//    myFile.close();
//  }
//  else {
//    Serial.println("No se pudo abrir la imagen, prueba nuevamente");
//    myFile.close();
//  }
//}
