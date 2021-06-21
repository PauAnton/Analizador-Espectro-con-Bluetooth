#include <Arduino.h>

#include <Wire.h>
#include "arduinoFFT.h" 

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_I2CDevice.h"  

#include "esp32_bt_music_receiver.h"


// Definir constantes
#define ANCHO_PANTALLA 128 // ancho pantalla OLED
#define ALTO_PANTALLA 64 // alto pantalla OLED
 
// Objeto de la clase Adafruit_SSD1306
Adafruit_SSD1306 display(ANCHO_PANTALLA, ALTO_PANTALLA, &Wire, -1);


arduinoFFT FFT = arduinoFFT();
#define SAMPLES 512              // Must be a power of 2
#define SAMPLING_FREQUENCY 44100 // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
#define amplitude 200            // Depending on your audio source level, you may need to increase this value
  unsigned int sampling_period_us;
  unsigned long microseconds;

  byte peak[] = {0,0,0,0,0,0,0};
  double vReal[SAMPLES];
  double vImag[SAMPLES];
  unsigned long newTime, oldTime;


// BlootoothA2DSink a2d_sink;

void startOLED(int SDA, int SCL);
void startFFT();

void writebands(int x, int y, String txt,int size);
void displayBand(int band, int dsize);
void espectro();

void setup() {

  Serial.begin(9600);

  startOLED(5,4); //SDA,SDL
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));


}


void loop() {

 espectro();

}

void writebands(int x, int y, String txt,int size){
   
  display.setTextSize(size);
  //display.setTextColor(SSD1306_WHITE);

 display.setCursor(x, y);
 display.println(txt);
  display.display();
}

void displayBand(int band, int dsize){

  display.setTextSize(1);
  display.setCursor(0, 10);

  int dmax = 50;
  if (dsize > dmax) dsize = dmax;
  if (band == 7) display.drawFastHLine(18*6,0, 14, SSD1306_WHITE );
  for (int s = 0; s <= dsize; s=s+2){display.drawFastHLine(18*band,64-s, 14,SSD1306_WHITE );}
  if (dsize > peak[band]) {peak[band] = dsize;}
}

void startOLED(int SDA, int SCL){

Wire.begin(SDA,SCL); // SDA, SCL

  // Iniciar pantalla OLED en la dirección 0x3C
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
  #ifdef __DEBUG__
    Serial.println("No se encuentra la pantalla OLED");
    #endif
    while (true);
  }

  // Limpiar buffer
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
}

void espectro(){

 display.clearDisplay();

writebands(0,0,".1 .2 .5 1K 2K 4K 8K",1);

  //Muestreo 
  for (int i = 0; i < SAMPLES; i++) {
    newTime = micros()-oldTime;
    oldTime = newTime;
    vReal[i] = analogRead(A0); // A conversion takes about 1uS on an ESP32
    vImag[i] = 0;
    while (micros() < (newTime + sampling_period_us)) { /* do nothing to wait */ }
  }

// Realizacion FFT 
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  //Representacion de bandas. EXCEL
  for (int i = 2; i < (SAMPLES/2); i++){ // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
    if (vReal[i] > 2000) { // Add a crude noise filter, 10 x amplitude or more
      if (i<=2 )             displayBand(0,(int)vReal[i]/amplitude); // 125Hz
      if (i >3   && i<=5 )   displayBand(1,(int)vReal[i]/amplitude); // 250Hz
      if (i >5   && i<=7 )   displayBand(2,(int)vReal[i]/amplitude); // 500Hz
      if (i >7   && i<=15 )  displayBand(3,(int)vReal[i]/amplitude); // 1000Hz
      if (i >15  && i<=30 )  displayBand(4,(int)vReal[i]/amplitude); // 2000Hz
      if (i >30  && i<=53 )  displayBand(5,(int)vReal[i]/amplitude); // 4000Hz
      if (i >53  && i<=200 ) displayBand(6,(int)vReal[i]/amplitude); // 8000Hz
      if (i >200           ) displayBand(7,(int)vReal[i]/amplitude); // 16000Hz
    }
    for (byte band = 0; band <= 6; band++) display.drawFastHLine(18*band,64-peak[band],14, SSD1306_WHITE );
  }
  if (millis()%4 == 0) {for (byte band = 0; band <= 6; band++) {if (peak[band] > 0) peak[band] -= 4;}} // Decay the peak
  display.display();

}
