/*
Sensor code for Accelerometer

 Sections of code related to interacting with accelerometer were copied from STM example code under the BSD 3-Clause License

 <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
All rights reserved.</center></h2>
*/


#include "arduinoFFT.h"
#include "SparkFun_SinglePairEthernet.h"

extern "C" {
#include <iis2dh_reg.h>
}

// Config Defines
// #define DEBUG_MODE

#ifndef DEBUG_MODE
#define ChipSelPin CS
#define InterruptPin G1
#define ResetPin G0
#endif

// // Sensor driver setup
extern "C" {
int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);

IIS2DH_ctx_t dev_ctx; /** xxxxxxx is the used part number **/
dev_ctx.write_reg = platform_write;
dev_ctx.read_reg = platform_read;
}

// SPE driver setup
SinglePairEthernet adin1110;
byte deviceMAC[6] = { 0x00, 0xE0, 0x22, 0xFE, 0xDA, 0x03 };
byte destinationMAC[6] = { 0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xCA };

// FFT setup
arduinoFFT FFT;
const uint16_t samples = 1024;
const double samplingFrequency = 10000;
double vReal[samples];
double vImag[samples];
byte outputBuffer[samples / 2];

unsigned long lastBlink = 0;

void setup() {
#ifdef DEBUG_MODE
  Serial.begin(9600);
#endif
  setupSPI();

  pinMode(LED_BUILTIN, OUTPUT);
  analogReadResolution(8);
  delay(500);
#ifndef DEBUG_MODE
  if (!adin1110.begin(deviceMAC, LED_BUILTIN, InterruptPin, ResetPin, ChipSelPin)) {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
  while (adin1110.getLinkStatus() != true)
    ;
#endif
}

void loop() {
  collectData();
  calculateFFT();
  sendSensData();

#ifndef DEBUG_MODE
  unsigned long now = millis();
  if ((now - lastBlink >= 1000) && (adin1110.getLinkStatus())) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = now;
  }
#endif
}

void collectData() {
  
  
  // for (int i = 0; i < samples; i++) {
  //   vImag[i] = 0;
  // }
  // long int delay1, delay2, delay3, delay4;
  // for (int i = 0; i < samples; i++) {
  //   vReal[i] = analogRead(A0);
  //   delayMicroseconds(19);
  //   delay1++;
  //   delay2++;
  //   delay3++;
  //   delay4++;
  // }
}

void calculateFFT() {
  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
  FFT.Compute(FFT_FORWARD);                        /* Compute FFT */
  FFT.ComplexToMagnitude();                        /* Compute magnitudes */
}

void sendSensData() {
  for (int i = 0; i < samples/2; i++) {
    outputBuffer[i] = (byte)vReal[i];

#ifdef DEBUG_MODE
    Serial.print(outputBuffer[i]);
    Serial.print(" ");
#endif
  }
#ifdef DEBUG_MODE
  Serial.println();
#endif

#ifndef DEBUG_MODE
  if (adin1110.getLinkStatus()) {
    adin1110.sendData(outputBuffer, sizeof(outputBuffer), destinationMAC);
  }
#endif
  delay(2000);
}


// TODO SPI Setup
void setupSPI(){

  /* Initialize mems driver interface */
  //dev_ctx.handle = &SENSOR_BUS;
  /* Initialize platform specific hardware */
  //platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  iis2dh_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != IIS2DH_ID) {
    while (1) {
      iis2dh_device_id_get(&dev_ctx, &whoamI);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(800);
      digitalWrite(LED_BUILTIN, LOW);
      delay(300);
    }
  }

  /* Enable Block Data Update */
  iis2dh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate to 5.376kHz */
  iis2dh_data_rate_set(&dev_ctx, IIS2DH_ODR_5kHz376_LP_1kHz344_NM_HP);
  /* Set full scale to 2g */
  iis2dh_full_scale_set(&dev_ctx, IIS2DH_2g);
  /* Disable temperature sensor */
  iis2dh_temperature_meas_set(&dev_ctx, IIS2DH_TEMP_DISABLE);
  /* Set device in continuous mode with 12 bit resol. */
  iis2dh_operating_mode_set(&dev_ctx, IIS2DH_LP_8bit);

/* Set FIFO watermark to 25 samples */
  iis2dh_fifo_watermark_set(&dev_ctx, 16);
  /* Set FIFO mode to Stream mode: Accumulate samples and
   * override old data
   */
  iis2dh_fifo_mode_set(&dev_ctx, IIS2DH_DYNAMIC_STREAM_MODE);
  /* Enable FIFO */
  iis2dh_fifo_set(&dev_ctx, PROPERTY_ENABLE);

}

void readNewData(){
  // TODO
  uint8_t flags;
  uint8_t num = 0;
  /* Check if FIFO level over threshold */
  iis2dh_fifo_fth_flag_get(&dev_ctx, &flags);

  if (flags) {
    /* Read number of sample in FIFO */
    iis2dh_fifo_data_level_get(&dev_ctx, &num);

    while (num-- > 0) {
      /* Read XL samples */
      iis2dh_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] =
        iis2dh_from_fs2_hr_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        iis2dh_from_fs2_hr_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        iis2dh_from_fs2_hr_to_mg(data_raw_acceleration[2]);
      sprintf((char *)tx_buffer,
              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
// Accelerometer Driver Defines
extern "C" {
  int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len){
    // TODO
    // Transmission
    // bit 0    rw = 0 when write, 1 when read from
    // bit 1    when 0, address remains unchanged. When 1, address increments each rw
    // bit 2-7  address field of register
    // bit 8-15 data (msb first) 
 
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE2));
    for(int i = 0; i < len; i++){
      SPI.transfer(bufP[i]);
    }
    SPI.endTransaction();
  }
  int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len){
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE2));
    for(int i = 0; i < len; i++){
      bufP[i] = SPI.transfer();
    }
    SPI.endTransaction();
  }
}