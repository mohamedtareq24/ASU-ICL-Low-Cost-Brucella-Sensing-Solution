#include <driver/i2s.h>
// I2S
#define I2S_SAMPLE_RATE (48000) // this yields a smapling frequncy of 30Khz
#define REF (ADC1_CHANNEL_4) //pin 32
#define SUM (ADC1_CHANNEL_7) //pin 35
#define MEASURE (ADC1_CHANNEL_5) //pin 33

#define I2S_DMA_BUF_LEN (1024)

// Sample post processing
#define PRINT_ALL_VALUES
#define AVERAGE_EVERY_N_SAMPLES (3)
#include "DacESP32.h"
DacESP32 dac1(GPIO_NUM_26);

size_t bytes_read;
//////////////////////////////////////////////////////////////
int f[] = {109, 116, 126, 135, 148, 160, 173, 187, 202, 219,
           236, 256, 276, 299, 323, 349, 377, 408, 441, 477,
           516, 557, 603, 651, 704, 761, 823, 890, 962, 1000,
           1125, 1216, 1315, 1421, 1537, 1661, 1796, 1942, 2100,
           2270, 2454, 2653, 2869, 3102, 3353, 3626, 3920, 4238, 4582,
           4954, 5356, 5791, 6261, 6769, 7319, 7913, 8555, 9250, 10000
          };
int f_2[] = {124.4 , 281.8, 350 , 466 , 633, 786, 1.05E+03,  1.50E+03,  2.00E+03,  2.40E+03,  3.28E+03,  4.16E+03,  4.60E+03,  5.20E+03,  5.90E+03,  6.30E+03,  7.16E+03,  7.60E+03,  8.03E+03,  8.39E+03,  8.90E+03,  9.30E+03,  1.07E+04};
int f_2_size = sizeof(f_2);
int FREQ ;

int16_t mag_ref    [52]      = {0};
int16_t mag_measure[52]      = {0};
int16_t mag_sum    [52]      = {0};

double Z  [52] = {0};        // impedance Array / frequncy
double phi[52] = {0};       // phase / frequncy
double ref[52] = {0};
double sum[52] = {0};
double measure[52] = {0};

int16_t buffer1_1[1024]         = {0};        /// intializing new buffers // buff length is about 10 periods
int16_t buffer1_2[1024]         = {0};
int16_t buffer1_3[1024]         = {0};
int16_t buffer1_4[1024]         = {0};
int16_t buffer1  [4096]         = {0};

int16_t buffer2_1  [1024]       = {0};
int16_t buffer2_2  [1024]       = {0};
int16_t buffer2_3  [1024]       = {0};
int16_t buffer2_4  [1024]       = {0};
int16_t buffer2    [4096]       = {0};

int16_t buffer3_1  [1024]       = {0};
int16_t buffer3_2  [1024]       = {0};
int16_t buffer3_3  [1024]       = {0};
int16_t buffer3_4  [1024]       = {0};
int16_t buffer3    [4096]       = {0};

int16_t measur_signal [4096]    = {0};
int16_t ref_signal    [4096]    = {0};
int16_t sum_signal    [4096]    = {0};

const int avg_samples_num = ((4096) / AVERAGE_EVERY_N_SAMPLES ) + 1 ;
int16_t avg_ref [avg_samples_num];
int16_t avg_measure[avg_samples_num];
int16_t avg_sum [avg_samples_num];
int16_t avg_measure_lagged[avg_samples_num];

#define GROUP_SIZE 10
int mx_input [57] , mx_output [57] ;

double calculateAngle(int a, int b, int c) {
  double bst = (a * a ) + (b * b ) - (c * c) ;
  double mak = (2 * a * b);
  double cosC = bst / mak;
  double angleC = acos(cosC) * 180 / 3.1415;  // Using the arccosine function to get the angle in radians
  return angleC;
}

int16_t calculateAndRemoveDCOffset(int16_t *data, int dataSize)
{
  int16_t maxVal  = 0;
  int16_t minVal  = 4095;
  int16_t sum     = 0 ;

  for (int i = 50; i < dataSize - 50; ++i) {
    if (data[i] > maxVal && data[i] < 2200) {
      maxVal = data[i];
    }
    if (data[i] < minVal && data[i] > 1600) {
      minVal = data[i];
    }
  }

  int16_t dcOffset = (maxVal + minVal) / 2;
  //  for (int i = 0; i < dataSize; ++i) {
  //    Serial.printf(" %d \n", data[i]-dcOffset);
  //    delay(2);
  //  }
  int16_t acMax = maxVal - dcOffset;
  return acMax;
}



void setup() {
  Serial.begin(115200);
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate =  I2S_SAMPLE_RATE,              // The format of the signal using ADC_BUILT_IN
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
}

i2s_config_t i2s_config = {
  .dma_buf_len = I2S_DMA_BUF_LEN                /// configuring buffer length per the new FREQ
};
int FreqIndex =   0;
int calibrated =  0;

void loop() {
  if (!calibrated)
  {
    Serial.printf("START Calibrating ??");
    Serial.println();
    while (!Serial.available()) {}
  }
  else
  {
    Serial.printf("START SWEEPING ");
    Serial.println();
    while (!Serial.available()) {}
  }
  i2s_config_t i2s_config =
  {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate =  I2S_SAMPLE_RATE,              // The format of the signal using ADC_BUILT_IN
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  for (int FreqIndex = 0 ; FreqIndex < 23 ; FreqIndex++)
  {
    FREQ = f_2[FreqIndex];
    dac1.setCwFrequency(FREQ);      ///// genrating input
    dac1.setCwScale(DAC_CW_SCALE_8);
    dac1.outputCW(FREQ);
    delay(20);
    int num_samples_period = 0.1 * (90 * FREQ / 1000) ;


    i2s_set_adc_mode(ADC_UNIT_1, REF);
    adc1_config_channel_atten(REF, ADC_ATTEN_DB_11);

    i2s_adc_enable(I2S_NUM_0);
    i2s_read(I2S_NUM_0, &buffer1_1, sizeof(buffer1_1), &bytes_read, 30);
    i2s_read(I2S_NUM_0, &buffer1_2, sizeof(buffer1_2), &bytes_read, 30);
    i2s_read(I2S_NUM_0, &buffer1_3, sizeof(buffer1_3), &bytes_read, 30);
    i2s_read(I2S_NUM_0, &buffer1_4, sizeof(buffer1_4), &bytes_read, 30);
    i2s_adc_disable(I2S_NUM_0);

    //////////////////////////////////////////////////////////////////////////////////////
    //disable and reconfigure for channel 2/////////////////////////////////
    ////////////////////////////////////////////////sampling 2nd_channel/////////////////////////////////////////////////////////////////

    i2s_set_adc_mode(ADC_UNIT_1, MEASURE);
    adc1_config_channel_atten(MEASURE, ADC_ATTEN_DB_11);
    i2s_adc_enable(I2S_NUM_0);
    i2s_read(I2S_NUM_0, &buffer2_1, sizeof(buffer2_1), &bytes_read, 30);
    i2s_read(I2S_NUM_0, &buffer2_2, sizeof(buffer2_2), &bytes_read, 30);
    i2s_read(I2S_NUM_0, &buffer2_3, sizeof(buffer2_3), &bytes_read, 30);
    i2s_read(I2S_NUM_0, &buffer2_4, sizeof(buffer2_4), &bytes_read, 30);
    i2s_adc_disable(I2S_NUM_0);

    //////////////////////////////// channel 3/////////////////////////////////////////////

    i2s_set_adc_mode(ADC_UNIT_1, SUM);
    adc1_config_channel_atten(SUM, ADC_ATTEN_DB_11);
    i2s_adc_enable(I2S_NUM_0);
    i2s_read(I2S_NUM_0, &buffer3_1, sizeof(buffer3_1), &bytes_read, 30);
    i2s_read(I2S_NUM_0, &buffer3_2, sizeof(buffer3_2), &bytes_read, 30);
    i2s_read(I2S_NUM_0, &buffer3_3, sizeof(buffer3_3), &bytes_read, 30);
    i2s_read(I2S_NUM_0, &buffer3_4, sizeof(buffer3_4), &bytes_read, 30);
    i2s_adc_disable(I2S_NUM_0);


    for (int i = 0; i < 1024; i++) {
      ref_signal   [i]    =   buffer1_1[i]        & 0x0fff;
      measur_signal[i]    =   buffer2_1[i]        & 0x0fff;
      sum_signal   [i]    =   buffer3_1[i]        & 0x0fff;

    }
    for (int i = 1024; i < 2048  ; ++i) {
      ref_signal [i]    =  buffer1_2[i - 1024]  & 0x0fff;
      measur_signal[i]  =  buffer2_2[i - 1024]  & 0x0fff;
      sum_signal[i]     =  buffer3_2[i - 1024]  & 0x0fff;
    }
    for (int i = 2048; i < 3072  ; ++i) {
      ref_signal [i]    =  buffer1_3[i - 2048]  & 0x0fff;
      measur_signal[i]  =  buffer2_3[i - 2048]  & 0x0fff;
      sum_signal[i]     =  buffer3_3[i - 2048]  & 0x0fff;
    }
    for (int i = 3072; i <= 4096  ; ++i) {
      ref_signal [i]    =  buffer1_4[i - 3072]  & 0x0fff;
      measur_signal[i]  =  buffer2_4[i - 3072]  & 0x0fff;
      sum_signal[i]     =  buffer3_4[i - 3072]  & 0x0fff;
    }

    int32_t read_counter = 0;
    int16_t averaged_reading1 = 0;
    int16_t averaged_reading2 = 0;
    int16_t averaged_reading3 = 0;
    int64_t read_sum1 = 0;
    int64_t read_sum2 = 0;
    int64_t read_sum3 = 0;



    int counter = 0;
    for (int i = 0; i < 4096 ; ++i) {
      read_sum1 += ref_signal [i] & 0x0FFF;
      read_sum2 += measur_signal[i] & 0x0FFF;
      read_sum3 += sum_signal[i] & 0x0FFF;
      ++read_counter;

      if (read_counter == AVERAGE_EVERY_N_SAMPLES)
      {
        avg_ref [counter]     = read_sum1 / AVERAGE_EVERY_N_SAMPLES;
        avg_measure[counter]  = read_sum2 / (AVERAGE_EVERY_N_SAMPLES);
        avg_sum[counter]      = read_sum3 / (AVERAGE_EVERY_N_SAMPLES);

        read_counter = 0;
        read_sum1 = 0;
        read_sum2 = 0;
        read_sum3 = 0;
        counter++;
      }
    }


    mag_ref     [FreqIndex]     = calculateAndRemoveDCOffset (avg_ref , avg_samples_num)      ;

    mag_measure [FreqIndex]     = calculateAndRemoveDCOffset (avg_measure, avg_samples_num)   ;

    mag_sum     [FreqIndex]     = calculateAndRemoveDCOffset (avg_sum, avg_samples_num) * 2 ;

    //    Serial.printf("%d:", f_2[FreqIndex]);
    //    Serial.printf("  ref:%d", mag_ref[FreqIndex]);
    //    Serial.printf("  mesure:%d", mag_measure [FreqIndex]);
    //    Serial.printf("  sum:%d\n", mag_sum[FreqIndex]);

    Z           [FreqIndex]     = ( (double)  mag_ref[FreqIndex] / (double) mag_measure[FreqIndex] ) * 95000   ; /// 100000 is the TIA gain
    phi         [FreqIndex]     =  calculateAngle(mag_ref[FreqIndex], mag_measure[FreqIndex], mag_sum[FreqIndex])  ;


        for (int i = 10; i < avg_samples_num - 20  ; ++i) {
          Serial.printf("ref:%d, measure:%d , sum:%d , FreqIndex:%d , calibrated:%d", avg_ref[i], avg_measure[i], avg_sum[i] , FreqIndex, calibrated);
          delay(10);
          Serial.println();
        }

//    Serial.printf("ref_%d = [", FreqIndex);
//    for (int i = 10; i < avg_samples_num - 20; i++) {
//      Serial.print(avg_ref[i]);
//      Serial.printf(" ");
//    }
//    Serial.println("]; \n");
//    
//    Serial.printf("measure_%d = [", FreqIndex);
//    for (int i = 10; i < avg_samples_num - 20; i++) {
//      Serial.print(avg_measure[i]);
//      Serial.printf(" ");
//    }
//    Serial.println("];");
//
//    Serial.printf("sum_%d = [", FreqIndex);
//    for (int i = 10; i < avg_samples_num - 20; i++) {
//      Serial.print(avg_sum[i]);
//      Serial.printf(" ");
//    }
//    Serial.println("];");
    
//    while (!Serial.available()) {
//    }
//    Serial.println(Serial.read());
  }
//  Serial.print("Z = [");
//  for (int i = 0; i < 23; i++) {
//    Serial.print(Z[i]);
//    Serial.printf(" ");
//  }
//  Serial.println("];");
//
//  Serial.print("phi = [");
//  for (int i = 0; i < 23; i++) {
//    Serial.print(phi[i]);
//    Serial.printf(" ");
//  }
//  Serial.println("];");

  calibrated ++;
  if (calibrated == 1) {
    while (1) {
      //Serial.printf("DONE");
      //delay (5000);
    }
  }
}
