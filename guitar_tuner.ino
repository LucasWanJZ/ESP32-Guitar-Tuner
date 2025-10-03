//Libraries 
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <math.h>


#define I2S_WS  16
#define I2S_DIN  14
#define I2S_BCK 17
#define I2S_PORT I2S_NUM_0
#define log2(x) log(x)/log(2)

#define bufferLen 512
#define FFT_SAMPLE bufferLen
#define SAMPLING_FREQUENCY 10000

int32_t audioPlaceHolder[bufferLen];
arduinoFFT FFT = arduinoFFT();

double vReal[FFT_SAMPLE];
double vImag[FFT_SAMPLE];
unsigned int samplingPeriod;

String notes[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
double SF1[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

TaskHandle_t audioReadingTask;
TaskHandle_t audioProcessingTask;
SemaphoreHandle_t audioDataReady;

void setup() {
  Serial.begin(115200);
  samplingPeriod = round(1000000 * (1.0/SAMPLING_FREQUENCY));
  initI2S();
  calcSF1();
  Serial.println(SF1[4]);

  audioDataReady = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(ReadingTask,"audio_reading",4096,NULL,1,&audioReadingTask,0);

  xTaskCreatePinnedToCore(ProcessingTask,"audio_processing",4096,NULL,1,&audioProcessingTask,1);
}

void loop() 
{
  // Not Needed because for(;;) is used for both task 
}

// I2S Setup 
void initI2S() 
{
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 10000,
    .bits_per_sample = i2s_bits_per_sample_t(32),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  
  
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_DIN
  };

  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_start(I2S_PORT);
}

// Calculate all standard frequencies of the notes in 1st Octave by using A4 note (440 Hz)
void calcSF1() 
{
  double semitones = -33.0; // C0 is located 45 semitones under A4 
  for (int i = 0; i < 12; i++) 
  {
    SF1[i] = 440 * pow(2.0,semitones/12.0);
    semitones++;
  }  
}

// AUDIO READING
void ReadingTask(void* parameter)
{
  int32_t local[bufferLen];
  size_t localIdx = 0;

  for (;;)
  {
    int32_t sample = 0;
    size_t bytesIn = 0;
    i2s_read(I2S_PORT, &sample, sizeof(sample), &bytesIn, portMAX_DELAY);

    // Write the sample to the local buffer
    local[localIdx] = sample;
    localIdx++;

    vTaskDelay(pdMS_TO_TICKS(samplingPeriod / 1000)); 
    
    // When local buffer is full, send it to processing task 
    if (localIdx >= bufferLen)
    {
      memcpy(audioPlaceHolder, local, sizeof(int32_t) * bufferLen);
      localIdx = 0;
      xSemaphoreGive(audioDataReady);
    }
  }
}


// AUDIO PROCESSING 
void ProcessingTask(void* parameter)
{
  for (;;)
  {
    // Wait for new audio data
    if (xSemaphoreTake(audioDataReady, portMAX_DELAY) == pdTRUE)
    {
      for (size_t i = 0; i < bufferLen; i++) {
        int32_t sample = audioPlaceHolder[i];
        vReal[i] = sample;
        vImag[i] = 0;
      }
    
      // Perform FFT 
      FFT.DCRemoval();
      FFT.Windowing(vReal, FFT_SAMPLE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal,vImag, FFT_SAMPLE,FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, FFT_SAMPLE);

      // Find the major peak frequency within FFT Results
      double peak = FFT.MajorPeak(vReal, FFT_SAMPLE,SAMPLING_FREQUENCY);

      // Only process peak if the note is between E2 and A4
      if (peak > 75 && peak <= 440) {
        noteDetection(peak);
      }
    }
  }
}

// Determine the correct note and its octave from the detected frequency 
void noteDetection(double peak) {
    int midi = calcSemitoneFromA4(peak);

    // Find the corresponding octave 
    int octave = log2(round((peak/SF1[midi%12]))) + 2;

    //find the dev
    double deviation = ((SF1[midi%12] * pow(2.0,double(octave - 2.0))) - peak); 
    
    Serial.println(notes[midi%12] + "(" + octave + ") :" + peak);

    if (deviation >   2) {
      Serial.println("Tighten the String !");
    } else if (deviation < -2) {
      Serial.println("Loosen the String !");
    }
}

// Calculate the number of semitones under A4 using the detected frequencies 
int calcSemitoneFromA4(double frequency) 
{
  return round(12.0 * log2(frequency/440.0) + 69.0);
}


