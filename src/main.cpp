#include <mbed.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <arm_math.h>
#include <drivers/LCD_DISCO_F429ZI.h>

#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'00'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

// Define window size for readings to determine whether they are tremors
#define WINDOW_SIZE 256
#define TIME_ELAPSE 10
#define FREQ_HIGH 6
#define FREQ_LOW 3

// #define THRESHOLD 20.0f
#define THRESHOLD 1
#define THRESHOLD_FRE 1

// Define RGB LED pins
DigitalOut greenLED(PG_13);
DigitalOut redLED(PG_14);

// Define SPI flags
EventFlags flags;

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

Timer cycleTimer;
bool lastAboveThreshold = false;
int cycleCount = 0;

void initializeGyroscope(SPI& spi)
{
    spi.format(8, 3);
    spi.frequency(1'000'000);//check

    uint8_t write_buf[2], read_buf[2];

    // Configure CTRL_REG1 register
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Configure CTRL_REG4 register
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);
}

void readGyroscope(SPI& spi, float& gx, float& gy, float& gz)
{
    uint8_t write_buf[7], read_buf[7];
    write_buf[0] = OUT_X_L | 0x80 | 0x40;
    spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
    flags.wait_all(SPI_FLAG);

    uint16_t raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t) read_buf[1]);
    uint16_t raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t) read_buf[3]);
    uint16_t raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t) read_buf[5]);

    gx = ((float) raw_gx) * SCALING_FACTOR;
    gy = ((float) raw_gy) * SCALING_FACTOR;
    gz = ((float) raw_gz) * SCALING_FACTOR;
}

#define WINDOW_DURATION_MS 2000
#define WINDOW_COUNT_THRESHOLD 3
#define SAMPLES_PER_WINDOW 40
#define TREMOR_WINDOW_COUNT_THRESHOLD 3

arm_rfft_fast_instance_f32 S;
void fft(float magnitudes[], float magnitudes_FFT[]) {
    arm_rfft_fast_init_f32(&S, WINDOW_SIZE);
    arm_rfft_fast_f32(&S, magnitudes, magnitudes_FFT, 0);
}

bool isTremorDetected_freq(float magnitudes_FFT[WINDOW_SIZE]){

    float sum = 0.0f;

    printf("Calling the function in FFT...\n");

    int low = (WINDOW_SIZE*TIME_ELAPSE*FREQ_LOW)/1000;
    int high = (WINDOW_SIZE*TIME_ELAPSE*FREQ_HIGH)/1000;

    //  Calculate the sum of magnitudes from index 8 to 16
    for (int i = low; i <= high; ++i) {
        sum += magnitudes_FFT[i];
    }

    // If the sum is greater than the threshold, return true; otherwise, return false
    if (sum > THRESHOLD_FRE) {
        printf("Tremor detected in FFT judgement! \n");
        return true;
    }

    printf("Tremor NOT detected in FFT judgement! \n");

    return false;
}

int windowCount = 0;

bool isTremorDetected(float magnitudes[WINDOW_SIZE], int* windowCount)
{
    // static int windowCount = 0;
    static int flaggedWindowCount = 0;

    printf("isTremorDetected in time domain is called... \n");
    printf("Looking at the suspected window count (windowCount): %d \n", *windowCount);

    int aboveThresholdCount = 0;
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        if (magnitudes[i] > THRESHOLD)
        {
            // printf("One over threshold gyro value dectected! Now aboveThresholdCount=%d \n", aboveThresholdCount);
            aboveThresholdCount++;
        }
    }

    if (aboveThresholdCount > WINDOW_SIZE / 2)
    {
        // This window is flagged
        // printf("window count: %d \n", windowCount);
        (*windowCount)++;
        // printf("WindowCount plus! Now # of suspected windows = %d \n", *windowCount);
    }

    if (*windowCount >= WINDOW_COUNT_THRESHOLD)
    {
        // Enough windows have been flagged, check if majority are flagged
        printf("Enough suspected windows detected! windowCount=%d \n", *windowCount);
        if (aboveThresholdCount > WINDOW_SIZE / 2)
        {
            flaggedWindowCount++;
        }

        if (flaggedWindowCount >= TREMOR_WINDOW_COUNT_THRESHOLD)
        {
            // Tremor detected
            *windowCount = 0;
            flaggedWindowCount = 0;
            printf("Tremor detected in time domain judgement! \n");
            return true;
        }
    }

    printf("Tremor NOT detected in time domain judgement! \n");

    return false;
}

int main() {
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    initializeGyroscope(spi);

    greenLED = 0;
    redLED = 0;
    LCD_DISCO_F429ZI lcd;

    float gx, gy, gz;
    float magnitudes[WINDOW_SIZE];
    float magnitudes_FFT[WINDOW_SIZE];
    int* windowCount_main = &windowCount;
    int currentIndex = 0;

    lcd.Clear(LCD_COLOR_BLUE);
    lcd.SetBackColor(LCD_COLOR_BLUE);
    lcd.SetTextColor(LCD_COLOR_WHITE);

    while(1) {
        readGyroscope(spi, gx, gy, gz);
        // float magnitude = sqrt(gx*gx + gy*gy + gz*gz);

        float multiple = gx*gy*gz;
        int a = 0;
        if (multiple < 0) {
            a = -1;
        } else if (multiple == 0) {
            a = 0;
        } else {
            a = 1;
        }
        float magnitude = sqrt(gx*gx + gy*gy + gz*gz)*a;

        magnitudes[currentIndex] = magnitude;
        currentIndex++;

        if (windowCount >= 10) {
            windowCount = 0;
            printf(" window count = 0 now \n");
        }

        if (currentIndex >= WINDOW_SIZE)
        {
            if (isTremorDetected(magnitudes, windowCount_main) || isTremorDetected_freq(magnitudes_FFT)) {
                printf("Tremor Detected!\n");
                lcd.Clear(LCD_COLOR_BLACK);
                lcd.SetBackColor(LCD_COLOR_RED);
                lcd.SetTextColor(LCD_COLOR_WHITE);
                lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Tremor detected!", CENTER_MODE);
                redLED = 1;
                greenLED = 0;
            } else {
                printf("No Tremor\n");
                lcd.Clear(LCD_COLOR_GREEN);
                lcd.SetBackColor(LCD_COLOR_GREEN);
                lcd.SetTextColor(LCD_COLOR_BLACK);
                lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Everything's good!", CENTER_MODE);
                redLED = 0;
                greenLED = 1;
            }
            currentIndex = 0;
        }
        thread_sleep_for(TIME_ELAPSE);
    }

    return 0;
}
