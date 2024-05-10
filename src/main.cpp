#include <mbed.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <math.h>
#include <stdlib.h>

#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28

// Define window size for readings to determine whether they are tremors
#define WINDOW_SIZE 100

// Define RGB LED pins
DigitalOut greenLED(PG_13);
DigitalOut redLED(PG_14);

// Define SPI flags
EventFlags flags;

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)
#define THRESHOLD 20.0f

// we do not really need frequency
// #define MIN_CYCLES_PER_SECOND 3
// #define MAX_CYCLES_PER_SECOND 6

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

    // printf("Gyroscope Values -> gx: %u, gy: %u, gz: %u\n", raw_gx, raw_gy, raw_gz);
}

#define WINDOW_DURATION_MS 2000
#define WINDOW_COUNT_THRESHOLD 5
#define SAMPLES_PER_WINDOW 40
#define TREMOR_WINDOW_COUNT_THRESHOLD 3

bool isTremorDetected(float magnitudes[WINDOW_SIZE])
{
    static int windowCount = 0;
    static int flaggedWindowCount = 0;

    int aboveThresholdCount = 0;
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        if (magnitudes[i] > THRESHOLD)
        {
            aboveThresholdCount++;
        }
    }

    if (aboveThresholdCount > WINDOW_SIZE / 2)
    {
        // This window is flagged
        windowCount++;
    }

    if (windowCount >= WINDOW_COUNT_THRESHOLD)
    {
        // Enough windows have been flagged, check if majority are flagged
        if (aboveThresholdCount > WINDOW_SIZE / 2)
        {
            flaggedWindowCount++;
        }

        if (flaggedWindowCount >= TREMOR_WINDOW_COUNT_THRESHOLD)
        {
            // Tremor detected
            windowCount = 0;
            flaggedWindowCount = 0;
            return true;
        }
    }
    else
    {
        // Reset counters if not enough windows are flagged
        windowCount = 0;
        flaggedWindowCount = 0;
    }

    return false;
}


int main() {
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    initializeGyroscope(spi);

    float gx, gy, gz;
    float magnitudes[WINDOW_SIZE];
    int currentIndex = 0;

    while(1) {
        readGyroscope(spi, gx, gy, gz);
        float magnitude = sqrt(gx*gx + gy*gy + gz*gz);
        printf("Magnitude -> %.2f\n", magnitude);

        // Store the magnitude in the array
        magnitudes[currentIndex] = magnitude;
        currentIndex++;

        // Add low pass filtering


        // Update magnitude data in Teleplot
        // teleplot.update("magnitude", magnitude, "m/s²");

        // If the window is filled, analyze the readings for tremors
        if (currentIndex >= WINDOW_SIZE)
        {
            // Check if a tremor is detected
            if (isTremorDetected(magnitudes)) {
                printf("Tremor Detected!\n");
                redLED = 1;
                greenLED = 0;
            } else {
                printf("No Tremor\n");
                redLED = 0;
                greenLED = 1;
            }
            currentIndex = 0;
        }
        thread_sleep_for(1000);
    }

    return 0;
}


// teleplot it to determine the threshold
// is frequency needed --> frequency is not really needed if we have cycles and magnitude
// take about maybe 20-50 samples per sec? -> find threshold and how many of them are they positives and then determine
// so for example if we sample for 2 seconds about 20 samples, we have 40 samples, we flag as yes or no and then
// flag that window, then the next window and so on, up until the duration of a parkinson tremor, then we say there is a tremor or not
// watch video of patient
// how long is a parkinson's tremor? needed to know how many windows would determine there is a tremor


// Algorithm:
// take magnitude and teleplot it to find threshold
// take 20 samples per second for two seconds
// if they are above the threshold, we flag this window
// after a few windows (dependent on the typical duration of a parkinsonian tremor), if i.e. majority of them are flagged yes,
// there is a tremor
