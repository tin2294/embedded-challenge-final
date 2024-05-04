#include <mbed.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28

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
#define THRESHOLD 5000.0f
#define MIN_CYCLES_PER_SECOND 3
#define MAX_CYCLES_PER_SECOND 6

Timer cycleTimer;
bool lastAboveThreshold = false;
int cycleCount = 0;

void initializeGyroscope(SPI& spi)
{
    spi.format(8, 3);
    spi.frequency(1'000'000);

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

    printf("Gyroscope Values -> gx: %u, gy: %u, gz: %u\n", raw_gx, raw_gy, raw_gz);
}

bool isTremorDetected(float magnitude)
{
    static bool timerStarted = false;
    static int cycleCount = 0;
    static Timer cycleTimer;

    if (magnitude > THRESHOLD && !timerStarted)
    {
        // Start of a cycle
        timerStarted = true;
        cycleCount++;
        cycleTimer.reset();
        cycleTimer.start();
    }
    else if (magnitude <= THRESHOLD && timerStarted)
    {
        // End of a cycle
        timerStarted = false;
        cycleTimer.stop();
        if (cycleTimer.read_ms() >= 1000)
        {
            // Check if the number of cycles per second falls within the tremor frequency range
            int cyclesPerSecond = cycleCount;
            cycleCount = 0;
            cycleTimer.reset();
            if (cyclesPerSecond >= MIN_CYCLES_PER_SECOND && cyclesPerSecond <= MAX_CYCLES_PER_SECOND)
            {
                return true; // Tremor detected
            }
        }
    }

    return false; // No tremor detected
}

int main() {
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    initializeGyroscope(spi);

    float gx, gy, gz;

    while(1) {
        readGyroscope(spi, gx, gy, gz);
        float angularVelocity = sqrt(gx * gx + gy * gy + gz * gz);

        // Add low pass filtering

        // Check if a tremor is detected
        if (isTremorDetected(angularVelocity)) {
            // Light up an LED or use any other visual indication
            // to signal the presence of a tremor
            // For example, you could use onboard LEDs like this:
            // myled1 = 1; // turn the LED on
            printf("Tremor Detected!\n");
            redLED = 1;
            greenLED = 0;
        } else {
            // Turn off the LED if no tremor is detected
            // myled1 = 0; // turn the LED off
            printf("No Tremor\n");
            redLED = 0;
            greenLED = 1;
        }

        thread_sleep_for(100);
    }

    return 0;
}