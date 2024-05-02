#include <mbed.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib> // for rand()

// Function to read gyroscope data (simulated for demonstration)
std::vector<std::vector<float>> readGyroscopeData() {
    std::vector<std::vector<float>> gyro_data;
    return gyro_data;
}

// Function to calculate mean amplitude of signal
float calculateMeanAmplitude(const std::vector<float>& signal) {
    float peak_to_peak_amp = std::abs(*std::max_element(signal.begin(), signal.end()) -
                                       *std::min_element(signal.begin(), signal.end()));
    return peak_to_peak_amp / signal.size();
}

// Function to calculate average regularity of signal
float calculateAverageRegularity(const std::vector<float>& signal) {
    float sum = 0.0f;
    for (size_t i = 1; i < signal.size(); ++i) {
        sum += std::abs(signal[i] - signal[i - 1]);
    }
    return sum / (signal.size() - 1);
}

// Function to detect tremor based on gyroscope data
bool detectTremor(const std::vector<float>& signal) {
    // Calculate frequency of signal (assuming sampling rate of 125 Hz)
    float sampling_rate = 125.0f;
    float signal_size = static_cast<float>(signal.size());
    float frequency = (signal_size / sampling_rate);

    // Calculate mean amplitude and average regularity
    float mean_amplitude = calculateMeanAmplitude(signal);
    float average_regularity = calculateAverageRegularity(signal);

    // Define thresholds for amplitude and regularity
    float amplitude_threshold = 0.1f; // Example threshold for mean amplitude
    float regularity_threshold = 0.01f; // Example threshold for average regularity

    // Check if frequency, mean amplitude, or average regularity exceed thresholds
    if ((frequency >= 3.0f && frequency <= 6.0f) || mean_amplitude > amplitude_threshold || average_regularity < regularity_threshold) {
        return true; // Tremor detected
    }

    return false; // No tremor detected
}

DigitalOut l1(LED1), l2(LED2);

void toggle() {
    l1 = !l1;
    l2 = !l2;
}

int main() {
    // Read gyroscope data (simulated for demonstration)
    // std::vector<std::vector<float>> gyro_data = readGyroscopeData();

    // Iterate over axes and detect tremor for each axis
    // bool tremor_detected = false;
    // for (int i = 0; i < 3; ++i) {
    //     tremor_detected = detectTremor(gyro_data[i]);
    //     if (tremor_detected) {
    //         break; // Exit loop if tremor detected in any axis
    //     }
    // }

    // bool tremor_detected = false;
    // for (size_t i = 0; i < gyro_data.size(); ++i) {
    //     tremor_detected = detectTremor(gyro_data[i]);
    //     if (tremor_detected) {
    //         break; // Exit loop if tremor detected in any axis
    //     }
    // }

    // // Output whether tremor was detected
    // if (tremor_detected) {
    //     std::cout << "Tremor detected." << std::endl;
    // } else {
    //     std::cout << "No tremor detected." << std::endl;
    // }

    // return 0;
    InterruptIn btn(BUTTON1, PullDown);
    btn.rise(&toggle);

    while(1) {

    }
}
