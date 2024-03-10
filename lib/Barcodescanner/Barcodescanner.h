#pragma once

#include <Arduino.h>

/*
Disclaimer for this project
The advanced reading function works fine, so should the basic one
Only to types of errors are handled at the time this can be expanded
but this project is finished for now
*/

/*
This is the header file for the Barcodescanner class
It has two reading functionalities, Basic and Advanced
They can be called via the Measure() functions
In the future this might change and the reading mode needs to be selected in the constructor
*/

class Barcodescanner
{
    public:
        // consturctor
        Barcodescanner(uint8_t sensorPin, uint16_t samplingrate);
        
        /**
        Measurement type
        Basic: fixed threshholds, no initial error protection, simple time tobit calculation
        Advanced:   edge detection,
                    time calculation in regions
        */
        void Basic_Measure();
        void Advanced_Measure();

    private:
    /*
    The constants and variables are split into diffrent sections
    */
    // General
        // constants
        static constexpr uint8_t SENSORVAL_MEM_SIZE = 10; 
        static constexpr uint8_t NUM_TIMES = 44;
        static constexpr uint32_t TIMEOUT_READING = 1000000; // 1 s

    // Measurement states
        /**
        State variables are used to determine the current State of the measurement
        in the corresponding function Baisc_Measure() or Advanced_Measure()
        */

        enum class Basic_MeasurementState
        {
            Reading,
            Covert,
            Error,
            Decode
        };
        Basic_MeasurementState Basic_State = Basic_MeasurementState::Reading;

        enum class Advanced_MeasurementState
        {
            Reading,
            Covert,
            Check,
            Error,
            Decode
        };
        Advanced_MeasurementState Advanced_State = Advanced_MeasurementState::Reading;

    // Sensor
        // variables
        uint8_t sensorPin;
        uint16_t sensorValue[SENSORVAL_MEM_SIZE];
        uint16_t samplingrate = 1000; // 1kHz 
        uint32_t sampletime = 1e6 / samplingrate; // 1ms,
        uint32_t lastSapleTime = 0;

        // functions
        void SampleSensor();

    // Data aquisition
        /*
        Time is saved relative to the previous edge
        */
        // variables
        uint32_t times[NUM_TIMES]; // 44 edges for EAN-8
        uint32_t t_change = 0; // keeps time of the last change
        uint8_t n_sample = 0;

        // functions and helper variables, basic and advanced
        // Basic
        static constexpr uint16_t UPPER_THRESHOLD = 790;
        static constexpr uint16_t LOWER_THRESHOLD = 775;
        void Basic_Reading();

        // Advanced
        // The Tolerances are choose as absolute values for faster computation
        static constexpr int16_t GRADIENT_STEP = 40; 
        static constexpr uint32_t TIMEOUT_START = 500000; // 500 ms
        int16_t gradient = 0; // calculates the slope of the signal
        int16_t last_gardient = 0; // keeps the last gradient
        uint8_t readingState = 0; // 0: waiting, 1: reading
        // start check
        uint16_t time_diff = 0;
        static constexpr uint32_t START_TIME_TOLERANCE = 30000; // 30 ms
        void Advanced_Reading();
        void Detect_Edge();

    // Time to Bit conversion
        // functions
        // Basic
        uint16_t initialVelocity = 0;
        uint16_t finalVelocity = 0;
        uint16_t avgVelocity = 0;
        void Basic_TimetoBits();

        // Advanced
        static constexpr uint8_t TIME_TOLERANCE = 30;
        uint32_t byteDuration[8];
        void Advanced_TimetoBits();

        
    // Error handling
        enum class ErrorTypes
        {   
            NoError,
            StartError,
            ReadError,
            DecodeError,
            /**
            @todo check if additional error types are needed
            */
        };

        ErrorTypes errorType = ErrorTypes::NoError;

        void Basic_Error();
        void Advanced_Error();

    // Decoding
        // reading dir should prevent the misinterpretation of the barcode
    uint8_t readingdir = 0; // 0: left to right, 1: right to left
    uint8_t bytes[8][4]; // binary values to store the bits of each digit in the barcode
    uint8_t value[8]; // decimal values of the barcode
    uint8_t checksum = 0; // checksum of the barcode

    uint8_t ean_lookup[10][4] = {
        {3, 2, 1, 1}, // 0
        {2, 2, 2, 1}, // 1
        {2, 1, 2, 2}, // 2
        {1, 4, 1, 1}, // 3
        {1, 1, 3, 2}, // 4
        {1, 2, 3, 1}, // 5
        {1, 1, 1, 4}, // 6
        {1, 3, 1, 2}, // 7
        {1, 2, 1, 3}, // 8
        {3, 1, 1, 2}, // 9
    };

    uint8_t ean_reversed_lookup[10][4] = {
        {1, 1, 2, 3}, // 0
        {1, 2, 2, 2}, // 1
        {2, 2, 1, 2}, // 2
        {1, 1, 4, 1}, // 3
        {2, 3, 1, 1}, // 4
        {1, 3, 2, 1}, // 5
        {4, 1, 1, 1}, // 6
        {2, 1, 3, 1}, // 7
        {3, 1, 2, 1}, // 8
        {2, 1, 1, 3}, // 9
    };

    // functions
    void Decode();
    bool Check_Array(uint8_t* a, uint8_t* b, uint8_t size);
};