#include "Barcodescanner.h"
#include <Arduino.h>

Barcodescanner::Barcodescanner(uint8_t sensorPin, uint16_t samplingrate)
{
    // sensor
    this->sensorPin = sensorPin;
    pinMode(sensorPin, INPUT);
    sensorValue[0] = analogRead(sensorPin);

    this->samplingrate = samplingrate;
    sampletime = 1e6 / samplingrate; // in us
}

void Barcodescanner::Basic_Measure()
{
    // measures the sensor value and calculates the edge time
    switch (Basic_State)
    {
    case Basic_MeasurementState::Reading:
        Basic_Reading();
        break;

    case Basic_MeasurementState::Covert:
        Basic_TimetoBits();
        break;

    case Basic_MeasurementState::Error:
        Basic_Error();
        break;

    case Basic_MeasurementState::Decode:
        Decode();
        break;

    default:
        break;
    };
}

void Barcodescanner::Advanced_Measure()
{
    // measures the sensor value and calculates the edge time
    switch (Advanced_State)
    {
    case Advanced_MeasurementState::Reading:
        Advanced_Reading();
        break;

    case Advanced_MeasurementState::Covert:
        Advanced_TimetoBits();
        break;

    case Advanced_MeasurementState::Error:
        Advanced_Error();
        break;

    case Advanced_MeasurementState::Decode:
        Decode();
        break;

    default:
        break;
    };
}

// Measurement
void Barcodescanner::SampleSensor()
{   
    // reads in the sensor value each sampletime
    // also keeps a register for gradient calculation
    if (micros() - lastSapleTime > sampletime)
    {
        lastSapleTime = micros();
        // shift the sensor values (FIFO)
        for(int i = 0; i < SENSORVAL_MEM_SIZE-1; i++)
        {
            sensorValue[i] = sensorValue[i+1];
        }
        sensorValue[SENSORVAL_MEM_SIZE-1] = analogRead(sensorPin); 
    }
}

// Data aquisition
void Barcodescanner::Basic_Reading()
{
    // get new value
    SampleSensor();
    
    if(n_sample == 0){
        t_change = micros(); 
    }
    // Check for threshold change
    if(sensorValue[SENSORVAL_MEM_SIZE-1] > UPPER_THRESHOLD && ((n_sample % 2) == 0))
    {
        // rising edge detected
        times[n_sample] = micros() - t_change;
        n_sample++;
        t_change = micros();
    
    }else if(sensorValue[SENSORVAL_MEM_SIZE-1] < LOWER_THRESHOLD && ((n_sample % 2) == 1))
    {
        // falling edge detected
        times[n_sample] = micros() - t_change;
        n_sample++;
        t_change = micros();
    }

    // change to next state
    if(n_sample >= NUM_TIMES)
    {
        Basic_State = Basic_MeasurementState::Covert;
        Serial.println("Covert");
    }

    // error when no edge is detected for a certain time
    if(micros() - t_change > TIMEOUT_READING)
    {
        errorType = ErrorTypes::ReadError;
        Basic_State = Basic_MeasurementState::Error;
    }
}

void Barcodescanner::Advanced_Reading()
{
    SampleSensor();

    if(n_sample == 0){
        t_change = micros(); 
        // detect first edge
        last_gardient = gradient;
        gradient = sensorValue[SENSORVAL_MEM_SIZE-1] - sensorValue[0]; // calculates over samplingrate*SENSORVAL_MEM_SIZE
        if( gradient > GRADIENT_STEP)
        {
            times[0] = micros(); // get start time
            t_change = micros(); 
        }
        // wait for the gradient to get smaller, for detection of upper threshold
        // this happens when the gradient is smaller and the lower and upper threshold are far enough apart
        if(last_gardient < gradient && gradient > GRADIENT_STEP )
        {
            n_sample++;
        }

    }else if(n_sample == 4) // error checking
    {
        // read as normal, but check the tollerance of the black stripes
        Barcodescanner::Detect_Edge();

        time_diff = abs(times[3] - times[1]); 
        if(time_diff > START_TIME_TOLERANCE)
        {
            errorType = ErrorTypes::ReadError;
            Advanced_State = Advanced_MeasurementState::Error;
            return;
        }

    }else if(n_sample == 44) // finished reading
    {
        Advanced_State = Advanced_MeasurementState::Covert;
        return;

    }else // normal reading
    {
        Barcodescanner::Detect_Edge();
    }

    // TIME OUT
    if(readingState == 0 && (micros() - t_change) > TIMEOUT_START)
    {
        errorType = ErrorTypes::ReadError;
        Advanced_State = Advanced_MeasurementState::Error;
    }else if(readingState == 1 && (micros() - t_change) > TIMEOUT_READING)
    {
        errorType = ErrorTypes::ReadError;
        Advanced_State = Advanced_MeasurementState::Error;
    }
}

void Barcodescanner::Detect_Edge()
{
    last_gardient = gradient;
    gradient = sensorValue[SENSORVAL_MEM_SIZE-1] - sensorValue[0];
    if(n_sample % 2 == 0)
    {
        if(gradient > GRADIENT_STEP && (gradient < last_gardient))
        {
            // rising edge detected
            times[n_sample] = micros() - t_change;
            n_sample++;
            t_change = micros();
        }
    }else{
        if(gradient < -GRADIENT_STEP && (gradient > last_gardient))
        {
            // falling edge detected
            times[n_sample] = micros() - t_change;
            n_sample++;
            t_change = micros();
        }
    }
}

// Conversion from the times to bits
void Barcodescanner::Basic_TimetoBits()
{
    // velocity in this context is the time taken to complete one bit
    // get reading velocity
    initialVelocity = (times[1] + times[2]+ times[3]) / 3;
    finalVelocity = (times[41] + times[42] + times[43]) / 3;
    avgVelocity = (initialVelocity + finalVelocity) / 2;
    
    // there are additional calculations in the advanced setting
    
    // round times to get the amout of bits for each stripe
    for(int i = 0; i < NUM_TIMES; i++)
    {
        times[i] = round(times[i] / avgVelocity);
    }
}

void Barcodescanner::Advanced_TimetoBits()
{
    // byte durations for local velocity
    // reset
    for(int i = 0; i < 8; i++)
    {
        byteDuration[i] = 0;
    }

    // colect times
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            // offset because of beginning marker
            byteDuration[i] += times[i*4+4+j]; 
        }
    }
    for(int i = 4; i < 8; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            // offset because of beginning and middle marker
            byteDuration[i] += times[i*4+9+j]; 
        }
    }

    // covert time to local velocity
    for(int i = 0; i < 8; i++)
    {
        byteDuration[i] = byteDuration[i] / 7;
    }

    // decode the bits
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            // offset because of beginning marker
            times[i*4+4+j] = round(times[i*4+4+j] / (float)byteDuration[i]);
        }
    }

    for(int i = 4; i < 8; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            // offset because of beginning and middle marker
            times[i*4+9+j] = round(times[i*4+9+j] / (float)byteDuration[i]);
        }
    }
    
    Advanced_State = Advanced_MeasurementState::Decode;
}

// basic error handling
void Barcodescanner::Basic_Error()
{
    // error handling
    switch (errorType)
    {
    case ErrorTypes::ReadError:
        Serial.println("Read Error");
        break;

    case ErrorTypes::DecodeError:
        Serial.println("Decode Error");
        break;

    default:
        break;
    }

    // reset state
    errorType = ErrorTypes::NoError;
    Basic_State = Basic_MeasurementState::Reading;

    // reset the values
    n_sample = 0;
    for(int i = 0; i < NUM_TIMES; i++)
    {
        times[i] = 0;
    }
}

void Barcodescanner::Advanced_Error()
{
    // error handling
    switch (errorType)
    {
    case ErrorTypes::ReadError:
        Serial.println("Read Error");
        break;

    case ErrorTypes::DecodeError:
        Serial.println("Decode Error");
        break;

    default:
        break;
    }

    // reset state
    errorType = ErrorTypes::NoError;
    Advanced_State = Advanced_MeasurementState::Reading;

    // reset the values
    n_sample = 0;
    for(int i = 0; i < NUM_TIMES; i++)
    {
        times[i] = 0;
    }
}

// Decode to decimal    
void Barcodescanner::Decode()
{
    // put the times into the bytes
    
    // left
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            // offset because of beginning marker
            bytes[i][j] = times[i*4+4+j]; 
        }
        
    }
    // right
    for(int i = 4; i < 8; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            // offset because of beginning and middle marker
            bytes[i][j] = times[i*4+9+j]; 
        }
        
    }
    // printout bytes
    Serial.println("Bytes: ");
    for(int i = 0; i < 8; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            Serial.print(bytes[i][j]);
        }
        Serial.println(" ");
    }
    // convert bytes to number
    
    // we check if the first byte is in the lookup table
    // if not we assume that the barcode is read in the other direction
    readingdir = 1;
    for(int i = 0; i < 10; i++)
    {
        if(Barcodescanner::Check_Array(ean_lookup[i], bytes[0], 4))
        {
            readingdir = 0;
            value[0] = i;
        }
    }

    // continue with the decoding
    // raise an error if the barcode does not match

    // left to right
    if(readingdir == 0)
    {
        for(int i = 0; i < 8; i++)
        {
            for(int j = 0; j < 10; j++)
            {
                if(Barcodescanner::Check_Array(ean_lookup[j], bytes[i], 4))
                {
                    value[i] = j;
                    Serial.print(i);
                    Serial.print(" Value: ");
                    Serial.println(j);
                    break;
                }

                // raise error
                if(j == 9 && 
                !(Barcodescanner::Check_Array(ean_lookup[j], bytes[i], 4)))
                {
                    errorType = ErrorTypes::DecodeError;
                    Basic_State = Basic_MeasurementState::Error;
                    Advanced_State = Advanced_MeasurementState::Error;
                    return;
                }
            }
        }
    // right to left
    }else{
        for(int i = 0; i < 8; i++)
        {
            for(int j = 0; j < 10; j++)
            {
                if(Barcodescanner::Check_Array(ean_reversed_lookup[j], bytes[i], 4))
                {
                    value[i] = j;
                    break;
                }

                // raise error
                if(j == 9 && 
                !(Barcodescanner::Check_Array(ean_reversed_lookup[j], bytes[i], 4)))
                {
                    errorType = ErrorTypes::DecodeError;
                    Basic_State = Basic_MeasurementState::Error;
                    Advanced_State = Advanced_MeasurementState::Error;
                    return;
                }
            }
        }
        // reverse ordering
        uint8_t temp[8];
        for(int i = 0; i < 8; i++)
        {
            temp[i] = value[7-i];
        }

        for(int i = 0; i < 8; i++)
        {
            value[i] = temp[i];
        }
    }

    // checksum calculation
    checksum = 0;
    for(int i = 0; i < 8; i++)
    {
        if(i % 2 == 0)
        {
            checksum += value[i] * 3;
        }else{
            checksum += value[i];
        }
    }
    Serial.print("Checksum: ");
    Serial.println(checksum);

    // check if the checksum is correct
    if(checksum % 10 != 0)
    {
        errorType = ErrorTypes::DecodeError;
        Basic_State = Basic_MeasurementState::Error;
        Advanced_State = Advanced_MeasurementState::Error;
        return;
    }

    // Decoding was successful 
    // printout the result
    Serial.print("Barcode: ");
    for(int i = 0; i < 8; i++)
    {
        Serial.print(value[i]);
    }
    Serial.println(" ");

    // Change back to reading
    Basic_State = Basic_MeasurementState::Reading;
    Advanced_State = Advanced_MeasurementState::Reading;

    // reset the values
    n_sample = 0;
    for(int i = 0; i < NUM_TIMES; i++)
    {
        times[i] = 0;
    }
}

bool Barcodescanner::Check_Array(uint8_t* a, uint8_t* b, uint8_t size)
{
    // compare two arrays given their start
    for(int i = 0; i < size; i++)
    {
        if(a[i] != b[i])
        {
            return false;
        }
    }
    return true;
}
