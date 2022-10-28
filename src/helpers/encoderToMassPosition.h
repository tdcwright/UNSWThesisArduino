#ifndef ENCODER_MASS_POSITION_H
#define ENCODER_MASS_POSITION_H

#define ENCODER_TO_OUTPUT_SHAFT_RATIO   1/360.0
#define MOTOR_PULLEY_TO_ROD_PULLY_RATIO 15/36.0
#define ROD_THREAD_PITCH 1.5

// Convert encoder reading to the position of the mass using the gearing ratios
double readingToPosition(long encoderPosition){
    return encoderPosition*\
            ENCODER_TO_OUTPUT_SHAFT_RATIO*\
            MOTOR_PULLEY_TO_ROD_PULLY_RATIO*\
            ROD_THREAD_PITCH;
}

#endif