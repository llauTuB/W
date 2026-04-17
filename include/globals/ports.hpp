#pragma once
#include <cstdint>


    namespace ports {

inline constexpr std::int8_t DRIVE_RIGHT_FRONT = 19;
inline constexpr std::int8_t DRIVE_RIGHT_MIDDLE = 11;
inline constexpr std::int8_t DRIVE_RIGHT_BACK = 12;

inline constexpr std::int8_t DRIVE_LEFT_FRONT = -15;    
inline constexpr std::int8_t DRIVE_LEFT_MIDDLE = -16;
inline constexpr std::int8_t DRIVE_LEFT_BACK = -17;

inline constexpr std::int8_t INTAKE_STAGE1_PORT = 21;
inline constexpr std::int8_t INTAKE_STAGE2_PORT = 9;
inline constexpr std::int8_t INTAKE_STAGE3_PORT = -3;

inline constexpr std::int8_t IMU_PORT = 18;

inline constexpr std::int8_t OPTICAL_PORT = 1;


inline constexpr std::int8_t VERTICAL_ENCODER_PORT = -7; 
inline constexpr std::int8_t HORIZONTAL_ENCODER_PORT = -6; 

inline constexpr std::int8_t DISTANCE_RIGHT_PORT = 4;
inline constexpr std::int8_t DISTANCE_LEFT_PORT = 10;
inline constexpr std::int8_t DISTANCE_FRONT_PORT = 5;
inline constexpr std::int8_t DISTANCE_BACK_PORT = 8;

inline constexpr char LOADER_PORT = 'B';
inline constexpr char DESCORE_PORT = 'E';
inline constexpr char UPSCORE_PORT = 'A';
inline constexpr char INTAKE_UPPER_PORT = 'C';
inline constexpr char MIDDLE_DESCORE_PORT = 'D';
inline constexpr char ENCODER_UPPER_PORT = 'F';

}