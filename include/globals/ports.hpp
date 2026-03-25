#pragma once
#include <cstdint>


    namespace ports {

inline constexpr std::int8_t DRIVE_RIGHT_FRONT = 18;
inline constexpr std::int8_t DRIVE_RIGHT_MIDDLE = 19;
inline constexpr std::int8_t DRIVE_RIGHT_BACK = 17;
inline constexpr std::int8_t DRIVE_LEFT_FRONT = -11;
inline constexpr std::int8_t DRIVE_LEFT_MIDDLE = -6;
inline constexpr std::int8_t DRIVE_LEFT_BACK = -7;

inline constexpr std::int8_t INTAKE_LOW_PORT = 21;
inline constexpr std::int8_t INTAKE_HIGH1_PORT = 9;
inline constexpr std::int8_t INTAKE_HIGH2_PORT = -14;

inline constexpr std::int8_t IMU_PORT = 12;

inline constexpr std::int8_t VERTICAL_ENCODER_PORT = 13; 
inline constexpr std::int8_t HORIZONTAL_ENCODER_PORT = 20; 

inline constexpr std::int8_t DISTANCE_RIGHT_PORT = 8;
inline constexpr std::int8_t DISTANCE_LEFT_PORT = 2;
inline constexpr std::int8_t DISTANCE_FRONT_PORT = 4;
inline constexpr std::int8_t DISTANCE_BACK_PORT = 10;

inline constexpr char LOADER_PORT = 'B';
inline constexpr char DESCORE_PORT = 'A';
inline constexpr char UPSCORE_PORT = 'D';
inline constexpr char DOUBLE_PARK_PORT = 'E';
}