#include "Arduino.h"

// pinout es32 d1 r32
// #if defined(ESP_H)
  // driver pinouts LolinD32 
  // #define MOT1_A 32    // 5
  // #define MOT1_B 33     // 10
  // #define MOT1_C 25    // 6
  // #define MOT1_EN 27   // 8
  // #define MOT2_A 12    // 3
  // #define MOT2_B 13    // 9
  // #define MOT2_C 14    // 11
  // #define MOT2_EN 27   // 7

  // driver pionouts ESP32-C6
  #define MOT1_A 0    // 5
  #define MOT1_B 2     // 10
  #define MOT1_C 3    // 6
  #define MOT1_EN 1   // 8
  #define MOT2_A 4    // 3
  #define MOT2_B 5    // 9
  #define MOT2_C 6    // 11
  #define MOT2_EN 7   // 7

  // encoder pinouts
//   #define ENC1_A 26    // 2
//   #define ENC1_B 17    // 4
//   #define ENC2_A 36    // A4
//   #define ENC2_B 19    // 12
// #else // nucleo/arduino pinout

// // driver pionouts
//   #define MOT1_A  5
//   #define MOT1_B  10
//   #define MOT1_C  6
//   #define MOT1_EN 8
//   #define MOT2_A  3
//   #define MOT2_B  9
//   #define MOT2_C  11
//   #define MOT2_EN 7
//   // encoder pinouts
//   #define ENC1_A 2
//   #define ENC1_B 4
//   #define ENC2_A A4
//   #define ENC2_B 12
  
// #endif