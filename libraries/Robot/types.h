#ifndef __TYPES_H__
#define __TYPES_H__

#include <stdint.h>

typedef uint8_t t_pin;

typedef struct {
  float x;
  float y;
  float phi;
} t_pose;

#endif