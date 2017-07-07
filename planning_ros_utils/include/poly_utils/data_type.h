#ifndef BASIC_POLY_DATA_H
#define BASIC_POLY_DATA_H
#include <motion_primitive_library/data_type.h>

typedef union {
  struct /*anonymous*/
      {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
} RGBValue;

#endif
