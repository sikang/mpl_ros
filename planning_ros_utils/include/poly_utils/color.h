#include <stdio.h>

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

