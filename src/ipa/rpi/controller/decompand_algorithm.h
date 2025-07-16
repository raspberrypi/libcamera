#pragma once

#include "algorithm.h"

namespace RPiController {

class DecompandAlgorithm : public Algorithm
{
public:
	DecompandAlgorithm(Controller *controller) : Algorithm(controller) {}
  virtual void initialValues(uint16_t lut[])
  {
    static const uint16_t defaultLut[] = {
      3072, 3072, 3072, 3072, 3136, 3200, 3264, 3328,
      3392, 3456, 3520, 3584, 3648, 3712, 3776, 3840,
      3904, 3968, 4032, 4096, 4608, 5120, 5632, 6144,
      6656, 7168, 7680, 8192, 8704, 9216, 9728, 10240,
      10752, 11264, 11776, 12288, 12800, 13312, 13824, 14336,
      14848, 15360, 17408, 19456, 21504, 23552, 25600, 27648,
      29696, 31744, 33792, 35840, 37888, 39936, 41984, 44032,
      46080, 48128, 50176, 52224, 54272, 56320, 58368, 60416,
      62464
    };

    for (size_t i = 0; i < sizeof(defaultLut) / sizeof(defaultLut[0]); ++i)
      lut[i] = defaultLut[i];
  }
};
} /* namespace RPiController */
