#ifndef BSEC_CONFIG_H
#define BSEC_CONFIG_H

#include <Arduino.h>

const uint8_t bsec_config_iaq[] = {
  #include "bsec_iaq_data.inc"
};

#endif
