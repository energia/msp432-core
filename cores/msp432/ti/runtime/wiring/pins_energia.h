#ifndef pins_energia_h
#define pins_energia_h

#if defined(BOARD_CC3200LP) || defined(BOARD_CC3200_LAUNCHXL)
#include <ti/runtime/wiring/cc3200/variants/CC3200_LAUNCHXL/pins_energia.h>
#elif defined(BOARD_CC3200STK_WIFI)
#include <ti/runtime/wiring/cc3200/variants/CC3200STK_WIFI/pins_energia.h>
#elif defined(BOARD_RedBearLab_CC3200)
#include <ti/runtime/wiring/cc3200/variants/RedBearLab_CC3200/pins_energia.h>
#elif defined(BOARD_RedBearLab_WiFi_Micro)
#include <ti/runtime/wiring/cc3200/variants/RedBearLab_WiFi_Micro/pins_energia.h>
#elif defined(BOARD_RedBearLab_WiFi_Mini)
#include <ti/runtime/wiring/cc3200/variants/RedBearLab_WiFi_Mini/pins_energia.h>
#elif defined(BOARD_MSP432LP) || defined(BOARD_MSP_EXP432P401R)
#include <ti/runtime/wiring/msp432/variants/MSP_EXP432P401R/pins_energia.h>
#elif defined(BOARD_CC2650STK_BLE)
#include <ti/runtime/wiring/cc26xx/variants/CC2650STK_BLE/pins_energia.h>
#elif defined(BOARD_CC2650_LAUNCHXL)
#include <ti/runtime/wiring/cc26xx/variants/CC2650_LAUNCHXL/pins_energia.h>
#elif defined(BOARD_LAUNCHXL_CC1310)
#include <ti/runtime/wiring/cc13xx/variants/LAUNCHXL_CC1310/pins_energia.h>
#elif defined(BOARD_LAUNCHXL_CC1350)
#include <ti/runtime/wiring/cc13xx/variants/LAUNCHXL_CC1350/pins_energia.h>
#endif

#endif
