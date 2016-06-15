/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ===========================================================================
 *  @file   WiFiCC3200.h
 *
 *  @brief  WiFi driver implementation for the SimpleLink Wi-Fi CC3200 device.
 *
 *  The WiFi header file should be included in the application as follows:
 *  @code
 *  #include <ti/drivers/WiFi.h>
 *  #include <ti/drivers/wifi/WiFiCC3200.h>
 *  @endcode
 *
 *  Refer to @ref WiFi.h for a complete description of APIs & example of use.
 *
 *  This WiFi driver implementation is designed for the SimpleLink Wi-Fi CC3200
 *  device. The CC3200 has a dedicated SPI peripheral utilized by this driver.
 *  Corresponding DMA channels are utilized as well. This means that in addition
 *  to a *WiFi_config* array, a *SPI_config* array SPI entry must be provided.
 *  See @ref SPI.h, for more details.
 *
 *  The SPI transport layer used by the CC3200 uses an interrupt to signal
 *  when the network processor requires attention. The interrupt number
 *  is configured in the WiFiCC3200_HWAttrs structure. GPIO interrupts are
 *  not used, unlike other WiFi driver implementations.
 *
 *  This WiFi driver implementation provides the SPI transport layer
 *  required by the SimpleLink Host Driver.  In the case of unsolicited events
 *  that require the application to make decisions (such as an unsolicited
 *  disconnect from an AP), the SimpleLink Host Driver specifies callbacks
 *  which must be modified to fit application requirements.  Contrary to other
 *  WiFi driver implementations, callback function pointers are not provided
 *  to WiFi_open() as arguments.  An example of how to modify the SimpleLink
 *  Host Driver callbacks is shown below:
 *
 *  @code
 *  void SimpleLinkWlanEventHandler(SlWlanEvent_t *pArgs)
 *  {
 *      switch(pArgs->Event){
 *          case SL_WLAN_CONNECT_EVENT:
 *              // CC3200 connected to an AP
 *              deviceConnected = true;
 *              break;
 *
 *          case SL_WLAN_DISCONNECT_EVENT:
 *              // CC3200 disconnected from an AP
 *              deviceConnected = false;
 *              break;
 *          case SL_WLAN_SMART_CONFIG_START_EVENT:
 *              break;
 *
 *          case SL_WLAN_SMART_CONFIG_STOP_EVENT:
 *              break;
 *      }
 *  }
 *
 *  void function0()
 *  {
 *      WiFi_Handle      handle;
 *      WiFi_Params      params;
 *
 *      WiFi_Params_init(&params);
 *      handle = WiFi_open(WiFi_configIndex, SPI_configIndex, NULL, &params);
 *      if (!handle) {
 *          System_abort("WiFi did not open");
 *      }
 *
 *      // Wi-Fi device's host driver APIs (such as socket()) may now be used
 *  }
 *  @endcode
 *
 *  For a list of all events and details on what data they may provide, see
 *  the Porting - Event Handlers section of the SimpleLink Host Driver API
 *  reference guide on <a href="http://processors.wiki.ti.com/index.php/CC3200">
 *  SimpleLink Wi-Fi CC3200 Wiki</a>.
 *
 *  The SimpleLink Host Driver APIs *sl_Start()* must be called by the user
 *  application to start the network processor.  Additionally, *sl_Start()* and
 *  *sl_Stop() can be called by the application to restart the CC3200 without
 *  having to close and reopen the WiFi Driver instance.
 *
 *  The SimpleLink Host Driver for the CC3200 device is featured as in
 *  TI-Middleware (tidrivers_install_dir/packages/ti/mw/wifi/cc3x00).
 *
 *  For more on the SimpleLink Host Driver APIs see
 *  <a href="http://processors.wiki.ti.com/index.php/CC3200"> SimpleLink Wi-Fi
 *  CC3200 Wiki</a>.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_wifi_WiFiCC3200__include
#define ti_drivers_wifi_WiFiCC3200__include

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <ti/drivers/SPI.h>
#include <ti/drivers/WiFi.h>

/*! @brief  WiFi function table for CC3200 devices */
extern const WiFi_FxnTable WiFiCC3200_fxnTable;

/*!
 *  @brief  WiFiCC3200 Hardware attributes
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For CCWare, these definitions are found in:
 *      - inc/hw_ints.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const WiFiCC3200_HWAttrs wiFiCC3000HWAttrs[] = {
 *      {
 *          INT_NWPIC,
 *      }
 *  };
 *  @endcode
 */
typedef struct WiFiCC3200_HWAttrs {
    unsigned int    wifiIntNum;
} WiFiCC3200_HWAttrs;

/*!
 *  @brief  WiFiCC3200 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct WiFiCC3200_Object {
    Hwi_Struct       wifiHwi;
    SPI_Handle       spiHandle;

    uint32_t         bitRate;
    unsigned int     spiIndex;

    void           (*wifiIntFxn)();

    bool             isOpen;
} WiFiCC3200_Object;

#ifdef  __cplusplus
}
#endif

#endif /* ti_drivers_wifi_WiFiCC3200__include */
