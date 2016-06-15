/*
 * Copyright (c) 2015, Texas Instruments Incorporated
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

/*
 *  ======== NVSCC26XX.h ========
 */

#ifndef ti_drivers_nvs_NVSCC26XX__include
#define ti_drivers_nvs_NVSCC26XX__include

#include <stdint.h>
#include <stdbool.h>

#if defined (__cplusplus)
extern "C" {
#endif

/*!
 *  @brief  Command to set the copy block for an NVS block.
 *
 *  Passing NVSCC26XX_CMD_SET_COPYBLOCK to NVS_control(), along with
 *  a block of memory, is used to set the copy block for an
 *  NVSCC26XX_HWAttrs structure.
 *  The copy block is used as scratch when writing to a flash block.  Since
 *  the block must be erased before writing to it, the data in the block
 *  that is outside of the region to be modified, must be preserved.  It
 *  will be copied into the copy block, along with the buffer of data
 *  passed to NVS_write().  The block is then erased and the copy block
 *  copied back to the block.  If the copy block is not known at compile
 *  time, for example, if it is allocated from heap memory, it can be set
 *  through NVS_control() using the command NVSCC26XX_CMD_SET_COPYBLOCK.
 *  The copy block is passed in the arg parameter of NVS_control().  The
 *  size of the copy block passed to NVS_control() must be at least
 *  as large as the block size, and it is up to the application to ensure
 *  this.
 *
 *  @sa NVSCC26XX_HWAttrs
 */
#define NVSCC26XX_CMD_SET_COPYBLOCK      NVS_CMD_RESERVED + 0

/*!
 *  @brief  Alignment error returned by NVSCC26XX_control().
 *
 *  This error is returned if the copy block passed to NVSCC26XX_control()
 *  is not aligned on a 4-byte boundary, or is NULL.
 *
 *  @sa NVSCC26XX_HWAttrs
 */
#define NVSCC26XX_STATUS_ECOPYBLOCK      (NVS_STATUS_RESERVED - 1)

/*!
 *  @brief      NVSCC26XX command structure for setting copy block.
 *
 *  This structure is used to hold the copy block information that is
 *  passed to NVS_control().  If copyBlock is a buffer in RAM, isRam
 *  should be set to TRUE.  If copyBlock is in Flash, set isRam to
 *  FALSE.
 */
typedef struct NVSCC26XX_CmdSetCopyBlockArgs
{
    void *copyBlock;     /*!< Address of the copy block */
    bool isRam;          /*!< TRUE if copyBlock is a RAM buffer */
} NVSCC26XX_CmdSetCopyBlockArgs;

/* NVS function table pointer */
extern const NVS_FxnTable NVSCC26XX_fxnTable;

/*!
 *  @brief      NVSCC26XX attributes
 *
 *  The block is the address of a region in flash of size blockSize bytes.
 *
 *  For CC26XX devices, the smallest erase page size is 4KB, so in most
 *  cases, blockSize should be set to 4KB for this device.  If the
 *  blockSize is less than the page size, care should be taken not to use
 *  the rest of the page.  A write to the block will cause the entire page
 *  to be erased!  A blockSize greater than the page size is not supported.
 *  The page size for the device can be obtained through NVS_getAttrs().
 *
 *  When the block is written to, a scratch region is needed to preserve
 *  the unmodified data in the block.  This scratch region, referred to as
 *  copyBlock, can be a page in flash or a buffer in RAM.  The application
 *  can set copyBlock in the HWAttrs directly, if it is known at compile
 *  time, or set copyBlock through NVS_control(), for example, if it is
 *  allocated on the heap.  The copyBlock can be shared accross multiple
 *  NVS instances.  It is up to the application to ensure that copyBlock
 *  is set before the first call to NVS_write().
 *  Using a blockSize less than the page size decreases RAM or heap only
 *  if copyBlock is not in flash.
 */
typedef struct NVSCC26XX_HWAttrs {
    void         *block;      /*!< Address of flash block to manage */
    size_t        blockSize;  /*!< The size of block */
    void         *copyBlock;  /*!< A RAM buffer or flash block to use for
                               *   scratch when writing to the block.
                               */
    bool          isRam;      /*!< TRUE if copyBlock is a RAM buffer */
} NVSCC26XX_HWAttrs;

/*
 *  @brief      NVSCC26XX Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct NVSCC26XX_Object {
    bool                 opened;   /* Has the obj been opened */
} NVSCC26XX_Object;

#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */

/*@}*/
#endif /* ti_drivers_nvs_NVSCC26XX__include */
