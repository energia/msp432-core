/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 *  ======== NVSCC3200.h ========
 */

#ifndef ti_drivers_nvs_NVSCC3200__include
#define ti_drivers_nvs_NVSCC3200__include

#include <stdint.h>
#include <stdbool.h>

#if defined (__cplusplus)
extern "C" {
#endif

/*!
 *  @brief  Command to set the copy block for an NVS block.
 *
 *  Passing NVSCC3200_CMD_SET_COPYBLOCK to NVS_control(), along with
 *  a block of memory, is used to set the copy block for an
 *  NVSCC3200_HWAttrs structure.
 *  The copy block is a RAM buffer used as scratch when writing to non-volatile
 *  storage.
 *
 *  If the copy block is not known at compile time, for example, if it
 *  is allocated from heap memory, it can be set through NVS_control()
 *  using the command NVSCC3200_CMD_SET_COPYBLOCK.
 *  The copy block is passed in the arg parameter of NVS_control().  The
 *  size of the copy block passed to NVS_control() must be at least
 *  as large as the flash page size, and it is up to the application to ensure
 *  this.
 *
 *  @sa NVSCC3200_HWAttrs
 */
#define NVSCC3200_CMD_SET_COPYBLOCK      NVS_CMD_RESERVED + 0

/*!
 *  @brief  Alignment error returned by NVSCC3200_control().
 *
 *  This error is returned if the copy block passed to NVSCC3200_control()
 *  is not aligned on a 4-byte boundary, or is NULL.
 *
 *  @sa NVSCC3200_HWAttrs
 */
#define NVSCC3200_STATUS_ECOPYBLOCK      (NVS_STATUS_RESERVED - 1)

/*!
 *  @brief      NVSCC3200 command structure for setting copy block.
 *
 *  This structure is used to hold the copy block information that is
 *  passed to NVS_control().
 */
typedef struct NVSCC3200_CmdSetCopyBlockArgs
{
    void *copyBlock;     /*!< Address of the RAM copy block */
} NVSCC3200_CmdSetCopyBlockArgs;

/* NVS function table pointer */
extern const NVS_FxnTable NVSCC3200_fxnTable;

/*!
 *  @brief      NVSCC3200 attributes
 *
 *  For CC3200 devices, there is no need to provide the flash block
 *  address, since this is managed by the SimpleLink libraries.  The
 *  SimpleLink libraries use the notion of "files" to manage flash
 *  blocks.  We use the id field of the NVSCC3200_HWAttrs to construct a
 *  file name that will correspond to a flash block.
 *
 *  When a file is written to, a scratch region is needed to preserve
 *  the unmodified data in the file.  This scratch region, referred to as
 *  copyBlock, should be a buffer in RAM.  The application
 *  can set copyBlock in the HWAttrs directly, if it is known at compile
 *  time, or set copyBlock through NVS_control(), for example, if it is
 *  allocated on the heap.  The copyBlock can be shared accross multiple
 *  NVS instances.  It is up to the application to ensure that copyBlock
 *  is set before the first call to NVS_write().
 *  The copyBlock should be large enough to hold the maximum file size of
 *  4 Kbytes.
 *  When writing data to a file, the contents of the file are first
 *  copied to the copyBlock.  The copyBlock is then updated with the new
 *  data, and then written back to the file.
 */
typedef struct NVSCC3200_HWAttrs {
    void         *copyBlock;  /*!< A RAM buffer or flash block to use for
                               *   scratch when writing to the block.
                               */
    size_t        blockSize;  /*!< The size of the copy block */
} NVSCC3200_HWAttrs;

/*
 *  @brief      NVSCC3200 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct NVSCC3200_Object {
    bool                 opened;   /* Has the obj been opened */
    int                  id;       /* Index to use for file name */
} NVSCC3200_Object;

#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */

/*@}*/
#endif /* ti_drivers_nvs_NVSCC3200__include */
