#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "rl_usb.h"

// Definition MEDIA_DRIVE is used to define Drive to be used for media
//   "M:" or "M0:" if media is Memory Card 0
#define MEDIA_DRIVE     "M0:"
 
 
#include "rl_fs.h"
#define  MEDIA_OWN_USB     (1U     )    // Media owned by USB (bit mask)
#define  MEDIA_OWN_CHG     (1U << 1)    // Media ownership change requested (bit mask)
extern 
volatile uint8_t  usbd_msc0_media_own;
volatile uint8_t  usbd_msc0_media_own;  // USB MSC0 media ownership
static   int32_t  drv_id;               // FAT drive id
static   bool     media_ok;             // Media is initialized and ok
 
 
// Called during USBD_Initialize to initialize the USB MSC class instance.
void USBD_MSC0_Initialize (void) {
  uint32_t param_status;
 
  usbd_msc0_media_own = MEDIA_OWN_USB;  // Initially media is owned by USB
  media_ok            = false;          // Current media status (not initialized = not ok)
 
  if (finit (MEDIA_DRIVE) != fsOK) {    // Initialize File System
    return;                             // Exit if failed
  }
 
  drv_id = fs_ioc_get_id (MEDIA_DRIVE); // Get ID of media drive
  if (drv_id < 0)           { return; } // If ID is invalid exit
 
  param_status = 0U;                    // Parameter for function call is 0
                                        // Initialize media
  if (fs_ioc_device_ctrl (drv_id, fsDevCtrlCodeControlMedia, &param_status) != fsOK) {
    return;                             // Exit if failed
  }
 
  if (fs_ioc_lock (drv_id)) {           // Lock media for USB usage
    return;                             // Exit if failed
  }
 
  media_ok = true;                      // Media was initialized and is ok
}
 
 
// \brief Called during USBD_Uninitialize to de-initialize the USB MSC class instance.
void USBD_MSC0_Uninitialize (void) {
  // Add code for de-initialization
}
 
 
// Get cache information.
// \param[out]    buffer               cache buffer address.
// \param[out]    size                 cache buffer size.
// \return        true                 operation succeeded.
// \return        false                operation failed.
bool USBD_MSC0_GetCacheInfo (uint32_t *buffer, uint32_t *size) {
  fsIOC_Cache cache_info;
 
  // Get cache settings of File System
  if (fs_ioc_get_cache(drv_id, &cache_info) != fsOK) {
    return false;                       // Exit if failed
  }
 
  // Use File Systems cache for MSC
  *buffer = (uint32_t)cache_info.buffer;// Cache buffer from File System
  *size   = cache_info.size;            // Cache size 
  return true;
}
 
 
// Get media capacity.
// \param[out]    block_count          total number of blocks on media.
// \param[out]    block_size           media block size.
// \return        true                 operation succeeded.
// \return        false                operation failed.
bool USBD_MSC0_GetMediaCapacity (uint32_t *block_count, uint32_t *block_size) {
  fsMediaInfo media_info;
 
  // Read media information of actual media
  if (fs_ioc_read_info(drv_id, &media_info) != fsOK) {
    return false;                       // Exit if failed
  }
 
  *block_count = media_info.block_cnt;  // Total number of blocks on media
  *block_size  = media_info.read_blen;  // Block size of blocks on media 
  return true;
}
 
 
// Read data from media.
// \param[in]     lba                  logical address of first block to read.
// \param[in]     cnt                  number of contiguous blocks to read from media.
// \param[out]    buf                  data buffer for data read from media.
// \return        true                 read succeeded.
// \return        false                read failed.
bool USBD_MSC0_Read (uint32_t lba, uint32_t cnt, uint8_t *buf) {
                                        // Read data directly from media
  if (fs_ioc_read_sector (drv_id, lba, buf, cnt) != fsOK) {
    return false;
  }
  return true;
}
 
 
// Write data to media.
// \param[in]     lba                  logical address of first block to write.
// \param[in]     cnt                  number of contiguous blocks to write to media.
// \param[out]    buf                  data buffer containing data to write to media.
// \return        true                 write succeeded.
// \return        false                write failed.
bool USBD_MSC0_Write (uint32_t lba, uint32_t cnt, const uint8_t *buf) {
                                        // Write data directly to media
  if (fs_ioc_write_sector (drv_id, lba, buf, cnt) != fsOK) {
    return false;
  }
  return true;
}
 
 
// Check media presence and write protect status.
//        (if media is not owned by USB it returns that media is not ready)
// \return                             media presence and write protected status
//                bit 1:               write protect bit
//                 - value 1:            media is write protected
//                 - value 0:            media is not write protected
//                bit 0:               media presence bit
//                 - value 1:            media is present
//                 - value 0:            media is not present
uint32_t USBD_MSC0_CheckMedia (void) {
       uint32_t param_status;
       uint8_t  media_state;            // Bit 0. media ready, Bit 1. media write protect
static uint8_t  media_ready_ex = 0U;    // Previous media ready state
       uint8_t  own;
 
  // Get current media status
  media_state = 0U;
  switch (fs_ioc_device_ctrl (drv_id, fsDevCtrlCodeCheckMedia, &param_status)) {
    case fsOK:
      if (param_status & FS_MEDIA_NOCHKMEDIA) {
        // If check media not available on hardware layer
        media_state  =  USBD_MSC_MEDIA_READY;
        break;
      }
      if (param_status & FS_MEDIA_INSERTED) {
        media_state  =  USBD_MSC_MEDIA_READY;
      }
      if (param_status & FS_MEDIA_PROTECTED) {
        media_state |=  USBD_MSC_MEDIA_PROTECTED;
      }
      break;
    case fsError:
    case fsUnsupported:
    case fsAccessDenied:
    case fsInvalidParameter:
    case fsInvalidDrive:
    case fsInvalidPath:
    case fsUninitializedDrive:
    case fsDriverError:
    case fsMediaError:
    case fsNoMedia:
    case fsNoFileSystem:
    case fsNoFreeSpace:
    case fsFileNotFound:
    case fsDirNotEmpty:
    case fsTooManyOpenFiles:
    case fsAlreadyExists:
    case fsNotDirectory:
      break;
  }
 
  // Store current owner so no new request can interfere
  own = usbd_msc0_media_own;
 
  // De-initialize media according to previous owner
  if (own & MEDIA_OWN_CHG) {                    // If owner change requested
    if (own & MEDIA_OWN_USB) {                  // If new requested owner is USB (previous owner was File System)
      (void)funmount (MEDIA_DRIVE);             // De-initialize media and dismount Drive
    } else {                                    // If new requested owner is File System (previous owner was USB)
      (void)fs_ioc_unlock (drv_id);             // Un-lock media
    }
  }
 
  // Initialize media according to current owner
  if ((own & MEDIA_OWN_CHG)        ||           // If owner change requested or
      (media_state ^ media_ready_ex)) {         // if media ready state has changed (disconnect(SD remove)/connect(SD insert))
    if (media_state & USBD_MSC_MEDIA_READY) {   // If media is ready
      if (own & MEDIA_OWN_USB){                 // If current owner is USB
        media_ok     = false;                   // Invalidate current media status (not initialized = not ok)
        param_status = 0U;                      // Parameter for function call is 0
        if (fs_ioc_device_ctrl (drv_id, fsDevCtrlCodeControlMedia, &param_status) == fsOK) {
                                                // Initialization of media has succeeded
          if (fs_ioc_lock (drv_id) == fsOK) {   // If lock media for USB usage has succeeded
            media_ok = true;                    // Media was initialized and is ok
          }
        }
      } else {                                  // If current owner is File System
        if (fmount (MEDIA_DRIVE) == fsOK) {     // Initialize media and Mount Drive for File System usage
          media_ok = true;                      // Media was initialized and is ok
        }
      }
    }
    if (own & MEDIA_OWN_CHG) {
      usbd_msc0_media_own &= ~MEDIA_OWN_CHG;    // Clear request to change media owner if it was handled
    }
    media_ready_ex = media_state & USBD_MSC_MEDIA_READY;
  }
 
  // If media is not ok or owned by File System return that it is not ready for USB
  if ((!media_ok) || (!(usbd_msc0_media_own & MEDIA_OWN_USB))) {
    return 0U;
  }
 
  return media_state;
}
//! [code_USBD_User_MSC]
