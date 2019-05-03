/*============================================================================

  Copyright (c) 2013-2015 Qualcomm Technologies, Inc. All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.

============================================================================*/
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_eeprom.h"

#define LITEON_T4K37_OTP_DEBUG
#undef DEBUG_INFO
#ifdef LITEON_T4K37_OTP_DEBUG
#define DEBUG_INFO(fmt, args...) pr_err(fmt, ##args)
#else
#define DEBUG_INFO(fmt, args...) do { } while (0)
#endif


//#define OTP_MESH_HWROLLOFF_SIZE 63
//#define MESH_HWROLLOFF_SIZE 130

//#define AF_OFFSET_L0 0x
//#define AF_OFFSET_L1 (AF_OFFSET_L0+64)
//#define AF_OFFSET_L2 (AF_OFFSET_L1+64)
//#define AF_OFFSET_L3 (AF_OFFSET_L2+64)
//#define AF_OFFSET_L4 (AF_OFFSET_L3+64)
/*
#define WB_OFFSET_L3 (AF_OFFSET_L0+8)
#define WB_OFFSET_L2 (WB_OFFSET_L3+7)
#define WB_OFFSET_L1 (WB_OFFSET_L2+7)
#define WB_OFFSET_L0 (WB_OFFSET_L1+7)

#define LSC_OFFSET (WB_OFFSET_L0+7)
#define LSC_R_OFFSET (LSC_OFFSET)
#define LSC_GR_OFFSET (LSC_R_OFFSET+2)
#define LSC_B_OFFSET (LSC_OFFSET+252)
#define LSC_GB_OFFSET (LSC_B_OFFSET+2)
#define AF_OFFSET AF_OFFSET_L3
#define WB_OFFSET WB_OFFSET_L3
*/
#define AF_OFFSET_INF  0
#define AF_OFFSET_MICRO 4
#define AF_OFFSET_START 6

#define SN_OFFSET 14
#define PN_OFFSET 18

#define BOLCK_EMPTY 0
#define BOLCK_NOT_EMPTY 1
#define MAX_EMPTY_BYTES 63
#define BOLCK_SIZE 64

#define MAX_VENDOR_BOLCK 4
#define MIN_VENDOR_BOLCK 0
#define AF_GOLDEN_START  100
#define AF_GOLDEN_INFINITY  150
#define AF_GOLDEN_MICRO  550
//#define AWBLSC_VALID_BIT 6

//#define QVALUE 1024.0
///* Data conversion macros */
//#define ABS(x)  (((x) > 0) ? (x):-(x))
//#define SIGN(x) (((x) > 0) ? (1):-(1))
//#define MIN(x, y) (((x) > (y)) ? (y):(x))
//#define MAX(x, y) (((x) > (y)) ? (x):(y))
//#define CLIP(z, x, y) MAX(MIN((z), (y)), (x))
//
///* defining pixel patterns */
//#define RGGB_PATTERN  0
//#define GRBG_PATTERN  1
//#define BGGR_PATTERN  2
//#define GBRG_PATTERN  3
//
//#define YCBYCR422_PATTERN   4
//#define YCRYCB422_PATTERN   5
//#define CBYCRY422_PATTERN   6
//#define CRYCBY422_PATTERN   7
//
//#define MESH_ROLLOFF_HORIZONTAL_GRIDS  12
//#define MESH_ROLLOFF_VERTICAL_GRIDS    9
//
//#define SENSOR_FULL_SIZE_WIDTH 4208
//#define SENSOR_FULL_SIZE_HEIGHT 3120
//
//#define max(x, y) (((x) > (y)) ? (x):(y))
//#define min(x, y) (((x) < (y)) ? (x):(y))

//
///* Temp data */
//static float otp_r[OTP_MESH_HWROLLOFF_SIZE], otp_gr[OTP_MESH_HWROLLOFF_SIZE];
//static float otp_gb[OTP_MESH_HWROLLOFF_SIZE], otp_b[OTP_MESH_HWROLLOFF_SIZE];
//static float mesh_r[MESH_HWROLLOFF_SIZE], mesh_gr[MESH_HWROLLOFF_SIZE];
//static float mesh_gb[MESH_HWROLLOFF_SIZE], mesh_b[MESH_HWROLLOFF_SIZE];
//static uint8_t bLscAwbValid;

/** liteon_t4k37_otp_get_calibration_items:
 *    @e_ctrl: address of pointer to
 *                   chromatix struct
 *
 * Loads data structure for enabling / disabling parameters that can be
 * calibrated
 *
 * Return:
 * void
 **/
/*
void liteon_t4k37_otp_get_calibration_items( void *e_ctrl )
{
  sensor_eeprom_data_t *ectrl = (sensor_eeprom_data_t *)e_ctrl;
  eeprom_calib_items_t *e_items = &(ectrl->eeprom_data.items);
  e_items->is_afc = TRUE;

//  if(TRUE == bLscAwbValid){
//    e_items->is_wbc = TRUE;
//    e_items->is_lsc = TRUE;
//    SHIGH("WBC and LSC Available and loaded");
//  } else {
//    e_items->is_wbc = FALSE;
//    e_items->is_lsc = FALSE;
//    SHIGH("WBC and LSC UNavailable and not loaded");
//  }
  e_items->is_wbc = FALSE;
  e_items->is_lsc = FALSE;
  e_items->is_dpc = FALSE;
}
*/
/** liteon_t4k37_otp_check_empty_page:
 *    @buff: address of page buffer
 *
 * Checks if the page has non zero data
 *
 * Return:
 * uint8_t :  BOLCK_EMPTY / BOLCK_NOT_EMPTY
 **/
uint8_t liteon_t4k37_otp_check_empty_page( uint8_t *buff )
{
  uint8_t retval = BOLCK_EMPTY;
  int i=0;

  for(i=0; i < MAX_EMPTY_BYTES; i++){
    if( buff[i] != 0 )
    {
      retval = BOLCK_NOT_EMPTY;
      break;
    }
  }
  return retval;
}



uint32_t  liteon_t4k37_otp_format_SN(uint8_t *data,
        uint32_t page_num )
{

	int sn_start_pos = page_num*64 + SN_OFFSET;
	uint32_t serial_num = 0;
	serial_num = (uint32_t) ((data[sn_start_pos] << 24) |
			(data[sn_start_pos + 1] << 16) |
			(data[sn_start_pos + 2] << 8) |
			(data[sn_start_pos + 3] << 0) );


    DEBUG_INFO("Serial Number= %d", serial_num);
    return serial_num;
}

uint32_t   liteon_t4k37_otp_format_PN(uint8_t *data,
        uint32_t page_num )
{

	int pn_start_pos = page_num*64 + PN_OFFSET;
	uint32_t product_num = 0;
	product_num = (uint32_t) ((data[pn_start_pos] << 24) |
			(data[pn_start_pos + 1] << 16) |
			(data[pn_start_pos + 2] << 8) |
			(data[pn_start_pos + 3] << 0) );


    DEBUG_INFO("Product Number= %d", product_num);
    return product_num;
}
/** liteon_t4k37_otp_format_afdata_internal:
 *    @e_ctrl: address of pointer to
 *                   chromatix struct
 *    @AF_START_OFFSET: start offset of page in eeprom memory
 *
 * Format Auto Focus calibration data for AF calibration
 *
 * Return:
 * void
 **/
/*
void  liteon_t4k37_otp_format_afdata_internal( sensor_eeprom_data_t *e_ctrl,
        uint32_t af_start_offset )
{
  e_ctrl->eeprom_data.afc.starting_dac =
    (uint16_t) (((e_ctrl->eeprom_params.buffer[af_start_offset+AF_OFFSET_START]) << 8) |
    (e_ctrl->eeprom_params.buffer[af_start_offset + AF_OFFSET_START +1]));

  e_ctrl->eeprom_data.afc.infinity_dac =
    (uint16_t)(((e_ctrl->eeprom_params.buffer[af_start_offset + AF_OFFSET_INF]) << 8) |
    (e_ctrl->eeprom_params.buffer[af_start_offset + AF_OFFSET_INF +1]));

  e_ctrl->eeprom_data.afc.macro_dac =
    (uint16_t)(((e_ctrl->eeprom_params.buffer[af_start_offset + AF_OFFSET_MICRO]) << 8) |
    (e_ctrl->eeprom_params.buffer[af_start_offset + 5]));

  DEBUG_INFO("AF Starting DAC = %d", e_ctrl->eeprom_data.afc.starting_dac);
  DEBUG_INFO("AF Macro DAC = %d", e_ctrl->eeprom_data.afc.macro_dac);
  DEBUG_INFO("AF Infinity DAC = %d", e_ctrl->eeprom_data.afc.infinity_dac);
}
*/
/** liteon_t4k37_otp_format_afdata_internal:
 *    @e_ctrl: address of pointer to
 *                   chromatix struct
 *
 *  Checks for non empty page to calibrate sensor for Auto Focus
 *  This function is called by liteon_t4k37_otp_format_afdata
 *
 * Return:
 * void
 **/
/*
static void liteon_t4k37_otp_format_afdata( sensor_eeprom_data_t *e_ctrl ,int page_num )
{
  SLOW("Enter");
  if(page_num > -1)
  {
	  //  0 < page_num < MAX_VENDOR_BOLCK
	  DEBUG_INFO( "Loading Page %d",page_num );
	  liteon_t4k37_otp_format_afdata_internal( e_ctrl, page_num*64);

  }
  else
  {
	  e_ctrl->eeprom_data.afc.starting_dac = AF_GOLDEN_START;
	  e_ctrl->eeprom_data.afc.infinity_dac =AF_GOLDEN_INFINITY;
	  e_ctrl->eeprom_data.afc.macro_dac =AF_GOLDEN_MICRO;
  }
  SLOW("Exit");
}
*/

/** liteon_t4k37_otp_format_wbdata:
 *    @e_ctrl: address of pointer to
 *                   chromatix struct
 *
 *  Checks for non empty page to calibrate sensor for Auto Focus
 *
 * Return:
 * void
 **/
/*
static void liteon_t4k37_otp_format_wbdata(sensor_eeprom_data_t *e_ctrl)
{
  SLOW("Enter liteon_t4k37_otp_format_wbdata");

  SLOW("Exit");
}
*/
/** liteon_t4k37_otp_format_lensshading:
 *    @e_ctrl: address of pointer to
 *                   chromatix struct
 *
 *  Loads lens shading data from the eeprom into the chromatix data
 *
 * Return:
 * void
 **/
/*
static void liteon_t4k37_otp_format_lensshading (void *e_ctrl)
{

  SLOW("Enter");


  SLOW("Exit");
}
*/
/** liteon_t4k37_otp_format_calibration_data:
 *    @e_ctrl: address of pointer to
 *                   chromatix struct
 *
 *  This function call all the sub function to read chromatix data and calibrate
 *  the sensor
 *
 * Return:
 * void
 **/
void liteon_t4k37_otp_format_calibration_data(struct msm_eeprom_ctrl_t *e_ctrl)
{
  int page_i;
  int page_num = -1;
  uint8_t *data = e_ctrl->cal_data.mapdata;
  DEBUG_INFO("Enter");

  for(page_i=MAX_VENDOR_BOLCK ; page_i>=MIN_VENDOR_BOLCK ; page_i--)
  {
	  if ( BOLCK_NOT_EMPTY ==  liteon_t4k37_otp_check_empty_page(
			  &data[page_i*64]) )
	  {
		  page_num = page_i;
		  break;
	  }
  }
  if(page_i >=0)
  {
	  e_ctrl->module_sn = liteon_t4k37_otp_format_SN(data,page_num);
	  e_ctrl->module_pn = liteon_t4k37_otp_format_PN(data,page_num);
	 // liteon_t4k37_otp_format_afdata(data,page_num);
  }
 // liteon_t4k37_otp_format_wbdata(ectrl);
  //liteon_t4k37_otp_format_lensshading(ectrl);

  DEBUG_INFO("Exit");
}

/** liteon_t4k37_otp_lib_func_ptr:
 *  This structure creates the function pointer for imx135 eeprom lib
 **/
/*
static eeprom_lib_func_t liteon_t4k37_otp_lib_func_ptr = {
    .get_calibration_items = liteon_t4k37_otp_get_calibration_items,
    .format_calibration_data = liteon_t4k37_otp_format_calibration_data,
    .do_af_calibration = eeprom_autofocus_calibration,
    .do_wbc_calibration = NULL,//eeprom_whitebalance_calibration,
    .do_lsc_calibration = NULL,//eeprom_lensshading_calibration,
    .do_dpc_calibration = NULL,
    .get_dpc_calibration_info = NULL,
    .get_raw_data = NULL,
};
*/
/** liteon_t4k37_otp_eeprom_open_lib:
 *    @e_ctrl: address of pointer to
 *                   chromatix struct
 *
 *  This function call all the sub function to read chromatix data and calibrate
 *  the sensor
 *
 * Return:
 * void* : Pinter to the liteon_t4k37_otp function table
 **/
/*
void* liteon_t4k37_otp_eeprom_open_lib(void) {
  return &liteon_t4k37_otp_lib_func_ptr;
}
*/
