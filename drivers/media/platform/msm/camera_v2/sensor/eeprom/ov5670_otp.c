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

#define LITEON_OV5670_OTP_DEBUG
#undef DEBUG_INFO
#ifdef LITEON_OV5670_OTP_DEBUG
#define DEBUG_INFO(fmt, args...) pr_err(fmt, ##args)
#else
#define DEBUG_INFO(fmt, args...) do { } while (0)
#endif


#define AF_OFFSET_INF  0
#define AF_OFFSET_MICRO 4
#define AF_OFFSET_START 6

#define SN_OFFSET 14
#define PN_OFFSET 18

#define BOLCK_EMPTY 0
#define BOLCK_NOT_EMPTY 1
#define MAX_EMPTY_BYTES 63
#define BOLCK_SIZE 64

#define MAX_VENDOR_BOLCK 2
#define MIN_VENDOR_BOLCK 0


/** liteon_ov5670_otp_check_empty_page:
 *    @buff: address of page buffer
 *
 * Checks if the page has non zero data
 *
 * Return:
 * uint8_t :  BOLCK_EMPTY / BOLCK_NOT_EMPTY
 **/
uint8_t liteon_ov5670_otp_check_empty_page( uint8_t *buff )
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



uint32_t  liteon_ov5670_otp_format_SN(uint8_t *data,
        uint32_t page_num )
{

	int sn_start_pos = page_num*24 + SN_OFFSET;
	uint32_t serial_num = 0;
	serial_num = (uint32_t) ((data[sn_start_pos] << 24) |
			(data[sn_start_pos + 1] << 16) |
			(data[sn_start_pos + 2] << 8) |
			(data[sn_start_pos + 3] << 0) );


    DEBUG_INFO("Ov5670 Serial Number= %d", serial_num);
    return serial_num;
}

uint32_t   liteon_ov5670_otp_format_PN(uint8_t *data,
        uint32_t page_num )
{

	int pn_start_pos = page_num*24 + PN_OFFSET;
	uint32_t product_num = 0;
	product_num = (uint32_t) ((data[pn_start_pos] << 24) |
			(data[pn_start_pos + 1] << 16) |
			(data[pn_start_pos + 2] << 8) |
			(data[pn_start_pos + 3] << 0) );


    DEBUG_INFO("Ov5670 Product Number= %d", product_num);
    return product_num;
}

/** liteon_ov5670_otp_format_calibration_data:
 *    @e_ctrl: address of pointer to
 *                   chromatix struct
 *
 *  This function call all the sub function to read chromatix data and calibrate
 *  the sensor
 *
 * Return:
 * void
 **/
void liteon_ov5670_otp_format_calibration_data(struct msm_eeprom_ctrl_t *e_ctrl)
{
  int page_i;
  int page_num = -1;
  uint8_t *data = e_ctrl->cal_data.mapdata;
  DEBUG_INFO("Ov5670 Enter");

  for(page_i=MAX_VENDOR_BOLCK ; page_i>=MIN_VENDOR_BOLCK ; page_i--)
  {
	  if ( BOLCK_NOT_EMPTY ==  liteon_ov5670_otp_check_empty_page(
			  &data[page_i*24]) )
	  {
		  page_num = page_i;
		  break;
	  }
  }
  if(page_i >=0)
  {
	  e_ctrl->module_sn = liteon_ov5670_otp_format_SN(data,page_num);
	  e_ctrl->module_pn = liteon_ov5670_otp_format_PN(data,page_num);
  }

  DEBUG_INFO("Ov5670 Exit");
}

