/*

SiI8620 Linux Driver

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.  See
the GNU General Public License for more details at http://www.gnu.org/licenses/gpl-2.0.html.

*/

/*
 *****************************************************************************
 * @file  si_app_devcap.h
 *
 * @brief Implementation of the Foo API.
 *
 *****************************************************************************
*/
#define DEVCAP_VAL_DEV_STATE		0
#define DEVCAP_VAL_MHL_VERSION		MHL_VERSION
#define DEVCAP_VAL_DEV_CAT		(MHL_DEV_CAT_SOURCE | MHL_DEV_CATEGORY_POW_BIT)
#define DEVCAP_VAL_ADOPTER_ID_H		(uint8_t)(SILICON_IMAGE_ADOPTER_ID >> 8)
#define DEVCAP_VAL_ADOPTER_ID_L		(uint8_t)(SILICON_IMAGE_ADOPTER_ID & 0xFF)
#define DEVCAP_VAL_VID_LINK_MODE	(MHL_DEV_VID_LINK_SUPP_RGB444 \
					| MHL_DEV_VID_LINK_SUPP_YCBCR422 \
					| MHL_DEV_VID_LINK_SUPP_YCBCR444 \
					| MHL_DEV_VID_LINK_SUPP_PPIXEL \
					| MHL_DEV_VID_LINK_SUPP_VGA \
					)
#define DEVCAP_VAL_AUD_LINK_MODE	MHL_DEV_AUD_LINK_2CH
#define DEVCAP_VAL_VIDEO_TYPE		0
#define DEVCAP_VAL_LOG_DEV_MAP		MHL_LOGICAL_DEVICE_MAP
#define DEVCAP_VAL_BANDWIDTH		0x0F
#define DEVCAP_VAL_FEATURE_FLAG		(MHL_FEATURE_RCP_SUPPORT \
					| MHL_FEATURE_RAP_SUPPORT \
					| MHL_FEATURE_SP_SUPPORT \
					| MHL_FEATURE_UCP_SEND_SUPPORT \
					| MHL_FEATURE_UCP_RECV_SUPPORT \
					)
#define DEVCAP_VAL_SCRATCHPAD_SIZE	MHL_SCRATCHPAD_SIZE
#define DEVCAP_VAL_INT_STAT_SIZE	MHL_INT_AND_STATUS_SIZE
#define DEVCAP_VAL_RESERVED		0

#define XDEVCAP_VAL_ECBUS_SPEEDS	(MHL_XDC_ECBUS_S_075 \
					| MHL_XDC_ECBUS_S_8BIT \
					| MHL_XDC_ECBUS_S_12BIT \
					| MHL_XDC_ECBUS_D_150 \
					| MHL_XDC_ECBUS_D_8BIT \
					)

#define XDEVCAP_VAL_TMDS_SPEEDS		(MHL_XDC_TMDS_150 \
					| MHL_XDC_TMDS_300 \
					| MHL_XDC_TMDS_600 \
					)
#define XDEVCAP_VAL_DEV_ROLES		(MHL_XDC_DEV_HOST | MHL_XDC_HID_HOST)
#define XDEVCAP_VAL_LOG_DEV_MAPX	MHL_XDC_LD_PHONE

