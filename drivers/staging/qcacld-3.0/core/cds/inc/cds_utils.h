/*
 * Copyright (c) 2014-2017 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

#if !defined(__CDS_UTILS_H)
#define __CDS_UTILS_H

/**=========================================================================

   \file  cds_utils.h

   \brief Connectivity driver services (CDS) utility APIs

   Various utility functions

   ========================================================================*/

/*--------------------------------------------------------------------------
   Include Files
   ------------------------------------------------------------------------*/
#include <qdf_types.h>
#include <qdf_status.h>
#include <qdf_event.h>
#include <qdf_lock.h>
#include "ani_global.h"

/*--------------------------------------------------------------------------
   Preprocessor definitions and constants
   ------------------------------------------------------------------------*/
#define CDS_DIGEST_SHA1_SIZE    (20)
#define CDS_DIGEST_MD5_SIZE     (16)

#define CDS_24_GHZ_BASE_FREQ   (2407)
#define CDS_5_GHZ_BASE_FREQ    (5000)
#define CDS_24_GHZ_CHANNEL_6   (6)
#define CDS_5_GHZ_CHANNEL_36   (36)
#define CDS_24_GHZ_CHANNEL_14  (14)
#define CDS_24_GHZ_CHANNEL_15  (15)
#define CDS_24_GHZ_CHANNEL_27  (27)
#define CDS_5_GHZ_CHANNEL_170  (170)
#define CDS_CHAN_SPACING_5MHZ  (5)
#define CDS_CHAN_SPACING_20MHZ (20)
#define CDS_CHAN_14_FREQ       (2484)
#define CDS_CHAN_15_FREQ       (2512)
#define CDS_CHAN_170_FREQ      (5852)

#define INVALID_SCAN_ID        0xFFFFFFFF

#define cds_log(level, args...) QDF_TRACE(QDF_MODULE_ID_QDF, level, ## args)
#define cds_logfl(level, format, args...) cds_log(level, FL(format), ## args)

#define cds_alert(format, args...) \
		cds_logfl(QDF_TRACE_LEVEL_FATAL, format, ## args)
#define cds_err(format, args...) \
		cds_logfl(QDF_TRACE_LEVEL_ERROR, format, ## args)
#define cds_warn(format, args...) \
		cds_logfl(QDF_TRACE_LEVEL_WARN, format, ## args)
#define cds_notice(format, args...) \
		cds_logfl(QDF_TRACE_LEVEL_INFO, format, ## args)
#define cds_info(format, args...) \
		cds_logfl(QDF_TRACE_LEVEL_INFO_HIGH, format, ## args)
#define cds_debug(format, args...) \
		cds_logfl(QDF_TRACE_LEVEL_DEBUG, format, ## args)

/**
 * enum cds_band_type - Band type - 2g, 5g or all
 * CDS_BAND_ALL: Both 2G and 5G are valid.
 * CDS_BAND_2GHZ: only 2G is valid.
 * CDS_BAND_5GHZ: only 5G is valid.
 */
enum cds_band_type {
	CDS_BAND_ALL = 0,
	CDS_BAND_2GHZ = 1,
	CDS_BAND_5GHZ = 2
};

/**
 * enum dbs_support - structure to define INI values and their meaning
 * ENABLE_DBS_CXN_AND_SCAN: Enable DBS support for connection and scan
 * DISABLE_DBS_CXN_AND_SCAN: Disable DBS support for connection and scan
 * DISABLE_DBS_CXN_AND_ENABLE_DBS_SCAN: disable dbs support for
 *			connection but keep dbs support for scan
 * DISABLE_DBS_CXN_AND_ENABLE_DBS_SCAN_WITH_ASYNC_SCAN_OFF: disable dbs support
 *			for connection but keep dbs for scan but switch
 *			off the async scan
 * ENABLE_DBS_CXN_AND_ENABLE_SCAN_WITH_ASYNC_SCAN_OFF: enable dbs support for
 *			connection and scan but switch off the async scan
 */
enum dbs_support {
	ENABLE_DBS_CXN_AND_SCAN,
	DISABLE_DBS_CXN_AND_SCAN,
	DISABLE_DBS_CXN_AND_ENABLE_DBS_SCAN,
	DISABLE_DBS_CXN_AND_ENABLE_DBS_SCAN_WITH_ASYNC_SCAN_OFF,
	ENABLE_DBS_CXN_AND_ENABLE_SCAN_WITH_ASYNC_SCAN_OFF,
};

/*-------------------------------------------------------------------------
   Function declarations and documenation
   ------------------------------------------------------------------------*/

QDF_STATUS cds_crypto_init(uint32_t *phCryptProv);

QDF_STATUS cds_crypto_deinit(uint32_t hCryptProv);

/**
 * cds_rand_get_bytes

 * FUNCTION:
 * Returns cryptographically secure pseudo-random bytes.
 *
 *
 * @param pbBuf - the caller allocated location where the bytes should be copied
 * @param numBytes the number of bytes that should be generated and
 * copied
 *
 * @return QDF_STATUS_SUCCSS if the operation succeeds
 */
QDF_STATUS cds_rand_get_bytes(uint32_t handle, uint8_t *pbBuf,
			      uint32_t numBytes);

uint32_t cds_chan_to_freq(uint8_t chan);
uint8_t cds_freq_to_chan(uint32_t freq);
enum cds_band_type cds_chan_to_band(uint32_t chan);

/**
 * cds_upper_to_lower: API to convert upper case string into lower case
 * @txt: input text
 * @length: length of input string
 *
 * Return: None
 */
void cds_upper_to_lower(uint8_t *txt, uint32_t length);
#ifdef WLAN_FEATURE_11W
bool cds_is_mmie_valid(uint8_t *key, uint8_t *ipn,
		       uint8_t *frm, uint8_t *efrm);
bool cds_attach_mmie(uint8_t *igtk, uint8_t *ipn, uint16_t key_id,
		     uint8_t *frm, uint8_t *efrm, uint16_t frmLen);
uint8_t cds_get_mmie_size(void);
#endif /* WLAN_FEATURE_11W */
QDF_STATUS sme_send_flush_logs_cmd_to_fw(tpAniSirGlobal pMac);
static inline void cds_host_diag_log_work(qdf_wake_lock_t *lock, uint32_t msec,
			    uint32_t reason) {
	if (((cds_get_ring_log_level(RING_ID_WAKELOCK) >= WLAN_LOG_LEVEL_ACTIVE)
	     && (WIFI_POWER_EVENT_WAKELOCK_HOLD_RX == reason)) ||
	    (WIFI_POWER_EVENT_WAKELOCK_HOLD_RX != reason)) {
		host_diag_log_wlock(reason, qdf_wake_lock_name(lock),
				    msec, WIFI_POWER_EVENT_WAKELOCK_TAKEN);
	}
}

/**
 * cds_copy_hlp_info() - Copy HLP info
 * @input_dst_mac: input HLP destination MAC address
 * @input_src_mac: input HLP source MAC address
 * @input_hlp_data_len: input HLP data length
 * @input_hlp_data: Pointer to input HLP data
 * @output_dst_mac: output HLP destination MAC address
 * @output_src_mac: output HLP source MAC address
 * @output_hlp_data_len: Pointer to output HLP data length
 * @output_hlp_data: output Pointer to HLP data
 *
 * Util API to copy HLP info from input to output
 *
 * Return: None
 */
void cds_copy_hlp_info(struct qdf_mac_addr *input_dst_mac,
		       struct qdf_mac_addr *input_src_mac,
		       uint16_t input_hlp_data_len,
		       uint8_t *input_hlp_data,
		       struct qdf_mac_addr *output_dst_mac,
		       struct qdf_mac_addr *output_src_mac,
		       uint16_t *output_hlp_data_len,
		       uint8_t *output_hlp_data);
#endif /* #if !defined __CDS_UTILS_H */
