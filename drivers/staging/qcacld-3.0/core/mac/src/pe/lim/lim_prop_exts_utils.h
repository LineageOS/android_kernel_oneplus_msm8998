/*
 * Copyright (c) 2011-2014, 2016-2017 The Linux Foundation. All rights reserved.
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

/*
 *
 * This file lim_prop_exts_utils.h contains the definitions
 * used by all LIM modules to support proprietary features.
 * Author:        Chandra Modumudi
 * Date:          12/11/02
 * History:-
 * Date           Modified by    Modification Information
 * --------------------------------------------------------------------
 *
 */

#ifndef __LIM_PROP_EXTS_UTILS_H
#define __LIM_PROP_EXTS_UTILS_H

/* Function templates */
void limQuietBss(tpAniSirGlobal, uint32_t);
void lim_cleanupMeasData(tpAniSirGlobal);
void limDeleteMeasTimers(tpAniSirGlobal);
void limStopMeasTimers(tpAniSirGlobal pMac);
void lim_cleanupMeasResources(tpAniSirGlobal);
void limRestorePreLearnState(tpAniSirGlobal);
void limCollectMeasurementData(tpAniSirGlobal, uint32_t *, tpSchBeaconStruct);
void limCollectRSSI(tpAniSirGlobal);
void limDeleteCurrentBssWdsNode(tpAniSirGlobal);
uint32_t limComputeAvg(tpAniSirGlobal, uint32_t, uint32_t);

/**
 * lim_check_vendor_ap_present() - checks if the Vendor OUIs are present
 * in the IE buffer
 *
 * @mac_ctx:       mac context.
 * @beacon_struct: pointer to beacon structure
 * @session:       pointer to pe session
 * @ie:            ie buffer
 * @ie_len:        length of ie buffer
 * @id:            action oui id enum
 *
 * This function parses the IE buffer and finds if any of the vendor OUI
 * is present in it.
 *
 * Return: true if the vendor OUI is present, else false
 */
bool lim_check_vendor_ap_present(tpAniSirGlobal mac_ctx,
				 tSirProbeRespBeacon *beacon_struct,
				 tpPESession session,
				 uint8_t *ie, uint16_t ie_len,
				 enum wmi_action_oui_id id);

/* / Function to extract AP's HCF capability from IE fields */
void lim_extract_ap_capability(tpAniSirGlobal, uint8_t *, uint16_t, uint8_t *,
			       uint16_t *, uint8_t *, int8_t *, tpPESession);

ePhyChanBondState lim_get_htcb_state(ePhyChanBondState aniCBMode);

#endif /* __LIM_PROP_EXTS_UTILS_H */
