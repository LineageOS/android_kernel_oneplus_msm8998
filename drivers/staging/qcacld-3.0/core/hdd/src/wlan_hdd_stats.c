/*
 * Copyright (c) 2012-2017 The Linux Foundation. All rights reserved.
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

/**
 * DOC : wlan_hdd_stats.c
 *
 * WLAN Host Device Driver statistics related implementation
 *
 */

#include "wlan_hdd_stats.h"
#include "sme_api.h"
#include "cds_sched.h"
#include "wlan_hdd_trace.h"
#include "wlan_hdd_lpass.h"
#include "hif.h"
#include "wlan_hdd_hostapd.h"
#include "wlan_hdd_debugfs_llstat.h"
#include "wma_api.h"
#include "wma.h"

/* 11B, 11G Rate table include Basic rate and Extended rate
 * The IDX field is the rate index
 * The HI field is the rate when RSSI is strong or being ignored
 *  (in this case we report actual rate)
 * The MID field is the rate when RSSI is moderate
 * (in this case we cap 11b rates at 5.5 and 11g rates at 24)
 * The LO field is the rate when RSSI is low
 *  (in this case we don't report rates, actual current rate used)
 */
static const struct {
	uint8_t beacon_rate_index;
	uint16_t supported_rate[4];
} supported_data_rate[] = {
/* IDX     HI  HM  LM LO (RSSI-based index */
	{
		2, {
			10, 10, 10, 0
		}
	}, {
		4, {
			20, 20, 10, 0
		}
	}, {
		11, {
			55, 20, 10, 0
		}
	}, {
		12, {
			60, 55, 20, 0
		}
	}, {
		18, {
			90, 55, 20, 0
		}
	}, {
		22, {
			110, 55, 20, 0
		}
	}, {
		24, {
			120, 90, 60, 0
		}
	}, {
		36, {
			180, 120, 60, 0
		}
	}, {
		44, {
			220, 180, 60, 0
		}
	}, {
		48, {
			240, 180, 90, 0
		}
	}, {
		66, {
			330, 180, 90, 0
		}
	}, {
		72, {
			360, 240, 90, 0
		}
	}, {
		96, {
			480, 240, 120, 0
		}
	}, {
		108, {
			540, 240, 120, 0
		}
	}
};
/* MCS Based rate table HT MCS parameters with Nss = 1 */
static struct index_data_rate_type supported_mcs_rate_nss1[] = {
/* MCS  L20   L40   S20  S40 */
	{0, {65, 135, 72, 150} },
	{1, {130, 270, 144, 300} },
	{2, {195, 405, 217, 450} },
	{3, {260, 540, 289, 600} },
	{4, {390, 810, 433, 900} },
	{5, {520, 1080, 578, 1200} },
	{6, {585, 1215, 650, 1350} },
	{7, {650, 1350, 722, 1500} }
};

/* HT MCS parameters with Nss = 2 */
static struct index_data_rate_type supported_mcs_rate_nss2[] = {
/* MCS  L20    L40   S20   S40 */
	{0, {130, 270, 144, 300} },
	{1, {260, 540, 289, 600} },
	{2, {390, 810, 433, 900} },
	{3, {520, 1080, 578, 1200} },
	{4, {780, 1620, 867, 1800} },
	{5, {1040, 2160, 1156, 2400} },
	{6, {1170, 2430, 1300, 2700} },
	{7, {1300, 2700, 1444, 3000} }
};

/* MCS Based VHT rate table MCS parameters with Nss = 1*/
static struct index_vht_data_rate_type supported_vht_mcs_rate_nss1[] = {
/* MCS  L80    S80     L40   S40    L20   S40*/
	{0, {293, 325}, {135, 150}, {65, 72} },
	{1, {585, 650}, {270, 300}, {130, 144} },
	{2, {878, 975}, {405, 450}, {195, 217} },
	{3, {1170, 1300}, {540, 600}, {260, 289} },
	{4, {1755, 1950}, {810, 900}, {390, 433} },
	{5, {2340, 2600}, {1080, 1200}, {520, 578} },
	{6, {2633, 2925}, {1215, 1350}, {585, 650} },
	{7, {2925, 3250}, {1350, 1500}, {650, 722} },
	{8, {3510, 3900}, {1620, 1800}, {780, 867} },
	{9, {3900, 4333}, {1800, 2000}, {780, 867} }
};

/*MCS parameters with Nss = 2*/
static struct index_vht_data_rate_type supported_vht_mcs_rate_nss2[] = {
/* MCS  L80    S80     L40   S40    L20   S40*/
	{0, {585, 650}, {270, 300}, {130, 144} },
	{1, {1170, 1300}, {540, 600}, {260, 289} },
	{2, {1755, 1950}, {810, 900}, {390, 433} },
	{3, {2340, 2600}, {1080, 1200}, {520, 578} },
	{4, {3510, 3900}, {1620, 1800}, {780, 867} },
	{5, {4680, 5200}, {2160, 2400}, {1040, 1156} },
	{6, {5265, 5850}, {2430, 2700}, {1170, 1300} },
	{7, {5850, 6500}, {2700, 3000}, {1300, 1444} },
	{8, {7020, 7800}, {3240, 3600}, {1560, 1733} },
	{9, {7800, 8667}, {3600, 4000}, {1560, 1733} }
};

/*array index ponints to MCS and array value points respective rssi*/
static int rssi_mcs_tbl[][10] = {
/*MCS 0   1     2   3    4    5    6    7    8    9*/
	{-82, -79, -77, -74, -70, -66, -65, -64, -59, -57},     /* 20 */
	{-79, -76, -74, -71, -67, -63, -62, -61, -56, -54},     /* 40 */
	{-76, -73, -71, -68, -64, -60, -59, -58, -53, -51} /* 80 */
};


#ifdef WLAN_FEATURE_LINK_LAYER_STATS
static struct hdd_ll_stats_context ll_stats_context;

/**
 * put_wifi_rate_stat() - put wifi rate stats
 * @stats: Pointer to stats context
 * @vendor_event: Pointer to vendor event
 *
 * Return: bool
 */
static bool put_wifi_rate_stat(tpSirWifiRateStat stats,
			       struct sk_buff *vendor_event)
{
	if (nla_put_u8(vendor_event,
		       QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_PREAMBLE,
		       stats->rate.preamble) ||
	    nla_put_u8(vendor_event,
		       QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_NSS,
		       stats->rate.nss) ||
	    nla_put_u8(vendor_event,
		       QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_BW,
		       stats->rate.bw) ||
	    nla_put_u8(vendor_event,
		       QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_MCS_INDEX,
		       stats->rate.rateMcsIdx) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_BIT_RATE,
			stats->rate.bitrate) ||
	    nla_put_u32(vendor_event,
			   QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_TX_MPDU,
			   stats->txMpdu) ||
	    nla_put_u32(vendor_event,
			   QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_RX_MPDU,
			   stats->rxMpdu) ||
	    nla_put_u32(vendor_event,
			   QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_MPDU_LOST,
			   stats->mpduLost) ||
	    nla_put_u32(vendor_event,
			   QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_RETRIES,
			   stats->retries) ||
	    nla_put_u32(vendor_event,
			   QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_RETRIES_SHORT,
			   stats->retriesShort) ||
	    nla_put_u32(vendor_event,
			   QCA_WLAN_VENDOR_ATTR_LL_STATS_RATE_RETRIES_LONG,
			   stats->retriesLong)) {
		hdd_err("QCA_WLAN_VENDOR_ATTR put fail");
		return false;
	}

	return true;
}

/**
 * put_wifi_peer_info() - put wifi peer info
 * @stats: Pointer to stats context
 * @vendor_event: Pointer to vendor event
 *
 * Return: bool
 */
static bool put_wifi_peer_info(tpSirWifiPeerInfo stats,
			       struct sk_buff *vendor_event)
{
	u32 i = 0;
	tpSirWifiRateStat pRateStats;

	if (nla_put_u32
		    (vendor_event, QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO_TYPE,
		    wmi_to_sir_peer_type(stats->type)) ||
	    nla_put(vendor_event,
		       QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO_MAC_ADDRESS,
		       QDF_MAC_ADDR_SIZE, &stats->peerMacAddress.bytes[0]) ||
	    nla_put_u32(vendor_event,
			   QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO_CAPABILITIES,
			   stats->capabilities) ||
	    nla_put_u32(vendor_event,
			   QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO_NUM_RATES,
			   stats->numRate)) {
		hdd_err("QCA_WLAN_VENDOR_ATTR put fail");
		goto error;
	}

	if (stats->numRate) {
		struct nlattr *rateInfo;
		struct nlattr *rates;

		rateInfo = nla_nest_start(vendor_event,
					  QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO_RATE_INFO);
		if (rateInfo == NULL)
			goto error;

		for (i = 0; i < stats->numRate; i++) {
			pRateStats = (tpSirWifiRateStat) ((uint8_t *)
							  stats->rateStats +
							  (i *
							   sizeof
							   (tSirWifiRateStat)));
			rates = nla_nest_start(vendor_event, i);
			if (rates == NULL)
				goto error;

			if (false ==
			    put_wifi_rate_stat(pRateStats, vendor_event)) {
				hdd_err("QCA_WLAN_VENDOR_ATTR put fail");
				return false;
			}
			nla_nest_end(vendor_event, rates);
		}
		nla_nest_end(vendor_event, rateInfo);
	}

	return true;
error:
	return false;
}

/**
 * put_wifi_wmm_ac_stat() - put wifi wmm ac stats
 * @stats: Pointer to stats context
 * @vendor_event: Pointer to vendor event
 *
 * Return: bool
 */
static bool put_wifi_wmm_ac_stat(tpSirWifiWmmAcStat stats,
				 struct sk_buff *vendor_event)
{
	if (nla_put_u32(vendor_event, QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_AC,
			stats->ac) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_TX_MPDU,
			stats->txMpdu) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RX_MPDU,
			stats->rxMpdu) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_TX_MCAST,
			stats->txMcast) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RX_MCAST,
			stats->rxMcast) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RX_AMPDU,
			stats->rxAmpdu) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_TX_AMPDU,
			stats->txAmpdu) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_MPDU_LOST,
			stats->mpduLost) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RETRIES,
			stats->retries) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RETRIES_SHORT,
			stats->retriesShort) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_RETRIES_LONG,
			stats->retriesLong) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_CONTENTION_TIME_MIN,
			stats->contentionTimeMin) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_CONTENTION_TIME_MAX,
			stats->contentionTimeMax) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_CONTENTION_TIME_AVG,
			stats->contentionTimeAvg) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_AC_CONTENTION_NUM_SAMPLES,
			stats->contentionNumSamples)) {
		hdd_err("QCA_WLAN_VENDOR_ATTR put fail");
		return false;
	}

	return true;
}

/**
 * put_wifi_interface_info() - put wifi interface info
 * @stats: Pointer to stats context
 * @vendor_event: Pointer to vendor event
 *
 * Return: bool
 */
static bool put_wifi_interface_info(tpSirWifiInterfaceInfo stats,
				    struct sk_buff *vendor_event)
{
	if (nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_MODE,
			stats->mode) ||
	    nla_put(vendor_event,
		    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_MAC_ADDR,
		    QDF_MAC_ADDR_SIZE, stats->macAddr.bytes) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_STATE,
			stats->state) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_ROAMING,
			stats->roaming) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_CAPABILITIES,
			stats->capabilities) ||
	    nla_put(vendor_event,
		    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_SSID,
		    strlen(stats->ssid), stats->ssid) ||
	    nla_put(vendor_event,
		    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_BSSID,
		    QDF_MAC_ADDR_SIZE, stats->bssid.bytes) ||
	    nla_put(vendor_event,
		    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_AP_COUNTRY_STR,
		    WNI_CFG_COUNTRY_CODE_LEN, stats->apCountryStr) ||
	    nla_put(vendor_event,
		    QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_INFO_COUNTRY_STR,
		    WNI_CFG_COUNTRY_CODE_LEN, stats->countryStr)) {
		hdd_err("QCA_WLAN_VENDOR_ATTR put fail");
		return false;
	}

	return true;
}

/**
 * put_wifi_iface_stats() - put wifi interface stats
 * @pWifiIfaceStat: Pointer to interface stats context
 * @num_peer: Number of peers
 * @vendor_event: Pointer to vendor event
 *
 * Return: bool
 */
static bool put_wifi_iface_stats(tpSirWifiIfaceStat pWifiIfaceStat,
				 u32 num_peers, struct sk_buff *vendor_event)
{
	int i = 0;
	struct nlattr *wmmInfo;
	struct nlattr *wmmStats;
	u64 average_tsf_offset;

	if (false == put_wifi_interface_info(&pWifiIfaceStat->info,
					     vendor_event)) {
		hdd_err("QCA_WLAN_VENDOR_ATTR put fail");
		return false;

	}

	average_tsf_offset =  pWifiIfaceStat->avg_bcn_spread_offset_high;
	average_tsf_offset =  (average_tsf_offset << 32) |
		pWifiIfaceStat->avg_bcn_spread_offset_low;

	if (nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_IFACE) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_NUM_PEERS,
			num_peers) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_BEACON_RX,
			pWifiIfaceStat->beaconRx) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_MGMT_RX,
			pWifiIfaceStat->mgmtRx) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_MGMT_ACTION_RX,
			pWifiIfaceStat->mgmtActionRx) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_MGMT_ACTION_TX,
			pWifiIfaceStat->mgmtActionTx) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_RSSI_MGMT,
			pWifiIfaceStat->rssiMgmt) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_RSSI_DATA,
			pWifiIfaceStat->rssiData) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_RSSI_ACK,
			pWifiIfaceStat->rssiAck) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_LEAKY_AP_DETECTED,
			pWifiIfaceStat->is_leaky_ap) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_LEAKY_AP_AVG_NUM_FRAMES_LEAKED,
			pWifiIfaceStat->avg_rx_frms_leaked) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_LEAKY_AP_GUARD_TIME,
			pWifiIfaceStat->rx_leak_window) ||
	    hdd_wlan_nla_put_u64(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_AVERAGE_TSF_OFFSET,
			average_tsf_offset)  ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_RTS_SUCC_CNT,
			pWifiIfaceStat->rts_succ_cnt) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_RTS_FAIL_CNT,
			pWifiIfaceStat->rts_fail_cnt) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_PPDU_SUCC_CNT,
			pWifiIfaceStat->ppdu_succ_cnt) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_PPDU_FAIL_CNT,
			pWifiIfaceStat->ppdu_fail_cnt)) {
		hdd_err("QCA_WLAN_VENDOR_ATTR put fail");
		return false;
	}

	wmmInfo = nla_nest_start(vendor_event,
				 QCA_WLAN_VENDOR_ATTR_LL_STATS_WMM_INFO);
	if (wmmInfo == NULL)
		return false;

	for (i = 0; i < WIFI_AC_MAX; i++) {
		wmmStats = nla_nest_start(vendor_event, i);
		if (wmmStats == NULL)
			return false;

		if (false ==
		    put_wifi_wmm_ac_stat(&pWifiIfaceStat->AccessclassStats[i],
					 vendor_event)) {
			hdd_err("put_wifi_wmm_ac_stat Fail");
			return false;
		}

		nla_nest_end(vendor_event, wmmStats);
	}
	nla_nest_end(vendor_event, wmmInfo);
	return true;
}

/**
 * hdd_map_device_to_ll_iface_mode() - map device to link layer interface mode
 * @deviceMode: Device mode
 *
 * Return: interface mode
 */
static tSirWifiInterfaceMode hdd_map_device_to_ll_iface_mode(int deviceMode)
{
	switch (deviceMode) {
	case QDF_STA_MODE:
		return WIFI_INTERFACE_STA;
	case QDF_SAP_MODE:
		return WIFI_INTERFACE_SOFTAP;
	case QDF_P2P_CLIENT_MODE:
		return WIFI_INTERFACE_P2P_CLIENT;
	case QDF_P2P_GO_MODE:
		return WIFI_INTERFACE_P2P_GO;
	case QDF_IBSS_MODE:
		return WIFI_INTERFACE_IBSS;
	default:
		/* Return Interface Mode as STA for all the unsupported modes */
		return WIFI_INTERFACE_STA;
	}
}

bool hdd_get_interface_info(hdd_adapter_t *pAdapter,
			    tpSirWifiInterfaceInfo pInfo)
{
	uint8_t *staMac = NULL;
	hdd_station_ctx_t *pHddStaCtx;
	tHalHandle hHal = WLAN_HDD_GET_HAL_CTX(pAdapter);
	tpAniSirGlobal pMac = PMAC_STRUCT(hHal);

	pInfo->mode = hdd_map_device_to_ll_iface_mode(pAdapter->device_mode);

	qdf_copy_macaddr(&pInfo->macAddr, &pAdapter->macAddressCurrent);

	if (((QDF_STA_MODE == pAdapter->device_mode) ||
	     (QDF_P2P_CLIENT_MODE == pAdapter->device_mode) ||
	     (QDF_P2P_DEVICE_MODE == pAdapter->device_mode))) {
		pHddStaCtx = WLAN_HDD_GET_STATION_CTX_PTR(pAdapter);
		if (eConnectionState_NotConnected ==
		    pHddStaCtx->conn_info.connState) {
			pInfo->state = WIFI_DISCONNECTED;
		}
		if (eConnectionState_Connecting ==
		    pHddStaCtx->conn_info.connState) {
			hdd_err("Session ID %d, Connection is in progress",
				pAdapter->sessionId);
			pInfo->state = WIFI_ASSOCIATING;
		}
		if ((eConnectionState_Associated ==
		     pHddStaCtx->conn_info.connState)
		    && (false == pHddStaCtx->conn_info.uIsAuthenticated)) {
			staMac =
				(uint8_t *) &(pAdapter->macAddressCurrent.
					      bytes[0]);
			hdd_warn("client " MAC_ADDRESS_STR
				" is in the middle of WPS/EAPOL exchange.",
				MAC_ADDR_ARRAY(staMac));
			pInfo->state = WIFI_AUTHENTICATING;
		}
		if (eConnectionState_Associated ==
		    pHddStaCtx->conn_info.connState) {
			pInfo->state = WIFI_ASSOCIATED;
			qdf_copy_macaddr(&pInfo->bssid,
					 &pHddStaCtx->conn_info.bssId);
			qdf_mem_copy(pInfo->ssid,
				     pHddStaCtx->conn_info.SSID.SSID.ssId,
				     pHddStaCtx->conn_info.SSID.SSID.length);
			/*
			 * NULL Terminate the string
			 */
			pInfo->ssid[pHddStaCtx->conn_info.SSID.SSID.length] = 0;
		}
	}

	qdf_mem_copy(pInfo->countryStr,
		     pMac->scan.countryCodeCurrent, WNI_CFG_COUNTRY_CODE_LEN);

	qdf_mem_copy(pInfo->apCountryStr,
		     pMac->scan.countryCodeCurrent, WNI_CFG_COUNTRY_CODE_LEN);

	return true;
}

/**
 * hdd_link_layer_process_peer_stats() - This function is called after
 * @pAdapter: Pointer to device adapter
 * @more_data: More data
 * @pData: Pointer to stats data
 *
 * Receiving Link Layer Peer statistics from FW.This function converts
 * the firmware data to the NL data and sends the same to the kernel/upper
 * layers.
 *
 * Return: None
 */
static void hdd_link_layer_process_peer_stats(hdd_adapter_t *pAdapter,
					      u32 more_data,
					      tpSirWifiPeerStat pData)
{
	hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);
	tpSirWifiPeerStat pWifiPeerStat;
	tpSirWifiPeerInfo pWifiPeerInfo;
	struct sk_buff *vendor_event;
	int status, i;
	struct nlattr *peers;
	int numRate;

	ENTER();

	pWifiPeerStat = pData;

	status = wlan_hdd_validate_context(pHddCtx);
	if (0 != status)
		return;

	hdd_debug("LL_STATS_PEER_ALL : numPeers %u, more data = %u",
		   pWifiPeerStat->numPeers, more_data);

	/*
	 * Allocate a size of 4096 for the peer stats comprising
	 * each of size = sizeof (tSirWifiPeerInfo) + numRate *
	 * sizeof (tSirWifiRateStat).Each field is put with an
	 * NL attribute.The size of 4096 is considered assuming
	 * that number of rates shall not exceed beyond 50 with
	 * the sizeof (tSirWifiRateStat) being 32.
	 */
	vendor_event = cfg80211_vendor_cmd_alloc_reply_skb(pHddCtx->wiphy,
				LL_STATS_EVENT_BUF_SIZE);

	if (!vendor_event) {
		hdd_err("cfg80211_vendor_cmd_alloc_reply_skb failed");
		return;
	}

	if (nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_PEER) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RESULTS_MORE_DATA,
			more_data) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_IFACE_NUM_PEERS,
			pWifiPeerStat->numPeers)) {
		hdd_err("QCA_WLAN_VENDOR_ATTR put fail");

		kfree_skb(vendor_event);
		return;
	}

	pWifiPeerInfo = (tpSirWifiPeerInfo) ((uint8_t *)
					     pWifiPeerStat->peerInfo);

	if (pWifiPeerStat->numPeers) {
		struct nlattr *peerInfo;

		peerInfo = nla_nest_start(vendor_event,
					  QCA_WLAN_VENDOR_ATTR_LL_STATS_PEER_INFO);
		if (peerInfo == NULL) {
			hdd_err("nla_nest_start failed");
			kfree_skb(vendor_event);
			return;
		}

		for (i = 1; i <= pWifiPeerStat->numPeers; i++) {
			peers = nla_nest_start(vendor_event, i);
			if (peers == NULL) {
				hdd_err("nla_nest_start failed");
				kfree_skb(vendor_event);
				return;
			}

			numRate = pWifiPeerInfo->numRate;

			if (false ==
			    put_wifi_peer_info(pWifiPeerInfo, vendor_event)) {
				hdd_err("put_wifi_peer_info fail");
				kfree_skb(vendor_event);
				return;
			}

			pWifiPeerInfo = (tpSirWifiPeerInfo) ((uint8_t *)
							     pWifiPeerStat->
							     peerInfo +
							     (i *
							      sizeof
							      (tSirWifiPeerInfo))
							     +
							     (numRate *
							      sizeof
							      (tSirWifiRateStat)));
			nla_nest_end(vendor_event, peers);
		}
		nla_nest_end(vendor_event, peerInfo);
	}

	cfg80211_vendor_cmd_reply(vendor_event);
	EXIT();
}

/**
 * hdd_link_layer_process_iface_stats() - This function is called after
 * @pAdapter: Pointer to device adapter
 * @pData: Pointer to stats data
 * @num_peers: Number of peers
 *
 * Receiving Link Layer Interface statistics from FW.This function converts
 * the firmware data to the NL data and sends the same to the kernel/upper
 * layers.
 *
 * Return: None
 */
static void hdd_link_layer_process_iface_stats(hdd_adapter_t *pAdapter,
					       tpSirWifiIfaceStat pData,
					       u32 num_peers)
{
	tpSirWifiIfaceStat pWifiIfaceStat;
	struct sk_buff *vendor_event;
	hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);
	int status;

	ENTER();

	pWifiIfaceStat = pData;

	status = wlan_hdd_validate_context(pHddCtx);
	if (0 != status)
		return;

	/*
	 * Allocate a size of 4096 for the interface stats comprising
	 * sizeof (tpSirWifiIfaceStat).The size of 4096 is considered
	 * assuming that all these fit with in the limit.Please take
	 * a call on the limit based on the data requirements on
	 * interface statistics.
	 */
	vendor_event = cfg80211_vendor_cmd_alloc_reply_skb(pHddCtx->wiphy,
				LL_STATS_EVENT_BUF_SIZE);

	if (!vendor_event) {
		hdd_err("cfg80211_vendor_cmd_alloc_reply_skb failed");
		return;
	}

	hdd_debug("WMI_LINK_STATS_IFACE Data");

	if (false == hdd_get_interface_info(pAdapter, &pWifiIfaceStat->info)) {
		hdd_err("hdd_get_interface_info get fail");
		kfree_skb(vendor_event);
		return;
	}

	if (false ==
	    put_wifi_iface_stats(pWifiIfaceStat, num_peers, vendor_event)) {
		hdd_err("put_wifi_iface_stats fail");
		kfree_skb(vendor_event);
		return;
	}

	cfg80211_vendor_cmd_reply(vendor_event);
	EXIT();
}

/**
 * hdd_llstats_radio_fill_channels() - radio stats fill channels
 * @adapter: Pointer to device adapter
 * @radiostat: Pointer to stats data
 * @vendor_event: vendor event
 *
 * Return: 0 on success; errno on failure
 */
static int hdd_llstats_radio_fill_channels(hdd_adapter_t *adapter,
					   tSirWifiRadioStat *radiostat,
					   struct sk_buff *vendor_event)
{
	tSirWifiChannelStats *channel_stats;
	struct nlattr *chlist;
	struct nlattr *chinfo;
	int i;

	chlist = nla_nest_start(vendor_event,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_CH_INFO);
	if (chlist == NULL) {
		hdd_err("nla_nest_start failed");
		return -EINVAL;
	}

	for (i = 0; i < radiostat->numChannels; i++) {
		channel_stats = (tSirWifiChannelStats *) ((uint8_t *)
				     radiostat->channels +
				     (i * sizeof(tSirWifiChannelStats)));

		chinfo = nla_nest_start(vendor_event, i);
		if (chinfo == NULL) {
			hdd_err("nla_nest_start failed");
			return -EINVAL;
		}

		if (nla_put_u32(vendor_event,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_INFO_WIDTH,
				channel_stats->channel.width) ||
		    nla_put_u32(vendor_event,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_INFO_CENTER_FREQ,
				channel_stats->channel.centerFreq) ||
		    nla_put_u32(vendor_event,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_INFO_CENTER_FREQ0,
				channel_stats->channel.centerFreq0) ||
		    nla_put_u32(vendor_event,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_INFO_CENTER_FREQ1,
				channel_stats->channel.centerFreq1) ||
		    nla_put_u32(vendor_event,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_ON_TIME,
				channel_stats->onTime) ||
		    nla_put_u32(vendor_event,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_CHANNEL_CCA_BUSY_TIME,
				channel_stats->ccaBusyTime)) {
			hdd_err("nla_put failed");
			return -EINVAL;
		}
		nla_nest_end(vendor_event, chinfo);
	}
	nla_nest_end(vendor_event, chlist);

	return 0;
}

/**
 * hdd_llstats_post_radio_stats() - post radio stats
 * @adapter: Pointer to device adapter
 * @more_data: More data
 * @radiostat: Pointer to stats data
 * @num_radio: Number of radios
 *
 * Return: 0 on success; errno on failure
 */
static int hdd_llstats_post_radio_stats(hdd_adapter_t *adapter,
					u32 more_data,
					tSirWifiRadioStat *radiostat,
					u32 num_radio)
{
	struct sk_buff *vendor_event;
	hdd_context_t *hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	int ret;

	/*
	 * Allocate a size of 4096 for the Radio stats comprising
	 * sizeof (tSirWifiRadioStat) + numChannels * sizeof
	 * (tSirWifiChannelStats).Each channel data is put with an
	 * NL attribute.The size of 4096 is considered assuming that
	 * number of channels shall not exceed beyond  60 with the
	 * sizeof (tSirWifiChannelStats) being 24 bytes.
	 */

	vendor_event = cfg80211_vendor_cmd_alloc_reply_skb(
					hdd_ctx->wiphy,
					LL_STATS_EVENT_BUF_SIZE);

	if (!vendor_event) {
		hdd_err("cfg80211_vendor_cmd_alloc_reply_skb failed");
		return -ENOMEM;
	}

	if (nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_TYPE_RADIO) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RESULTS_MORE_DATA,
			more_data) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_NUM_RADIOS,
			num_radio) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ID,
			radiostat->radio) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME,
			radiostat->onTime) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_TX_TIME,
			radiostat->txTime) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_RX_TIME,
			radiostat->rxTime) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_SCAN,
			radiostat->onTimeScan) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_NBD,
			radiostat->onTimeNbd) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_GSCAN,
			radiostat->onTimeGscan) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_ROAM_SCAN,
			radiostat->onTimeRoamScan) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_PNO_SCAN,
			radiostat->onTimePnoScan) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_ON_TIME_HS20,
			radiostat->onTimeHs20) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_NUM_TX_LEVELS,
			radiostat->total_num_tx_power_levels)    ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_NUM_CHANNELS,
			radiostat->numChannels)) {
		hdd_err("QCA_WLAN_VENDOR_ATTR put fail");
		goto failure;
	}

	if (radiostat->total_num_tx_power_levels) {
		if (nla_put(vendor_event,
			    QCA_WLAN_VENDOR_ATTR_LL_STATS_RADIO_TX_TIME_PER_LEVEL,
			    sizeof(u32) *
			    radiostat->total_num_tx_power_levels,
			    radiostat->tx_time_per_power_level)) {
			hdd_err("nla_put fail");
			goto failure;
		}
	}

	if (radiostat->numChannels) {
		ret = hdd_llstats_radio_fill_channels(adapter, radiostat,
						      vendor_event);
		if (ret)
			goto failure;
	}

	cfg80211_vendor_cmd_reply(vendor_event);
	return 0;

failure:
	kfree_skb(vendor_event);
	return -EINVAL;
}

/**
 * hdd_link_layer_process_radio_stats() - This function is called after
 * @pAdapter: Pointer to device adapter
 * @more_data: More data
 * @pData: Pointer to stats data
 * @num_radios: Number of radios
 *
 * Receiving Link Layer Radio statistics from FW.This function converts
 * the firmware data to the NL data and sends the same to the kernel/upper
 * layers.
 *
 * Return: None
 */
static void hdd_link_layer_process_radio_stats(hdd_adapter_t *pAdapter,
					       u32 more_data,
					       tpSirWifiRadioStat pData,
					       u32 num_radio)
{
	int status, i, nr, ret;
	tSirWifiRadioStat *pWifiRadioStat = pData;
	hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

	ENTER();

	status = wlan_hdd_validate_context(pHddCtx);
	if (0 != status)
		return;

	hdd_debug("LL_STATS_RADIO: number of radios: %u", num_radio);

	for (i = 0; i < num_radio; i++) {
		hdd_debug("LL_STATS_RADIO"
		       " radio: %u onTime: %u txTime: %u rxTime: %u"
		       " onTimeScan: %u onTimeNbd: %u"
		       " onTimeGscan: %u onTimeRoamScan: %u"
		       " onTimePnoScan: %u  onTimeHs20: %u"
		       " numChannels: %u total_num_tx_pwr_levels: %u"
		       " on_time_host_scan: %u, on_time_lpi_scan: %u",
		       pWifiRadioStat->radio, pWifiRadioStat->onTime,
		       pWifiRadioStat->txTime, pWifiRadioStat->rxTime,
		       pWifiRadioStat->onTimeScan, pWifiRadioStat->onTimeNbd,
		       pWifiRadioStat->onTimeGscan,
		       pWifiRadioStat->onTimeRoamScan,
		       pWifiRadioStat->onTimePnoScan,
		       pWifiRadioStat->onTimeHs20, pWifiRadioStat->numChannels,
		       pWifiRadioStat->total_num_tx_power_levels,
		       pWifiRadioStat->on_time_host_scan,
		       pWifiRadioStat->on_time_lpi_scan);
		pWifiRadioStat++;
	}

	pWifiRadioStat = pData;
	for (nr = 0; nr < num_radio; nr++) {
		ret = hdd_llstats_post_radio_stats(pAdapter, more_data,
						   pWifiRadioStat, num_radio);
		if (ret)
			return;

		pWifiRadioStat++;
	}

	EXIT();
}

/**
 * hdd_ll_process_radio_stats() - Wrapper function for cfg80211/debugfs
 * @adapter: Pointer to device adapter
 * @more_data: More data
 * @data: Pointer to stats data
 * @num_radios: Number of radios
 * @resp_id: Response ID from FW
 *
 * Receiving Link Layer Radio statistics from FW. This function is a wrapper
 * function which calls cfg80211/debugfs functions based on the response ID.
 *
 * Return: None
 */
static void hdd_ll_process_radio_stats(hdd_adapter_t *adapter,
		uint32_t more_data, void *data, uint32_t num_radio,
		uint32_t resp_id)
{
	if (DEBUGFS_LLSTATS_REQID == resp_id)
		hdd_debugfs_process_radio_stats(adapter, more_data,
			(tpSirWifiRadioStat)data, num_radio);
	else
		hdd_link_layer_process_radio_stats(adapter, more_data,
			(tpSirWifiRadioStat)data, num_radio);
}

/**
 * hdd_ll_process_iface_stats() - Wrapper function for cfg80211/debugfs
 * @adapter: Pointer to device adapter
 * @data: Pointer to stats data
 * @num_peers: Number of peers
 * @resp_id: Response ID from FW
 *
 * Receiving Link Layer Radio statistics from FW. This function is a wrapper
 * function which calls cfg80211/debugfs functions based on the response ID.
 *
 * Return: None
 */
static void hdd_ll_process_iface_stats(hdd_adapter_t *adapter,
			void *data, uint32_t num_peers, uint32_t resp_id)
{
	if (DEBUGFS_LLSTATS_REQID == resp_id)
		hdd_debugfs_process_iface_stats(adapter,
				(tpSirWifiIfaceStat) data, num_peers);
	else
		hdd_link_layer_process_iface_stats(adapter,
				(tpSirWifiIfaceStat) data, num_peers);
}

/**
 * hdd_ll_process_peer_stats() - Wrapper function for cfg80211/debugfs
 * @adapter: Pointer to device adapter
 * @more_data: More data
 * @data: Pointer to stats data
 * @resp_id: Response ID from FW
 *
 * Receiving Link Layer Radio statistics from FW. This function is a wrapper
 * function which calls cfg80211/debugfs functions based on the response ID.
 *
 * Return: None
 */
static void hdd_ll_process_peer_stats(hdd_adapter_t *adapter,
		uint32_t more_data, void *data, uint32_t resp_id)
{
	if (DEBUGFS_LLSTATS_REQID == resp_id)
		hdd_debugfs_process_peer_stats(adapter, data);
	else
		hdd_link_layer_process_peer_stats(adapter, more_data,
						  (tpSirWifiPeerStat) data);
}

/**
 * wlan_hdd_cfg80211_link_layer_stats_callback() - This function is called
 * @ctx: Pointer to hdd context
 * @indType: Indication type
 * @pRsp: Pointer to response
 *
 * After receiving Link Layer indications from FW.This callback converts the
 * firmware data to the NL data and send the same to the kernel/upper layers.
 *
 * Return: None
 */
void wlan_hdd_cfg80211_link_layer_stats_callback(void *ctx,
							int indType, void *pRsp)
{
	hdd_context_t *pHddCtx = (hdd_context_t *) ctx;
	struct hdd_ll_stats_context *context;
	hdd_adapter_t *pAdapter = NULL;
	tpSirLLStatsResults linkLayerStatsResults = (tpSirLLStatsResults) pRsp;
	int status;

	status = wlan_hdd_validate_context(pHddCtx);
	if (status)
		return;

	pAdapter = hdd_get_adapter_by_vdev(pHddCtx,
					   linkLayerStatsResults->ifaceId);

	if (NULL == pAdapter) {
		hdd_err("vdev_id %d does not exist with host",
			linkLayerStatsResults->ifaceId);
		return;
	}

	hdd_debug("Link Layer Indication indType: %d", indType);

	switch (indType) {
	case SIR_HAL_LL_STATS_RESULTS_RSP:
	{
		hdd_debug("LL_STATS RESP paramID = 0x%x, ifaceId = %u, respId= %u , moreResultToFollow = %u, num radio = %u result = %p",
			linkLayerStatsResults->paramId,
			linkLayerStatsResults->ifaceId,
			linkLayerStatsResults->rspId,
			linkLayerStatsResults->moreResultToFollow,
			linkLayerStatsResults->num_radio,
			linkLayerStatsResults->results);

		context = &ll_stats_context;
		spin_lock(&context->context_lock);
		/* validate response received from target */
		if ((context->request_id != linkLayerStatsResults->rspId) ||
		  !(context->request_bitmap & linkLayerStatsResults->paramId)) {
			spin_unlock(&context->context_lock);
			hdd_err("Error : Request id %d response id %d request bitmap 0x%x response bitmap 0x%x",
			context->request_id, linkLayerStatsResults->rspId,
			context->request_bitmap, linkLayerStatsResults->paramId);
			return;
		}
		spin_unlock(&context->context_lock);

		if (linkLayerStatsResults->paramId & WMI_LINK_STATS_RADIO) {
			hdd_ll_process_radio_stats(pAdapter,
				linkLayerStatsResults->moreResultToFollow,
				linkLayerStatsResults->results,
				linkLayerStatsResults->num_radio,
				linkLayerStatsResults->rspId);

			spin_lock(&context->context_lock);
			if (!linkLayerStatsResults->moreResultToFollow)
				context->request_bitmap &= ~(WMI_LINK_STATS_RADIO);
			spin_unlock(&context->context_lock);

		} else if (linkLayerStatsResults->paramId &
				WMI_LINK_STATS_IFACE) {
			hdd_ll_process_iface_stats(pAdapter,
				linkLayerStatsResults->results,
				linkLayerStatsResults->num_peers,
				linkLayerStatsResults->rspId);

			spin_lock(&context->context_lock);
			/* Firmware doesn't send peerstats event if no peers are
			 * connected. HDD should not wait for any peerstats in
			 * this case and return the status to middleware after
			 * receiving iface stats
			 */
			if (!linkLayerStatsResults->num_peers)
				context->request_bitmap &=
					~(WMI_LINK_STATS_ALL_PEER);
			context->request_bitmap &= ~(WMI_LINK_STATS_IFACE);
			spin_unlock(&context->context_lock);

		} else if (linkLayerStatsResults->
			   paramId & WMI_LINK_STATS_ALL_PEER) {
			hdd_ll_process_peer_stats(pAdapter,
				linkLayerStatsResults->moreResultToFollow,
				linkLayerStatsResults->results,
				linkLayerStatsResults->rspId);

			spin_lock(&context->context_lock);
			if (!linkLayerStatsResults->moreResultToFollow)
				context->request_bitmap &= ~(WMI_LINK_STATS_ALL_PEER);
			spin_unlock(&context->context_lock);

		} else {
			hdd_err("INVALID LL_STATS_NOTIFY RESPONSE");
		}

		spin_lock(&context->context_lock);
		/* complete response event if all requests are completed */
		if (0 == context->request_bitmap)
			complete(&context->response_event);
		spin_unlock(&context->context_lock);

		break;
	}
	default:
		hdd_warn("invalid event type %d", indType);
		break;
	}
}

void hdd_lost_link_info_cb(void *context,
				  struct sir_lost_link_info *lost_link_info)
{
	hdd_context_t *hdd_ctx = (hdd_context_t *)context;
	int status;
	hdd_adapter_t *adapter;

	status = wlan_hdd_validate_context(hdd_ctx);
	if (0 != status)
		return;

	if (NULL == lost_link_info) {
		hdd_err("lost_link_info is NULL");
		return;
	}

	adapter = hdd_get_adapter_by_vdev(hdd_ctx, lost_link_info->vdev_id);
	if (NULL == adapter) {
		hdd_err("invalid adapter");
		return;
	}

	adapter->rssi_on_disconnect = lost_link_info->rssi;
	hdd_debug("rssi on disconnect %d", adapter->rssi_on_disconnect);
}

const struct
nla_policy
	qca_wlan_vendor_ll_set_policy[QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_MAX + 1] = {
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_CONFIG_MPDU_SIZE_THRESHOLD] = {
						.type = NLA_U32},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_CONFIG_AGGRESSIVE_STATS_GATHERING] = {
						.type = NLA_U32},
};

/**
 * __wlan_hdd_cfg80211_ll_stats_set() - set link layer stats
 * @wiphy: Pointer to wiphy
 * @wdev: Pointer to wdev
 * @data: Pointer to data
 * @data_len: Data length
 *
 * Return: int
 */
static int
__wlan_hdd_cfg80211_ll_stats_set(struct wiphy *wiphy,
				   struct wireless_dev *wdev,
				   const void *data,
				   int data_len)
{
	int status;
	struct nlattr *tb_vendor[QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_MAX + 1];
	tSirLLStatsSetReq LinkLayerStatsSetReq;
	struct net_device *dev = wdev->netdev;
	hdd_adapter_t *pAdapter = WLAN_HDD_GET_PRIV_PTR(dev);
	hdd_context_t *pHddCtx = wiphy_priv(wiphy);

	ENTER_DEV(dev);

	if (QDF_GLOBAL_FTM_MODE == hdd_get_conparam()) {
		hdd_warn("Command not allowed in FTM mode");
		return -EPERM;
	}

	status = wlan_hdd_validate_context(pHddCtx);
	if (0 != status)
		return -EINVAL;

	if (nla_parse(tb_vendor, QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_MAX,
		      (struct nlattr *)data,
		      data_len, qca_wlan_vendor_ll_set_policy)) {
		hdd_err("maximum attribute not present");
		return -EINVAL;
	}

	if (!tb_vendor
	    [QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_CONFIG_MPDU_SIZE_THRESHOLD]) {
		hdd_err("MPDU size Not present");
		return -EINVAL;
	}

	if (!tb_vendor
	    [QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_CONFIG_AGGRESSIVE_STATS_GATHERING]) {
		hdd_err("Stats Gathering Not Present");
		return -EINVAL;
	}

	/* Shall take the request Id if the Upper layers pass. 1 For now. */
	LinkLayerStatsSetReq.reqId = 1;

	LinkLayerStatsSetReq.mpduSizeThreshold =
		nla_get_u32(tb_vendor
			    [QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_CONFIG_MPDU_SIZE_THRESHOLD]);

	LinkLayerStatsSetReq.aggressiveStatisticsGathering =
		nla_get_u32(tb_vendor
			    [QCA_WLAN_VENDOR_ATTR_LL_STATS_SET_CONFIG_AGGRESSIVE_STATS_GATHERING]);

	LinkLayerStatsSetReq.staId = pAdapter->sessionId;

	hdd_debug("LL_STATS_SET reqId = %d, staId = %d, mpduSizeThreshold = %d, Statistics Gathering = %d",
		LinkLayerStatsSetReq.reqId, LinkLayerStatsSetReq.staId,
		LinkLayerStatsSetReq.mpduSizeThreshold,
		LinkLayerStatsSetReq.aggressiveStatisticsGathering);

	if (QDF_STATUS_SUCCESS != sme_ll_stats_set_req(pHddCtx->hHal,
						       &LinkLayerStatsSetReq)) {
		hdd_err("sme_ll_stats_set_req Failed");
		return -EINVAL;
	}

	pAdapter->isLinkLayerStatsSet = 1;
	EXIT();
	return 0;
}

/**
 * wlan_hdd_cfg80211_ll_stats_set() - set ll stats
 * @wiphy: Pointer to wiphy
 * @wdev: Pointer to wdev
 * @data: Pointer to data
 * @data_len: Data length
 *
 * Return: 0 if success, non-zero for failure
 */
int wlan_hdd_cfg80211_ll_stats_set(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data,
					int data_len)
{
	int ret = 0;

	cds_ssr_protect(__func__);
	ret = __wlan_hdd_cfg80211_ll_stats_set(wiphy, wdev, data, data_len);
	cds_ssr_unprotect(__func__);

	return ret;
}

const struct
nla_policy
	qca_wlan_vendor_ll_get_policy[QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_MAX + 1] = {
	/* Unsigned 32bit value provided by the caller issuing the GET stats
	 * command. When reporting
	 * the stats results, the driver uses the same value to indicate
	 * which GET request the results
	 * correspond to.
	 */
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_CONFIG_REQ_ID] = {.type = NLA_U32},

	/* Unsigned 32bit value . bit mask to identify what statistics are
	 * requested for retrieval
	 */
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_CONFIG_REQ_MASK] = {.type = NLA_U32}
};

static int wlan_hdd_send_ll_stats_req(hdd_context_t *hdd_ctx,
				      tSirLLStatsGetReq *req)
{
	unsigned long rc;
	struct hdd_ll_stats_context *context;

	context = &ll_stats_context;
	spin_lock(&context->context_lock);
	context->request_id = req->reqId;
	context->request_bitmap = req->paramIdMask;
	INIT_COMPLETION(context->response_event);
	spin_unlock(&context->context_lock);

	if (QDF_STATUS_SUCCESS !=
			sme_ll_stats_get_req(hdd_ctx->hHal, req)) {
		hdd_err("sme_ll_stats_get_req Failed");
		return -EINVAL;
	}

	rc = wait_for_completion_timeout(&context->response_event,
			msecs_to_jiffies(WLAN_WAIT_TIME_LL_STATS));
	if (!rc) {
		hdd_err("Target response timed out request id %d request bitmap 0x%x",
			context->request_id, context->request_bitmap);
		return -ETIMEDOUT;
	}

	return 0;
}

int wlan_hdd_ll_stats_get(hdd_adapter_t *adapter, uint32_t req_id,
			  uint32_t req_mask)
{
	int ret;
	tSirLLStatsGetReq get_req;
	hdd_station_ctx_t *hddstactx = WLAN_HDD_GET_STATION_CTX_PTR(adapter);
	hdd_context_t *hdd_ctx = WLAN_HDD_GET_CTX(adapter);

	ENTER();

	if (QDF_GLOBAL_FTM_MODE == hdd_get_conparam()) {
		hdd_warn("Command not allowed in FTM mode");
		return -EPERM;
	}

	ret = wlan_hdd_validate_context(hdd_ctx);
	if (0 != ret)
		return -EINVAL;

	if (hddstactx->hdd_ReassocScenario) {
		hdd_err("Roaming in progress, cannot process the request");
		return -EBUSY;
	}

	if (!adapter->isLinkLayerStatsSet)
		hdd_info("isLinkLayerStatsSet: %d; STATs will be all zero",
			adapter->isLinkLayerStatsSet);

	get_req.reqId = req_id;
	get_req.paramIdMask = req_mask;
	get_req.staId = adapter->sessionId;

	if (wlan_hdd_validate_session_id(adapter->sessionId)) {
		hdd_err("invalid session id: %d", adapter->sessionId);
		return -EINVAL;
	}

	rtnl_lock();
	ret = wlan_hdd_send_ll_stats_req(hdd_ctx, &get_req);
	rtnl_unlock();
	if (0 != ret)
		hdd_err("Send LL stats req failed, id:%u, mask:%d, session:%d",
			req_id, req_mask, adapter->sessionId);

	EXIT();
	return ret;

}

/**
 * __wlan_hdd_cfg80211_ll_stats_get() - get link layer stats
 * @wiphy: Pointer to wiphy
 * @wdev: Pointer to wdev
 * @data: Pointer to data
 * @data_len: Data length
 *
 * Return: int
 */
static int
__wlan_hdd_cfg80211_ll_stats_get(struct wiphy *wiphy,
				   struct wireless_dev *wdev,
				   const void *data,
				   int data_len)
{
	int ret;
	hdd_context_t *pHddCtx = wiphy_priv(wiphy);
	struct nlattr *tb_vendor[QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_MAX + 1];
	tSirLLStatsGetReq LinkLayerStatsGetReq;
	struct net_device *dev = wdev->netdev;
	hdd_adapter_t *pAdapter = WLAN_HDD_GET_PRIV_PTR(dev);
	hdd_station_ctx_t *hddstactx = WLAN_HDD_GET_STATION_CTX_PTR(pAdapter);

	/* ENTER() intentionally not used in a frequently invoked API */

	if (QDF_GLOBAL_FTM_MODE == hdd_get_conparam()) {
		hdd_warn("Command not allowed in FTM mode");
		return -EPERM;
	}

	ret = wlan_hdd_validate_context(pHddCtx);
	if (0 != ret)
		return -EINVAL;

	if (!pAdapter->isLinkLayerStatsSet) {
		hdd_warn("isLinkLayerStatsSet: %d",
			 pAdapter->isLinkLayerStatsSet);
		return -EINVAL;
	}

	if (hddstactx->hdd_ReassocScenario) {
		hdd_err("Roaming in progress, cannot process the request");
		return -EBUSY;
	}

	if (nla_parse(tb_vendor, QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_MAX,
		      (struct nlattr *)data,
		      data_len, qca_wlan_vendor_ll_get_policy)) {
		hdd_err("max attribute not present");
		return -EINVAL;
	}

	if (!tb_vendor[QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_CONFIG_REQ_ID]) {
		hdd_err("Request Id Not present");
		return -EINVAL;
	}

	if (!tb_vendor[QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_CONFIG_REQ_MASK]) {
		hdd_err("Req Mask Not present");
		return -EINVAL;
	}

	LinkLayerStatsGetReq.reqId =
		nla_get_u32(tb_vendor
			    [QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_CONFIG_REQ_ID]);
	LinkLayerStatsGetReq.paramIdMask =
		nla_get_u32(tb_vendor
			    [QCA_WLAN_VENDOR_ATTR_LL_STATS_GET_CONFIG_REQ_MASK]);

	LinkLayerStatsGetReq.staId = pAdapter->sessionId;

	if (wlan_hdd_validate_session_id(pAdapter->sessionId)) {
		hdd_err("invalid session id: %d", pAdapter->sessionId);
		return -EINVAL;
	}

	ret = wlan_hdd_send_ll_stats_req(pHddCtx, &LinkLayerStatsGetReq);
	if (0 != ret) {
		hdd_err("Failed to send LL stats request (id:%u)",
			LinkLayerStatsGetReq.reqId);
		return ret;
	}

	EXIT();
	return 0;
}

/**
 * wlan_hdd_cfg80211_ll_stats_get() - get ll stats
 * @wiphy: Pointer to wiphy
 * @wdev: Pointer to wdev
 * @data: Pointer to data
 * @data_len: Data length
 *
 * Return: 0 if success, non-zero for failure
 */
int wlan_hdd_cfg80211_ll_stats_get(struct wiphy *wiphy,
				struct wireless_dev *wdev,
				const void *data,
				int data_len)
{
	int ret = 0;

	cds_ssr_protect(__func__);
	ret = __wlan_hdd_cfg80211_ll_stats_get(wiphy, wdev, data, data_len);
	cds_ssr_unprotect(__func__);

	return ret;
}

const struct
nla_policy
	qca_wlan_vendor_ll_clr_policy[QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_MAX + 1] = {
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_REQ_MASK] = {.type = NLA_U32},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_STOP_REQ] = {.type = NLA_U8},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_RSP_MASK] = {.type = NLA_U32},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_STOP_RSP] = {.type = NLA_U8},
};

/**
 * __wlan_hdd_cfg80211_ll_stats_clear() - clear link layer stats
 * @wiphy: Pointer to wiphy
 * @wdev: Pointer to wdev
 * @data: Pointer to data
 * @data_len: Data length
 *
 * Return: int
 */
static int
__wlan_hdd_cfg80211_ll_stats_clear(struct wiphy *wiphy,
				    struct wireless_dev *wdev,
				    const void *data,
				    int data_len)
{
	hdd_context_t *pHddCtx = wiphy_priv(wiphy);
	struct nlattr *tb_vendor[QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_MAX + 1];
	tSirLLStatsClearReq LinkLayerStatsClearReq;
	struct net_device *dev = wdev->netdev;
	hdd_adapter_t *pAdapter = WLAN_HDD_GET_PRIV_PTR(dev);
	u32 statsClearReqMask;
	u8 stopReq;
	int status;
	struct sk_buff *temp_skbuff;

	ENTER_DEV(dev);

	if (QDF_GLOBAL_FTM_MODE == hdd_get_conparam()) {
		hdd_warn("Command not allowed in FTM mode");
		return -EPERM;
	}

	status = wlan_hdd_validate_context(pHddCtx);
	if (0 != status)
		return -EINVAL;

	if (!pAdapter->isLinkLayerStatsSet) {
		hdd_warn("isLinkLayerStatsSet : %d",
			  pAdapter->isLinkLayerStatsSet);
		return -EINVAL;
	}

	if (nla_parse(tb_vendor, QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_MAX,
		      (struct nlattr *)data,
		      data_len, qca_wlan_vendor_ll_clr_policy)) {
		hdd_err("STATS_CLR_MAX is not present");
		return -EINVAL;
	}

	if (!tb_vendor[QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_REQ_MASK] ||
	    !tb_vendor[QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_STOP_REQ]) {
		hdd_err("Error in LL_STATS CLR CONFIG PARA");
		return -EINVAL;
	}

	statsClearReqMask = LinkLayerStatsClearReq.statsClearReqMask =
				    nla_get_u32(tb_vendor
						[QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_REQ_MASK]);

	stopReq = LinkLayerStatsClearReq.stopReq =
			  nla_get_u8(tb_vendor
				     [QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_STOP_REQ]);

	/*
	 * Shall take the request Id if the Upper layers pass. 1 For now.
	 */
	LinkLayerStatsClearReq.reqId = 1;

	LinkLayerStatsClearReq.staId = pAdapter->sessionId;

	hdd_debug("LL_STATS_CLEAR reqId = %d, staId = %d, statsClearReqMask = 0x%X, stopReq = %d",
		LinkLayerStatsClearReq.reqId,
		LinkLayerStatsClearReq.staId,
		LinkLayerStatsClearReq.statsClearReqMask,
		LinkLayerStatsClearReq.stopReq);

	if (QDF_STATUS_SUCCESS == sme_ll_stats_clear_req(pHddCtx->hHal,
					&LinkLayerStatsClearReq)) {
		temp_skbuff = cfg80211_vendor_cmd_alloc_reply_skb(wiphy,
								  2 *
								  sizeof(u32) +
								  2 *
								  NLMSG_HDRLEN);
		if (temp_skbuff != NULL) {
			if (nla_put_u32(temp_skbuff,
					QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_RSP_MASK,
					statsClearReqMask) ||
			    nla_put_u32(temp_skbuff,
					QCA_WLAN_VENDOR_ATTR_LL_STATS_CLR_CONFIG_STOP_RSP,
					stopReq)) {
				hdd_err("LL_STATS_CLR put fail");
				kfree_skb(temp_skbuff);
				return -EINVAL;
			}

			/* If the ask is to stop the stats collection
			 * as part of clear (stopReq = 1), ensure
			 * that no further requests of get go to the
			 * firmware by having isLinkLayerStatsSet set
			 * to 0.  However it the stopReq as part of
			 * the clear request is 0, the request to get
			 * the statistics are honoured as in this case
			 * the firmware is just asked to clear the
			 * statistics.
			 */
			if (stopReq == 1)
				pAdapter->isLinkLayerStatsSet = 0;

			return cfg80211_vendor_cmd_reply(temp_skbuff);
		}
		EXIT();
		return -ENOMEM;
	}

	return -EINVAL;
}

/**
 * wlan_hdd_cfg80211_ll_stats_clear() - clear ll stats
 * @wiphy: Pointer to wiphy
 * @wdev: Pointer to wdev
 * @data: Pointer to data
 * @data_len: Data length
 *
 * Return: 0 if success, non-zero for failure
 */
int wlan_hdd_cfg80211_ll_stats_clear(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data,
					int data_len)
{
	int ret = 0;

	cds_ssr_protect(__func__);
	ret = __wlan_hdd_cfg80211_ll_stats_clear(wiphy, wdev, data, data_len);
	cds_ssr_unprotect(__func__);

	return ret;
}

/**
 * hdd_populate_per_peer_ps_info() - populate per peer sta's PS info
 * @wifi_peer_info: peer information
 * @vendor_event: buffer for vendor event
 *
 * Return: 0 success
 */
static inline int
hdd_populate_per_peer_ps_info(tSirWifiPeerInfo *wifi_peer_info,
			      struct sk_buff *vendor_event)
{
	if (!wifi_peer_info) {
		hdd_err("Invalid pointer to peer info.");
		return -EINVAL;
	}

	if (nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_STATE,
			wifi_peer_info->power_saving) ||
	    nla_put(vendor_event,
		    QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_MAC_ADDRESS,
		    QDF_MAC_ADDR_SIZE, &wifi_peer_info->peerMacAddress)) {
		hdd_err("QCA_WLAN_VENDOR_ATTR put fail.");
		return -EINVAL;
	}
	return 0;
}

/**
 * hdd_populate_wifi_peer_ps_info() - populate peer sta's power state
 * @data: stats for peer STA
 * @vendor_event: buffer for vendor event
 *
 * Return: 0 success
 */
static int hdd_populate_wifi_peer_ps_info(tSirWifiPeerStat *data,
					  struct sk_buff *vendor_event)
{
	uint32_t peer_num, i;
	tSirWifiPeerInfo *wifi_peer_info;
	struct nlattr *peer_info, *peers;

	if (!data) {
		hdd_err("Invalid pointer to Wifi peer stat.");
		return -EINVAL;
	}

	peer_num = data->numPeers;
	if (peer_num == 0) {
		hdd_err("Peer number is zero.");
		return -EINVAL;
	}

	if (nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_NUM,
			peer_num)) {
		hdd_err("QCA_WLAN_VENDOR_ATTR put fail");
		return -EINVAL;
	}

	peer_info = nla_nest_start(vendor_event,
			       QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_CHG);
	if (peer_info == NULL) {
		hdd_err("nla_nest_start failed");
		return -EINVAL;
	}

	for (i = 0; i < peer_num; i++) {
		wifi_peer_info = &data->peerInfo[i];
		peers = nla_nest_start(vendor_event, i);

		if (peers == NULL) {
			hdd_err("nla_nest_start failed");
			return -EINVAL;
		}

		if (hdd_populate_per_peer_ps_info(wifi_peer_info, vendor_event))
			return -EINVAL;

		nla_nest_end(vendor_event, peers);
	}
	nla_nest_end(vendor_event, peer_info);

	return 0;
}

/**
 * hdd_populate_tx_failure_info() - populate TX failure info
 * @tx_fail: TX failure info
 * @skb: buffer for vendor event
 *
 * Return: 0 Success
 */
static inline int
hdd_populate_tx_failure_info(struct sir_wifi_iface_tx_fail *tx_fail,
			     struct sk_buff *skb)
{
	int status = 0;

	if (tx_fail == NULL || skb == NULL)
		return -EINVAL;

	if (nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TID,
			tx_fail->tid) ||
	    nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_NUM_MSDU,
			tx_fail->msdu_num) ||
	    nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_STATUS,
			tx_fail->status)) {
		hdd_err("QCA_WLAN_VENDOR_ATTR put fail");
		status = -EINVAL;
	}

	return status;
}

/**
 * hdd_populate_wifi_channel_cca_info() - put channel cca info to vendor event
 * @info: cca info array for all channels
 * @vendor_event: vendor event buffer
 *
 * Return: 0 Success, EINVAL failure
 */
static int
hdd_populate_wifi_channel_cca_info(struct sir_wifi_chan_cca_stats *cca,
				   struct sk_buff *vendor_event)
{
	/* There might be no CCA info for a channel */
	if (cca == NULL)
		return 0;

	if (nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IDLE_TIME,
			cca->idle_time) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_TIME,
			cca->tx_time) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IN_BSS_TIME,
			cca->rx_in_bss_time) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_OUT_BSS_TIME,
			cca->rx_out_bss_time) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BUSY,
			cca->rx_busy_time) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BAD,
			cca->rx_in_bad_cond_time) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BAD,
			cca->tx_in_bad_cond_time) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_NO_AVAIL,
			cca->wlan_not_avail_time) ||
	    nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IFACE_ID,
			cca->vdev_id)) {
		hdd_err(FL("QCA_WLAN_VENDOR_ATTR put fail"));
		return -EINVAL;
	}
	return 0;
}

/**
 * hdd_populate_wifi_signal_info - put chain signal info
 * @info: RF chain signal info
 * @skb: vendor event buffer
 *
 * Return: 0 Success, EINVAL failure
 */
static int
hdd_populate_wifi_signal_info(struct sir_wifi_peer_signal_stats *peer_signal,
			      struct sk_buff *skb)
{
	uint32_t i, chain_count;
	struct nlattr *chains, *att;

	/* There might be no signal info for a peer */
	if (peer_signal == NULL)
		return 0;

	chain_count = peer_signal->num_chain < WIFI_MAX_CHAINS ?
		      peer_signal->num_chain : WIFI_MAX_CHAINS;
	if (nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_ANT_NUM,
			chain_count)) {
		hdd_err(FL("QCA_WLAN_VENDOR_ATTR put fail"));
		return -EINVAL;
	}

	att = nla_nest_start(skb,
			     QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_SIGNAL);
	if (att == NULL) {
		hdd_err(FL("nla_nest_start failed"));
		return -EINVAL;
	}

	for (i = 0; i < chain_count; i++) {
		chains = nla_nest_start(skb, i);

		if (chains == NULL) {
			hdd_err(FL("nla_nest_start failed"));
			return -EINVAL;
		}

		hdd_debug(FL("SNR=%d, NF=%d, Rx=%d, Tx=%d"),
			  peer_signal->per_ant_snr[i],
			  peer_signal->nf[i],
			  peer_signal->per_ant_rx_mpdus[i],
			  peer_signal->per_ant_tx_mpdus[i]);
		if (nla_put_u32(skb,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_SNR,
				peer_signal->per_ant_snr[i]) ||
		    nla_put_u32(skb,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_NF,
				peer_signal->nf[i]) ||
		    nla_put_u32(skb,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU,
				peer_signal->per_ant_rx_mpdus[i]) ||
		    nla_put_u32(skb,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MPDU,
				peer_signal->per_ant_tx_mpdus[i])) {
			hdd_err(FL("QCA_WLAN_VENDOR_ATTR put fail"));
			return -EINVAL;
		}
		nla_nest_end(skb, chains);
	}
	nla_nest_end(skb, att);

	return 0;
}

/**
 * hdd_populate_wifi_wmm_ac_tx_info() - put AC TX info
 * @info: tx info
 * @skb: vendor event buffer
 *
 * Return: 0 Success, EINVAL failure
 */
static int
hdd_populate_wifi_wmm_ac_tx_info(struct sir_wifi_tx *tx_stats,
				 struct sk_buff *skb)
{
	uint32_t *agg_size, *succ_mcs, *fail_mcs, *delay;

	/* There might be no TX info for a peer */
	if (tx_stats == NULL)
		return 0;

	agg_size = tx_stats->mpdu_aggr_size;
	succ_mcs = tx_stats->success_mcs;
	fail_mcs = tx_stats->fail_mcs;
	delay = tx_stats->delay;

	if (nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MSDU,
			tx_stats->msdus) ||
	    nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MPDU,
			tx_stats->mpdus) ||
	    nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_PPDU,
			tx_stats->ppdus) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BYTES,
			tx_stats->bytes) ||
	    nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP,
			tx_stats->drops) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP_BYTES,
			tx_stats->drop_bytes) ||
	    nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_RETRY,
			tx_stats->retries) ||
	    nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_NO_ACK,
			tx_stats->failed) ||
	    nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_AGGR_NUM,
			tx_stats->aggr_len) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_SUCC_MCS_NUM,
			tx_stats->success_mcs_len) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_FAIL_MCS_NUM,
			tx_stats->fail_mcs_len) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_DELAY_ARRAY_SIZE,
			tx_stats->delay_len))
		goto put_attr_fail;

	if (agg_size) {
		if (nla_put(skb,
			    QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_AGGR,
			    tx_stats->aggr_len, agg_size))
			goto put_attr_fail;
	}

	if (succ_mcs) {
		if (nla_put(skb,
			    QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_SUCC_MCS,
			    tx_stats->success_mcs_len, succ_mcs))
			goto put_attr_fail;
	}

	if (fail_mcs) {
		if (nla_put(skb,
			    QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_FAIL_MCS,
			    tx_stats->fail_mcs_len, fail_mcs))
			goto put_attr_fail;
	}

	if (delay) {
		if (nla_put(skb,
			    QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DELAY,
			    tx_stats->delay_len, delay))
			goto put_attr_fail;
	}
	return 0;

put_attr_fail:
	hdd_err(FL("QCA_WLAN_VENDOR_ATTR put fail"));
	return -EINVAL;
}

/**
 * hdd_populate_wifi_wmm_ac_rx_info() - put AC RX info
 * @info: rx info
 * @skb: vendor event buffer
 *
 * Return: 0 Success, EINVAL failure
 */
static int
hdd_populate_wifi_wmm_ac_rx_info(struct sir_wifi_rx *rx_stats,
				 struct sk_buff *skb)
{
	uint32_t *mcs, *aggr;

	/* There might be no RX info for a peer */
	if (rx_stats == NULL)
		return 0;

	aggr = rx_stats->mpdu_aggr;
	mcs = rx_stats->mcs;

	if (nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU,
			rx_stats->mpdus) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_BYTES,
			rx_stats->bytes) ||
	    nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU,
			rx_stats->ppdus) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU_BYTES,
			rx_stats->ppdu_bytes) ||
	    nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_LOST,
			rx_stats->mpdu_lost) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_RETRY,
			rx_stats->mpdu_retry) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DUP,
			rx_stats->mpdu_dup) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DISCARD,
			rx_stats->mpdu_discard) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_AGGR_NUM,
			rx_stats->aggr_len) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MCS_NUM,
			rx_stats->mcs_len))
		goto put_attr_fail;

	if (aggr) {
		if (nla_put(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_AGGR,
			    rx_stats->aggr_len, aggr))
			goto put_attr_fail;
	}

	if (mcs) {
		if (nla_put(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MCS,
			    rx_stats->mcs_len, mcs))
			goto put_attr_fail;
	}

	return 0;

put_attr_fail:
	hdd_err(FL("QCA_WLAN_VENDOR_ATTR put fail"));
	return -EINVAL;
}

/**
 * hdd_populate_wifi_wmm_ac_info() - put WMM AC info
 * @info: per AC stats
 * @skb: vendor event buffer
 *
 * Return: 0 Success, EINVAL failure
 */
static int
hdd_populate_wifi_wmm_ac_info(struct sir_wifi_ll_ext_wmm_ac_stats *ac_stats,
			      struct sk_buff *skb)
{
	struct nlattr *wmm;

	wmm = nla_nest_start(skb, ac_stats->type);
	if (wmm == NULL)
		goto nest_start_fail;

	if (hdd_populate_wifi_wmm_ac_tx_info(ac_stats->tx_stats, skb) ||
	    hdd_populate_wifi_wmm_ac_rx_info(ac_stats->rx_stats, skb))
		goto put_attr_fail;

	nla_nest_end(skb, wmm);
	return 0;

nest_start_fail:
	hdd_err(FL("nla_nest_start failed"));
	return -EINVAL;

put_attr_fail:
	hdd_err(FL("QCA_WLAN_VENDOR_ATTR put fail"));
	return -EINVAL;
}

/**
 * hdd_populate_wifi_ll_ext_peer_info() - put per peer info
 * @info: peer stats
 * @skb: vendor event buffer
 *
 * Return: 0 Success, EINVAL failure
 */
static int
hdd_populate_wifi_ll_ext_peer_info(struct sir_wifi_ll_ext_peer_stats *peers,
				   struct sk_buff *skb)
{
	uint32_t i;
	struct nlattr *wmm_ac;

	if (nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_ID,
			peers->peer_id) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IFACE_ID,
			peers->vdev_id) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_TIMES,
			peers->sta_ps_inds) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_DURATION,
			peers->sta_ps_durs) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PROBE_REQ,
			peers->rx_probe_reqs) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MGMT,
			peers->rx_oth_mgmts) ||
	    nla_put(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_MAC_ADDRESS,
		    QDF_MAC_ADDR_SIZE, peers->mac_address) ||
	    hdd_populate_wifi_signal_info(&peers->peer_signal_stats, skb)) {
		hdd_err(FL("put peer signal attr failed"));
		return -EINVAL;
	}

	wmm_ac = nla_nest_start(skb,
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_STATUS);
	if (wmm_ac == NULL) {
		hdd_err(FL("nla_nest_start failed"));
		return -EINVAL;
	}

	for (i = 0; i < WLAN_MAX_AC; i++) {
		if (hdd_populate_wifi_wmm_ac_info(&peers->ac_stats[i], skb)) {
			hdd_err(FL("put WMM AC attr failed"));
			return -EINVAL;
		}
	}

	nla_nest_end(skb, wmm_ac);
	return 0;
}

/**
 * hdd_populate_wifi_ll_ext_stats() - put link layer extension stats
 * @info: link layer stats
 * @skb: vendor event buffer
 *
 * Return: 0 Success, EINVAL failure
 */
static int
hdd_populate_wifi_ll_ext_stats(struct sir_wifi_ll_ext_stats *stats,
			       struct sk_buff *skb)
{
	uint32_t i;
	struct nlattr *peer, *peer_info, *channels, *channel_info;

	if (nla_put_u32(skb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_EVENT_MODE,
			stats->trigger_cond_id) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CCA_BSS_BITMAP,
			stats->cca_chgd_bitmap) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_SIGNAL_BITMAP,
			stats->sig_chgd_bitmap) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BITMAP,
			stats->tx_chgd_bitmap) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BITMAP,
			stats->rx_chgd_bitmap) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CHANNEL_NUM,
			stats->channel_num) ||
	    nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_NUM,
			stats->peer_num)) {
		goto put_attr_fail;
	}

	channels = nla_nest_start(skb,
				  QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CCA_BSS);
	if (channels == NULL) {
		hdd_err(FL("nla_nest_start failed"));
		return -EINVAL;
	}

	for (i = 0; i < stats->channel_num; i++) {
		channel_info = nla_nest_start(skb, i);
		if (channel_info == NULL) {
			hdd_err(FL("nla_nest_start failed"));
			return -EINVAL;
		}

		if (hdd_populate_wifi_channel_cca_info(&stats->cca[i], skb))
			goto put_attr_fail;
		nla_nest_end(skb, channel_info);
	}
	nla_nest_end(skb, channels);

	peer_info = nla_nest_start(skb,
				   QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER);
	if (peer_info == NULL) {
		hdd_err(FL("nla_nest_start failed"));
		return -EINVAL;
	}

	for (i = 0; i < stats->peer_num; i++) {
		peer = nla_nest_start(skb, i);
		if (peer == NULL) {
			hdd_err(FL("nla_nest_start failed"));
			return -EINVAL;
		}

		if (hdd_populate_wifi_ll_ext_peer_info(&stats->peer_stats[i],
						       skb))
			goto put_attr_fail;
		nla_nest_end(skb, peer);
	}

	nla_nest_end(skb, peer_info);
	return 0;

put_attr_fail:
	hdd_err(FL("QCA_WLAN_VENDOR_ATTR put fail"));
	return -EINVAL;
}

/**
 * wlan_hdd_cfg80211_link_layer_stats_ext_callback() - Callback for LL ext
 * @ctx: HDD context
 * @rsp: msg from FW
 *
 * This function is an extension of
 * wlan_hdd_cfg80211_link_layer_stats_callback. It converts
 * monitoring parameters offloaded to NL data and send the same to the
 * kernel/upper layers.
 *
 * Return: None
 */
void wlan_hdd_cfg80211_link_layer_stats_ext_callback(tHddHandle ctx,
						     tSirLLStatsResults *rsp)
{
	hdd_context_t *hdd_ctx;
	struct sk_buff *skb = NULL;
	uint32_t param_id, index;
	hdd_adapter_t *adapter = NULL;
	tSirLLStatsResults *linkLayer_stats_results;
	tSirWifiPeerStat *peer_stats;
	uint8_t *results;
	int status;

	ENTER();

	if (!ctx) {
		hdd_err("Invalid HDD context.");
		return;
	}

	if (!rsp) {
		hdd_err("Invalid result.");
		return;
	}

	hdd_ctx = (hdd_context_t *)ctx;
	linkLayer_stats_results = (tSirLLStatsResults *)rsp;

	status = wlan_hdd_validate_context(hdd_ctx);
	if (0 != status)
		return;

	adapter = hdd_get_adapter_by_vdev(hdd_ctx,
					  linkLayer_stats_results->ifaceId);

	if (NULL == adapter) {
		hdd_err("vdev_id %d does not exist with host.",
			linkLayer_stats_results->ifaceId);
		return;
	}

	index = QCA_NL80211_VENDOR_SUBCMD_LL_STATS_EXT_INDEX;
	skb = cfg80211_vendor_event_alloc(hdd_ctx->wiphy,
			NULL, LL_STATS_EVENT_BUF_SIZE + NLMSG_HDRLEN,
			index, GFP_KERNEL);
	if (!skb) {
		hdd_err("cfg80211_vendor_event_alloc failed.");
		return;
	}

	results = linkLayer_stats_results->results;
	param_id = linkLayer_stats_results->paramId;
	hdd_info("LL_STATS RESP paramID = 0x%x, ifaceId = %u, result = %p",
		 linkLayer_stats_results->paramId,
		 linkLayer_stats_results->ifaceId,
		 linkLayer_stats_results->results);
	if (param_id & WMI_LL_STATS_EXT_PS_CHG) {
		peer_stats = (tSirWifiPeerStat *)results;
		status = hdd_populate_wifi_peer_ps_info(peer_stats, skb);
	} else if (param_id & WMI_LL_STATS_EXT_TX_FAIL) {
		struct sir_wifi_iface_tx_fail *tx_fail;

		tx_fail = (struct sir_wifi_iface_tx_fail *)results;
		status = hdd_populate_tx_failure_info(tx_fail, skb);
	} else if (param_id & WMI_LL_STATS_EXT_MAC_COUNTER) {
		hdd_info("MAC counters stats");
		status = hdd_populate_wifi_ll_ext_stats(
				(struct sir_wifi_ll_ext_stats *)
				rsp->results, skb);
	} else {
		hdd_info("Unknown link layer stats");
		status = -EINVAL;
	}

	if (status == 0)
		cfg80211_vendor_event(skb, GFP_KERNEL);
	else
		kfree_skb(skb);
	EXIT();
}

static const struct nla_policy
qca_wlan_vendor_ll_ext_policy[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_MAX + 1] = {
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_CFG_PERIOD] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_GLOBAL] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_CFG_THRESHOLD] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BITMAP] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BITMAP] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CCA_BSS_BITMAP] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_SIGNAL_BITMAP] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MSDU] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MPDU] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_PPDU] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BYTES] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP_BYTES] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_RETRY] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_NO_ACK] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_NO_BACK] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_AGGR] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_SUCC_MCS] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_FAIL_MCS] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DELAY] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_BYTES] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU_BYTES] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_LOST] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_RETRY] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DUP] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DISCARD] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MCS] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_AGGR] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_TIMES] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_DURATION] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PROBE_REQ] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MGMT] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IDLE_TIME] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_TIME] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BUSY] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BAD] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BAD] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_NO_AVAIL] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IN_BSS_TIME] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_OUT_BSS_TIME] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_SNR] = {
		.type = NLA_U32
	},
	[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_NF] = {
		.type = NLA_U32
	},
};

/**
 * __wlan_hdd_cfg80211_ll_stats_ext_set_param - config monitor parameters
 * @wiphy: wiphy handle
 * @wdev: wdev handle
 * @data: user layer input
 * @data_len: length of user layer input
 *
 * this function is called in ssr protected environment.
 *
 * return: 0 success, none zero for failure
 */
static int __wlan_hdd_cfg80211_ll_stats_ext_set_param(struct wiphy *wiphy,
						      struct wireless_dev *wdev,
						      const void *data,
						      int data_len)
{
	int status;
	uint32_t period;
	struct net_device *dev = wdev->netdev;
	hdd_adapter_t *adapter = WLAN_HDD_GET_PRIV_PTR(dev);
	hdd_context_t *hdd_ctx = wiphy_priv(wiphy);
	struct sir_ll_ext_stats_threshold thresh = {0,};
	struct nlattr *tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_MAX + 1];

	ENTER_DEV(dev);

	if (QDF_GLOBAL_FTM_MODE == hdd_get_conparam()) {
		hdd_warn(FL("command not allowed in ftm mode"));
		return -EPERM;
	}

	status = wlan_hdd_validate_context(hdd_ctx);
	if (0 != status)
		return -EPERM;

	if (nla_parse(tb, QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_MAX,
		      (struct nlattr *)data, data_len,
		      qca_wlan_vendor_ll_ext_policy)) {
		hdd_err(FL("maximum attribute not present"));
		return -EPERM;
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_CFG_PERIOD]) {
		period = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_CFG_PERIOD]);

		if (period != 0 && period < LL_STATS_MIN_PERIOD)
			period = LL_STATS_MIN_PERIOD;

		/*
		 * Only enable/disbale counters.
		 * Keep the last threshold settings.
		 */
		goto set_period;
	}

	/* global thresh is not enabled */
	if (!tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_CFG_THRESHOLD]) {
		thresh.global = false;
		hdd_warn(FL("global thresh is not set"));
	} else {
		thresh.global_threshold = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_CFG_THRESHOLD]);
		thresh.global = true;
		hdd_debug(FL("globle thresh is %d"), thresh.global_threshold);
	}

	if (!tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_GLOBAL]) {
		thresh.global = false;
		hdd_warn(FL("global thresh is not enabled"));
	} else {
		thresh.global = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_GLOBAL]);
		hdd_debug(FL("global is %d"), thresh.global);
	}

	thresh.enable_bitmap = false;
	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BITMAP]) {
		thresh.tx_bitmap = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BITMAP]);
		thresh.enable_bitmap = true;
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BITMAP]) {
		thresh.rx_bitmap = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BITMAP]);
		thresh.enable_bitmap = true;
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CCA_BSS_BITMAP]) {
		thresh.cca_bitmap = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_CCA_BSS_BITMAP]);
		thresh.enable_bitmap = true;
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_SIGNAL_BITMAP]) {
		thresh.signal_bitmap = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_SIGNAL_BITMAP]);
		thresh.enable_bitmap = true;
	}

	if (!thresh.global && !thresh.enable_bitmap) {
		hdd_warn(FL("threshold will be disabled."));
		thresh.enable = false;

		/* Just disable threshold */
		goto set_thresh;
	} else {
		thresh.enable = true;
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MSDU]) {
		thresh.tx.msdu = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MSDU]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MPDU]) {
		thresh.tx.mpdu = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_MPDU]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_PPDU]) {
		thresh.tx.ppdu = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_PPDU]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BYTES]) {
		thresh.tx.bytes = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BYTES]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP]) {
		thresh.tx.msdu_drop = nla_get_u32(
			tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP_BYTES]) {
		thresh.tx.byte_drop = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DROP_BYTES]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_RETRY]) {
		thresh.tx.mpdu_retry = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_RETRY]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_NO_ACK]) {
		thresh.tx.mpdu_fail = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_NO_ACK]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_NO_BACK]) {
		thresh.tx.ppdu_fail = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_NO_BACK]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_AGGR]) {
		thresh.tx.aggregation = nla_get_u32(tb[
				  QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_AGGR]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_SUCC_MCS]) {
		thresh.tx.succ_mcs = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_SUCC_MCS]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_FAIL_MCS]) {
		thresh.tx.fail_mcs = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_FAIL_MCS]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DELAY]) {
		thresh.tx.delay = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_DELAY]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU]) {
		thresh.rx.mpdu = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_BYTES]) {
		thresh.rx.bytes = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_BYTES]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU]) {
		thresh.rx.ppdu = nla_get_u32(tb[
				QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU_BYTES]) {
		thresh.rx.ppdu_bytes = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PPDU_BYTES]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_LOST]) {
		thresh.rx.mpdu_lost = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_LOST]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_RETRY]) {
		thresh.rx.mpdu_retry = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_RETRY]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DUP]) {
		thresh.rx.mpdu_dup = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DUP]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DISCARD]) {
		thresh.rx.mpdu_discard = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MPDU_DISCARD]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_AGGR]) {
		thresh.rx.aggregation = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_AGGR]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MCS]) {
		thresh.rx.mcs = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MCS]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_TIMES]) {
		thresh.rx.ps_inds = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_TIMES]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_DURATION]) {
		thresh.rx.ps_durs = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_PEER_PS_DURATION]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PROBE_REQ]) {
		thresh.rx.probe_reqs = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_PROBE_REQ]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MGMT]) {
		thresh.rx.other_mgmt = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_MGMT]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IDLE_TIME]) {
		thresh.cca.idle_time = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IDLE_TIME]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_TIME]) {
		thresh.cca.tx_time = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_TIME]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IN_BSS_TIME]) {
		thresh.cca.rx_in_bss_time = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_IN_BSS_TIME]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_OUT_BSS_TIME]) {
		thresh.cca.rx_out_bss_time = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_OUT_BSS_TIME]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BUSY]) {
		thresh.cca.rx_busy_time = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BUSY]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BAD]) {
		thresh.cca.rx_in_bad_cond_time = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_RX_BAD]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BAD]) {
		thresh.cca.tx_in_bad_cond_time = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_TX_BAD]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_NO_AVAIL]) {
		thresh.cca.wlan_not_avail_time = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_NO_AVAIL]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_SNR]) {
		thresh.signal.snr = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_SNR]);
	}

	if (tb[QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_NF]) {
		thresh.signal.nf = nla_get_u32(tb[
			QCA_WLAN_VENDOR_ATTR_LL_STATS_EXT_ANT_NF]);
	}

set_thresh:
	hdd_info(FL("send thresh settings to target"));
	if (QDF_STATUS_SUCCESS != sme_ll_stats_set_thresh(hdd_ctx->hHal,
							  &thresh)) {
		hdd_err(FL("sme_ll_stats_set_thresh failed."));
		return -EINVAL;
	}
	return 0;

set_period:
	hdd_info(FL("send period to target"));
	status = wma_cli_set_command(adapter->sessionId,
				     WMI_PDEV_PARAM_STATS_OBSERVATION_PERIOD,
				     period, PDEV_CMD);
	if (status) {
		hdd_err(FL("wma_cli_set_command set_period failed."));
		return -EINVAL;
	}
	return 0;
}

/**
 * wlan_hdd_cfg80211_ll_stats_ext_set_param - config monitor parameters
 * @wiphy: wiphy handle
 * @wdev: wdev handle
 * @data: user layer input
 * @data_len: length of user layer input
 *
 * return: 0 success, einval failure
 */
int wlan_hdd_cfg80211_ll_stats_ext_set_param(struct wiphy *wiphy,
					     struct wireless_dev *wdev,
					     const void *data,
					     int data_len)
{
	int ret;

	cds_ssr_protect(__func__);
	ret = __wlan_hdd_cfg80211_ll_stats_ext_set_param(wiphy, wdev,
							 data, data_len);
	cds_ssr_unprotect(__func__);

	return ret;
}
#endif /* WLAN_FEATURE_LINK_LAYER_STATS */

#ifdef WLAN_FEATURE_STATS_EXT
/**
 * __wlan_hdd_cfg80211_stats_ext_request() - ext stats request
 * @wiphy: Pointer to wiphy
 * @wdev: Pointer to wdev
 * @data: Pointer to data
 * @data_len: Data length
 *
 * Return: int
 */
static int __wlan_hdd_cfg80211_stats_ext_request(struct wiphy *wiphy,
						 struct wireless_dev *wdev,
						 const void *data,
						 int data_len)
{
	tStatsExtRequestReq stats_ext_req;
	struct net_device *dev = wdev->netdev;
	hdd_adapter_t *pAdapter = WLAN_HDD_GET_PRIV_PTR(dev);
	int ret_val;
	QDF_STATUS status;
	hdd_context_t *hdd_ctx = wiphy_priv(wiphy);

	ENTER_DEV(dev);

	ret_val = wlan_hdd_validate_context(hdd_ctx);
	if (ret_val)
		return ret_val;

	if (QDF_GLOBAL_FTM_MODE == hdd_get_conparam()) {
		hdd_warn("Command not allowed in FTM mode");
		return -EPERM;
	}

	stats_ext_req.request_data_len = data_len;
	stats_ext_req.request_data = (void *)data;

	status = sme_stats_ext_request(pAdapter->sessionId, &stats_ext_req);

	if (QDF_STATUS_SUCCESS != status)
		ret_val = -EINVAL;

	return ret_val;
}

/**
 * wlan_hdd_cfg80211_stats_ext_request() - ext stats request
 * @wiphy: Pointer to wiphy
 * @wdev: Pointer to wdev
 * @data: Pointer to data
 * @data_len: Data length
 *
 * Return: int
 */
int wlan_hdd_cfg80211_stats_ext_request(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data,
					int data_len)
{
	int ret;

	cds_ssr_protect(__func__);
	ret = __wlan_hdd_cfg80211_stats_ext_request(wiphy, wdev,
						    data, data_len);
	cds_ssr_unprotect(__func__);

	return ret;
}

/**
 * wlan_hdd_cfg80211_stats_ext_callback() - ext stats callback
 * @ctx: Pointer to HDD context
 * @msg: Message received
 *
 * Return: nothing
 */
void wlan_hdd_cfg80211_stats_ext_callback(void *ctx,
						 tStatsExtEvent *msg)
{

	hdd_context_t *pHddCtx = (hdd_context_t *) ctx;
	struct sk_buff *vendor_event;
	int status;
	int ret_val;
	tStatsExtEvent *data = msg;
	hdd_adapter_t *pAdapter = NULL;

	status = wlan_hdd_validate_context(pHddCtx);
	if (status)
		return;

	pAdapter = hdd_get_adapter_by_vdev(pHddCtx, data->vdev_id);

	if (NULL == pAdapter) {
		hdd_err("vdev_id %d does not exist with host", data->vdev_id);
		return;
	}

	vendor_event = cfg80211_vendor_event_alloc(pHddCtx->wiphy,
						   NULL,
						   data->event_data_len +
						   sizeof(uint32_t) +
						   NLMSG_HDRLEN + NLMSG_HDRLEN,
						   QCA_NL80211_VENDOR_SUBCMD_STATS_EXT_INDEX,
						   GFP_KERNEL);

	if (!vendor_event) {
		hdd_err("cfg80211_vendor_event_alloc failed");
		return;
	}

	ret_val = nla_put_u32(vendor_event, QCA_WLAN_VENDOR_ATTR_IFINDEX,
			      pAdapter->dev->ifindex);
	if (ret_val) {
		hdd_err("QCA_WLAN_VENDOR_ATTR_IFINDEX put fail");
		kfree_skb(vendor_event);

		return;
	}

	ret_val = nla_put(vendor_event, QCA_WLAN_VENDOR_ATTR_STATS_EXT,
			  data->event_data_len, data->event_data);

	if (ret_val) {
		hdd_err("QCA_WLAN_VENDOR_ATTR_STATS_EXT put fail");
		kfree_skb(vendor_event);

		return;
	}

	cfg80211_vendor_event(vendor_event, GFP_KERNEL);

}

void wlan_hdd_cfg80211_stats_ext2_callback(void *ctx,
	struct stats_ext2_event *pmsg)
{
	hdd_context_t *hdd_ctx = (hdd_context_t *)ctx;
	int status, data_size;
	struct sk_buff *vendor_event;

	status = wlan_hdd_validate_context(hdd_ctx);
	if (0 != status)
		return;

	if (NULL == pmsg) {
		hdd_err("msg received here is null");
		return;
	}

	data_size = sizeof(struct stats_ext2_event) +
		(pmsg->hole_cnt)*sizeof(pmsg->hole_info_array[0]);

	vendor_event = cfg80211_vendor_event_alloc(hdd_ctx->wiphy,
			NULL,
			data_size + NLMSG_HDRLEN + NLMSG_HDRLEN,
			QCA_NL80211_VENDOR_SUBCMD_STATS_EXT_INDEX,
			GFP_KERNEL);

	if (!vendor_event) {
		hdd_err("vendor_event_alloc failed for STATS_EXT2");
		return;
	}

	if (nla_put_u32(vendor_event,
			QCA_WLAN_VENDOR_ATTR_RX_AGGREGATION_STATS_HOLES_NUM,
			pmsg->hole_cnt)) {
		hdd_err("%s put fail",
			"QCA_WLAN_VENDOR_ATTR_RX_AGGREGATION_STATS_HOLES_NUM");
		kfree_skb(vendor_event);
		return;
	}

	if (nla_put(vendor_event,
			QCA_WLAN_VENDOR_ATTR_RX_AGGREGATION_STATS_HOLES_INFO,
			(pmsg->hole_cnt)*sizeof(pmsg->hole_info_array[0]),
			(void *)(pmsg->hole_info_array))) {
		hdd_err("%s put fail",
			"QCA_WLAN_VENDOR_ATTR_RX_AGGREGATION_STATS_HOLES_INFO");
		kfree_skb(vendor_event);
		return;
	}

	cfg80211_vendor_event(vendor_event, GFP_KERNEL);
}

#endif /* End of WLAN_FEATURE_STATS_EXT */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)) && !defined(WITH_BACKPORTS)
static inline void wlan_hdd_fill_station_info_signal(struct station_info
						     *sinfo)
{
	sinfo->filled |= STATION_INFO_SIGNAL;
}
#else
static inline void wlan_hdd_fill_station_info_signal(struct station_info
						     *sinfo)
{
	sinfo->filled |= BIT(NL80211_STA_INFO_SIGNAL);
}
#endif

/**
 * __wlan_hdd_cfg80211_get_station() - get station statistics
 * @wiphy: Pointer to wiphy
 * @dev: Pointer to network device
 * @mac: Pointer to mac
 * @sinfo: Pointer to station info
 *
 * Return: 0 for success, non-zero for failure
 */
static int __wlan_hdd_cfg80211_get_station(struct wiphy *wiphy,
					   struct net_device *dev,
					   const uint8_t *mac,
					   struct station_info *sinfo)
{
	hdd_adapter_t *pAdapter = WLAN_HDD_GET_PRIV_PTR(dev);
	hdd_station_ctx_t *pHddStaCtx = WLAN_HDD_GET_STATION_CTX_PTR(pAdapter);
	int ssidlen = pHddStaCtx->conn_info.SSID.SSID.length;
	uint8_t rate_flags;
	uint8_t mcs_index;

	hdd_context_t *pHddCtx = (hdd_context_t *) wiphy_priv(wiphy);
	struct hdd_config *pCfg = pHddCtx->config;

	uint8_t OperationalRates[CSR_DOT11_SUPPORTED_RATES_MAX];
	uint32_t ORLeng = CSR_DOT11_SUPPORTED_RATES_MAX;
	uint8_t ExtendedRates[CSR_DOT11_EXTENDED_SUPPORTED_RATES_MAX];
	uint32_t ERLeng = CSR_DOT11_EXTENDED_SUPPORTED_RATES_MAX;
	uint8_t MCSRates[SIZE_OF_BASIC_MCS_SET];
	uint32_t MCSLeng = SIZE_OF_BASIC_MCS_SET;
	uint16_t maxRate = 0;
	int8_t snr = 0;
	uint16_t myRate;
	uint16_t currentRate = 0;
	uint8_t maxSpeedMCS = 0;
	uint8_t maxMCSIdx = 0;
	uint8_t rateFlag = 1;
	uint8_t i, j, rssidx;
	uint8_t nss = 1;
	int status, mode = 0, maxHtIdx;
	struct index_vht_data_rate_type *supported_vht_mcs_rate;
	struct index_data_rate_type *supported_mcs_rate;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
	bool rssi_stats_valid = false;
#endif

	uint32_t vht_mcs_map;
	enum eDataRate11ACMaxMcs vhtMaxMcs;
	int32_t rcpi_value;

	ENTER_DEV(dev);

	if (QDF_GLOBAL_FTM_MODE == hdd_get_conparam()) {
		hdd_warn("Command not allowed in FTM mode");
		return -EINVAL;
	}

	if (wlan_hdd_validate_session_id(pAdapter->sessionId)) {
		hdd_err("invalid session id: %d", pAdapter->sessionId);
		return -EINVAL;
	}

	if ((eConnectionState_Associated != pHddStaCtx->conn_info.connState) ||
	    (0 == ssidlen)) {
		hdd_debug("Not associated or Invalid ssidlen, %d",
			ssidlen);
		/*To keep GUI happy */
		return 0;
	}

	if (true == pHddStaCtx->hdd_ReassocScenario) {
		hdd_debug("Roaming is in progress, cannot continue with this request");
		/*
		 * supplicant reports very low rssi to upper layer
		 * and handover happens to cellular.
		 * send the cached rssi when get_station
		 */
		sinfo->signal = pAdapter->rssi;
		wlan_hdd_fill_station_info_signal(sinfo);
		return 0;
	}

	status = wlan_hdd_validate_context(pHddCtx);

	if (0 != status)
		return status;

	if (pHddCtx->rcpi_enabled)
		wlan_hdd_get_rcpi(pAdapter, (uint8_t *)mac, &rcpi_value,
				  RCPI_MEASUREMENT_TYPE_AVG_MGMT);

	wlan_hdd_get_station_stats(pAdapter);

	if (pAdapter->hdd_stats.summary_stat.rssi)
		pAdapter->rssi = pAdapter->hdd_stats.summary_stat.rssi;

	/* for new connection there might be no valid previous RSSI */
	if (!pAdapter->rssi) {
		hdd_get_rssi_snr_by_bssid(pAdapter,
				pHddStaCtx->conn_info.bssId.bytes,
				&pAdapter->rssi, NULL);
	}

	sinfo->signal = pAdapter->rssi;
	snr = pAdapter->hdd_stats.summary_stat.snr;
	hdd_debug("snr: %d, rssi: %d",
		pAdapter->hdd_stats.summary_stat.snr,
		pAdapter->hdd_stats.summary_stat.rssi);
	pHddStaCtx->conn_info.signal = sinfo->signal;
	pHddStaCtx->conn_info.noise =
		pHddStaCtx->conn_info.signal - snr;

	wlan_hdd_fill_station_info_signal(sinfo);

	/*
	 * we notify connect to lpass here instead of during actual
	 * connect processing because rssi info is not accurate during
	 * actual connection.  lpass will ensure the notification is
	 * only processed once per association.
	 */
	hdd_lpass_notify_connect(pAdapter);

	rate_flags = pAdapter->hdd_stats.ClassA_stat.tx_rate_flags;
	mcs_index = pAdapter->hdd_stats.ClassA_stat.mcs_index;

	/* convert to the UI units of 100kbps */
	myRate = pAdapter->hdd_stats.ClassA_stat.tx_rate * 5;
	if (!(rate_flags & eHAL_TX_RATE_LEGACY)) {
		nss = pAdapter->hdd_stats.ClassA_stat.nss;
		if (wma_is_current_hwmode_dbs()) {
			hdd_debug("Hw mode is DBS, Reduce nss to 1");
			nss--;
		}

		if (eHDD_LINK_SPEED_REPORT_ACTUAL == pCfg->reportMaxLinkSpeed) {
			/* Get current rate flags if report actual */
			rate_flags =
				pAdapter->hdd_stats.ClassA_stat.mcs_rate_flags;
		}

		if (mcs_index == INVALID_MCS_IDX)
			mcs_index = 0;
	}

	hdd_debug("RSSI %d, RLMS %u, rate %d, rssi high %d, rssi mid %d, rssi low %d, rate_flags 0x%x, MCS %d",
		 sinfo->signal, pCfg->reportMaxLinkSpeed, myRate,
		 (int)pCfg->linkSpeedRssiHigh, (int)pCfg->linkSpeedRssiMid,
		 (int)pCfg->linkSpeedRssiLow, (int)rate_flags, (int)mcs_index);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)) || defined(WITH_BACKPORTS)
	/* assume basic BW. anything else will override this later */
	sinfo->txrate.bw = RATE_INFO_BW_20;
#endif

	if (eHDD_LINK_SPEED_REPORT_ACTUAL != pCfg->reportMaxLinkSpeed) {
		/* we do not want to necessarily report the current speed */
		if (eHDD_LINK_SPEED_REPORT_MAX == pCfg->reportMaxLinkSpeed) {
			/* report the max possible speed */
			rssidx = 0;
		} else if (eHDD_LINK_SPEED_REPORT_MAX_SCALED ==
			   pCfg->reportMaxLinkSpeed) {
			/* report the max possible speed with RSSI scaling */
			if (sinfo->signal >= pCfg->linkSpeedRssiHigh) {
				/* report the max possible speed */
				rssidx = 0;
			} else if (sinfo->signal >= pCfg->linkSpeedRssiMid) {
				/* report middle speed */
				rssidx = 1;
			} else if (sinfo->signal >= pCfg->linkSpeedRssiLow) {
				/* report middle speed */
				rssidx = 2;
			} else {
				/* report actual speed */
				rssidx = 3;
			}
		} else {
			/* unknown, treat as eHDD_LINK_SPEED_REPORT_MAX */
			hdd_err("Invalid value for reportMaxLinkSpeed: %u",
			       pCfg->reportMaxLinkSpeed);
			rssidx = 0;
		}

		maxRate = 0;

		/* Get Basic Rate Set */
		if (0 !=
		    sme_cfg_get_str(WLAN_HDD_GET_HAL_CTX(pAdapter),
				    WNI_CFG_OPERATIONAL_RATE_SET,
				    OperationalRates,
				    &ORLeng)) {
			hdd_err("cfg get returned failure");
			/*To keep GUI happy */
			return 0;
		}

		for (i = 0; i < ORLeng; i++) {
			for (j = 0;
			     j < ARRAY_SIZE(supported_data_rate); j++) {
				/* Validate Rate Set */
				if (supported_data_rate[j].beacon_rate_index ==
				    (OperationalRates[i] & 0x7F)) {
					currentRate =
						supported_data_rate[j].
						supported_rate[rssidx];
					break;
				}
			}
			/* Update MAX rate */
			maxRate =
				(currentRate > maxRate) ? currentRate : maxRate;
		}

		/* Get Extended Rate Set */
		if (0 !=
		    sme_cfg_get_str(WLAN_HDD_GET_HAL_CTX(pAdapter),
				    WNI_CFG_EXTENDED_OPERATIONAL_RATE_SET,
				    ExtendedRates, &ERLeng)) {
			hdd_err("cfg get returned failure");
			/*To keep GUI happy */
			return 0;
		}

		for (i = 0; i < ERLeng; i++) {
			for (j = 0;
			     j < ARRAY_SIZE(supported_data_rate); j++) {
				if (supported_data_rate[j].beacon_rate_index ==
				    (ExtendedRates[i] & 0x7F)) {
					currentRate =
						supported_data_rate[j].
						supported_rate[rssidx];
					break;
				}
			}
			/* Update MAX rate */
			maxRate =
				(currentRate > maxRate) ? currentRate : maxRate;
		}
		/* Get MCS Rate Set --
		 * Only if we are connected in non legacy mode and not reporting
		 * actual speed
		 */
		if ((3 != rssidx) && !(rate_flags & eHAL_TX_RATE_LEGACY)) {
			if (0 !=
			    sme_cfg_get_str(WLAN_HDD_GET_HAL_CTX(pAdapter),
					    WNI_CFG_CURRENT_MCS_SET, MCSRates,
					    &MCSLeng)) {
				hdd_err("cfg get returned failure");
				/*To keep GUI happy */
				return 0;
			}
			rateFlag = 0;
			supported_vht_mcs_rate =
				(struct index_vht_data_rate_type *)
				((nss ==
				  1) ? &supported_vht_mcs_rate_nss1 :
				 &supported_vht_mcs_rate_nss2);

			if (rate_flags & eHAL_TX_RATE_VHT80)
				mode = 2;
			else if ((rate_flags & eHAL_TX_RATE_VHT40) ||
				 (rate_flags & eHAL_TX_RATE_HT40))
				mode = 1;
			else
				mode = 0;

			/* VHT80 rate has seperate rate table */
			if (rate_flags &
			    (eHAL_TX_RATE_VHT20 | eHAL_TX_RATE_VHT40 |
			     eHAL_TX_RATE_VHT80)) {
				sme_cfg_get_int(WLAN_HDD_GET_HAL_CTX(pAdapter),
						WNI_CFG_VHT_TX_MCS_MAP,
						&vht_mcs_map);
				vhtMaxMcs = (enum eDataRate11ACMaxMcs)
					(vht_mcs_map & DATA_RATE_11AC_MCS_MASK);
				if (rate_flags & eHAL_TX_RATE_SGI)
					rateFlag |= 1;
				if (DATA_RATE_11AC_MAX_MCS_7 == vhtMaxMcs)
					maxMCSIdx = 7;
				else if (DATA_RATE_11AC_MAX_MCS_8 ==
					   vhtMaxMcs)
					maxMCSIdx = 8;
				else if (DATA_RATE_11AC_MAX_MCS_9 ==
					   vhtMaxMcs) {
					/*
					 * IEEE_P802.11ac_2013.pdf page 325, 326
					 * - MCS9 is valid for VHT20 when
					 *   Nss = 3 or Nss = 6
					 * - MCS9 is not valid for VHT20 when
					 *   Nss = 1,2,4,5,7,8
					 */
					if ((rate_flags & eHAL_TX_RATE_VHT20) &&
					     (nss != 3 && nss != 6))
						maxMCSIdx = 8;
					else
						maxMCSIdx = 9;
				}

				if (rssidx != 0) {
					for (i = 0; i <= maxMCSIdx; i++) {
						if (sinfo->signal <=
						    rssi_mcs_tbl[mode][i]) {
							maxMCSIdx = i;
							break;
						}
					}
				}

				if (rate_flags & eHAL_TX_RATE_VHT80) {
					currentRate =
					  supported_vht_mcs_rate[mcs_index].
					  supported_VHT80_rate[rateFlag];
					maxRate =
					  supported_vht_mcs_rate[maxMCSIdx].
						supported_VHT80_rate[rateFlag];
				} else if (rate_flags & eHAL_TX_RATE_VHT40) {
					currentRate =
					  supported_vht_mcs_rate[mcs_index].
					  supported_VHT40_rate[rateFlag];
					maxRate =
					  supported_vht_mcs_rate[maxMCSIdx].
						supported_VHT40_rate[rateFlag];
				} else if (rate_flags & eHAL_TX_RATE_VHT20) {
					currentRate =
					  supported_vht_mcs_rate[mcs_index].
					  supported_VHT20_rate[rateFlag];
					maxRate =
					  supported_vht_mcs_rate[maxMCSIdx].
					  supported_VHT20_rate[rateFlag];
				}

				maxSpeedMCS = 1;
				if (currentRate > maxRate)
					maxRate = currentRate;

			} else {
				if (rate_flags & eHAL_TX_RATE_HT40)
					rateFlag |= 1;
				if (rate_flags & eHAL_TX_RATE_SGI)
					rateFlag |= 2;

				supported_mcs_rate =
					(struct index_data_rate_type *)
					((nss ==
					  1) ? &supported_mcs_rate_nss1 :
					 &supported_mcs_rate_nss2);

				maxHtIdx = MAX_HT_MCS_IDX;
				if (rssidx != 0) {
					for (i = 0; i < MAX_HT_MCS_IDX; i++) {
						if (sinfo->signal <=
						    rssi_mcs_tbl[mode][i]) {
							maxHtIdx = i + 1;
							break;
						}
					}
				}

				for (i = 0; i < MCSLeng; i++) {
					for (j = 0; j < maxHtIdx; j++) {
						if (supported_mcs_rate[j].
						    beacon_rate_index ==
						    MCSRates[i]) {
							currentRate =
							  supported_mcs_rate[j].
							  supported_rate
							  [rateFlag];
							maxMCSIdx =
							  supported_mcs_rate[j].
							  beacon_rate_index;
							break;
						}
					}

					if ((j < MAX_HT_MCS_IDX)
					    && (currentRate > maxRate)) {
						maxRate = currentRate;
					}
					maxSpeedMCS = 1;
				}
			}
		}

		else if (!(rate_flags & eHAL_TX_RATE_LEGACY)) {
			maxRate = myRate;
			maxSpeedMCS = 1;
			maxMCSIdx = mcs_index;
		}
		/* report a value at least as big as current rate */
		if ((maxRate < myRate) || (0 == maxRate)) {
			maxRate = myRate;
			if (rate_flags & eHAL_TX_RATE_LEGACY) {
				maxSpeedMCS = 0;
			} else {
				maxSpeedMCS = 1;
				maxMCSIdx = mcs_index;
				/*
				 * IEEE_P802.11ac_2013.pdf page 325, 326
				 * - MCS9 is valid for VHT20 when
				 *   Nss = 3 or Nss = 6
				 * - MCS9 is not valid for VHT20 when
				 *   Nss = 1,2,4,5,7,8
				 */
				if ((rate_flags & eHAL_TX_RATE_VHT20) &&
				    (maxMCSIdx > 8) &&
				    (nss != 3 && nss != 6)) {
					maxMCSIdx = 8;
				}
			}
		}

		if (rate_flags & eHAL_TX_RATE_LEGACY) {
			sinfo->txrate.legacy = maxRate;
#ifdef LINKSPEED_DEBUG_ENABLED
			pr_info("Reporting legacy rate %d\n",
				sinfo->txrate.legacy);
#endif /* LINKSPEED_DEBUG_ENABLED */
		} else {
			sinfo->txrate.mcs = maxMCSIdx;
			sinfo->txrate.nss = nss;
			if (rate_flags & eHAL_TX_RATE_VHT80) {
				sinfo->txrate.flags |= RATE_INFO_FLAGS_VHT_MCS;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)) || defined(WITH_BACKPORTS)
				sinfo->txrate.bw = RATE_INFO_BW_80;
#else
				sinfo->txrate.flags |=
					RATE_INFO_FLAGS_80_MHZ_WIDTH;
#endif
			} else if (rate_flags & eHAL_TX_RATE_VHT40) {
				sinfo->txrate.flags |= RATE_INFO_FLAGS_VHT_MCS;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)) || defined(WITH_BACKPORTS)
				sinfo->txrate.bw = RATE_INFO_BW_40;
#else
				sinfo->txrate.flags |=
					RATE_INFO_FLAGS_40_MHZ_WIDTH;
#endif
			} else if (rate_flags & eHAL_TX_RATE_VHT20) {
				sinfo->txrate.flags |= RATE_INFO_FLAGS_VHT_MCS;
			} else
				sinfo->txrate.flags |= RATE_INFO_FLAGS_VHT_MCS;
			if (rate_flags &
			    (eHAL_TX_RATE_HT20 | eHAL_TX_RATE_HT40)) {
				sinfo->txrate.flags |= RATE_INFO_FLAGS_MCS;
				if (rate_flags & eHAL_TX_RATE_HT40) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)) || defined(WITH_BACKPORTS)
					sinfo->txrate.bw = RATE_INFO_BW_40;
#else
					sinfo->txrate.flags |=
						RATE_INFO_FLAGS_40_MHZ_WIDTH;
#endif
				}
			}
			if (rate_flags & eHAL_TX_RATE_SGI) {
				if (!
				    (sinfo->txrate.
				     flags & RATE_INFO_FLAGS_VHT_MCS))
					sinfo->txrate.flags |=
						RATE_INFO_FLAGS_MCS;
				sinfo->txrate.flags |= RATE_INFO_FLAGS_SHORT_GI;
			}
#ifdef LINKSPEED_DEBUG_ENABLED
			pr_info("Reporting MCS rate %d flags %x\n",
				sinfo->txrate.mcs, sinfo->txrate.flags);
#endif /* LINKSPEED_DEBUG_ENABLED */
		}
	} else {
		/* report current rate instead of max rate */

		if (rate_flags & eHAL_TX_RATE_LEGACY) {
			/* provide to the UI in units of 100kbps */
			sinfo->txrate.legacy = myRate;
#ifdef LINKSPEED_DEBUG_ENABLED
			pr_info("Reporting actual legacy rate %d\n",
				sinfo->txrate.legacy);
#endif /* LINKSPEED_DEBUG_ENABLED */
		} else {
			/* must be MCS */
			sinfo->txrate.mcs = mcs_index;
			sinfo->txrate.nss = nss;
			sinfo->txrate.flags |= RATE_INFO_FLAGS_VHT_MCS;
			if (rate_flags & eHAL_TX_RATE_VHT80) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)) || defined(WITH_BACKPORTS)
				sinfo->txrate.bw = RATE_INFO_BW_80;
#else
				sinfo->txrate.flags |=
					RATE_INFO_FLAGS_80_MHZ_WIDTH;
#endif
			} else if (rate_flags & eHAL_TX_RATE_VHT40) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)) || defined(WITH_BACKPORTS)
				sinfo->txrate.bw = RATE_INFO_BW_40;
#else
				sinfo->txrate.flags |=
					RATE_INFO_FLAGS_40_MHZ_WIDTH;
#endif
			}
			if (rate_flags &
			    (eHAL_TX_RATE_HT20 | eHAL_TX_RATE_HT40)) {
				sinfo->txrate.flags |= RATE_INFO_FLAGS_MCS;
				if (rate_flags & eHAL_TX_RATE_HT40) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)) || defined(WITH_BACKPORTS)
					sinfo->txrate.bw = RATE_INFO_BW_40;
#else
					sinfo->txrate.flags |=
						RATE_INFO_FLAGS_40_MHZ_WIDTH;
#endif
				}
			}
			if (rate_flags & eHAL_TX_RATE_SGI) {
				sinfo->txrate.flags |= RATE_INFO_FLAGS_MCS;
				sinfo->txrate.flags |= RATE_INFO_FLAGS_SHORT_GI;
			}
#ifdef LINKSPEED_DEBUG_ENABLED
			pr_info("Reporting actual MCS rate %d flags %x\n",
				sinfo->txrate.mcs, sinfo->txrate.flags);
#endif /* LINKSPEED_DEBUG_ENABLED */
		}
	}

	sinfo->tx_bytes = pAdapter->stats.tx_bytes;

	sinfo->tx_packets =
		pAdapter->hdd_stats.summary_stat.tx_frm_cnt[0] +
		pAdapter->hdd_stats.summary_stat.tx_frm_cnt[1] +
		pAdapter->hdd_stats.summary_stat.tx_frm_cnt[2] +
		pAdapter->hdd_stats.summary_stat.tx_frm_cnt[3];

	sinfo->tx_retries =
		pAdapter->hdd_stats.summary_stat.multiple_retry_cnt[0] +
		pAdapter->hdd_stats.summary_stat.multiple_retry_cnt[1] +
		pAdapter->hdd_stats.summary_stat.multiple_retry_cnt[2] +
		pAdapter->hdd_stats.summary_stat.multiple_retry_cnt[3];

	sinfo->tx_failed =
		pAdapter->hdd_stats.summary_stat.fail_cnt[0] +
		pAdapter->hdd_stats.summary_stat.fail_cnt[1] +
		pAdapter->hdd_stats.summary_stat.fail_cnt[2] +
		pAdapter->hdd_stats.summary_stat.fail_cnt[3];

	sinfo->rx_bytes = pAdapter->stats.rx_bytes;
	sinfo->rx_packets = pAdapter->stats.rx_packets;

	qdf_mem_copy(&pHddStaCtx->conn_info.txrate,
		     &sinfo->txrate, sizeof(sinfo->txrate));

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)) && !defined(WITH_BACKPORTS)
	sinfo->filled |= STATION_INFO_TX_BITRATE |
			 STATION_INFO_TX_BYTES   |
			 STATION_INFO_TX_PACKETS |
			 STATION_INFO_TX_RETRIES |
			 STATION_INFO_TX_FAILED  |
			 STATION_INFO_RX_BYTES   |
			 STATION_INFO_RX_PACKETS;
#else
	sinfo->filled |= BIT(NL80211_STA_INFO_TX_BYTES)   |
			 BIT(NL80211_STA_INFO_TX_BITRATE) |
			 BIT(NL80211_STA_INFO_TX_PACKETS) |
			 BIT(NL80211_STA_INFO_TX_RETRIES) |
			 BIT(NL80211_STA_INFO_TX_FAILED)  |
			 BIT(NL80211_STA_INFO_RX_BYTES)   |
			 BIT(NL80211_STA_INFO_RX_PACKETS);
#endif

	if (rate_flags & eHAL_TX_RATE_LEGACY)
		hdd_debug("Reporting legacy rate %d pkt cnt tx %d rx %d",
			sinfo->txrate.legacy, sinfo->tx_packets,
			sinfo->rx_packets);
	else
		hdd_debug("Reporting MCS rate %d flags 0x%x pkt cnt tx %d rx %d",
			sinfo->txrate.mcs, sinfo->txrate.flags,
			sinfo->tx_packets, sinfo->rx_packets);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
	sinfo->signal_avg = WLAN_HDD_TGT_NOISE_FLOOR_DBM;
	for (i = 0; i < NUM_CHAINS_MAX; i++) {
		sinfo->chain_signal_avg[i] =
			   pAdapter->hdd_stats.per_chain_rssi_stats.rssi[i];
		sinfo->chains |= 1 << i;
		if (sinfo->chain_signal_avg[i] > sinfo->signal_avg &&
				   sinfo->chain_signal_avg[i] != 0)
			sinfo->signal_avg = sinfo->chain_signal_avg[i];

		hdd_debug("RSSI for chain %d, vdev_id %d is %d",
			i, pAdapter->sessionId, sinfo->chain_signal_avg[i]);

		if (!rssi_stats_valid && sinfo->chain_signal_avg[i])
			rssi_stats_valid = true;
	}

	if (rssi_stats_valid) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)) && !defined(WITH_BACKPORTS)
		sinfo->filled |= STATION_INFO_CHAIN_SIGNAL_AVG;
		sinfo->filled |= STATION_INFO_SIGNAL_AVG;
#else
		sinfo->filled |= BIT(NL80211_STA_INFO_CHAIN_SIGNAL_AVG);
		sinfo->filled |= BIT(NL80211_STA_INFO_SIGNAL_AVG);
#endif
	}
#endif


	MTRACE(qdf_trace(QDF_MODULE_ID_HDD,
			 TRACE_CODE_HDD_CFG80211_GET_STA,
			 pAdapter->sessionId, maxRate));
	EXIT();
	return 0;
}

/**
 * wlan_hdd_cfg80211_get_station() - get station statistics
 * @wiphy: Pointer to wiphy
 * @dev: Pointer to network device
 * @mac: Pointer to mac
 * @sinfo: Pointer to station info
 *
 * Return: 0 for success, non-zero for failure
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0))
int wlan_hdd_cfg80211_get_station(struct wiphy *wiphy,
				  struct net_device *dev, const uint8_t *mac,
				  struct station_info *sinfo)
#else
int wlan_hdd_cfg80211_get_station(struct wiphy *wiphy,
				  struct net_device *dev, uint8_t *mac,
				  struct station_info *sinfo)
#endif
{
	int ret;

	cds_ssr_protect(__func__);
	ret = __wlan_hdd_cfg80211_get_station(wiphy, dev, mac, sinfo);
	cds_ssr_unprotect(__func__);

	return ret;
}

/**
 * __wlan_hdd_cfg80211_dump_station() - dump station statistics
 * @wiphy: Pointer to wiphy
 * @dev: Pointer to network device
 * @idx: variable to determine whether to get stats or not
 * @mac: Pointer to mac
 * @sinfo: Pointer to station info
 *
 * Return: 0 for success, non-zero for failure
 */
static int __wlan_hdd_cfg80211_dump_station(struct wiphy *wiphy,
				struct net_device *dev,
				int idx, u8 *mac,
				struct station_info *sinfo)
{
	hdd_context_t *hdd_ctx = (hdd_context_t *) wiphy_priv(wiphy);

	hdd_debug("%s: idx %d", __func__, idx);
	if (idx != 0)
		return -ENOENT;
	qdf_mem_copy(mac, hdd_ctx->config->intfMacAddr[0].bytes,
				QDF_MAC_ADDR_SIZE);
	return __wlan_hdd_cfg80211_get_station(wiphy, dev, mac, sinfo);
}

/**
 * wlan_hdd_cfg80211_dump_station() - dump station statistics
 * @wiphy: Pointer to wiphy
 * @dev: Pointer to network device
 * @idx: variable to determine whether to get stats or not
 * @mac: Pointer to mac
 * @sinfo: Pointer to station info
 *
 * Return: 0 for success, non-zero for failure
 */
int wlan_hdd_cfg80211_dump_station(struct wiphy *wiphy,
				struct net_device *dev,
				int idx, u8 *mac,
				struct station_info *sinfo)
{
	int ret;

	cds_ssr_protect(__func__);
	ret = __wlan_hdd_cfg80211_dump_station(wiphy, dev, idx, mac, sinfo);
	cds_ssr_unprotect(__func__);
	return ret;
}

/**
 * hdd_get_stats() - Function to retrieve interface statistics
 * @dev: pointer to network device
 *
 * This function is the ndo_get_stats method for all netdevs
 * registered with the kernel
 *
 * Return: pointer to net_device_stats structure
 */
struct net_device_stats *hdd_get_stats(struct net_device *dev)
{
	hdd_adapter_t *adapter = WLAN_HDD_GET_PRIV_PTR(dev);

	ENTER_DEV(dev);
	return &adapter->stats;
}
/**
 * __wlan_hdd_cfg80211_dump_survey() - get survey related info
 * @wiphy: Pointer to wiphy
 * @dev: Pointer to network device
 * @idx: Index
 * @survey: Pointer to survey info
 *
 * Return: 0 for success, non-zero for failure
 */
static int __wlan_hdd_cfg80211_dump_survey(struct wiphy *wiphy,
					   struct net_device *dev,
					   int idx, struct survey_info *survey)
{
	hdd_adapter_t *pAdapter = WLAN_HDD_GET_PRIV_PTR(dev);
	hdd_context_t *pHddCtx;
	hdd_station_ctx_t *pHddStaCtx;
	tHalHandle halHandle;
	uint32_t channel = 0, freq = 0, opfreq; /* Initialization Required */
	int status, i, j = 0;
	bool filled = false;

	ENTER_DEV(dev);

	hdd_debug("dump survey index: %d", idx);
	if (idx > QDF_MAX_NUM_CHAN - 1)
		return -EINVAL;

	pHddCtx = WLAN_HDD_GET_CTX(pAdapter);
	status = wlan_hdd_validate_context(pHddCtx);
	if (0 != status)
		return status;

	if (0 == pHddCtx->config->fEnableSNRMonitoring) {
		hdd_debug("gEnableSNRMonitoring is 0");
		return -ENONET;
	}

	if (NULL == pHddCtx->chan_info) {
		hdd_err("chan_info is NULL");
		return -EINVAL;

	}

	if (QDF_GLOBAL_FTM_MODE == hdd_get_conparam()) {
		hdd_err("Command not allowed in FTM mode");
		return -EINVAL;
	}

	pHddStaCtx = WLAN_HDD_GET_STATION_CTX_PTR(pAdapter);

	if (pHddStaCtx->hdd_ReassocScenario) {
		hdd_debug("Roaming in progress, hence return");
		return -ENONET;
	}

	halHandle = WLAN_HDD_GET_HAL_CTX(pAdapter);

	sme_get_operation_channel(halHandle, &channel, pAdapter->sessionId);
	hdd_wlan_get_freq(channel, &opfreq);

	mutex_lock(&pHddCtx->chan_info_lock);
	freq = pHddCtx->chan_info[idx].freq;

	for (i = 0; i < HDD_NUM_NL80211_BANDS && !filled; i++) {
		struct ieee80211_supported_band *band = wiphy->bands[i];
		if (NULL == wiphy->bands[i])
			continue;
		for (j = 0; j < wiphy->bands[i]->n_channels && !filled; j++) {
			if (band->channels[j].center_freq != (uint16_t)freq)
				continue;

			survey->channel = &band->channels[j];
			survey->noise =
				pHddCtx->chan_info[idx].noise_floor;
			survey->filled = SURVEY_INFO_NOISE_DBM;

			if (opfreq == freq)
				survey->filled |= SURVEY_INFO_IN_USE;

			filled = true;

			if (pHddCtx->chan_info[idx].clock_freq == 0)
				continue;

			/*
			 * time = cycle_count * cycle
			 * cycle = 1 / clock_freq
			 * Since the unit of clock_freq reported from
			 * FW is MHZ, and we want to calculate time in
			 * ms level, the result is
			 * time = cycle / (clock_freq * 1000)
			 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
			survey->time =
				pHddCtx->chan_info[idx].delta_cycle_count /
				(pHddCtx->chan_info[idx].clock_freq * 1000);
			survey->time_busy =
				pHddCtx->chan_info[idx].delta_rx_clear_count /
				(pHddCtx->chan_info[idx].clock_freq * 1000);
			survey->time_tx =
				pHddCtx->chan_info[idx].delta_tx_frame_count /
				(pHddCtx->chan_info[idx].clock_freq * 1000);

			survey->filled |= SURVEY_INFO_TIME |
					  SURVEY_INFO_TIME_BUSY |
					  SURVEY_INFO_TIME_TX;
#else
			survey->channel_time =
				pHddCtx->chan_info[idx].delta_cycle_count /
				(pHddCtx->chan_info[idx].clock_freq * 1000);
			survey->channel_time_busy =
				pHddCtx->chan_info[idx].delta_rx_clear_count /
				(pHddCtx->chan_info[idx].clock_freq * 1000);
			survey->channel_time_tx =
				pHddCtx->chan_info[idx].delta_tx_frame_count /
				(pHddCtx->chan_info[idx].clock_freq * 1000);

			survey->filled |= SURVEY_INFO_CHANNEL_TIME |
					  SURVEY_INFO_CHANNEL_TIME_BUSY |
					  SURVEY_INFO_CHANNEL_TIME_TX;
#endif
		}
	}
	mutex_unlock(&pHddCtx->chan_info_lock);

	if (!filled)
		return -ENONET;
	EXIT();
	return 0;
}

/**
 * wlan_hdd_cfg80211_dump_survey() - get survey related info
 * @wiphy: Pointer to wiphy
 * @dev: Pointer to network device
 * @idx: Index
 * @survey: Pointer to survey info
 *
 * Return: 0 for success, non-zero for failure
 */
int wlan_hdd_cfg80211_dump_survey(struct wiphy *wiphy,
				  struct net_device *dev,
				  int idx, struct survey_info *survey)
{
	int ret;

	cds_ssr_protect(__func__);
	ret = __wlan_hdd_cfg80211_dump_survey(wiphy, dev, idx, survey);
	cds_ssr_unprotect(__func__);

	return ret;
}
/**
 * hdd_init_ll_stats_ctx() - initialize link layer stats context
 *
 * Return: none
 */
inline void hdd_init_ll_stats_ctx(void)
{
	spin_lock_init(&ll_stats_context.context_lock);
	init_completion(&ll_stats_context.response_event);
	ll_stats_context.request_bitmap = 0;
}

/**
 * hdd_display_hif_stats() - display hif stats
 *
 * Return: none
 *
 */
void hdd_display_hif_stats(void)
{
	void *hif_ctx = cds_get_context(QDF_MODULE_ID_HIF);

	if (!hif_ctx)
		return;

	hif_display_stats(hif_ctx);
}

/**
 * hdd_clear_hif_stats() - clear hif stats
 *
 * Return: none
 */
void hdd_clear_hif_stats(void)
{
	void *hif_ctx = cds_get_context(QDF_MODULE_ID_HIF);

	if (!hif_ctx)
		return;
	hif_clear_stats(hif_ctx);
}

/**
 * hdd_is_rcpi_applicable() - validates RCPI request
 * @adapter: adapter upon which the measurement is requested
 * @mac_addr: peer addr for which measurement is requested
 * @rcpi_value: pointer to where the RCPI should be returned
 * @reassoc: used to return cached RCPI during reassoc
 *
 * Return: true for success, false for failure
 */

static bool hdd_is_rcpi_applicable(hdd_adapter_t *adapter,
				   struct qdf_mac_addr *mac_addr,
				   int32_t *rcpi_value,
				   bool *reassoc)
{
	hdd_station_ctx_t *hdd_sta_ctx;

	if (adapter->device_mode == QDF_STA_MODE ||
	    adapter->device_mode == QDF_P2P_CLIENT_MODE) {
		hdd_sta_ctx = WLAN_HDD_GET_STATION_CTX_PTR(adapter);
		if (hdd_sta_ctx->conn_info.connState !=
		    eConnectionState_Associated)
			return false;

		if (hdd_sta_ctx->hdd_ReassocScenario) {
			/* return the cached rcpi, if mac addr matches */
			hdd_debug("Roaming in progress, return cached RCPI");
			if (!qdf_mem_cmp(&adapter->rcpi.mac_addr,
			    mac_addr, sizeof(*mac_addr))) {
				*rcpi_value = adapter->rcpi.rcpi;
				*reassoc = true;
				return true;
			}
			return false;
		}

		if (qdf_mem_cmp(mac_addr, &hdd_sta_ctx->conn_info.bssId,
		    sizeof(*mac_addr))) {
			hdd_err("mac addr is different from bssid connected");
			return false;
		}
	} else if (adapter->device_mode == QDF_SAP_MODE ||
		   adapter->device_mode == QDF_P2P_GO_MODE) {
		if (!test_bit(SOFTAP_BSS_STARTED, &adapter->event_flags)) {
			hdd_err("Invalid rcpi request, softap not started");
			return false;
		}

		/* check if peer mac addr is associated to softap */
		if (!hdd_is_peer_associated(adapter, mac_addr)) {
			hdd_err("invalid peer mac-addr: not associated");
			return false;
		}
	} else {
		hdd_err("Invalid rcpi request");
		return false;
	}

	*reassoc = false;
	return true;
}

/**
 * wlan_hdd_get_rcpi_cb() - callback function for rcpi response
 * @context: Pointer to rcpi context
 * @rcpi_req: Pointer to rcpi response
 *
 * Return: None
 */
static void wlan_hdd_get_rcpi_cb(void *context, struct qdf_mac_addr mac_addr,
				 int32_t rcpi, QDF_STATUS status)
{
	hdd_adapter_t *adapter;
	struct statsContext *rcpi_context;

	if (!context) {
		hdd_err("No rcpi context");
		return;
	}

	rcpi_context = context;
	adapter = rcpi_context->pAdapter;
	if (adapter->magic != WLAN_HDD_ADAPTER_MAGIC) {
		hdd_err("Invalid adapter magic");
		return;
	}

	/*
	 * there is a race condition that exists between this callback
	 * function and the caller since the caller could time out
	 * either before or while this code is executing.  we use a
	 * spinlock to serialize these actions
	 */
	spin_lock(&hdd_context_lock);
	if (rcpi_context->magic != RCPI_CONTEXT_MAGIC) {
		/*
		 * the caller presumably timed out so there is nothing
		 * we can do
		 */
		spin_unlock(&hdd_context_lock);
		hdd_warn("Invalid RCPI context magic");
		return;
	}

	rcpi_context->magic = 0;
	adapter->rcpi.mac_addr = mac_addr;
	if (status != QDF_STATUS_SUCCESS)
		/* peer rcpi is not available for requested mac addr */
		adapter->rcpi.rcpi = 0;
	else
		adapter->rcpi.rcpi = rcpi;

	/* notify the caller */
	complete(&rcpi_context->completion);

	/* serialization is complete */
	spin_unlock(&hdd_context_lock);
}

/**
 * __wlan_hdd_get_rcpi() - local function to get RCPI
 * @adapter: adapter upon which the measurement is requested
 * @mac: peer addr for which measurement is requested
 * @rcpi_value: pointer to where the RCPI should be returned
 * @measurement_type: type of rcpi measurement
 *
 * Return: 0 for success, non-zero for failure
 */
static int __wlan_hdd_get_rcpi(hdd_adapter_t *adapter,
			       uint8_t *mac,
			       int32_t *rcpi_value,
			       enum rcpi_measurement_type measurement_type)
{
	hdd_context_t *hdd_ctx;
	static struct statsContext rcpi_context;
	int status = 0;
	unsigned long rc;
	struct qdf_mac_addr mac_addr;
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	struct sme_rcpi_req *rcpi_req;
	bool reassoc;

	ENTER();

	/* initialize the rcpi value to zero, useful in error cases */
	*rcpi_value = 0;

	if (hdd_get_conparam() == QDF_GLOBAL_FTM_MODE) {
		hdd_err("Command not allowed in FTM mode");
		return -EINVAL;
	}

	if (!adapter) {
		hdd_err("adapter context is NULL");
		return -EINVAL;
	}

	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	status = wlan_hdd_validate_context(hdd_ctx);
	if (status)
		return -EINVAL;

	if (!hdd_ctx->rcpi_enabled) {
		hdd_warn("RCPI not supported");
		return -EINVAL;
	}

	if (!mac) {
		hdd_err("RCPI peer mac-addr is NULL");
		return -EINVAL;
	}

	qdf_mem_copy(&mac_addr, mac, QDF_MAC_ADDR_SIZE);

	if (!hdd_is_rcpi_applicable(adapter, &mac_addr, rcpi_value, &reassoc))
		return -EINVAL;
	if (reassoc)
		return 0;

	rcpi_req = qdf_mem_malloc(sizeof(*rcpi_req));
	if (!rcpi_req) {
		hdd_err("unable to allocate memory for RCPI req");
		return -EINVAL;
	}

	init_completion(&rcpi_context.completion);
	rcpi_context.pAdapter = adapter;
	rcpi_context.magic = RCPI_CONTEXT_MAGIC;

	rcpi_req->mac_addr = mac_addr;
	rcpi_req->session_id = adapter->sessionId;
	rcpi_req->measurement_type = measurement_type;
	rcpi_req->rcpi_callback = wlan_hdd_get_rcpi_cb;
	rcpi_req->rcpi_context = &rcpi_context;

	qdf_status = sme_get_rcpi(hdd_ctx->hHal, rcpi_req);
	if (qdf_status != QDF_STATUS_SUCCESS) {
		hdd_err("Unable to retrieve RCPI");
		status = qdf_status_to_os_return(qdf_status);
	} else {
		/* request was sent -- wait for the response */
		rc = wait_for_completion_timeout(&rcpi_context.completion,
					msecs_to_jiffies(WLAN_WAIT_TIME_RCPI));
		if (!rc) {
			hdd_err("SME timed out while retrieving RCPI");
			status = -EINVAL;
		}
	}
	qdf_mem_free(rcpi_req);

	spin_lock(&hdd_context_lock);
	rcpi_context.magic = 0;
	spin_unlock(&hdd_context_lock);

	if (status) {
		hdd_err("rcpi computation is failed");
	} else {
		if (qdf_mem_cmp(&mac_addr, &adapter->rcpi.mac_addr,
		    sizeof(mac_addr))) {
			hdd_err("mac addr is not matching from call-back");
			status = -EINVAL;
		} else {
			*rcpi_value = adapter->rcpi.rcpi;
			hdd_debug("RCPI = %d", *rcpi_value);
		}
	}

	EXIT();
	return status;
}

int wlan_hdd_get_rcpi(hdd_adapter_t *adapter, uint8_t *mac,
		      int32_t *rcpi_value,
		      enum rcpi_measurement_type measurement_type)
{
	int ret;

	cds_ssr_protect(__func__);
	ret = __wlan_hdd_get_rcpi(adapter, mac, rcpi_value, measurement_type);
	cds_ssr_unprotect(__func__);

	return ret;
}
