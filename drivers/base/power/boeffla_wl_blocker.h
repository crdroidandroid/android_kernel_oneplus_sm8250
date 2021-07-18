/*
 * Author: andip71, 01.09.2017
 *
 * Version 1.1.0
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define BOEFFLA_WL_BLOCKER_VERSION	"1.1.0"

#define LIST_WL_DEFAULT				"qcom_rx_wakelock;998000.qcom,qup_uart;bluetooth_timer;hal_bluetooth_lock;smp2p-sleepstate;wlan;wlan_pno_wl;fastcg_wake_lock;c440000.qcom,spmi:qcom,pm8150b@2:qcom,qpnp-smb5;bq_delt_soc_wake_lock;alarmtimer"

#define LENGTH_LIST_WL				255
#define LENGTH_LIST_WL_DEFAULT		255
#define LENGTH_LIST_WL_SEARCH		LENGTH_LIST_WL + LENGTH_LIST_WL_DEFAULT + 5
