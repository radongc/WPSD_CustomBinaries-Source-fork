/*
*   Copyright (C) 2026 WPSD
*
*   APX-to-XG-100P SA Location Bridge
*
*   Bridges GPS location data from Motorola APX voice Link Control words
*   to L3Harris XG-100P Situational Awareness format (SAP 32 / SNDCP
*   Location data PDUs). The XG-100P's SA monitor only processes SAP 32
*   data PDUs; it ignores GPS embedded in voice LC. This bridge extracts
*   GPS from APX voice transmissions and re-transmits it as a SAP 32 PDU
*   that the XG-100P can display.
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include "P25SABridge.h"
#include "P25Defines.h"
#include "P25Trellis.h"
#include "P25Utils.h"
#include "P25NID.h"
#include "Defines.h"
#include "Sync.h"
#include "CRC.h"
#include "Utils.h"
#include "Log.h"

#include <cstdint>
#include <cstring>
#include <cmath>

const unsigned int P25_SAP_SNDCP = 32U;

CP25SABridge::CP25SABridge(unsigned int delayMs) :
m_delayMs(delayMs),
m_gpsValid(false),
m_gpsSrcId(0U),
m_gpsLatitude(0.0),
m_gpsLongitude(0.0),
m_pendingTransmit(false),
m_delayTimer(1000U, 0U, delayMs)
{
}

CP25SABridge::~CP25SABridge()
{
}

void CP25SABridge::processVoiceLC(const unsigned char* rs, unsigned int srcId)
{
	unsigned char lcf  = rs[0U];
	unsigned char mfId = rs[1U];

	if (lcf == P25_LCF_GROUP || lcf == P25_LCF_PRIVATE)
		return;

	LogMessage("P25 SA Bridge, non-standard voice LC from RID %u: LCF=$%02X MFId=$%02X"
		" data=%02X %02X %02X %02X %02X %02X %02X",
		srcId, lcf, mfId,
		rs[2U], rs[3U], rs[4U], rs[5U], rs[6U], rs[7U], rs[8U]);

	// LC word layout (9 bytes after RS decode):
	//   rs[0] = LCF, rs[1] = MFId, rs[2..8] = 7 bytes payload
	//
	// GPS encoding per TIA-102.BAHA: signed integers scaled by 360.0 / 2^25.
	// With 24-bit fields in the 7-byte payload:
	//   rs[2..4] = latitude  (24-bit signed, MSB-first)
	//   rs[5..7] = longitude (24-bit signed, MSB-first)
	//   rs[8]    = flags / accuracy

	int32_t rawLat = ((int32_t)(int8_t)rs[2U] << 16) |
	                 ((int32_t)rs[3U] << 8) |
	                  (int32_t)rs[4U];

	int32_t rawLon = ((int32_t)(int8_t)rs[5U] << 16) |
	                 ((int32_t)rs[6U] << 8) |
	                  (int32_t)rs[7U];

	double lat = (double)rawLat * (360.0 / 33554432.0);   // 2^25
	double lon = (double)rawLon * (360.0 / 33554432.0);

	if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
		m_gpsValid   = true;
		m_gpsSrcId   = srcId;
		m_gpsLatitude  = lat;
		m_gpsLongitude = lon;

		LogMessage("P25 SA Bridge, GPS from RID %u: lat=%.6f lon=%.6f (raw lat=%d lon=%d)",
			srcId, lat, lon, rawLat, rawLon);
	} else {
		LogMessage("P25 SA Bridge, GPS sanity check failed: lat=%.2f lon=%.2f "
			"(raw %d, %d) - format may differ, check raw LC bytes above",
			lat, lon, rawLat, rawLon);
	}
}

void CP25SABridge::logSAP32PDU(unsigned int llId, const unsigned char* header, unsigned int headerLen)
{
	LogMessage("P25 SA Bridge, captured SAP 32 PDU header from RID %u:", llId);
	CUtils::dump(1U, "P25 SA Bridge, PDU Header bytes", header, headerLen);
}

void CP25SABridge::logSAP32DataBlock(const unsigned char* dataBlock, unsigned int blockLen, unsigned int blockIndex)
{
	LogMessage("P25 SA Bridge, captured SAP 32 data block %u (%u bytes):", blockIndex, blockLen);
	CUtils::dump(1U, "P25 SA Bridge, PDU Data Block bytes", dataBlock, blockLen);
}

void CP25SABridge::onVoiceEnd()
{
	if (m_gpsValid) {
		LogMessage("P25 SA Bridge, voice ended from RID %u with GPS lat=%.6f lon=%.6f, "
			"scheduling SAP 32 PDU in %ums",
			m_gpsSrcId, m_gpsLatitude, m_gpsLongitude, m_delayMs);
		m_pendingTransmit = true;
		m_delayTimer.start();
	}
}

void CP25SABridge::clock(unsigned int ms)
{
	m_delayTimer.clock(ms);
}

bool CP25SABridge::hasPendingPDU() const
{
	return m_pendingTransmit && m_delayTimer.isRunning() && m_delayTimer.hasExpired();
}

unsigned int CP25SABridge::getPendingPDU(unsigned char* pdu, CP25NID& nid)
{
	if (!m_gpsValid) {
		reset();
		return 0U;
	}

	// LRRP uses 360.0 / 2^32 scaling for 32-bit latitude/longitude
	int32_t lrrpLat = (int32_t)(m_gpsLatitude  * (4294967296.0 / 360.0));
	int32_t lrrpLon = (int32_t)(m_gpsLongitude * (4294967296.0 / 360.0));

	// --- PDU Header (12 bytes before trellis FEC) ---
	//
	// Byte layout matches what CP25Control::writeModem parses:
	//   header[0]     = format/flags
	//   header[1]     = [2 bits flags][6 bits SAP]
	//   header[2]     = MFId
	//   header[3..5]  = LLId (source RID, 24-bit)
	//   header[6]     = [1 bit flag][7 bits blocks-to-follow]
	//   header[7..9]  = pad/offset/reserved
	//   header[10..11]= CRC-CCITT-16
	unsigned char header[P25_PDU_HEADER_LENGTH_BYTES];
	::memset(header, 0x00U, P25_PDU_HEADER_LENGTH_BYTES);

	header[0U] = 0x00U;
	header[1U] = P25_SAP_SNDCP & 0x3FU;                    // SAP 32
	header[2U] = 0x00U;                                     // standard MFId
	header[3U] = (m_gpsSrcId >> 16) & 0xFFU;
	header[4U] = (m_gpsSrcId >> 8)  & 0xFFU;
	header[5U] = m_gpsSrcId & 0xFFU;
	header[6U] = 0x01U;                                     // 1 data block
	header[7U] = 0x00U;
	header[8U] = 0x00U;
	header[9U] = 0x00U;

	CCRC::addCCITT162(header, P25_PDU_HEADER_LENGTH_BYTES);

	// --- Data block (12 bytes, unconfirmed / rate-1/2 trellis) ---
	//
	// LRRP position report payload.  Byte layout modeled on
	// TIA-102.BAHA LRRP short position report.
	// This will be refined once the XG-100P template is captured
	// via logSAP32DataBlock().
	unsigned char dataBlock[P25_PDU_UNCONFIRMED_LENGTH_BYTES];
	::memset(dataBlock, 0x00U, P25_PDU_UNCONFIRMED_LENGTH_BYTES);

	dataBlock[0U]  = 0x05U;                                 // short position report type
	dataBlock[1U]  = 0x09U;                                 // length of following position data
	dataBlock[2U]  = (lrrpLat >> 24) & 0xFFU;
	dataBlock[3U]  = (lrrpLat >> 16) & 0xFFU;
	dataBlock[4U]  = (lrrpLat >> 8)  & 0xFFU;
	dataBlock[5U]  = lrrpLat & 0xFFU;
	dataBlock[6U]  = (lrrpLon >> 24) & 0xFFU;
	dataBlock[7U]  = (lrrpLon >> 16) & 0xFFU;
	dataBlock[8U]  = (lrrpLon >> 8)  & 0xFFU;
	dataBlock[9U]  = lrrpLon & 0xFFU;
	dataBlock[10U] = 0x00U;
	dataBlock[11U] = 0x00U;

	CUtils::dump(1U, "P25 SA Bridge, TX PDU header", header, P25_PDU_HEADER_LENGTH_BYTES);
	CUtils::dump(1U, "P25 SA Bridge, TX PDU data block", dataBlock, P25_PDU_UNCONFIRMED_LENGTH_BYTES);

	// --- Assemble the raw bit buffer (no SS bits yet) ---
	//
	//   [Sync 48 bits][NID 64 bits][Header FEC 192 bits][Data FEC 192 bits]
	//   Total = 496 raw bits = 62 bytes.
	//   Sync and NID regions are overwritten after SS insertion.
	const unsigned int headerOffset    = P25_SYNC_LENGTH_BYTES + P25_NID_LENGTH_BYTES;         // 14
	const unsigned int dataBlockOffset = headerOffset + P25_PDU_FEC_LENGTH_BYTES;              // 38
	const unsigned int totalRawBits    = P25_SYNC_LENGTH_BITS + P25_NID_LENGTH_BITS
	                                   + 2U * P25_PDU_FEC_LENGTH_BITS;                         // 496

	unsigned char rawPDU[80U];
	::memset(rawPDU, 0x00U, 80U);

	CP25Trellis trellis;
	trellis.encode12(header, rawPDU + headerOffset);
	trellis.encode12(dataBlock, rawPDU + dataBlockOffset);

	// Encode raw bits → output with SS bit insertion
	::memset(pdu, 0x00U, 256U);

	unsigned int newBitLength = CP25Utils::encode(rawPDU, pdu + 2U, totalRawBits);
	unsigned int newByteLength = newBitLength / 8U;
	if ((newBitLength % 8U) > 0U)
		newByteLength++;

	CSync::addP25Sync(pdu + 2U);
	nid.encode(pdu + 2U, P25_DUID_PDU);

	pdu[0U] = TAG_DATA;
	pdu[1U] = 0x00U;

	LogMessage("P25 SA Bridge, transmitting SAP 32 PDU for RID %u: lat=%.6f lon=%.6f (%u bytes)",
		m_gpsSrcId, m_gpsLatitude, m_gpsLongitude, newByteLength + 2U);

	m_pendingTransmit = false;
	m_gpsValid = false;
	m_delayTimer.stop();

	return newByteLength + 2U;
}

void CP25SABridge::reset()
{
	m_gpsValid        = false;
	m_gpsSrcId        = 0U;
	m_gpsLatitude     = 0.0;
	m_gpsLongitude    = 0.0;
	m_pendingTransmit = false;
	m_delayTimer.stop();
}
