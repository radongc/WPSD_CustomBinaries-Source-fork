/*
*   Copyright (C) 2026 WPSD
*
*   APX-to-XG-100P SA Location Bridge
*
*   Bridges GPS location data from Motorola APX voice Link Control words
*   to L3Harris XG-100P Situational Awareness format (SAP 31 LRRP data
*   PDUs). The XG-100P's SA monitor only processes SAP 31 data PDUs;
*   it ignores GPS embedded in voice LC. This bridge extracts GPS from
*   APX voice transmissions and re-transmits it using a captured SAP 31
*   PDU template from the XG-100P.
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

CP25SABridge::CP25SABridge(unsigned int delayMs) :
m_delayMs(delayMs),
m_gpsValid(false),
m_gpsSrcId(0U),
m_gpsLatitude(0.0),
m_gpsLongitude(0.0),
m_pendingTransmit(false),
m_delayTimer(1000U, 0U, delayMs),
m_templateValid(false),
m_templateBlockCount(0U)
{
	::memset(m_templateHeader, 0x00U, 12U);
	for (unsigned int i = 0U; i < SA_BRIDGE_MAX_BLOCKS; i++) {
		::memset(m_templateBlocks[i], 0x00U, 18U);
		m_templateBlockConfirmed[i] = true;
	}
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

	// Motorola APX GPS LC (LCF=$06, MFId=$90):
	//   rs[0] = 0x06 (LCF)
	//   rs[1] = 0x90 (MFId, Motorola)
	//   rs[2] = flags / reserved
	//   rs[3..5] = latitude  (24-bit signed, MSB-first, scaled by 180.0 / 2^24)
	//   rs[6..8] = longitude (24-bit signed, MSB-first, scaled by 360.0 / 2^24)

	int32_t rawLat = ((int32_t)(int8_t)rs[3U] << 16) |
	                 ((int32_t)rs[4U] << 8) |
	                  (int32_t)rs[5U];

	int32_t rawLon = ((int32_t)(int8_t)rs[6U] << 16) |
	                 ((int32_t)rs[7U] << 8) |
	                  (int32_t)rs[8U];

	double lat = (double)rawLat * (180.0 / 16777216.0);
	double lon = (double)rawLon * (360.0 / 16777216.0);

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

void CP25SABridge::logPDU(unsigned int sap, unsigned int llId, unsigned int blockCount, const unsigned char* header, unsigned int headerLen)
{
	LogMessage("P25 SA Bridge, captured SAP %u PDU header from LLId %u, %u blocks:", sap, llId, blockCount);
	CUtils::dump(2U, "P25 SA Bridge, PDU Header bytes", header, headerLen);

	if (sap == 31U && blockCount <= SA_BRIDGE_MAX_BLOCKS) {
		::memcpy(m_templateHeader, header, headerLen);
		m_templateBlockCount = blockCount;
		m_templateValid = false;
		LogMessage("P25 SA Bridge, capturing SAP 31 template (%u blocks)...", blockCount);
	}
}

void CP25SABridge::logPDUDataBlock(unsigned int sap, const unsigned char* dataBlock, unsigned int blockLen, unsigned int blockIndex)
{
	LogMessage("P25 SA Bridge, captured SAP %u data block %u (%u bytes):", sap, blockIndex, blockLen);
	CUtils::dump(2U, "P25 SA Bridge, PDU Data Block bytes", dataBlock, blockLen);

	if (sap == 31U && blockIndex < SA_BRIDGE_MAX_BLOCKS) {
		::memset(m_templateBlocks[blockIndex], 0x00U, 18U);
		if (blockLen == P25_PDU_CONFIRMED_LENGTH_BYTES) {
			::memcpy(m_templateBlocks[blockIndex], dataBlock, 18U);
			m_templateBlockConfirmed[blockIndex] = true;
		} else {
			::memcpy(m_templateBlocks[blockIndex], dataBlock, 12U);
			m_templateBlockConfirmed[blockIndex] = false;
		}

		if (blockIndex + 1U == m_templateBlockCount) {
			m_templateValid = true;
			LogMessage("P25 SA Bridge, SAP 31 template captured successfully (%u blocks)", m_templateBlockCount);
		}
	}
}

void CP25SABridge::onVoiceEnd()
{
	if (m_gpsValid) {
		LogMessage("P25 SA Bridge, voice ended from RID %u with GPS lat=%.6f lon=%.6f, "
			"scheduling HARDCODED SAP 31 PDU in %ums",
			m_gpsSrcId, m_gpsLatitude, m_gpsLongitude, m_delayMs);
		m_pendingTransmit = true;
		m_delayTimer.start();
	}
}

void CP25SABridge::clock(unsigned int ms)
{
	m_delayTimer.clock(ms);
}

bool CP25SABridge::hasPendingPDU()
{
	return m_pendingTransmit && m_delayTimer.isRunning() && m_delayTimer.hasExpired();
}

unsigned int CP25SABridge::getPendingPDU(unsigned char* pdu, CP25NID& nid, unsigned int& bitLength)
{
	bitLength = 0U;

	if (!m_gpsValid) {
		reset();
		return 0U;
	}

	unsigned char txHeader[] = {
		0x35U, 0xDFU, 0xA4U, 0xFFU, 0xFFU, 0xFFU, 0x82U, 0x09U, 0x00U, 0x00U, 0x00U, 0x00U
	};
	CCRC::addCCITT162(txHeader, P25_PDU_HEADER_LENGTH_BYTES);

	const unsigned char blk0[] = {
		0x85U, 0x5DU, 0xDBU, 0x5DU, 0xB8U, 0xAFU, 0xADU, 0xB6U, 0xAFU, 0x10U, 0xC5U, 0x7FU,
		0x6DU, 0xAAU, 0xDBU, 0x0DU, 0x93U, 0x09U
	};
	const unsigned char blk1[] = {
		0x2DU, 0x53U, 0x1BU, 0x6BU, 0x42U, 0xBBU, 0x6DU, 0xB6U, 0xDBU, 0x6DU, 0xB6U, 0xDBU,
		0x6DU, 0xB0U, 0xDBU, 0x6DU, 0xB6U, 0xDBU
	};

	const unsigned int blockCount = 2U;
	const unsigned int totalFECBlocks = 1U + blockCount;
	const unsigned int headerOffset = P25_SYNC_LENGTH_BYTES + P25_NID_LENGTH_BYTES;
	const unsigned int totalRawBits = P25_SYNC_LENGTH_BITS + P25_NID_LENGTH_BITS
	                                + totalFECBlocks * P25_PDU_FEC_LENGTH_BITS;

	unsigned char rawPDU[120U];
	::memset(rawPDU, 0x00U, 120U);

	CP25Trellis trellis;
	trellis.encode12(txHeader, rawPDU + headerOffset);
	trellis.encode34(blk0, rawPDU + headerOffset + 1U * P25_PDU_FEC_LENGTH_BYTES);
	trellis.encode34(blk1, rawPDU + headerOffset + 2U * P25_PDU_FEC_LENGTH_BYTES);

	::memset(pdu, 0x00U, 200U);

	unsigned int newBitLength = CP25Utils::encode(rawPDU, pdu + 2U, totalRawBits);
	unsigned int newByteLength = newBitLength / 8U;
	if ((newBitLength % 8U) > 0U)
		newByteLength++;

	CSync::addP25Sync(pdu + 2U);
	nid.encode(pdu + 2U, P25_DUID_PDU);

	pdu[0U] = TAG_HEADER;
	pdu[1U] = 0x00U;

	bitLength = newBitLength;

	CUtils::dump(2U, "P25 SA Bridge, TX header (pre-trellis)", txHeader, P25_PDU_HEADER_LENGTH_BYTES);
	LogMessage("P25 SA Bridge, transmitting SAP 31 PDU (%u frame bytes, %u air bits, %u blocks)",
		newByteLength, newBitLength, blockCount);

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
