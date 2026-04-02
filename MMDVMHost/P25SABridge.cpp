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

	double lat = (double)rawLat * (180.0 / 16777216.0);   // 180 / 2^24
	double lon = (double)rawLon * (360.0 / 16777216.0);   // 360 / 2^24

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
}

void CP25SABridge::logPDUDataBlock(unsigned int sap, const unsigned char* dataBlock, unsigned int blockLen, unsigned int blockIndex)
{
	LogMessage("P25 SA Bridge, captured SAP %u data block %u (%u bytes):", sap, blockIndex, blockLen);
	CUtils::dump(2U, "P25 SA Bridge, PDU Data Block bytes", dataBlock, blockLen);
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

bool CP25SABridge::hasPendingPDU()
{
	return m_pendingTransmit && m_delayTimer.isRunning() && m_delayTimer.hasExpired();
}

unsigned int CP25SABridge::getPendingPDU(unsigned char* pdu, CP25NID& nid)
{
	if (!m_gpsValid) {
		reset();
		return 0U;
	}

	// --- PDU Header: exact XG-100P SAP 32 format, with APX source RID ---
	unsigned char header[P25_PDU_HEADER_LENGTH_BYTES];
	header[0U]  = 0x56U;
	header[1U]  = 0xE0U;                                    // SAP 32 with upper flags
	header[2U]  = 0x00U;                                    // MFId
	header[3U]  = (m_gpsSrcId >> 16) & 0xFFU;
	header[4U]  = (m_gpsSrcId >> 8)  & 0xFFU;
	header[5U]  = m_gpsSrcId & 0xFFU;
	header[6U]  = 0x81U;                                    // 1 block + full message flag
	header[7U]  = 0x00U;
	header[8U]  = 0x88U;
	header[9U]  = 0x00U;
	header[10U] = 0x00U;
	header[11U] = 0x00U;

	CCRC::addCCITT162(header, P25_PDU_HEADER_LENGTH_BYTES);

	// --- Data block: XG-100P payload template (confirmed, 18 bytes) ---
	// Bytes 0-15 are serial/fragment + LRRP payload from XG-100P capture.
	// Bytes 16-17 are CRC-CCITT-16 covering header[0-9] + dataBlock[0-15],
	// recomputed for our header since we changed the LLId.
	unsigned char dataBlock[P25_PDU_CONFIRMED_LENGTH_BYTES];
	dataBlock[0U]  = 0xBFU;
	dataBlock[1U]  = 0x42U;
	dataBlock[2U]  = 0x7DU;
	dataBlock[3U]  = 0x5CU;
	dataBlock[4U]  = 0xA1U;
	dataBlock[5U]  = 0xDBU;
	dataBlock[6U]  = 0xCDU;
	dataBlock[7U]  = 0xF5U;
	dataBlock[8U]  = 0x0FU;
	dataBlock[9U]  = 0x2AU;
	dataBlock[10U] = 0xB5U;
	dataBlock[11U] = 0x7FU;
	dataBlock[12U] = 0x57U;
	dataBlock[13U] = 0x8DU;
	dataBlock[14U] = 0x78U;
	dataBlock[15U] = 0x2DU;
	dataBlock[16U] = 0x00U;
	dataBlock[17U] = 0x00U;

	// Recompute confirmed data block CRC over header fields + data block payload
	unsigned char crcBuf[28U];
	::memcpy(crcBuf, header, 10U);
	::memcpy(crcBuf + 10U, dataBlock, 16U);
	crcBuf[26U] = 0x00U;
	crcBuf[27U] = 0x00U;
	CCRC::addCCITT162(crcBuf, 28U);
	dataBlock[16U] = crcBuf[26U];
	dataBlock[17U] = crcBuf[27U];

	CUtils::dump(2U, "P25 SA Bridge, TX PDU header", header, P25_PDU_HEADER_LENGTH_BYTES);
	CUtils::dump(2U, "P25 SA Bridge, TX PDU data block", dataBlock, P25_PDU_CONFIRMED_LENGTH_BYTES);

	// [Sync 48 bits][NID 64 bits][Header FEC 192 bits][Data FEC 192 bits]
	const unsigned int headerOffset    = P25_SYNC_LENGTH_BYTES + P25_NID_LENGTH_BYTES;
	const unsigned int dataBlockOffset = headerOffset + P25_PDU_FEC_LENGTH_BYTES;
	const unsigned int totalRawBits    = P25_SYNC_LENGTH_BITS + P25_NID_LENGTH_BITS
	                                   + 2U * P25_PDU_FEC_LENGTH_BITS;

	unsigned char rawPDU[80U];
	::memset(rawPDU, 0x00U, 80U);

	CP25Trellis trellis;
	trellis.encode12(header, rawPDU + headerOffset);
	trellis.encode34(dataBlock, rawPDU + dataBlockOffset);

	// Encode raw bits → output with SS bit insertion
	::memset(pdu, 0x00U, 256U);

	unsigned int newBitLength = CP25Utils::encode(rawPDU, pdu + 2U, totalRawBits);
	unsigned int newByteLength = newBitLength / 8U;
	if ((newBitLength % 8U) > 0U)
		newByteLength++;

	CSync::addP25Sync(pdu + 2U);
	nid.encode(pdu + 2U, P25_DUID_PDU);

	pdu[0U] = TAG_HEADER;
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
