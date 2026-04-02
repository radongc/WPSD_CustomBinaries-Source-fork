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
m_rawValid(false),
m_rawBitLength(0U),
m_beaconTimer(1000U, 10U, 0U),
m_beaconActive(false),
m_templateValid(false),
m_templateBlockCount(0U)
{
	::memset(m_rawPDU, 0x00U, 300U);
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

void CP25SABridge::captureRawPDU(unsigned int sap, const unsigned char* rfPDU, unsigned int bitLength)
{
	if (sap != 31U)
		return;

	if (m_rawValid && m_beaconActive) {
		LogMessage("P25 SA Bridge, ignoring new SAP 31 capture (beacon already active with locked template)");
		return;
	}

	unsigned int byteLen = bitLength / 8U;
	if ((bitLength % 8U) > 0U)
		byteLen++;

	if (byteLen > 300U) {
		LogMessage("P25 SA Bridge, raw PDU too large (%u bytes), skipping", byteLen);
		return;
	}

	::memcpy(m_rawPDU, rfPDU, byteLen);
	m_rawBitLength = bitLength;
	m_rawValid = true;

	LogMessage("P25 SA Bridge, captured raw SAP 31 PDU bitstream (%u bits, %u bytes)", bitLength, byteLen);

	if (!m_beaconActive) {
		m_beaconActive = true;
		m_beaconTimer.start();
		LogMessage("P25 SA Bridge, starting continuous SA beacon (10s interval)");
	}
}

void CP25SABridge::onVoiceEnd()
{
	if (m_gpsValid) {
		LogMessage("P25 SA Bridge, voice ended from RID %u with GPS lat=%.6f lon=%.6f, "
			"scheduling RAW REPLAY SAP 31 PDU in %ums (raw template %s)",
			m_gpsSrcId, m_gpsLatitude, m_gpsLongitude, m_delayMs,
			m_rawValid ? "available" : "NOT available");
		m_pendingTransmit = true;
		m_delayTimer.start();
	}
}

void CP25SABridge::clock(unsigned int ms)
{
	m_delayTimer.clock(ms);
	m_beaconTimer.clock(ms);
}

bool CP25SABridge::hasPendingPDU()
{
	if (m_pendingTransmit && m_delayTimer.isRunning() && m_delayTimer.hasExpired())
		return true;

	if (m_beaconActive && m_rawValid && m_beaconTimer.isRunning() && m_beaconTimer.hasExpired())
		return true;

	return false;
}

unsigned int CP25SABridge::getPendingPDU(unsigned char* pdu, CP25NID& nid, unsigned int& bitLength)
{
	bitLength = 0U;

	if (!m_rawValid) {
		LogMessage("P25 SA Bridge, no raw SAP 31 template captured yet, skipping TX");
		m_pendingTransmit = false;
		m_delayTimer.stop();
		return 0U;
	}

	const unsigned int headerOffset = P25_SYNC_LENGTH_BYTES + P25_NID_LENGTH_BYTES;

	CP25Trellis trellis;
	unsigned char header[P25_PDU_HEADER_LENGTH_BYTES];
	if (trellis.decode12(m_rawPDU + headerOffset, header)) {
		unsigned int blockCount = header[6U] & 0x7FU;
		unsigned int padCount   = (header[7U] >> 3) & 0x1FU;
		CUtils::dump(2U, "P25 SA Bridge, TX header (original)", header, P25_PDU_HEADER_LENGTH_BYTES);
		LogMessage("P25 SA Bridge, TX header: SAP=%u MFID=$%02X LLId=%u blocks=%u pad=%u",
			header[1U] & 0x3FU, header[2U],
			(header[3U] << 16) | (header[4U] << 8) | header[5U],
			blockCount, padCount);

		unsigned char lrrpPayload[256U];
		unsigned int lrrpLen = 0U;

		for (unsigned int i = 0U; i < blockCount; i++) {
			unsigned char blockData[P25_PDU_CONFIRMED_LENGTH_BYTES];
			unsigned int blockOffset = headerOffset + P25_PDU_FEC_LENGTH_BYTES + i * P25_PDU_FEC_LENGTH_BYTES;
			if (trellis.decode34(m_rawPDU + blockOffset, blockData)) {
				unsigned int serial = blockData[0U] >> 1;
				CUtils::dump(2U, "P25 SA Bridge, TX block (real)", blockData, P25_PDU_CONFIRMED_LENGTH_BYTES);
				LogMessage("P25 SA Bridge, TX block %u serial=%u", i, serial);

				unsigned int dataBytes = 16U;
				if (i == blockCount - 1U && padCount > 0U)
					dataBytes = (padCount < 16U) ? 16U - padCount : 0U;
				if (lrrpLen + dataBytes <= 256U) {
					for (unsigned int b = 0U; b < dataBytes; b++) {
						unsigned int srcBit = 7U + b * 8U;
						unsigned int srcByte = srcBit / 8U;
						unsigned int srcShift = 7U - (srcBit % 8U);
						unsigned char val = 0U;
						for (unsigned int bit = 0U; bit < 8U; bit++) {
							unsigned int pos = 7U + b * 8U + bit;
							unsigned int byteIdx = pos / 8U;
							unsigned int bitIdx  = 7U - (pos % 8U);
							if (byteIdx < 18U && (blockData[byteIdx] & (1U << bitIdx)))
								val |= (1U << (7U - bit));
						}
						lrrpPayload[lrrpLen++] = val;
					}
				}
			} else {
				LogMessage("P25 SA Bridge, TX block %u trellis decode FAILED", i);
			}
		}

		if (lrrpLen > 0U) {
			CUtils::dump(2U, "P25 SA Bridge, LRRP payload (extracted)", lrrpPayload, lrrpLen);
			if (lrrpLen >= 3U)
				LogMessage("P25 SA Bridge, LRRP source RID (first 3 bytes): %u (0x%02X%02X%02X)",
					(lrrpPayload[0U] << 16) | (lrrpPayload[1U] << 8) | lrrpPayload[2U],
					lrrpPayload[0U], lrrpPayload[1U], lrrpPayload[2U]);
		}
	}

	::memset(pdu, 0x00U, 600U);

	unsigned int airBitLength = CP25Utils::encode(m_rawPDU, pdu + 2U, m_rawBitLength);
	unsigned int airByteLength = airBitLength / 8U;
	if ((airBitLength % 8U) > 0U)
		airByteLength++;

	CSync::addP25Sync(pdu + 2U);
	nid.encode(pdu + 2U, P25_DUID_PDU);

	pdu[0U] = TAG_HEADER;
	pdu[1U] = 0x00U;

	bitLength = airBitLength;

	LogMessage("P25 SA Bridge, transmitting FULL SAP 31 PDU (%u air bytes, %u raw bits)",
		airByteLength, m_rawBitLength);

	m_pendingTransmit = false;
	m_gpsValid = false;
	m_delayTimer.stop();
	m_beaconTimer.start();

	return airByteLength + 2U;
}

void CP25SABridge::reset()
{
	m_gpsValid        = false;
	m_gpsSrcId        = 0U;
	m_gpsLatitude     = 0.0;
	m_gpsLongitude    = 0.0;
	m_pendingTransmit = false;
	m_delayTimer.stop();
	m_beaconTimer.stop();
	m_beaconActive    = false;
}
