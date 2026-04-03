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
		decodeSAP31(rfPDU, bitLength);
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

	decodeSAP31(rfPDU, bitLength);

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

void CP25SABridge::decodeSAP31(const unsigned char* rfPDU, unsigned int bitLength)
{
	const unsigned int headerOffset = P25_SYNC_LENGTH_BYTES + P25_NID_LENGTH_BYTES;

	CP25Trellis trellis;
	unsigned char header[P25_PDU_HEADER_LENGTH_BYTES];
	if (!trellis.decode12(rfPDU + headerOffset, header))
		return;

	unsigned int sap        = header[1U] & 0x3FU;
	unsigned int mfid       = header[2U];
	unsigned int llId       = (header[3U] << 16) | (header[4U] << 8) | header[5U];
	unsigned int blockCount = header[6U] & 0x7FU;
	unsigned int padCount   = (header[7U] >> 3) & 0x1FU;
	unsigned int format     = (header[0U] >> 4) & 0x03U;

	LogMessage("P25 SA Bridge, LRRP decode: SAP=%u MFID=$%02X LLId=%u fmt=%u blocks=%u pad=%u",
		sap, mfid, llId, format, blockCount, padCount);

	unsigned char payload[256U];
	unsigned int payloadLen = 0U;

	for (unsigned int i = 0U; i < blockCount && i < SA_BRIDGE_MAX_BLOCKS; i++) {
		unsigned char blockData[P25_PDU_CONFIRMED_LENGTH_BYTES];
		unsigned int blockOffset = headerOffset + P25_PDU_FEC_LENGTH_BYTES + i * P25_PDU_FEC_LENGTH_BYTES;

		if (!trellis.decode34(rfPDU + blockOffset, blockData))
			continue;

		unsigned int copyLen = 18U;
		if (i == blockCount - 1U && padCount > 0U && padCount < 18U)
			copyLen = 18U - padCount;

		if (payloadLen + copyLen <= 256U) {
			::memcpy(payload + payloadLen, blockData, copyLen);
			payloadLen += copyLen;
		}
	}

	if (payloadLen == 0U)
		return;

	CUtils::dump(2U, "P25 SA Bridge, LRRP raw payload (full block bytes)", payload, payloadLen);

	unsigned char cfPayload[256U];
	unsigned int cfLen = 0U;
	for (unsigned int i = 0U; i < blockCount && i < SA_BRIDGE_MAX_BLOCKS; i++) {
		unsigned char blockData[P25_PDU_CONFIRMED_LENGTH_BYTES];
		unsigned int blockOffset = headerOffset + P25_PDU_FEC_LENGTH_BYTES + i * P25_PDU_FEC_LENGTH_BYTES;
		if (!trellis.decode34(rfPDU + blockOffset, blockData))
			continue;
		unsigned int dataBytes = 16U;
		if (i == blockCount - 1U && padCount > 0U && padCount < 16U)
			dataBytes = 16U - padCount;
		if (cfLen + dataBytes <= 256U) {
			for (unsigned int b = 0U; b < dataBytes; b++) {
				unsigned char val = 0U;
				for (unsigned int bit = 0U; bit < 8U; bit++) {
					unsigned int pos = 7U + b * 8U + bit;
					unsigned int byteIdx = pos / 8U;
					unsigned int bitIdx  = 7U - (pos % 8U);
					if (byteIdx < 18U && (blockData[byteIdx] & (1U << bitIdx)))
						val |= (1U << (7U - bit));
				}
				cfPayload[cfLen++] = val;
			}
		}
	}

	if (cfLen > 0U)
		CUtils::dump(2U, "P25 SA Bridge, LRRP confirmed-shifted payload (7-bit offset)", cfPayload, cfLen);

	const double S32 = 360.0 / 4294967296.0;

	const char* labels[2] = { "full", "cfmt" };
	const unsigned char* bufs[2] = { payload, cfPayload };
	const unsigned int lens[2] = { payloadLen, cfLen };

	for (unsigned int p = 0U; p < 2U; p++) {
		const unsigned char* buf = bufs[p];
		unsigned int len = lens[p];

		for (unsigned int i = 0U; i + 3U < len; i++) {
			uint32_t v = ((uint32_t)buf[i] << 24) | ((uint32_t)buf[i+1U] << 16) |
			             ((uint32_t)buf[i+2U] << 8) | (uint32_t)buf[i+3U];
			double deg = (double)(int32_t)v * S32;

			if (deg >= 39.5 && deg <= 41.5)
				LogMessage("P25 SA Bridge, LAT candidate [%s] byte %u: %.6f (0x%08X)",
					labels[p], i, deg, v);
			if (deg >= -83.5 && deg <= -81.5)
				LogMessage("P25 SA Bridge, LON candidate [%s] byte %u: %.6f (0x%08X)",
					labels[p], i, deg, v);
		}

		for (unsigned int i = 0U; i + 3U < len; i++) {
			uint32_t v = ((uint32_t)buf[i] << 24) | ((uint32_t)buf[i+1U] << 16) |
			             ((uint32_t)buf[i+2U] << 8) | (uint32_t)buf[i+3U];
			float f;
			::memcpy(&f, &v, 4U);
			if (f >= 39.5f && f <= 41.5f)
				LogMessage("P25 SA Bridge, LAT float [%s] byte %u: %.6f (0x%08X)",
					labels[p], i, (double)f, v);
			if (f >= -83.5f && f <= -81.5f)
				LogMessage("P25 SA Bridge, LON float [%s] byte %u: %.6f (0x%08X)",
					labels[p], i, (double)f, v);
		}
	}
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

	LogMessage("P25 SA Bridge, transmitting SAP 31 PDU (%u air bytes)", airByteLength);

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
