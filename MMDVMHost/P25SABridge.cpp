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
#include <cstdio>
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
m_templateBlockCount(0U),
m_firstCapValid(false),
m_firstCapLen(0U),
m_firstCapCfLen(0U)
{
	::memset(m_rawPDU, 0x00U, 300U);
	::memset(m_templateHeader, 0x00U, 12U);
	::memset(m_firstCapPayload, 0x00U, 256U);
	::memset(m_firstCapCfPayload, 0x00U, 256U);
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

	{
		char ascii[257U];
		unsigned int n = payloadLen < 256U ? payloadLen : 256U;
		for (unsigned int i = 0U; i < n; i++) {
			unsigned char c = payload[i];
			ascii[i] = (c >= 0x20U && c <= 0x7EU) ? (char)c : '.';
		}
		ascii[n] = '\0';
		LogMessage("P25 SA Bridge, LRRP raw payload ASCII: \"%s\"", ascii);

		for (unsigned int i = 0U; i + 5U < payloadLen; i++) {
			if (payload[i] == '$' && payload[i+1U] >= 'A' && payload[i+1U] <= 'Z' &&
			    payload[i+2U] >= 'A' && payload[i+2U] <= 'Z') {
				char nmea[128U];
				unsigned int j = 0U;
				while (j < 127U && i + j < payloadLen) {
					unsigned char c = payload[i + j];
					if (c == '\r' || c == '\n' || c < 0x20U || c > 0x7EU)
						break;
					nmea[j] = (char)c;
					j++;
				}
				nmea[j] = '\0';
				LogMessage("P25 SA Bridge, NMEA candidate at byte %u (full-block): %s", i, nmea);
			}
		}
	}

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

	if (cfLen > 0U) {
		CUtils::dump(2U, "P25 SA Bridge, LRRP confirmed-shifted payload (7-bit offset)", cfPayload, cfLen);

		char ascii[257U];
		unsigned int n = cfLen < 256U ? cfLen : 256U;
		for (unsigned int i = 0U; i < n; i++) {
			unsigned char c = cfPayload[i];
			ascii[i] = (c >= 0x20U && c <= 0x7EU) ? (char)c : '.';
		}
		ascii[n] = '\0';
		LogMessage("P25 SA Bridge, LRRP confirmed-shifted payload ASCII: \"%s\"", ascii);

		for (unsigned int i = 0U; i + 5U < cfLen; i++) {
			if (cfPayload[i] == '$' && cfPayload[i+1U] >= 'A' && cfPayload[i+1U] <= 'Z' &&
			    cfPayload[i+2U] >= 'A' && cfPayload[i+2U] <= 'Z') {
				char nmea[128U];
				unsigned int j = 0U;
				while (j < 127U && i + j < cfLen) {
					unsigned char c = cfPayload[i + j];
					if (c == '\r' || c == '\n' || c < 0x20U || c > 0x7EU)
						break;
					nmea[j] = (char)c;
					j++;
				}
				nmea[j] = '\0';
				LogMessage("P25 SA Bridge, NMEA candidate at byte %u (cf-shifted): %s", i, nmea);
			}
		}
	}

	if (!m_firstCapValid) {
		if (payloadLen <= 256U) {
			::memcpy(m_firstCapPayload, payload, payloadLen);
			m_firstCapLen = payloadLen;
		}
		if (cfLen <= 256U) {
			::memcpy(m_firstCapCfPayload, cfPayload, cfLen);
			m_firstCapCfLen = cfLen;
		}
		m_firstCapValid = true;
		LogMessage("P25 SA Bridge, stored first SAP 31 capture as diff reference "
		           "(%u full-block / %u cf-shifted bytes) - move radio and beacon again to locate GPS fields",
		           payloadLen, cfLen);
	} else {
		diffAgainstFirst("full-block", payload, payloadLen, m_firstCapPayload, m_firstCapLen);
		diffAgainstFirst("cf-shifted", cfPayload, cfLen, m_firstCapCfPayload, m_firstCapCfLen);
	}

	parseLRRP(cfPayload, cfLen);
}

void CP25SABridge::diffAgainstFirst(const char* label, const unsigned char* cur, unsigned int curLen,
                                    const unsigned char* first, unsigned int firstLen)
{
	unsigned int cmpLen = curLen < firstLen ? curLen : firstLen;
	if (cmpLen == 0U)
		return;

	char line[640U];
	unsigned int pos = 0U;
	unsigned int changes = 0U;

	for (unsigned int i = 0U; i < cmpLen && pos < 620U; i++) {
		if (cur[i] != first[i]) {
			int n = ::snprintf(line + pos, 640U - pos, " [%u:%02X->%02X]",
			                   i, first[i], cur[i]);
			if (n > 0)
				pos += (unsigned int)n;
			changes++;
		}
	}
	line[pos] = '\0';

	if (changes == 0U)
		LogMessage("P25 SA Bridge, %s payload IDENTICAL to first capture (%u bytes) - radio hasn't moved",
		           label, cmpLen);
	else
		LogMessage("P25 SA Bridge, %s payload diff vs first (%u/%u bytes changed):%s",
		           label, changes, cmpLen, line);
}

void CP25SABridge::parseLRRP(const unsigned char* data, unsigned int len)
{
	if (len < 6U)
		return;

	unsigned int off = 0U;
	while (off < len && data[off] != 0x24U)
		off++;

	if (off >= len) {
		LogMessage("P25 SA Bridge, LRRP parse: no '$' start marker found in cf-shifted payload");
		return;
	}

	LogMessage("P25 SA Bridge, LRRP parse: '$' start marker at cf-shifted offset %u", off);

	// Hypothesis: [0x24] [msgType] [topTag] [topLen] [value bytes ...]
	if (off + 4U >= len)
		return;

	unsigned char msgType = data[off + 1U];
	unsigned char topTag  = data[off + 2U];
	unsigned char topLen  = data[off + 3U];

	LogMessage("P25 SA Bridge, LRRP header: msgType=0x%02X topTag=0x%02X topLen=%u (0x%02X)",
	           msgType, topTag, topLen, topLen);

	unsigned int innerStart = off + 4U;
	unsigned int innerEnd   = innerStart + topLen;
	if (innerEnd > len)
		innerEnd = len;

	// Walk as 1-byte-tag / 1-byte-len TLVs
	unsigned int cur = innerStart;
	unsigned int tlvCount = 0U;
	while (cur + 2U <= innerEnd && tlvCount < 32U) {
		unsigned char t = data[cur];
		unsigned char l = data[cur + 1U];

		if (l == 0U) {
			cur++;
			continue;
		}
		if (cur + 2U + l > innerEnd)
			break;

		char hex[80U];
		unsigned int hp = 0U;
		unsigned int show = l < 16U ? l : 16U;
		for (unsigned int i = 0U; i < show && hp < 72U; i++)
			hp += (unsigned int)::snprintf(hex + hp, 80U - hp, "%02X ", data[cur + 2U + i]);
		hex[hp] = '\0';

		if (l == 4U) {
			uint32_t v = ((uint32_t)data[cur+2U] << 24) | ((uint32_t)data[cur+3U] << 16) |
			             ((uint32_t)data[cur+4U] << 8)  |  (uint32_t)data[cur+5U];
			double as2p32 = (double)(int32_t)v * (360.0 / 4294967296.0);
			double asE7   = (double)(int32_t)v / 1e7;
			LogMessage("P25 SA Bridge, LRRP TLV @%u tag=0x%02X len=%u val=%s "
			           "[int32=%d  360/2^32=%.6f  deg*1e7=%.6f]",
			           cur, t, l, hex, (int32_t)v, as2p32, asE7);
		} else if (l == 8U) {
			uint32_t hi = ((uint32_t)data[cur+2U] << 24) | ((uint32_t)data[cur+3U] << 16) |
			              ((uint32_t)data[cur+4U] << 8)  |  (uint32_t)data[cur+5U];
			uint32_t lo = ((uint32_t)data[cur+6U] << 24) | ((uint32_t)data[cur+7U] << 16) |
			              ((uint32_t)data[cur+8U] << 8)  |  (uint32_t)data[cur+9U];
			double dLat = (double)(int32_t)hi * (360.0 / 4294967296.0);
			double dLon = (double)(int32_t)lo * (360.0 / 4294967296.0);
			LogMessage("P25 SA Bridge, LRRP TLV @%u tag=0x%02X len=%u val=%s "
			           "[as 2x32b 360/2^32: lat=%.6f lon=%.6f]",
			           cur, t, l, hex, dLat, dLon);
		} else {
			LogMessage("P25 SA Bridge, LRRP TLV @%u tag=0x%02X len=%u val=%s%s",
			           cur, t, l, hex, l > 16U ? "..." : "");
		}

		cur += 2U + l;
		tlvCount++;
	}

	LogMessage("P25 SA Bridge, LRRP parse: walked %u TLVs in top value (%u..%u), stopped at %u",
	           tlvCount, innerStart, innerEnd, cur);
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
