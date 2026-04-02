/*
*   Copyright (C) 2026 WPSD
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

#if !defined(P25SABridge_H)
#define P25SABridge_H

#include "P25NID.h"
#include "Timer.h"

const unsigned int SA_BRIDGE_MAX_BLOCKS = 9U;

class CP25SABridge {
public:
	CP25SABridge(unsigned int delayMs);
	~CP25SABridge();

	void processVoiceLC(const unsigned char* rs, unsigned int srcId);

	void logPDU(unsigned int sap, unsigned int llId, unsigned int blockCount, const unsigned char* header, unsigned int headerLen);
	void logPDUDataBlock(unsigned int sap, const unsigned char* dataBlock, unsigned int blockLen, unsigned int blockIndex);

	void captureRawPDU(unsigned int sap, const unsigned char* rfPDU, unsigned int bitLength);

	void onVoiceEnd();

	void clock(unsigned int ms);

	bool hasPendingPDU();
	unsigned int getPendingPDU(unsigned char* pdu, CP25NID& nid, unsigned int& bitLength);

	void reset();

private:
	unsigned int m_delayMs;

	bool         m_gpsValid;
	unsigned int m_gpsSrcId;
	double       m_gpsLatitude;
	double       m_gpsLongitude;

	bool         m_pendingTransmit;
	CTimer       m_delayTimer;

	bool          m_rawValid;
	unsigned int  m_rawBitLength;
	unsigned char m_rawPDU[300U];

	bool          m_templateValid;
	unsigned int  m_templateBlockCount;
	unsigned char m_templateHeader[12U];
	unsigned char m_templateBlocks[SA_BRIDGE_MAX_BLOCKS][18U];
	bool          m_templateBlockConfirmed[SA_BRIDGE_MAX_BLOCKS];
};

#endif
