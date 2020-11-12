#include "ESP32WROOM.h"

//#define DEBUG
extern volatile uint32_t g_ul_ms_ticks;
extern "C" {
	void wdt_restart(Wdt* p_wdt);
}

MyRingBuffer::MyRingBuffer(void)
{
	memset(_aucBuffer, 0, BUFFER_MAX_SIZE);
	_iHead = 0;
	_iTail = 0;
}

void MyRingBuffer::store_byte(uint8_t c)
{
	int i = (uint32_t)(_iHead + 1) % BUFFER_MAX_SIZE;

	_aucBuffer[_iHead] = c;
	_iHead = i;
	if (i == _iTail)
	{
		_iTail = (uint32_t)(_iTail + 1) % BUFFER_MAX_SIZE;
	}
}

bool MyRingBuffer::is_full(void)
{
	int i = (uint32_t)(_iHead + 1) % BUFFER_MAX_SIZE;
	return i == _iTail;
}

int MyRingBuffer::find_byte(byte c, int begin, int end)
{
	int len = length();
	if (end == -1 || end > len) end = len;
	if (begin >= end) return -1;
	int i = _iTail + begin, j = begin;
	while (j < end)
	{
		if (_aucBuffer[i] == c)
		{
			return j;
		}
		i = (uint32_t)(i + 1) % BUFFER_MAX_SIZE;
		j++;
	}
	return -1;
}

// kmp algorithm for find_bytes
static void Next(byte* T, int* next, int size) {
	int i = 1;
	next[1] = 0;
	int j = 0;
	while (i < size) {
		if (j == 0 || T[i - 1] == T[j - 1]) {
			i++;
			j++;
			next[i] = j;
		}
		else {
			j = next[j];
		}
	}
}

int MyRingBuffer::find_bytes(byte* p, size_t n)
{
	int next[FINDBYTES_MAX_SIZE + 1];
	Next(p, next, n);
	int i = 1;
	int j = 1;
	int s_size = length();
	while (i <= s_size && j <= n) {
		if (j == 0 || read_byte(i - 1) == p[j - 1]) {
			i++;
			j++;
		}
		else {
			j = next[j];
		}
	}
	if (j > n) {
		return i - n - 1;
	}
	return -1;
}

int MyRingBuffer::find_word(uint16_t w, int begin)
{
	if (begin >= length()) return -1;
	int i = _iTail + begin, j = begin;
	uint8_t L = w & 0xff;
	uint8_t H = w >> 8;
	while (i != _iHead)
	{
		if (_aucBuffer[i] == L)
		{
			int k = (uint32_t)(i + 1) % BUFFER_MAX_SIZE;
			if (k != _iHead)
			{
				if (_aucBuffer[k] == H)
				{
					return j;
				}
			}
		}
		i = (uint32_t)(i + 1) % BUFFER_MAX_SIZE;
		j++;
	}
	return -1;
}

bool MyRingBuffer::cmp_bytes(byte* p, size_t n, int begin)
{
	if (n == 0)
	{
		n = strlen((char*)p);
	}
	if (begin + n > length()) return false;
	int i = (uint32_t)(_iTail + begin) % BUFFER_MAX_SIZE;
	int j = 0;
	while (j < n)
	{
		if (p[j] != _aucBuffer[i])
		{
			return false;
		}
		i = (uint32_t)(i + 1) % BUFFER_MAX_SIZE;
		j++;
	}
	return true;
}

void MyRingBuffer::cut(int count)
{
	if (count > length())
		clear();
	else
		_iTail = (uint32_t)(_iTail + count) % BUFFER_MAX_SIZE;
}

int MyRingBuffer::length(void)
{
	if (_iTail <= _iHead)
		return _iHead - _iTail;
	else
		return _iHead + BUFFER_MAX_SIZE - _iTail;
}

void MyRingBuffer::clear(void)
{
	_iTail = _iHead;
}

uint8_t MyRingBuffer::operator[](int index)
{
	return read_byte(index);
}

uint8_t MyRingBuffer::read_byte(int index)
{
	int i = (uint32_t)(_iTail + index) % BUFFER_MAX_SIZE;
	return _aucBuffer[i];
}

size_t MyRingBuffer::read_bytes(uint8_t* buffer, size_t n, int begin)
{
	if (begin >= length()) return 0;
	int i = (uint32_t)(_iTail + begin) % BUFFER_MAX_SIZE;
	int j = 0;
	while (i != _iHead && j < n)
	{
		buffer[j] = _aucBuffer[i];
		i = (uint32_t)(i + 1) % BUFFER_MAX_SIZE;
		j++;
	}
	return j;
}

void MyRingBuffer::set_byte(int index, byte v)
{
	if (index >= length()) return;
	int i = (uint32_t)(_iTail + index) % BUFFER_MAX_SIZE;
	_aucBuffer[i] = v;
}

void MyRingBuffer::set_bytes(int begin, int end, byte v)
{
	int len = length();
	if (end > len) end = len;
	if (begin >= end) return;
	int i = (uint32_t)(_iTail + begin) % BUFFER_MAX_SIZE;
	end = (uint32_t)(_iTail + end) % BUFFER_MAX_SIZE;
	while (i != end)
	{
		_aucBuffer[i] == v;
		i = (uint32_t)(i + 1) % BUFFER_MAX_SIZE;
	}
}

Esp32::Esp32()
	: m_pSerial(NULL)
	, m_lastSendTime(0)
	, m_busy(false)
	, m_lastCMD(CMD_NONE)
	, m_pWifi(NULL)
	, m_isMUX(false)
	, m_rxDataLinkID(0)
	, m_rxDataRestSize(0)
	, m_rxGetAPIP(0)
	, m_rxGetSTAIP(0)
	, m_rxGetAPMask(0)
	, m_rxGetSTAMask(0)
	, m_apFound(false)
{
}

void Esp32::loop(void)
{
	bool hasRead = false;
	while (m_pSerial->available())
	{
		m_rxBuffer.store_byte(m_pSerial->read());
		hasRead = true;
	}
#ifdef WIFI_DEBUG
	static uint32_t tick = 0;
	if (hasRead || g_ul_ms_ticks - tick > 3000)
	{
		int len = m_rxBuffer.length();
		char* str = new char[len + 1];
		int i;
		for (i = 0; i < len; i++)
		{
			str[i] = m_rxBuffer[i];
		}
		str[i] = 0;
		WIFI_DEBUG_printf("\r\nRECEIVE %d:\r\n%s\r\n", len, str);
		tick = g_ul_ms_ticks;
		delete[] str;
	}
#endif
	if (hasRead)
	{
		parseReceived();
	}
	checkTimeout();
}

void Esp32::init(void)
{
	setSerial(WIFI_SERIAL, WIFI_BAUDRATE, WIFI_FLOWCTR);
}

void Esp32::setSerial(USARTClass& hSerial, int aBaud, bool en)
{
	m_pSerial = &hSerial;
	m_pSerial->setCTSPin(DIGIFI_CTS);
	m_pSerial->enableCTS(en);
	m_pSerial->begin(aBaud);
}

USARTClass& Esp32::getSerial()
{
	return *m_pSerial;
}

bool Esp32::reset(IWifi* pWifi)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_RESET;
	m_pSerial->println("AT+RST");
	return true;
}

bool Esp32::recovery(IWifi* pWifi)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_RECOVERY;
	m_pSerial->println("AT+RESTORE");
	return true;
}

bool Esp32::startSoftAP(IWifi* pWifi)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_SETMODE;
	m_pSerial->println("AT+CWMODE=2");
	return true;
}

bool Esp32::startStation(IWifi* pWifi)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_SETMODE;
	m_pSerial->println("AT+CWMODE=1");
	return true;
}

bool Esp32::startAPStation(IWifi* pWifi)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_SETMODE;
	m_pSerial->println("AT+CWMODE=3");
	return true;
}

bool Esp32::setSoftAP(IWifi* pWifi, const char ssid[], const char pwd[], int ch, EncryptType ecn, int max_conn, bool hidden)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_SETSOFTAP;
	char str[4];
	m_pSerial->print("AT+CWSAP=\"");
	m_pSerial->print(ssid);
	m_pSerial->print("\",\"");
	m_pSerial->print(pwd);
	m_pSerial->print("\",");
	m_pSerial->print(itoa(ch, str, 10));
	m_pSerial->write(',');
	m_pSerial->print(itoa(ecn, str, 10));
	if (max_conn > 0)
	{
		m_pSerial->write(',');
		m_pSerial->print(itoa(max_conn, str, 10));
		m_pSerial->write(',');
		m_pSerial->write('0' + hidden);
	}
	m_pSerial->println();
	return true;
}

bool Esp32::connectAP(IWifi* pWifi, const char ssid[], const char pwd[], const char* bssid)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_CONNECTAP;
	m_pSerial->print("AT+CWJAP=\"");
	m_pSerial->print(ssid);
	m_pSerial->print("\",\"");
	m_pSerial->print(pwd);
	m_pSerial->write('"');
	if (bssid != NULL)
	{
		m_pSerial->print(",\"");
		m_pSerial->print(bssid);
		m_pSerial->write('"');
	}
	m_pSerial->println();
	return true;
}

bool Esp32::configSanAP(IWifi* pWifi, bool sort, uint8_t mask)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_CONFIGSCANAP;
	m_pSerial->print("AT+CWLAPOPT=");
	m_pSerial->write('0' + sort);
	m_pSerial->write(',');
	m_pSerial->print(mask, 10);
	m_pSerial->println();
	return true;
}

bool Esp32::scanAP(IWifi* pWifi, const char ssid[])
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_SCANAP;
	m_pSerial->print("AT+CWLAP");
	if (ssid != NULL)
	{
		m_apFound = true;
		m_pSerial->print("=\"");
		m_pSerial->print(ssid);
		m_pSerial->write('"');
	}
	m_pSerial->println();
	return true;
}

bool Esp32::autoConnAP(IWifi* pWifi, bool isAuto)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_AUTOCONN;
	m_pSerial->print("AT+CWAUTOCONN=");
	m_pSerial->write('0' + isAuto);
	m_pSerial->println();
	return true;
}

bool Esp32::getIP(IWifi* pWifi)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_GETIP;
	m_pSerial->println("AT+CIFSR");
	m_rxGetAPIP = 0;
	m_rxGetSTAIP = 0;
	return true;
}

bool Esp32::getAPIP(IWifi* pWifi)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_GETAPIP;
	m_pSerial->println("AT+CIPAP?");
	return true;
}

bool Esp32::getSTAIP(IWifi* pWifi)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_GETSTAIP;
	m_pSerial->println("AT+CIPSTA?");
	return true;
}

bool Esp32::getNetStatus(IWifi* pWifi)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_GETNETSTATUS;
	m_pSerial->println("AT+CIPSTATUS");
	return true;
}

bool Esp32::setMUX(IWifi* pWifi, bool isMUX)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_SETMUX;
	m_pSerial->print("AT+CIPMUX=");
	m_pSerial->write('0' + isMUX);
	m_pSerial->println();
	m_isMUX = isMUX;
	return true;
}

bool Esp32::startTCPServer(IWifi* pWifi, uint16_t port)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_TCPSERVER;
	m_pSerial->print("AT+CIPSERVER=1,");
	m_pSerial->println(port, 10);
	return true;
}

bool Esp32::stopTCPServer(IWifi* pWifi, uint16_t port)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_TCPSERVER_STOP;
	m_pSerial->print("AT+CIPSERVER=0,");
	m_pSerial->println(port, 10);
	return true;
}

bool Esp32::TCPConnectMUX(IWifi* pWifi, uint8_t link_id, uint32_t remote_ip, uint16_t remote_port)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_TCPCONNECT;
	m_pSerial->print("AT+CIPSTART=");
	m_pSerial->write('0' + link_id);
	m_pSerial->print(",\"TCP\",\"");
	m_pSerial->print(remote_ip >> 24, 10);
	m_pSerial->write('.');
	m_pSerial->print((remote_ip >> 16) & 0xff, 10);
	m_pSerial->write('.');
	m_pSerial->print((remote_ip >> 8) & 0xff, 10);
	m_pSerial->write('.');
	m_pSerial->print(remote_ip & 0xff, 10);
	m_pSerial->print("\",");
	m_pSerial->println(remote_port, 10);
	return true;
}

bool Esp32::TCPConnect(IWifi* pWifi, uint32_t remote_ip, uint16_t remote_port)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_TCPCONNECT;
	m_pSerial->print("AT+CIPSTART=");
	m_pSerial->print("\"TCP\",\"");
	m_pSerial->print(remote_ip >> 24, 10);
	m_pSerial->write('.');
	m_pSerial->print((remote_ip >> 16) & 0xff, 10);
	m_pSerial->write('.');
	m_pSerial->print((remote_ip >> 8) & 0xff, 10);
	m_pSerial->write('.');
	m_pSerial->print(remote_ip & 0xff, 10);
	m_pSerial->print("\",");
	m_pSerial->println(remote_port, 10);
	return true;
}

bool Esp32::UDPConnectMUX(IWifi* pWifi, uint8_t link_id, uint32_t remote_ip, uint16_t remote_port, uint16_t local_port, int mode)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_UDPCONNECT;
	m_pSerial->print("AT+CIPSTART=");
	m_pSerial->write('0' + link_id);
	m_pSerial->print(",\"UDP\",\"");
	m_pSerial->print(remote_ip >> 24, 10);
	m_pSerial->write('.');
	m_pSerial->print((remote_ip >> 16) & 0xff, 10);
	m_pSerial->write('.');
	m_pSerial->print((remote_ip >> 8) & 0xff, 10);
	m_pSerial->write('.');
	m_pSerial->print(remote_ip & 0xff, 10);
	m_pSerial->print("\",");
	m_pSerial->print(remote_port, 10);
	if (local_port > 0)
	{
		m_pSerial->write(',');
		m_pSerial->print(local_port, 10);
		m_pSerial->write(',');
		m_pSerial->write('0' + mode);
	}
	m_pSerial->println();
	return true;
}

bool Esp32::UDPConnect(IWifi* pWifi, uint32_t remote_ip, uint16_t remote_port, uint16_t local_port, int mode)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_UDPCONNECT;
	m_pSerial->print("AT+CIPSTART=");
	m_pSerial->print("\"UDP\",\"");
	m_pSerial->print(remote_ip >> 24, 10);
	m_pSerial->write('.');
	m_pSerial->print((remote_ip >> 16) & 0xff, 10);
	m_pSerial->write('.');
	m_pSerial->print((remote_ip >> 8) & 0xff, 10);
	m_pSerial->write('.');
	m_pSerial->print(remote_ip & 0xff, 10);
	m_pSerial->print("\",");
	m_pSerial->print(remote_port, 10);
	if (local_port > 0)
	{
		m_pSerial->write(',');
		m_pSerial->print(local_port, 10);
		m_pSerial->write(',');
		m_pSerial->write('0' + mode);
	}
	m_pSerial->println();
	return true;
}

bool Esp32::sendBytesMUX(IWifi* pWifi, uint8_t link_id, byte* buffer, size_t size, uint32_t remote_ip, uint16_t remote_port)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_SENDBYTES;
	m_sendBuffer = buffer;
	m_sendSize = size;
	m_pSerial->print("AT+CIPSEND=");
	m_pSerial->write('0' + link_id);
	m_pSerial->write(',');
	m_pSerial->print(size, 10);
	if (remote_ip != 0 && remote_port != 0)
	{
		m_pSerial->print(",\"");
		m_pSerial->print(remote_ip >> 24, 10);
		m_pSerial->write('.');
		m_pSerial->print((remote_ip >> 16) & 0xff, 10);
		m_pSerial->write('.');
		m_pSerial->print((remote_ip >> 8) & 0xff, 10);
		m_pSerial->write('.');
		m_pSerial->print(remote_ip & 0xff, 10);
		m_pSerial->print("\",");
		m_pSerial->print(remote_port, 10);
	}
	m_pSerial->println();
	return true;
}

bool Esp32::sendBytes(IWifi* pWifi, byte* buffer, size_t size, uint32_t remote_ip, uint16_t remote_port)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_SENDBYTES;
	m_sendBuffer = buffer;
	m_sendSize = size;
	m_pSerial->print("AT+CIPSEND=");
	m_pSerial->print(size, 10);
	if (remote_ip != 0 && remote_port != 0)
	{
		m_pSerial->print(",\"");
		m_pSerial->print(remote_ip >> 24, 10);
		m_pSerial->write('.');
		m_pSerial->print((remote_ip >> 16) & 0xff, 10);
		m_pSerial->write('.');
		m_pSerial->print((remote_ip >> 8) & 0xff, 10);
		m_pSerial->write('.');
		m_pSerial->print(remote_ip & 0xff, 10);
		m_pSerial->print("\",");
		m_pSerial->print(remote_port, 10);
	}
	m_pSerial->println();
	return true;
}

bool Esp32::sendStringMUX(IWifi* pWifi, uint8_t link_id, const char s[], uint32_t remote_ip, uint16_t remote_port)
{
	return false;
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_SENDSTRING;
	m_sendBuffer = (byte*)s;
	m_pSerial->print("AT+CIPSENDEX=");
	m_pSerial->write('0' + link_id);
	m_pSerial->write(',');
	m_pSerial->print(SEND_MAXSIZE, 10);
	if (remote_ip != 0 && remote_port != 0)
	{
		m_pSerial->print(",\"");
		m_pSerial->print(remote_ip >> 24, 10);
		m_pSerial->write('.');
		m_pSerial->print((remote_ip >> 16) & 0xff, 10);
		m_pSerial->write('.');
		m_pSerial->print((remote_ip >> 8) & 0xff, 10);
		m_pSerial->write('.');
		m_pSerial->print(remote_ip & 0xff, 10);
		m_pSerial->print("\",");
		m_pSerial->print(remote_port, 10);
	}
	m_pSerial->println();
	return true;
}

bool Esp32::sendString(IWifi* pWifi, const char s[], uint32_t remote_ip, uint16_t remote_port)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_SENDSTRING;
	m_sendBuffer = (byte*)s;
	m_pSerial->print("AT+CIPSENDEX=");
	m_pSerial->print(SEND_MAXSIZE, 10);
	if (remote_ip != 0 && remote_port != 0)
	{
		m_pSerial->print(",\"");
		m_pSerial->print(remote_ip >> 24, 10);
		m_pSerial->write('.');
		m_pSerial->print((remote_ip >> 16) & 0xff, 10);
		m_pSerial->write('.');
		m_pSerial->print((remote_ip >> 8) & 0xff, 10);
		m_pSerial->write('.');
		m_pSerial->print(remote_ip & 0xff, 10);
		m_pSerial->print("\",");
		m_pSerial->print(remote_port, 10);
	}
	m_pSerial->println();
	return true;
}

bool Esp32::closeConnect(IWifi* pWifi, uint8_t link_id)
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_CLOSECONNECT;
	m_pSerial->print("AT+CIPCLOSE=");
	m_pSerial->write('0' + link_id);
	m_pSerial->println();
	return true;
}

bool Esp32::DomainResolution(IWifi* pWifi, char domain[])
{
	if (m_busy)
	{
		return false;
	}
	m_busy = true;
	m_lastSendTime = g_ul_ms_ticks;
	m_pWifi = pWifi;
	m_lastCMD = CMD_DOMAIN;
	m_pSerial->print("AT+CIPDOMAIN=\"");
	m_pSerial->print(domain);
	m_pSerial->println("\"");
	return true;
}

void Esp32::doSend(void)
{
	m_lastSendTime = g_ul_ms_ticks;
	if (m_lastCMD == CMD_SENDSTRING)
	{
		m_lastCMD = CMD_DOSEND;
		size_t size = strnlen((const char*)m_sendBuffer, SEND_MAXSIZE);
		wdt_restart(WDT);
		m_pSerial->write(m_sendBuffer, size);
	}
	else if (m_lastCMD == CMD_SENDBYTES)
	{
		m_lastCMD = CMD_DOSEND;
		wdt_restart(WDT);
		m_pSerial->write(m_sendBuffer, m_sendSize);
	}
}

void Esp32::parseReceived(void)
{
	while (true)
	{
		int len = m_rxBuffer.length();
		if (len == 0)
			break;
		if (processNetworkData()) // if matched command but not enough data
			break;
		if (processDisconnectAP()) // if matched command but not enough data
			break;
		if (processGetIP()) // if matched command but not enough data
			break;
		if (processGetAPIP()) // if matched command but not enough data
			break;
		if (processGetSTAIP()) // if matched command but not enough data
			break;
		if (processDomainResolution()) // if matched command but not enough data
			break;
		if (processSend()) // if matched command but not enough data
			break;
		if (processSendEnd()) // if matched command but not enough data
			break;
		if (processReset()) // if matched command but not enough data
			break;
		if (processBusy()) // if matched command but not enough data
			break;
		if (processError()) // if matched command but not enough data
			break;
		if (processOK()) // if matched command but not enough data
			break;
		if (len == m_rxBuffer.length()) // if not processed
		{
			int index = m_rxBuffer.find_word(WORD_CRLF);
			if (index != -1)
				m_rxBuffer.cut(index + 2);
			else if (m_rxBuffer.is_full())
			{
				m_rxBuffer.clear();
			}
			else
				break;
		}
	}
}

bool Esp32::processNetworkData(void)
{
	int len = m_rxBuffer.length();
	// if receiving network data not end
	if (m_rxDataRestSize > 0)
	{
		if (m_rxDataRestSize > len)
		{
			if (m_pWifi != NULL)
				m_pWifi->cbReceivedData(m_rxDataLinkID, m_rxBuffer, 0, len);
			m_rxBuffer.clear();
			m_rxDataRestSize -= len;
			return true;
		}
		else
		{
			if (m_pWifi != NULL)
				m_pWifi->cbReceivedData(m_rxDataLinkID, m_rxBuffer, 0, m_rxDataRestSize);
			m_rxBuffer.cut(m_rxDataRestSize);
			m_rxDataRestSize = 0;
		}
	}
	while (true)
	{
		strip();
		len = m_rxBuffer.length();
		int index = m_rxBuffer.find_word(WORD_CRLF);
		// if received network data
		if (m_rxBuffer.cmp_bytes((byte*)"+IPD,", 5))
		{
			bool hasError = false;
			int offset = (int)m_isMUX << 1;
			if (len < 7 + offset && index == -1) // if command not end, wait for more data
				return true;
			int index1 = m_rxBuffer.find_byte(':', 6 + offset, index); // get the start sign of the network data
			if (index1 != -1)
			{
				int link_id;
				if (m_isMUX) // if multi-connection mode
				{
					link_id = m_rxBuffer.read_byte(5) - '0';
				}
				else
				{
					link_id = 0;
				}
				if (link_id >= 0 && link_id <= 4) // if link_id in the correct range
				{
					int index2 = m_rxBuffer.find_byte(',', 6 + offset, index1);
					if (index2 == -1 || index2 > index1)
					{
						index2 = index1;
					}
					int n_len = index2 - 5 - offset;
					if (n_len > 0 && n_len < 6) // data length should be less than 6 figures
					{
						char s[6];
						int c = m_rxBuffer.read_bytes((byte*)s, n_len, 5 + offset);
						s[c] = 0;
						int n = atoi(s);
						if (len < index1 + n + 1)
						{
							if (m_pWifi != NULL)
							{
								m_pWifi->cbReceivedData(link_id, m_rxBuffer, index1 + 1, len);
							}
							m_rxDataLinkID = link_id;
							m_rxDataRestSize = n - (len - (index1 + 1));
							m_rxBuffer.clear();
							return true;
						}
						else if (n > 0)
						{
							if (m_pWifi != NULL)
								m_pWifi->cbReceivedData(link_id, m_rxBuffer, index1 + 1, index1 + 1 + n);
							m_rxBuffer.cut(index1 + 1 + n);
							continue;
						}
						else
							hasError = true;
					}
					else
						hasError = true;
				}
				else
					hasError = true;
			}
			else if (len > 43)
			{
				hasError = true;
			}
			else
				return true;

			if (index != -1)
			{
				m_rxBuffer.cut(index + 2);
				continue;
			}
			else
			{
				m_rxBuffer.clear();
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	return false;
}

bool Esp32::processDisconnectAP(void)
{
	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"WIFI DISCONNECT\r\n", 17))
	{
		if (m_pWifi != NULL)
			m_pWifi->cbDisconnectAP();
		m_rxBuffer.cut(17);
	}
	return false;
}

bool Esp32::processGetIP(void)
{
	if (m_lastCMD != CMD_GETIP)
		return false;
	while (true)
	{
		strip();
		bool isAPIP = m_rxBuffer.cmp_bytes((byte*)"+CIFSR:APIP,\"", 13);
		bool isSTAIP = m_rxBuffer.cmp_bytes((byte*)"+CIFSR:STAIP,\"", 14);
		if (!(isAPIP || isSTAIP))
			break;
		int len = m_rxBuffer.length();
		int index = m_rxBuffer.find_word(WORD_CRLF);
		int offset = (int)isSTAIP;
		if (len < 30 + offset && index == -1) // if command not end, wait more data
			return true;
		else if (index == -1) // if line too long and no CRLF, clear incorrect data
		{
			if (m_pWifi != NULL)
			{
				m_pWifi->cbGetIP(RESPONSE_UNKNOWN_ERROR, 0, 0);
				WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
			}
			m_rxBuffer.clear();
			m_lastCMD = CMD_NONE;
			m_busy = false;
			return false;
		}
		else if (index > 31 + offset) // if line too long with CRLF, cut incorrect data
		{
			if (m_pWifi != NULL)
			{
				m_pWifi->cbGetIP(RESPONSE_UNKNOWN_ERROR, 0, 0);
				WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
			}
			m_rxBuffer.cut(index + 2); // cut this line
			m_lastCMD = CMD_NONE;
			m_busy = false;
			break;
		}
		int p1, p2, p3, p4;
		char s[18];
		int c = m_rxBuffer.read_bytes((byte*)s, 17, 13 + offset);
		s[c] = 0;
		int n = sscanf(s, "%d.%d.%d.%d", &p1, &p2, &p3, &p4);
		if (n < 4)
		{
			m_rxBuffer.cut(index + 2);
			continue;
		}
		uint32_t ip = ((uint32_t)p1 << 24) | ((uint32_t)p2 << 16) | ((uint32_t)p3 << 8) | (uint32_t)p4;
		if (isAPIP)
			m_rxGetAPIP = ip;
		else
			m_rxGetSTAIP = ip;
		m_rxBuffer.cut(index + 2); // processed, cut this line
	}

	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"OK\r\n", 4))
	{
		if (m_pWifi != NULL)
			m_pWifi->cbGetIP(RESPONSE_OK, m_rxGetAPIP, m_rxGetSTAIP);
		m_rxBuffer.cut(4); // cut this line
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	return false;
}

bool Esp32::processGetAPIP(void)
{
	if (m_lastCMD != CMD_GETAPIP)
		return false;
	while (true)
	{
		strip();
		int len = m_rxBuffer.length();
		bool isIP = false, isMask = false, isGateway = false;
		if (len < 11)
			break;
		else if (m_rxBuffer.cmp_bytes((byte*)"+CIPAP:ip:\"", 11))
			isIP = true;
		else if (m_rxBuffer.cmp_bytes((byte*)"+CIPAP:netmask:\"", 16))
			isMask = true;
		else if (m_rxBuffer.cmp_bytes((byte*)"+CIPAP:gateway:\"", 16))
			isGateway = true;
		if (!(isIP || isMask || isGateway))
			break;
		int index = m_rxBuffer.find_word(WORD_CRLF);
		if (isGateway) // if gateway, ignore
		{
			if (index != -1)
			{
				m_rxBuffer.cut(index + 2); // cut this line
				continue;
			}
			else
				return true;
		}
		int offset = (int)isMask + ((int)isMask << 2);
		if (len < 29 + offset && index == -1) // if command not end, wait more data
			return true;
		else if (index == -1) // if line too long and no CRLF, clear incorrect data
		{
			if (m_pWifi != NULL)
			{
				m_pWifi->cbGetAPIP(RESPONSE_UNKNOWN_ERROR, 0, 0);
				WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
			}
			m_rxBuffer.clear();
			m_lastCMD = CMD_NONE;
			m_busy = false;
			return false;
		}
		else if (index > 27 + offset) // if line too long with CRLF, cut incorrect data
		{
			if (m_pWifi != NULL)
			{
				m_pWifi->cbGetAPIP(RESPONSE_UNKNOWN_ERROR, 0, 0);
				WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
			}
			m_rxBuffer.cut(index + 2); // cut this line
			m_lastCMD = CMD_NONE;
			m_busy = false;
			continue;
		}
		int p1, p2, p3, p4;
		char s[18];
		int c = m_rxBuffer.read_bytes((byte*)s, 15, 11 + offset);
		s[c] = 0;
		int n = sscanf(s, "%d.%d.%d.%d", &p1, &p2, &p3, &p4);
		if (n < 4)
		{
			m_rxBuffer.cut(index + 2);
			continue;
		}
		uint32_t ip = ((uint32_t)p1 << 24) | ((uint32_t)p2 << 16) | ((uint32_t)p3 << 8) | (uint32_t)p4;
		if (isIP)
			m_rxGetAPIP = ip;
		else
			m_rxGetAPMask = ~ip;
		m_rxBuffer.cut(index + 2); // processed, cut this line
	}

	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"OK\r\n", 4))
	{
		if (m_pWifi != NULL)
			m_pWifi->cbGetAPIP(RESPONSE_OK, m_rxGetAPIP, m_rxGetAPMask);
		m_rxBuffer.cut(4); // cut this line
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	return false;
}

bool Esp32::processGetSTAIP(void)
{
	if (m_lastCMD != CMD_GETSTAIP)
		return false;
	while (true)
	{
		strip();
		int len = m_rxBuffer.length();
		bool isIP = false, isMask = false, isGateway = false;
		if (len < 11)
			break;
		else if (m_rxBuffer.cmp_bytes((byte*)"+CIPSTA:ip:\"", 12))
			isIP = true;
		else if (m_rxBuffer.cmp_bytes((byte*)"+CIPSTA:netmask:\"", 17))
			isMask = true;
		else if (m_rxBuffer.cmp_bytes((byte*)"+CIPSTA:gateway:\"", 17))
			isGateway = true;
		if (!(isIP || isMask || isGateway))
			break;
		int index = m_rxBuffer.find_word(WORD_CRLF);
		if (isGateway) // if gateway, ignore
		{
			if (index != -1)
			{
				m_rxBuffer.cut(index + 2); // cut this line
				continue;
			}
			else
				return true;
		}
		int offset = (int)isMask + ((int)isMask << 2);
		if (len < 30 + offset && index == -1) // if command not end, wait more data
			return true;
		else if (index == -1) // if line too long and no CRLF, clear incorrect data
		{
			if (m_pWifi != NULL)
			{
				m_pWifi->cbGetSTAIP(RESPONSE_UNKNOWN_ERROR, 0, 0);
				WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
			}
			m_rxBuffer.clear();
			m_lastCMD = CMD_NONE;
			m_busy = false;
			return false;
		}
		else if (index > 28 + offset) // if line too long with CRLF, cut incorrect data
		{
			if (m_pWifi != NULL)
			{
				m_pWifi->cbGetSTAIP(RESPONSE_UNKNOWN_ERROR, 0, 0);
				WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
			}
			m_rxBuffer.cut(index + 2); // cut this line
			m_lastCMD = CMD_NONE;
			m_busy = false;
			break;
		}
		int p1, p2, p3, p4;
		char s[18];
		int c = m_rxBuffer.read_bytes((byte*)s, 15, 12 + offset);
		s[c] = 0;
		int n = sscanf(s, "%d.%d.%d.%d", &p1, &p2, &p3, &p4);
		if (n < 4)
		{
			m_rxBuffer.cut(index + 2);
			continue;
		}
		uint32_t ip = ((uint32_t)p1 << 24) | ((uint32_t)p2 << 16) | ((uint32_t)p3 << 8) | (uint32_t)p4;
		if (isIP)
			m_rxGetSTAIP = ip;
		else
			m_rxGetSTAMask = ~ip;
		m_rxBuffer.cut(index + 2); // processed, cut this line
	}

	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"OK\r\n", 4))
	{
		if (m_pWifi != NULL)
			m_pWifi->cbGetSTAIP(RESPONSE_OK, m_rxGetSTAIP, m_rxGetSTAMask);
		m_rxBuffer.cut(4); // cut this line
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	return false;
}

bool Esp32::processDomainResolution(void)
{
	if (m_lastCMD != CMD_DOMAIN)
		return false;
	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"+CIPDOMAIN:", 11))
	{
		int len = m_rxBuffer.length();
		int index = m_rxBuffer.find_word(WORD_CRLF);
		if (len < 28 && index == -1) // if command not end, wait more data
			return true;
		else if (index == -1) // if line too long and no CRLF, clear incorrect data
		{
			if (m_pWifi != NULL)
			{
				m_pWifi->cbDomainResolution(RESPONSE_UNKNOWN_ERROR, 0);
				WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
			}
			m_rxBuffer.clear();
			m_lastCMD = CMD_NONE;
			m_busy = false;
			return false;
		}
		else if (index > 26) // if line too long with CRLF, cut incorrect data
		{
			if (m_pWifi != NULL)
			{
				m_pWifi->cbDomainResolution(RESPONSE_UNKNOWN_ERROR, 0);
				WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
			}
			m_rxBuffer.cut(index + 2); // cut this line
			m_lastCMD = CMD_NONE;
			m_busy = false;
			return false;
		}
		int p1, p2, p3, p4;
		char s[18];
		int c = m_rxBuffer.read_bytes((byte*)s, 17, 11);
		s[c] = 0;
		int n = sscanf(s, "%d.%d.%d.%d", &p1, &p2, &p3, &p4);
		if (n < 4)
		{
			if (m_pWifi != NULL)
			{
				m_pWifi->cbDomainResolution(RESPONSE_UNKNOWN_ERROR, 0);
				WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
			}
			m_rxBuffer.cut(index + 2);
			m_lastCMD = CMD_NONE;
			m_busy = false;
			return false;
		}
		uint32_t ip = ((uint32_t)p1 << 24) | ((uint32_t)p2 << 16) | ((uint32_t)p3 << 8) | (uint32_t)p4;
		if (m_pWifi != NULL)
		{
			m_pWifi->cbDomainResolution(RESPONSE_OK, ip);
		}
		m_rxBuffer.cut(index + 2); // processed, cut this line
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	else if (m_rxBuffer.cmp_bytes((byte*)"ERROR\r\n", 7))
	{
		if (m_pWifi != NULL)
		{
			m_pWifi->cbDomainResolution(RESPONSE_DOMAIN_FAIL, 0);
			WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
		}
		m_rxBuffer.cut(7);
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	return false;
}

bool Esp32::processSend(void)
{
	if (m_lastCMD != CMD_SENDBYTES && m_lastCMD != CMD_SENDSTRING)
		return false;
	strip();
	if (m_rxBuffer[0] == '>')
	{
		doSend();
		m_rxBuffer.cut(1);
	}
	return false;
}

bool Esp32::processSendEnd(void)
{
	if (m_lastCMD != CMD_DOSEND)
		return false;
	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"SEND OK\r\n", 9))
	{
		if (m_pWifi != NULL)
			m_pWifi->cbSend(RESPONSE_OK);
		m_rxBuffer.cut(9);
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	else if (m_rxBuffer.cmp_bytes((byte*)"SEND FAIL\r\n", 11))
	{
		if (m_pWifi != NULL)
		{
			m_pWifi->cbSend(RESPONSE_SEND_FAILED);
			WIFI_DEBUG_printf("\r\FAILED %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
		}
		m_rxBuffer.cut(11);
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	else if (m_rxBuffer.cmp_bytes((byte*)"ERROR\r\n", 7))
	{
		if (m_pWifi != NULL)
		{
			m_pWifi->cbSend(RESPONSE_SEND_ERROR);
			WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
		}
		m_rxBuffer.cut(7);
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	return false;
}

bool Esp32::processScanAP(void)
{
	if (m_lastCMD != CMD_SCANAP)
		return false;
	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"+CWLAP:", 7))
	{
		int len = m_rxBuffer.length();
		int index = m_rxBuffer.find_word(WORD_CRLF);
		if (index == -1) // if command not end, wait more data
			return true;
		m_apFound = true;
		m_rxBuffer.cut(index + 2);
	}
	return false;
}

bool Esp32::processConnectAP(void)
{
	if (m_lastCMD != CMD_CONNECTAP)
		return false;
	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"WIFI GOT IP\r\n\r\nOK\r\n", 19))
	{
		if (m_pWifi != NULL)
			m_pWifi->cbConnectAP(RESPONSE_OK);
		m_rxBuffer.cut(19);
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	else if (m_rxBuffer.cmp_bytes((byte*)"ERROR\r\n", 7))
	{
		if (m_pWifi != NULL)
		{
			m_pWifi->cbConnectAP(RESPONSE_CONNECTAP_FAIL);
			WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
		}
		m_rxBuffer.cut(7);
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	return false;
}

bool Esp32::processReset(void)
{
	if (m_lastCMD != CMD_RESET)
		return false;
	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"ready\r\n", 7))
	{
		if (m_pWifi != NULL)
			m_pWifi->cbReset(RESPONSE_OK);
		m_rxBuffer.cut(7);
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	return false;
}

bool Esp32::processBusy(void)
{
	if (m_lastCMD == CMD_DOSEND)
		return false;
	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"busy p...\r\n", 11))
	{
		if (m_pWifi != NULL)
		{
			responseStatus(RESPONSE_BUSY);
			WIFI_DEBUG_printf("\r\busy %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
		}
		m_rxBuffer.cut(11);
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	return false;
}

bool Esp32::processError(void)
{
	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"ERROR\r\n", 7))
	{
		if (m_pWifi != NULL)
		{
			responseStatus(RESPONSE_UNKNOWN_ERROR);
			WIFI_DEBUG_printf("\r\ERROR %s\t%u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime);
		}
		m_rxBuffer.cut(7);
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
	return false;
}

bool Esp32::processOK(void)
{
	strip();
	if (m_rxBuffer.cmp_bytes((byte*)"OK\r\n", 4))
	{
		if (m_pWifi != NULL)
		{
			// reset, geting ip, geting AP/STA ip, domain resolution, sending data and connecting AP are judged to be OK in a specific way
			switch (m_lastCMD)
			{
			case Esp32::CMD_SETMODE:
				m_pWifi->cbSetMode(RESPONSE_OK);
				m_lastCMD = CMD_NONE;
				m_busy = false;
				break;
			case Esp32::CMD_SETSOFTAP:
				m_pWifi->cbSetMode(RESPONSE_OK);
				m_lastCMD = CMD_NONE;
				m_busy = false;
				break;
			case Esp32::CMD_SETMUX:
				m_pWifi->cbSetMUX(RESPONSE_OK);
				m_lastCMD = CMD_NONE;
				m_busy = false;
				break;
			case Esp32::CMD_SCANAP:
				m_pWifi->cbScanAP(RESPONSE_OK, m_apFound);
				m_lastCMD = CMD_NONE;
				m_busy = false;
				break;
			case Esp32::CMD_AUTOCONN:
				m_pWifi->cbAutoConnAP(RESPONSE_OK);
				m_lastCMD = CMD_NONE;
				m_busy = false;
				break;
			case Esp32::CMD_CONNECTAP:
				m_pWifi->cbConnectAP(RESPONSE_OK);
				m_lastCMD = CMD_NONE;
				m_busy = false;
				break;
			case Esp32::CMD_UDPCONNECT:
				m_pWifi->cbUDPConnect(RESPONSE_OK);
				m_lastCMD = CMD_NONE;
				m_busy = false;
				break;
			default:
				break;
			}
		}
		m_rxBuffer.cut(4);
	}
	return false;
}

void Esp32::checkTimeout(void)
{
	uint32_t timeout = NORMAL_TIMEOUT;
	switch (m_lastCMD)
	{
	case Esp32::CMD_NONE:
		return;
	case Esp32::CMD_RESET: // reset will take more time
		timeout = RESET_TIMEOUT;
		break;
	case Esp32::CMD_SCANAP: // scaning ap will take more time
		timeout = SCANAP_TIMEOUT;
		break;
	case Esp32::CMD_CONNECTAP: // connecting ap will take more time
		timeout = CONNECTAP_TIMEOUT;
		break;
	default:
		break;
	}
	if (g_ul_ms_ticks - m_lastSendTime > timeout)
	{
		if (m_pWifi != NULL)
		{
			responseStatus(RESPONSE_TIMEOUT);
			WIFI_DEBUG_printf("\r\ntimeout %s\t%u %u %u\r\n", printCMD(m_lastCMD), g_ul_ms_ticks, m_lastSendTime, timeout);
		}
		m_lastCMD = CMD_NONE;
		m_busy = false;
	}
}

void Esp32::strip(void)
{
	// strip CR/LF at the start
	int len = m_rxBuffer.length();
	int i;
	for (i = 0; i < len; i++)
	{
		char c = m_rxBuffer[i];
		if (c != '\r' && c != '\n')
			break;
	}
	m_rxBuffer.cut(i);
}

void Esp32::responseStatus(ResponseType state)
{
	switch (m_lastCMD)
	{
	case Esp32::CMD_NONE:
		break;
	case Esp32::CMD_RESET:
		m_pWifi->cbReset(state);
		break;
	case Esp32::CMD_RECOVERY:
		break;
	case Esp32::CMD_SETMODE:
		m_pWifi->cbSetMode(state);
		break;
	case Esp32::CMD_SETSOFTAP:
		m_pWifi->cbSetSoftAP(state);
		break;
	case Esp32::CMD_CONNECTAP:
		m_pWifi->cbConnectAP(state);
		break;
	case Esp32::CMD_CONFIGSCANAP:
		break;
	case Esp32::CMD_SCANAP:
		m_pWifi->cbScanAP(state, m_apFound);
		break;
	case Esp32::CMD_AUTOCONN:
		m_pWifi->cbAutoConnAP(state);
		break;
	case Esp32::CMD_GETIP:
		m_pWifi->cbGetIP(state, 0, 0);
		break;
	case Esp32::CMD_GETAPIP:
		m_pWifi->cbGetAPIP(state, 0, 0);
		break;
	case Esp32::CMD_GETSTAIP:
		m_pWifi->cbGetSTAIP(state, 0, 0);
		break;
	case Esp32::CMD_GETNETSTATUS:
		break;
	case Esp32::CMD_SETMUX:
		m_pWifi->cbSetMUX(state);
		break;
	case Esp32::CMD_TCPSERVER:
		break;
	case Esp32::CMD_TCPSERVER_STOP:
		break;
	case Esp32::CMD_TCPCONNECT:
		break;
	case Esp32::CMD_UDPCONNECT:
		m_pWifi->cbUDPConnect(state);
		break;
	case Esp32::CMD_SENDBYTES:
		m_pWifi->cbSend(state);
		break;
	case Esp32::CMD_SENDSTRING:
		m_pWifi->cbSend(state);
		break;
	case Esp32::CMD_DOSEND:
		break;
	case Esp32::CMD_CLOSECONNECT:
		break;
	case Esp32::CMD_DOMAIN:
		m_pWifi->cbDomainResolution(state, 0);
		break;
	default:
		break;
	}
}

const char* Esp32::printCMD(CMDType cmd)
{
	switch (cmd)
	{
	case Esp32::CMD_NONE:
		return("CMD_NONE");
		break;
	case Esp32::CMD_RESET:
		return("CMD_RESET");
		break;
	case Esp32::CMD_RECOVERY:
		return("CMD_RECOVERY");
		break;
	case Esp32::CMD_SETMODE:
		return("CMD_SETMODE");
		break;
	case Esp32::CMD_SETSOFTAP:
		return("CMD_SETSOFTAP");
		break;
	case Esp32::CMD_CONNECTAP:
		return("CMD_CONNECTAP");
		break;
	case Esp32::CMD_CONFIGSCANAP:
		return("CMD_CONFIGSCANAP");
		break;
	case Esp32::CMD_SCANAP:
		return("CMD_SCANAP");
		break;
	case Esp32::CMD_GETIP:
		return("CMD_GETIP");
		break;
	case Esp32::CMD_GETAPIP:
		return("CMD_GETAPIP");
		break;
	case Esp32::CMD_GETSTAIP:
		return("CMD_GETSTAIP");
		break;
	case Esp32::CMD_GETNETSTATUS:
		return("CMD_GETNETSTATUS");
		break;
	case Esp32::CMD_SETMUX:
		return("CMD_SETMUX");
		break;
	case Esp32::CMD_TCPSERVER:
		return("CMD_TCPSERVER");
		break;
	case Esp32::CMD_TCPSERVER_STOP:
		return("CMD_TCPSERVER_STOP");
		break;
	case Esp32::CMD_TCPCONNECT:
		return("CMD_TCPCONNECT");
		break;
	case Esp32::CMD_UDPCONNECT:
		return("CMD_UDPCONNECT");
		break;
	case Esp32::CMD_SENDBYTES:
		return("CMD_SENDBYTES");
		break;
	case Esp32::CMD_SENDSTRING:
		return("CMD_SENDSTRING");
		break;
	case Esp32::CMD_DOSEND:
		return("CMD_DOSEND");
		break;
	case Esp32::CMD_CLOSECONNECT:
		return("CMD_CLOSECONNECT");
		break;
	case Esp32::CMD_DOMAIN:
		return("CMD_DOMAIN");
		break;
	default:
		return("CMD_error!!!!!");
		break;
	}
}

Esp32 esp32;
