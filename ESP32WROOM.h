// DigiX WiFi module example - released by Digistump LLC/Erik Kettenburg under CC-BY-SA 3.0


#ifndef _ESP32WROOM_h
#define _ESP32WROOM_h

#ifdef WIN32
#include "win32_test/win32_test.h"
#include "win32_test/serial_test.h"
#include <string.h>
#include <stdio.h>
#include "conf_wifi.h"
#else
#include "Arduino.h"    
#include "Print.h"
#include <string.h>
#include "conf_wifi.h"
#endif

#ifdef WIFI_DEBUG
#define WIFI_DEBUG_printf(...) printf(__VA_ARGS__)
#else
#define WIFI_DEBUG_printf(...) 
#endif

#define DIGIFI_RTS  57
#define DIGIFI_CTS  58

class IWifi;

class MyRingBuffer
{
public:
	const static int BUFFER_MAX_SIZE = FRAME_MAX_SIZE + 1;
	const static int FINDBYTES_MAX_SIZE = 20;
	uint8_t _aucBuffer[BUFFER_MAX_SIZE];
	int _iHead;
	int _iTail;

public:
	MyRingBuffer(void);
	void store_byte(uint8_t c);
	bool is_full(void);
	int find_byte(byte c, int begin = 0, int end = -1);
	int find_bytes(byte* p, size_t n);
	int find_word(uint16_t w, int begin = 0);
	bool cmp_bytes(byte* p, size_t n = 0, int begin = 0);
	uint8_t read_byte(int index);
	size_t read_bytes(uint8_t* buffer, size_t n, int begin = 0);
	void set_byte(int index, byte v);
	void set_bytes(int begin, int end, byte v);
	void cut(int count);
	int length(void);
	void clear(void);

	uint8_t operator[] (int index);
};

class Esp32
{
    public:
        Esp32();
		const static int SSID_MAX_LEN = 32;
		const static int PWD_MAX_LEN = 63;
		const static uint32_t NORMAL_TIMEOUT = 1000;
		const static uint32_t RESET_TIMEOUT = 10000;
		const static uint32_t SCANAP_TIMEOUT = 10000;
		const static uint32_t CONNECTAP_TIMEOUT = 30000;
		const static int SEND_MAXSIZE = 2048;
		const static uint16_t WORD_CRLF = '\r' + '\n' * 256;
		
		enum CMDType {
			CMD_NONE,
			CMD_RESET,
			CMD_RECOVERY,
			CMD_SETMODE,
			CMD_SETSOFTAP,
			CMD_CONNECTAP,
			CMD_CONFIGSCANAP,
			CMD_SCANAP,
			CMD_AUTOCONN,
			CMD_GETIP,
			CMD_GETAPIP,
			CMD_GETSTAIP,
			CMD_GETNETSTATUS,
			CMD_SETMUX,
			CMD_TCPSERVER,
			CMD_TCPSERVER_STOP,
			CMD_TCPCONNECT,
			CMD_UDPCONNECT,
			CMD_SENDBYTES,
			CMD_SENDSTRING,
			CMD_DOSEND,
			CMD_CLOSECONNECT,
			CMD_DOMAIN
		};
		enum ResponseType {
			RESPONSE_OK = 0,
			RESPONSE_TIMEOUT,
			RESPONSE_BUSY,
			RESPONSE_CONNECTAP_FAIL,
			RESPONSE_DOMAIN_FAIL,
			RESPONSE_SEND_READY,
			RESPONSE_SEND_ERROR,
			RESPONSE_SEND_FAILED,
			RESPONSE_UNKNOWN_ERROR
		};
		// for AT command AT+CWLAPOPT
		enum ScanAPMask {
			ECHO_ECN = 1,
			ECHO_SSID = 2,
			ECHO_RSSI = 4,
			ECHO_MAC = 8,
			ECHO_CHANNEL = 16
		};
		enum EncryptType{
			OPEN = 0,
			WEP = 1,
			WPA_PSK = 2,
			WPA2_PSK = 3,
			WPA_WPA2_PSK = 4
		} ;
		enum ConnType {
			TCP,
			UDP
		};
		typedef struct _CONN_INFO {
			uint8_t link_id;
			ConnType type;
			uint32_t remote_ip;
			uint16_t remote_port;
			uint16_t local_port;
			bool is_server;
		} ConnInfo;
        
		void loop(void);
		void init(void);
        void setSerial(USARTClass& hSerial, int aBaud = 115200, bool en = false);
		USARTClass& getSerial();

		bool reset(IWifi* pWifi);
		bool recovery(IWifi* pWifi);

		bool startSoftAP(IWifi* pWifi);
		bool startStation(IWifi* pWifi);
		bool startAPStation(IWifi* pWifi);
		bool setSoftAP(IWifi* pWifi, const char ssid[], const char pwd[], int ch, EncryptType ecn, int max_conn = 0, bool hidden = false);

		bool connectAP(IWifi* pWifi, const char ssid[], const char pwd[], const char *bssid = NULL);
		bool configSanAP(IWifi* pWifi, bool sort, uint8_t mask);
		bool scanAP(IWifi* pWifi, const char ssid[]);
		bool autoConnAP(IWifi* pWifi, bool isAuto); // switch on auto-connect-AP may cause scan-AP to fail
		bool getIP(IWifi* pWifi);
		bool getAPIP(IWifi* pWifi);
		bool getSTAIP(IWifi* pWifi);

		bool getNetStatus(IWifi* pWifi);
		bool setMUX(IWifi* pWifi, bool isMUX);
		bool startTCPServer(IWifi* pWifi, uint16_t port);
		bool stopTCPServer(IWifi* pWifi, uint16_t port);
		// for multi-connect mode
		bool TCPConnectMUX(IWifi* pWifi, uint8_t link_id, uint32_t remote_ip, uint16_t remote_port);
		// for single-connect mode
		bool TCPConnect(IWifi* pWifi, uint32_t remote_ip, uint16_t remote_port);
		// for multi-connect mode
		bool UDPConnectMUX(IWifi* pWifi, uint8_t link_id, uint32_t remote_ip, uint16_t remote_port, uint16_t local_port = 0, int mode = 0);
		// for single-connect mode
		bool UDPConnect(IWifi* pWifi, uint32_t remote_ip, uint16_t remote_port, uint16_t local_port = 0, int mode = 0);
		bool sendBytesMUX(IWifi* pWifi, uint8_t link_id, byte* buffer, size_t size, uint32_t remote_ip = 0, uint16_t remote_port = 0);
		bool sendBytes(IWifi* pWifi, byte* buffer, size_t size, uint32_t remote_ip = 0, uint16_t remote_port = 0);
		bool sendStringMUX(IWifi* pWifi, uint8_t link_id, const char s[], uint32_t remote_ip = 0, uint16_t remote_port = 0);
		bool sendString(IWifi* pWifi, const char s[], uint32_t remote_ip = 0, uint16_t remote_port = 0);
		bool closeConnect(IWifi* pWifi, uint8_t link_id);
		bool DomainResolution(IWifi* pWifi, char domain[]);
       
    private:
		MyRingBuffer m_rxBuffer;
        USARTClass* m_pSerial;
		uint32_t m_lastSendTime;
		bool m_busy;
		CMDType m_lastCMD;
		byte* m_sendBuffer;
		size_t m_sendSize;
		bool m_isMUX;
		int m_rxDataLinkID;
		size_t m_rxDataRestSize;
		uint32_t m_rxGetAPIP;
		uint32_t m_rxGetAPMask;
		uint32_t m_rxGetSTAIP;
		uint32_t m_rxGetSTAMask;
		bool m_apFound;
		IWifi* m_pWifi;

		void doSend(void);
		void parseReceived(void);
		void checkTimeout(void);
		bool processNetworkData(void); // NOTE: CRLF before "+IPD", none at the end
		bool processDisconnectAP(void);
		bool processGetIP(void);
		bool processGetAPIP(void);
		bool processGetSTAIP(void);
		bool processDomainResolution(void);
		bool processSend(void); // NOTE: if send too much data, "busy" will response before "SEND OK"
		bool processSendEnd(void);
		bool processScanAP(void);
		bool processConnectAP(void);
		bool processReset(void);
		bool processBusy(void);
		bool processError(void);
		bool processOK(void);
		void strip(void);
		void responseStatus(ResponseType state);

		const char* printCMD(CMDType cmd);
};

class IWifi
{
public:
	virtual void cbReset(Esp32::ResponseType) = 0;
	virtual void cbSetMode(Esp32::ResponseType) = 0;
	virtual void cbSetSoftAP(Esp32::ResponseType) = 0;
	virtual void cbAutoConnAP(Esp32::ResponseType) = 0;
	virtual void cbScanAP(Esp32::ResponseType, bool) = 0;
	virtual void cbConnectAP(Esp32::ResponseType) = 0;
	virtual void cbGetIP(Esp32::ResponseType, uint32_t AP_IP, uint32_t STA_IP) = 0;
	virtual void cbGetAPIP(Esp32::ResponseType, uint32_t ip, uint32_t mask) = 0;
	virtual void cbGetSTAIP(Esp32::ResponseType, uint32_t ip, uint32_t mask) = 0;
	virtual void cbGetNetStatus(Esp32::ResponseType, int link_id) = 0;
	virtual void cbSetMUX(Esp32::ResponseType) = 0;
	virtual void cbUDPConnect(Esp32::ResponseType) = 0;
	virtual void cbDomainResolution(Esp32::ResponseType, uint32_t ip) = 0;
	virtual void cbDisconnectAP(void) = 0;
	virtual void cbReceivedData(int link_id, MyRingBuffer&, int begin, int end) = 0;
	virtual void cbSend(Esp32::ResponseType) = 0;
};

extern Esp32 esp32;

#endif

