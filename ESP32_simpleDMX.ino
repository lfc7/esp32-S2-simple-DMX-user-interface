/*

 */
#include "FS.h"
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiAP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
//#include <Hash.h>
//#include <ESPmDNS.h>
#include "mdns_defines.h"
#include <MDNS_Generic.h>
#include <Update.h>
#include <WebSocketsServer.h>
#include <WebSocketsClient.h>
#include <esp_dmx.h>
#include <ArtnetWifi.h>
#include <elapsedMillis.h>
#include "ParseCommand.h" // parse serial command

// web pages
#include "index.h"

#include "manager_html.h"
#include "ok_html.h"
#include "failed_html.h"

#include "error_404.h"
#include "error_405.h"
 

#define SSID_DMXui	"DMXui-01" //should be different for each devices
#define PASS_DMXui	"DMXuiDMXui"
#define DNS_DMXui	"dmxui"
#define HTTP_PORT		80
#define WEBSOCKET_PORT	81

#define PING_INTERVAL				1000
#define PONG_TIMEOUT				3000
#define DISCONNECT_TIMEOUT_COUNT	2

#define	DMXTX_pin	16
#define	DMXRX_pin	17
#define	DMXDIR_pin	18

#define	DMXPORT		1

#define REFRESH_DMX_RATE	23 // in ms ; approx 44Hz

#define DMX_CH  		512
#define FADERS_MAX  	128
#define MEM_MAX			8
#define	FX_MAXNB		2

#define NB_FX			4

#define DISPLAY_REFRESH				10 // *20ms
#define	SEND_ALL_STATE_TO_WS_TIMER	200 // in ms

#define LED_PIN				LED_BUILTIN  // ESP32 pin connected to LED
#define	FLASH_TIMER 		800 //millis
#define	BLINK_TIMER			500 //millis

#define uS_TO_S_FACTOR		1000000ULL

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#define FORMAT_SPIFFS_IF_FAILED true

//~ //timer
//~ #define _TIMERINTERRUPT_LOGLEVEL_     3
//~ // To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
//~ #include "ESP32TimerInterrupt.h"
//~ ESP32Timer ITimer0(0);

//~ //interrupts
//~ portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
//~ volatile int interruptCounter;

File myFile;
WiFiUDP udp;
MDNS mdns(udp);
ParseCommand sCmd;
ArtnetWifi artnet;

typedef uint8_t   fract8;
typedef uint16_t  accum88;    ///< ANSI: unsigned short _Accum. 8 bits int, 8 bits fraction

template <class T> int FILE_writeAnything(File f, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    f.write(*p++);
  return i;
}

template <class T> int FILE_readAnything(File f, T& value)
{
  byte* p = (byte*)(void*)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value) && f.available(); i++)
    *p++ = f.read();
  return i;
}

//WIFI *****************************************************************
/* AP IP Address details */
IPAddress AP_local_ip(10,0,0,1);
IPAddress AP_gateway(10,0,0,1);
IPAddress AP_subnet(255,255,255,0);

struct cli_s{
	char ssid[48];
	char password[48];
};
cli_s client;

RTC_DATA_ATTR int flag_wifi_client=false;

//web spiffs ************************
String filesDropdownOptions = "";
String savePath = "";
String savePathInput = "";
bool rebooting = false;

//DMX  *******************************
dmx_port_t dmxPort = DMXPORT;
uint8_t   dmxTxBuffer[DMX_CH] = {0};
elapsedMillis refresh_DMX_rate=0;

uint8_t	master=255;
bool flag_black_all=false;

struct dimmer{
	uint8_t value=0;
	uint16_t patch=0;
	uint8_t	max_val=255;
	uint8_t	type=0;
};
dimmer dimmers[DMX_CH];

void reset_dimmers()
{
	for(int i=0; i< DMX_CH;i++)
	{
		dimmers[i].value=0;
		dimmers[i].patch=0;
		dimmers[i].max_val=255;
		dimmers[i].type=0;
	}
}

struct fader{
	uint8_t value=0;
	bool	fx[NB_FX];
	char	name[9]={0};
};
fader faders[FADERS_MAX];

struct fx_settings{
	uint8_t type=0;
	uint8_t speed=60;
	uint8_t	max_val=255;
	uint8_t min_val=0;
};
fx_settings	fxs[FX_MAXNB];

struct scene{
	uint8_t value=0;
	fader	faders_memstate[FADERS_MAX];
	bool	fx[NB_FX];
	char	name[9]={0};
};
scene scenes[MEM_MAX];

enum {
	CLD, //clear dimmers
	FULL, // full dimmers
	BLACK, // black master
	CLF, // clear faders
	CLS, // clear scenes
	PATCH // patch 1:1
};

enum {
	DIM,
	ONOFF,
	QUADHI,
	QUADLO,
	INVDIM
};

enum {
	FXA,
	INV_FXA,
	FXB,
	INV_FXB
};

enum{
	FX_TYPE,
	FX_BEAT,
	FX_MIN,
	FX_MAX
};

enum{
	FX_DIM,
	FX_STROBE,
	FX_SIN,
	FX_SAW
};

enum {
	WS_DMX,
	WS_DIMVAL,
	WS_DIMMAX,
	WS_DIMPTCH,
	WS_DIMTYPE,
	WS_FADVAL,
	WS_FADFX,
	WS_FADNAME,
	WS_MASTVAL,
	WS_FXVAL,
	WS_SCNVAL,
	WS_SCNNAME,
	WS_BLACK
};

//ARTNET***********************************************************
struct artnet_s{
	bool enable=false;
	uint16_t universe=0;
};
artnet_s artnetDMX;

uint32_t artnetLastSeq=0;

//UI **************************************************************
uint8_t ui_faders_page_state=1;
uint16_t ui_faders_by_page=16;
uint16_t ui_faders_page_lw_ch=1;
uint16_t ui_faders_page_hi_ch=16;
char ui_faders_html[30];

bool flag_continuous_refresh_vu_ui=false;
elapsedMillis refresh_vu_ui_rate=0;

//speed test dmx frame***********************
uint32_t	speedTest_res=0;
int		dmxRateLPF=0;

//deep sleep *******************************************
esp_sleep_wakeup_cause_t wakeup_reason;

//LED ***************************************************
elapsedMillis	flash_led_timer, blink_led_timer;
bool			flag_flash_led, flag_blink_led;

//serial debug flag *********************************
boolean debug_flag = false;

//BeatGenerators *****************************

/// Pre-calculated lookup table used in sin8() and cos8() functions
const uint8_t b_m16_interleave[] = { 0, 49, 49, 41, 90, 27, 117, 10 };

uint8_t squarewave8( uint8_t in, uint8_t pulsewidth=128)
{
    if( in < pulsewidth || (pulsewidth == 255)) {
        return 255;
    } else {
        return 0;
    }
}

uint8_t  sin8( uint8_t theta)
{
    uint8_t offset = theta;
    if( theta & 0x40 ) {
        offset = (uint8_t)255 - offset;
    }
    offset &= 0x3F; // 0..63

    uint8_t secoffset  = offset & 0x0F; // 0..15
    if( theta & 0x40) ++secoffset;

    uint8_t section = offset >> 4; // 0..3
    uint8_t s2 = section * 2;
    const uint8_t* p = b_m16_interleave;
    p += s2;
    uint8_t b   =  *p;
    ++p;
    uint8_t m16 =  *p;

    uint8_t mx = (m16 * secoffset) >> 4;

    int8_t y = mx + b;
    if( theta & 0x80 ) y = -y;

    y += 128;

    return y;
}

/// Generates a 16-bit "sawtooth" wave at a given BPM, with BPM
/// specified in Q8.8 fixed-point format.
/// @param beats_per_minute_88 the frequency of the wave, in Q8.8 format
/// @param timebase the time offset of the wave from the millis() timer
/// @warning The BPM parameter **MUST** be provided in Q8.8 format! E.g.
/// for 120 BPM it would be 120*256 = 30720. If you just want to specify
/// "120", use beat16() or beat8().
uint16_t beat88( accum88 beats_per_minute_88, uint32_t timebase = 0)
{
    // BPM is 'beats per minute', or 'beats per 60000ms'.
    // To avoid using the (slower) division operator, we
    // want to convert 'beats per 60000ms' to 'beats per 65536ms',
    // and then use a simple, fast bit-shift to divide by 65536.
    //
    // The ratio 65536:60000 is 279.620266667:256; we'll call it 280:256.
    // The conversion is accurate to about 0.05%, more or less,
    // e.g. if you ask for "120 BPM", you'll get about "119.93".
    return (((millis()) - timebase) * beats_per_minute_88 * 280) >> 16;
}

/// Generates a 16-bit "sawtooth" wave at a given BPM
/// @param beats_per_minute the frequency of the wave, in decimal
/// @param timebase the time offset of the wave from the millis() timer
uint16_t beat16( accum88 beats_per_minute, uint32_t timebase = 0)
{
    // Convert simple 8-bit BPM's to full Q8.8 accum88's if needed
    if( beats_per_minute < 256) beats_per_minute <<= 8;
    return beat88(beats_per_minute, timebase);
}

/// Generates an 8-bit "sawtooth" wave at a given BPM
/// @param beats_per_minute the frequency of the wave, in decimal
/// @param timebase the time offset of the wave from the millis() timer
uint8_t beat8( accum88 beats_per_minute, uint32_t timebase = 0)
{
    return beat16( beats_per_minute, timebase) >> 8;
}

/// Generates an 8-bit sine wave at a given BPM that oscillates within
/// a given range.
/// @param beats_per_minute the frequency of the wave, in decimal
/// @param lowest the lowest output value of the sine wave
/// @param highest the highest output value of the sine wave
/// @param timebase the time offset of the wave from the millis() timer
/// @param phase_offset phase offset of the wave from the current position
uint8_t beatsin8( accum88 beats_per_minute, uint8_t lowest = 0, uint8_t highest = 255, uint32_t timebase = 0, uint8_t phase_offset = 0)
{
    uint8_t beat = beat8( beats_per_minute, timebase);
    uint8_t beatsin = sin8( beat + phase_offset);
    uint8_t rangewidth = highest - lowest;
    uint8_t scaledbeat = scale8( beatsin, rangewidth);
    uint8_t result = lowest + scaledbeat;
    return result;
}

/// @} BeatGenerators

/// @} lib8tion, to exclude timekeeping functions

uint8_t scale8( uint8_t i, fract8 scale)
{
  return (((uint16_t)i) * (1+(uint16_t)(scale))) >> 8;
}

/*
//websocket client (slave mode) ***************************************
WebSocketsClient webSocket_client;
void initWebSocketClient()
{
	// server address, port and URL
	webSocket_client.begin("192.168.0.123", 81, "/vu.html");

	// event handler
	webSocket_client.onEvent(webSocketEvent_client);
	
}

void webSocketEvent_client(WStype_t type, uint8_t* payload, size_t length) 
{
  switch (type) {
	  
    case WStype_DISCONNECTED :
      if(debug_flag)Serial.printf("[INFO] disconnected from master\n");
    break;

    case WStype_CONNECTED :
      if(debug_flag)Serial.printf("[INFO]  Connected to master\n");
    break;

   case WStype_BIN:
      if(debug_flag)Serial.printf("[INFO] Received bin: %s\n");
      if(payload[0]==WS_DMX)
      {
		for (int i = 0; i < DMX_CH; i++) 
		{
			if(i < length - 1)
			{
				dmxTxBuffer[i] = payload[i + 1];  
			}else{
				dmxTxBuffer[i] = 0;
			}
		}
	  }
       
    break;
  }
}
//UDP slave ************************************************************
WiFiUDP UdpSlave;
void initUdpSlave()
{
	UdpSlave.begin(DMX2UDPport);
}

void check_udp()
{
	Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  // Just test touch pin - Touch0 is T0 which is on GPIO 4.
  Udp.printf(String(touchRead(T0)).c_str(),2);
  Udp.endPacket();
}

*/

//WEB_SOCKET_SERVER ****************************************************
AsyncWebServer server(HTTP_PORT);

WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);  // WebSocket server on port 81

char WSstrbuff[80];

void initWebSocket()
{
  // Initialize WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  //webSocket.enableHeartbeat(uint32_t pingInterval, uint32_t pongTimeout, uint8_t disconnectTimeoutCount);
  webSocket.enableHeartbeat(PING_INTERVAL, PONG_TIMEOUT, DISCONNECT_TIMEOUT_COUNT);

  delay(500);
  if(debug_flag)
  {
	Serial.print("[BOOT] WebSocket server's port: ");
	Serial.println(WEBSOCKET_PORT);
	Serial.flush();
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) 
{
  IPAddress ip_WS;
  switch (type) {
	  
    case WStype_DISCONNECTED :
      //blink_led(true);
      cmd_unlock_refresh_vu_ui();
      if(debug_flag)Serial.printf("[INFO] [%u] Disconnected!\n", num);
    break;

    case WStype_CONNECTED :
      ip_WS = webSocket.remoteIP(num);
      if(debug_flag)Serial.printf("[INFO] [%u] Connected from %d.%d.%d.%d\n", num, ip_WS[0], ip_WS[1], ip_WS[2], ip_WS[3]);

      // Send a response back to the client
      snprintf(WSstrbuff,ARRAY_SIZE(WSstrbuff),"Connected from %d.%d.%d.%d\n", ip_WS[0], ip_WS[1], ip_WS[2], ip_WS[3]);
      webSocket.sendTXT(num, WSstrbuff);

      //blink_led(false);
      //set_led(false);
    break;

   case WStype_TEXT :
      if(debug_flag)Serial.printf("[INFO] [%u] Received text: %s\n", num, payload);

      sCmd.readCommand((const char*)payload);
      //flash_led();

      // Send a response back to the client
      //webSocket.sendTXT(num, "Received:  " + String((char*)payload));

      // Send a response back to ALL the clients
      //webSocket.broadcastTXT("OK " + String((char*)payload));
 
    break;
  }
}

void WSui_SendDmxState()
{
	uint8_t data[DMX_CH+1];
	uint8_t * bytePtr = (uint8_t*) &data; 
	
	//send dmx view***********************
	data[0]=WS_DMX; //send DMX 
	memmove(bytePtr + 1, dmxTxBuffer, ARRAY_SIZE(dmxTxBuffer));
	webSocket.broadcastBIN(data, sizeof(data));
}

void WSui_SendDimmersState()
{
	uint8_t data[DMX_CH+1];
	uint8_t * bytePtr = (uint8_t*) &data; 
	
	/*
	//send dmx view***********************
	data[0]=WS_DMX; //send DMX 
	memmove(bytePtr + 1, dmxTxBuffer, ARRAY_SIZE(dmxTxBuffer));    
	webSocket.broadcastBIN(data, sizeof(data));
	* */
	
	//send dimmers states***********************
	data[0]=WS_DIMVAL; //send dimmers id 
	for(int d=0; d < DMX_CH; d++)
	{
		data[d+1]=dimmers[d].value;
	}
	webSocket.broadcastBIN(data, sizeof(data));
	
	data[0]=WS_DIMMAX; //send dimmers max 
	for(int d=0; d < DMX_CH; d++)
	{
		data[d+1]=dimmers[d].max_val;
	}
	webSocket.broadcastBIN(data, sizeof(data));
	
	data[0]=WS_DIMPTCH; //send dimmers patch 
	for(int d=0; d < DMX_CH; d++)
	{
		data[d+1]=(uint8_t)dimmers[d].patch;
	}
	webSocket.broadcastBIN(data, sizeof(data));
	
	data[0]=WS_DIMTYPE; //send dimmers type 
	for(int d=0; d < DMX_CH; d++)
	{
		data[d+1]=(uint8_t)dimmers[d].type;
	}
	webSocket.broadcastBIN(data, sizeof(data));
	
	data[0]=WS_BLACK; //send blk type 
	data[1]=(uint8_t)flag_black_all;
	webSocket.broadcastBIN(data, 2);

	if(debug_flag)Serial.println("send dimmers state");
	
}

void WSui_SendFadersState()
{
	uint8_t data[DMX_CH+1];
	uint8_t * bytePtr = (uint8_t*) &data; 
	
	/*
	//send dmx view***********************
	data[0]=WS_DMX; //send DMX 
	memcpy(bytePtr + 1, dmxTxBuffer, ARRAY_SIZE(dmxTxBuffer));    
	webSocket.broadcastBIN(data, sizeof(data));
	* */
	
	//send faders states***********************
	data[0]=WS_FADVAL; //send faders val
	for(int d=0; d < FADERS_MAX; d++)
	{
		data[d+1]=faders[d].value;
	}
	webSocket.broadcastBIN(data, FADERS_MAX + 1);
	
	data[0]=WS_FADFX; //send faders fx 
	for(int d=0; d < FADERS_MAX; d++)
	{
		data[d+1]=faders[d].fx[3] + faders[d].fx[2] * 2 + faders[d].fx[1] * 4 + faders[d].fx[0] * 8;
	}
	webSocket.broadcastBIN(data, FADERS_MAX + 1);
	
	data[0]=WS_FADNAME; //send faders name 
	for(int d=0; d < FADERS_MAX; d++)
	{
		if(faders[d].name[0] != 0)
		{
			data[1]=d;
			mempcpy(bytePtr + 2, faders[d].name, 9); 
			webSocket.broadcastBIN(data, 11);
		}
	}
	
	data[0]=WS_MASTVAL; //send masterVal 
	data[1]=master;
	webSocket.broadcastBIN(data, 2);
	
	if(debug_flag)Serial.println("send faders state");
	
}

void WSui_SendScenesState()
{
	uint8_t data[DMX_CH+1];
	uint8_t * bytePtr = (uint8_t*) &data; 
	
	/*
	//send dmx view***********************
	data[0]=WS_DMX; //send DMX 
	memcpy(bytePtr + 1, dmxTxBuffer, ARRAY_SIZE(dmxTxBuffer));    
	webSocket.broadcastBIN(data, sizeof(data));
	* */
	
	//send scenes states***********************
	data[0]=WS_SCNVAL; //send faders val
	for(int d=0; d < MEM_MAX; d++)
	{
		data[d+1]=scenes[d].value;
	}
	webSocket.broadcastBIN(data, MEM_MAX + 1);
	
	/*
	data[0]=WS_FADFX; //send faders fx 
	for(int d=0; d < FADERS_MAX; d++)
	{
		data[d+1]=faders[d].fx[3] + faders[d].fx[2] * 2 + faders[d].fx[1] * 4 + faders[d].fx[0] * 8;
	}
	webSocket.broadcastBIN(data, FADERS_MAX + 1);
	*/
	
	data[0]=WS_SCNNAME; //send scenes name 
	for(int d=0; d < MEM_MAX; d++)
	{
		if(scenes[d].name[0] != 0)
		{
			data[1]=d;
			mempcpy(bytePtr + 2, scenes[d].name, 9); 
			webSocket.broadcastBIN(data, 11);
		}
		
	}
	
	data[0]=WS_MASTVAL; //send masterVal 
	data[1]=master;
	webSocket.broadcastBIN(data, 2);
	
	if(debug_flag)Serial.println("send scenes state");
	
}

void	WSui_SendFXsState()
{
	uint8_t data[DMX_CH+1];
	uint8_t * bytePtr = (uint8_t*) &data; 
	uint8_t count;
	
	/*
	//send dmx view***********************
	data[0]=WS_DMX; //send DMX 
	memcpy(bytePtr + 1, dmxTxBuffer, ARRAY_SIZE(dmxTxBuffer));    
	webSocket.broadcastBIN(data, sizeof(data));
	* */
	
	//send fxs states***********************
	data[0]=WS_FXVAL; //send fxs val
	count=1;
	for(int d=0; d < FX_MAXNB; d++)
	{
		
		data[ count ]=fxs[d].type;
		count++;
		data[ count ]=fxs[d].speed;
		count++;
		data[ count ]=fxs[d].min_val;
		count++;
		data[ count ]=fxs[d].max_val;
		count++;
	}
	webSocket.broadcastBIN(data, count);
}

//onboard led *********************************************************
void flash_led()
{
	flag_flash_led=true;
	flash_led_timer=0;
}

void blink_led(boolean state)
{
	flag_blink_led=state;
}

void set_led(boolean state)
{
  digitalWrite(LED_PIN,state);
}

void toggle_led()
{
  digitalWrite(LED_PIN,!digitalRead(LED_PIN));
}

void update_led()
{
	if(flag_flash_led)
	{
		if(flash_led_timer < FLASH_TIMER / 2)
		{
			digitalWrite(LED_PIN,true);
		}
		else if(flash_led_timer < FLASH_TIMER)
		{
			digitalWrite(LED_PIN,false);
		}else{
			flag_flash_led=false;
		}
	} 
  else if(flag_blink_led) 
  { 
		if(blink_led_timer < BLINK_TIMER)
		{
			digitalWrite(LED_PIN,bitRead(blink_led_timer,6)); //pulse at 32ms
		}
		else if(blink_led_timer < (2*BLINK_TIMER))
		{
			digitalWrite(LED_PIN,false);
		}
		else
		{
			blink_led_timer =0;
		}
	} 
}

//WIFI ****************************************************************
void initWifi()
{
	int count=0;
	if(loadDataFromSPIFFS("wifi.bin", client))flag_wifi_client=true;
	
	WiFi.disconnect(true);
	
	if(flag_wifi_client)
	{
		WiFi.mode(WIFI_AP_STA);
	}else{
		WiFi.mode(WIFI_AP);
	}
	delay(500);

	//version AP
	if(debug_flag)Serial.println("[WiFi] create WiFi AP");

	WiFi.softAPConfig(AP_local_ip, AP_gateway, AP_subnet);
	WiFi.softAP(SSID_DMXui, PASS_DMXui);
	delay(200);
	
	//version STA
	if(flag_wifi_client)
	{
		WiFi.begin(client.ssid, client.password);
		if(debug_flag)Serial.print("[WiFi] Connecting to WiFi:");
		while (WiFi.status() != WL_CONNECTED) {
			delay(1000);
			if(debug_flag)Serial.print(".");
			count++;
			if(count > 15)break;
		}
		if(debug_flag)Serial.println();
		
		if(count > 15)
		{
			//error on client restart in wifi AP mode
			if(debug_flag)Serial.println("[WiFi] Error can not connect as client");
			SPIFFS.remove("/spiffs/wifi.bin");
			flag_wifi_client=false;
			dodo10s; 
		}
	}

	if(debug_flag)Serial.print("[Wifi] Web Server's IP(s): ");
	if(debug_flag)Serial.print(WiFi.softAPIP());
	if(flag_wifi_client)
	{
		if(debug_flag)Serial.print(", ");
		if(debug_flag)Serial.print(WiFi.localIP());
	}
	if(debug_flag)Serial.println();
	if(debug_flag)Serial.flush();
}

//mDNS init
void init_mDNS()
{
	if(flag_wifi_client)
	{
		mdns.begin(WiFi.localIP(), DNS_DMXui);
	}else{
		mdns.begin(WiFi.softAPIP(), DNS_DMXui);
	}
	delay(100);
	mdns.addServiceRecord("dmxui._http", 80, MDNSServiceTCP);
	if(debug_flag)Serial.print("[BOOT] mDNS host: ");
	if(debug_flag)Serial.println(DNS_DMXui);
}
 
//HTTP ************************************************************
void initHttp()
{
  // Serve the specified HTML pages
  
	//index.html *************
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		if(debug_flag)Serial.println("Web Server: home page");
		String html = HTML_CONTENT_HOME;  // Use the HTML content from the index.h file
		IPAddress ip=request->client()->remoteIP();
		char buffip[20];
		snprintf(buffip,20,"%d.%d.%d.%d",ip[0], ip[1], ip[2], ip[3]);
		html.replace("%YOUR_IP%", buffip);
		snprintf(buffip,20,"%ld",esp_get_minimum_free_heap_size());
		html.replace("%MINFREE%", buffip);
		snprintf(buffip,20,"%u ms, avg: %d ms",speedTest_res,dmxRateLPF);
		html.replace("%DMXRATE%", buffip);
		if(!artnetDMX.enable)
		{
			snprintf(buffip,20,"display:none;");
			html.replace("%ARTNET%", buffip);
		}else{
			snprintf(buffip,20,"color:red;");
			html.replace("%ARTNET%", buffip);
		}
		request->send(200, "text/html", html);
	});

	//dimmers.html *************
	server.on("/dimmers.html", HTTP_GET, [](AsyncWebServerRequest *request) {
		if(debug_flag)Serial.println("DMX-UI: dimmers");
		request->send(SPIFFS, "/dimmers.html", "text/html");
		//request->send(SPIFFS, “/dimmers.html”, String(), false, fader_processor);
	});
  
	//faders.html *************
	server.on("/faders.html", HTTP_GET, [](AsyncWebServerRequest *request) {
		if(debug_flag)Serial.println("DMX-UI: faders");
		// Check for the 'page' parameter in the query string
		if (request->hasArg("page")) 
		{
			String page = request->arg("page");
			if (page == ">") 
			{
				ui_faders_page_change(ui_faders_page_state + 1);
			} 
			else if (page == "<") 
			{
				ui_faders_page_change(ui_faders_page_state - 1);
			} 
			else if (page.toInt())
			{
				ui_faders_page_change(page.toInt());
			}
		}
		//request->send(SPIFFS, "/index.html", "text/html");
		request->send(SPIFFS, "/faders.html", String(), false, faders_processor);
	});
	
	//scenes.html *************
	server.on("/scenes.html", HTTP_GET, [](AsyncWebServerRequest *request) {
		if(debug_flag)Serial.println("DMX-UI: scenes");
		request->send(SPIFFS, "/scenes.html", "text/html");
		//request->send(SPIFFS, “/dimmers.html”, String(), false, fader_processor);
	});
	
	//fx.html *********************
	server.on("/fx1.html", HTTP_GET, [](AsyncWebServerRequest *request) {
		if(debug_flag)Serial.println("DMX-UI: fx1");
		//request->send(SPIFFS, "/dimmers.html", "text/html");
		request->send(SPIFFS, "/fx.html", String(), false, fx1_processor);
	});
	
	server.on("/fx2.html", HTTP_GET, [](AsyncWebServerRequest *request) {
		if(debug_flag)Serial.println("DMX-UI: fx2");
		//request->send(SPIFFS, "/dimmers.html", "text/html");
		request->send(SPIFFS, "/fx.html", String(), false, fx2_processor);
	});
	
	//settings.html *********************
	server.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest *request) {
		if(debug_flag)Serial.println("DMX-UI: settings");
		bool redir=false;
		if (request->hasArg("saveALL")) 
		{
			saveDMXDatasToSPIFFS();
			redir=true;
		}
		
		if (request->hasArg("loadALL")) 
		{
			loadDMXDatasFromSPIFFS();
			redir=true;
		}
		
		if (request->hasArg("loadDIM")) 
		{
			loadDMXDatas_DIM();
			redir=true;
		}
		
		if (request->hasArg("loadFAD")) 
		{
			loadDMXDatas_FAD();
			redir=true;
		}
		
		if (request->hasArg("loadFX")) 
		{
			loadDMXDatas_FX();
			redir=true;
		}
		
		if (request->hasArg("loadSCNALL")) 
		{
			loadDMXDatas_SCNALL();
			redir=true;
		}
		
		if (request->hasArg("loadSCN")) 
		{
			String scn = request->arg("loadSCN");
			if (scn.toInt() > 0 && scn.toInt() < 9)
			{
				loadDMXDatas_SCN((uint8_t)scn.toInt());
			} 
			redir=true;
		}
		
		//saves
		if (request->hasArg("saveDIM")) 
		{
			saveDMXDatas_DIM();
			redir=true;
		}
		
		if (request->hasArg("saveFAD")) 
		{
			saveDMXDatas_FAD();
			redir=true;
		}
		
		if (request->hasArg("saveFX")) 
		{
			saveDMXDatas_FX();
			redir=true;
		}
		
		if (request->hasArg("saveSCNALL")) 
		{
			saveDMXDatas_SCNALL();
			redir=true;
		}
		
		if (request->hasArg("saveSCN")) 
		{
			String scn = request->arg("saveSCN");
			if (scn.toInt() > 0 && scn.toInt() < 9)
			{
				saveDMXDatas_SCN((uint8_t)scn.toInt());
			} 
			redir=true;
		}
		
		if (request->hasArg("RESET_ALL")) 
		{
			resetallDMXDatas();
			redir=true;
		}
		
		if (request->hasArg("REBOOT")) 
		{
			webSocket.broadcastTXT("SYSTEM REBOOT !!");
			//delay(500);
			rebooting=true;
			redir=true;
		}
		
		if(redir)
		{
			request->redirect("/settings.html");
		}
		else
		{
			request->send(SPIFFS, "/settings.html", "text/html");
		}
		//request->send(SPIFFS, “/fx.html”, String(), false, fx1_processor);
	});
	
	//vu.html *********************
	server.on("/vu.html", HTTP_GET, [](AsyncWebServerRequest *request) {
		if(debug_flag)Serial.println("DMX-UI: vu");
		request->send(SPIFFS, "/vu.html", "text/html");
		//request->send(SPIFFS, “/fx.html”, String(), false, fx1_processor);
	});

	//wifi_client.html *********************
	server.on("/wifi_client.html", HTTP_GET, [](AsyncWebServerRequest *request) {
		if(debug_flag)Serial.println("DMX-UI: wifi client");
		if(request->hasArg("newSSID") && request->hasArg("newPASS")) 
		{
			String newssid = request->arg("newSSID");
			snprintf(client.ssid,48,"%s",newssid.c_str());
			String newpass = request->arg("newPASS");
			snprintf(client.password,48,"%s",newpass.c_str());
			saveDataToSPIFFS("wifi.bin",client);
		}
		if (request->hasArg("rebootNOW")) 
		{
			flag_wifi_client=true;
			request->redirect("/");
			rebooting=true; // reboot in client mode
		}
		if (request->hasArg("disableCLI")) 
		{
			flag_wifi_client=false;
			SPIFFS.remove("/spiffs/wifi.bin");
			request->redirect("/");
			rebooting=true; // reboot in AP mode
		}
		request->send(SPIFFS, "/wifi_client.html", String(), false, wifi_processor);
	});
	
	//Artnet client.html *********************
	server.on("/artnet_client.html", HTTP_GET, [](AsyncWebServerRequest *request) {
		if(debug_flag)Serial.println("DMX-UI: ArtNet client");
		if(request->hasArg("enableARTNET") && request->hasArg("univARTNET")) 
		{
			String ena = request->arg("enableARTNET");
			String univ = request->arg("univARTNET");
			if (ena.toInt() > 0)
			{
				artnetDMX.enable=true;
			}else{
				artnetDMX.enable=false;
			}
			artnetDMX.universe=univ.toInt();
			saveDataToSPIFFS("artnet.bin",artnetDMX);
			request->redirect("/artnet_client.html");
		}
		if (request->hasArg("rebootNOW")) 
		{
			rebooting=true; // reboot in client mode
			request->redirect("/");
		}
		request->send(SPIFFS, "/artnet_client.html", String(), false, artnet_processor);
	});


	//********WEB SPIFFS ***************************************
	
	//manager *********************
	server.on("/manager", HTTP_GET, [](AsyncWebServerRequest *request)
	{
	request->send_P(200, "text/html", manager_html, spiffs_processor);
	});

	//update *********************
	server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request)
	{
		rebooting = !Update.hasError();
		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", rebooting ? ok_html : failed_html);

		response->addHeader("Connection", "close");
		request->send(response);
	},
	[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
	{
		if(!index)
		{
			Serial.print("Updating: ");
			Serial.println(filename.c_str());

			if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000))
			{
				Update.printError(Serial);
			}
		}
		
		if(!Update.hasError())
		{
			if(Update.write(data, len) != len)
			{
				Update.printError(Serial);
			}
		}
		if(final)
		{
			if(Update.end(true))
			{
				Serial.print("The update is finished: ");
				Serial.println(convertFileSize(index + len));
			}
			else
			{
				Update.printError(Serial);
			}
		}
	});

	//download *********************
	server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request)
	{

		String inputMessage = request->getParam("download_path")->value();
		if(debug_flag)Serial.println("download path:");
		if(debug_flag)Serial.println(inputMessage);
		if(inputMessage)
		{
			AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/spiffs/" + inputMessage, String(), true);
			response->addHeader("Server", "ESP Async Web Server");
			//response->addHeader("Content-Disposition","attachment; filename=\"" + inputMessage "\"");
			request->send(response);
			//request->send(SPIFFS, "/spiffs/" + inputMessage, String(), true);
	  
		}
		//request->redirect("/manager");
	});

	//delete *********************
	server.on("/delete", HTTP_GET, [](AsyncWebServerRequest *request)
	{
		char filename[36];
		filename[0]=0; //end C string
		String inputMessage = request->getParam("delete_path")->value();
		if(debug_flag)Serial.print("[DEBUG] delete path:");
		if(debug_flag)Serial.println(inputMessage);
		snprintf(filename,36,"/spiffs/%s",inputMessage.c_str());
		if(inputMessage)
		{
			if(debug_flag)Serial.print("[INFO] deleting:");
			if(debug_flag)Serial.println(filename);
			if(!SPIFFS.remove(filename))
			{
				snprintf(filename,36,"/%s",inputMessage.c_str());
				SPIFFS.remove(filename);
			}
		}
		request->redirect("/manager");
	});

	//upload *********************
	server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) 
	{
		request->send(200);
	}, uploadFile);

	// END WEB SPIFFS *****************
	
	server.serveStatic("/doc", SPIFFS, "/dmxui.pdf");
	
	// 404 and 405 error handler ********
	server.onNotFound([](AsyncWebServerRequest *request) 
	{
		if (request->method() == HTTP_GET) 
		{
		// Handle 404 Not Found error
			if(debug_flag)Serial.println("Web Server: 404 Not Found");
			String html = HTML_404;  // Use the HTML content from the error_404.h file
			request->send(404, "text/html", html);
		} 
		else
		{
			// Handle 405 Method Not Allowed error
			if(debug_flag)Serial.println("Web Server: 405 Method Not Allowed");
			String html = HTML_405;  // Use the HTML content from the error_405.h file
			request->send(405, "text/html", html);
		}
	});

	//start server
	server.begin();
	
	//init mDNS
	init_mDNS();
	
	if(debug_flag)Serial.println("[BOOT] DMXui Web server started");
}

// ************fin http init ********************************************************************************


String faders_processor(const String& var)
{
	if(var == "FADERS")
	{
		String editfad=String(ui_faders_html);
		return editfad;
	}
	if(var == "PAGE")
	{
		String editpag=String(ui_faders_page_state);
		return editpag;
	}
	return String();
}

String fx1_processor(const String& var)
{
	if(var == "FXS")
	{
		String editfx= "fx,1,4,X,0";
		return editfx;
	}
	if(var == "NUMFX")
	{
		String editpag="1";
		return editpag;
	}
	if(var == "COLORFX1")
	{
		String editfx1="border-color: orange";
		return editfx1;
	}
	return String();
}

String fx2_processor(const String& var)
{
	if(var == "FXS")
	{
		String editfx= "fx,1,4,X,1";
		return editfx;
	}
	if(var == "NUMFX")
	{
		String editpag="2";
		return editpag;
	}
	if(var == "COLORFX2")
	{
		String editfx2="border-color: orange";
		return editfx2;
	}
	return String();
}

String wifi_processor(const String& var)
{
	if(var == "STA_STATUS")
	{
		if(WiFi.status() != WL_CONNECTED)
		{
			return String("NOT connected !!!");
		}else{
			return String("CONNECTED");
		}
	}
	if(var == "STA_IP")
	{
		return WiFi.localIP().toString();
	}
	if(var == "STA_SSID")
	{
		return String(client.ssid);
	}
	if(var == "STA_RSSI")
	{
		return String(WiFi.RSSI());
	}
	return String();
}

String artnet_processor(const String& var)
{
	if(var == "IP")
	{
		if(flag_wifi_client)
		{
			return WiFi.localIP().toString();
		}
		return WiFi.softAPIP().toString();
	}
	if(var == "UNIV")
	{
		return String(artnetDMX.universe);
	}
	if(var == "ENABLE")
	{
		if(artnetDMX.enable)
		{
			return String("checked");
		}else{
			return String("");
		}
	}

	return String();
}

String spiffs_processor(const String& var)
{
  if(var == "SPIFFS_FREE_BYTES")
  {
    return convertFileSize((SPIFFS.totalBytes() - SPIFFS.usedBytes()));
  }

  if(var == "SPIFFS_USED_BYTES")
  {
    return convertFileSize(SPIFFS.usedBytes());
  }

  if(var == "SPIFFS_TOTAL_BYTES")
  {
    return convertFileSize(SPIFFS.totalBytes());
  }
  
  if(var == "LISTEN_FILES")
  {
    return listDir(SPIFFS, "/", 0);
  }

/*
  if(var == "EDIT_FILES")
  {
    String editDropdown = "<select name=\"edit_path\" id=\"edit_path\">";
    editDropdown += "<option value=\"choose\">Select file to edit</option>";
    editDropdown += "<option value=\"new\">New text file</option>";
    editDropdown += filesDropdownOptions;      
    editDropdown += "</select>";
    return editDropdown;
  }
  * */
  
  if(var == "DELETE_FILES")
  {
    String deleteDropdown = "<select name=\"delete_path\" id=\"delete_path\">";
    deleteDropdown += "<option value=\"choose\">Select file to delete</option>";
    deleteDropdown += filesDropdownOptions;      
    deleteDropdown += "</select>";
    return deleteDropdown;
  }
  
  if(var == "DOWNLOAD_FILES")
  {
    String deleteDropdown = "<select name=\"download_path\" id=\"download_path\">";
    deleteDropdown += "<option value=\"choose\">Select file to download</option>";
    deleteDropdown += filesDropdownOptions;      
    deleteDropdown += "</select>";
    return deleteDropdown;
  }
 /* 
  if(var == "SAVE_PATH_INPUT")
  {
    if(savePath == "/new.txt")
    {
      savePathInput = "<input type=\"text\" id=\"save_path\" name=\"save_path\" value=\"" + savePath + "\" >";
    }
    else
    {
      savePathInput = "";
    }
    return savePathInput;
  }
  * */
  return String();
}

String listDir(fs::FS &fs, const char * dirname, uint8_t levels)
{
  char filename[21];
  filesDropdownOptions = "";
  //String filename;
  String listenFiles = "<table><tr><th id=\"first_td_th\">List the library: </th><th>";
  listenFiles += dirname;
  listenFiles += "</th></tr>";

  File root = fs.open(dirname);
  String fail = "";
  if(!root)
  {
    fail = " the library cannot be opened";
    return fail;
  }
  if(!root.isDirectory())
  {
    fail = " this is not a library";
    return fail;
  }

  File file = root.openNextFile();
  while(file)
  {
    if(file.isDirectory())
    {
      listenFiles += "<tr><td id=\"first_td_th\">Library: ";
      listenFiles += file.name();

      filesDropdownOptions += "<option value=\"";
      filesDropdownOptions += file.name();
      filesDropdownOptions += "\">";
      filesDropdownOptions += file.name();
      filesDropdownOptions += "</option>";

      listenFiles += "</td><td> - </td></tr>";

      if(levels)
      {
        listDir(fs, file.name(), levels -1);
      }
    }
    else 
    {
      snprintf(filename,21,"%s",file.name());

	if( debug_flag || (! isHTML(filename) && ! isPDF(filename)))
      {
		  listenFiles += "<tr><td id=\"first_td_th\">File: ";
		  listenFiles += file.name();
	  }

      if(! isHTML(filename) && ! isPDF(filename))
      {
		filesDropdownOptions += "<option value=\"";
		filesDropdownOptions += file.name();
		filesDropdownOptions += "\">";
		filesDropdownOptions += file.name();
		filesDropdownOptions += "</option>";
	  }

		if( debug_flag || (! isHTML(filename) && ! isPDF(filename)))
		{
			listenFiles += " </td><td>\tSize: ";
			listenFiles += convertFileSize(file.size());
			listenFiles += "</td></tr>";
		}
    }
    file = root.openNextFile();
  }
  listenFiles += "</table>";
  return listenFiles;  
}

bool isHTML(char* filename) {
  int8_t len = strlen(filename);
  bool result;
  if ( strstr(strlwr(filename + (len - 5)), ".html"))
  {
    result = true;
  } else {
    result = false;
  }
  return result;
}

bool isPDF(char* filename) {
  int8_t len = strlen(filename);
  bool result;
  if ( strstr(strlwr(filename + (len - 4)), ".pdf"))
  {
    result = true;
  } else {
    result = false;
  }
  return result;
}

void uploadFile(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) 
{
  if(!index)
  {
    request->_tempFile = SPIFFS.open("/" + filename, "w");
  }
  if(len)
  {
    request->_tempFile.write(data, len);
  }
  if(final)
  {
    request->_tempFile.close();
    request->redirect("/manager");
  }
}

String convertFileSize(const size_t bytes)
{
  if(bytes < 1024)
  {
    return String(bytes) + " B";
  }
  else if (bytes < 1048576)
  {
    return String(bytes / 1024.0) + " kB";
  }
  else if (bytes < 1073741824)
  {
    return String(bytes / 1048576.0) + " MB";
  }
  return String("too big");
}


//SPIFFS file system replace SD****************************************
bool initSPIFFS()
{
	if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
	{ 
		Serial.println("[BOOT] SPIFFS init failed");
		return false;
	}
	//("initialization done.");
	Serial.println( "[BOOT] SPIFFS init OK");
	return true;
}

void readFile(fs::FS &fs, const char * path)
{
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message)
{
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

template <class T> bool saveDataToSPIFFS(const char * filename, T& value)
{
	char path[32];
	int res=0;
	snprintf(path,32,"/spiffs/%s", filename);
	myFile = SPIFFS.open(path, FILE_WRITE);
	if(!myFile){
		if(debug_flag)Serial.print("[ERROR] ");
		if(debug_flag)Serial.print(path);
	   if(debug_flag)Serial.println(" : There was an error opening the file for writing");
	}else{
		res=FILE_writeAnything(myFile, value);
	}
	myFile.close();    
	if(res){
		if(debug_flag)Serial.print("[FILE] ");
		if(debug_flag)Serial.print(path);
        if(debug_flag)Serial.println(" FILE_writeAnything written");
    } else {
		if(debug_flag)Serial.print("[ERROR] ");
		if(debug_flag)Serial.print(path);
        if(debug_flag)Serial.println(" FILE_writeAnything write failed");
    }
	return (res>0);
}

template <class T> bool loadDataFromSPIFFS(const char * filename, T& value)
{
	char path[32];
	int res=0;
	snprintf(path,32,"/spiffs/%s", filename);
	myFile = SPIFFS.open(path);
	if(!myFile){
		if(debug_flag)
		{
			Serial.print("[ERROR] ");
			Serial.print(path);
		   Serial.println(" : There was an error opening the file for reading");
		}
	}else{
		res=FILE_readAnything(myFile, value);
	}
	myFile.close();   
	if(res>0){
		if(debug_flag)
		{
			Serial.print("[FILE] ");
			Serial.print(path);
			Serial.println(" FILE_readAnything read success");
		}
    } else {
		if(debug_flag)
		{
			Serial.print("[ERROR] ");
			Serial.print(path);
			Serial.println(" FILE_readAnything read failed");
		}
    }
	return (res>0);
}

void loadDMXDatasFromSPIFFS()
{
	char filename[16];
	//load dimmers
	snprintf(filename,16,"dimmers.bin");
	loadDataFromSPIFFS(filename,dimmers);
	
	//load faders
	snprintf(filename,16,"faders.bin");
	loadDataFromSPIFFS(filename,faders);
	
	//load fx
	snprintf(filename,16,"fxs.bin");
	loadDataFromSPIFFS(filename,fxs);
	
	//load scenes
	for(int s=0; s < MEM_MAX; s++)
	{
		snprintf(filename,16,"scene%d.bin",(s+1));
		loadDataFromSPIFFS(filename,scenes[s]);
	}
	webSocket.broadcastTXT("[FILE] All files loaded");
	delay(500);
}

void saveDMXDatasToSPIFFS()
{
	char filename[16];
	//save dimmers
	snprintf(filename,16,"dimmers.bin");
	saveDataToSPIFFS(filename,dimmers);
	
	//save faders
	snprintf(filename,16,"faders.bin");
	saveDataToSPIFFS(filename,faders);
	
	//save fx
	snprintf(filename,16,"fxs.bin");
	saveDataToSPIFFS(filename,fxs);
	
	//save scenes
	for(int s=0; s < MEM_MAX; s++)
	{
		snprintf(filename,16,"scene%d.bin",(s+1));
		saveDataToSPIFFS(filename,scenes[s]);
	}
	webSocket.broadcastTXT("[FILE] All files saved");
	delay(500);
}

void loadDMXDatas_DIM()
{
	char filename[16];
	//load dimmers
	snprintf(filename,16,"dimmers.bin");
	loadDataFromSPIFFS(filename,dimmers);
	webSocket.broadcastTXT("[FILE] Dimmers loaded");
	delay(500);
}

void loadDMXDatas_FAD()
{
	char filename[16];
	//load faders
	snprintf(filename,16,"faders.bin");
	loadDataFromSPIFFS(filename,faders);
	webSocket.broadcastTXT("[FILE] Faders loaded");
	delay(500);
}
void loadDMXDatas_FX()
{
	char filename[16];
	//load fx
	snprintf(filename,16,"fxs.bin");
	loadDataFromSPIFFS(filename,fxs);
	webSocket.broadcastTXT("[FILE] FXs loaded");
	delay(500);
}

void loadDMXDatas_SCNALL()
{
	char filename[16];
	//load scenes
	for(int s=0; s < MEM_MAX; s++)
	{
		snprintf(filename,16,"scene%d.bin",(s+1));
		loadDataFromSPIFFS(filename,scenes[s]);
	}
	webSocket.broadcastTXT("[FILE] Scenes 1 to 8 loaded");
	delay(500);
}

void saveDMXDatas_DIM()
{
	char filename[16];
	//save dimmers
	snprintf(filename,16,"dimmers.bin");
	saveDataToSPIFFS(filename,dimmers);
	webSocket.broadcastTXT("[FILE] Dimmers saved");
	delay(500);
}

void saveDMXDatas_FAD()
{
	char filename[16];
	//save faders
	snprintf(filename,16,"faders.bin");
	saveDataToSPIFFS(filename,faders);
	webSocket.broadcastTXT("[FILE] Faders saved");
	delay(500);
}

void saveDMXDatas_FX()
{
	char filename[16];
	//save fx
	snprintf(filename,16,"fxs.bin");
	saveDataToSPIFFS(filename,fxs);
	webSocket.broadcastTXT("[FILE] FXs saved");
	delay(500);
}

void saveDMXDatas_SCNALL()
{
	char filename[16];
	//save scenes
	for(int s=0; s < MEM_MAX; s++)
	{
		snprintf(filename,16,"scene%d.bin",(s+1));
		saveDataToSPIFFS(filename,scenes[s]);
	}
	webSocket.broadcastTXT("[FILE] Scenes saved");
	delay(500);
}

void resetallDMXDatas()
{
	reset_dimmers();
	memset(faders, 0, sizeof(faders));
	memset(fxs, 0, sizeof(fxs));
	memset(scenes, 0, sizeof(scenes));
	saveDMXDatasToSPIFFS();
	webSocket.broadcastTXT(" !!! DMX datas reset !!!");
	if(debug_flag)Serial.println(" !!! DMX datas reset !!!");
	delay(500);
}

//parse WS command *****************************************************
// serial command ******************************************************
void init_sCmd()
{
  // Setup callbacks for SerialCommand commands
  //sCmd.addCommand("sendCC", cmd_CC);  //
  
  sCmd.addCommand("D", cmd_set_dimmer_lvl);  //
  sCmd.addCommand("P", cmd_set_dimmer_patch);  //
  sCmd.addCommand("M", cmd_set_dimmer_max);  //
  sCmd.addCommand("T", cmd_set_dimmer_type);  //
  sCmd.addCommand("R", cmd_refresh_dimmers_ui);  //
  
  sCmd.addCommand("F", cmd_set_fader_lvl);  //
  sCmd.addCommand("z", cmd_fader_assign_fx);  //
  sCmd.addCommand("n", cmd_set_fader_name);  //
  sCmd.addCommand("r", cmd_refresh_faders_ui);  //
  
  sCmd.addCommand("X", cmd_set_fx);  //
  sCmd.addCommand("v", cmd_refresh_fxs_ui);
  
  sCmd.addCommand("S", cmd_set_scene_lvl);  //
  sCmd.addCommand("N", cmd_set_scene_name);  //
  sCmd.addCommand("V", cmd_refresh_scenes_ui);  //
  
  sCmd.addCommand("W", cmd_refresh_vu_ui);  //
  sCmd.addCommand("w", cmd_lock_refresh_vu_ui); 
  sCmd.addCommand("m", cmd_unlock_refresh_vu_ui); 
  
  sCmd.addCommand("f", cmd_fonctions);  //
  
  sCmd.addCommand("G", cmd_set_master);  //

  sCmd.addCommand("s", cmd_record_fader_state_to_sceneArray);  //
  
  sCmd.addCommand("o", cmd_saveDMXDatas_SCN);
  
  sCmd.addCommand("Z", cmd_load_scene_to_fader);  //
  
  sCmd.addCommand("p", cmd_change_fader_page);  //

  sCmd.setDefaultHandler( cmd_error );
}

void cmd_error(const char *command)
{
    //sprintf(WSstrbuff, "chs_ls,%d,%d", aNum + 1 , lastChaseStep[aNum] + 1);
    if(debug_flag)
    {
		Serial.print("[ERROR] ");
		Serial.print("WS read error :");
		Serial.print(command);
		Serial.println();
	}
  
}

void cmd_set_dimmer_lvl()
{
	int aNum, bNum;
	uint16_t ch;
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	ch = constrain(aNum,0,511);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer 
	dimmers[ch].value = constrain(aNum,0,255);
	
}

void cmd_set_fader_lvl()
{
	int aNum, bNum;
	uint16_t ch;
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	ch = constrain(aNum,1,FADERS_MAX);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	bNum = atoi(arg); // Converts a char string to an integer 
	faders[ch - 1].value = constrain(bNum,0,255);
	
}


void cmd_set_scene_lvl()
{
	int aNum, bNum;
	uint16_t ch;
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	ch = constrain(aNum,1,MEM_MAX);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	bNum = atoi(arg); // Converts a char string to an integer 
	scenes[ch - 1].value = constrain(bNum,0,255);
	
}


void cmd_set_dimmer_patch()
{
	int aNum, bNum;
	uint16_t ch;
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	ch = constrain(aNum,0,511);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	if(aNum > 128)
	{
		dimmers[ch].patch = 255;
	}else{
		dimmers[ch].patch = constrain(aNum,0,FADERS_MAX - 1);
	}
	
	
}

void cmd_set_dimmer_max()
{
	int aNum, bNum;
	uint16_t ch;
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	ch = constrain(aNum,0,511);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer 
	dimmers[ch].max_val = constrain(aNum,0,255);
	
}

void cmd_set_dimmer_type()
{
	int aNum, bNum;
	uint16_t ch;
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	ch = constrain(aNum,0,511);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer 
	dimmers[ch].type = constrain(aNum,0,4);
	
}

void cmd_fonctions()
{
	int aNum, bNum;
	char *arg;
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	bNum = constrain(aNum,0,5);
	
	if(bNum == CLD)
	{
		for(int i=0; i < DMX_CH; i++)
		{
			dimmers[i].value = 0;
		}
		WSui_SendDimmersState();
	}
	
	else if(bNum == FULL)
	{
		for(int i=0; i < DMX_CH; i++)
		{
			dimmers[i].value = 255;
		}
		WSui_SendDimmersState();
	}
	
	else if(bNum == BLACK)
	{
		flag_black_all=!flag_black_all;
		WSui_SendDimmersState();
	}
	
	else if(bNum == CLF)
	{
		for(int i=0; i < FADERS_MAX; i++)
		{
			faders[i].value = 0;
		}
		WSui_SendFadersState();
	}
	
	else if(bNum == CLS)
	{
		for(int i=0; i < MEM_MAX; i++)
		{
			scenes[i].value = 0;
		}
		WSui_SendScenesState();
	}
	
	else if(bNum == PATCH)
	{
		for(int i=0; i < DMX_CH; i++)
		{
			dimmers[i].patch = 255; //255 is not patched to fader
			if(i < FADERS_MAX)dimmers[i].patch = i;
			
		}
		WSui_SendDimmersState();
	}
	
	//webSocketSendAllState();
}

void cmd_set_master()
{
	int aNum,mast;
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	master = constrain(aNum,0,255);
}

void cmd_fader_assign_fx()
{
	int aNum, bNum;
	uint16_t ch;
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	ch = constrain(aNum,1,FADERS_MAX);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer 
	bNum = constrain(aNum,0,NB_FX-1);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer 
	faders[ch - 1].fx[bNum] = constrain(aNum,0,1);
}

void cmd_set_fx()
{
	int aNum;
	uint8_t fx_nb,fx_set_nb,fx_val;
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	fx_nb = constrain(aNum,0,FX_MAXNB - 1);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	fx_set_nb=constrain(aNum,0,3);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	fx_val=constrain(aNum,0,255);
	
	if(fx_set_nb == FX_TYPE)
	{
		fxs[fx_nb].type=fx_val;
	}
	else if(fx_set_nb == FX_BEAT)
	{
		fxs[fx_nb].speed=fx_val;
	}
	else if(fx_set_nb == FX_MAX)
	{
		fxs[fx_nb].max_val=fx_val;
	}
	else if(fx_set_nb == FX_MIN)
	{
		fxs[fx_nb].min_val=fx_val;
	}
}

void cmd_set_fader_name()
{
	int aNum, bNum;
	uint16_t ch;
	char	buff[9];
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	ch = constrain(aNum,1,FADERS_MAX-1);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	memset(faders[ch-1].name, 0, 9);
	snprintf(faders[ch-1].name,9,"%s",arg);
	WSui_SendFadersState();
}

void cmd_set_scene_name()
{
	int aNum;
	uint16_t ch;
	char	buff[9];
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	ch = constrain(aNum,1,MEM_MAX);
	
	arg = sCmd.next();
	if (arg == NULL)return;
	memset(scenes[ch-1].name, 0, 9);
	snprintf(scenes[ch-1].name,9,"%s",arg);
	WSui_SendScenesState();
}
 

void cmd_save_fader_state_to_scene()
{
	int aNum;
	uint8_t scn;
	char filename[16];
	char *arg;

	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	scn = constrain(aNum,1,MEM_MAX);
	for(int f=0; f < FADERS_MAX; f++)
	{
		scenes[(scn-1)].faders_memstate[f]=faders[f];
	}
	saveDMXDatas_SCN(scn);
}

void cmd_saveDMXDatas_SCN()
{
	int aNum;
	uint8_t scn;
	char filename[16];
	char *arg;

	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	saveDMXDatas_SCN(aNum);
}

void saveDMXDatas_SCN(uint8_t scn)
{
	char filename[16];
	if (scn < 1 || scn > MEM_MAX)return;
	snprintf(filename,16,"scene%u.bin",scn);
	saveDataToSPIFFS(filename,scenes[scn-1]);
}

void loadDMXDatas_SCN(uint8_t scn)
{
	char filename[16];
	if (scn < 1 || scn > MEM_MAX)return;
	snprintf(filename,16,"scene%u.bin",scn);
	loadDataFromSPIFFS(filename,scenes[scn-1]);
}

void cmd_record_fader_state_to_sceneArray()
{
	int aNum;
	uint8_t scn;
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	scn = constrain(aNum,1,MEM_MAX);
	for(int f=0; f < FADERS_MAX; f++)
	{
		scenes[(scn-1)].faders_memstate[f]=faders[f];
	}
}

void cmd_load_scene_to_fader()
{
	int aNum;
	uint8_t scn;
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	aNum = atoi(arg); // Converts a char string to an integer
	scn = constrain(aNum,1,MEM_MAX);
	for(int f=0; f < FADERS_MAX; f++)
	{
		faders[f]=scenes[(scn-1)].faders_memstate[f];
	}
	WSui_SendFadersState();
	

}

void cmd_change_fader_page()
{
	int aNum;
	char up='<';
	char dw='>';
	char *arg;
	
	arg = sCmd.next();
	if (arg == NULL)return;
	if(arg[0] == '>')
	{
		ui_faders_page_change(ui_faders_page_state + 1); //ui_faders_page_state
	}
	if(arg[0] == '<')
	{
		ui_faders_page_change(ui_faders_page_state - 1);
	}
	
	aNum = atoi(arg); // Converts a char string to an integer
	master = constrain(aNum,0,255);
}

void cmd_refresh_dimmers_ui()
{
	WSui_SendDimmersState();
}

void cmd_refresh_faders_ui()
{
	WSui_SendFadersState();
}

void cmd_refresh_scenes_ui()
{
	WSui_SendScenesState();
}

void cmd_refresh_fxs_ui()
{
	WSui_SendFXsState();
}

void cmd_refresh_vu_ui()
{
	WSui_SendDmxState();
}

void cmd_lock_refresh_vu_ui()
{
	flag_continuous_refresh_vu_ui=true;
}

void cmd_unlock_refresh_vu_ui()
{
	flag_continuous_refresh_vu_ui=false;
}


//HTTP UI **************************************************************
void ui_faders_page_change(uint8_t page)
{
	int page_max=(FADERS_MAX / ui_faders_by_page);
	uint16_t ch_lw,ch_hi;
	
	ui_faders_page_state=constrain(page,1,page_max);
	ch_lw=((ui_faders_page_state - 1) * ui_faders_by_page)+1;
	ch_hi=ch_lw + ui_faders_by_page;
	ui_faders_page_hi_ch=constrain(ch_hi,1,FADERS_MAX+1);
	ui_faders_page_lw_ch=constrain(ch_lw,1,FADERS_MAX+1);
	
	snprintf(ui_faders_html,30,"faders,%u,%u,F",ui_faders_page_lw_ch,ui_faders_page_hi_ch);
	if(debug_flag)Serial.print("[INFO] nav to: " );
	if(debug_flag)Serial.println(ui_faders_html);
}


//DMX*******************************************************************
/** DMX ***********************************************************/

void initDMX()
{
	dmx_config_t config = DMX_CONFIG_DEFAULT;
	dmx_personality_t personalities[] = {};
    int personality_count = 0;
	dmx_driver_install(dmxPort, &config, personalities, personality_count);

	/* Now set the DMX hardware pins to the pins that we want to use and setup
	will be complete! */
	dmx_set_pin(dmxPort, DMXTX_pin, DMXRX_pin, DMXDIR_pin);
}

void refreshDMX()
{
	//char WSstrbuff[30];
	uint8_t buf_val,temp_dimmer_output,fx_temp, fx_best;
	uint8_t temp_fader_output;
	for (int dim = 0; dim < DMX_CH; dim++)//scan circuits one by one
	{
		temp_dimmer_output = 0;
		buf_val=0;
		//buf_val=faders[dimmers[dim].patch].value;
		
		if(dimmers[dim].value > 0)
		{
			buf_val=dimmers[dim].value;
		}
		else if(dimmers[dim].patch != 255)
		{
			if(faders[dimmers[dim].patch].value > 0)
			{
				buf_val=faders[dimmers[dim].patch].value;
				
				//apply fx !!!!!!!!!!
				fx_best=0;
				fx_temp=0;
				
				if(faders[dimmers[dim].patch].fx[0])
				{
					fx_temp=getFXres(0,false,faders[dimmers[dim].patch].value);
					fx_best=fx_temp;
					buf_val=fx_best;
				}
				
				
				if(faders[dimmers[dim].patch].fx[1])
				{
					fx_temp=getFXres(0,true,faders[dimmers[dim].patch].value);
					if(fx_temp > fx_best)
					{
						fx_best=fx_temp;
						buf_val=fx_best;
					}
				}

				
				if(faders[dimmers[dim].patch].fx[2])
				{
					fx_temp=getFXres(1,false,faders[dimmers[dim].patch].value);
					if(fx_temp > fx_best)
					{
						fx_best=fx_temp;
						buf_val=fx_best;
					}
				}

				
				if(faders[dimmers[dim].patch].fx[3])
				{
					fx_temp=getFXres(1,true,faders[dimmers[dim].patch].value);
					if(fx_temp > fx_best)
					{
						fx_best=fx_temp;
						buf_val=fx_best;
					}
				}

			}
			else
			{
				buf_val=getFadersValFromScenes(dimmers[dim].patch);
			}
		}//end of:  if dim val else fad val

		//*************dimmers direct **********************************
		if(dimmers[dim].type == ONOFF)
		{
			if(buf_val > 127)
			{
				temp_dimmer_output = dimmers[dim].max_val;
			}else{
				temp_dimmer_output = 0;
			}
		}
		else if(dimmers[dim].type == QUADHI)
		{
			uint16_t quad;
			quad=sqrt(buf_val * 255U);
			temp_dimmer_output = map((uint8_t)quad, 0, 255 , 0 , dimmers[dim].max_val);
		}
		else if(dimmers[dim].type == QUADLO)
		{
			uint16_t quad;
			quad=buf_val * buf_val / 255U;
			temp_dimmer_output = map((uint8_t)quad, 0, 255 , 0 , dimmers[dim].max_val);
		}
		else if(dimmers[dim].type == INVDIM)
		{
			temp_dimmer_output = map(buf_val, 0, 255, dimmers[dim].max_val, 0 );
		}
		else
		{
			temp_dimmer_output = map(buf_val, 0, 255 , 0 , dimmers[dim].max_val);
		}  
		
		if(flag_black_all)
		{
			dmxTxBuffer[dim]=0;
		}
		else if(dimmers[dim].value > 0)
		{
			dmxTxBuffer[dim]=temp_dimmer_output;
		}else{
			dmxTxBuffer[dim]=map(temp_dimmer_output,0,255,0,master);
		}
		
	} //end of: for dimmers
  

}

void sendDMX()
{
	uint8_t data[DMX_PACKET_SIZE];
	uint8_t * bytePtr = (uint8_t*) &data; 
	data[0]=0; //send DMX 
	memmove(bytePtr + 1, dmxTxBuffer, ARRAY_SIZE(dmxTxBuffer));
	//dmx_write_offset(DMXPORT, 1, dmxTxBuffer,512);
	dmx_write(DMXPORT, data, DMX_PACKET_SIZE);
	dmx_send_num(DMXPORT, DMX_PACKET_SIZE);
}

uint8_t getFXres(uint8_t fx_nb, bool inv, uint8_t in_val)
{
	uint8_t buff_res,buff_val,buff_val2;
	buff_res=in_val;
	switch (fxs[fx_nb].type) 
		{
			case FX_DIM:
				buff_res=map(in_val,0,255,fxs[fx_nb].min_val,fxs[fx_nb].max_val);
			break;
			
			case FX_STROBE:
				buff_val=squarewave8( beat8( fxs[fx_nb].speed), fxs[fx_nb].min_val);
				buff_val2=map(buff_val,0,255,0,fxs[fx_nb].max_val);
				buff_res=map(buff_val2,0,255,0,in_val);
			break;
			
			case FX_SIN:
				buff_val=beatsin8( fxs[fx_nb].speed, fxs[fx_nb].min_val, fxs[fx_nb].max_val, 0, 0);
				buff_res=map(buff_val,0,255,0,in_val);
			break;
			
			case FX_SAW:
				buff_val=beat8(fxs[fx_nb].speed);
				buff_val2=map(buff_val,0,255,fxs[fx_nb].min_val,fxs[fx_nb].max_val);
				buff_res=map(buff_val2,0,255,0,in_val);
			break;
			
			default:
				return in_val;
			break;
		}
	if(inv)return 255-buff_res;
	return buff_res;
}

uint8_t getFadersValFromScenes(uint16_t fader)
{
	uint8_t	res,aNum,bNum,fx_best,fx_temp;
	res=0;
	for(int m=0; m < MEM_MAX; m++)
	{
		if(scenes[m].value > 0)
		{
			//TODO apply fx to scenes[m].value!!!!!!!!!!
			aNum=scenes[m].faders_memstate[fader].value;
			
			//TODO apply fx to faders_memstate[fader] !!!!!!!!!!
			if(aNum > 0)
			{
				//apply fx !!!!!!!!!!
				fx_best=0;
				fx_temp=0;
				
				if(scenes[m].faders_memstate[fader].fx[0])
				{
					fx_temp=getFXres(0,false,aNum);
					fx_best=fx_temp;
					aNum=fx_best;
				}
				
				
				if(scenes[m].faders_memstate[fader].fx[1])
				{
					fx_temp=getFXres(0,true,aNum);
					if(fx_temp > fx_best)
					{
						fx_best=fx_temp;
						aNum=fx_best;
					}
				}

				
				if(scenes[m].faders_memstate[fader].fx[2])
				{
					fx_temp=getFXres(1,false,aNum);
					if(fx_temp > fx_best)
					{
						fx_best=fx_temp;
						aNum=fx_best;
					}
				}

				
				if(scenes[m].faders_memstate[fader].fx[3])
				{
					fx_temp=getFXres(1,true,aNum);
					if(fx_temp > fx_best)
					{
						fx_best=fx_temp;
						aNum=fx_best;
					}
				}
				
				bNum=map(aNum,0,255,0,scenes[m].value);
				if(bNum > res)res=bNum;

			}
			
			bNum=map(aNum,0,255,0,scenes[m].value);
			if(bNum > res)res=bNum;
		}
	}
	
	return res;
}

int getDmxRateLPF( uint16_t rawData )
{
  dmxRateLPF = dmxRateLPF - (( dmxRateLPF - (int)rawData) / 2);
  return dmxRateLPF;
}

//ARTNET **************************************************************
void initArtNet()
{
	artnetDMX.enable=false;
	if(! loadDataFromSPIFFS("artnet.bin", artnetDMX))return;
	if(artnetDMX.enable)
	{
		artnet.setArtDmxCallback(artnetDMXCallback);
		artnet.begin();
	}
}

void check_artnet()
{
	if(artnetDMX.enable)artnet.read();
}

void artnetDMXCallback(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data)
{
	if(universe != artnetDMX.universe)return; //select universe
	//if(artnetLastSeq > sequence)return; // discard old missed seq
	//artnetLastSeq = sequence;
	for (int i = 0; i < DMX_CH; i++) 
	{
		if(i < length)
		{
			dmxTxBuffer[i] = data[i];  
		}else{
			dmxTxBuffer[i] = 0;
		}
	}

	if(debug_flag)
	{
		bool tail = false;
		Serial.print("DMX: Univ: ");
		Serial.print(universe, DEC);
		Serial.print(", Seq: ");
		Serial.print(sequence, DEC);
		Serial.print(", Data (");
		Serial.print(length, DEC);
		Serial.print("): ");

		if (length > 16) {
		length = 16;
		tail = true;
		}
		// send out the buffer
		for (int i = 0; i < length; i++)
		{
		Serial.print(data[i], HEX);
		Serial.print(" ");
		}
		if (tail) {
		Serial.print("...");
		}
		Serial.println();
	}
}

//DEEP SLEEP **********************************************************
void dodo10s()
{
	Serial.println("********* DODO 10s ***********");
	Serial.flush();
	server.end();
	WiFi.disconnect(true);
	delay(500);
	WiFi.mode(WIFI_OFF);
	delay(1000);
	esp_sleep_enable_timer_wakeup(10ULL * uS_TO_S_FACTOR);
	esp_deep_sleep_start();
}



//SETUP ***********************************************************
void setup() 
{
	wakeup_reason = esp_sleep_get_wakeup_cause();
	
	Serial.begin(115200);
	delay(6000);

	//init GPIO
	pinMode(LED_PIN, OUTPUT);
	
	//init filesystem
	initSPIFFS();
	
	//serial debug mode
	if(SPIFFS.exists("/debug.txt"))debug_flag=true;
	
	//chip ID
	if(debug_flag)Serial.println(ESP_getChipId());
	
	// Init Wifi server
	initWifi();

	// Initialize WebSocket server
	initWebSocket();
	
	//init WS command parsing
	init_sCmd();
	
	// Initialize Http server
	initHttp();
	
	//init saved datas
	loadDMXDatasFromSPIFFS();
	
	// Init faders page
	ui_faders_page_change(1);
	
	//init DMX
	initDMX();
	
	//init ARTNET
	initArtNet();
	
	//// 
	flash_led();
}

//LOOP *************************************************************
void loop() 
{
  if(rebooting)
  {
    delay(1000);
    ESP.restart();
  }
  
  webSocket.loop();
  
  mdns.run();
  
  check_artnet();
  
  /*
  //interrupt take care
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    //refreshChasers();
  }
  */
  
  if(refresh_DMX_rate >= REFRESH_DMX_RATE)
  {
    speedTest_res=refresh_DMX_rate;
    refresh_DMX_rate = 0;
    
    if(!artnetDMX.enable)refreshDMX();
        
    sendDMX();
    
    getDmxRateLPF(uint16_t(speedTest_res));
    toggle_led();
  }
  
  if(flag_continuous_refresh_vu_ui)
  {
	if(refresh_vu_ui_rate > 200)
	{
		WSui_SendDmxState();
		refresh_vu_ui_rate=0;
	}
  }
  
  update_led();
  
}


//EOF *****************************************************************
