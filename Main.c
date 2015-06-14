/*
 *
 * This file is part of the RTX4100 API firmware
 * Miguel Colom, 2014 - http://mcolom.info
 *
 * This file may be licensed under the terms of of the
 * GNU General Public License Version 2 (the ``GPL'').
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the GPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the GPL along with this
 * program. If not, go to http://www.gnu.org/licenses/gpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */


/****************************************************************************
*                               Include files
****************************************************************************/

// Decomment to activate debugging using terminal LUART
//#define USE_LUART_TERMINAL

// Decomment to activate normal operation using SPI
#define SPI_COMMUNICATION

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>

#include <Core/RtxCore.h>
#include <ROS/RosCfg.h>
#include <PortDef.h>
#include <Api/Api.h>
#include <Cola/Cola.h>
#include <Protothreads/Protothreads.h>
#include <NetUtils/NetUtils.h>
#include <Drivers/DrvButtons.h>


#include <PtApps/AppCommon.h>
#include <PtApps/AppLed.h>
#include <PtApps/AppSocket.h>
#include <PtApps/AppWiFi.h>
#include <PtApps/AppWebconfig.h>
#include <PtApps/AppSntp.h>

#include <Drivers/DrvIntTemp.h>

#ifdef USE_LUART_TERMINAL
#include <Drivers/DrvLeuart.h>
#endif

#ifdef SPI_COMMUNICATION
#include <Drivers/DrvSpi.h>
#endif


//*******TEST ALEJANDRO CASTRO******//

#include <RtxEai/RtxEai.h>
#include <Drivers/DrvSpiSlave.h>
#include <BuildInfo/BuildInfo.h>
// For debug (EAI)
#define printf(...) RtxEaiPrintf(ColaIf->ColaTaskId,__VA_ARGS__)
// Data for configure softAP
#define SOFT_AP_STATIC_IP       "192.168.1.88"
#define SOFT_AP_SUBNET_MASK     "255.255.255.0"
#define SOFT_AP_GATEWAY         "192.168.1.1"
#define SOFT_AP_DHCP            TRUE
#define SOFT_AP_ESSID           "SCK_AP_" //partial SSID
#define SOFT_AP_SECURITY_TYPE   AWST_NONE
#define SOFT_AP_COUNTRY_CODE    "ES"
#define SOFT_AP_CHANNEL         2437 //Channel 6 center frequency
#define SOFT_AP_INACT           10 // minutes
#define SOFT_AP_BEACON          100 // ms
#define GATEWAY     "192.168.1.1"
#define DATE_STR_LEN 30
#define HTTP_PORT 80
#define RX_IDLE_TIMEOUT  (30*RS_T1SEC)

#define SOFT_AP_DHCP_POOL_START 0xC0A801C8 //"192.168.1.200"
#define SOFT_AP_DHCP_POOL_END   0xC0A801FF //"192.168.1.255"
#define SOFT_AP_LEASE_TIME      86400



/****************************************************************************
*                              Macro definitions
****************************************************************************/
// Max WiFi transmission power
#define MAX_TX_POWER 18

// Macros to print with the LUART
#define PRINT(x) UartPrint((rsuint8*)x)
#define PRINTLN(x) UartPrintLn((rsuint8*)x)

// Buffer sizes
#define TMP_STR_LENGTH 100
#define CMD_STR_LENGTH TMP_STR_LENGTH
#define TX_BUFFER_LENGTH 500

// Maximum number of arguments for terminal commands
#define MAX_ARGV 3

// Timeout in seconds for DNS resolutions
#define APP_DNS_RESOLVE_RSP_TIMEOUT (10*RS_T1SEC)

/****************************************************************************
*                     Enumerations/Type definitions/Structs
****************************************************************************/
// Application data stored at the NVS
typedef struct {
  ApInfoType ap_info;
  rsuint8 use_dhcp;
  struct pt ChildPt;
  PtEntryType* PtEntryPtr; 
  rschar DateStr[DATE_STR_LEN];
  rsint16 Temperature;
  rsint16 Humidity;
  rsint32 Pressure;
  rsint16 Lux;
  rsbool MultiSensor;
  rsbool HttpServerStarted;
  rsuint8 SPI1 [5];
  rschar HWVERSION[25];
  rschar SWVERSION[25];
  rschar platform [30];
  rsuint32 HttpInstance;
  ApiSocketAddrType static_address, static_subnet, static_gateway;
} app_dataType;
 

/****************************************************************************
*                            Global variables/const
****************************************************************************/
// Static application data
static app_dataType app_data;


// Timers
static const RosTimerConfigType PacketDelayTimer =
  ROSTIMER(COLA_TASK, APP_PACKET_DELAY_TIMEOUT,
  APP_PACKET_DELAY_TIMER);

static const RosTimerConfigType DnsRspTimer =
  ROSTIMER(COLA_TASK, APP_DNS_RSP_TIMEOUT,
  APP_DNS_RSP_TIMER);

// TCP flags
static char TCP_is_connected; // True when the TCP connection has been stablished
static char TCP_received; // True when data has been received at the TCP connection

static int socketHandle; // The socket ID of the TCP connection
static rsuint8 *TCP_receive_buffer_ptr; // TCP receive buffer
static int TCP_Rx_bufferLength; // Number of bytes received at the TCP buffer

// Energy control
static rsuint8 is_suspended;


/****************************************************************************
*                            Local variables/const
****************************************************************************/
static RsListEntryType PtList; // Protothread control

#ifdef USE_LUART_TERMINAL  
// Buffers for LUART terminal I/O 
static char TmpStr[TMP_STR_LENGTH];
static char CmdStr[CMD_STR_LENGTH];
rsuint16 ShellRxIdx;

rsuint16 argc;
char *argv[MAX_ARGV];
#endif  

// TCP send/receive/ buffers
rsuint8 tx_buffer[TX_BUFFER_LENGTH];
rsuint8 rx_buffer[TX_BUFFER_LENGTH];

/****************************************************************************
*                                Implementation
***************************************************************************/

#ifdef USE_LUART_TERMINAL
/**
 * @brief Send a string to the dock (without adding a carriage return
 * and the end
 * @param str : string to print
 **/
static void UartPrint(rsuint8 *str) {
  DrvLeuartTxBuf(str, strlen((char*)str));
}

/**
 * @brief Send a string to the dock, adding a carriage return at the end
 * @param str : string to print
 **/
static void UartPrintLn(rsuint8 *str) {
  UartPrint(str);
  DrvLeuartTx('\r');
  DrvLeuartTx('\n');
}

/**
 * @brief Prints the IP and MAC addresses after DHCP or static config
 **/
static void print_IP_config(void) {
  const ApiWifiMacAddrType *pMacAddr = AppWifiGetMacAddr();

  if (AppWifiIpConfigIsStaticIp())
    PRINTLN("Static IP");
  else
    PRINTLN("DHCP");

  sprintf(TmpStr, "MAC: %02X-%02X-%02X-%02X-%02X-%02X", (*pMacAddr)[0],
    (*pMacAddr)[1], (*pMacAddr)[2], (*pMacAddr)[3], (*pMacAddr)[4],
    (*pMacAddr)[5]);
  PRINTLN(TmpStr);

  inet_ntoa(AppWifiIpv4GetAddress(), TmpStr);
  PRINT("IP: ");
  PRINTLN(TmpStr);
  //
  inet_ntoa(AppWifiIpv4GetSubnetMask(), TmpStr);
  PRINT("Subnet: ");
  PRINTLN(TmpStr);
  //
  inet_ntoa(AppWifiIpv4GetGateway(), TmpStr);
  PRINT("Gateway: ");
  PRINTLN(TmpStr);
  
  if (AppWifiIpv4GetPrimaryDns()) {
    inet_ntoa(AppWifiIpv4GetPrimaryDns(), TmpStr); 
    PRINT("Primary DNS: ");
    PRINTLN(TmpStr);
  }
  if (AppWifiIpv4GetSecondaryDns()) {
    inet_ntoa(AppWifiIpv4GetSecondaryDns(), TmpStr); 
    PRINT("Secondary DNS: ");
    PRINTLN(TmpStr);
  }
  PRINTLN("");
}

/**
 * @brief Prints the SSID of the connected WiFi network
 **/
static void print_SSID(void) {
  AppLedSetLedState(LED_STATE_CONNECTED);
  sprintf(TmpStr, "WiFi connected to SSID: %s\n",
    AppWifiGetSsid(AppWifiGetCurrentApIdx()));
  PRINTLN(TmpStr);
}

/**
 * @brief Checks if the given character is valid terminal input
 * @param c : character to check
 **/
static rsbool is_valid_cmd_char(char c) {
  if (isprint((rsuint8)c))
    return TRUE;

  switch (c) {
  case '\r':
  case 0x8: // backspace;
  case 0x7F: // backspace;
  case 0x1A: // ctrl-z
    return TRUE;
  }
  return FALSE;
}

/**
 * @brief Extract the terminal arguments for the given input
 * @param str : input string
 **/
rsuint16 ParseArgs(char *str, char *argv[]) {
  rsuint16 i = 0;
  char *ch = str;

  if (*ch == '\r')
    return 0;

  while (*ch != '\0') {
    i++;

    // Check if length exceeds
    if (i > MAX_ARGV) {
      #ifdef USE_LUART_TERMINAL
      PRINT("Too many arguments\n");
      #endif
      return 0;
    }

    argv[i - 1] = ch;
    while (*ch != ' ' && *ch != '\0' && *ch != '\r') {
      if (*ch == '"') { // Allow space characters inside double quotes
        ch++;
        argv[i - 1] = ch; // Drop the first double quote char
        while (*ch != '"') {
          if (*ch == '\0' || *ch == '\r') {
            #ifdef USE_LUART_TERMINAL
            PRINTLN("Syntax error");
            #endif
            return 0;
          }
          ch++; // Record until next double quote char
        }
        break;
      }
      else {
        ch++;
      }
    }

    if (*ch == '\r')
      break;

    if (*ch != '\0') {
      *ch = '\0';
      ch++;
      while (*ch == ' ') {
        ch++;
      }
    }
  }
  
  return i;
}
#endif

/**
 * @brief Helper function to extract a substring from a string
 * @param dest : extracted substring pointer
 * @param orig : input string pointer
 * @param orig_ptr : offset at the input string to extract next string
 **/
int extract_substring(rsuint8 *dest, rsuint8 *orig, int orig_ptr) {
  int dest_ptr = 0;
  
  if (orig[orig_ptr] == '\n' || orig[orig_ptr] == 0)
    return -1; // No more data
  
  while (orig[orig_ptr] != '\n' && orig[orig_ptr] != 0)
    dest[dest_ptr++] = orig[orig_ptr++];

  dest[dest_ptr] = 0; // End-of-string zero  
  return 1 + orig_ptr; // Return next position after '\n'
}

/**
 * @brief Saves the application info object contents to NVS
 **/
void Wifi_save_appInfo_to_NVS() {
  NvsWrite(NVS_OFFSET(Free),
           sizeof(app_dataType),
           (rsuint8*)&app_data);
}

/**
 * @brief Retrieves the application info object contents to NVS
 **/
void Wifi_read_appInfo_from_NVS() {
  NvsRead(NVS_OFFSET(Free),
          sizeof(app_dataType),
          (rsuint8 *)&app_data);
}

/**
 * @brief Fulfills an ApInfoType object from a string
 * @param ap_data : input string
 * @param ap_info : AP info object to fulfill
 **/
void get_ap_info_from_str(rsuint8 *ap_data, ApInfoType *ap_info) {
  int ap_ptr = 0;
  
  ap_info->KeyIndex = 0;

  // Extract SSID
  ap_ptr = extract_substring(ap_info->Ssid, ap_data, ap_ptr);
  ap_info->SsidLength = (rsuint8)strlen((char*)ap_info->Ssid);
  #ifdef USE_LUART_TERMINAL
  sprintf(TmpStr, "SSID=%s, ssid_len=%d", ap_info->Ssid, ap_info->SsidLength); PRINTLN(TmpStr);
  #endif

  // Extract encryption algorithm
  rsuint8 securityType_str[100];
  ap_ptr = extract_substring(securityType_str, ap_data, ap_ptr);
  #ifdef USE_LUART_TERMINAL
  sprintf(TmpStr, "Encryption=%s", securityType_str); PRINTLN(TmpStr);  
  #endif

  // Extract key
  ap_ptr = extract_substring(ap_info->Key, ap_data, ap_ptr);
  #ifdef USE_LUART_TERMINAL
  sprintf(TmpStr, "key=%s", ap_info->Key); PRINTLN(TmpStr);
  #endif
  //
  ap_info->KeyLength = (rsuint8)strlen((char*)ap_info->Key);

  // Set securityType and cipher according to securityType_str
  ap_info->SecurityType = AWST_NONE; // Just to prevent the warning
  if (!strcasecmp((char*)securityType_str, "WPA")) {
    #ifdef USE_LUART_TERMINAL
    PRINTLN("Set WPA");
    #endif
    ap_info->SecurityType = AWST_WPA;
    ap_info->Mcipher = AWCT_TKIP; 
    ap_info->Ucipher = AWCT_TKIP;
  }
  else if (!strcasecmp((char*)securityType_str, "WPA2")) {
    #ifdef USE_LUART_TERMINAL
    PRINTLN("Set WPA2");
    #endif
    ap_info->SecurityType = AWST_WPA2;
    ap_info->Mcipher = AWCT_CCMP; 
    ap_info->Ucipher = AWCT_CCMP;
  }
  else if (!strcasecmp((char*)securityType_str, "NONE")) {
    #ifdef USE_LUART_TERMINAL
    PRINTLN("Set encryption NONE");
    #endif
    ap_info->SecurityType = AWST_NONE;
  }

  // Extract extra parameter with encryption subalgorithm, if given
  ap_ptr = extract_substring(securityType_str, ap_data, ap_ptr);
  if (ap_ptr != -1) {
    #ifdef USE_LUART_TERMINAL
    PRINT("subAlgo="); PRINTLN(securityType_str);
    #endif
    
    // Update cipher subalgorithm
    if (!strcasecmp((char*)securityType_str, "TKIP")) {
      #ifdef USE_LUART_TERMINAL
      PRINTLN("Set TKIP");
      #endif
      ap_info->Mcipher = AWCT_TKIP; 
      ap_info->Ucipher = AWCT_TKIP;
    }
    else if (!strcasecmp((char*)securityType_str, "AES")) {
      #ifdef USE_LUART_TERMINAL
      PRINTLN("Set AES");
      #endif
      ap_info->Mcipher = AWCT_CCMP; 
      ap_info->Ucipher = AWCT_CCMP;
    }
    else {
      #ifdef USE_LUART_TERMINAL
      PRINT("Unknown encryption ");
      PRINTLN(securityType_str);
      #endif
    }    
  }
  #ifdef USE_LUART_TERMINAL
  else
    PRINTLN("No extra subalgo param");
  #endif
}

/**
 * @brief Setups an AP
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 * @param ap_data : input configuration string. If NULL, it will use
 * the data stored at the NVS to configure the AP
 **/
static PT_THREAD(PtWifi_setup_AP(struct pt *Pt, const RosMailType *Mail, rsuint8 *ap_data)) {
  // Examples of AP data strings:
  // "SSID\nWPA2\npassword\n"
  // "SSID\nWPA2\password\nAES\n"  
  
  static struct pt childPt;
  
  PT_BEGIN(Pt);

  ApInfoType *ap_info = &app_data.ap_info;
  // Decode AP configuration, only if a config. string is given.
  // If not, the default config (read from NVS at the beginning)
  // will be used.
  if (ap_data != NULL)
    get_ap_info_from_str(ap_data, ap_info);
   
  // Save AP information to NVS. Connect must be called afterwards
  rsuint8 ssid_len = (rsuint8)strlen((char*)ap_info->Ssid);
  
  ApiWifiCipherInfoType cipher;
  cipher.Ucipher = ap_info->Ucipher;
  cipher.Mcipher = ap_info->Mcipher;
  
  AppWifiSetApInfo(0, ssid_len, ap_info->Ssid, ap_info->SecurityType,
                   cipher, 0, ap_info->KeyLength, ap_info->Key);
  AppWifiWriteApInfoToNvs();

  // Store AP configuration, if a config. string is given
  if (ap_data == NULL) {
    #ifdef USE_LUART_TERMINAL
    sprintf(TmpStr, "NULL ap_data. Using SSID %s in NVS", ap_info->Ssid);
    PRINTLN(TmpStr);
    #endif
  }
  else
    Wifi_save_appInfo_to_NVS();

  // Disconnect, if associated to an old AP
  if (AppWifiIsAssociated()) {
    PT_SPAWN(Pt, &childPt, PtAppWifiDisconnect(&childPt, Mail));
    #ifdef USE_LUART_TERMINAL
    if (IS_RECEIVED(API_WIFI_DISCONNECT_IND)) {
      PRINTLN("Disconnected from old AP\n");
    }
    #endif
  }

  PT_END(Pt);
}

/**
 * @brief Suspends the WiFi chip
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 **/
static PT_THREAD(PtWifi_suspend(struct pt *Pt, const RosMailType *Mail)) {
  PT_BEGIN(Pt);
  is_suspended = true;
  POWER_TEST_PIN_TOGGLE;
  SendApiWifiSuspendReq(COLA_TASK, 10*60*1000); // ms
  PT_WAIT_UNTIL(Pt, IS_RECEIVED(API_WIFI_SUSPEND_CFM));
  POWER_TEST_PIN_TOGGLE;
  void EMU_EnterEM2();
  EMU_EnterEM2(); // uC enter suspend, too. Use an external interrupt to wake up!
  PT_END(Pt);
}


/**
 * @brief Resumes the suspended WiFi chip
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 **/
static PT_THREAD(PtWifi_resume(struct pt *Pt, const RosMailType *Mail)) {
  PT_BEGIN(Pt);
  POWER_TEST_PIN_TOGGLE;
  SendApiWifiResumeReq(COLA_TASK);
  PT_WAIT_UNTIL(Pt, IS_RECEIVED(API_WIFI_RESUME_CFM));
  POWER_TEST_PIN_TOGGLE;
  is_suspended = false;
  PT_END(Pt);
}

/**
 * @brief Sets the powersave profile
 * @param profile : powersave profile, 0: low power, 1: medium power,
 * 2: high power, 3: max power
 **/
void Wifi_set_power_save_profile(rsuint8 profile) {
  rsuint8 p;
  switch (profile) {
    case 0: p = POWER_SAVE_LOW_IDLE; break; // low power
    case 1: p = POWER_SAVE_MEDIUM_IDLE; break; // medium power
    case 2: p = POWER_SAVE_HIGH_IDLE; break; // high power
    case 3: p = POWER_SAVE_MAX_POWER; break; // max power
    default: p = 0xff;
  }
  
  if (p != 0xff)
    AppWifiSetPowerSaveProfile(p);
}

/**
 * @brief Associates and connects to an already configured AP
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 **/
static PT_THREAD(PtWifi_connect(struct pt *Pt, const RosMailType *Mail)) {
  static struct pt childPt;

  PT_BEGIN(Pt);
  
  if (!is_suspended) {
    Wifi_set_power_save_profile(3); // max power
    AppWifiSetTxPower(MAX_TX_POWER);
    
    // Avoid corrupt SSID
    SendApiWifiSetSsidReq(COLA_TASK, 0, NULL);
    PT_WAIT_UNTIL(Pt, IS_RECEIVED(API_WIFI_SET_SSID_CFM));
  
    // Read AP info
    #ifdef USE_LUART_TERMINAL
    PRINTLN("SendApiGetApinfoReq...");
    #endif
    SendApiGetApinfoReq(COLA_TASK);
    PT_YIELD_UNTIL(Pt, IS_RECEIVED(API_GET_APINFO_CFM));
    
    // Scan for known AP's
    #ifdef USE_LUART_TERMINAL
    PRINTLN("PtAppWifiScan...");
    #endif
    PT_SPAWN(Pt, &childPt, PtAppWifiScan(&childPt, Mail));

    // Connect to AP if it is available
    if (AppWifiIsApAvailable()) {
      #ifdef USE_LUART_TERMINAL
      PRINTLN("AppWifiIsApAvailable: YES");
      PRINTLN("PtWifi_connect spawning PtAppWifiConnect...");
      #endif

      AppLedSetLedState(LED_STATE_CONNECTING);

      // Wait 1s
      RosTimerStart(APP_PACKET_DELAY_TIMER, (1000 * RS_T1MS), &PacketDelayTimer);
      PT_WAIT_UNTIL(Pt, IS_RECEIVED(APP_PACKET_DELAY_TIMEOUT));  
      
      PT_SPAWN(Pt, &childPt, PtAppWifiConnect(&childPt, Mail));
      AppLedSetLedState(LED_STATE_IDLE);

      // Wait 2s
      RosTimerStart(APP_PACKET_DELAY_TIMER, (2000 * RS_T1MS), &PacketDelayTimer);
      PT_WAIT_UNTIL(Pt, IS_RECEIVED(APP_PACKET_DELAY_TIMEOUT));  
      
      if (AppWifiIsConnected()) {
        // Connected to AP
        #ifdef USE_LUART_TERMINAL
        print_SSID();
        print_IP_config();
        #endif
        PtMailHandled = TRUE;

        // Update DNS client with default gateway addr
        SendApiDnsClientAddServerReq(COLA_TASK, AppWifiIpv4GetGateway(), AppWifiIpv6GetAddr()->Gateway);
      }
      else {
        #ifdef USE_LUART_TERMINAL
        PRINTLN("Unable to connect");
        #endif
      }
    }
    else {
      #ifdef USE_LUART_TERMINAL
      PRINTLN("AppWifiIsApAvailable: NO");
      #endif
      // Avoid to store a corrupt SSID
      SendApiWifiSetSsidReq(COLA_TASK, 0, NULL);
      PT_WAIT_UNTIL(Pt, IS_RECEIVED(API_WIFI_SET_SSID_CFM));
    }    
  }

  PT_END(Pt);
}

/**
 * @brief Checks if associated and connected to the AP
 * @return True if associated and connected to the AP. False otherwise
 **/
rsuint8 Wifi_is_connected() {
  return AppWifiIsConnected();
}

/**
 * @brief Sends a query to close the currently open TCP connection
 **/
void Wifi_TCP_close() {
  if (is_suspended)
    return;
  SendApiSocketCloseReq(COLA_TASK, socketHandle);
  socketHandle = 0;
}

/**
 * @brief Sends len bytes in tx_buffer using the TCP connection
 * @param len : number of bytes to send
 **/
void Wifi_TCP_send(rsuint16 len) {
  if (is_suspended)
    return;

  #ifdef USE_LUART_TERMINAL
  PRINTLN("Send...");
  #endif
  SendApiSocketSendReq(COLA_TASK, socketHandle, tx_buffer, len, 0);
}

/**
 * @brief Receive data in the RX buffer. Must be called by the user when
 * it polls the status and sees that TCP_received is activated.
 * @param len : number of bytes to send
 **/
char Wifi_TCP_receive() {
  if (is_suspended)
    return false;
  
  if (!TCP_received) {
    #ifdef USE_LUART_TERMINAL
    PRINTLN("No TCP data received!");
    #endif
    return false;
  }

  #ifdef USE_LUART_TERMINAL
  sprintf(TmpStr, "TCP received BufferLength: %d", TCP_Rx_bufferLength);
  PRINTLN(TmpStr);

  int i;
  for (i = 0; i < TCP_Rx_bufferLength; i++) {
    char chr[] = {0, 0};
    chr[0] = rx_buffer[i];
    PRINT(chr);
  }
  PRINTLN("");
  #endif

  SendApiSocketFreeBufferReq(COLA_TASK,
                             socketHandle,
                             TCP_receive_buffer_ptr);
  TCP_received = false; // clear flag
  return true;
}

/**
 * @brief Obtains the system status
 * @return bit wise system status. Bit 0: Wifi connected,
 * bit 1: TCP connected, bit 1: TCP data received.
 **/
rsuint8 Wifi_get_status() {
  rsuint8 status = 0;
  status |= ((Wifi_is_connected() & 1) << 0);
  status |= ((TCP_is_connected & 1) << 1);
  status |= ((TCP_received & 1) << 2);
  status |= ((is_suspended & 1) << 3);
  return status;
}

/**
 * @brief Sets the transmit wireless transmit power
 * @param power : wireless transmit power
 **/
void Wifi_set_tx_power(rsuint8 power) {
  if (power > MAX_TX_POWER)
    power = MAX_TX_POWER;
  AppWifiSetTxPower(power);
}

/**
 * @brief Powers on/off the WiFi chip
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 * @param on : true for poweron, false for poweroff
 **/
static PT_THREAD(PtWifi_power_on_off(struct pt *Pt,
                 const RosMailType *Mail,
                 char on)) {
  static struct pt childPt;
  #ifdef USE_LUART_TERMINAL
  PRINTLN("SPAWN PtWifi_power_on_off");
  #endif

  PT_BEGIN(Pt);
  if (on)
    PT_SPAWN(Pt, &childPt, PtAppWifiPowerOn(&childPt, Mail));
  else
    PT_SPAWN(Pt, &childPt, PtAppWifiPowerOff(&childPt, Mail));
  PT_END(Pt);
}

/**
 * @brief Disassociates and disconnects from the WiFi AP
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 **/
static PT_THREAD(PtWifi_disconnect(struct pt *Pt, const RosMailType *Mail)) {
  static struct pt childPt;
  PT_BEGIN(Pt);  
  PT_SPAWN(Pt, &childPt, PtAppWifiDisconnect(&childPt, Mail));
  PT_END(Pt);
}

/**
 * @brief Configures the IP connection (DHCP, static)
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 * @param config : configuration string. NULL to read configuration from
 * the NVS
 **/
static PT_THREAD(PtWifi_IP_config(struct pt *Pt, const RosMailType *Mail, rsuint8 *config)) {
  static const char *ip_format = "%d.%d.%d.%d";
  static char buffer[30];

  PT_BEGIN(Pt);
  
  // Read IP config parameters
  char load_from_NVS = (config == NULL);
  if (load_from_NVS) {
    #ifdef USE_LUART_TERMINAL
    PRINTLN("Reading IP config info from NVS");
    #endif
    Wifi_read_appInfo_from_NVS();
  }
  else {
    app_data.use_dhcp = (config[0] == 'd' || config[0] == 'D');

    if (!app_data.use_dhcp) {
      // Extract IP address
      sprintf(buffer, ip_format, config[1], config[2], config[3], config[4]);
      inet_aton(buffer, &app_data.static_address.Ip.V4.Addr);
      #ifdef USE_LUART_TERMINAL
      sprintf(TmpStr, "IP address: %s", buffer); PRINTLN(TmpStr);
      #endif

      // Extract subnet
      sprintf(buffer, ip_format, config[5], config[6], config[7], config[8]);
      inet_aton(buffer, &app_data.static_subnet.Ip.V4.Addr);
      #ifdef USE_LUART_TERMINAL
      sprintf(TmpStr, "Subnet: %s", buffer); PRINTLN(TmpStr);    
      #endif
      
      // Extract gateway
      sprintf(buffer, ip_format, config[9], config[10], config[11], config[12]);
      inet_aton(buffer, &app_data.static_gateway.Ip.V4.Addr);
      #ifdef USE_LUART_TERMINAL
      sprintf(TmpStr, "Gateway: %s", buffer); PRINTLN(TmpStr);
      #endif
    }

    // Store IP config information to NVS
    Wifi_save_appInfo_to_NVS();
  }

  // Once the config has been read, do IP config now
  if (app_data.use_dhcp) {
    // DHCP
    #ifdef USE_LUART_TERMINAL
    PRINTLN("Do DHCP");
    #endif
    AppWifiIpv4Config(FALSE, 0, 0, 0, 0);
    if (AppWifiIsConnected())
      PT_WAIT_UNTIL(Pt, IS_RECEIVED(APP_EVENT_IP_ADDR_RECEIVED) ||
                        IS_RECEIVED(API_WIFI_DISCONNECT_IND));
  }
  else {
    // Static IP address
    #ifdef USE_LUART_TERMINAL
    PRINTLN("Do static IP");

    // Print IP address
    inet_ntoa(app_data.static_address.Ip.V4.Addr, buffer);
    sprintf(TmpStr, "IP address: %s", buffer); PRINTLN(TmpStr);

    // Print subnet
    inet_ntoa(app_data.static_subnet.Ip.V4.Addr, buffer);
    sprintf(TmpStr, "Subnet: %s", buffer); PRINTLN(TmpStr);    

    // Extract gateway
    inet_ntoa(app_data.static_gateway.Ip.V4.Addr, buffer);
    sprintf(TmpStr, "Gateway: %s", buffer); PRINTLN(TmpStr);
    #endif

    AppWifiIpv4Config(TRUE, app_data.static_address.Ip.V4.Addr,
                            app_data.static_subnet.Ip.V4.Addr,
                            app_data.static_gateway.Ip.V4.Addr, 0);
    AppWifiWriteStaticIpToNvs();
  }

  PT_END(Pt);
}

/**
 * @brief Resolves a domain name using the DNS service
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 * @param name : domain name to resolve
 * @param o_response : resolved IP address
 **/
static PT_THREAD(PtWifi_DNS_resolve(struct pt *Pt, const RosMailType *Mail, rsuint8 *name, rsuint32 *o_response)) {
  *o_response = 0;
  
  PT_BEGIN(Pt);
 
  SendApiDnsClientResolveReq(COLA_TASK, 0, strlen((char*)name), name);

  // Wait for response from DNS Client
  RosTimerStart(APP_DNS_RSP_TIMER, APP_DNS_RESOLVE_RSP_TIMEOUT, &DnsRspTimer);

  PT_WAIT_UNTIL(Pt, (IS_RECEIVED(API_DNS_CLIENT_RESOLVE_CFM) ||
                     IS_RECEIVED(APP_DNS_RSP_TIMEOUT)) &&
                     !PtMailHandled);
  PtMailHandled = TRUE;
  if (IS_RECEIVED(API_DNS_CLIENT_RESOLVE_CFM)) {
    RosTimerStop(APP_DNS_RSP_TIMER);
    if (((ApiDnsClientResolveCfmType *)Mail)->Status == RSS_SUCCESS) {
      #ifdef USE_LUART_TERMINAL
      PRINTLN("DNS success");
      #endif
      
      // Store the resolved IP (a 32-bit unsigned integer)
      *o_response = (rsuint32)((ApiDnsClientResolveCfmType *)Mail)->IpV4;

      char buffer[50];
      inet_ntoa(*o_response, buffer);

      #ifdef USE_LUART_TERMINAL
      sprintf(TmpStr, "DNS response for %s: %s", name, buffer);
      PRINTLN(TmpStr);
      #endif
    }
    else {
      #ifdef USE_LUART_TERMINAL
      PRINTLN("DNS Failed!");
      #endif
    }
  }
  else {
    #ifdef USE_LUART_TERMINAL
    PRINTLN("No response from DNS client");
    #endif
  }

  PT_END(Pt);
}

/**
 * @brief Event handled. Fired when the TCP connection has been
 * stablished
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 **/
static PT_THREAD(PtWifi_TCP_on_connect(struct pt *Pt, const RosMailType *Mail)) {
  AppSocketDataType *pInst = (AppSocketDataType *)PtInstDataPtr;
  socketHandle = pInst->SocketHandle;

  PT_BEGIN(Pt);
  #ifdef USE_LUART_TERMINAL
  PRINTLN("PtWifi_TCP_on_connect FIRED");
  #endif
  
  TCP_is_connected = true;
                     
  // Do not exit from the protothread until the TCP socket is closed
  PT_WAIT_UNTIL(Pt, IS_RECEIVED(API_SOCKET_CLOSE_IND));
  
  PT_END(Pt);
}

/**
 * @brief Starts a new TCP connection to the given server
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 * @param addr : server IP address and TCP port
 **/
static PT_THREAD(PtWifi_TCP_start(struct pt *Pt, const RosMailType *Mail, ApiSocketAddrType addr)) {
  #ifdef USE_LUART_TERMINAL
  AppSocketDataType *pInst = (AppSocketDataType *)PtInstDataPtr;
  #endif

  PT_BEGIN(Pt);
  
  if (!is_suspended) {
    TCP_is_connected = false;
    
    AppSocketStartTcpClient(&PtList, addr, PtWifi_TCP_on_connect);  

    #ifdef USE_LUART_TERMINAL
    if (pInst->LastError != RSS_SUCCESS) {
      sprintf(TmpStr, "AppSocketStartTcpClient ERROR! pInst->LastError == %d", pInst->LastError);
      PRINTLN(TmpStr);
    }
    #endif
  }

  PT_END(Pt);
}

//////////////////**************ALEJANDRO CASTRO IMPLEMENTATION***********///////////////////////
/*
 * @brief Sets the softAP mode
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
*/
static PT_THREAD(SoftAPMode(struct pt *Pt, const RosMailType *Mail)) {
    static struct pt childPt;
    PT_BEGIN(Pt);
    // Reset Wifi
    PT_SPAWN(Pt, &childPt, PtAppWifiReset(&childPt, Mail));
	
	// Set power save parameters  
    AppWifiSetPowerSaveProfile(POWER_SAVE_HIGH_IDLE); // 200ms idle interval
    AppWifiSetListenInterval(100);   	// 100ms     
    	
    const ApiWifiMacAddrType *pMacAddr = AppWifiGetMacAddr();
	
	//Set parameteres SoftAP
    //SSID = MASK + MAC
    rsuint8 ssid[32];	
    snprintf((char*)ssid, sizeof(ssid), "%s%02X%02X%02X", 
            SOFT_AP_ESSID, (*pMacAddr)[3], (*pMacAddr)[4], (*pMacAddr)[5]);
    AppWifiApSetSoftApInfo((rsuint8*)ssid, SOFT_AP_SECURITY_TYPE, 
            FALSE, 0, NULL, SOFT_AP_CHANNEL, SOFT_AP_INACT, 
            (rsuint8*)SOFT_AP_COUNTRY_CODE, SOFT_AP_BEACON);
    
    //Start SoftAP
    PT_SPAWN(Pt, &childPt, PtAppWifiStartSoftAp(&childPt, Mail));
    //static IP
    ApiIpV4AddressType address, subnetMask;
    inet_aton(SOFT_AP_STATIC_IP , &address);
    inet_aton(SOFT_AP_SUBNET_MASK, &subnetMask);
    AppWifiIpv4Config(true,address,subnetMask,address,address);
	//start SOFT Dhcp
    SendApiWifiApSetDhcpPoolReq(COLA_TASK, SOFT_AP_DHCP_POOL_START, SOFT_AP_DHCP_POOL_END, SOFT_AP_LEASE_TIME);     
    //wait a client
    PT_WAIT_UNTIL(Pt, IS_RECEIVED(API_WIFI_CONNECT_IND));

    PT_END(Pt);
}

// Definition Header HTTP
static rsuint32 AddHeader(rsuint8 *BufferPtr, rsuint32 BufferLength, rsuint8* DataPtr, rsuint32 Instance)
{
  rsuint8 *p = BufferPtr;
  rsuint32 l = 0;

  l += HttpAddHeader(p+l, BufferLength-l, "Content-Type", "text/html; charset=utf-8");
  l += HttpAddHeader(p+l, BufferLength-l, "Server", "RTX41xx demo WEB server");
  l += HttpAddHeader(p+l, BufferLength-l, "Date", app_data.DateStr);

  return l;
}
// Definition Main Page. Display temperatures of SHT21 and RTX4100
static rsuint32 GenerateMainPage(rsuint8 *BufferPtr, rsuint32 BufferLength, rsuint32 Offset, rsuint8 *DataPtr, rsuint32 Instance)
{
  
  rsuint32 n = 0;
  rsuint16 l = BufferLength;
  rschar *p = (rschar*)BufferPtr;
  const char* const start = 
"<!DOCTYPE html>"
"<html>"
"<head>"
	"<meta charset='UTF-8'>"
	"<title>RTX WEBSERVER</title>"
	"<style>"
        "body{"
            "font-family: nunito, sans-serif;"
            "background: #00131F;"
        "}"
        ".boton{"
            "font-size: 20px;"
            "background: #00131F;"
            "color:#329AB4;"
            "border: solid 3px #00131F;"
            "font-weight: bolder;"
            "transition: 2s;"
        "}"
        ".boton:hover{"
            "color:#fff;"
            "border-bottom: solid 3px #fff;"
        "}"
        ".selected{"
            "font-size: 20px;"
            "background: #002740;"
            "color:white;"
            "border: solid 3px #002740;"
            "font-weight: bolder;"
            "transition: 2s;"            
        "}"
        ".content{"
            "background: #002740;"
            "padding: 2em;"			
            "text-align: center;"
            "width: 100%;"
            "margin: 0 auto;"
        "}"
        ".content p{"
            "margin: 0 auto;"
            "margin-bottom: 1em;"
            "margin-top: 1em;"
            "background-color: #163B52;"
            "width: 50%s;"
            "padding: 1em;"
            "font-weight: bolder;"
            "color:#fff;"
            "border: solid 0px white;"
            "border-radius: 10px;"
            "box-shadow: 0 0 8px #414141;"
        "}"
        ".content p span{"
            "color:#66d5f0;"
			"font-size: 1.5em;"
        "}"
    "</style>"
"</head>"
"<body>"	
    "<div>"
        "<input class=\"selected\" type=\"button\" value=\"Inicio\">"
        "<input class=\"boton\" type=\"button\" value=\"Informacion\" onclick=\"location.href='Info'\">"		
        "<input class=\"boton\" type=\"button\" value=\"Soft AP Conf\" onclick=\"location.href='SoftAP'\">"


           "<div class=\"content\">"            
               "<p>Temperatura modulo:<br><br><span>%d.%d C</span></p>"
               "<p>Temperatura sensor:<br><br><span>%d C</span></p>"
               "<p>Humedad:<br><br><span>%d %s</span></p>"
        "</div> "
    "</div>"
"</body>"
"</html>";
  n = snprintf(p, l, start,"%",app_data.Temperature/10, app_data.Temperature%10,app_data.SPI1[0],app_data.SPI1[1],"%");
  return n;
}
// Definition Info Page. Displays Mac, last version of colapp, Wifi Hardware and Software Version and platform version. 
static rsuint32 Info(rsuint8 *BufferPtr, rsuint32 BufferLength, rsuint32 Offset, rsuint8 *DataPtr, rsuint32 Instance)
{
  
  rsuint32 n = 0;
  rsuint16 l = BufferLength;
  rschar *p = (rschar*)BufferPtr;
  const ApiWifiMacAddrType *pMacAddr = AppWifiGetMacAddr();
  const char* const start = 
"<!DOCTYPE html>"
"<html>"
"<head>"
	"<meta charset='UTF-8'>"
	"<title>RTX WEBSERVER</title>"
	"<style>"
        "body{"
            "font-family: nunito, sans-serif;"
            "background: #00131F;"
        "}"
        ".boton{"
            "font-size: 20px;"
            "background: #00131F;"
            "color:#329AB4;"
            "border: solid 3px #00131F;"
            "font-weight: bolder;"
            "transition: 2s;"
        "}"
        ".boton:hover{"
            "color:#fff;"
            "border-bottom: solid 3px #fff;"
        "}"
        ".selected{"
            "font-size: 20px;"
            "background: #002740;"
            "color:white;"
            "border: solid 3px #002740;"
            "font-weight: bolder;"
            "transition: 2s;"            
        "}"
        ".content{"
            "background: #002740;"
            "padding: 1em;"			
            "text-align: center;"
            "width: 100%;"
            "margin: 0 auto;"
			"font-size: 30px;"
			"font-weight: bolder;"
			"color:#fff;"	
        "}"
		".content p sub{"
            "color:#fff;"
			"font-size: 15px;"
        "}"
    "</style>"
"</head>"
"<body>"	
    "<div>"
        "<input class=\"boton\" type=\"button\" value=\"Inicio\" onclick=\"location.href='/'\">"        
        "<input class=\"selected\" type=\"button\" value=\"Informacion\" >"
           "<div class=\"content\">  "          
			   "<p>Informacion del sistema<br>"
			   "<sub>MAC: <font color=\"#66d5f0\">%02X-%02X-%02X-%02X-%02X-%02X</font></sub>" 
			   "<br><sub>WebServer Version: <font color=\"#66d5f0\">%s 20%02X-%02X-%02X %02X:%02X</font></sub><br> "
			   "<sub>Wifi Hw Version: <font color=\"#66d5f0\">%s</font></sub>" 
			   "<br><sub>Wifi SW Version: <font color=\"#66d5f0\">%s </font></sub><br> "
			   "<sub>Platform Version: <font color=\"#66d5f0\">%s</font></sub>" 
        "</div> "
    "</div>"
"</body>"
"</html>";


  n = snprintf(p, l, start,(*pMacAddr)[0], (*pMacAddr)[1], (*pMacAddr)[2], (*pMacAddr)[3], (*pMacAddr)[4], (*pMacAddr)[5],(const char*)ReleaseLabel,LinkDate[0], LinkDate[1], LinkDate[2], LinkDate[3], LinkDate[4],app_data.HWVERSION,app_data.SWVERSION,app_data.platform);

  return n;
}
// Definition of Configure Page. Configure SSID, Security Type and password. In SSID Input display the actual SSID.
static rsuint32 SOFTAPPage(rsuint8 *BufferPtr, rsuint32 BufferLength, rsuint32 Offset, rsuint8 *DataPtr, rsuint32 Instance)
{
  
  rsuint32 n = 0;
  rsuint16 l = BufferLength;
  rschar *p = (rschar*)BufferPtr;
  
  const char* const start = 
"<!DOCTYPE html>"
"<html>"
"<head>"
	"<meta charset='UTF-8'>"
	"<title>RTX WEBSERVER</title>"
	"<style>"
        "body{"
            "font-family: nunito, sans-serif;"
            "background: #00131F;"
        "}"
        ".boton{"
            "font-size: 20px;"
            "background: #00131F;"
            "color:#329AB4;"
            "border: solid 3px #00131F;"
            "font-weight: bolder;"
            "transition: 2s;"
        "}"
        ".boton:hover{"
            "color:#fff;"
            "border-bottom: solid 3px #fff;"
        "}"
        ".selected{"
            "font-size: 20px;"
            "background: #002740;"
            "color:white;"
            "border: solid 3px #002740;"
            "font-weight: bolder;"
            "transition: 2s;"            
        "}"
        ".content{"
            "background: #002740;"
            "padding: 1em;"			
            "text-align: left;"
            "width: 100%;"
            "margin: 0 auto;"
			"font-size: 30px;"
			"font-weight: bolder;"
			"color:#fff;"	
        "}"
        ".content span{"
            "color:#66d5f0;"
			"font-size: 15px;"
        "}"

    "</style>"
"</head>"
"<body>"	
    "<div>"
        "<input class=\"boton\" type=\"button\" value=\"Inicio\" onclick=\"location.href='/'\">"        
        "<input class=\"selected\" type=\"button\" value=\"Soft AP Conf\" >"
           "<div class=\"content\">  "          
               "<p>Configuracion Soft AP<br>"
			   "<form action=\"softap\" method=\"get\">"
			   "<br><span>SSID</span>  <input type=\"text\" size= 50 name=\"ssid\" value=\"%s\"> "
			   "<span>Seguridad</span>"
			   "<select name=\"Seguridad \">" 
			   "<option value=\"None\">None </option>" 
			   "<option value=\"WPA\">WPA</option>"
               "</select>"
			   "<br><span>Password</span>  <input type=\"text\" size= 50 name=\"ssid\" value=\"\"> "
			   "<br><input type=\"submit\" value=\"Save Settings\">"
			   "<form method=\"link\" action=\"/\">"
			   "</form>"   
        "</div> "
    "</div>"
"</body>"
"</html>";
 
  n = snprintf(p, l, start,AppWifiApGetSoftApSsid());
  return n;
}
//Resource Callback of GenerationMainPage. Generates the Main Page resource
static RsStatusType OnMainPage(AhHttpMethodIdType HttpMethod, rschar *PathPtr, rschar *QueryPtr, rsuint32 Instance)
{
  if (HttpMethod != AHM_GET)
  {
    return RSS_NOT_SUPPORTED;
  }

SendApiHttpServerSendResponseReq(COLA_TASK, Instance, 200, "OK", 
                                   GenerateMainPage(NULL, 0, 0, NULL, 0), // Calc the size of the main page
                                   NULL,                               // Static body not used  
                                   AddHeader,                          // Call back used to add HTTP headers 
                                   GenerateMainPage);                  // Callback used to generate the main page   
  return RSS_SUCCESS;
}
//Resource Callback of InfoPage. Generates the Info Page resource
static RsStatusType InfoPage(AhHttpMethodIdType HttpMethod, rschar *PathPtr, rschar *QueryPtr, rsuint32 Instance)
{
  if (HttpMethod != AHM_GET)
  {
    return RSS_NOT_SUPPORTED;
  }

SendApiHttpServerSendResponseReq(COLA_TASK, Instance, 200, "OK", 
                                   Info(NULL, 0, 0, NULL, 0), // Calc the size of the main page
                                   NULL,                               // Static body not used  
                                   AddHeader,                          // Call back used to add HTTP headers 
                                   Info);                  // Callback used to generate the main page   
  return RSS_SUCCESS;
}
//Resource Callback of SoftAPPAGE. Generates the Conf Page resource. 
static RsStatusType ConfSoftAP(AhHttpMethodIdType HttpMethod, rschar *PathPtr, rschar *QueryPtr, rsuint32 Instance)
{
  if (HttpMethod != AHM_GET)
  {
    return RSS_NOT_SUPPORTED;
  }

SendApiHttpServerSendResponseReq(COLA_TASK, Instance, 200, "OK", 
                                   SOFTAPPage(NULL, 0, 0, NULL, 0), // Calc the size of the main page
                                   NULL,                               // Static body not used  
                                   AddHeader,                          // Call back used to add HTTP headers 
                                   SOFTAPPage);                  // Callback used to generate the main page   
  return RSS_SUCCESS;
}  
/*// I try to implement application form and implement NVS system  but I don't have time :(:(//
static RsStatusType ConfSoftAP(AhHttpMethodIdType HttpMethod, rschar *PathPtr, rschar *QueryPtr, rsuint32 Instance)
{
  static const char* const okPage = 
    "<!DOCTYPE html>"
    "<html>"
      "<body>"
        "<h1>Se ha modificado los parametros correctament. Reinicia el modulo</h1>"
        "<form method=\"link\" action=\"/\">"
        "<input type=\"submit\" value=\"OK\">"
        "</form>"
      "</body>"
    "</html>";
  static const char* const failPage = 
    "<!DOCTYPE html>"
    "<html>"
      "<body>"
        "<h1>No has introducido la contraseña!</h1>"
        "<form method=\"link\" action=\"/SOFTAPPage\">"
        "<input type=\"submit\" value=\"OK\">"
        "</form>"
      "</body>"
    "</html>";
 AhQueryDataType query[3];
  rsuint8 queryCount = HttpSplitQueryString(3, query, QueryPtr);

  if (HttpMethod != AHM_GET)
  {
    return RSS_NOT_SUPPORTED;
  }

  if (queryCount == 3)
  {
    rschar ssid [25];
	rschar SecurityType [25];
    rschar Key [25];
    
    // Static ip ?
    if (query[0].ValuePtr[0] == '1')
    {
      ok = FALSE;
      if ((query[0].ValuePtr,)==NULL)
      {
        if ((query[1].ValuePtr)=="NONE")
        {
          if (query[2].ValuePtr)==NULL)
          {
            ok = TRUE;
          }
        }
      }
    }
    else
    {
      SendApiHttpServerSendResponseReq(COLA_TASK, Instance, 200, "OK", strlen(failPage), (rsuint8*)failPage, NULL, NULL);
	  return RSS_SUCCESS;
    }
  }
  else
  {
    SendApiHttpServerSendResponseReq(COLA_TASK, Instance, 200, "OK", SOFTAPPage(NULL, 0, 0, NULL, 0), NULL, NULL, SOFTAPPage);
	return RSS_SUCCESS;
  }
  return RSS_SUCCESS;
}
  
}  
*/

/**
 * @brief : It controls the SPI communication. This implementation reads commands, but the main protothread only reads temperatures
            In future works, the implementation of command SPI, will activate SOFT AP and Web Server modes and read temperatures.
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 **/
static PT_THREAD(SPI_temperatures(struct pt *Pt, const RosMailType *Mail)) {
    static struct pt childPt;
    PT_BEGIN(Pt);
    
    //SPI SLAVE
        //define baud rate
        const rsuint32 baud_rate = 9600;
        
		// starts  SPI slave driver
        PT_SPAWN(Pt, &childPt, PtDrvSpiSlaveInit(&childPt, Mail, baud_rate));
		// Defines a loop while command is equal command 16 or 17
        //if(app_data.SPI1[2]==16 || 17){	
             //while (app_data.SPI1[2]==16 || 17) {
			  while(1){
			  // waits to receibed a SPI data	 
			   PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_SLAVE_RX_DATA));
               RosTimerStart(APP_PACKET_DELAY_TIMER, (3000 * RS_T1MS), &PacketDelayTimer);
			   PT_YIELD_UNTIL(Pt, IS_RECEIVED(APP_PACKET_DELAY_TIMEOUT));  
               
             //reads and put data in buffer			   
               rsuint8 recibidospi[DrvSpiSlaveRxGetSize()];
               DrvSpiSlaveRx(&recibidospi[0], sizeof(recibidospi));
               
               app_data.SPI1[0] = recibidospi[0];
               app_data.SPI1[1] = recibidospi[1];
               app_data.SPI1[2] = recibidospi[2];  			
               
             // sends the same data to Arduino  
               DrvSpiSlaveTxStart(&app_data.SPI1[0], 3);
			 // empty the buffer  
			   DrvSpiSlaveRxFlush();
        }
/*Protothread exit if command is different that 16 or 17
 PT_EXIT(Pt);		
}
		// Defines a loop while command is different 16 or 17
        while (app_data.SPI1[2]!=16 || 17) {

			  PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_SLAVE_RX_DATA));
              RosTimerStart(APP_PACKET_DELAY_TIMER, (3000 * RS_T1MS), &PacketDelayTimer);
			  PT_YIELD_UNTIL(Pt, IS_RECEIVED(APP_PACKET_DELAY_TIMEOUT));  
              
              printf ("Intentando enviar datos...\n");
              
              rsuint8 recibidospi[DrvSpiSlaveRxGetSize()];
              DrvSpiSlaveRx(&recibidospi[0], sizeof(recibidospi));
              
              app_data.SPI1[0] = recibidospi[0];
              app_data.SPI1[1] = recibidospi[1];
              app_data.SPI1[2] = recibidospi[2];  			        
              
              DrvSpiSlaveTxStart(&app_data.SPI1[0], 3);
              printf("ENVIADO\n");
			  DrvSpiSlaveRxFlush();
        }
*/
    PT_END(Pt);
}

/**
 * @brief Main implementation protothread. Starts Soft AP, Intern Driver Temperature of RTX, Reads Wifi HW&SW and platform version,
          Starts HTTP server, add pages and starts the SPI communication
		  In future works, when command SPI works, this protothread only starts Http Server and add pages.
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 **/
 
static PT_THREAD(WebServer(struct pt *Pt, const RosMailType* Mail))
{
	static struct pt SensorUpdatePt;
	static struct pt childPt;
  PT_BEGIN(Pt);
    // Starts Soft Ap MODE
	PT_SPAWN(Pt, &childPt, SoftAPMode(&childPt, Mail));
	// Measure internal temperature inside the EFM32 chip
	PT_SPAWN(Pt, &SensorUpdatePt, PtDrvIntTempMeasure(&SensorUpdatePt, Mail, &app_data.Temperature));
	//Reads the Wifi Hardware and Software Version
	SendApiWifiGetVersionReq(COLA_TASK);
    PT_WAIT_UNTIL(Pt, IS_RECEIVED(API_WIFI_GET_VERSION_CFM));
    sprintf(app_data.SWVERSION, "%d.%d.%d.%d.%d",
          (int)(((ApiWifiGetVersionCfmType *)Mail)->SwVersion & 0xF0000000) >> 28,
          (int)(((ApiWifiGetVersionCfmType *)Mail)->SwVersion & 0x0F000000) >> 24,
          (int)(((ApiWifiGetVersionCfmType *)Mail)->SwVersion & 0x00FC0000) >> 18,
          (int)(((ApiWifiGetVersionCfmType *)Mail)->SwVersion & 0x0003FF00) >> 8,
          (int)(((ApiWifiGetVersionCfmType *)Mail)->SwVersion & 0x000000FF));
    sprintf(app_data.HWVERSION, "%d.%d.%d.%d.%d",
          (int)(((ApiWifiGetVersionCfmType *)Mail)->HwVersion & 0xF0000000) >> 28,
          (int)(((ApiWifiGetVersionCfmType *)Mail)->HwVersion & 0x0F000000) >> 24,
          (int)(((ApiWifiGetVersionCfmType *)Mail)->HwVersion & 0x00FC0000) >> 18,
          (int)(((ApiWifiGetVersionCfmType *)Mail)->HwVersion & 0x0003FF00) >> 8,
          (int)(((ApiWifiGetVersionCfmType *)Mail)->HwVersion & 0x000000FF));
	//Reads the Platform Version
    SendApiGetPlatformVersionReq(COLA_TASK);
    PT_WAIT_UNTIL(Pt, IS_RECEIVED(API_GET_PLATFORM_VERSION_CFM));
    sprintf(app_data.platform, "%X.%X.%X.%X (20%02X-%02X-%02X %02X:%02X)",
          ((ApiGetPlatformVersionCfmType *)Mail)->Version >> 8,
          (rsuint8)((ApiGetPlatformVersionCfmType *)Mail)->Version,
          ((ApiGetPlatformVersionCfmType *)Mail)->SubVersion,
          ((ApiGetPlatformVersionCfmType *)Mail)->BuildNumber,
          ((ApiGetPlatformVersionCfmType *)Mail)->LinkDate[0],
          ((ApiGetPlatformVersionCfmType *)Mail)->LinkDate[1],
          ((ApiGetPlatformVersionCfmType *)Mail)->LinkDate[2],
          ((ApiGetPlatformVersionCfmType *)Mail)->LinkDate[3],
          ((ApiGetPlatformVersionCfmType *)Mail)->LinkDate[4]);
  // Start HTTP server
  SendApiHttpServerInitReq(COLA_TASK, 80, NULL);
  PT_WAIT_UNTIL(Pt, IS_RECEIVED(API_HTTP_SERVER_INIT_CFM) && ((ApiHttpServerInitCfmType*)Mail)->Port == 80);
  if (((ApiHttpServerInitCfmType*)Mail)->Status == RSS_SUCCESS)
  {
    // Server started
    app_data.HttpServerStarted = TRUE;
    app_data.HttpInstance = ((ApiHttpServerInitCfmType*)Mail)->Instance;

    // Add main page
    SendApiHttpServerAddResourceReq(COLA_TASK, app_data.HttpInstance, "/", OnMainPage);
	SendApiHttpServerAddResourceReq(COLA_TASK, app_data.HttpInstance, "/Info", InfoPage);
	SendApiHttpServerAddResourceReq(COLA_TASK, app_data.HttpInstance, "/SoftAP", ConfSoftAP);	
  }
  PT_SPAWN(Pt, &childPt, SPI_temperatures(&childPt, Mail));
  PT_END(Pt);
}

//////////////////**************END IMPLEMENTATION***********///////////////////////
/**
 * @brief Test procedure which can be called from the debug terminal
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 **/
static PT_THREAD(PtTest(struct pt *Pt, const RosMailType *Mail)) {
  #ifdef USE_LUART_TERMINAL  
  static struct pt childPt;
  #endif  
  
  PT_BEGIN(Pt);

  #ifdef USE_LUART_TERMINAL  
  PRINTLN("*** This is PtTest");

  // Setup a new AP
  PRINTLN("* Test SPAWN PtWifi_setup_AP");
  //rsuint8 *ap_data = (rsuint8*)"SSID\nWPA2\npassword\nAES\n"; // ESSID, cipher, key
  rsuint8 *ap_data = NULL;
  //rsuint8 *ap_data = NULL;
  PT_SPAWN(Pt, &childPt, PtWifi_setup_AP(&childPt, Mail, ap_data));
  PRINTLN("");
  

  
  
  /*PRINTLN("* Test SPAWN PtWifi_IP_config static IP");
  rsuint8 config_static[] = {'s',
                             1, 2, 3, 4,     // IP address
                             5, 6, 7, 8,     // subnet
                             9, 10, 11, 12}; // gateway
  PT_SPAWN(Pt, &childPt, PtWifi_IP_config(&childPt, Mail, config_static));
  PRINTLN("");
  
  PRINTLN("* Test SPAWN PtWifi_IP_config static IP");
  // Read from NVS
  PT_SPAWN(Pt, &childPt, PtWifi_IP_config(&childPt, Mail, NULL));
  PRINTLN("");
  return;*/
  
  
  /*PRINTLN("* Test SPAWN PtWifi_IP_config DHCP");
  rsuint8 config_dhcp[] = {'d'};
  PT_SPAWN(Pt, &childPt, PtWifi_IP_config(&childPt, Mail, config_dhcp));
  PRINTLN("");*/

  PRINTLN("* Test SPAWN PtWifi_IP_config DHCP");
  // Read from NVS
  PT_SPAWN(Pt, &childPt, PtWifi_IP_config(&childPt, Mail, NULL));
  PRINTLN("");

  PRINTLN("* Test SPAWN PtWifi_connect");
  
  if (!Wifi_is_connected()) {
    PRINTLN("Not connected to AP");
    PT_SPAWN(Pt, &childPt, PtWifi_connect(&childPt, Mail));
  }
  else
    PRINTLN("Already connected to AP");
  PRINTLN("");
  
  /*PRINTLN("* Test SPAWN PtWifi_disconnect");
  PT_SPAWN(Pt, &childPt, PtWifi_disconnect(&childPt, Mail));
  PRINTLN("");
    
  PRINTLN("* Test SPAWN PtWifi_connect");
  PT_SPAWN(Pt, &childPt, PtWifi_connect(&childPt, Mail));
  PRINTLN("");
  

  PRINTLN("* Test SPAWN PtWifi_DNS_resolve");
  const char *name = "www.uoc.edu";
  rsuint32 response;
  PT_SPAWN(Pt, &childPt, PtWifi_DNS_resolve(&childPt, Mail, (rsuint8*)name, &response));
  sprintf(TmpStr, "response is %u", (size_t)response);
  PRINTLN(TmpStr);  
  PRINTLN("");*/


  /*PRINTLN("* Test SPAWN PtWifi_disconnect");
  PT_SPAWN(Pt, &childPt, PtWifi_disconnect(&childPt, Mail));
  PRINTLN("");
  
  PRINTLN("* Test SPAWN PtWifi_disconnect");
  PT_SPAWN(Pt, &childPt, PtWifi_disconnect(&childPt, Mail));
  PRINTLN("");*/
  #endif

  PT_END(Pt);
}



/**
 * @brief Main protothread. It controls the SPI or the debug terminal
 * @param Pt : current protothread pointer
 * @param Mail : protothread mail
 **/
static PT_THREAD(PtMain(struct pt *Pt, const RosMailType *Mail)) {
  static struct pt childPt;
  
  PT_BEGIN(Pt);
  
  // Read the app configuration from NVS
  Wifi_read_appInfo_from_NVS();
  
  // Reset the Atheros WiFi chip
  AppLedSetLedState(LED_STATE_ACTIVE);
  PT_SPAWN(Pt, &childPt, PtAppWifiReset(&childPt, Mail));
  SendApiCalibrateLfrcoReq(COLA_TASK, 3600); // Calibrate LFRCO every hour
  AppLedSetLedState(LED_STATE_IDLE);

  // Use LEUART1 for debug messages
  #ifdef USE_LUART_TERMINAL
  PT_SPAWN(Pt, &childPt, PtDrvLeuartInit(&childPt, Mail));
  #endif

  #ifdef USE_LUART_TERMINAL
  PRINTLN(""); PRINTLN("Ready"); PRINTLN("");
  #endif

  // Shell terminal (only if USE_LUART_TERMINAL defined)
  #ifdef USE_LUART_TERMINAL
  while (1) {
    //Flush UART RX buffer
    DrvLeuartRxFlush();
    #ifdef USE_LUART_TERMINAL
    PRINT("> ");
    #endif

    // Read from UART to we have a command line
    while (1) {
      char c;

      // read all
      while (DrvLeuartRx((rsuint8 *)&c, 1)) {
        if (is_valid_cmd_char(c)) {
          if (c == 0x8 || c == 0x7F) { // backspace
            if (ShellRxIdx) {
              ShellRxIdx--;
              DrvLeuartTx(c);
            }
          }
          else {
            // Echo char back to PC terminal
            DrvLeuartTx(c);

            // Check for  end of line
            if (c == '\r') {
              // end of line
              TmpStr[ShellRxIdx++] = '\0';
              goto ProcessCommandLine;
            }
            else {
              // Append char
              TmpStr[ShellRxIdx++] = c;
              if (ShellRxIdx == TMP_STR_LENGTH) {
                // Temp buffer full
                goto ProcessCommandLine;
              }
            }
          }
        }
      }

      // Allow other task to run
      PT_YIELD(Pt);
    }

  
  ProcessCommandLine:
    DrvLeuartTx('\n');

    // Process the command line received
    memcpy(CmdStr, TmpStr, ShellRxIdx);
    argc = ParseArgs(CmdStr, argv);
    if (argc) {
      if (strcmp(argv[0], "test") == 0) {
        PT_SPAWN(Pt, &childPt, PtTest(&childPt, Mail));
      }      
      else if (strcmp(argv[0], "tcpstart") == 0) {
        // Resolve DNS name
        const char *name = "www.example.com";
        rsuint32 response;
        PT_SPAWN(Pt, &childPt, PtWifi_DNS_resolve(&childPt, Mail, (rsuint8*)name, &response));
        sprintf(TmpStr, "DNS response (rsuint32) is %u", (size_t)response);
        PRINTLN(TmpStr);  
        PRINTLN("");        

        // Configure IP address
        ApiSocketAddrType addr;
        addr.Ip.V4.Addr = response;
        addr.Domain = ASD_AF_INET;
        addr.Port = 80;
        // Start TCP connection
        PT_SPAWN(Pt, &childPt, PtWifi_TCP_start(&childPt, Mail, addr));
      }
      else if (strcmp(argv[0], "status") == 0 || strcmp(argv[0], "s") == 0) {
        rsuint8 status = Wifi_get_status();
        sprintf(TmpStr, "status: %d", status); PRINTLN(TmpStr);
        sprintf(TmpStr, "Wifi_is_connected(): %d", status & 1 << 0); PRINTLN(TmpStr);
        sprintf(TmpStr, "TCP_is_connected: %d", status & 1 << 1); PRINTLN(TmpStr);
        sprintf(TmpStr, "TCP_received: %d", status & 1 << 2); PRINTLN(TmpStr);
        sprintf(TmpStr, "is_suspended: %d", status & 1 << 3); PRINTLN(TmpStr);

        AppSocketDataType *pInst = (AppSocketDataType *)PtInstDataPtr;
        sprintf(TmpStr, "pInst->LastError: %d", pInst->LastError); PRINTLN(TmpStr);
      }
      else if (strcmp(argv[0], "tcpclose") == 0) {
        Wifi_TCP_close();
      }
      else if (strcmp(argv[0], "disc") == 0) {
        PT_SPAWN(Pt, &childPt, PtWifi_disconnect(&childPt, Mail));
      }
      else if (strcmp(argv[0], "send") == 0) {
        PRINTLN("Send...");  
        strcpy((char*)tx_buffer, "GET / HTTP/1.0\n\n");
        size_t bytes_to_send = strlen((char*)tx_buffer);

        sprintf(TmpStr, "Sending %s (%d bytes)", (char*)tx_buffer, strlen((char*)tx_buffer));
        PRINTLN(TmpStr);        
        
        Wifi_TCP_send(bytes_to_send);
      }
      else if (strcmp(argv[0], "receive") == 0) {
        Wifi_TCP_receive();
      }
      else if (strcmp(argv[0], "disc") == 0) {
        if (AppWifiIsAssociated())
          PT_SPAWN(Pt, &childPt, PtAppWifiDisconnect(&childPt, Mail));
      }
      else if (strcmp(argv[0], "suspend") == 0) {
        PT_SPAWN(Pt, &childPt, PtWifi_suspend(&childPt, Mail));
      }
      else if (strcmp(argv[0], "resume") == 0) {
        PT_SPAWN(Pt, &childPt, PtWifi_resume(&childPt, Mail));
      }
      else
      {
        sprintf(TmpStr, "unknown cmd: %s", argv[0]);
        PRINTLN(TmpStr);
      }
    }
    ShellRxIdx = 0;
  }
  #endif
 
  #ifdef SPI_COMMUNICATION

  
//////////////********* ALEJANDRO COMMAND IMPLEMENTATION**************//////
// Whent Initialize SOFT AP or close HTTP server, the SPI comunication close too...No sense!!! >o<'
// Actually the application starts directly in Webserver Protothread!! Sorry guys! :(
  while (1) {

    // Wait until SPI data is received

	PT_SPAWN(Pt, &childPt, SPI_temperatures(&childPt, Mail));


    switch (app_data.SPI1[2]) {
      case 1: { // get status
        rsuint8 status = Wifi_get_status();
        DrvSpiTxStart(&status, 1);
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_TX_DONE));
        break;
      }
      case 2: { // DNS resolve
        // Read name to resolve (ex: "www.example.com")

        // First read the size of the name
        rsuint8 name_size;
        DrvSpiRx(&name_size, sizeof(name_size));
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));
        
        // Second, read the name
        rsuint8 name[100];
        DrvSpiRx((rsuint8*)&name, name_size);
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));
        name[name_size] = 0; // put trailing zero

        // Resolve
        rsuint32 response;
        PT_SPAWN(Pt, &childPt, PtWifi_DNS_resolve(&childPt, Mail, name, &response));
        
        // Send response
        DrvSpiTxStart((rsuint8*)&response, sizeof(response));
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_TX_DONE));
        break;
      }
      case 3: { // IP config
        // First read the size of the config
        rsuint8 config_size;
        DrvSpiRx(&config_size, sizeof(config_size));
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));

        // Second, read the config
        rsuint8 config[100];
        if (config_size > 0) {
          DrvSpiRx((rsuint8*)&config, config_size);
          PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));
        }

        // Do IP config        
        PT_SPAWN(Pt, &childPt, PtWifi_IP_config(&childPt, Mail,
                                      config_size > 0 ? config : NULL));
        break;
      }
      case 4: { // TCP start
        // Given the IP address of the server (rsuint32), start the connection.
        // The upper layer must poll in order to check when the connection
        // has been stablished.
        ApiSocketAddrType addr;
        addr.Domain = ASD_AF_INET;

        // Read the IP of the TCP server (rsuint32)
        DrvSpiRx((rsuint8*)&addr.Ip.V4.Addr, sizeof(addr.Ip.V4.Addr));
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));

        // Read the port of the TCP server
        DrvSpiRx((rsuint8*)&addr.Port, sizeof(addr.Port));
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));

        // Start TCP connection
        PT_SPAWN(Pt, &childPt, PtWifi_TCP_start(&childPt, Mail, addr));
        break;
      }
      case 5: { // Associate & connect to the WiFi AP
        PT_SPAWN(Pt, &childPt, PtWifi_connect(&childPt, Mail));
        break;
      }
      case 6: { // WiFi AP deassociate & disconnect
        PT_SPAWN(Pt, &childPt, PtWifi_disconnect(&childPt, Mail));
        break;
      }
      case 7: { // setup AP
        // Read ap_data size
        rsuint8 ap_data_size;
        DrvSpiRx(&ap_data_size, sizeof(ap_data_size));
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));
        
        // Read ap_data
        rsuint8 ap_data[100];
        if (ap_data_size > 0) {
          DrvSpiRx((rsuint8*)&ap_data, ap_data_size);
          PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));
        }        
        
        PT_SPAWN(Pt, &childPt, PtWifi_setup_AP(&childPt, Mail,
                                  ap_data_size > 0 ? ap_data : NULL));
        break;
      }
      case 8: { // TCP socket close
        Wifi_TCP_close();
        break;
      }
      case 9: { // TCP receive
        // The TCP data is already in rx_buffer
        // Number of bytes: TCP_Rx_bufferLength
        DrvSpiTxStart(rx_buffer, TCP_Rx_bufferLength);
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_TX_DONE));         
        break;
      }
      case 10: { // TCP send
        // Read the number of bytes to send (rsuint16)
        rsuint16 len;
        DrvSpiRx((rsuint8*)&len, sizeof(len));
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));
        
        if (len > TX_BUFFER_LENGTH)
          len = TX_BUFFER_LENGTH;
          
        // Read data to send into tx_buffer
        DrvSpiRx(tx_buffer, len);
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));
        
        // Send data using the TCP socket
        SendApiSocketSendReq(COLA_TASK, socketHandle, tx_buffer, len, 0);
        break;
      }
      case 11: { // Wifi chip power on/off        
        // Read parameter (0=off, 1=on)
        rsuint8 param;
        DrvSpiRx(&param, sizeof(param));
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));
        
        PT_SPAWN(Pt, &childPt, PtWifi_power_on_off(&childPt, Mail,
                                                   param));        
        break;
      }
      case 12: { // Wifi set powersave profile      
        // Read parameter
        // 0: low power, 1: medium power, 2: high power, 3: max power
        rsuint8 param;
        DrvSpiRx(&param, sizeof(param));
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));
        
        // Set powersave profile
        Wifi_set_power_save_profile(param);
        break;
      }
      case 13: { // Wifi set transmit power        
        // Read parameter
        rsuint8 param;
        DrvSpiRx(&param, sizeof(param));
        PT_WAIT_UNTIL(Pt, IS_RECEIVED(SPI_RX_DATA));
        
        // Set transmit power
        Wifi_set_tx_power(param);
        break;
      }
      case 14: { // Wifi chip suspend
        PT_SPAWN(Pt, &childPt, PtWifi_suspend(&childPt, Mail));
        break;
      }
      case 15: { // Wifi chip resume
        PT_SPAWN(Pt, &childPt, PtWifi_resume(&childPt, Mail));
        break;
      }
      case 16: { // Mode Soft AP . This mode init SoftAp mode, starts DHCP server,	
        //PT_SPAWN(Pt, &childPt, SoftAPMode(&childPt, Mail));
        break;
      }	  
      case 17: { // Mode Webserver. Initialize the WEB and read the temperatures
		/*PT_SPAWN(Pt, &childPt, WebServer(&childPt, Mail));
        PT_SPAWN(Pt, &childPt, SPI_temperatures(&childPt, Mail));
		if (app_data.HttpServerStarted)
        {
          SendApiHttpTerminateReq(COLA_TASK, 80);
          PT_WAIT_UNTIL(Pt, IS_RECEIVED(API_HTTP_TERMINATE_CFM) && ((ApiHttpTerminateCfmType*)Mail)->Instance == app_data.HttpInstance);
          app_data.HttpServerStarted = FALSE;
        }*/
        break;
      }
    }
  }
  #endif

  PT_END(Pt);
}

/**
 * @brief Main CoLa task event handler
 * @param Mail : protothread mail
 **/
void ColaTask(const RosMailType *Mail) {
  // Pre-dispatch mail handling
  switch (Mail->Primitive) {
    case INITTASK:
      // Init GPIO PIN used for timing of POWER measurements
      POWER_TEST_PIN_INIT;

      // Init the Buttons driver
      DrvButtonsInit();
	  
	  DrvIntTemp_Init();

      // Init the Protothreads lib
      PtInit(&PtList);

      // Init the LED application
      AppLedInit(&PtList);

      // Init the WiFi management application
      AppWifiInit(&PtList);

      // Start the Main protothread
      //PtStart(&PtList, PtMain, NULL, NULL);
      PtStart(&PtList, WebServer, NULL, NULL);
      break;

    case TERMINATETASK:
      RosTaskTerminated(ColaIf->ColaTaskId);
      break;
      
    case API_SOCKET_SEND_CFM:
      #ifdef USE_LUART_TERMINAL
      PRINTLN("API_SOCKET_SEND_CFM (send confirmation)");    
      
      if (((ApiSocketSendCfmType *)Mail)->Status == RSS_SUCCESS)
        PRINTLN("Send OK");
      else
      PRINTLN("Send ERROR");
      #endif
      break;

    case APP_EVENT_SOCKET_CLOSED:
      #ifdef USE_LUART_TERMINAL
      PRINTLN("APP_EVENT_SOCKET_CLOSED");
      #endif
      TCP_is_connected = false;
      break;    

    case API_SOCKET_CLOSE_IND:
      #ifdef USE_LUART_TERMINAL
      PRINTLN("API_SOCKET_CLOSE_IND");
      #endif
      TCP_is_connected = false;
      break;

    case API_SOCKET_RECEIVE_IND: {
      #ifdef USE_LUART_TERMINAL
        PRINTLN("API_SOCKET_RECEIVE_IND");
      #endif

      // Save pointer to TCP allocated buffer.
      // The buffer will be freed in Wifi_TCP_receive().
      ApiSocketReceiveIndType *socket = (ApiSocketReceiveIndType *)Mail;
      TCP_receive_buffer_ptr = socket->BufferPtr;
      TCP_Rx_bufferLength = socket->BufferLength;
      
      // Move data to rx_buffer
      TCP_Rx_bufferLength = socket->BufferLength;
      if (TCP_Rx_bufferLength >= TX_BUFFER_LENGTH)
        TCP_Rx_bufferLength = TX_BUFFER_LENGTH;
      memcpy(rx_buffer, socket->BufferPtr, TCP_Rx_bufferLength);

      // Activate the flag that indicates that TCP data has been received.
      // The buffer is not freed. The data must be read with Wifi_TCP_receive,
      // which will read the data clear and clear the buffer.    
      TCP_received = true;
      break;
    }
  }

  // Dispatch mail to all protothreads started
  PtDispatchMail(&PtList, Mail);
}

// End of file.
