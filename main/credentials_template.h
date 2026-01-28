#ifndef CREDENTIALS_H
#define CREDENTIALS_H

// WiFi Configuration Template
// Copy this file to "credentials.h" and fill in your own values

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
#define NTP_SERVER "pool.ntp.org"  // Or your local NTP server IP

// Timezone Configuration
// United States
// #define TIMEZONE "EST5EDT,M3.2.0,M11.1.0"   // Eastern (New York)
#define TIMEZONE "CST6CDT,M3.2.0,M11.1.0"   // Central (Chicago/Wichita)
// #define TIMEZONE "MST7MDT,M3.2.0,M11.1.0"   // Mountain (Denver)
// #define TIMEZONE "PST8PDT,M3.2.0,M11.1.0"   // Pacific (LA)
// #define TIMEZONE "MST7"                     // Arizona (no DST)
// #define TIMEZONE "AKST9AKDT,M3.2.0,M11.1.0" // Alaska
// #define TIMEZONE "HST10"                    // Hawaii (no DST)

// Europe
// #define TIMEZONE "GMT0BST,M3.5.0/1,M10.5.0" // UK (London)
// #define TIMEZONE "CET-1CEST,M3.5.0,M10.5.0/3" // Central Europe
// Format: https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html


#endif // CREDENTIALS_H
