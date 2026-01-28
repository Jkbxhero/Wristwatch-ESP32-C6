This is my first real project.  I started with vibe coding, as I am new to all of this and went through the docs and line by line to make a better functioning project.  Please feel free to amend or make changes.  For instance, I host my own NTP server as I need it for multiple projects.  

The hardware I changed so I could add a 1000 mAh battery.  The 400 mAh battery just wasn't enough.  Even with the sleep mode.  I am going to upload the STL file I used.  I am not sure if it will print with filament.  It likely will with some post printing work, but I do that with all of my models.  I chose resin, specifically ELEGOO tough black for this particular Use Case.  after sanding down the dimples I painted in flat black and added a satin clear coat as I know long term skin exposure likes to eat away at things.  Can't speak to longevity yet but after a couple weeks it is still holding up well.

## Setup Instructions

### 1. Configure WiFi Credentials

Copy the credentials template:
```bash
cp main/credentials_template.h main/credentials.h
```

Edit `main/credentials.h` with your WiFi details:
```c
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASS "YourPassword"
#define NTP_SERVER "pool.ntp.org"  // or your local NTP server
```


### 2. Build and Flash
```bash
idf.py build
idf.py flash monitor
```
```

              ✓ Setup instructions