#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <Servo.h>
#include <SoftwareSerial.h>
// #include <SolarPosition.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#define PI 3.14159265358979323846
Servo alt_Servo;
Servo azi_Servo;
const char* ssid       = "Sun Tracker";  // Tên SSID của trạm AP
const char* password   = "00000000";     // Mật khẩu của trạm AP
const int alt_servoPin = D5;
const int azi_servoPin = D6;
double global_lat      = 0.0;
double global_lon      = 0.0;
float alt_result;
float azi_result;
String global_uuid  = "";
String global_date  = "";
String global_time  = "";
String ssid_STA     = "SKYBOT";
String password_STA = "skybot@2023";
String message;
String app_script =
    "https://script.google.com/macros/s/"
    "AKfycbwbQCfjtduMRfhy9w4ewlmd6NqzcDpj1cFauRv2GtZfzQrp0JQZeG8QLoHn-mxpMsJX/"
    "exec";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600);  // GMT+7 = 7 * 3600
struct sunPos {
    float sun_azi;
    float sun_alt;
};
sunPos sun;
long JulianDate1(int year, int month, int day) {
    int a = (14 - month) / 12;
    int y = year + 4800 - a;
    int m = month + 12 * a - 3;
    return day + (153 * m + 2) / 5 + 365 * y + y / 4 - 32083;
}
sunPos calculateSolarPosition(const String& latlon,
                              const String& raw_datetime) {
    sunPos result;
    // Parse latitude and longitude from latlon string
    float Latitude  = latlon.substring(0, latlon.indexOf(',')).toFloat();
    float Longitude = latlon.substring(latlon.indexOf(',') + 1).toFloat();

    // Parse datetime from raw_datetime string
    struct tm timeinfo;
    strptime(raw_datetime.c_str(), "%Y-%m-%dT%H:%M:%S.000Z", &timeinfo);

    // Convert tm structure to time_t
    time_t tParam = mktime(&timeinfo);

    // Constants and variables for solar position calculation
    const float DAYS_PER_JULIAN_CENTURY = 36525.0;
    const long Y2K_JULIAN_DAY           = 2451545;

    long JD_whole;
    float JD_frac;
    float rightAscension;
    float Declination;
    float hourAngle;
    float GreenwichHourAngle;
    float elapsedT;
    float solarLongitude;
    float solarMeanAnomaly;
    float earthOrbitEccentricity;
    float sunCenter;
    float solarTrueLongitude;
    float equatorObliquity;

    // Calculate Julian Date
    JD_whole = JulianDate1(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
                           timeinfo.tm_mday);
    JD_frac =
        (timeinfo.tm_hour + timeinfo.tm_min / 60.0 + timeinfo.tm_sec / 3600.0) /
            24.0 -
        0.5;

    elapsedT = JD_whole - Y2K_JULIAN_DAY;
    elapsedT = (elapsedT + JD_frac) / DAYS_PER_JULIAN_CENTURY;

    // Calculate solar coordinates
    solarLongitude = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * elapsedT, 360);
    solarMeanAnomaly = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * elapsedT, 360);
    earthOrbitEccentricity = 0.016708617 - 0.000042037 * elapsedT;

    sunCenter = DEG_TO_RAD *
                ((1.9146 - 0.004847 * elapsedT) * sin(solarMeanAnomaly) +
                 (0.019993 - 0.000101 * elapsedT) * sin(2 * solarMeanAnomaly) +
                 0.00029 * sin(3 * solarMeanAnomaly));

    equatorObliquity = DEG_TO_RAD * (23 + 26 / 60.0 + 21.448 / 3600.0 -
                                     46.815 / 3600.0 * elapsedT);

    GreenwichHourAngle =
        280.46061837 + 360.98564736629 * (JD_whole - Y2K_JULIAN_DAY + JD_frac);
    GreenwichHourAngle = fmod(GreenwichHourAngle, 360.0);

    solarTrueLongitude = solarLongitude + sunCenter;
    rightAscension     = atan2(sin(solarTrueLongitude) * cos(equatorObliquity),
                               cos(solarTrueLongitude));
    Declination        = asin(sin(equatorObliquity) * sin(solarTrueLongitude));
    hourAngle = DEG_TO_RAD * GreenwichHourAngle + Longitude - rightAscension;

    // Calculate elevation and azimuth
    result.sun_alt =
        asin(sin(Latitude * DEG_TO_RAD) * sin(Declination) +
             cos(Latitude * DEG_TO_RAD) * cos(Declination) * cos(hourAngle));
    result.sun_azi =
        PI + atan2(sin(hourAngle),
                   cos(hourAngle) * sin(Latitude * DEG_TO_RAD) -
                       tan(Declination) * cos(Latitude * DEG_TO_RAD));

    // Convert azimuth to degrees
    result.sun_azi = result.sun_azi * 180.0 / PI;

    return result;
}
struct PayloadData {
    String A;
    String B;
    String C;
    String D;
    String E;
};
String mode_controll = "GG";
PayloadData nowpayload;
unsigned long global_millis = 0;
WiFiClientSecure client;
ESP8266WebServer server(3000);
struct DateTime {
    int year;
    int month;
    int day;
    int hour;
    int min;
    int sec;
};
struct Position {
    double lat;
    double lon;
};
struct SolarPositionData {
    double altitude;
    double azimuth;
};

void control_servo() {
    if (sun.sun_azi >= 270 && sun.sun_azi <= 360) {  // quad 4
        azi_result = sun.sun_azi - 180;
        alt_result = sun.sun_alt;
    } else if (sun.sun_azi >= 0 && sun.sun_azi <= 90) {  // quad 1
        azi_result = 90 - sun.sun_azi;
        alt_result = sun.sun_alt;
    } else if (sun.sun_azi >= 90 && sun.sun_azi <= 180) {  // quad 2
        azi_result = sun.sun_azi;
        alt_result = sun.sun_alt + 90;
    } else {
        azi_result = sun.sun_azi - 180;
        alt_result = sun.sun_alt + 90;
    }
    // }

    alt_Servo.write(alt_result);
    azi_Servo.write(azi_result);
    Serial.print("alt Servo :");
    Serial.println(alt_result);
    Serial.print("azi Servo :");
    Serial.println(azi_result);
}

void handleRoot() {
    String html = R"=====(
    <!DOCTYPE html>
<html lang="en">
<head>
     <meta charset="UTF-8">
     <meta name="viewport" content="width=100%, initial-scale=1.0">
     <title>Sun Tracker</title>
     <style>
          body {
               background-color: #3db1ce;
               display: flex;
               flex-direction: row;
               align-items: center;
               justify-content: space-around;
               padding-top: 10vh;
               font-size: 15px;
               font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto,
                    Oxygen, Ubuntu, Cantarell, "Open Sans", "Helvetica Neue", sans-serif;
          }
          
          .sec {
               display: block;
               position: relative;
               border: 5px solid black;
               border-radius: 10px;
               padding: 20px;
               background-color: rgb(26, 196, 63);
               color: hsl(0, 0%, 19%);
               box-shadow: -20px 20px 20px rgb(206, 141, 21);
          }
          
          .button {
               align-items: center;
               background-color: #fee6e3;
               border: 2px solid #111;
               border-radius: 8px;
               box-sizing: border-box;
               color: #111;
               cursor: pointer;
               display: flex;
               font-family: Inter, sans-serif;
               font-size: 16px;
               height: 48px;
               justify-content: center;
               line-height: 24px;
               max-width: 100%;
               padding: 0 25px;
               position: relative;
               text-align: center;
               text-decoration: none;
               user-select: none;
               -webkit-user-select: none;
               touch-action: manipulation;
          }
          
          .button:after {
               background-color: #111;
               border-radius: 8px;
               content: "";
               display: block;
               height: 48px;
               left: 0;
               width: 100%;
               position: absolute;
               top: -2px;
               transform: translate(8px, 8px);
               transition: transform 0.2s ease-out;
               z-index: -1;
          }
          
          .button:hover:after {
               transform: translate(0, 0);
          }
          
          .button:active {
               background-color: #ffdeda;
               outline: 0;
          }
          
          .button:hover {
               outline: 0;
          }
          
          @media (min-width: 768px) {
               .button {
                    padding: 0 40px;
               }
          }
          
          .input {
               padding: 5px;
               border: solid 2px rgb(199, 29, 29);
               border-radius: 5px;
               background-color: chocolate;
               color: rgb(56, 56, 56);
               font-size: medium;
               font-weight: 700;
               width: fit-content;
          }
          
          .sectitle {
               text-align: center;
          }
          
          .submitbtn {
               background-color: rgb(206, 141, 21);
               font-weight: 900;
          }
     </style>
</head>
<body>
     <div class="sec sec2">
          <h2 class="sectitle">BẢNG CÀI ĐẶT</h2>
          <button class="button" onclick="getLocationDateTime()">Lấy Vị Trí, Ngày và Thời Gian Hiện Tại</button><br>
          <button class="button" onclick="getSystemDateTime()">Lấy Thời Gian Hiện Tại</button><br>
          <form action="/senddata" method="post" onsubmit="handleSubmit(event)">
               <label class="label" for="lat">Vĩ độ (Lat):</label><br>
               <input class="input" type="number" step="any" id="lat" name="lat"><br><br>

               <label class="label" for="lon">Kinh độ (Lon):</label><br>
               <input class="input" type="number" step="any" id="lon" name="lon"><br><br>

               <label class="label" for="date">Ngày (Date):</label><br>
               <input class="input" type="date" id="date" name="date"><br><br>

               <label class="label" for="time">Thời gian (Time):</label><br>
               <input class="input" type="time" id="time" name="time" step="1"><br><br>

               <input class="button submitbtn" type="submit" value="SEND">
          </form>
     </div>
     <div class="sec sec2">
          <h2 class="sectitle">Buttons</h2>
          <button onclick='buttonClick(1)'>1</button>
          <button onclick='buttonClick(2)'>2</button>
          <button onclick='buttonClick(3)'>3</button>
          <button onclick='buttonClick(4)'>4</button>
     </div>

     <script>
          function buttonClick(buttonNumber) {
               alert('Button ' + buttonNumber + ' clicked');
          }
          function getLocationDateTime() {
               if (navigator.geolocation) {
                    navigator.geolocation.getCurrentPosition(function (position) {
                         var lat = position.coords.latitude;
                         var lon = position.coords.longitude;
                         // Gán giá trị vào các input
                         document.getElementById("lat").value = lat;
                         document.getElementById("lon").value = lon;
                    });
               } else {
                    alert("Trình duyệt của bạn không hỗ trợ định vị.");
               }
          }
          function getSystemDateTime() {
               var currentDate = new Date();
               var date = currentDate.toISOString().split('T')[0]; // Lấy ngày YYYY-MM-DD
               var time = currentDate.toTimeString().split(' ')[0]; // Lấy thời gian hh:mm:ss

               // Gán giá trị vào các input
               document.getElementById("date").value = date;
               document.getElementById("time").value = time;
               startClock();
          }

          function startClock() {
               // Thiết lập bộ đếm thời gian liên tục
               setInterval(getSystemDateTime, 1000); // Cập nhật thời gian mỗi 1 giây (1000 ms)
          }
          function handleSubmit(event) {
               event.preventDefault(); // Ngăn chặn hành vi mặc định của form (reload trang)

               // Lấy dữ liệu từ form
               const formData = new FormData(event.target);
               const data = {};
               formData.forEach((value, key) => {
                    data[key] = value;
               });

               // Gửi dữ liệu đi với fetch API hoặc AJAX
               fetch('/senddata', {
                    method: 'POST',
                    body: JSON.stringify(data),
                    headers: {
                         'Content-Type': 'application/json'
                    }
               }).then(response => response.json())
                    .then(data => {
                         console.log('Success:', data);
                         // Xử lý kết quả trả về nếu cần thiết
                    })
                    .catch((error) => {
                         console.error('Error:', error);
                    });
          }
     </script>
</body>
</html>
    )=====";

    server.send(200, "text/html", html);
}
void handleGGSheet() {
    std::unique_ptr<BearSSL::WiFiClientSecure> client(
        new BearSSL::WiFiClientSecure);
    client->setInsecure();
    HTTPClient https;
    String url = app_script + "?read=true";
    Serial.println("Reading Data From Google Sheet.....");
    Serial.print("URL: ");
    Serial.println(url);
    https.begin(*client, url.c_str());
    //-----------------------------------------------------------------------------------
    // Removes the error "302 Moved Temporarily Error"
    https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    //-----------------------------------------------------------------------------------
    // Get the returning HTTP status code
    int httpCode = https.GET();
    Serial.print("HTTP Status Code: ");
    Serial.println(httpCode);
    //-----------------------------------------------------------------------------------
    if (httpCode <= 0) {
        Serial.println("Error on HTTP request");
        https.end();
        return;
    }
    String payload = https.getString();
    if (httpCode == 200) message = payload;
    Serial.print(payload);
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
    }
    nowpayload.B = doc["B"].as<String>();
    nowpayload.C = doc["C"].as<String>();
    nowpayload.A = doc["A"].as<String>();
    nowpayload.D = doc["D"].as<String>();
    nowpayload.E = doc["E"].as<String>();
    https.end();
}
String formatTime(time_t rawTime) {
    struct tm* timeinfo;
    timeinfo = gmtime(&rawTime);  // Sử dụng gmtime để lấy thời gian UTC

    char buffer[30];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02d.000Z",
             timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
             timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    return String(buffer);
}
void setup() {
    Serial.begin(38400);
    alt_Servo.attach(alt_servoPin, 500, 2500);
    azi_Servo.attach(azi_servoPin, 500, 2500);
    azi_Servo.write(0);
    alt_Servo.write(0);
    WiFi.softAP(ssid, password);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    WiFi.begin(ssid_STA, password_STA);
    //     WiFi.begin("Galaxy S22 Ultra C5C9", "stsvn2024");

    // Chờ kết nối
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(WiFi.localIP());
    timeClient.begin();
    timeClient.update();
    delay(5000);
}

void sendDatatoSheet(String uuid, String no, String latlon, float alt,
                     float azi, String timestamp, String device, String user) {
    std::unique_ptr<BearSSL::WiFiClientSecure> client(
        new BearSSL::WiFiClientSecure);
    client->setInsecure();
    HTTPClient https;
    https.begin(*client, app_script);
    https.addHeader("Content-Type", "application/json");
    String datasend = "{\"uuid\":\"" + uuid + "\",\"no\":\"" + no +
                      "\",\"latlon\" : \"" + latlon + "\",\"alt\":\"" + alt +
                      "\",\"azi\":\"" + azi + "\",\"timestamp\":\"" +
                      timestamp + "\",\"device\": \"" + device +
                      "\",\"user\": \"" + user + "\"}";
    https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpResponseCode = https.POST(datasend);
    Serial.println(datasend);
    if (httpResponseCode > 0) {
        Serial.print("code: ");
        Serial.println(httpResponseCode);
        String response = https.getString();
        // Serial.println("post google sheet ok!");
    } else {
        // Serial.print("Error on sending POST: ");
        Serial.println(httpResponseCode);
    }
    https.end();
}
void loop() {
    unsigned long currentMillis = millis();

    // Đảm bảo mỗi lần lặp ít nhất 1 giây
    if (currentMillis - global_millis >= 2000 && mode_controll == "GG") {
        time_t epochtime      = timeClient.getEpochTime();
        String formatDatetime = formatTime(epochtime);
        handleGGSheet();
        sunPos position = calculateSolarPosition(nowpayload.B, formatDatetime);
        // sunPos position = calculateSolarPosition(nowpayload.B, nowpayload.E);
        sun.sun_alt = position.sun_alt * 180.0 / PI;
        sun.sun_azi = position.sun_azi;
        control_servo();
        sendDatatoSheet(nowpayload.A, "0", nowpayload.B, sun.sun_alt,
                        sun.sun_azi, formatDatetime, "GATE_ST_001",
                        "hieunguyen130701iuh@gmail.com");
        global_millis = currentMillis;
    }
}
