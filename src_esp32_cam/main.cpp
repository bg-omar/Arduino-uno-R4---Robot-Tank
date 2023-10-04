/*
UNO BLE -->     DC:54:75:C3:D9:EC   -
PC USB Dongel   00:1F:E2:C8:82:BA
ESP 1           66:CB:3E:E9:02:8A
ESP             3c:e9:0e:89:80:84
ESP small cam   3C:E9:0E:88:65:16
PS4 Controller: A4:AE:11:E1:8B:B3 (SONYWA) GooglyEyes
PS5 Controller: 88:03:4C:B5:00:66
*/
/********************************************** Setup booting the arduino **************************************/
// section Defines
/***************************************************************************************************************/

//#define USE_WIFI_PHOTO
#define USE_WIFI_VIDEO

/********************************************** Setup booting the arduino **************************************/
// section Includes
/***************************************************************************************************************/

#include <arduino.h>
#include <FS.h>
#include "secrets.h"
#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

#ifdef USE_WIFI_PHOTO
    #include "WifiCam.hpp"
    #include <WiFi.h>
    static const char* WIFI_SSID = SECRET_WIFI_SSID;
    static const char* WIFI_PASS = SECRET_WIFI_PASS;
    esp32cam::Resolution initialResolution;
    WebServer server(80);
#endif

#ifdef USE_WIFI_VIDEO
    #include "esp_camera.h"
    #include <WiFi.h>
    #include "esp_timer.h"
    #include "img_converters.h"
    #include "fb_gfx.h"
    #include "soc/soc.h" //disable brownout problems
    #include "soc/rtc_cntl_reg.h"  //disable brownout problems
    #include "esp_http_server.h"

    //Replace with your network credentials
    const char* ssid = SECRET_WIFI_SSID;
    const char* password = SECRET_WIFI_PASS;

    #define PART_BOUNDARY "123456789000000000000987654321"

    #define CAMERA_MODEL_AI_THINKER
    #define PWDN_GPIO_NUM     32
    #define RESET_GPIO_NUM    -1
    #define XCLK_GPIO_NUM      0
    #define SIOD_GPIO_NUM     26
    #define SIOC_GPIO_NUM     27

    #define Y9_GPIO_NUM       35
    #define Y8_GPIO_NUM       34
    #define Y7_GPIO_NUM       39
    #define Y6_GPIO_NUM       36
    #define Y5_GPIO_NUM       21
    #define Y4_GPIO_NUM       19
    #define Y3_GPIO_NUM       18
    #define Y2_GPIO_NUM        5
    #define VSYNC_GPIO_NUM    25
    #define HREF_GPIO_NUM     23
    #define PCLK_GPIO_NUM     22

    static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
    static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
    static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    httpd_handle_t stream_httpd = nullptr;

    static esp_err_t stream_handler(httpd_req_t *req){
        camera_fb_t * fb = nullptr;
        esp_err_t res = ESP_OK;
        size_t _jpg_buf_len = 0;
        uint8_t * _jpg_buf = nullptr;
        char * part_buf[64];

        res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
        if(res != ESP_OK){
            return res;
        }

        while(true){
            fb = esp_camera_fb_get();
            if (!fb) {
                Serial.println("Camera capture failed");
                res = ESP_FAIL;
            } else {
                if(fb->width > 400){
                    if(fb->format != PIXFORMAT_JPEG){
                        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                        esp_camera_fb_return(fb);
                        fb = nullptr;
                        if(!jpeg_converted){
                            Serial.println("JPEG compression failed");
                            res = ESP_FAIL;
                        }
                    } else {
                        _jpg_buf_len = fb->len;
                        _jpg_buf = fb->buf;
                    }
                }
            }
            if(res == ESP_OK){
                size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
                res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
            }
            if(res == ESP_OK){
                res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
            }
            if(res == ESP_OK){
                res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
            }
            if(fb){
                esp_camera_fb_return(fb);
                fb = nullptr;
                _jpg_buf = nullptr;
            } else if(_jpg_buf){
                free(_jpg_buf);
                _jpg_buf = nullptr;
            }
            if(res != ESP_OK){
                break;
            }
            //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
        }
        return res;
    }

    void startCameraServer(){
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port = 80;

        httpd_uri_t index_uri = {
                .uri       = "/",
                .method    = HTTP_GET,
                .handler   = stream_handler,
                .user_ctx  = nullptr
        };

        Serial.printf("server: '%d'\n", config.server_port);
        if (httpd_start(&stream_httpd, &config) == ESP_OK) {
            httpd_register_uri_handler(stream_httpd, &index_uri);
        }
    }
#endif
void setupWiFi() {
    #ifdef USE_WIFI_PHOTO
        WiFi.persistent(false);
            WiFiClass::mode(WIFI_STA);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            if (WiFi.waitForConnectResult() != WL_CONNECTED) {
                send("WiFi failure");
                delay(5000);
                ESP.restart();
            }
            send("WiFi connected");

            {
                using namespace esp32cam;

                initialResolution = Resolution::find(1024, 768);

                Config cfg;
                cfg.setPins(pins::AiThinker);
                cfg.setResolution(initialResolution);
                cfg.setJpeg(80);

                bool ok = Camera.begin(cfg);
                if (!ok) {
                    send("camera initialize failure");
                    delay(5000);
                    ESP.restart();
                }
                send("camera initialize success");
            }

            send("camera starting");
            Serial.print("http://");
            Serial.print(WiFi.localIP());

            addRequestHandlers();
            server.begin();
    #endif
    #ifdef USE_WIFI_VIDEO
        WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
        Serial.setDebugOutput(false);

        camera_config_t config;
        config.ledc_channel = LEDC_CHANNEL_0;
        config.ledc_timer = LEDC_TIMER_0;
        config.pin_d0 = Y2_GPIO_NUM;
        config.pin_d1 = Y3_GPIO_NUM;
        config.pin_d2 = Y4_GPIO_NUM;
        config.pin_d3 = Y5_GPIO_NUM;
        config.pin_d4 = Y6_GPIO_NUM;
        config.pin_d5 = Y7_GPIO_NUM;
        config.pin_d6 = Y8_GPIO_NUM;
        config.pin_d7 = Y9_GPIO_NUM;
        config.pin_xclk = XCLK_GPIO_NUM;
        config.pin_pclk = PCLK_GPIO_NUM;
        config.pin_vsync = VSYNC_GPIO_NUM;
        config.pin_href = HREF_GPIO_NUM;
        config.pin_sccb_sda = SIOD_GPIO_NUM;
        config.pin_sccb_scl = SIOC_GPIO_NUM;
        config.pin_pwdn = PWDN_GPIO_NUM;
        config.pin_reset = RESET_GPIO_NUM;
        config.xclk_freq_hz = 20000000;
        config.pixel_format = PIXFORMAT_JPEG;

        if(psramFound()){
            config.frame_size = FRAMESIZE_SVGA; //FRAMESIZE_UXGA;
            config.jpeg_quality = 10;
            config.fb_count = 1;
        } else {
            config.frame_size = FRAMESIZE_SVGA;
            config.jpeg_quality = 10;
            config.fb_count = 1;
        }

        // Camera init
        esp_err_t err = esp_camera_init(&config);
        if (err != ESP_OK) {
            Serial.printf("Camera init failed with error 0x%x", err);
            return;
        }
        // Wi-Fi connection
        WiFi.begin(ssid, password);
        while (WiFiClass::status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("");
        Serial.println("WiFi connected");

        Serial.println("Camera Stream: ");
        Serial.println(WiFi.localIP());

        // Start streaming web server
        startCameraServer();
    #endif

    }
/********************************************** Setup booting the arduino **************************************/
// section Variables & Defines
/***************************************************************************************************************/

#define EVENTS 1
#define BUTTONS 0
#define JOYSTICKS 1
#define SENSORS 0

unsigned long lastTimeStamp = 0;

int lastBattery;

#ifdef TESTO_PESTO
    const uint32_t message_interval = 5000;
    const char* emojis[8] = {
            "🌟🌟🌟", "😎👌🔥", "✅", "🇺🇦🛡️🇺🇦🛡️🇺🇦",
            "🤓", "¯\\_(ツ)_/¯", "🚀🌘", "🤯"
    };
#endif
/********************************************** Setup booting the arduino **************************************/
// section Function Declarations
/***************************************************************************************************************/


/********************************************** Setup booting the arduino **************************************/
// section Functions
/***************************************************************************************************************/


void onConnect() {
    lastBattery = PS4.Battery();
    Serial.println(&"PS4 Connected!" [ PS4.Battery()]);
    Serial.println("");
}

void onDisConnect() {
    Serial.println("Disconnected!");
}

void send(char32_t texting) {
    Serial.println(texting);
}


void notify() {
#if EVENTS
    boolean sqd = PS4.event.button_down.square,     squ = PS4.event.button_up.square,
            trd = PS4.event.button_down.triangle,   tru = PS4.event.button_up.triangle,
            crd = PS4.event.button_down.cross,      cru = PS4.event.button_up.cross,
            cid = PS4.event.button_down.circle,     ciu = PS4.event.button_up.circle,
            upd = PS4.event.button_down.up,         upu = PS4.event.button_up.up,
            rid = PS4.event.button_down.right,      riu = PS4.event.button_up.right,
            dod = PS4.event.button_down.down,       dou = PS4.event.button_up.down,
            led = PS4.event.button_down.left,       leu = PS4.event.button_up.left,
            l1d = PS4.event.button_down.l1,         l1u = PS4.event.button_up.l1,
            r1d = PS4.event.button_down.r1,         r1u = PS4.event.button_up.r1,
            l3d = PS4.event.button_down.l3,         l3u = PS4.event.button_up.l3,
            r3d = PS4.event.button_down.r3,         r3u = PS4.event.button_up.r3,
            psd = PS4.event.button_down.ps,         psu = PS4.event.button_up.ps,
            tpd = PS4.event.button_down.touchpad,   tpu = PS4.event.button_up.touchpad,
            opd = PS4.event.button_down.options,    opu = PS4.event.button_up.options,
            shd = PS4.event.button_down.share,      shu = PS4.event.button_up.share;

    if      (sqd) send(3110);
    else if (squ) send(3101);
    else if (crd) send(3210);
    else if (cru) send(3201);
    else if (cid) send(3310);
    else if (ciu) send(3301);
    else if (trd) send(3410);
    else if (tru) send(3401);
    else if (upd) send(1110);
    else if (upu) send(1101);
    else if (rid) send(1210);
    else if (riu) send(1201);
    else if (dod) send(1310);
    else if (dou) send(1301);
    else if (led) send(1410);
    else if (leu) send(1401);

    else if (l1d) send(2110);
    else if (l1u) send(2101);
    else if (r1d) send(2210);
    else if (r1u) send(2201);
    else if (l3d) send(2310);
    else if (l3u) send(2301);
    else if (r3d) send(2410);
    else if (r3u) send(2401);
    else if (psd) send(2510);
    else if (psu) send(2501);
    else if (tpd) send(2710);
    else if (tpu) send(2701);
    else if (shd) send(2810);
    else if (shu) send(2801);
    else if (opd) send(2910);
    else if (opu) send(2901);

#endif

#if BUTTONS
    boolean sq = PS4.Square(),
              tr = PS4.Triangle();
      if (sq)
        Serial.print(" SQUARE pressed");
      if (tr)
        Serial.print(" TRIANGLE pressed");
      if (sq | tr)
        send();
#endif

#if JOYSTICKS
    Serial.printf("lx:%4d, ly:%4d, rx:%4d, ry:%4d, l2:%4d, r2:%4d \r\n",
                  (PS4.LStickX() <= -15 || PS4.LStickX() >= 15 ) ? 6127 + PS4.LStickX() : 0,
                  (PS4.LStickY() <= -15 || PS4.LStickY() >= 15 ) ? 7127 + PS4.LStickY() : 0,
                  (PS4.RStickX() <= -15 || PS4.RStickX() >= 15 ) ? 8127 + PS4.RStickX() : 0,
                  (PS4.RStickY() <= -15 || PS4.RStickY() >= 15 ) ? 9127 + PS4.RStickY() : 0,
                  (PS4.L2()) ? PS4.L2Value() : 0,
                  (PS4.R2()) ? PS4.R2Value() : 0);
#endif

#if SENSORS
    Serial.printf("gx:%5d,gy:%5d,gz:%5d,ax:%5d,ay:%5d,az:%5d ",
            PS4.GyrX(),
            PS4.GyrY(),
            PS4.GyrZ(),
            PS4.AccX(),
            PS4.AccY(),
            PS4.AccZ());
#endif
    lastTimeStamp = millis();
}


/********************************************** Setup booting the arduino **************************************/
// section Setup
/***************************************************************************************************************/

void setup() {
    Serial.begin(115200);
    PS4.attach(notify);
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisConnect);
    PS4.begin("3C:E9:0E:88:65:16"); //mac Address that ESP should use

    /* Remove Paired Devices  */
    uint8_t pairedDeviceBtAddr[20][6];
    int count = esp_bt_gap_get_bond_device_num();
    esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    for (int i = 0; i < count; i++) {
        esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
    }
    Serial.println("my BT mac -> 3C:E9:0E:88:65:16");
    Serial.println("Ready for PS4");

    delay(500);
    setupWiFi();

}

/*********************************** Loop **********************************/
// section Loop
/***************************************************************************/

void loop() {
    #ifdef USE_WIFI_PHOTO
        server.handleClient();
    #endif

    #ifdef TESTO_PESTO
        const uint32_t now = millis();
        static uint32_t last_message = 0;
        if (now - last_message > message_interval) {
            last_message += message_interval;

            Serial.printf("PESTO!!! ESP32-CAM! %s\r\n", emojis[random(8)]);
        }

        if (Serial.available()) {
            char data = Serial.read();
            // Process the received data here
            Serial.print(data);
            // Echo back the data to the serial port
            Serial.write(data);
        }
    #endif

    // Below has all accessible outputs from the controller
    if (PS4.isConnected()) {
        if (PS4.Up())        send(1100);
        if (PS4.Right())     send(1200);
        if (PS4.Down())      send(1300);
        if (PS4.Left())      send(1400);

        if (PS4.UpRight())   send(1500);
        if (PS4.DownRight()) send(1600);
        if (PS4.DownLeft())  send(1700);
        if (PS4.UpLeft())    send(1800);

        if (PS4.L1())        send(2100);
        if (PS4.R1())        send(2200);

        if (PS4.L3())        send(2300);
        if (PS4.R3())        send(2400);

        if (PS4.PSButton())  send(2500);
        if (PS4.Touchpad())  send(2700);

        if (PS4.Share())     send(2800);
        if (PS4.Options())   send(2900);

        if (PS4.Square())    send(3100);
        if (PS4.Cross())     send(3200);
        if (PS4.Circle())    send(3300);
        if (PS4.Triangle())  send(3400);

        if (PS4.Charging())  send(3500);
        if (PS4.Audio())     send(3600);
        if (PS4.Mic())       send(3700);
        if (PS4.Battery() < lastBattery) send(3900 + PS4.Battery());

//#if JOYSTICKS
//#else
        if (PS4.L2()) { Serial.println(4000 + PS4.L2Value());  }
        if (PS4.R2()) { Serial.println(5000 + PS4.R2Value());  }

        if (PS4.LStickX() <= -15 || PS4.LStickX() >= 15 ) {
            Serial.println(6127 + PS4.LStickX());  // 6 000 - 6 254
        }
        if (PS4.LStickY() <= -15 || PS4.LStickY() >= 15 ) {
            Serial.println(7127 + PS4.LStickY());  // 7 000 - 7 254
        }
        if (PS4.RStickX() <= -15 || PS4.RStickX() >= 15 ) {
            Serial.println(8127 + PS4.RStickX());  // 8 000 - 8 254
        }
        if (PS4.RStickY() <= -15 || PS4.RStickY() >= 15 ) {
            Serial.println(9127 + PS4.RStickY());  // 9 000 - 9 254
        }
//#endif
        delay(3);
    }
}
