#include <ESP8266WiFi.h> //Import two new library
#include <ESP8266WiFiMulti.h>   
#include <ESP8266WebServer.h>
// #include <SoftwareSerial.h>

#define WIFI_SSID "BillieGuo"
#define WIFI_PW "260116917" 

// 192.168.4.1


ESP8266WiFiMulti wifiMulti;
ESP8266WebServer esp8266_server(80);
// SoftwareSerial ArduinoSerial; // RX, TX

String webPage = "";
uint8_t videoFrame[1024] = {};

void setup() {
    pinMode(LED_BUILTIN, OUTPUT); // Set up pin IO modes
    digitalWrite(LED_BUILTIN, HIGH); // Turn off the lights
    Serial.begin(115200);
    // ArduinoSerial.begin(9600);
    
    WiFi.softAP("ESP8266", "12345678"); // Set up a hotspot with the name ESP8266 and password 12345678
    Serial.println("Hotspot started");
    Serial.print("IP address is: ");
    Serial.println(WiFi.softAPIP());

    // esp8266_server.on("/message", HTTP_GET, []() { // Handle HTTP GET request at /message
    // String message = esp8266_server.arg("msg"); // Get the value of the 'msg' parameter
    // Serial.println(message); // Print the message
    // esp8266_server.send(200, "text/plain", "Message received"); // Send a response
    // });

    // esp8266_server.begin(); // Start the server
    // Serial.println("Server started");
//    wifiMulti.addAP("BillieGuo", "260116917");  
//    wifiMulti.addAP("wifi2", "87654321"); 
//    wifiMulti.addAP("wifi3", "13572468"); 
    // WiFi.begin(WIFI_SSID, WIFI_PW); // Start Wi-Fi connection
    // while (WiFi.status() != WL_CONNECTED) { // Waiting for connection success
    //   delay(250);
    //   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //   Serial.println("Connecting to WiFi..");
    // }
    // Serial.println("Connected to the WiFi network");
    // delay(100);
    // digitalWrite(LED_BUILTIN, LOW); //Turn on the light to indicate Wi-Fi success
    // Serial.println("-------------------------");
    // Serial.println("Hello from ESP8266");
    // Serial.print("Connected to hotspot: ");
    // Serial.println(WIFI_SSID);
    // Serial.print("IP address is: ");
    // Serial.println(WiFi.localIP());
    // Serial.println("-------------------------");
  webPage += "<div align=\"center\"><h1>REAL TIME IMAGE</h1>";
  webPage += "<img id=\"img_stream\" width=\"640px\" height=\"640px\"></img>";
  webPage += "<div align=\"center\"><h1>ESP8266 CAR</h1>";
  webPage += "<a href=\"FL\"><button style=\"height:200px;width:200px\"><font size=\"20\">CCW</font></button></a>";
  webPage += "<a href=\"F2\"><button style=\"height:200px;width:200px\"><font size=\"20\">F</font></button></a>";
  webPage += "<a href=\"FR\"><button style=\"height:200px;width:200px\"><font size=\"20\">CW</font></button></a><br>";
  webPage += "<a href=\"L2\"><button style=\"height:200px;width:200px\"><font size=\"20\">L</font></button></a>";
  webPage += "<a href=\"S2\"><button style=\"height:200px;width:200px\"><font size=\"20\">STOP</font></button></a>";
  webPage += "<a href=\"R2\"><button style=\"height:200px;width:200px\"><font size=\"20\">R</font></button></a><br>";
  webPage += "<a href=\"BL\"><button style=\"height:200px;width:200px\"><font size=\"20\">AUTO</font></button></a>";
  webPage += "<a href=\"B2\"><button style=\"height:200px;width:200px\"><font size=\"20\">B</font></button></a>";
  webPage += "<a href=\"BR\"><button style=\"height:200px;width:200px\"><font size=\"20\">SW</font></button></a></div>";
  webPage += "<div align=\"center\"><h1>SPEED CHANGE</h1>";
  webPage += "<a href=\"SE1\"><button style=\"height:200px;width:200px\"><font size=\"20\">1</font></button></a>";
  webPage += "<a href=\"SE2\"><button style=\"height:200px;width:200px\"><font size=\"20\">2</font></button></a>";
  webPage += "<a href=\"SE3\"><button style=\"height:200px;width:200px\"><font size=\"20\">3</font></button></a><br>";
  webPage += "<script>
        let hexString =\"你图片的16进制字符串\";
        const imageUrl = imgRun(hexString);
        const imgElement = document.getElementById(\"img_stream\");
        imgElement.src = imageUrl;

        function imgRun(value) {
            let pos = 0;
            let len = value.length;
            if (len % 2 != 0) {
                return null;
            }
            len /= 2;
            const hex = [];
            for (let i = 0; i < len; i++) {
                const s = value.substr(pos, 2);
                const v = parseInt(s, 16);
                hex.push(v);
                pos += 2;
            }
            let binary = '';
            const bytes = new Uint8Array(hex);
            const len2 = bytes.byteLength;
            for (let i = 0; i < len2; i++) {
                binary += String.fromCharCode(bytes[i]);
            }
            return 'data:image/png;base64,' + window.btoa(binary);
        }
    </script>"
  // webPage += "<a href=\"SE4\"><button style=\"height:200px;width:200px\"><font size=\"20\">4</font></button></a>";
  // webPage += "<a href=\"SE5\"><button style=\"height:200px;width:200px\"><font size=\"20\">5</font></button></a>";
  // webPage += "<a href=\"SE6\"><button style=\"height:200px;width:200px\"><font size=\"20\">6</font></button></a><br>";
  // webPage += "<a href=\"SE7\"><button style=\"height:200px;width:200px\"><font size=\"20\">7</font></button></a>";
  // webPage += "<a href=\"SE8\"><button style=\"height:200px;width:200px\"><font size=\"20\">8</font></button></a>";
  // webPage += "<a href=\"SE9\"><button style=\"height:200px;width:200px\"><font size=\"20\">9</font></button></a>";
  //https://blog.csdn.net/gonepoo/article/details/107453041
  esp8266_server.on("/", []() {
    esp8266_server.send(200, "text/html", webPage);
  });
  esp8266_server.on("/video", handleVideo);
  esp8266_server.on("/FL", []() {
    esp8266_server.send(200, "text/html", webPage);
    // FL();
    Serial.println("FL");
    //ArduinoSerial.println("FL");
  });
  esp8266_server.on("/F2", []() {
    esp8266_server.send(200, "text/html", webPage);
    // F2();
    Serial.println("F2");
    //ArduinoSerial.println("F2");
  });
  esp8266_server.on("/FR", []() {
    esp8266_server.send(200, "text/html", webPage);
    // FR();
    Serial.println("FR");
    //ArduinoSerial.println("FR");
  });
  esp8266_server.on("/L2", []() {
    esp8266_server.send(200, "text/html", webPage);
    // L2();
    Serial.println("L2");
    //ArduinoSerial.println("L2");
  });
  esp8266_server.on("/S2", []() {
    esp8266_server.send(200, "text/html", webPage);
    // S2();
    Serial.println("S2");
    //ArduinoSerial.println("S2");
  });
  esp8266_server.on("/R2", []() {
    esp8266_server.send(200, "text/html", webPage);
    // R2();
    Serial.println("R2");
    //ArduinoSerial.println("R2");
  });
  esp8266_server.on("/BL", []() {
    esp8266_server.send(200, "text/html", webPage);
    // BL();
    Serial.println("BL");
    //ArduinoSerial.println("BL");
  });
  esp8266_server.on("/B2", []() {
    esp8266_server.send(200, "text/html", webPage);
    // B2();
    Serial.println("B2");
    //ArduinoSerial.println("B2");
  });
  esp8266_server.on("/BR", []() {
    esp8266_server.send(200, "text/html", webPage);
    // BR();
    Serial.println("BR");
    //ArduinoSerial.println("BR");
  });
  // speed setting
  esp8266_server.on("/SE1", []() {
    esp8266_server.send(200, "text/html", webPage);
    // CS(1);
    Serial.println("SE1");
    //ArduinoSerial.println("SE1");
  });
  esp8266_server.on("/SE2", []() {
    esp8266_server.send(200, "text/html", webPage);
    // CS(2);
    Serial.println("SE2");
    //ArduinoSerial.println("SE2");
  });
  esp8266_server.on("/SE3", []() {
    esp8266_server.send(200, "text/html", webPage);
    // CS(3);
    Serial.println("SE3");
    //ArduinoSerial.println("SE3");
  });
  esp8266_server.on("/SE4", []() {
    esp8266_server.send(200, "text/html", webPage);
    // CS(4);
    Serial.println("SE4");
    //ArduinoSerial.println("SE4");
  });
  esp8266_server.on("/SE5", []() {
    esp8266_server.send(200, "text/html", webPage);
    // CS(5);
    Serial.println("SE5");
    //ArduinoSerial.println("SE5");
  });
  esp8266_server.on("/SE6", []() {
    esp8266_server.send(200, "text/html", webPage);
    // CS(6);
    Serial.println("SE6");
    //ArduinoSerial.println("SE6");
  });
  esp8266_server.on("/SE7", []() {
    esp8266_server.send(200, "text/html", webPage);
    // CS(7);
    Serial.println("SE7");
    //ArduinoSerial.println("SE7");
  });
  esp8266_server.on("/SE8", []() {
    esp8266_server.send(200, "text/html", webPage);
    // CS(8);
    Serial.println("SE8");
    //ArduinoSerial.println("SE8");
  });
  esp8266_server.on("/SE9", []() {
    esp8266_server.send(200, "text/html", webPage);
    // CS (9);
    Serial.println("SE9");
    //ArduinoSerial.println("SE9");
  });

    esp8266_server.begin();                    
    esp8266_server.onNotFound(handleNotFound);        
    Serial.println("HTTP esp8266_server started");
}

void handleRoot() {  
  esp8266_server.send(200, "text/plain", "Hello from ESP8266");  

  Serial.println("connected.");
}
 
void handleNotFound(){                                        
  esp8266_server.send(404, "text/plain", "404: Not found");   
}

void handleVideo() {
  const char* videoData = reinterpret_cast<const char*>(videoFrame);
  esp8266_server.send(200, "text/plain", videoData);
}

void loop() {
    esp8266_server.handleClient(); 

    if (Serial.available() > 0) { // 判断是否有可用的串口数据
      String receivedData = ""; // 缓存清零
      receivedData = Serial.readString();
      receivedData.getBytes(videoFrame, receivedData.length() + 1);

      esp8266_server.send(200, "text/html", webPage);
      delay(2); // 延时等待响应
      if (receivedData.length() > 0) { // 如果receivedData有数据
        Serial.println(receivedData); // 打印receivedData数据
      }
    }
}
