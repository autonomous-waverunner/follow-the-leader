#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <SoftwareSerial.h>
#include <FS.h>

IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);    // create a websocket server on port 81

#define LED D4  
#define LED_D0 LED_BUILTIN        // Led in NodeMCU at pin GPIO16 (D0).
SoftwareSerial nodemcu (13, 15, false, 256);

char const webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
   <head>
      <title>Follow the leader</title>
      <style>
      body{font-family: "Trebuchet MS", Arial, Helvetica, sans-serif;}
      textarea#rxConsole {
        width: 500px;
        height: 220px;
        border: 3px solid #cccccc;
        padding: 5px;
        font-family: Tahoma, sans-serif;
        background-position: bottom right;
        display: block;
        outline: none;
      }
  
      textarea {
        height:auto;
        max-width:600px;
        font-size:13px;
        width:100%;
        display:block;
      }

      h1  {
        font-family: "Trebuchet MS", Arial, Helvetica, sans-serif;
        font-size: 44px;
        font-weight: bold;
        margin-top: 0px;
        margin-bottom: 1px;
      }
      
      button {
        outline: none !important;
        border: none;
        background: transparent;
      }
      
      button:hover {
        cursor: pointer;
      }
      
      .center {
        margin: auto;
        width: auto;
        border: 3px solid black;
        padding: 10px;
      }

      label {display:block;}
  
      .customers {
        font-family: "Trebuchet MS", Arial, Helvetica, sans-serif;
        border-collapse: collapse;
        width: 100%;
        cellpadding: 5px;
        cellspacing : 5px;
      }

      .customers td, .customers th {
        border: 1px solid #ddd;
        padding: 8px;
      }
    
      .customers tr:nth-child(even){background-color: #f2f2f2;}
    
      .customers tr:hover {background-color: #ddd;}
    
      .customers th {
        padding-top: 12px;
        padding-bottom: 12px;
        text-align: left;
        background-color: #4CAF50;
        color: white;
      }
    </style>
    <script>
    var websock;
         
    function init(){
      
      var host = window.location.hostname;
 
      websock = new WebSocket('ws://' + host + ':81/');
      
      websock.onopen = function(evt) {
          console.log('websock open');
      };
         
      websock.onerror = function(evt) {
          console.log(evt);
      };
         
      websock.onmessage = function(evt) {
          console.log(evt);
          return false;
      };
             
      websock.onclose = function(){
          console.log('websock close');
      };
    }
         
    function buttonclick() {
         
            var DistRef     = document.getElementById("DistRefValue").value || 0;
            var deltaT      = document.getElementById("deltaTValue").value || 0;
            var KpGas       = document.getElementById("KpGasValue").value || 0;
            var KiGas       = document.getElementById("KiGasValue").value || 0;
            var KdGas       = document.getElementById("KdGasValue").value || 0;
            var TfGas       = document.getElementById("TfGasValue").value || 0;
            var maxGas      = document.getElementById("maxGasValue").value || 0;
            var minGas      = document.getElementById("minGasValue").value || 0;
            var filterGas   = document.getElementById("filterGasValue").value || 0;
            var KpAngle     = document.getElementById("KpAngleValue").value || 0;
            var KiAngle     = document.getElementById("KiAngleValue").value || 0;
            var KdAngle     = document.getElementById("KdAngleValue").value || 0;
            var TfAngle     = document.getElementById("TfAngleValue").value || 0;
            var maxAngle    = document.getElementById("maxAngleValue").value || 0;
            var minAngle    = document.getElementById("minAngleValue").value || 0;
            var filterAngle = document.getElementById("filterAngleValue").value || 0;
            var GasPID      = document.getElementById("GasPIDValue").value || 0;
            var AnglePID    = document.getElementById("AnglePIDValue").value || 0;
         
            var str = [DistRef, deltaT, KpGas, KiGas, KdGas, TfGas, maxGas, minGas, filterGas, KpAngle, KiAngle, KdAngle, TfAngle, maxAngle, minAngle, filterAngle, GasPID, AnglePID];
           
            var ct = new Date();
         
            document.getElementById("rxConsole").value += '\n' + "[ " + ct.getHours() + ":" + ct.getMinutes() + ":" + ct.getSeconds() + " ] -> " + str.join();
            document.getElementById("rxConsole").scrollTop = document.getElementById("rxConsole").scrollHeight

            var str_p = "P," + str.join();
            console.log(str_p);
             
            websock.send(str_p);
        }
    
        function ResetSTM32(){
            websock.send("SystemReset");
        }
         
        function clearConsole(){
            document.getElementById('rxConsole').value = '';
        }
      </script>
   </head>
   <body onload="javascript:init();">
      <center>
         <div>
            <h1>Follow the leader : OTAT</h1>
      <div class="center">
        <table width="95%" cellspacing="10" cellpadding="10" >
          <tr>
            <td colspan="2" align="center">
              <!-- General parameters -->
              <table border="1px" cellpadding="2" cellspacing="2"  class="customers">
                <col width="25%">
                <col width="75%">
                <tbody>
                  <tr>
                    <th colspan="3">General parameters</th>
                  </tr>
                  <tr>
                    <td>Distance ref:</td>
                    <td><input id="DistRefValue" type="number" value="0" /></td>
                  </tr>
                  <tr>
                    <td>&Delta;T:</td>
                    <td><input id="deltaTValue" type="number" value="0" /></td>
                  </tr>
                </tbody>
              </table>
            </td>
          </tr>
          <tr>
            <td width="50%" align="center">
              <!-- Throttle parameters -->
              <table border="1px" cellpadding="2" cellspacing="2" class="customers">
                <tbody>
                  <tr>
                    <th colspan="3">Throttle parameters</th>
                  </tr>
                  <tr>
                    <td>Kp gas:</td>
                    <td><input id="KpGasValue" type="number" value="0" /></td>
                  </tr>
                  <tr>
                    <td>Ki gas:</td>
                    <td><input id="KiGasValue" type="number" value="0" /></td>
                  </tr>
                  <tr>
                    <td>Kd gas:</td>
                    <td><input id="KdGasValue" type="number" value="0" /></td>
                  </tr>
                  <tr>
                    <td>Tf gas:</td>
                    <td><input type="number" id="TfGasValue" value="0"></td>
                  </tr>
                  <tr>
                    <td>max gas:</td>
                    <td><input type="number" id="maxGasValue" value="0"></td>
                  </tr>
                  <tr>
                    <td>min gas:</td>
                    <td><input type="number" id="minGasValue" value="0"></td>
                  </tr>
                  <tr>
                    <td>filter gas:</td>
                    <td><input type="number" id="filterGasValue" value="0"></td>
                  </tr>
                  <tr>
                    <td>GAS PID scaler:</td>
                    <td><input type="number" id="GasPIDValue" value="0"></td>
                  </tr>
                </tbody>
              </table>
            </td>
            <td width="50%" align="center">
              <!-- Nozzle parameters -->
              <table border="1px" cellpadding="2" cellspacing="2" class="customers">
                <tbody>
                  <tr>
                    <th colspan="3">Nozzle parameters</th>
                  </tr>
                  <tr>
                    <td>Kp angle:</td>
                    <td><input type="number" id="KpAngleValue" value="0"></td>
                  </tr>
                  <tr>
                    <td>Ki angle:</td>
                    <td><input type="number" id="KiAngleValue" value="0"></td>
                  </tr>
                  <tr>
                    <td>Kd angle:</td>
                    <td><input type="number" id="KdAngleValue" value="0"></td>
                  </tr>
                  <tr>
                    <td>Tf angle:</td>
                    <td><input type="number" id="TfAngleValue" value="0"></td>
                  </tr>
                  <tr>
                    <td>max angle:</td>
                    <td><input type="number" id="maxAngleValue" value="0"></td>
                  </tr>
                  <tr>
                    <td>min angle:</td>
                    <td><input type="number" id="minAngleValue" value="0"></td>
                  </tr>
                  <tr>
                    <td>filter angle:</td>
                    <td><input type="number" id="filterAngleValue" value="0"></td>
                  </tr>
                  <tr>
                    <td>ANGLE PID scaler:</td>
                    <td><input type="number" id="AnglePIDValue" value="0"></td>
                  </tr>
                </tbody>
              </table>
            </td>
          </tr>
      <tr>
      <td colspan="2" align="center">
        <div>
          <br><br>
          <input type="button" value="Apply Changes" onclick="buttonclick();" >
          <input type="button" value="Clear Console" onclick="clearConsole(this);" >
          <input type="button" value="Reset STM32" onclick="ResetSTM32();" >
          <div class="rxd">
             <br><br>
             <label for="rxConsole">Parameters sent console: </label>
             <textarea id="rxConsole" id="rxConsole" readonly></textarea>
          </div
        </div>
      </td>
      </tr>
        </table>
        </div>
            
         </div>
      </center>
   </body>
</html>
)=====";

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);  
  Serial.begin(115200);
  
  Serial.println(); 
  Serial.print("Setting soft-AP configuration ... ");

  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");

  Serial.print("Setting soft-AP ... ");
  WiFi.mode(WIFI_AP);
  Serial.println(WiFi.softAP("Follow the leader") ? "Ready" : "Failed!");
  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());
  


  server.on("/", [](){
     server.send_P(200, "text/html", webpage);
  });
  server.on("/restart", restartServer);
  
  //server.onNotFound(handleNotFound);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  nodemcu.begin(115200);

  pinMode(13, INPUT);
  pinMode(15, OUTPUT);
  pinMode(LED_D0, OUTPUT);
  pinMode(LED, OUTPUT);    // LED pin as output.
}

void loop() 
{
    webSocket.loop();
    server.handleClient();
    
    if ( nodemcu.available() ) 
    {
        digitalWrite(LED, LOW);
        //String external_data = nodemcu.readString();
        Serial.write(nodemcu.read());
        //char charBuf[70];
        //char* buf = (char*) malloc(sizeof(char)*(external_data.length() + 1));
        //external_data.toCharArray(buf, external_data.length() + 1);
        //webSocket.broadcastTXT(external_data, strlen(external_data));
        digitalWrite(LED, HIGH);
    }
    if(Serial.available()) nodemcu.write(Serial.read());
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len)
{
    //Serial.printf("webSocketEvent(%d, %d, ...)\r\n", num, type);
    if(type == WStype_TEXT){
        digitalWrite(LED, LOW);
        //String _payload = String((char *) &payload[0]);
        Serial.printf("%s\r\n", payload);
        /*Serial.printf("[%u] get Text: %s\r\n", num, payload);*/
        //webSocket.broadcastTXT(payload, len);
        digitalWrite(LED, HIGH);
    } 
}

void restartServer(){
   server.send(200, "text/plain", "Restarting...");
   delay(1000);
   ESP.restart();
}
