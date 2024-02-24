/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-web-server-multiple-pages
 */
const char ok_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <title>Update successful</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body {
        background-color: #f7f7f7;
        font-size: 16px;
        font-family: sans-serif;
      }
      #spacer_30 {
        height: 30px;
      }
    </style>
  </head>
  <body>
    <center>
      <h2>The update was successful.</h2>
      <div id="spacer_30"></div>
      <button onclick="window.location.href='/';">to homepage</button>
    </center>
  </body>
</html> )rawliteral";
