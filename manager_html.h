/*
 * modified 
 */
 
/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-web-server-multiple-pages
 */
const char manager_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <title>DMXui - FILES</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=0.8">
    <style>
      body {
        background-color: #a7a7a7;
        /*font-size: 12px;*/
        font-family: monospace;
      }
      #submit {
        width:120px;
      }
      #edit_path {
        width:250px;
      }
      #delete_path {
        width:250px;
      }
      #spacer_50 {
        height: 50px;
      }
      #spacer_20 {
        height: 20px;
      }
      #spacer_10 {
        height: 10px;
      }
      table {
        background-color: #dddddd;
        border-collapse: collapse;
        width:650px;
      }
      td, th {
        border: 1px solid #dddddd;
        text-align: left;
        padding: 8px;
      }
      #first_td_th {
        width:400px;
      }
      tr:nth-child(even) {
        background-color: #ffffff;
      }
      fieldset {
        width:700px;
        background-color: #f7f7f7;
      }
      #format_notice {
        color: #ff0000;
      }
      a.dmxlink {
      text-decoration: none; 
      font-family: sans-serif;
      display: inline-block;
      width: 35px;
      height: 25px;
      line-height: 25px;
      margin-right: 15px;
      color: #fff;
      /* border: 2px solid #aaa; */
      background: #555;
      border-radius: 3px;
      text-align: center;

      }
      .links {
      width: 400px;
      margin: 0 auto;
      text-align: center; 
      }
      legend {
		font-weight: bold;
		background-color: orange;
	  }
    </style>
    <script>
      function validateFormUpdate()
      {
        var inputElement = document.getElementById('update');
        var files = inputElement.files;
        if(files.length==0)
        {
          alert("You have not chosen a file!");
          return false;
        }
        var value = inputElement.value;
        var dotIndex = value.lastIndexOf(".")+1;
        var valueExtension = value.substring(dotIndex);
        if(valueExtension != "bin")
        {
          alert("Incorrect file type!");
          return false;
        }
      }
      
      function validateFormUpload()
      {
        var inputElement = document.getElementById('upload_data');
        var files = inputElement.files;
        if(files.length==0)
        {
          alert("You have not chosen a file!");
          return false;
        }
      }
      
      function validateFormDownload()
      {
        var downloadSelectValue = document.getElementById('download_path').value;
        if(downloadSelectValue == "choose" )
        {
          alert("You have not chosen a file!");
          return false;
        }
      }
      
      function validateFormDelete()
      {
        var deleteSelectValue = document.getElementById('delete_path').value;
        if(deleteSelectValue == "choose" )
        {
          alert("You have not chosen a file!");
          return false;
        }
      }

    </script>
  </head>
  <body>
  <div class="links">
	<a href="/dimmers.html" class="dmxlink"> DIM </a>
	<a href="/faders.html" class="dmxlink"> FAD </a>
	<a href="/scenes.html" class="dmxlink"> SCN </a>
	<a href="/fx1.html" class="dmxlink"> FX1 </a>
	<a href="/fx2.html" class="dmxlink"> FX2 </a>
	<a href="/settings.html" class="dmxlink"> SET </a>
  </div>
    <center>
		<div id="spacer_10"></div>
      
      <h2>*** Files Manager ***</h2>

      <div id="spacer_10"></div>

      <fieldset>
		  <legend>Files list</legend>
          <div id="spacer_10"></div>
          <p>Full SPIFFS storage: %SPIFFS_TOTAL_BYTES%, thereof used: %SPIFFS_USED_BYTES%, still available: %SPIFFS_FREE_BYTES%</p>
          <div id="spacer_10"></div>
          %LISTEN_FILES%
          <div id="spacer_10"></div>
      </fieldset>

      <div id="spacer_10"></div>

      <fieldset>
      <legend>Upload file</legend>
          <div id="spacer_10"></div>
          <form method="POST" action="/upload" enctype="multipart/form-data">
            <table><tr><td id="first_td_th">
            <input type="file" id="upload_data" name="upload_data">
            </td><td>
            <input type="submit" id="submit" value="File upload!" onclick="return validateFormUpload()">
            </td></tr></table>
          </form>
          <div id="spacer_10"></div>
      </fieldset>
      
      
	  <div id="spacer_10"></div>

      <fieldset>
        <legend>Download file</legend>
          <div id="spacer_10"></div>
          <form method="GET" action="/download">
            <table><tr><td id="first_td_th">          
            %DOWNLOAD_FILES%
            </td><td>
            <input type="submit" id="submit" value="Download" onclick="return validateFormDownload()">
            </td></tr></table>
          </form>
          <div id="spacer_10"></div>
      </fieldset>
      
      <div id="spacer_10"></div>

      <fieldset>
        <legend>Delete file</legend>
          <div id="spacer_10"></div>
          <form method="GET" action="/delete">
            <table><tr><td id="first_td_th">          
            %DELETE_FILES%
            </td><td>
            <input type="submit" id="submit" value="Delete" onclick="return validateFormDelete()">
            </td></tr></table>
          </form>
          <div id="spacer_10"></div>
      </fieldset>
      <div id="spacer_10"></div>
      
      <h2>*** DMXui Firmware Update ***</h2>

      <div id="spacer_10"></div>
      <fieldset>
          <legend>ESP32-S2 firmware update</legend>
          <div id="spacer_10"></div>
          <form method="POST" action="/update" enctype="multipart/form-data">
            <table><tr><td id="first_td_th">
            <input type="file" id="update" name="update">
            </td><td>
				<input type="submit" id="submit" value="Update!" onclick="return validateFormUpdate()">
            </td></tr></table>
          </form>
          <div id="spacer_10"></div>
      </fieldset>
      
		<div id="spacer_10"></div>
		<div id="spacer_10"></div>
		
      <iframe style="display:none" name="self_page"></iframe>
    </center>
  </body>
</html> )rawliteral";
