/*
 * index.html
 * */

const char *HTML_CONTENT_HOME = R"=====(
<!DOCTYPE html>
<html>
<head>
<title>DMXui - home</title>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=0.7, maximum-scale=0.70">
<link rel="icon" href="data:,">

<style>
button { font-weight: bold; font-size: width/2pt;}
body {
	background-color:#483C32;
	font-size: 12px;
	font-family: sans-serif;
}
.links {
  margin:auto;
  text-align: center; 
  
}
a {
	text-decoration: none; 
	font-family: sans-serif;
	display: inline-block;
	width: 150px;
    height: 20px;
    line-height: 20px;
	margin: auto;
	margin-bottom: 5px;
	color: #fff;
	border: 2px solid #aaa;
	background: #555;
	border-radius: 3px;
	}
.nav_cont
{ 
	margin: auto;
	width: 400px;
	/*padding: 10px;*/
	  display: flex;
  /*justify-content: space-evenly;*/
}
.dim {
	width: 400px;
	flex-basis: auto;
	margin: auto;
	text-align: center;
	background-color:#aaa;
	color:#000;
	border-radius: 5px;
}
#spacer_20 {
	height: 20px;
}
pre.sign {
    font: 0.7em monospace,"Courier New";
    font-weight: bold;
    font-family: monospace, "Courier New","Lucida Console";
    color: #fffb00;
    background-color: #483c32;
    border-radius: 5px;
}
</style>
</head>
<body>
<div class="dim" style="width: 400px;">
<pre class="sign">
   ____     __          ______              _       ____ ___  ___  ___ 
  / __/_ __/ /___ _____/_  __/______  ___  (_)___  |_  // _ \/ _ \/ _ \
 / _// // / __/ // / __// / / __/ _ \/ _ \/ / __/ _/_ K/ // / // / // /
/_/  \_,_/\__/\_,_/_/  /_/ /_/  \___/_//_/_/\__/ /____/\___/\___/\___/ 
                                                                       
</pre>
</div>
<div class="dim">
	<h3>DMX User Interface</h3>
</div>
<div class="dim" style="background-color:#fff;%ARTNET%">
	<div>WARNING ArtNet-client is enabled, all others DMX functions are disabled</div>
</div>
<div id="nav_cont" class="nav_cont">
	<div class="links">
		<table>
			<tr><td><div style="height: 10px"></div></td></tr>
			<tr><td><a href="/dimmers.html">DIMMERS</a></td></tr>
			<tr><td><div style="height: 10px"></div></td></tr>
			<tr><td><a href="/faders.html">FADERS</a></td></tr>
			<tr><td><div style="height: 10px"></div></td></tr>
			<tr><td><a href="/scenes.html">SCENES</a></td></tr>
			<tr><td><div style="height: 10px"></div></td></tr>
			<tr><td><a href="/fx1.html">FX1</a></td></tr>
			<tr><td><div style="height: 10px"></div></td></tr>
			<tr><td><a href="/fx2.html">FX2</a></td></tr>
			<tr><td><div style="height: 10px"></div></td></tr>
			<tr><td><a href="/vu.html">VU DMX</a></td></tr>
			<tr><td><div style="height: 10px"></div></td></tr>
			<tr><td><a href="/settings.html">SETTINGS</a></td></tr>
		</table>
	</div>
</div>
<div id="spacer_20"></div>
<div class="dim" style="width: 200px;">
	<div>your IP: %YOUR_IP%</div>
</div>
<div id="spacer_20"></div>
<div class="dim" style="width: 300px;background-color: #483c32;">
	<div style="font-size: larger;color: #fffb00;">esp32-S2-FN2R2</div>
	<div style="font-size: larger;color: #fffb00;">firmware:0.9-test</div>
	<div style="color: #777777;">free heap (min): %MINFREE% bytes</div>
	<div style="color: #777777;">dmx refresh rate: %DMXRATE%</div>
</div>
</body>
</html>
)=====";
