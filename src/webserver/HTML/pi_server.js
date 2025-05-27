// server.js
// USB–Serial bridge: serves UI and forwards /f,/r,/a,/c,/s to the ESP32

const express = require('express');
const { SerialPort } = require('serialport');
const path = require('path');

const app = express();
const HTTP_PORT = 8000;
const RETRY_MS = 5000;
let SERIAL_PATH = '/dev/ttyUSB0';   // override below if needed
const BAUD_RATE = 115200;

// 1) List available ports on startup
SerialPort.list()
  .then(ports => {
    console.log('Available serial ports:');
    ports.forEach(p => console.log(' ', p.path));
    console.log('----');
    console.log('If your ESP32 is on a different path, edit SERIAL_PATH and restart.');
  })
  .catch(err => console.error('Error listing ports:', err));

// 2) Serve static files (index.html, styles.css, main.js, images/)
app.use(express.static(path.join(__dirname)));

// 3) Map UI routes to serial
const cmdMap = { f: 'f', r: 'r', a: 'a', c: 'c', s: 's' };
let routesRegistered = false;
let serverStarted = false;

function registerRoutes(port) {
  if (routesRegistered) return;
  Object.entries(cmdMap).forEach(([route, cmdChar]) => {
    app.get(`/${route}`, (req, res) => {
      console.log(`Received /${route}`);
      port.write(cmdChar, err => {
        if (err) {
          console.error(`Write error for /${route}:`, err.message);
          return res.status(500).send('Serial Error');
        }
        res.send('OK');
      });
    });
  });
  routesRegistered = true;
}

// 4) Try opening serial, retry on failure
function openSerial() {
  console.log(`Opening serial port ${SERIAL_PATH}...`);
  const port = new SerialPort({
    path: SERIAL_PATH,
    baudRate: BAUD_RATE,
    autoOpen: false
  });

  port.open(err => {
    if (err) {
      console.warn(`Cannot open ${SERIAL_PATH}: ${err.message}`);
      return setTimeout(openSerial, RETRY_MS);
    }
    console.log(`Serial open on ${SERIAL_PATH}`);
    registerRoutes(port);
    if (!serverStarted) {
      app.listen(HTTP_PORT, () =>
        console.log(`HTTP server listening on port ${HTTP_PORT}`)
      );
      serverStarted = true;
    }
    port.on('close',   () => setTimeout(openSerial, RETRY_MS));
    port.on('error',   err => console.error('Serial error:', err.message));
  });
}


// Allow 2s for the port list to print
setTimeout(openSerial, 2000);
