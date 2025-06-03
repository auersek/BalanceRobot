// server.js
const express = require('express');
const { SerialPort } = require('serialport');
const path = require('path');

const app = express();
const HTTP_PORT = 8000;
const RETRY_MS = 5000;
let SERIAL_PATH = '/dev/tty.usbserial-110'; // Update this if needed
const BAUD_RATE = 115200;

let serialPort = null;
let routesRegistered = false;
let serverStarted = false;

// --- 1) List serial ports on startup ---
SerialPort.list()
  .then(ports => {
    console.log('[Serial] Available ports:');
    ports.forEach(p => console.log(' ', p.path));
    console.log(`\n[Hint] If not using "${SERIAL_PATH}", update the path in server.js\n`);
  })
  .catch(err => console.error('[Serial] Listing error:', err));

// --- 2) Serve frontend from /public ---
app.use(express.static(path.join(__dirname)));

// --- 3) Register UI routes (/f, /r, etc.) to serial writes ---
function registerRoutes(port) {
  if (routesRegistered) return;
  const cmdMap = { f: 'f', r: 'r', a: 'a', c: 'c', s: 's' };

  Object.entries(cmdMap).forEach(([route, cmdChar]) => {
    app.get(`/${route}`, (req, res) => {
      console.log(`[UI] /${route}`);
      if (!port || !port.isOpen) {
        console.warn(`[Serial] Command "${cmdChar}" skipped — serial not connected`);
        return res.status(500).send('Serial not connected');
      }
      port.write(cmdChar, err => {
        if (err) {
          console.error(`[Serial] Write error: ${err.message}`);
          return res.status(500).send('Serial error');
        }
        res.send('OK');
      });
    });
  });

  // ✅ NEW — /x?pos=X123Y456
  app.get('/x', (req, res) => {
    const coords = req.query.pos;
    if (!coords) return res.status(400).send('Missing coordinates');
    if (!port || !port.isOpen) return res.status(500).send('Serial not connected');
    port.write('x' + coords, err => {
      if (err) {
        console.error(`[Serial] Write error for /x: ${err.message}`);
        return res.status(500).send('Serial error');
      }
      console.log(`[Serial] Sent "x${coords}"`);
      res.send('OK');
    });
  });

  routesRegistered = true;
}

// --- 4) Serial connect / retry logic ---
function openSerial() {
  console.log(`[Serial] Attempting to open ${SERIAL_PATH}...`);

  serialPort = new SerialPort({
    path: SERIAL_PATH,
    baudRate: BAUD_RATE,
    autoOpen: false
  });

  serialPort.open(err => {
    if (err) {
      console.warn(`[Serial] Cannot open: ${err.message}`);
      return setTimeout(openSerial, RETRY_MS);
    }

    console.log(`[Serial] Connected on ${SERIAL_PATH}`);
    registerRoutes(serialPort);

    serialPort.on('close', () => {
      console.log('[Serial] Disconnected. Retrying...');
      routesRegistered = false;
      setTimeout(openSerial, RETRY_MS);
    });

    serialPort.on('error', err => {
      console.error('[Serial] Error:', err.message);
    });
  });

  // Start the HTTP server once
  if (!serverStarted) {
    app.listen(HTTP_PORT, () => {
      console.log(`[Server] Web UI available at http://localhost:${HTTP_PORT}`);
      console.log(`[Server] Waiting for serial connection...`);
    });
    serverStarted = true;
  }
}

// --- 5) Kick things off ---
setTimeout(openSerial, 2000);
