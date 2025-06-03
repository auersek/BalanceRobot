// server4.js
const express = require('express');
const { SerialPort } = require('serialport');
const path = require('path');

const app = express();
const HTTP_PORT = 8000;
const RETRY_MS = 5000;
const SERIAL_PATH = '/dev/tty.usbserial-110'; // Update if needed
const BAUD_RATE = 115200;

let serialPort = null;
let routesRegistered = false;
let serverStarted = false;


SerialPort.list()
  .then(ports => {
    console.log('Available ports:');
    ports.forEach(p => console.log(' -', p.path));
    console.log(`\nUsing "${SERIAL_PATH}" — update this path in server4.js if needed\n`);
  })
  .catch(err => console.error(' Port listing error:', err));

app.use(express.static(path.join(__dirname)));

function registerRoutes(port) {
  if (routesRegistered) return;


  const simpleRoutes = ['f', 'r', 'a', 'c', 's'];
  simpleRoutes.forEach(cmd => {
    app.get(`/${cmd}`, (req, res) => {
      if (!port?.isOpen) {
        console.warn(`Skipped "/${cmd}" — serial not connected`);
        return res.status(500).send('Serial not connected');
      }
      port.write(cmd + '\n', err => {
        if (err) {
          console.error(`Write error for "/${cmd}": ${err.message}`);
          return res.status(500).send('Serial error');
        }
        console.log(` Sent "${cmd}"`);
        res.send('OK');
      });
    });
  });


  app.get('/x:cmd', (req, res) => {
    let cmd = req.params.cmd.trim();

    if (!/^x\d+y\d+$/i.test('x' + cmd)) {
      return res.status(400).send('Invalid coordinate format');
    }

    if (!port?.isOpen) {
      return res.status(500).send('Serial not connected');
    }

    port.write('x' + cmd + '\n', err => {
      if (err) {
        console.error(`Write error for "x${cmd}": ${err.message}`);
        return res.status(500).send('Serial error');
      }
      console.log(` Sent "x${cmd}"`);
      res.send('OK');
    });
  });

  routesRegistered = true;
}


function openSerial() {
  console.log(`Attempting connection to ${SERIAL_PATH}...`);

  serialPort = new SerialPort({
    path: SERIAL_PATH,
    baudRate: BAUD_RATE,
    autoOpen: false,
  });

  serialPort.open(err => {
    if (err) {
      console.warn(`Failed to open: ${err.message}`);
      return setTimeout(openSerial, RETRY_MS);
    }

    console.log(`Connected on ${SERIAL_PATH}`);
    registerRoutes(serialPort);

    serialPort.on('close', () => {
      console.log('Disconnected. Retrying...');
      routesRegistered = false;
      setTimeout(openSerial, RETRY_MS);
    });

    serialPort.on('error', err => {
      console.error('Error:', err.message);
    });
  });

  if (!serverStarted) {
    app.listen(HTTP_PORT, () => {
      console.log(`[Server] UI available at http://localhost:${HTTP_PORT}`);
      console.log('[Server] Waiting for serial device...');
    });
    serverStarted = true;
  }
}


setTimeout(openSerial, 2000);
