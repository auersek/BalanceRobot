// main4.js
const boardBaseURL = "http://172.20.10.2";

// === 1) Send commands to server ===
function sendCommand(cmd) {
  fetch(`${boardBaseURL}/${cmd}`)
    .then(res => console.log(`Sent "${cmd}", status ${res.status}`))
    .catch(err => console.error('Command error:', err));
}

// === 2) Keyboard input mapping ===
const keyState = {};
const keyMap = {
  ArrowUp:    'f',
  ArrowDown:  'r',
  ArrowLeft:  'a',
  ArrowRight: 'c',
  ' ':        's',
  f: 'f', r: 'r', a: 'a', c: 'c', s: 's'
};

window.addEventListener('keydown', e => {
  const cmd = keyMap[e.key];
  if (!cmd || keyState[e.key]) return;
  e.preventDefault();
  keyState[e.key] = true;
  sendCommand(cmd);
  document.querySelector(`.btn[data-cmd="${cmd}"]`)?.classList.add('active');
});

window.addEventListener('keyup', e => {
  const cmd = keyMap[e.key];
  if (!cmd) return;
  keyState[e.key] = false;
  sendCommand('s');
  document.querySelector(`.btn[data-cmd="${cmd}"]`)?.classList.remove('active');
});

// === 3) Mouse button interactions ===
document.querySelectorAll('.btn').forEach(btn => {
  const cmd = btn.dataset.cmd;

  btn.addEventListener('mousedown', () => {
    if (cmd) {
      sendCommand(cmd);
      btn.classList.add('active');
    }
  });

  btn.addEventListener('mouseup', () => {
    sendCommand('s');
    btn.classList.remove('active');
  });

  btn.addEventListener('mouseleave', () => {
    btn.classList.remove('active');
  });
});

// === 4) Coordinate sending ===
const sendButton = document.querySelector('.send-coords');

sendButton.addEventListener('mousedown', () => {
  const x = document.getElementById('x-input').value;
  const y = document.getElementById('y-input').value;

  if (!x || !y) return;

  const cmd = `x${x}y${y}`;
  fetch(`${boardBaseURL}/${cmd}`)
    .then(res => console.log(`Sent command: ${cmd}, status ${res.status}`))
    .catch(err => console.error('coordinate command error:', err));

  sendButton.classList.add('active');
});

sendButton.addEventListener('mouseup', () => {
  sendButton.classList.remove('active');
});

sendButton.addEventListener('mouseleave', () => {
  sendButton.classList.remove('active');
});

// === 5) Battery bar simulation ===
let battery = 100;

function updateBatteryUI(level) {
  const fill = document.getElementById('battery-fill');
  const text = document.getElementById('battery-text');

  fill.style.width = `${level}%`;
  text.textContent = `${level}%`;

  if (level > 60) {
    fill.style.background = '#4fd453';
  } else if (level > 30) {
    fill.style.background = 'orange';
  } else {
    fill.style.background = 'red';
  }
}

setInterval(() => {
  battery = Math.max(0, battery - 1);
  updateBatteryUI(battery);
}, 3000);