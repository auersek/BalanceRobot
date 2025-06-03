const boardIP = window.location.hostname;
const HTTP_PORT = window.location.port || 80;

function sendCommand(cmd) {
  fetch(`http://${boardIP}:${HTTP_PORT}/${cmd}`)
    .then(res => console.log(`Sent "${cmd}", status ${res.status}`))
    .catch(err => console.error('Command error:', err));
}

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

document.querySelector('.send-coords').addEventListener('mousedown', () => {
  const x = document.getElementById('x-input').value;
  const y = document.getElementById('y-input').value;
  const cmd = `X${x}Y${y}`;

  fetch(`http://${boardIP}:${HTTP_PORT}/x?pos=${cmd}`)
    .then(res => console.log(`Sent x command: ${cmd}, status ${res.status}`))
    .catch(err => console.error('X command error:', err));

  document.querySelector('.send-coords').classList.add('active');
});

document.querySelector('.send-coords').addEventListener('mouseup', () => {
  document.querySelector('.send-coords').classList.remove('active');
});

document.querySelector('.send-coords').addEventListener('mouseleave', () => {
  document.querySelector('.send-coords').classList.remove('active');
});

let battery = 100;

function updateBatteryUI(level) {
  const fill = document.getElementById('battery-fill');
  const text = document.getElementById('battery-text');
  
  fill.style.width = `${level}%`;
  text.textContent = `${level}%`;

  if (level > 60) {
    fill.style.background = 'linear-gradient(to right, #4caf50, #a6d2e8)';
  } else if (level > 30) {
    fill.style.background = 'orange';
  } else {
    fill.style.background = 'red';
  }
}

// Simulated drain:
setInterval(() => {
  battery = Math.max(0, battery - 1);
  updateBatteryUI(battery);
}, 3000); // drains every 3s