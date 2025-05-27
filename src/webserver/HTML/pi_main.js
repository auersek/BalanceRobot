// main.js

const boardIP   = window.location.hostname;
const HTTP_PORT = window.location.port || 80;

function sendCommand(cmd) {
  fetch(`http://${boardIP}:${HTTP_PORT}/${cmd}`)
    .then(res => console.log(`Sent "${cmd}", status ${res.status}`))
    .catch(err => console.error('Command error:', err));
}

// Wire up button clicks
document.querySelectorAll('.btn').forEach(btn => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.cmd;
    if (cmd) sendCommand(cmd);
  });
});

// Keyboard support with de-duping
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
  sendCommand('s');  // always stop on release
  document.querySelector(`.btn[data-cmd="${cmd}"]`)?.classList.remove('active');
});
