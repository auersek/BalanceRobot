const boardIP   = window.location.hostname;
const HTTP_PORT = window.location.port || 80;

function sendCommand(cmd) {
  fetch(`http://${boardIP}:${HTTP_PORT}/${cmd}`)
    .then(res => console.log(`Sent "${cmd}", status ${res.status}`))
    .catch(err => console.error('Command error:', err));
}

// Handle keyboard input
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

// Handle mouse press/release
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
