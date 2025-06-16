
let motorChart, controlChart;
function initPowerCharts() {
  const mCtx = document.getElementById('motorChart').getContext('2d');
  motorChart = new Chart(mCtx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [{
        label: 'Motor Current (A)',
        data: [],
        borderColor: '#E67E22',
        backgroundColor: 'rgba(230,126,34,0.2)',
        fill: true
      }]
    },
    options: {
      animation: false,
      scales: {
        x: { title:{ display:true, text:'Time' } },
        y: { title:{ display:true, text:'Current (A)' } }
      }
    }
  });

  // Control Current chart
  const cCtx = document.getElementById('controlChart').getContext('2d');
  controlChart = new Chart(cCtx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [{
        label: 'Control Current (A)',
        data: [],
        borderColor: '#1ABC9C',
        backgroundColor: 'rgba(26,188,156,0.2)',
        fill: true
      }]
    },
    options: {
      animation: false,
      scales: {
        x: { title:{ display:true, text:'Time' } },
        y: { title:{ display:true, text:'Current (A)' } }
      }
    }
  });
}


const views = ['control','images','power'];
let current = null;
const pollIds = {};

document.querySelectorAll('.sidebar button').forEach(btn => {
  btn.addEventListener('click', () => switchTo(btn.dataset.view));
});

window.addEventListener('DOMContentLoaded', () => {
  initPowerCharts();       
  switchTo('control');      
});

function switchTo(view) {
  if (view === current) return;
  if (current) {
    document.getElementById(`view-${current}`).style.display = 'none';
    clearInterval(pollIds[current]);
  }
  document.getElementById(`view-${view}`).style.display = 'block';
  startPoll(view);
  current = view;
}

function startPoll(view) {
  if (view === 'control') {
    fetchPose();
    pollIds.control = setInterval(fetchPose, 100);
  }
  else if (view === 'images') {
    fetchImages();
    pollIds.images = setInterval(fetchImages, 5000);
  }
  else if (view === 'power') {
    fetchMetrics();
    pollIds.power = setInterval(fetchMetrics, 1000);
  }
}

// ──────────────────────────────────────────────────────────────
// 3) Power: fetch & update charts + battery icon
// ──────────────────────────────────────────────────────────────

async function fetchMetrics() {
  try {
    const res = await fetch('/power');
    if (!res.ok) throw new Error(res.status);
    const { time, mI, cI, battPct } = await res.json();
    const t = new Date(time).toLocaleTimeString();

    motorChart.data.labels.push(t);
    motorChart.data.datasets[0].data.push(mI);

    controlChart.data.labels.push(t);
    controlChart.data.datasets[0].data.push(cI);

    [motorChart, controlChart].forEach(ch => {
      if (ch.data.labels.length > 60) {
        ch.data.labels.shift();
        ch.data.datasets[0].data.shift();
      }
      ch.update('quiet');
    });

    updateBatteryUI(battPct);

  } catch (e) {
    console.warn('fetchMetrics error', e);
  }
}

function updateBatteryUI(level) {
  const fill = document.getElementById('battery-fill');
  const text = document.getElementById('battery-text');
  if (!fill || !text) return;
  const pct = Math.round(level);
  fill.style.width = `${pct}%`;
  text.textContent = `${pct}%`;
  fill.style.background = pct > 60
    ? '#4fd453'
    : pct > 30
      ? 'orange'
      : 'red';
}

const ESP_IP = "http://172.20.10.5";
const POSE_URL  = `http://localhost:8000/pose`;
const GOTO_BASE = `${ESP_IP}/goto`;
const GRID_W      = 100;
const GRID_H      = 100;
const CANVAS_SIZE = 500;
const CELL_PX     = CANVAS_SIZE / GRID_W;

let robotCell = { x:15, y:15 };
const canvas = document.getElementById('gridCanvas');
const ctx    = canvas.getContext('2d');
canvas.width = CANVAS_SIZE;
canvas.height = CANVAS_SIZE;
canvas.style.touchAction = 'none';

const CellState = { UNKNOWN:0, FREE:1, OBSTACLE:2, ROBOT:3 };
let gridState = Array.from({length:GRID_H}, ()=>Array(GRID_W).fill(CellState.UNKNOWN));
let path = [];

function heuristic(x1,y1,x2,y2){ return Math.abs(x1-x2)+Math.abs(y1-y2); }
function getNeighbors(x,y){ /* */ }
function aStar(start,goal){ /* */ }

function drawGrid(){
  ctx.clearRect(0,0,CANVAS_SIZE,CANVAS_SIZE);
  for(let y=0;y<GRID_H;y++){
    for(let x=0;x<GRID_W;x++){
      switch(gridState[y][x]){
        case CellState.UNKNOWN:  ctx.fillStyle='#444'; break;
        case CellState.FREE:     ctx.fillStyle='#fff'; break;
        case CellState.OBSTACLE: ctx.fillStyle='#000'; break;
        case CellState.ROBOT:    ctx.fillStyle='#0f0'; break;
      }
      ctx.fillRect(x*CELL_PX,y*CELL_PX,CELL_PX,CELL_PX);
      ctx.strokeStyle='#222';
      ctx.strokeRect(x*CELL_PX,y*CELL_PX,CELL_PX,CELL_PX);
    }
  }
}

drawGrid();

async function fetchPose(){
  try {
    const res = await fetch(POSE_URL);
    const msg = await res.json();

    console.log("Received pose:", msg);

    for (let y = 0; y < GRID_H; y++) {
      for (let x = 0; x < GRID_W; x++) {
        if (gridState[y][x] === CellState.ROBOT || gridState[y][x] === CellState.OBSTACLE) {
          gridState[y][x] = CellState.FREE;
        }
      }
    }

    if (Array.isArray(msg.obstacles)) {
      msg.obstacles.forEach(([ox, oy]) => {
        if (ox >= 0 && oy >= 0 && oy < GRID_H && ox < GRID_W) {
          gridState[oy][ox] = CellState.OBSTACLE;
        }
      });
      console.log(`Marked ${msg.obstacles.length} obstacles`);
    }

    if (msg.x >= 0 && msg.y >= 0 && msg.y < GRID_H && msg.x < GRID_W) {
      robotCell.x = msg.x;
      robotCell.y = msg.y;
      gridState[msg.y][msg.x] = CellState.ROBOT;
    }

  } catch (e) {
    console.warn('fetchPose error', e);
  }

  drawGrid();
}


canvas.addEventListener('click',evt=>{
  const rect = canvas.getBoundingClientRect();
  const gx = Math.floor((evt.clientX-rect.left)*(canvas.width/rect.width)/CELL_PX);
  const gy = Math.floor((evt.clientY-rect.top)*(canvas.height/rect.height)/CELL_PX);
  if(gx<0||gx>=GRID_W||gy<0||gy>=GRID_H) return;
  path = aStar([robotCell.x,robotCell.y],[gx,gy]);
  if(path.length>1){
    const [nx,ny] = path[1];
    fetch(`${GOTO_BASE}?gx=${nx}&gy=${ny}`).catch(e=>console.warn(e));
  }
  drawGrid();
});

async function fetchImages(){
  try{
    const res = await fetch('/images');
    const files = await res.json();
    const container = document.getElementById('thumbnails');
    container.innerHTML = '';
    files.forEach(f=>{
      const img = document.createElement('img');
      img.src = f.url; img.style.width='150px'; img.style.margin='8px';
      container.appendChild(img);
    });
  }catch(e){
    console.warn('fetchImages error', e);
  }
}

const keyState = {};
const keyMap = {
  ArrowUp:'f',ArrowDown:'r',ArrowLeft:'a',ArrowRight:'c',
  ' ':'s', f:'f', r:'r', a:'a', c:'c', s:'s', q:'q'
};

window.addEventListener('keydown',e=>{
  const cmd = keyMap[e.key];
  if(!cmd||keyState[e.key])return;
  e.preventDefault();keyState[e.key]=true;
  sendCommand(cmd);
  document.querySelector(`.btn[data-cmd="${cmd}"]`)?.classList.add('active');
});
window.addEventListener('keyup',e=>{
  const cmd = keyMap[e.key];
  if(!cmd)return;
  keyState[e.key]=false;
  sendCommand('s');
  document.querySelector(`.btn[data-cmd="${cmd}"]`)?.classList.remove('active');
});


function sendCommand(cmd){
  fetch(`${ESP_IP}/${cmd}`).catch(e => console.error('Command error', e));
}


document.querySelectorAll('.btn').forEach(btn=>{
  const cmd=btn.dataset.cmd;
  btn.addEventListener('mousedown',()=>{
    sendCommand(cmd);btn.classList.add('active');
  });
  btn.addEventListener('mouseup',()=>{
    sendCommand('s');btn.classList.remove('active');
  });
  btn.addEventListener('mouseleave',()=>btn.classList.remove('active'));
});
