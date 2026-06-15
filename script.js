/* ============================================================
   SMART COURIER SIMULATOR
   Algorithms : Greedy Nearest Neighbor O(n²) vs Brute Force O(n!)
   Navigation : Road-grid pathfinding (A* on intersection nodes)
   Subject    : Perancangan dan Analisis Algoritma — VRP
   ============================================================ */

// ─────────────────────────────────────────────
// CANVAS
// ─────────────────────────────────────────────
const canvas = document.getElementById('mapCanvas');
const ctx    = canvas.getContext('2d');

function resizeCanvas() {
  const area = document.querySelector('.map-area');
  const rect = area.getBoundingClientRect();
  canvas.width  = Math.floor(rect.width);
  canvas.height = Math.floor(rect.height);
}

window.addEventListener('resize', () => {
  resizeCanvas();
  buildRoadGraph();
  drawAll();
});

// ─────────────────────────────────────────────
// CONSTANTS
// ─────────────────────────────────────────────
const GRID_COLS  = 4;
const GRID_ROWS  = 4;
const MAX_HOUSES = 5;   // 5! = 120 — fast enough for brute force

// Inset padding so the road grid never touches the canvas edge.
// All roads and nodes live INSIDE this margin.
const MAP_PAD = 80;    // px padding from each canvas edge — ensures routes visible at corners

const HOUSE_NAMES = ['Rumah A','Rumah B','Rumah C','Rumah D','Rumah E'];

const COLOR = {
  road        : '#1e3a5f',
  warehouse   : '#f59e0b',
  house       : '#60a5fa',
  houseVisited: '#10b981',
  routeGreedy : '#10b981',
  routeBrute  : '#a78bfa',
  courier     : '#ef4444',
};

const SPEED_PX = { 1: 2.5, 2: 5.5, 3: 11 };

// ─────────────────────────────────────────────
// GRID GEOMETRY HELPERS
// Returns the pixel coordinate of the road grid origin and cell size.
// All roads are drawn INSIDE the padded area.
// ─────────────────────────────────────────────
function gridGeom() {
  const W = canvas.width, H = canvas.height;
  const innerW = W - MAP_PAD * 2;
  const innerH = H - MAP_PAD * 2;
  const cellW  = innerW / GRID_COLS;
  const cellH  = innerH / GRID_ROWS;
  return { ox: MAP_PAD, oy: MAP_PAD, cellW, cellH, innerW, innerH };
}

// Pixel position of intersection (col, row) — col: 0..GRID_COLS, row: 0..GRID_ROWS
function intersectionPx(col, row) {
  const { ox, oy, cellW, cellH } = gridGeom();
  return { x: ox + col * cellW, y: oy + row * cellH };
}

// ─────────────────────────────────────────────
// STATE
// ─────────────────────────────────────────────
let state = {
  nodes           : [],
  warehouse       : null,
  greedyRoute     : [],
  bruteRoute      : [],
  greedyDist      : 0,
  bruteDist       : 0,
  greedyWaypoints : [],
  bruteWaypoints  : [],

  courier: {
    x: 0, y: 0,
    waypoints : [],
    wpIdx     : 0,
    segT      : 0,
    running   : false,
    delivered : 0,
    totalDist : 0,
    mode      : 'greedy',
    nodeStops : [],
    stopWpIdx : [],
    nextStop  : 0,
  },

  speed            : 1,
  animFrame        : null,
  greedyRan        : false,
  bruteRan         : false,
  routeDisplayMode : 'greedy',

  roadGraph: { intersections: [] },
};

// ─────────────────────────────────────────────
// ROAD GRAPH — intersection nodes
// ─────────────────────────────────────────────
function buildRoadGraph() {
  const pts = [];
  for (let r = 0; r <= GRID_ROWS; r++) {
    for (let c = 0; c <= GRID_COLS; c++) {
      const px = intersectionPx(c, r);
      pts.push({ x: px.x, y: px.y, col: c, row: r });
    }
  }
  state.roadGraph.intersections = pts;
}

function intersectionKey(pt) { return pt.col + '_' + pt.row; }

function intersectionNeighbours(pt) {
  return state.roadGraph.intersections.filter(q =>
    (q.row === pt.row && Math.abs(q.col - pt.col) === 1) ||
    (q.col === pt.col && Math.abs(q.row - pt.row) === 1)
  );
}

// ─────────────────────────────────────────────
// A* ON ROAD GRID
// ─────────────────────────────────────────────
function snapToRoad(px, py) {
  let best = null, bestD = Infinity;
  for (const pt of state.roadGraph.intersections) {
    const d = Math.hypot(pt.x - px, pt.y - py);
    if (d < bestD) { bestD = d; best = pt; }
  }
  return best;
}

function astarRoad(startPt, goalPt) {
  if (startPt === goalPt ||
      (startPt.col === goalPt.col && startPt.row === goalPt.row)) return [startPt];

  const open     = new Set([startPt]);
  const cameFrom = new Map();
  const gScore   = new Map();
  const fScore   = new Map();

  for (const n of state.roadGraph.intersections) {
    gScore.set(intersectionKey(n), Infinity);
    fScore.set(intersectionKey(n), Infinity);
  }
  gScore.set(intersectionKey(startPt), 0);
  fScore.set(intersectionKey(startPt),
    Math.hypot(goalPt.x - startPt.x, goalPt.y - startPt.y));

  while (open.size > 0) {
    let current = null, cf = Infinity;
    for (const n of open) {
      const f = fScore.get(intersectionKey(n));
      if (f < cf) { cf = f; current = n; }
    }

    if (current.col === goalPt.col && current.row === goalPt.row) {
      const path = [current];
      let c = current;
      while (cameFrom.has(intersectionKey(c))) {
        c = cameFrom.get(intersectionKey(c));
        path.unshift(c);
      }
      return path;
    }

    open.delete(current);
    for (const nb of intersectionNeighbours(current)) {
      const tentG = gScore.get(intersectionKey(current)) +
                    Math.hypot(nb.x - current.x, nb.y - current.y);
      if (tentG < gScore.get(intersectionKey(nb))) {
        cameFrom.set(intersectionKey(nb), current);
        gScore.set(intersectionKey(nb), tentG);
        fScore.set(intersectionKey(nb), tentG +
          Math.hypot(goalPt.x - nb.x, goalPt.y - nb.y));
        open.add(nb);
      }
    }
  }
  return [startPt, goalPt];
}

// ─────────────────────────────────────────────
// ROAD PATH
// ─────────────────────────────────────────────
function getRoadPath(nodeA, nodeB) {
  const snapA = snapToRoad(nodeA.roadX, nodeA.roadY);
  const snapB = snapToRoad(nodeB.roadX, nodeB.roadY);
  const intersPath = astarRoad(snapA, snapB);

  const wps = [];

  // Node visual centre → its road snap point
  wps.push({ x: nodeA.x, y: nodeA.y });
  if (Math.hypot(snapA.x - nodeA.x, snapA.y - nodeA.y) > 2)
    wps.push({ x: snapA.x, y: snapA.y });

  // Walk intersections
  for (const ip of intersPath) {
    const last = wps[wps.length - 1];
    if (Math.hypot(ip.x - last.x, ip.y - last.y) > 1)
      wps.push({ x: ip.x, y: ip.y });
  }

  // Last intersection → destination snap → destination node centre
  const lastWp = wps[wps.length - 1];
  if (Math.hypot(snapB.x - lastWp.x, snapB.y - lastWp.y) > 2)
    wps.push({ x: snapB.x, y: snapB.y });
  if (Math.hypot(nodeB.x - wps[wps.length-1].x,
                 nodeB.y - wps[wps.length-1].y) > 2)
    wps.push({ x: nodeB.x, y: nodeB.y });

  return wps;
}

function buildTripWaypoints(nodeRoute) {
  const waypoints = [];
  const stopWpIdx = [];

  for (let i = 0; i < nodeRoute.length - 1; i++) {
    const seg = getRoadPath(state.nodes[nodeRoute[i]], state.nodes[nodeRoute[i + 1]]);
    if (i === 0) waypoints.push(...seg);
    else         waypoints.push(...seg.slice(1));
    stopWpIdx.push(waypoints.length - 1);
  }
  return { waypoints, stopWpIdx };
}

function roadSegmentDist(nodeA, nodeB) {
  const path = getRoadPath(nodeA, nodeB);
  let d = 0;
  for (let i = 0; i < path.length - 1; i++)
    d += Math.hypot(path[i+1].x - path[i].x, path[i+1].y - path[i].y);
  return d;
}

function routeRoadDistance(nodeRoute) {
  let total = 0;
  for (let i = 0; i < nodeRoute.length - 1; i++)
    total += roadSegmentDist(state.nodes[nodeRoute[i]], state.nodes[nodeRoute[i+1]]);
  return total;
}

// ─────────────────────────────────────────────
// NODE GENERATION
// Nodes placed at INNER intersections only (not on the border row/col = 0 or max),
// so their icons and route lines are never clipped by the canvas edge.
// ─────────────────────────────────────────────
function generateNodes() {
  resizeCanvas();
  buildRoadGraph();

  const { cellW, cellH } = gridGeom();

  // Use only intersections that are NOT on the outer border of the grid.
  // Border intersections (col=0, col=GRID_COLS, row=0, row=GRID_ROWS) are kept
  // as road nodes for routing, but buildings are placed only at interior ones.
  const interiorIntersections = state.roadGraph.intersections.filter(pt =>
    pt.col > 0 && pt.col < GRID_COLS && pt.row > 0 && pt.row < GRID_ROWS
  );

  // Shuffle and pick nodes
  const pool = interiorIntersections.slice();
  shuffle(pool);
  const picked = pool.slice(0, 1 + MAX_HOUSES);

  state.nodes = picked.map((inter, i) => {
    // Offset visual icon slightly toward the center of an adjacent cell
    // so the building doesn't sit exactly on the road line.
    const adjCols = [inter.col - 1, inter.col].filter(c => c >= 0 && c < GRID_COLS);
    const adjRows = [inter.row - 1, inter.row].filter(r => r >= 0 && r < GRID_ROWS);

    let vx = inter.x, vy = inter.y;
    if (adjCols.length && adjRows.length) {
      const targetX = (adjCols[i % adjCols.length] + 0.5) * cellW;
      const targetY = (adjRows[i % adjRows.length] + 0.5) * cellH;
      const dx = targetX - (inter.x - MAP_PAD);   // relative to grid origin
      const dy = targetY - (inter.y - MAP_PAD);
      const len = Math.hypot(dx, dy) || 1;
      const pull = Math.min(cellW, cellH) * 0.30;
      vx = inter.x + (dx / len) * pull;
      vy = inter.y + (dy / len) * pull;
    }

    if (i === 0) return { x: vx, y: vy, roadX: inter.x, roadY: inter.y,
                          name: '🏭 Gudang', type: 'warehouse', visited: false };
    return       { x: vx, y: vy, roadX: inter.x, roadY: inter.y,
                  name: HOUSE_NAMES[i - 1], type: 'house', visited: false };
  });

  state.warehouse = state.nodes[0];
}

function shuffle(arr) {
  for (let i = arr.length - 1; i > 0; i--) {
    const j = Math.floor(Math.random() * (i + 1));
    [arr[i], arr[j]] = [arr[j], arr[i]];
  }
}

// ─────────────────────────────────────────────
// EUCLIDEAN DISTANCE (heuristic only)
// ─────────────────────────────────────────────
function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }

// ─────────────────────────────────────────────
// ALGORITHM 1 — Greedy Nearest Neighbour O(n²)
// ─────────────────────────────────────────────
function greedyNearestNeighbor() {
  const n = state.nodes.length;
  const visited = new Array(n).fill(false);
  visited[0] = true;
  let route = [0], current = 0;

  for (let step = 0; step < n - 1; step++) {
    let nearest = -1, nearestD = Infinity;
    for (let j = 1; j < n; j++) {
      if (!visited[j]) {
        const d = dist(state.nodes[current], state.nodes[j]);
        if (d < nearestD) { nearestD = d; nearest = j; }
      }
    }
    if (nearest !== -1) {
      visited[nearest] = true; route.push(nearest); current = nearest;
    }
  }
  route.push(0);
  return route;
}

// ─────────────────────────────────────────────
// ALGORITHM 2 — Brute Force O(n!)
// ─────────────────────────────────────────────
function permutations(arr) {
  if (arr.length <= 1) return [arr.slice()];
  const result = [];
  for (let i = 0; i < arr.length; i++) {
    const rest = arr.slice(0, i).concat(arr.slice(i + 1));
    for (const p of permutations(rest)) result.push([arr[i], ...p]);
  }
  return result;
}

function factorial(n) { return n <= 1 ? 1 : n * factorial(n - 1); }

function bruteForce() {
  const houseIdx = state.nodes.slice(1).map((_, i) => i + 1);
  const totalP   = factorial(houseIdx.length);

  document.getElementById('bfPerms').textContent = totalP + ' perm';
  const bfStatus = document.getElementById('bfStatus');
  bfStatus.textContent = 'Menghitung ' + totalP + ' rute...';
  bfStatus.className   = 'bf-status computing';

  let bestRoute = null, bestD = Infinity;
  for (const perm of permutations(houseIdx)) {
    const route = [0, ...perm, 0];
    const d     = routeRoadDistance(route);
    if (d < bestD) { bestD = d; bestRoute = route; }
  }

  bfStatus.textContent = '\u2713 Selesai! ' + totalP + ' rute diperiksa';
  bfStatus.className   = 'bf-status done';
  return { route: bestRoute, dist: bestD };
}

// ─────────────────────────────────────────────
// SIDEBAR RENDERING
// ─────────────────────────────────────────────
function makeStepEl(num, name, d, colorClass) {
  const step   = document.createElement('div');
  step.className = 'route-step';
  const numEl  = document.createElement('span');
  numEl.className = 'step-num' + (colorClass ? ' ' + colorClass : '');
  numEl.textContent = num;
  const nameEl = document.createElement('span');
  nameEl.className = 'step-name';
  nameEl.textContent = name;
  const dEl    = document.createElement('span');
  dEl.className = 'step-dist';
  dEl.textContent = Math.round(d) + 'px';
  step.appendChild(numEl); step.appendChild(nameEl); step.appendChild(dEl);
  return step;
}

function renderGreedyRoute() {
  const list   = document.getElementById('greedyRouteList');
  const distEl = document.getElementById('greedyDistance');
  list.innerHTML = '';
  for (let i = 0; i < state.greedyRoute.length - 1; i++) {
    const from = state.nodes[state.greedyRoute[i]];
    const to   = state.nodes[state.greedyRoute[i + 1]];
    list.appendChild(makeStepEl(i + 1, to.name, roadSegmentDist(from, to), ''));
  }
  distEl.textContent = Math.round(state.greedyDist) + ' px';
}

function renderBruteRoute() {
  const list   = document.getElementById('bruteRouteList');
  const distEl = document.getElementById('bruteDistance');
  list.innerHTML = '';
  if (!state.bruteRoute || state.bruteRoute.length < 2) {
    list.innerHTML = '<div class="route-placeholder">Jalankan Brute Force untuk melihat route optimal</div>';
    distEl.textContent = '\u2014';
    return;
  }
  for (let i = 0; i < state.bruteRoute.length - 1; i++) {
    const from = state.nodes[state.bruteRoute[i]];
    const to   = state.nodes[state.bruteRoute[i + 1]];
    list.appendChild(makeStepEl(i + 1, to.name, roadSegmentDist(from, to), 'brute'));
  }
  distEl.textContent = Math.round(state.bruteDist) + ' px';
}

function renderComparison() {
  const section = document.getElementById('compareSection');
  if (!state.greedyRan || !state.bruteRan) { section.style.display = 'none'; return; }
  section.style.display = 'block';
  const gd = Math.round(state.greedyDist), bd = Math.round(state.bruteDist);
  document.getElementById('cmpGreedy').textContent = gd + ' px';
  document.getElementById('cmpManual').textContent = bd + ' px';
  const diff = Math.abs(gd - bd);
  document.getElementById('compareSelisih').textContent = 'Selisih: ' + diff + ' px';
  const verdict = document.getElementById('compareVerdict');
  if (diff === 0) {
    verdict.className   = 'compare-verdict tie';
    verdict.textContent = '\uD83E\uDD1D Greedy menemukan solusi optimal! Identik dengan Brute Force.';
  } else if (gd <= bd) {
    verdict.className   = 'compare-verdict greedy-wins';
    verdict.textContent = '\u2705 Greedy optimal di kasus ini! Hemat ' + diff + ' px.';
  } else {
    verdict.className   = 'compare-verdict manual-wins';
    verdict.textContent = '\uD83D\uDD2C Brute Force lebih optimal! Hemat ' + diff +
                          'px (' + ((diff/gd)*100).toFixed(1) + '%).';
  }
}

function updateStatus(status, target, delivered, total, traveled) {
  const badge = document.getElementById('courierStatus');
  badge.textContent = status; badge.className = 'status-badge';
  if (status === 'BERJALAN') badge.classList.add('running');
  if (status === 'SELESAI')  badge.classList.add('done');
  document.getElementById('courierTarget').textContent    = target || '\u2014';
  document.getElementById('courierDelivered').textContent = delivered + ' / ' + total;
  document.getElementById('courierTraveled').textContent  = Math.round(traveled) + ' px';
  document.getElementById('progressBar').style.width =
    (total > 0 ? (delivered / total) * 100 : 0) + '%';
}