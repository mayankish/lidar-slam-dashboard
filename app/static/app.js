// lidar-slam-dashboard frontend. Plain vanilla JS, no build step, no
// framework -- connects to /ws for live grid pushes, polls /status for
// link/health stats, and posts to /control. Served as-is by FastAPI's
// StaticFiles (see ../README.md "Build & run instructions").
//
// Two independent "liveness" signals are tracked deliberately, because
// they answer different questions: wsConnected (is *this browser tab*
// talking to the dashboard server) vs link liveness (is the dashboard
// server actually receiving bot telemetry on UDP :5005). A browser can be
// happily connected over WebSocket while the bot/simulator is completely
// silent -- collapsing these into one indicator would hide that.

const canvas = document.getElementById("gridCanvas");
const ctx = canvas.getContext("2d");
const canvasOverlay = document.getElementById("canvasOverlay");

const wsDot = document.getElementById("wsDot");
const wsLabel = document.getElementById("wsLabel");
const linkDot = document.getElementById("linkDot");
const linkLabel = document.getElementById("linkLabel");
const sweepCountEl = document.getElementById("sweepCount");
const gridMetaEl = document.getElementById("gridMeta");
const sweepDirLabelEl = document.getElementById("sweepDirLabel");

const statFrames = document.getElementById("statFrames");
const statCrcFail = document.getElementById("statCrcFail");
const statSeqLoss = document.getElementById("statSeqLoss");
const statLastFrame = document.getElementById("statLastFrame");

const batteryFill = document.getElementById("batteryFill");
const batteryLabel = document.getElementById("batteryLabel");
const faultFlagsEl = document.getElementById("faultFlags");
const healthAgeEl = document.getElementById("healthAge");

const activityLogEl = document.getElementById("activityLog");
const toastEl = document.getElementById("toast");

let lastSweepCount = 0;
let toastTimer = null;

// ---------- small UI helpers ----------

function logActivity(message, kind = "info") {
  const li = document.createElement("li");
  const time = document.createElement("span");
  time.className = "log-time";
  time.textContent = new Date().toLocaleTimeString();
  const text = document.createElement("span");
  text.className = `log-${kind}`;
  text.textContent = message;
  li.appendChild(time);
  li.appendChild(text);
  activityLogEl.appendChild(li);
  // Cap the log so a long-running tab doesn't accumulate unbounded DOM nodes.
  while (activityLogEl.children.length > 40) {
    activityLogEl.removeChild(activityLogEl.firstChild);
  }
}

function showToast(message, kind = "info") {
  toastEl.textContent = message;
  toastEl.className = kind === "err" ? "visible toast-err" : kind === "ok" ? "visible toast-ok" : "visible";
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { toastEl.className = ""; }, 2600);
}

function timeAgo(epochSeconds) {
  if (epochSeconds == null) return "never";
  const deltaMs = Date.now() - epochSeconds * 1000;
  if (deltaMs < 0) return "just now";
  const s = Math.floor(deltaMs / 1000);
  if (s < 2) return "just now";
  if (s < 60) return `${s}s ago`;
  const m = Math.floor(s / 60);
  if (m < 60) return `${m}m ago`;
  return `${Math.floor(m / 60)}h ago`;
}

// ---------- connection indicators ----------

function setWsState(connected) {
  wsDot.className = "dot " + (connected ? "dot-ok" : "dot-bad");
  wsLabel.textContent = connected ? "connected" : "disconnected";
}

// A frame received in the last 4s counts as "live" -- chosen to comfortably
// cover the simulators' fastest per-sample interval plus one missed beat,
// without flapping between "live" and "stale" on normal jitter.
const LINK_LIVE_WINDOW_S = 4;

function setLinkState(lastFrameAt) {
  const live = lastFrameAt != null && (Date.now() / 1000 - lastFrameAt) < LINK_LIVE_WINDOW_S;
  linkDot.className = "dot " + (live ? "dot-ok" : lastFrameAt == null ? "dot-bad" : "dot-warn");
  linkLabel.textContent = live ? "telemetry live" : lastFrameAt == null ? "no telemetry" : "telemetry stale";
}

// ---------- grid rendering ----------

function renderGrid(grid) {
  const { width_cells, height_cells, cells, origin_x, origin_y, resolution_m, sweep_count } = grid;
  if (!width_cells || !height_cells) return;

  if (canvas.width !== width_cells || canvas.height !== height_cells) {
    canvas.width = width_cells;
    canvas.height = height_cells;
  }

  const imageData = ctx.createImageData(width_cells, height_cells);
  for (let row = 0; row < height_cells; row++) {
    const rowData = cells[row];
    for (let col = 0; col < width_cells; col++) {
      const color = cellColorRgb(rowData[col]);
      const idx = (row * width_cells + col) * 4;
      imageData.data[idx] = color[0];
      imageData.data[idx + 1] = color[1];
      imageData.data[idx + 2] = color[2];
      imageData.data[idx + 3] = 255;
    }
  }
  ctx.putImageData(imageData, 0, 0);

  // Sensor origin marker, drawn on top of the ImageData.
  ctx.fillStyle = "#3ddbd1";
  ctx.fillRect(origin_x - 2, origin_y - 2, 5, 5);

  canvasOverlay.classList.toggle("hidden", sweep_count > 0);
  gridMetaEl.textContent = `${width_cells} × ${height_cells} cells · ${resolution_m} m/cell`;
  sweepCountEl.textContent = String(sweep_count);

  if (sweep_count > lastSweepCount) {
    logActivity(`sweep #${sweep_count} integrated into the grid`, "ok");
  }
  lastSweepCount = sweep_count;
}

// Quantized cell value -> [r,g,b]. -1 = unknown (charcoal), 0 = free (cool
// slate), 100 = occupied (warm amber), interpolated in between. Mirrors the
// server's quantization in OccupancyGrid.to_serializable().
function cellColorRgb(v) {
  if (v < 0) return [26, 30, 36];
  const t = Math.min(1, Math.max(0, v / 100));
  const r = Math.round(40 + t * (245 - 40));
  const g = Math.round(58 + t * (166 - 58));
  const b = Math.round(72 + t * (35 - 72));
  return [r, g, b];
}

// ---------- websocket ----------

function connectWs() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.onopen = () => {
    setWsState(true);
    logActivity("dashboard connection established", "ok");
  };
  ws.onclose = () => {
    setWsState(false);
    logActivity("dashboard connection lost, retrying…", "err");
    setTimeout(connectWs, 2000); // fixed-delay reconnect, fine for a LAN dashboard
  };
  ws.onerror = () => setWsState(false);
  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.kind === "grid_update") {
      renderGrid(msg.grid);
      if (msg.last_sweep_dir != null) {
        sweepDirLabelEl.textContent = msg.last_sweep_dir === 0 ? "last sweep: forward →" : "last sweep: reverse ←";
      }
    }
  };
}

// ---------- status polling ----------

function renderFaultFlags(flags) {
  if (flags == null) return "—";
  if (flags === 0) return "nominal";
  return `0x${flags.toString(16).padStart(4, "0")}`;
}

function batteryPercent(mv) {
  // Rough 2S Li-ion range (6000mV empty .. 8400mV full) purely for the bar
  // visualization -- the dashboard doesn't know the bot's actual pack
  // chemistry/cell count, this is just a sane default for display.
  if (mv == null) return null;
  const pct = ((mv - 6000) / (8400 - 6000)) * 100;
  return Math.max(0, Math.min(100, pct));
}

async function pollStatus() {
  try {
    const res = await fetch("/status");
    const data = await res.json();

    const ls = data.link_stats || {};
    statFrames.textContent = String(ls.frames_received ?? 0);
    statCrcFail.textContent = String(ls.frames_crc_failed ?? 0);
    statSeqLoss.textContent = String(ls.total_lost ?? 0);
    statLastFrame.textContent = timeAgo(ls.last_frame_at);
    setLinkState(ls.last_frame_at);

    const health = data.health;
    if (health) {
      const pct = batteryPercent(health.battery_mv);
      if (pct != null) {
        batteryFill.style.width = `${pct}%`;
        batteryFill.style.background = pct > 50 ? "var(--ok)" : pct > 20 ? "var(--warn)" : "var(--danger)";
      }
      batteryLabel.textContent = `${health.battery_mv} mV`;
      faultFlagsEl.textContent = renderFaultFlags(health.fault_flags);
      healthAgeEl.textContent = timeAgo(health.received_at);
    } else {
      batteryLabel.textContent = "— mV";
      faultFlagsEl.textContent = "—";
      healthAgeEl.textContent = "—";
    }
  } catch (e) {
    setLinkState(null);
    logActivity(`status fetch failed: ${e}`, "err");
  }
}

// ---------- controls ----------

async function sendControl(cmd, param1 = 0, param2 = 0, label = cmd) {
  try {
    const res = await fetch("/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ cmd, param1, param2 }),
    });
    if (!res.ok) {
      const body = await res.json().catch(() => ({}));
      throw new Error(body.detail || `HTTP ${res.status}`);
    }
    showToast(`sent: ${label}`, "ok");
    logActivity(`control sent: ${label}`, "ok");
  } catch (e) {
    showToast(`failed: ${label} (${e.message})`, "err");
    logActivity(`control failed: ${label} — ${e.message}`, "err");
  }
}

document.getElementById("btnStart").onclick = () => sendControl("start_scan", 0, 0, "start scan");
document.getElementById("btnStop").onclick = () => sendControl("stop_scan", 0, 0, "stop scan");
document.getElementById("btnPing").onclick = () => sendControl("ping", 0, 0, "ping");
document.getElementById("btnSetRange").onclick = () => {
  const minDeg = parseInt(document.getElementById("minDeg").value, 10) || 0;
  const maxDeg = parseInt(document.getElementById("maxDeg").value, 10) || 180;
  sendControl("set_sweep_range", minDeg * 100, maxDeg * 100, `sweep range ${minDeg}–${maxDeg}°`);
};

connectWs();
pollStatus();
setInterval(pollStatus, 2000);
