import "./styles.css";

const app = document.getElementById("app");

app.innerHTML = `
  <div class="ui-root">
    <div class="ambient-glow"></div>
    <header class="topbar panel">
      <div class="brand-wrap">
        <span class="kicker">Confluence</span>
        <h1>Fault Detection Demo UI</h1>
      </div>

      <div class="header-controls">
        <button id="theme-toggle" class="theme-toggle" type="button" aria-label="Toggle theme" aria-pressed="false">
          <span class="theme-toggle-track"><span class="theme-toggle-thumb"></span></span>
          <span id="theme-label">Dark</span>
        </button>

        <div class="header-pills">
          <span class="pill" id="connection-pill">Disconnected</span>
          <span class="pill" id="status-pill">Operational</span>
        </div>
      </div>
    </header>

    <main class="columns">
      <section class="panel section status-column">
        <div class="section-title">Status</div>
        <div class="status-stack" id="status-stack">
          <div class="status-item" data-status="operational">Operational</div>
          <div class="status-item" data-status="fault_detected">Fault Detected</div>
          <div class="status-item" data-status="fault_fixing">Fault Fixing</div>
        </div>
      </section>

      <section class="panel section details-column">
        <div class="section-title">Details</div>
        <div class="details-content" id="details-content"></div>
      </section>

      <section class="panel section telemetry-column">
        <div class="section-title">Telemetry & Log</div>
        <div class="log-card card">
          <h3 class="subheading">State Change Log</h3>
          <div id="log-list" class="log-list"></div>
        </div>
        <div id="telemetry-grid" class="telemetry-grid"></div>
      </section>
    </main>
  </div>
`;

const statusStack = document.getElementById("status-stack");
const detailsContent = document.getElementById("details-content");
const telemetryGrid = document.getElementById("telemetry-grid");
const logList = document.getElementById("log-list");
const connectionPill = document.getElementById("connection-pill");
const statusPill = document.getElementById("status-pill");
const themeToggle = document.getElementById("theme-toggle");
const themeLabel = document.getElementById("theme-label");

const THEME_KEY = "confluence-ui-theme";

function getPreferredTheme() {
  const stored = window.localStorage.getItem(THEME_KEY);
  if (stored === "dark" || stored === "light") {
    return stored;
  }
  return window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light";
}

function applyTheme(theme) {
  const normalized = theme === "dark" ? "dark" : "light";
  document.documentElement.dataset.theme = normalized;
  if (themeToggle) {
    themeToggle.setAttribute("aria-pressed", normalized === "dark" ? "true" : "false");
  }
  if (themeLabel) {
    themeLabel.textContent = normalized === "dark" ? "Dark" : "Light";
  }
}

if (themeToggle) {
  themeToggle.addEventListener("click", () => {
    const current = document.documentElement.dataset.theme === "dark" ? "dark" : "light";
    const next = current === "dark" ? "light" : "dark";
    window.localStorage.setItem(THEME_KEY, next);
    applyTheme(next);
  });
}

applyTheme(getPreferredTheme());

function esc(value) {
  return String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/\"/g, "&quot;")
    .replace(/'/g, "&#39;");
}

function fmtNum(value, digits = 2) {
  if (value === null || value === undefined || value === "") {
    return "unknown";
  }
  const n = Number(value);
  if (Number.isNaN(n)) {
    return esc(value);
  }
  return n.toFixed(digits);
}

function fmtBool(value) {
  if (value === null || value === undefined) {
    return "unknown";
  }
  return value ? "true" : "false";
}

function fmtTs(ts) {
  if (!ts) {
    return "--";
  }
  const date = new Date(ts * 1000);
  return date.toLocaleTimeString();
}

function fmtAge(ts) {
  if (!ts) {
    return "--";
  }
  const age = Math.max(0, Date.now() / 1000 - Number(ts));
  return `${age.toFixed(1)}s ago`;
}

function statusLabel(status) {
  if (status === "fault_detected") return "Fault Detected";
  if (status === "fault_fixing") return "Fault Fixing";
  return "Operational";
}

function statusColorClass(status) {
  if (status === "fault_detected") return "is-red";
  if (status === "fault_fixing") return "is-blue";
  return "is-green";
}

function kvRow(key, value) {
  return `<div class="kv"><span class="k">${esc(key)}</span><span class="v">${esc(value)}</span></div>`;
}

function renderStatus(snapshot) {
  const status = snapshot.derived.current_status;
  statusStack.querySelectorAll(".status-item").forEach((item) => {
    item.classList.toggle("active", item.dataset.status === status);
  });

  statusPill.textContent = statusLabel(status);
  statusPill.className = `pill ${statusColorClass(status)}`;
}

function renderOperationalDetails(snapshot) {
  const telemetry = snapshot.telemetry;
  const model = snapshot.model;

  return `
    <article class="card">
      <h3>Flight Status</h3>
      <div class="kv-grid">
        ${kvRow("armed", fmtBool(telemetry.armed))}
        ${kvRow("gate_open", fmtBool(telemetry.gate_open))}
        ${kvRow("throttle", fmtNum(telemetry.throttle, 0))}
        ${kvRow("throttle_up", fmtBool(telemetry.throttle_up))}
        ${kvRow("heartbeat", fmtAge(telemetry.last_message_ts))}
        ${kvRow("stream_rate", `${fmtNum(telemetry.raw_stream_rate_hz, 2)} Hz`)}
      </div>
    </article>

    <article class="card">
      <h3>Model Health</h3>
      <div class="kv-grid">
        ${kvRow("running", fmtBool(model.running))}
        ${kvRow("loaded", fmtBool(model.loaded))}
        ${kvRow("last_confidence", fmtNum(model.last_confidence, 3))}
        ${kvRow("last_inference", fmtAge(model.last_inference_ts))}
        ${kvRow("batches_received", telemetry.batches_received ?? 0)}
        ${kvRow("inferences_run", telemetry.inferences_run ?? 0)}
      </div>
    </article>

    <article class="card">
      <h3>Position</h3>
      <div class="kv-grid">
        ${kvRow("lat", fmtNum(telemetry.gps?.lat, 6))}
        ${kvRow("lon", fmtNum(telemetry.gps?.lon, 6))}
        ${kvRow("alt_m", fmtNum(telemetry.gps?.alt_m, 2))}
        ${kvRow("local_x", fmtNum(telemetry.local_position_ned?.x, 2))}
        ${kvRow("local_y", fmtNum(telemetry.local_position_ned?.y, 2))}
        ${kvRow("local_z", fmtNum(telemetry.local_position_ned?.z, 2))}
      </div>
    </article>
  `;
}

function renderFaultDetectedDetails(snapshot) {
  const active = snapshot.fault.active || {};
  const params = active.parameters || {};
  const paramsRows = Object.keys(params).length
    ? Object.entries(params)
        .map(([k, v]) => kvRow(k, v))
        .join("")
    : `<div class="empty">No parameter fix map found.</div>`;

  return `
    <article class="card">
      <h3>Fault Summary</h3>
      <span class="badge is-red">Fault Detected</span>
      <div class="kv-grid">
        ${kvRow("label", active.label ?? "unknown")}
        ${kvRow("source", active.source ?? "unknown")}
        ${kvRow("confidence", fmtNum(active.confidence, 3))}
        ${kvRow("raw_pred", active.raw_pred ?? "--")}
        ${kvRow("stable_pred", active.stable_pred ?? "--")}
        ${kvRow("detected", fmtTs(active.timestamp))}
      </div>
    </article>

    <article class="card">
      <h3>Planned Fix</h3>
      <div class="kv-grid">${paramsRows}</div>
    </article>

    <article class="card">
      <h3>Raw Fault JSON</h3>
      <pre class="json-block">${esc(JSON.stringify(active.raw_json || {}, null, 2))}</pre>
    </article>
  `;
}

function computeFixProgress(derived) {
  const start = Number(derived.fixing_started_at || 0);
  const end = Number(derived.fixing_ends_at || 0);
  if (!start || !end || end <= start) {
    return { pct: 0, remain: 0 };
  }
  const now = Date.now() / 1000;
  const pct = Math.min(1, Math.max(0, (now - start) / (end - start)));
  const remain = Math.max(0, end - now);
  return { pct, remain };
}

function renderFixingDetails(snapshot) {
  const fix = snapshot.fault.last_fix || {};
  const changes = Array.isArray(fix.changes) ? fix.changes : [];
  const progress = computeFixProgress(snapshot.derived);

  const rows = changes.length
    ? changes
        .map((change) => {
          const ok = change.ok;
          const okText = ok === null || ok === undefined ? "--" : ok ? "ok" : "fail";
          return `
            <tr>
              <td class="mono">${esc(change.param)}</td>
              <td>${esc(change.previous_value ?? "unknown")}</td>
              <td>${esc(change.new_value ?? "unknown")}</td>
              <td>${esc(okText)}</td>
              <td>${esc(change.reason ?? "")}</td>
            </tr>
          `;
        })
        .join("")
    : `<tr><td colspan="5" class="empty">No injector detail rows available.</td></tr>`;

  return `
    <article class="card">
      <h3>Fault Fixing</h3>
      <span class="badge is-blue">Applying Parameter Changes</span>
      <div class="progress-shell">
        <div class="progress-fill" style="width: ${(progress.pct * 100).toFixed(1)}%;"></div>
      </div>
      <div class="progress-label">
        <span>Animated Demo Duration</span>
        <span>${progress.remain.toFixed(1)}s remaining</span>
      </div>
    </article>

    <article class="card">
      <h3>Parameter Transition</h3>
      <table class="table">
        <thead>
          <tr>
            <th>Param</th>
            <th>Previous</th>
            <th>New</th>
            <th>Result</th>
            <th>Reason</th>
          </tr>
        </thead>
        <tbody>${rows}</tbody>
      </table>
      <div class="kv-grid">
        ${kvRow("source", fix.source ?? "unknown")}
        ${kvRow("queued", fmtBool(fix.queued))}
      </div>
    </article>

    <article class="card">
      <h3>Raw Injector JSON</h3>
      <pre class="json-block">${esc(JSON.stringify(fix.raw_json || {}, null, 2))}</pre>
    </article>
  `;
}

function renderDetails(snapshot) {
  const status = snapshot.derived.current_status;
  if (status === "fault_detected") {
    detailsContent.innerHTML = renderFaultDetectedDetails(snapshot);
    return;
  }
  if (status === "fault_fixing") {
    detailsContent.innerHTML = renderFixingDetails(snapshot);
    return;
  }
  detailsContent.innerHTML = renderOperationalDetails(snapshot);
}

function toStrip(values, min, max, variant = "cool") {
  if (!Array.isArray(values) || values.length === 0) {
    return `<div class="empty">No samples yet.</div>`;
  }
  const span = Math.max(0.0001, max - min);
  const bars = values
    .slice(-20)
    .map((sample) => {
      const value = Number(sample.v);
      const pct = Number.isNaN(value) ? 0.15 : Math.max(0.06, Math.min(1, (value - min) / span));
      return `<span class="${variant}" style="height:${(pct * 100).toFixed(1)}%"></span>`;
    })
    .join("");
  return `<div class="strip">${bars}</div>`;
}

function renderTelemetry(snapshot) {
  const telemetry = snapshot.telemetry;
  const model = snapshot.model;
  const topTypes = Object.entries(telemetry.message_counts || {})
    .sort((a, b) => Number(b[1]) - Number(a[1]))
    .slice(0, 6)
    .map(([k, v]) => `${k}:${v}`)
    .join("  ");

  telemetryGrid.innerHTML = `
    <article class="card compact">
      <h3>Core Feed</h3>
      <div class="kv-grid">
        ${kvRow("armed", fmtBool(telemetry.armed))}
        ${kvRow("gate_open", fmtBool(telemetry.gate_open))}
        ${kvRow("throttle", fmtNum(telemetry.throttle, 0))}
        ${kvRow("stream_rate", `${fmtNum(telemetry.raw_stream_rate_hz, 2)} Hz`)}
        ${kvRow("last_type", telemetry.last_message_type ?? "--")}
        ${kvRow("last_msg", fmtAge(telemetry.last_message_ts))}
      </div>
    </article>

    <article class="card compact">
      <h3>Kinematics</h3>
      <div class="kv-grid">
        ${kvRow("groundspeed", fmtNum(telemetry.groundspeed, 2))}
        ${kvRow("airspeed", fmtNum(telemetry.airspeed, 2))}
        ${kvRow("heading", fmtNum(telemetry.heading, 1))}
        ${kvRow("lat", fmtNum(telemetry.gps?.lat, 6))}
        ${kvRow("lon", fmtNum(telemetry.gps?.lon, 6))}
        ${kvRow("alt_m", fmtNum(telemetry.gps?.alt_m, 2))}
      </div>
    </article>

    <article class="card compact">
      <h3>Model Confidence</h3>
      <div class="kv-grid">
        ${kvRow("loaded", fmtBool(model.loaded))}
        ${kvRow("running", fmtBool(model.running))}
        ${kvRow("confidence", fmtNum(model.last_confidence, 3))}
        ${kvRow("last_inference", fmtAge(model.last_inference_ts))}
      </div>
      ${toStrip(model.confidence_history, 0, 1, "cool")}
    </article>

    <article class="card compact">
      <h3>Throttle Trend</h3>
      ${toStrip(telemetry.throttle_history, 950, 2050, "hot")}
    </article>

    <article class="card compact">
      <h3>Message Counts</h3>
      <div class="mono">${esc(topTypes || "No messages yet")}</div>
    </article>
  `;
}

function renderLog(snapshot) {
  const entries = Array.isArray(snapshot.log) ? snapshot.log.slice(0, 11) : [];
  if (!entries.length) {
    logList.innerHTML = `<div class="log-entry empty">Waiting for state transitions...</div>`;
    return;
  }

  logList.innerHTML = entries
    .map((entry) => {
      return `
        <div class="log-entry">
          <div class="log-line">
            <span class="log-title">${esc(entry.title)}</span>
            <span class="log-time">${fmtTs(entry.timestamp)}</span>
          </div>
          <div class="log-detail">${esc(entry.detail || entry.kind || "")}</div>
        </div>
      `;
    })
    .join("");
}

function renderConnection(snapshot) {
  const connected = Boolean(snapshot.connection.connected);
  connectionPill.textContent = connected ? "Live Stream" : "Disconnected";
  connectionPill.className = `pill ${connected ? "is-green" : "is-red"}`;
}

function render(snapshot) {
  renderStatus(snapshot);
  renderDetails(snapshot);
  renderTelemetry(snapshot);
  renderLog(snapshot);
  renderConnection(snapshot);
}

async function pollState() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const snapshot = await response.json();
    render(snapshot);
  } catch (_error) {
    connectionPill.textContent = "API Error";
    connectionPill.className = "pill is-red";
  }
}

setInterval(pollState, 250);
pollState();
