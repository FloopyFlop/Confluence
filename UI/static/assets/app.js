(function(){const e=document.createElement("link").relList;if(e&&e.supports&&e.supports("modulepreload"))return;for(const n of document.querySelectorAll('link[rel="modulepreload"]'))l(n);new MutationObserver(n=>{for(const r of n)if(r.type==="childList")for(const c of r.addedNodes)c.tagName==="LINK"&&c.rel==="modulepreload"&&l(c)}).observe(document,{childList:!0,subtree:!0});function a(n){const r={};return n.integrity&&(r.integrity=n.integrity),n.referrerPolicy&&(r.referrerPolicy=n.referrerPolicy),n.crossOrigin==="use-credentials"?r.credentials="include":n.crossOrigin==="anonymous"?r.credentials="omit":r.credentials="same-origin",r}function l(n){if(n.ep)return;n.ep=!0;const r=a(n);fetch(n.href,r)}})();const N=document.getElementById("app");N.innerHTML=`
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
`;const F=document.getElementById("status-stack"),h=document.getElementById("details-content"),L=document.getElementById("telemetry-grid"),v=document.getElementById("log-list"),p=document.getElementById("connection-pill"),_=document.getElementById("status-pill"),g=document.getElementById("theme-toggle"),$=document.getElementById("theme-label"),b="confluence-ui-theme";function T(){const t=window.localStorage.getItem(b);return t==="dark"||t==="light"?t:window.matchMedia("(prefers-color-scheme: dark)").matches?"dark":"light"}function k(t){const e=t==="dark"?"dark":"light";document.documentElement.dataset.theme=e,g&&g.setAttribute("aria-pressed",e==="dark"?"true":"false"),$&&($.textContent=e==="dark"?"Dark":"Light")}g&&g.addEventListener("click",()=>{const e=(document.documentElement.dataset.theme==="dark"?"dark":"light")==="dark"?"light":"dark";window.localStorage.setItem(b,e),k(e)});k(T());function o(t){return String(t??"").replace(/&/g,"&amp;").replace(/</g,"&lt;").replace(/>/g,"&gt;").replace(/\"/g,"&quot;").replace(/'/g,"&#39;")}function i(t,e=2){if(t==null||t==="")return"unknown";const a=Number(t);return Number.isNaN(a)?o(t):a.toFixed(e)}function u(t){return t==null?"unknown":t?"true":"false"}function w(t){return t?new Date(t*1e3).toLocaleTimeString():"--"}function f(t){return t?`${Math.max(0,Date.now()/1e3-Number(t)).toFixed(1)}s ago`:"--"}function D(t){return t==="fault_detected"?"Fault Detected":t==="fault_fixing"?"Fault Fixing":"Operational"}function M(t){return t==="fault_detected"?"is-red":t==="fault_fixing"?"is-blue":"is-green"}function s(t,e){return`<div class="kv"><span class="k">${o(t)}</span><span class="v">${o(e)}</span></div>`}function E(t){const e=t.derived.current_status;F.querySelectorAll(".status-item").forEach(a=>{a.classList.toggle("active",a.dataset.status===e)}),_.textContent=D(e),_.className=`pill ${M(e)}`}function S(t){var l,n,r,c,d,m;const e=t.telemetry,a=t.model;return`
    <article class="card">
      <h3>Flight Status</h3>
      <div class="kv-grid">
        ${s("armed",u(e.armed))}
        ${s("gate_open",u(e.gate_open))}
        ${s("throttle",i(e.throttle,0))}
        ${s("throttle_up",u(e.throttle_up))}
        ${s("heartbeat",f(e.last_message_ts))}
        ${s("stream_rate",`${i(e.raw_stream_rate_hz,2)} Hz`)}
      </div>
    </article>

    <article class="card">
      <h3>Model Health</h3>
      <div class="kv-grid">
        ${s("running",u(a.running))}
        ${s("loaded",u(a.loaded))}
        ${s("last_confidence",i(a.last_confidence,3))}
        ${s("last_inference",f(a.last_inference_ts))}
        ${s("batches_received",e.batches_received??0)}
        ${s("inferences_run",e.inferences_run??0)}
      </div>
    </article>

    <article class="card">
      <h3>Position</h3>
      <div class="kv-grid">
        ${s("lat",i((l=e.gps)==null?void 0:l.lat,6))}
        ${s("lon",i((n=e.gps)==null?void 0:n.lon,6))}
        ${s("alt_m",i((r=e.gps)==null?void 0:r.alt_m,2))}
        ${s("local_x",i((c=e.local_position_ned)==null?void 0:c.x,2))}
        ${s("local_y",i((d=e.local_position_ned)==null?void 0:d.y,2))}
        ${s("local_z",i((m=e.local_position_ned)==null?void 0:m.z,2))}
      </div>
    </article>
  `}function O(t){const e=t.fault.active||{},a=e.parameters||{},l=Object.keys(a).length?Object.entries(a).map(([n,r])=>s(n,r)).join(""):'<div class="empty">No parameter fix map found.</div>';return`
    <article class="card">
      <h3>Fault Summary</h3>
      <span class="badge is-red">Fault Detected</span>
      <div class="kv-grid">
        ${s("label",e.label??"unknown")}
        ${s("source",e.source??"unknown")}
        ${s("confidence",i(e.confidence,3))}
        ${s("raw_pred",e.raw_pred??"--")}
        ${s("stable_pred",e.stable_pred??"--")}
        ${s("detected",w(e.timestamp))}
      </div>
    </article>

    <article class="card">
      <h3>Planned Fix</h3>
      <div class="kv-grid">${l}</div>
    </article>

    <article class="card">
      <h3>Raw Fault JSON</h3>
      <pre class="json-block">${o(JSON.stringify(e.raw_json||{},null,2))}</pre>
    </article>
  `}function I(t){const e=Number(t.fixing_started_at||0),a=Number(t.fixing_ends_at||0);if(!e||!a||a<=e)return{pct:0,remain:0};const l=Date.now()/1e3,n=Math.min(1,Math.max(0,(l-e)/(a-e))),r=Math.max(0,a-l);return{pct:n,remain:r}}function P(t){const e=t.fault.last_fix||{},a=Array.isArray(e.changes)?e.changes:[],l=I(t.derived),n=a.length?a.map(r=>{const c=r.ok,d=c==null?"--":c?"ok":"fail";return`
            <tr>
              <td class="mono">${o(r.param)}</td>
              <td>${o(r.previous_value??"unknown")}</td>
              <td>${o(r.new_value??"unknown")}</td>
              <td>${o(d)}</td>
              <td>${o(r.reason??"")}</td>
            </tr>
          `}).join(""):'<tr><td colspan="5" class="empty">No injector detail rows available.</td></tr>';return`
    <article class="card">
      <h3>Fault Fixing</h3>
      <span class="badge is-blue">Applying Parameter Changes</span>
      <div class="progress-shell">
        <div class="progress-fill" style="width: ${(l.pct*100).toFixed(1)}%;"></div>
      </div>
      <div class="progress-label">
        <span>Animated Demo Duration</span>
        <span>${l.remain.toFixed(1)}s remaining</span>
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
        <tbody>${n}</tbody>
      </table>
      <div class="kv-grid">
        ${s("source",e.source??"unknown")}
        ${s("queued",u(e.queued))}
      </div>
    </article>

    <article class="card">
      <h3>Raw Injector JSON</h3>
      <pre class="json-block">${o(JSON.stringify(e.raw_json||{},null,2))}</pre>
    </article>
  `}function j(t){const e=t.derived.current_status;if(e==="fault_detected"){h.innerHTML=O(t);return}if(e==="fault_fixing"){h.innerHTML=P(t);return}h.innerHTML=S(t)}function y(t,e,a,l="cool"){if(!Array.isArray(t)||t.length===0)return'<div class="empty">No samples yet.</div>';const n=Math.max(1e-4,a-e);return`<div class="strip">${t.slice(-20).map(c=>{const d=Number(c.v),m=Number.isNaN(d)?.15:Math.max(.06,Math.min(1,(d-e)/n));return`<span class="${l}" style="height:${(m*100).toFixed(1)}%"></span>`}).join("")}</div>`}function C(t){var n,r,c;const e=t.telemetry,a=t.model,l=Object.entries(e.message_counts||{}).sort((d,m)=>Number(m[1])-Number(d[1])).slice(0,6).map(([d,m])=>`${d}:${m}`).join("  ");L.innerHTML=`
    <article class="card compact">
      <h3>Core Feed</h3>
      <div class="kv-grid">
        ${s("armed",u(e.armed))}
        ${s("gate_open",u(e.gate_open))}
        ${s("throttle",i(e.throttle,0))}
        ${s("stream_rate",`${i(e.raw_stream_rate_hz,2)} Hz`)}
        ${s("last_type",e.last_message_type??"--")}
        ${s("last_msg",f(e.last_message_ts))}
      </div>
    </article>

    <article class="card compact">
      <h3>Kinematics</h3>
      <div class="kv-grid">
        ${s("groundspeed",i(e.groundspeed,2))}
        ${s("airspeed",i(e.airspeed,2))}
        ${s("heading",i(e.heading,1))}
        ${s("lat",i((n=e.gps)==null?void 0:n.lat,6))}
        ${s("lon",i((r=e.gps)==null?void 0:r.lon,6))}
        ${s("alt_m",i((c=e.gps)==null?void 0:c.alt_m,2))}
      </div>
    </article>

    <article class="card compact">
      <h3>Model Confidence</h3>
      <div class="kv-grid">
        ${s("loaded",u(a.loaded))}
        ${s("running",u(a.running))}
        ${s("confidence",i(a.last_confidence,3))}
        ${s("last_inference",f(a.last_inference_ts))}
      </div>
      ${y(a.confidence_history,0,1,"cool")}
    </article>

    <article class="card compact">
      <h3>Throttle Trend</h3>
      ${y(e.throttle_history,950,2050,"hot")}
    </article>

    <article class="card compact">
      <h3>Message Counts</h3>
      <div class="mono">${o(l||"No messages yet")}</div>
    </article>
  `}function A(t){const e=Array.isArray(t.log)?t.log.slice(0,11):[];if(!e.length){v.innerHTML='<div class="log-entry empty">Waiting for state transitions...</div>';return}v.innerHTML=e.map(a=>`
        <div class="log-entry">
          <div class="log-line">
            <span class="log-title">${o(a.title)}</span>
            <span class="log-time">${w(a.timestamp)}</span>
          </div>
          <div class="log-detail">${o(a.detail||a.kind||"")}</div>
        </div>
      `).join("")}function H(t){const e=!!t.connection.connected;p.textContent=e?"Live Stream":"Disconnected",p.className=`pill ${e?"is-green":"is-red"}`}function B(t){E(t),j(t),C(t),A(t),H(t)}async function x(){try{const t=await fetch("/api/state",{cache:"no-store"});if(!t.ok)throw new Error(`HTTP ${t.status}`);const e=await t.json();B(e)}catch{p.textContent="API Error",p.className="pill is-red"}}setInterval(x,250);x();
