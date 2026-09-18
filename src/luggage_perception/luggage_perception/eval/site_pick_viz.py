"""Eval-only HTML visualizer for site pick bag replay. Not online."""
from __future__ import division

import json
import os


_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>Site pick replay</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<style>
 body { font-family: sans-serif; margin: 12px; background: #111; color: #eee; }
 h1 { font-size: 18px; margin: 0 0 8px; }
 .meta { font-size: 13px; color: #bbb; margin-bottom: 10px; }
 .row { display: flex; gap: 12px; flex-wrap: wrap; }
 .card { background: #1c1c1c; padding: 8px; border-radius: 6px; }
 img { max-width: 640px; width: 100%; background: #000; }
 input[type=range] { width: 100%; }
 .warn { color: #f6c344; }
</style>
</head>
<body>
<h1>示教包回灌：当前算法取箱 vs 实机关节/TCP</h1>
<div class="meta" id="meta"></div>
<div class="row">
  <div class="card">
    <div>帧 <span id="frameLabel">0</span></div>
    <input id="slider" type="range" min="0" max="0" value="0"/>
    <div style="position:relative;display:inline-block">
      <img id="overlay" alt="overlay"/>
      <div id="labelMark" style="position:absolute;width:14px;height:14px;
        border:2px solid #00ff88;border-radius:50%;margin:-8px 0 0 -8px;
        display:none;pointer-events:none"></div>
    </div>
    <pre id="pickText"></pre>
  </div>
  <div class="card" style="flex:1;min-width:420px">
    <div id="xyz" style="height:420px"></div>
  </div>
</div>
<div class="card" id="labelPanel" style="margin-top:12px;display:none">
  <b>标注模式</b>：在图上点击“可安全吸附的箱盖中心”。
  <button id="copyLabels" type="button">复制 labels JSON</button>
  <span id="labelCount"></span>
  <textarea id="labelOut" rows="6" style="width:100%"></textarea>
</div>
<div class="card" style="margin-top:12px">
  <div id="joints" style="height:360px"></div>
</div>
<script>
const DATA = __DATA__;
function $(id) { return document.getElementById(id); }
function setFrame(i) {
  const frames = DATA.frames || [];
  if (!frames.length) return;
  i = Math.max(0, Math.min(frames.length - 1, i));
  const fr = frames[i];
  $("slider").value = i;
  $("frameLabel").textContent = i + " / " + (frames.length - 1)
    + "  t=" + (fr.stamp_sec || 0).toFixed(3);
  if (fr.overlay_jpeg_b64) {
    $("overlay").src = "data:image/jpeg;base64," + fr.overlay_jpeg_b64;
  }
  const pick = fr.pick || null;
  const lines = [
    "join=" + (fr.join || "-"),
    "cargo_pts=" + (fr.n_cargo_points || 0),
    "yolo_conf=" + (fr.yolo_conf == null ? "-" : fr.yolo_conf.toFixed(3)),
    "top_valid=" + (pick ? pick.top_surface_valid : false),
    "reason=" + (fr.reason || "-"),
    "tf=" + (fr.tf_ok ? "ok" : "miss"),
  ];
  if (pick) {
    lines.push("pick_xyz=" + pick.xyz.map(v => v.toFixed(3)).join(", "));
    lines.push("top_z=" + pick.top_z.toFixed(3)
      + "  height_valid=" + pick.height_valid);
  }
  if (fr.tcp_xyz) {
    lines.push("tcp_xyz=" + fr.tcp_xyz.map(v => v.toFixed(3)).join(", "));
  }
  $("pickText").textContent = lines.join("\\n");
}
function traces3d() {
  const traces = [];
  const tcp = DATA.tcp || {};
  if (tcp.xyz && tcp.xyz.length) {
    traces.push({
      type: "scatter3d", mode: "lines", name: "recorded TCP",
      x: tcp.xyz.map(p => p[0]),
      y: tcp.xyz.map(p => p[1]),
      z: tcp.xyz.map(p => p[2]),
      line: {color: "#8ab4f8", width: 3},
    });
  }
  const picks = (DATA.frames || []).filter(f => f.pick);
  if (picks.length) {
    traces.push({
      type: "scatter3d", mode: "markers", name: "estimated pick",
      x: picks.map(f => f.pick.xyz[0]),
      y: picks.map(f => f.pick.xyz[1]),
      z: picks.map(f => f.pick.xyz[2]),
      marker: {size: 4, color: "#f28b82"},
    });
  }
  const sel = DATA.selected;
  const fr = (DATA.frames || [])[sel];
  if (fr && fr.waypoints && fr.waypoints.length) {
    traces.push({
      type: "scatter3d", mode: "lines+markers", name: "pick waypoints",
      x: fr.waypoints.map(w => w.xyz[0]),
      y: fr.waypoints.map(w => w.xyz[1]),
      z: fr.waypoints.map(w => w.xyz[2]),
      marker: {size: 5, color: "#81c995"},
      line: {color: "#81c995", width: 4},
    });
  }
  const planned = DATA.planned || {};
  if (planned.xyz && planned.xyz.length) {
    traces.push({
      type: "scatter3d", mode: "lines", name: "MoveIt plan TCP",
      x: planned.xyz.map(p => p[0]),
      y: planned.xyz.map(p => p[1]),
      z: planned.xyz.map(p => p[2]),
      line: {color: "#fdd663", width: 4, dash: "dot"},
    });
  }
  return traces;
}
function drawXyz() {
  Plotly.react("xyz", traces3d(), {
    paper_bgcolor: "#1c1c1c", plot_bgcolor: "#1c1c1c",
    font: {color: "#eee"},
    scene: {
      xaxis: {title: "X"}, yaxis: {title: "Y"}, zaxis: {title: "Z"},
      aspectmode: "data",
    },
    margin: {t: 24, l: 0, r: 0, b: 0},
    legend: {orientation: "h"},
  }, {responsive: true});
}
function drawJoints() {
  const rec = DATA.joints || {};
  const names = rec.names || [];
  const traces = names.map((name, j) => ({
    type: "scatter", mode: "lines", name: "rec " + name,
    x: rec.t_sec || [],
    y: (rec.q || []).map(row => row[j]),
    line: {width: 1},
  }));
  const planned = DATA.planned || {};
  if (planned.t_sec && planned.q) {
    names.forEach((name, j) => {
      traces.push({
        type: "scatter", mode: "lines", name: "plan " + name,
        x: planned.t_sec,
        y: planned.q.map(row => row[j]),
        line: {width: 2, dash: "dot"},
      });
    });
  }
  Plotly.react("joints", traces, {
    paper_bgcolor: "#1c1c1c", plot_bgcolor: "#1c1c1c",
    font: {color: "#eee"},
    xaxis: {title: "t (s)"}, yaxis: {title: "joint (rad)"},
    margin: {t: 24}, legend: {orientation: "h"},
  }, {responsive: true});
}
(function init() {
  const d = DATA.domain || {};
  $("meta").innerHTML =
    "bag <code>" + (DATA.bag || "") + "</code> · 感知 "
    + (d.perception || "offline")
    + " · MoveIt domain " + (d.moveit == null ? "off" : d.moveit)
    + " · 禁止 domain " + (d.reserved == null ? 7 : d.reserved)
    + (DATA.planned && DATA.planned.message
       ? " · <span class=warn>" + DATA.planned.message + "</span>" : "");
  const n = (DATA.frames || []).length;
  $("slider").max = Math.max(0, n - 1);
  $("slider").addEventListener("input", e => setFrame(+e.target.value));
  setFrame(DATA.selected || 0);
  drawXyz();
  drawJoints();
  initLabelMode();
})();
function initLabelMode() {
  if (!DATA.label_mode) return;
  $("labelPanel").style.display = "block";
  const labels = [];
  const mark = $("labelMark");
  const img = $("overlay");
  img.addEventListener("click", e => {
    const fr = (DATA.frames || [])[+$("slider").value];
    if (!fr || !fr.frame_id) return;
    const r = img.getBoundingClientRect();
    const sx = (img.naturalWidth || r.width) / r.width;
    const sy = (img.naturalHeight || r.height) / r.height;
    const u = Math.round((e.clientX - r.left) * sx);
    const v = Math.round((e.clientY - r.top) * sy);
    labels.push({
      frame_id: fr.frame_id,
      stamp: fr.stamp_sec,
      suction_safe_lid_center_pixel: [u, v],
    });
    mark.style.display = "block";
    mark.style.left = (u / sx) + "px";
    mark.style.top = (v / sy) + "px";
    $("labelCount").textContent = "已标 " + labels.length + " 帧";
    $("labelOut").value = JSON.stringify({labels: labels}, null, 2);
  });
  $("copyLabels").addEventListener("click", () => {
    $("labelOut").select();
    document.execCommand("copy");
  });
}
</script>
</body>
</html>
"""


def write_viz_html(payload, path):
    """Write a self-contained Plotly page. *payload* is JSON-serialisable."""
    blob = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
    blob = blob.replace("<", "\\u003c")
    html = _HTML.replace("__DATA__", blob)
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(html)
    return path
