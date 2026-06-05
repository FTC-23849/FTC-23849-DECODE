"""
FTC Motor Current Log Viewer
Run: python server.py
Then open: http://localhost:5000
"""

from flask import Flask, jsonify, render_template_string
import pandas as pd
import os
import glob

app = Flask(__name__)
LOG_DIR = os.path.dirname(os.path.abspath(__file__))

MOTOR_COLS = [
    "lf_A", "rf_A", "lb_A", "rb_A",
    "frontIntake_A", "backIntake_A",
    "lShooter_A", "rShooter_A",
    "motor_total_A", "floodgate_A"
]

FUSE_LIMIT = 20  # amps — change to match your fuse

HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>FTC Current Logger</title>
    <script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: Arial, sans-serif; background: #1a1a2e; color: #eee; padding: 16px; }
        h1 { color: #e94560; margin-bottom: 16px; font-size: 20px; }

        .controls-row1 { display: flex; flex-wrap: wrap; gap: 12px; align-items: flex-start; margin-bottom: 12px; }
        .controls-row2 { margin-bottom: 16px; }

        .card { background: #16213e; border-radius: 8px; padding: 12px; }

        select { background: #0f3460; color: #eee; border: 1px solid #e94560;
                 border-radius: 4px; padding: 6px 10px; font-size: 14px; cursor: pointer; }

        .filter-group { display: flex; flex-wrap: wrap; gap: 6px; }
        .filter-group label { display: flex; align-items: center; gap: 5px;
                              background: #0f3460; border-radius: 4px; padding: 4px 10px;
                              cursor: pointer; font-size: 13px; user-select: none; }
        .filter-group label:hover { background: #e94560; }
        .filter-group input[type=checkbox] { accent-color: #e94560; width: 14px; height: 14px; }

        .btn { background: #e94560; color: white; border: none; border-radius: 4px;
               padding: 6px 14px; cursor: pointer; font-size: 13px; }
        .btn:hover { background: #c73652; }
        .btn-outline { background: transparent; border: 1px solid #e94560; color: #e94560; }
        .btn-outline:hover { background: #e94560; color: white; }

        #chart { width: 100%; height: 75vh; }

        .stats { display: flex; gap: 16px; flex-wrap: wrap; margin-bottom: 12px; }
        .stat { background: #16213e; border-radius: 6px; padding: 8px 14px; font-size: 13px; }
        .stat span { color: #e94560; font-weight: bold; font-size: 16px; display: block; }

        .fuse-warn { background: #5a1a1a; border: 1px solid #e94560; border-radius: 6px;
                     padding: 8px 14px; font-size: 13px; display: none; }
    </style>
</head>
<body>
    <h1>⚡ FTC Motor Current Logger</h1>

    <!-- Row 1: compact controls -->
    <div class="controls-row1">
        <div class="card">
            <div style="margin-bottom:8px; font-size:13px; color:#aaa;">Run File</div>
            <select id="fileSelect" onchange="loadFile()">
                <option value="">— select a run —</option>
            </select>
        </div>

        <div class="card">
            <div style="margin-bottom:8px; font-size:13px; color:#aaa;">Chart Type</div>
            <select id="chartType" onchange="redraw()"
                    style="background:#0f3460;color:#eee;border:1px solid #e94560;border-radius:4px;padding:6px 10px;font-size:14px;cursor:pointer;">
                <option value="line">Line</option>
                <option value="stacked">Stacked Area</option>
                <option value="area">Area (Overlap)</option>
                <option value="bar">Bar</option>
                <option value="scatter">Scatter (Dots)</option>
            </select>
        </div>

        <div class="card" style="display:flex;flex-direction:column;gap:8px;">
            <div style="font-size:13px; color:#aaa;">Fuse Limit (A)</div>
            <input id="fuseInput" type="number" value="{{ fuse }}" min="0" max="100" step="1"
                   style="width:70px; background:#0f3460; color:#eee; border:1px solid #e94560;
                          border-radius:4px; padding:5px 8px; font-size:14px;"
                   onchange="redraw()">
        </div>

        <div class="card" style="display:flex;align-items:flex-end;gap:8px;">
            <button class="btn" onclick="loadFile()">↻ Refresh</button>
            <button class="btn btn-outline" id="peakBtn" onclick="togglePeak()">📍 Mark Peak</button>
        </div>
    </div>

    <!-- Row 2: full-width channel filter -->
    <div class="controls-row2">
        <div class="card">
            <div style="margin-bottom:8px; font-size:13px; color:#aaa;">
                Show / Hide Channels &nbsp;
                <button class="btn btn-outline" onclick="selectAll(true)" style="padding:2px 8px;font-size:12px;">All</button>
                <button class="btn btn-outline" onclick="selectAll(false)" style="padding:2px 8px;font-size:12px;margin-left:4px;">None</button>
            </div>
            <div class="filter-group" id="checkboxes"></div>
        </div>
    </div>

    <div class="stats" id="stats"></div>
    <div class="fuse-warn" id="fuseWarn">⚠️ <b id="fuseWarnText"></b></div>

    <div id="chart"></div>

    <script>
    const ALL_COLS = {{ cols | tojson }};
    const COLORS = [
        '#e94560','#00d4ff','#7fff00','#ff9f00','#bf5fff',
        '#ff6b6b','#4ecdc4','#ffe66d','#a8e6cf','#ff8b94',
        '#2196f3','#ff5722'
    ];

    let currentData = null;
    let showPeak = false;
    let peakIndex = -1;

    // Build checkboxes
    const cbContainer = document.getElementById('checkboxes');
    ALL_COLS.forEach((col, i) => {
        const lbl = document.createElement('label');
        lbl.style.borderLeft = `3px solid ${COLORS[i % COLORS.length]}`;
        lbl.innerHTML = `<input type="checkbox" id="cb_${col}" checked onchange="redraw()"> ${col.replace('_A','')}`;
        cbContainer.appendChild(lbl);
    });

    // Load file list on page load
    fetch('/files').then(r => r.json()).then(files => {
        const sel = document.getElementById('fileSelect');
        files.forEach((f, i) => {
            const opt = document.createElement('option');
            opt.value = f.file;
            opt.text = f.label;
            if (i === 0) opt.selected = true;
            sel.appendChild(opt);
        });
        if (files.length > 0) loadFile();
    });

    function loadFile() {
        const f = document.getElementById('fileSelect').value;
        if (!f) return;
        fetch(`/data/${f}`).then(r => r.json()).then(d => {
            if (d.error) { document.getElementById('chart').innerHTML = `<p style="color:red;padding:20px">Server error: ${d.error}</p>`; return; }
            currentData = d;
            try { redraw(); } catch(e) { document.getElementById('chart').innerHTML = `<p style="color:red;padding:20px">redraw() error: ${e}</p>`; console.error(e); }
            try { updateStats(); } catch(e) { console.error('updateStats error:', e); }
        }).catch(e => {
            document.getElementById('chart').innerHTML = `<p style="color:red;padding:20px">Fetch error: ${e}</p>`;
        });
    }

    function redraw() {
        if (!currentData) return;
        const fuse = parseFloat(document.getElementById('fuseInput').value) || 20;
        const chartType = document.getElementById('chartType').value;
        const traces = [];

        const activeCols = ALL_COLS.filter(col => {
            const cb = document.getElementById(`cb_${col}`);
            return cb && cb.checked && currentData[col];
        });

        activeCols.forEach((col, i) => {
            const color = COLORS[ALL_COLS.indexOf(col) % COLORS.length];
            const label = col.replace('_A','');
            let trace = {
                x: currentData.timestamps,
                y: currentData[col],
                name: label,
                hovertemplate: '%{x}<br>%{y:.3f} A<extra>' + label + '</extra>'
            };

            if (chartType === 'line') {
                trace.type = 'scatter';
                trace.mode = 'lines';
                trace.line = { color, width: 1.5 };

            } else if (chartType === 'stacked') {
                trace.type = 'scatter';
                trace.mode = 'lines';
                trace.stackgroup = 'one';
                trace.fill = i === 0 ? 'tozeroy' : 'tonexty';
                trace.line = { color, width: 1 };
                trace.fillcolor = color + '55';

            } else if (chartType === 'area') {
                trace.type = 'scatter';
                trace.mode = 'lines';
                trace.fill = 'tozeroy';
                trace.line = { color, width: 1.5 };
                trace.fillcolor = color + '33';

            } else if (chartType === 'bar') {
                trace.type = 'bar';
                trace.marker = { color };

            } else if (chartType === 'scatter') {
                trace.type = 'scatter';
                trace.mode = 'markers';
                trace.marker = { color, size: 3 };
            }

            traces.push(trace);
        });

        // Fuse limit line — only on non-stacked charts (stacked Y is cumulative so it's misleading)
        if (chartType !== 'stacked') {
            traces.push({
                x: [currentData.timestamps[0], currentData.timestamps[currentData.timestamps.length-1]],
                y: [fuse, fuse],
                name: `Fuse (${fuse}A)`,
                type: 'scatter',
                mode: 'lines',
                line: { color: 'red', width: 1.5, dash: 'dash' },
                hoverinfo: 'skip'
            });
        }

        const barMode = chartType === 'bar' ? 'stack' : undefined;

        const layout = {
            paper_bgcolor: '#1a1a2e',
            plot_bgcolor: '#0f3460',
            font: { color: '#eee', size: 12 },
            barmode: barMode,
            xaxis: {
                title: 'Time',
                type: chartType === 'bar' ? 'category' : 'date',
                gridcolor: '#1e4080',
                tickformat: '%H:%M:%S',
                nticks: 20
            },
            yaxis: {
                title: chartType === 'stacked' ? 'Cumulative Current (A)' : 'Current (A)',
                gridcolor: '#1e4080',
                rangemode: 'tozero',
                fixedrange: true
            },
            legend: {
                bgcolor: '#16213e',
                bordercolor: '#333',
                borderwidth: 1,
                orientation: 'h',
                y: -0.18
            },
            hovermode: 'x unified',
            margin: { t: 20, b: 130, l: 60, r: 20 }
        };

        // Peak marker
        if (showPeak && peakIndex >= 0 && currentData.timestamps[peakIndex]) {
            const peakTime = currentData.timestamps[peakIndex];
            const peakTotal = currentData.hub_total_A ? currentData.hub_total_A[peakIndex] : '?';
            layout.shapes = [{
                type: 'line',
                x0: peakTime, x1: peakTime,
                y0: 0, y1: 1,
                yref: 'paper',
                line: { color: 'orange', width: 2, dash: 'dot' }
            }];
            layout.annotations = [{
                x: peakTime,
                y: 1,
                yref: 'paper',
                text: `⚡ Peak: ${typeof peakTotal === 'number' ? peakTotal.toFixed(2) : peakTotal}A`,
                showarrow: true,
                arrowhead: 2,
                arrowcolor: 'orange',
                font: { color: 'orange', size: 12 },
                bgcolor: '#1a1a2e',
                bordercolor: 'orange',
                borderwidth: 1,
                ax: 40, ay: -30
            }];
        } else {
            layout.shapes = [];
            layout.annotations = [];
        }

        const config = {
            responsive: true,
            scrollZoom: true,       // mouse wheel zooms X axis
            displayModeBar: true,
            modeBarButtonsToRemove: ['lasso2d', 'select2d', 'autoScale2d'],
            modeBarButtonsToAdd: [{
                name: 'Reset X Zoom',
                icon: Plotly.Icons.home,
                click: () => Plotly.relayout('chart', { 'xaxis.autorange': true })
            }]
        };
        layout.dragmode = 'zoom';
        Plotly.react('chart', traces, layout, config);
    }

    function togglePeak() {
        showPeak = !showPeak;
        const btn = document.getElementById('peakBtn');
        btn.style.background = showPeak ? '#e94560' : '';
        btn.style.color = showPeak ? 'white' : '';
        redraw();
    }

    function updateStats() {
        if (!currentData || !currentData.hub_total_A) return;
        const fuse = parseFloat(document.getElementById('fuseInput').value) || 20;
        const total = currentData.hub_total_A;
        // Track peak index for marker
        peakIndex = total.reduce((maxIdx, v, i, arr) =>
            (v !== null && (arr[maxIdx] === null || v > arr[maxIdx])) ? i : maxIdx, 0);
        const max = Math.max(...total).toFixed(2);
        const avg = (total.reduce((a,b)=>a+b,0)/total.length).toFixed(2);
        const overCount = total.filter(v => v > fuse).length;
        const validTs = currentData.timestamps.filter(t => t != null);
        const duration = validTs.length > 1
            ? ((new Date(validTs[validTs.length-1]) - new Date(validTs[0])) / 1000).toFixed(1)
            : 0;

        document.getElementById('stats').innerHTML = `
            <div class="stat">Peak Hub Current<span>${max} A</span></div>
            <div class="stat">Avg Hub Current<span>${avg} A</span></div>
            <div class="stat">Samples Over Fuse<span style="color:${overCount>0?'#e94560':'#7fff00'}">${overCount}</span></div>
            <div class="stat">Run Duration<span>${duration} s</span></div>
            <div class="stat">Sample Rate<span>${(total.length / duration * 1000 / 1000).toFixed(0)} Hz</span></div>
        `;

        const warn = document.getElementById('fuseWarn');
        if (overCount > 0) {
            warn.style.display = 'block';
            document.getElementById('fuseWarnText').textContent =
                `hub_total exceeded ${fuse}A in ${overCount} samples — fuse blowing risk!`;
        } else {
            warn.style.display = 'none';
        }
    }

    function selectAll(val) {
        ALL_COLS.forEach(col => {
            const cb = document.getElementById(`cb_${col}`);
            if (cb) cb.checked = val;
        });
        redraw();
    }
    </script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML, fuse=FUSE_LIMIT, cols=MOTOR_COLS)

def parse_filename(filename):
    """
    Parses filenames like:
      TeleOpMTI_20260530_143012_to_143512.csv   → clean run
      TeleOpMTI_20260530_143012.csv             → interrupted (robot died)
    Returns a display label.
    """
    import re
    name = os.path.splitext(filename)[0]
    # New format: OpMode_YYYYMMDD_HHmmss_to_HHmmss
    m = re.match(r'^(.+?)_(\d{8})_(\d{6})_to_(\d{6})$', name)
    if m:
        opmode, date, start, end = m.groups()
        d = f"{date[6:8]}/{date[4:6]}/{date[0:4]}"
        s = f"{start[0:2]}:{start[2:4]}:{start[4:6]}"
        e = f"{end[0:2]}:{end[2:4]}:{end[4:6]}"
        return f"{opmode}  {d}  {s} → {e}"
    # New format without end time: interrupted run
    m2 = re.match(r'^(.+?)_(\d{8})_(\d{6})$', name)
    if m2:
        opmode, date, start = m2.groups()
        d = f"{date[6:8]}/{date[4:6]}/{date[0:4]}"
        s = f"{start[0:2]}:{start[2:4]}:{start[4:6]}"
        return f"{opmode}  {d}  {s}  ⚡ INTERRUPTED"
    # Old format fallback: motor_current_YYYYMMDD_HHmmss
    m3 = re.match(r'^motor_current_(\d{8})_(\d{6})$', name)
    if m3:
        date, start = m3.groups()
        d = f"{date[6:8]}/{date[4:6]}/{date[0:4]}"
        s = f"{start[0:2]}:{start[2:4]}:{start[4:6]}"
        return f"[legacy]  {d}  {s}"
    return filename

@app.route("/files")
def list_files():
    files = sorted(glob.glob(os.path.join(LOG_DIR, "*.csv")), reverse=True)
    result = []
    for f in files:
        base = os.path.basename(f)
        result.append({"file": base, "label": parse_filename(base)})
    return jsonify(result)

@app.route("/data/<filename>")
def get_data(filename):
    # Safety check — only serve CSV files from log dir, no path traversal
    if not filename.endswith(".csv") or "/" in filename or "\\" in filename:
        return jsonify({"error": "invalid file"}), 400
    path = os.path.join(LOG_DIR, filename)
    if not os.path.exists(path):
        return jsonify({"error": "not found"}), 404

    df = pd.read_csv(path)
    if df.empty:
        return jsonify({"error": "empty file"}), 400
    # Drop incomplete/truncated rows (e.g. last row cut off mid-write by power loss)
    ts_col = "wall_clock_ms" if "wall_clock_ms" in df.columns else "elapsed_ms"
    df[ts_col] = pd.to_numeric(df[ts_col], errors="coerce")
    if ts_col == "wall_clock_ms":
        df = df[df[ts_col] > 1_000_000_000_000]  # must be > year 2001 in ms
    else:
        df = df[df[ts_col] >= 0]
    df = df.dropna(subset=[ts_col])
    if df.empty:
        return jsonify({"error": "no valid rows"}), 400

    # All timestamps displayed in PDT (America/Los_Angeles, UTC-7)
    PDT = "America/Los_Angeles"

    # Support both old format (elapsed_ms) and new format (wall_clock_ms)
    if "wall_clock_ms" in df.columns:
        df["timestamps"] = (
            pd.to_datetime(df["wall_clock_ms"], unit="ms", utc=True)
            .dt.tz_convert(PDT)
            .dt.strftime("%Y-%m-%dT%H:%M:%S.%f")
        )
    else:
        # Reconstruct wall clock from filename timestamp + elapsed_ms
        import re
        base = os.path.splitext(filename)[0]
        m = re.search(r'(\d{8})_(\d{6})', base)
        try:
            file_start = pd.to_datetime(m.group(1) + m.group(2), format="%Y%m%d%H%M%S") if m else pd.Timestamp("2000-01-01")
        except Exception:
            file_start = pd.Timestamp("2000-01-01")
        df["timestamps"] = (
            (file_start + pd.to_timedelta(df["elapsed_ms"], unit="ms"))
            .dt.tz_localize("UTC")
            .dt.tz_convert(PDT)
            .dt.strftime("%Y-%m-%dT%H:%M:%S.%f")
        )

    result = {"timestamps": df["timestamps"].tolist()}
    for col in MOTOR_COLS:
        if col in df.columns:
            result[col] = [None if pd.isna(v) else round(float(v), 4) for v in df[col]]

    return jsonify(result)

if __name__ == "__main__":
    import webbrowser, threading, time
    def open_browser():
        time.sleep(0.8)
        webbrowser.open("http://localhost:5000")
    threading.Thread(target=open_browser, daemon=True).start()
    print("Starting FTC Current Log Viewer at http://localhost:5000")
    app.run(host="0.0.0.0", port=5000, debug=False)
