"""制御スクリプト実行中のモーター状態を、同一LAN上の別端末のブラウザへリアルタイム配信する
Webダッシュボード。

CLAUDE.md の方針（「ヘッドレスRaspberry Pi/Linux向けの構成のため、GUI依存関係を持ち込まない
こと」）に従い、Flask等の新規Webフレームワークやtkinter等のデスクトップGUIツールキットは
一切使わず、標準ライブラリの http.server (ThreadingHTTPServer) のみでWebサーバーを実装する。
ブラウザ側もSSE (Server-Sent Events) のネイティブAPI（EventSource）とバニラJSのみで完結させ、
追加のJSライブラリ・外部CDN・Webフォントには依存しない。

設計上のポイント:
- publish(t) は制御ループから毎イテレーション呼ばれる想定で、TMotorManager_mit_can.LOG_FUNCTIONS
  （update() でキャッシュ済みの状態を返すだけのゼロ引数ゲッター）を読むだけの O(1) 処理。
  CAN通信・ネットワークI/Oは一切行わないため、1kHz制御ループ（システム同定用途など）から
  毎回呼んでも安全。
- ブラウザへの実配信（SSE push）は、publish() を呼ぶスレッドとは別の daemon thread が
  push_interval（既定0.1秒=10Hz）ごとに独立して行う。制御ループの実周期（100Hz〜1kHz）や
  ブラウザ側の接続状況が制御ループ側の速度に一切影響しないよう、完全に非同期化している。
- SafetyMonitor / SyncMultiMotorLogger と同じく motors・motor_names のパラレルリストを
  受け取る設計にしており、単一モーター（motors=[motor]）・複数モーター（exp_003/007相当）の
  どちらにもそのまま使える。
- SafetyMonitor はオプションで連携できる（safety_monitor 引数）。publish() は
  safety_monitor.check() のみを呼ぶ（安全上限を超えているかを読むだけの純粋関数）。
  safety_monitor.update_and_check() は使わない — これは内部で motor.update() を呼ぶため、
  制御ループ側が既にその周期の update() を呼んだ後に publish() を呼ぶ想定の中で二重に
  update() してしまうと、CAN送信のタイミング・状態遷移を余計に乱すおそれがあるため。
  ダッシュボードは安全状態を表示するだけで、緊急停止の実行はあくまで制御ループ側
  （safety_monitor.update_and_check()）の責務のまま変えない。
"""

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class DashboardServer:
    """複数モーターの LOG_FUNCTIONS 値を、ブラウザへSSEでライブ配信するダッシュボードサーバー。"""

    def __init__(
        self, motors, motor_names, log_vars, host="0.0.0.0", port=8000, push_interval=0.1, safety_monitor=None
    ):
        """
        Args:
            motors: TMotorManager_mit_can インスタンスのリスト（with ブロックで既に __enter__ 済みのもの）。
            motor_names: motors と同じ順序のモーター識別名リスト。
            log_vars: 各モーターについて配信する変数名のリスト（TMotorManager_mit_can.LOG_FUNCTIONS のキー）。
            host: バインドするアドレス。既定 "0.0.0.0"（同一LAN上の別端末のブラウザから
                http://<このPiのIP>:<port>/ でアクセスできるようにする。認証は行わないため、
                信頼できないネットワークでは使わないこと）。
            port: バインドするポート。既定 8000。
            push_interval: ブラウザへのSSE配信間隔 [秒]。既定 0.1（10Hz）。制御ループの実際の
                周期とは独立。
            safety_monitor: lib.safety_monitor.SafetyMonitor インスタンス（省略可）。指定すると
                publish() のたびに safety_monitor.check() の結果（安全上限超過の有無・
                メッセージ）をダッシュボード上のバナーとして表示する。ダッシュボード自身は
                これを読むだけで、緊急停止の実行やモーターへのコマンド送信は一切行わない。
        """
        self._motors = motors
        self._motor_names = motor_names
        self._log_vars = log_vars
        self._host = host
        self._port = port
        self._push_interval = push_interval
        self._safety_monitor = safety_monitor

        self._lock = threading.Lock()
        self._state = {
            "t": None,
            "updated_at": None,
            "motors": {name: {} for name in motor_names},
            "safety_ok": None,
            "safety_message": None,
        }
        self._stop_event = threading.Event()

        self._httpd = None
        self._thread = None

    def __enter__(self):
        handler_cls = _make_handler_class(self)
        self._httpd = ThreadingHTTPServer((self._host, self._port), handler_cls)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, exc_type, exc_value, tb):
        self._stop_event.set()
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
        return False

    def publish(self, t):
        """制御ループから毎イテレーション呼ぶ。共有状態を更新するだけでネットワークI/Oは行わない。"""
        snapshot = {
            name: {var: motor.LOG_FUNCTIONS[var]() for var in self._log_vars}
            for name, motor in zip(self._motor_names, self._motors)
        }
        safety_ok = None
        safety_message = None
        if self._safety_monitor is not None:
            exceeded, message = self._safety_monitor.check()
            safety_ok = not exceeded
            safety_message = message
        with self._lock:
            self._state["t"] = t
            self._state["motors"] = snapshot
            self._state["updated_at"] = time.time()
            self._state["safety_ok"] = safety_ok
            self._state["safety_message"] = safety_message

    def _snapshot_payload(self):
        """HTTPハンドラ側から呼ぶ。共有状態のスナップショット＋鮮度（age_seconds）を返す。"""
        with self._lock:
            updated_at = self._state["updated_at"]
            payload = {
                "t": self._state["t"],
                "updated_at": updated_at,
                "motors": self._state["motors"],
                "safety_ok": self._state["safety_ok"],
                "safety_message": self._state["safety_message"],
            }
        payload["age_seconds"] = round(time.time() - updated_at, 2) if updated_at is not None else None
        return payload

    @property
    def push_interval(self):
        return self._push_interval

    @property
    def stop_event(self):
        return self._stop_event

    @property
    def url(self):
        """表示用URL。host="0.0.0.0" のままでは他端末から使えないため、実際にパケットを送らない
        OS側のルーティング解決でLAN到達可能なIPを推定して埋め込む。推定できない場合は
        手動確認を促す文言にフォールバックする。"""
        ip = _detect_lan_ip()
        host_for_display = ip if ip else "<このPiのIPアドレス（hostname -I 等で確認）>"
        return f"http://{host_for_display}:{self._port}/"


def _detect_lan_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except OSError:
        return None


def _make_handler_class(dashboard):
    class _DashboardRequestHandler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            # 毎リクエストのアクセスログは、制御スクリプト自身のコンソール出力と混ざって
            # 見づらくなるため抑制する。
            pass

        def do_GET(self):
            if self.path == "/":
                self._serve_html()
            elif self.path == "/api/state":
                self._serve_state_json()
            elif self.path == "/events":
                self._serve_events()
            else:
                self.send_error(404)

        def _serve_html(self):
            body = _DASHBOARD_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_state_json(self):
            body = json.dumps(dashboard._snapshot_payload()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while not dashboard.stop_event.is_set():
                    chunk = f"data: {json.dumps(dashboard._snapshot_payload())}\n\n".encode("utf-8")
                    self.wfile.write(chunk)
                    self.wfile.flush()
                    if dashboard.stop_event.wait(dashboard.push_interval):
                        break
            except (BrokenPipeError, ConnectionResetError):
                return

    return _DashboardRequestHandler


_DASHBOARD_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<title>TMotorCANControl ダッシュボード</title>
<style>
  body {
    font-family: -apple-system, "Segoe UI", "Hiragino Sans", sans-serif;
    background: #f5f6f8;
    color: #1a1a1a;
    margin: 0;
    padding: 1.5rem;
  }
  h1 { font-size: 1.3rem; margin: 0 0 0.75rem; }
  #freshness { margin-bottom: 0.5rem; color: #555; font-size: 0.9rem; }
  #freshness.stale { color: #b91c1c; font-weight: bold; }
  #safety-banner {
    margin-bottom: 1rem;
    font-size: 0.9rem;
    font-weight: bold;
    padding: 0.4rem 0.7rem;
    border-radius: 6px;
    display: inline-block;
  }
  #safety-banner.ok { color: #15803d; background: #dcfce7; }
  #safety-banner.exceeded { color: #b91c1c; background: #fee2e2; }
  #motors { display: flex; flex-wrap: wrap; gap: 1rem; }
  .motor-card {
    background: #fff;
    border: 1px solid #ddd;
    border-radius: 8px;
    padding: 1rem;
    min-width: 260px;
    box-shadow: 0 1px 2px rgba(0, 0, 0, 0.05);
  }
  .motor-card.stale { border-color: #b91c1c; }
  .motor-card h2 { font-size: 1rem; margin: 0 0 0.5rem; }
  .stale-badge { color: #b91c1c; font-size: 0.85rem; margin-bottom: 0.5rem; }
  .row { display: flex; justify-content: space-between; font-family: monospace; font-size: 0.95rem; padding: 2px 0; }
  .row-label { color: #555; }
  .row-value { font-weight: bold; }
  .row-unit { color: #999; margin-left: 0.4rem; }
  canvas { display: block; margin-top: 0.5rem; border: 1px solid #eee; border-radius: 4px; }
  .chart-caption { font-size: 0.75rem; color: #888; margin-top: 2px; }
</style>
</head>
<body>
<h1>TMotorCANControl ダッシュボード</h1>
<div id="freshness">データ待機中...</div>
<div id="safety-banner" hidden></div>
<div id="motors"></div>
<script>
const LABELS = {
  output_angle: ["位置", "rad"],
  output_velocity: ["速度", "rad/s"],
  output_acceleration: ["加速度", "rad/s\\u00b2"],
  current: ["電流", "A"],
  output_torque: ["トルク", "Nm"],
  motor_angle: ["モーター角度", "rad"],
  motor_velocity: ["モーター速度", "rad/s"],
  motor_acceleration: ["モーター加速度", "rad/s\\u00b2"],
  motor_torque: ["モータートルク", "Nm"],
  mosfet_temperature: ["温度", "\\u00b0C"],
};
const MAX_POINTS = 300; // 10Hz push を想定して直近30秒分
const motorState = {};
let builtCards = false;
let lastMessageAt = null;

function labelFor(varName) {
  return LABELS[varName] || [varName, ""];
}

function buildCards(motors) {
  const container = document.getElementById("motors");
  container.innerHTML = "";
  for (const name of Object.keys(motors)) {
    const vars = Object.keys(motors[name]);
    const plotVar = vars.length > 0 ? vars[0] : null;
    motorState[name] = { history: [], plotVar: plotVar };

    let rows = "";
    for (const v of vars) {
      const info = labelFor(v);
      rows += '<div class="row"><span class="row-label">' + info[0] +
        '</span><span id="val-' + name + '-' + v + '" class="row-value">--</span>' +
        '<span class="row-unit">' + info[1] + '</span></div>';
    }

    const card = document.createElement("section");
    card.className = "motor-card";
    card.id = "card-" + name;
    let html = "<h2>" + name + "</h2>";
    html += '<div class="stale-badge" id="stale-' + name + '" hidden>\\u26a0 データが更新されていません</div>';
    html += rows;
    if (plotVar) {
      html += '<canvas id="chart-' + name + '" width="320" height="110"></canvas>';
      html += '<div class="chart-caption">' + labelFor(plotVar)[0] + ' の推移</div>';
    }
    card.innerHTML = html;
    container.appendChild(card);
  }
  builtCards = true;
}

function updateFreshnessBanner(ageSeconds) {
  const el = document.getElementById("freshness");
  if (ageSeconds === null || ageSeconds === undefined) {
    el.textContent = "データ待機中...";
    el.className = "";
    return;
  }
  el.textContent = "最終更新: " + ageSeconds.toFixed(2) + "秒前";
  el.className = ageSeconds > 1.0 ? "stale" : "";
}

function updateSafetyBanner(safetyOk, safetyMessage) {
  const el = document.getElementById("safety-banner");
  if (safetyOk === null || safetyOk === undefined) {
    // SafetyMonitor が接続されていないダッシュボード（安全状態は不明であって「正常」ではない
    // ため、誤って安全と伝えないようバナー自体を出さない）
    el.hidden = true;
    return;
  }
  el.hidden = false;
  if (safetyOk) {
    el.textContent = "安全: 正常";
    el.className = "ok";
  } else {
    el.textContent = "\\u26a0 安全上限超過: " + (safetyMessage || "");
    el.className = "exceeded";
  }
}

function drawChart(name) {
  const canvas = document.getElementById("chart-" + name);
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const history = motorState[name].history;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (history.length < 2) return;

  const values = history.map(function (p) { return p[1]; });
  const minV = Math.min.apply(null, values);
  const maxV = Math.max.apply(null, values);
  const span = (maxV - minV) || 1;

  ctx.beginPath();
  ctx.strokeStyle = "#2b6cb0";
  ctx.lineWidth = 1.5;
  history.forEach(function (point, i) {
    const x = (i / (MAX_POINTS - 1)) * canvas.width;
    const y = canvas.height - ((point[1] - minV) / span) * canvas.height;
    if (i === 0) { ctx.moveTo(x, y); } else { ctx.lineTo(x, y); }
  });
  ctx.stroke();
}

function handleMessage(payload) {
  lastMessageAt = performance.now();
  if (!builtCards) buildCards(payload.motors);

  updateFreshnessBanner(payload.age_seconds);
  updateSafetyBanner(payload.safety_ok, payload.safety_message);
  const stale = payload.age_seconds !== null && payload.age_seconds !== undefined && payload.age_seconds > 1.0;

  for (const name of Object.keys(payload.motors)) {
    const vars = payload.motors[name];
    for (const v of Object.keys(vars)) {
      const el = document.getElementById("val-" + name + "-" + v);
      if (el) {
        const value = vars[v];
        el.textContent = typeof value === "number" ? value.toFixed(3) : String(value);
      }
    }
    const state = motorState[name];
    if (state && state.plotVar && vars[state.plotVar] !== undefined) {
      state.history.push([payload.t, vars[state.plotVar]]);
      if (state.history.length > MAX_POINTS) state.history.shift();
      drawChart(name);
    }
    const staleBadge = document.getElementById("stale-" + name);
    if (staleBadge) staleBadge.hidden = !stale;
    const card = document.getElementById("card-" + name);
    if (card) card.className = "motor-card" + (stale ? " stale" : "");
  }
}

const source = new EventSource("/events");
source.onmessage = function (event) {
  try {
    handleMessage(JSON.parse(event.data));
  } catch (e) {
    console.error("failed to parse SSE payload", e);
  }
};
source.onerror = function () {
  document.getElementById("freshness").textContent = "サーバーとの接続が切れました。再接続を試みています...";
  document.getElementById("freshness").className = "stale";
};

setInterval(function () {
  if (lastMessageAt === null) return;
  const secondsSinceMessage = (performance.now() - lastMessageAt) / 1000;
  if (secondsSinceMessage > 2.0) {
    document.getElementById("freshness").textContent =
      "サーバーからの応答がありません（" + secondsSinceMessage.toFixed(1) + "秒）。接続を確認してください。";
    document.getElementById("freshness").className = "stale";
  }
}, 500);
</script>
</body>
</html>
"""
