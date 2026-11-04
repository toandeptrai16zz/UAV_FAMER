#!/usr/bin/env python3
"""UAV live telemetry dashboard (offline, stdlib only).

Receives the unchanged 88-byte STM32 telemetry protocol over UDP 14550,
uses the existing project's exact decoder, records every good frame to CSV,
and serves browser charts on localhost:8765. No flight-control commands.
"""
from __future__ import annotations

import argparse
import csv
import json
import signal
import socket
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from uav_ground_station import decode_frame

HELLO = b'UAVT_HELLO'
CSV_FIELDS = [
    'received_at_utc', 'rx_elapsed_s', 'seq_gap_since_previous',
    'seq', 'timestamp_ms', 'flags',
    'link_ok', 'failsafe', 'armed', 'spinning', 'imu_fresh', 'kill_active',
    'alt_hold', 'imu_calibrated', 'range_valid', 'dma_error', 'output_active', 'config_ready',
    'rc_throttle', 'rc_roll', 'rc_pitch', 'rc_yaw', 'roll_deg', 'pitch_deg',
    'gyro_roll_dps', 'gyro_pitch_dps', 'gyro_yaw_dps',
    'accel_x_raw', 'accel_y_raw', 'accel_z_raw',
    'pid_roll', 'pid_pitch', 'pid_yaw', 'base_dshot', 'mixer_scale', 'mixer_offset',
    'mix_m1', 'mix_m2', 'mix_m3', 'mix_m4',
    'req_m1', 'req_m2', 'req_m3', 'req_m4',
    'out_m1', 'out_m2', 'out_m3', 'out_m4',
    'battery_v', 'current_a', 'range_cm',
    'imu_dt_max_ms', 'motor_dt_max_ms', 'dshot_errors', 'rx_age_ms',
]
PLOT_FIELDS = [
    'roll_deg', 'pitch_deg',
    'gyro_roll_dps', 'gyro_pitch_dps', 'gyro_yaw_dps',
    'accel_x_raw', 'accel_y_raw', 'accel_z_raw',
    'rc_throttle', 'rc_roll', 'rc_pitch', 'rc_yaw',
    'pid_roll', 'pid_pitch', 'pid_yaw',
    'out_m1', 'out_m2', 'out_m3', 'out_m4',
    'range_cm',
]


class Collector:
    def __init__(self, ip: str, port: int, csv_path: Path) -> None:
        self.ip = ip
        self.port = port
        self.csv_path = csv_path
        self.started = time.monotonic()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.samples = deque(maxlen=1400)  # sampled at <=10Hz: up to 140 seconds
        self.rx_times = deque()
        self.last_sample_t = -1e20
        self.last_rx_t = None
        self.last_seq = None
        self.last_data = None
        self.packets = 0
        self.invalid = 0
        self.seq_gap = 0
        self.out_of_order = 0
        self.hello_errors = 0
        self.last_error = ''
        self.last_hello = 0.0

    def run(self):
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', self.port))
            sock.settimeout(0.15)
            with self.csv_path.open('w', encoding='utf-8', newline='', buffering=1) as fp:
                writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS, extrasaction='ignore')
                writer.writeheader()
                flush_at = time.monotonic()
                while not self.stop_event.is_set():
                    now = time.monotonic()
                    if now - self.last_hello >= 0.75:
                        try:
                            sock.sendto(HELLO, (self.ip, self.port))
                        except OSError as exc:
                            with self.lock:
                                self.hello_errors += 1
                                self.last_error = str(exc)
                        self.last_hello = now
                    try:
                        raw, addr = sock.recvfrom(4096)
                    except socket.timeout:
                        raw = b''
                        addr = None
                    except OSError as exc:
                        with self.lock:
                            self.last_error = str(exc)
                        continue
                    if raw:
                        if raw == HELLO:
                            continue  # local loopback HELLO is not a telemetry error
                        # Avoid mixing in packets from other sources.
                        if addr and addr[0] != self.ip:
                            continue
                        row = decode_frame(raw)
                        now = time.monotonic()
                        if row is None:
                            with self.lock:
                                self.invalid += 1
                        else:
                            gap = 0
                            with self.lock:
                                seq = int(row['seq'])
                                if self.last_seq is not None:
                                    delta = (seq - self.last_seq) & 0xFFFF
                                    if 1 <= delta < 0x8000:
                                        gap = delta - 1
                                        self.seq_gap += gap
                                    else:
                                        self.out_of_order += 1
                                self.last_seq = seq
                                self.packets += 1
                                self.last_rx_t = now
                                self.last_data = row
                                self.rx_times.append(now)
                                while self.rx_times and self.rx_times[0] < now - 5.0:
                                    self.rx_times.popleft()
                                if now - self.last_sample_t >= 0.1:
                                    entry = {'t': round(now - self.started, 3)}
                                    entry.update({k: row.get(k) for k in PLOT_FIELDS})
                                    self.samples.append(entry)
                                    self.last_sample_t = now
                            writer.writerow({
                                'received_at_utc': datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
                                'rx_elapsed_s': round(now - self.started, 3),
                                'seq_gap_since_previous': gap,
                                **row,
                            })
                    if time.monotonic() - flush_at >= 1.0:
                        fp.flush()
                        flush_at = time.monotonic()
        except OSError as exc:
            with self.lock:
                self.last_error = str(exc)
            print(f'[ERROR] Unable to bind UDP port {self.port}: {exc}', file=sys.stderr)
        finally:
            sock.close()

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            age = None if self.last_rx_t is None else round(now - self.last_rx_t, 2)
            while self.rx_times and self.rx_times[0] < now - 5.0:
                self.rx_times.popleft()
            hz = round(len(self.rx_times)/min(5.0, max(now - self.started, 0.1)), 1)
            return {
                'mode': 'LIVE',
                'status': 'WAITING' if age is None else ('STALE' if age > 1.5 else 'LIVE'),
                'age_s': age,
                'ip': self.ip, 'port': self.port,
                'packets': self.packets, 'invalid': self.invalid,
                'seq_gap': self.seq_gap, 'out_of_order': self.out_of_order,
                'hello_errors': self.hello_errors, 'error': self.last_error,
                'receive_hz_5s': hz,
                'uptime_s': round(now - self.started, 1),
                'last': self.last_data,
                'history': list(self.samples),
                'csv_path': str(self.csv_path.resolve()),
                'notes': {
                    'dps310': 'NOT_IN_V1_FRAME',
                    'battery': 'NOT_MEASURED_UNLESS_FIELD_PRESENT',
                    'seq_gaps': 'Includes skipped STM32 TX slots as well as packets lost in transit',
                    'accel': 'Raw MPU6500 counts, not g',
                },
            }


HTML = r'''<!doctype html>
<html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>UAV Flight Telemetry | Live Ground Station</title>
<style>
:root{color-scheme:dark;--bg:#09111f;--panel:#142137;--line:#2b4160;--muted:#94aac4;--ink:#e8f3ff;--a:#68caff;--ok:#43d7a6;--warn:#f9b96e;--bad:#ff7582}
*{box-sizing:border-box}body{background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,Segoe UI,sans-serif;margin:0}header{background:#101d30;border-bottom:1px solid var(--line);padding:18px max(18px,calc((100% - 1480px)/2));display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:12px}
h1{font-size:20px;margin:0}h2{font-size:15px;margin:0 0 10px}small,.muted{color:var(--muted)}main{max-width:1500px;padding:20px;margin:0 auto}.pill{border:1px solid var(--line);border-radius:99px;padding:7px 13px;font-weight:700}.live{color:var(--ok)}.stale{color:var(--bad)}.waiting{color:var(--warn)}
.metrics{display:grid;grid-template-columns:repeat(6,minmax(130px,1fr));gap:12px;margin-bottom:16px}.metric,.card{background:var(--panel);border:1px solid var(--line);border-radius:12px}.metric{padding:15px}.metric label{display:block;font-size:10px;letter-spacing:.1em;text-transform:uppercase;color:var(--muted)}.metric b{font-size:24px;display:block;margin-top:5px;font-variant-numeric:tabular-nums}.metric span{font-size:11px;color:var(--muted)}
.row{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-bottom:14px}.card{padding:16px}.chart{height:236px;position:relative}canvas{width:100%;height:100%;display:block}.legend{font-size:11px;color:var(--muted);margin-top:8px}.legend i{display:inline-block;width:9px;height:9px;border-radius:50%;margin:0 5px 0 11px}.legend i:first-child{margin-left:0}.badge{color:var(--ok);font-weight:700}.info{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}.datum{background:#0e1b2c;border:1px solid #223953;padding:10px;border-radius:8px}.datum em{display:block;font-size:11px;font-style:normal;color:var(--muted)}.datum strong{font-variant-numeric:tabular-nums;font-size:16px}.toolbar{display:flex;gap:10px;align-items:center;flex-wrap:wrap}.toolbar select{background:#20334d;color:white;border:1px solid #526b89;border-radius:7px;padding:7px}.alert{background:#262018;border:1px solid #665238;border-radius:9px;color:#ffd5a2;padding:11px 13px;margin:14px 0;font-size:12px}footer{padding:0 6px 26px;font-size:12px;color:var(--muted)}
@media(max-width:1050px){.metrics{grid-template-columns:repeat(3,1fr)}.info{grid-template-columns:repeat(2,1fr)}}@media(max-width:680px){.metrics{grid-template-columns:repeat(2,1fr)}.row{grid-template-columns:1fr}main{padding:12px}.metric b{font-size:20px}.chart{height:210px}}
</style></head><body>
<header><div><h1>UAV / Ground Station</h1><small>STM32G474RE · ESP32-CAM Direct AP · UDP 14550 · Thu nhận và ghi log (chỉ đọc)</small></div><div class="toolbar"><label>Biểu đồ <select id="window"><option value="15">15 giây</option><option value="30" selected>30 giây</option><option value="60">60 giây</option><option value="120">120 giây</option></select></label><span id="connection" class="pill waiting">ĐANG CHỜ</span></div></header>
<main>
<div class="metrics">
<div class="metric"><label>Gói nhận</label><b id="packets">—</b><span>CRC hợp lệ</span></div>
<div class="metric"><label>Tốc độ 5 giây</label><b id="rate">—</b><span>Hz thực nhận</span></div>
<div class="metric"><label>Tuổi dữ liệu</label><b id="age">—</b><span>Giây từ gói cuối</span></div>
<div class="metric"><label>Nhảy sequence</label><b id="gaps">—</b><span>Gồm mất gói + STM32 bỏ lượt TX</span></div>
<div class="metric"><label>Gói sai CRC / format</label><b id="invalid">—</b><span>UDP đến laptop không hợp lệ</span></div>
<div class="metric"><label>Trạng thái FC</label><b id="flight">—</b><span id="flight-sub">ARM · KILL · FAILSAFE</span></div>
</div>
<div class="card" style="margin-bottom:14px"><h2>Cảm biến và trạng thái mới nhất <span class="muted" id="time">—</span></h2><div class="info" id="latest"></div></div>
<div class="row"><div class="card"><h2>01 · Góc nghiêng / MPU6500</h2><div class="chart"><canvas id="attitude"></canvas></div><div class="legend" id="attitude-legend"></div></div><div class="card"><h2>02 · Gyroscope (°/s)</h2><div class="chart"><canvas id="gyro"></canvas></div><div class="legend" id="gyro-legend"></div></div></div>
<div class="row"><div class="card"><h2>03 · Accelerometer (raw counts)</h2><div class="chart"><canvas id="accel"></canvas></div><div class="legend" id="accel-legend"></div></div><div class="card"><h2>04 · Radio / SBUS (raw)</h2><div class="chart"><canvas id="rc"></canvas></div><div class="legend" id="rc-legend"></div></div></div>
<div class="row"><div class="card"><h2>05 · PID correction (không phải Kp/Ki/Kd)</h2><div class="chart"><canvas id="pid"></canvas></div><div class="legend" id="pid-legend"></div></div><div class="card"><h2>06 · ESC DShot output M1–M4</h2><div class="chart"><canvas id="motor"></canvas></div><div class="legend" id="motor-legend"></div></div></div>
<div class="row"><div class="card"><h2>07 · MT-06 khoảng cách thẳng đứng (cm)</h2><div class="chart"><canvas id="range"></canvas></div><div class="legend" id="range-legend"></div></div><div class="card"><h2>08 · Ghi nhận chất lượng liên kết</h2><div class="card" style="border:0;background:#0e1b2c"><div class="info" id="link-detail" style="grid-template-columns:repeat(2,minmax(0,1fr))"></div><small>Sequence gap không chứng minh riêng mất gói Wi-Fi: STM32 cũng bỏ lượt gửi khi UART bận. Rate 5 giây là tốc độ gói laptop thực nhận, không phải độ trễ điều khiển.</small></div><h2 style="margin-top:14px">DPS310 & nguồn</h2><p class="muted">DPS310 đã có driver trên STM32 nhưng <strong>chưa truyền</strong> áp suất/nhiệt độ/độ cao trong frame v1. Điện áp và dòng pin hiện <strong>chưa đo</strong> (giá trị 0xFFFF) nếu không lắp thêm cảm biến. Dashboard không tự tạo số liệu.</p></div></div>
<div class="alert">Chỉ theo dõi và ghi log. Dashboard này không ARM, không thay PID, không điều khiển motor. Không dựa vào Wi-Fi/biểu đồ để điều khiển bay hoặc thay thế failsafe RC. Kiểm tra an toàn trên bàn trước bất kỳ thử nghiệm nào.</div>
<footer id="file-path">CSV đang ghi trên laptop…</footer>
</main>
<script>
const groups={
 attitude:[['roll_deg','Roll','#68caff'],['pitch_deg','Pitch','#fbba69']],
 gyro:[['gyro_roll_dps','Roll rate','#68caff'],['gyro_pitch_dps','Pitch rate','#f1b56c'],['gyro_yaw_dps','Yaw rate','#af92ff']],
 accel:[['accel_x_raw','Accel X','#68caff'],['accel_y_raw','Accel Y','#f1b56c'],['accel_z_raw','Accel Z','#8ee3a0']],
 rc:[['rc_throttle','Throttle','#e4e96c'],['rc_roll','RC Roll','#68caff'],['rc_pitch','RC Pitch','#ffb16c'],['rc_yaw','RC Yaw','#bf9aff']],
 pid:[['pid_roll','Roll corr.','#68caff'],['pid_pitch','Pitch corr.','#f1b56c'],['pid_yaw','Yaw corr.','#af92ff']],
 motor:[['out_m1','M1','#68caff'],['out_m2','M2','#f3b96d'],['out_m3','M3','#8ee3a0'],['out_m4','M4','#b29aff']],
 range:[['range_cm','MT-06 range','#8ee3a0']],
};
for(const [id,fields] of Object.entries(groups)){document.getElementById(id+'-legend').innerHTML=fields.map(([k,l,c])=>`<i style="background:${c}"></i>${l}`).join(' ')}
const val=(v,d=1)=>v===null||v===undefined?'—':typeof v==='number'?Number(v).toFixed(d):String(v);
const box=(label,value)=>`<div class="datum"><em>${label}</em><strong>${value}</strong></div>`;
function drawChart(id,all,win){
 let c=document.getElementById(id),ctx=c.getContext('2d'),bounds=c.getBoundingClientRect(),dpr=window.devicePixelRatio||1;
 if(bounds.width<=0||bounds.height<=0)return;
 c.width=Math.ceil(bounds.width*dpr);c.height=Math.ceil(bounds.height*dpr);ctx.scale(dpr,dpr);
 const w=bounds.width,h=bounds.height,L=53,R=12,T=12,B=26;
 ctx.clearRect(0,0,w,h);ctx.fillStyle='#102035';ctx.fillRect(0,0,w,h);
 let maxT=all.length?all[all.length-1].t:0,minT=Math.max(0,maxT-win),a=all.filter(x=>x.t>=minT);
 let values=[]; for(let item of a)for(let [key] of groups[id])if(Number.isFinite(item[key]))values.push(item[key]);
 let lo=values.length?Math.min(...values):0,hi=values.length?Math.max(...values):1;
 if(lo===hi){lo-=1;hi+=1;}const pad=(hi-lo)*.1;lo-=pad;hi+=pad;
 ctx.font='11px system-ui';ctx.textBaseline='middle';ctx.lineWidth=1;
 for(let j=0;j<5;j++){
  let y=T+(h-T-B)*j/4,v=hi-(hi-lo)*j/4;
  ctx.strokeStyle='#2c405c';ctx.beginPath();ctx.moveTo(L,y);ctx.lineTo(w-R,y);ctx.stroke();
  ctx.fillStyle='#90a6c4';ctx.textAlign='right';ctx.fillText(v.toFixed(Math.abs(v)<10?1:0),L-7,y);
 }
 for(let j=0;j<5;j++){let x=L+(w-L-R)*j/4,t=maxT-win+win*j/4;
  ctx.fillStyle='#90a6c4';ctx.textAlign='center';ctx.fillText(Math.max(0,t).toFixed(0)+'s',x,h-11);}
 if(a.length===0){ctx.textAlign='center';ctx.fillStyle='#7890ae';ctx.fillText('ĐANG CHỜ DỮ LIỆU',w/2,h/2);return;}
 const xx=t=>L+(w-L-R)*(t-(maxT-win))/win, yy=v=>T+(h-T-B)*(hi-v)/(hi-lo);
 for(let [key,label,color] of groups[id]){
  ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();let started=false;
  for(let item of a){let value=item[key];if(!Number.isFinite(value)){started=false;continue;}
    let x=xx(item.t),y=yy(value);if(!started){ctx.moveTo(x,y);started=true;}else ctx.lineTo(x,y);
  }
  ctx.stroke();
 }
}
let history=[],win=30;
document.getElementById('window').addEventListener('change',e=>win=Number(e.target.value));
async function update(){
 try{
  let r=await fetch('/api/snapshot',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);let s=await r.json();history=s.history||[];
  const status=document.getElementById('connection');status.className='pill '+(s.status==='LIVE'?'live':s.status==='STALE'?'stale':'waiting');status.textContent=s.status==='LIVE'?'● ĐANG NHẬN':s.status==='STALE'?'● MẤT TELEMETRY':'● CHỜ TELEMETRY';
  for(let [id,text] of Object.entries({packets:s.packets.toLocaleString('vi-VN'),rate:s.receive_hz_5s+' Hz',age:s.age_s===null?'—':s.age_s+' s',gaps:s.seq_gap.toLocaleString('vi-VN'),invalid:s.invalid.toLocaleString('vi-VN')}))document.getElementById(id).textContent=text;
  let d=s.last||{};document.getElementById('time').textContent=d.timestamp_ms===undefined?'':'· STM32 t = '+d.timestamp_ms+' ms';
  document.getElementById('flight').textContent=s.status!=='LIVE'?'—':d.failsafe?'FAILSAFE':d.armed?'ARMED':'DISARM';
  document.getElementById('flight-sub').textContent='KILL: '+(d.kill_active?'ON':'OFF')+' · Link: '+(d.link_ok?'OK':'NO')+' · IMU: '+(d.imu_fresh?'FRESH':'STALE');
  document.getElementById('latest').innerHTML=[
   box('Roll / Pitch',val(d.roll_deg,2)+'° / '+val(d.pitch_deg,2)+'°'),
   box('Gyro XYZ (°/s)',[d.gyro_roll_dps,d.gyro_pitch_dps,d.gyro_yaw_dps].map(v=>val(v)).join(' / ')),
   box('Accel XYZ (raw)',[d.accel_x_raw,d.accel_y_raw,d.accel_z_raw].map(v=>val(v,0)).join(' / ')),
   box('MT-06 range',d.range_cm===null||d.range_cm===undefined?'Không có mẫu':val(d.range_cm,0)+' cm'),
   box('Ga / base DShot',val(d.rc_throttle,0)+' / '+val(d.base_dshot,0)),
   box('4 motor DShot',[d.out_m1,d.out_m2,d.out_m3,d.out_m4].map(v=>val(v,0)).join(' · ')),
   box('PID correction',[d.pid_roll,d.pid_pitch,d.pid_yaw].map(v=>val(v,1)).join(' / ')),
   box('Nguồn pin / dòng',d.battery_v==null?'Chưa đo':val(d.battery_v,2)+' V'+(d.current_a==null?'':' / '+val(d.current_a,2)+' A'))
  ].join('');
  document.getElementById('link-detail').innerHTML=box('Sequence gaps (lũy kế)',s.seq_gap.toLocaleString())+box('Invalid packets',s.invalid)+box('Đảo thứ tự / trùng seq',s.out_of_order)+box('Lỗi HELLO',s.hello_errors)+box('Uptime',s.uptime_s+' s')+box('Hệ thống',s.status);
  document.getElementById('file-path').textContent='CSV ghi trên laptop: '+s.csv_path+(s.error?' · Lỗi: '+s.error:'');
  for(const id of Object.keys(groups))drawChart(id,history,win);
 }catch(e){document.getElementById('connection').textContent='LỖI WEB: '+e.message;document.getElementById('connection').className='pill stale';}
}
update();setInterval(update,800);window.addEventListener('resize',()=>{for(let id of Object.keys(groups))drawChart(id,history,win)});
</script></body></html>'''


def main():
    parser = argparse.ArgumentParser(description='Offline realtime UAV dashboard + CSV logger')
    parser.add_argument('--ip', default='192.168.4.1', help='ESP32-CAM Direct AP address')
    parser.add_argument('--port', type=int, default=14550, help='UDP telemetry port')
    parser.add_argument('--http-port', type=int, default=8765, help='Local dashboard web port')
    parser.add_argument('--http-bind', default='127.0.0.1', help='Only localhost by default')
    parser.add_argument('--csv', default=None, help='CSV file destination')
    args = parser.parse_args()
    path = Path(args.csv) if args.csv else Path('logs') / ('uav_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv')
    collector = Collector(args.ip, args.port, path)
    receiver = threading.Thread(target=collector.run, name='udp-collector', daemon=True)
    receiver.start()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            endpoint = urlsplit(self.path).path
            if endpoint == '/api/snapshot':
                body = json.dumps(collector.snapshot(), separators=(',', ':'), ensure_ascii=False).encode('utf-8')
                mime = 'application/json; charset=utf-8'
            elif endpoint in ('/', '/index.html'):
                body = HTML.encode('utf-8')
                mime = 'text/html; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (ConnectionResetError, BrokenPipeError):
                pass
        def log_message(self, *_args):
            pass

    try:
        server = ThreadingHTTPServer((args.http_bind, args.http_port), Handler)
    except OSError as exc:
        collector.stop_event.set()
        print(f'[ERROR] Cannot open dashboard port {args.http_port}: {exc}', file=sys.stderr)
        return 1
    print(f'[UAV] Wi-Fi Direct: connect laptop to UAV-DIRECT-XXXXXX (192.168.4.x)')
    print(f'[UAV] UDP listen 0.0.0.0:{args.port} -> send HELLO to {args.ip}:{args.port}')
    print(f'[UAV] Dashboard: http://127.0.0.1:{args.http_port}')
    print(f'[UAV] CSV: {path.resolve()}')
    print('[UAV] Close the old uav_ground_station.py before starting; Ctrl+C to stop')
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print('\n[UAV] Shutting down...')
    finally:
        server.server_close()
        collector.stop_event.set()
        receiver.join(timeout=2.0)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
