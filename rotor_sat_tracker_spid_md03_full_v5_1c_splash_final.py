# -*- coding: utf-8 -*-
# rotor_sat_tracker_spid_md03_full_v5_1c_splash_final.py
"""
SPID XY Rotator Controller by SP1JMF — v5.1c PRO FIX (pełny)
"""
import os, sys, math, time, threading, socket, struct, requests, configparser, webbrowser, traceback
from datetime import datetime, timedelta
import tkinter as tk
from tkinter import ttk, messagebox
import serial, serial.tools.list_ports
import pytz
from pyorbital.orbital import Orbital
from pyorbital import tlefile

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    import cartopy.crs as ccrs
    import cartopy.feature as cfeature
    CARTOPY_AVAILABLE=True
except Exception:
    CARTOPY_AVAILABLE=False

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE=True
except Exception:
    PIL_AVAILABLE=False

APP_TITLE="Spid XY Rotator Controller by SP1JMF"
APP_VERSION="2.1.3c"
BAUD=115200
POLL_SEC=0.5
PST_TCP_PORT=12000
UDP_FORWARD_PORT=12001
LOG_FILE="rotor_log.txt"
XY_LOG_FILE="xy_log.txt"
TLE_OFFLINE_FILE="amateur_sats_full.tle"
CFG_FILE="config.ini"
SPLASH_PNG="splash_spid.png"
AUTO_UPDATE_TLE_H=24

def clamp(v,lo,hi): return max(lo,min(hi,v))
def to_hex(b:bytes): return " ".join(f"{x:02X}" for x in b)
def az_deg_to_counts(az): return int(round((az%360.0)*65535.0/360.0)) & 0xFFFF
def el_deg_to_counts(el): return int(round(clamp(el,0.0,180.0)*65535.0/180.0)) & 0xFFFF
def counts_to_az_deg(c): return (int(c)&0xFFFF)*360.0/65536.0
def counts_to_el_deg(c): return (int(c)&0xFFFF)*180.0/65536.0

def azel_to_xy(az,el):
    azr=math.radians(az); elr=math.radians(el)
    return math.cos(elr)*math.sin(azr), math.cos(elr)*math.cos(azr)

def log_line(msg):
    ts=datetime.utcnow().strftime("[%Y-%m-%d %H:%M:%S] ")
    line=f"{ts}{msg}"
    print(line)
    try:
        with open(LOG_FILE,"a",encoding="utf-8") as f: f.write(line+"\n")
    except Exception: pass

def rot2prog_crc(*chunks: bytes) -> int:
    s=0
    for c in chunks:
        s=(s+sum(c)) & 0xFF
    return (-s) & 0xFF

def tle_mod10_checksum(line: str) -> int:
    s=0
    for ch in line[:68]:
        if ch.isdigit(): s += ord(ch)-48
        elif ch=='-': s += 1
    return s % 10

def tle_sanitize_and_fix(line: str) -> str:
    clean=''.join(ch for ch in line if 32 <= ord(ch) <= 126).rstrip()
    if len(clean)<68: clean = clean.ljust(68)
    cs = tle_mod10_checksum(clean)
    return clean[:68] + str(cs)

def tle_fix_pair(l1: str, l2: str):
    return tle_sanitize_and_fix(l1), tle_sanitize_and_fix(l2)

class ROT2ProgBinary:
    def build_set_position(self, az_deg: float, el_deg: float) -> bytes:
        azc=az_deg_to_counts(az_deg); elc=el_deg_to_counts(el_deg)
        header=b"\x57\x02"
        payload=struct.pack(">HHHHHH", azc, elc, 0, 0, 0, 0)
        crc=rot2prog_crc(header, payload)
        return header+payload+bytes([crc])
    def build_stop(self) -> bytes:
        header=b"\x57\x00"
        payload=b"\x00"*10
        crc=rot2prog_crc(header, payload)
        return header+payload+bytes([crc])
    def parse_status(self, frame: bytes):
        if len(frame)<12 or frame[0]!=0x57 or frame[1]!=0x03: return None
        azc,elc=struct.unpack(">HH", frame[2:6])
        return round(counts_to_az_deg(azc),1), round(counts_to_el_deg(elc),1)

class RotorLink:
    def __init__(self): self.mode="serial"; self.ser=None; self.sock=None; self.lock=threading.Lock()
    def connect_serial(self, port, baud=BAUD):
        self.disconnect()
        try:
            self.ser=serial.Serial(port, baud, timeout=0.1); self.mode="serial"
            log_line(f"[SER] Connected {port}@{baud}"); return True,""
        except Exception as e: return False,str(e)
    def connect_tcp(self, host, port=23):
        self.disconnect()
        try:
            s=socket.socket(socket.AF_INET, socket.SOCK_STREAM); s.settimeout(3.0); s.connect((host,int(port))); s.settimeout(0.1)
            self.sock=s; self.mode="tcp"; log_line(f"[TCP] Connected {host}:{port}"); return True,""
        except Exception as e: return False,str(e)
    def disconnect(self):
        with self.lock:
            try:
                if self.ser: self.ser.close()
            except: pass
            self.ser=None
            try:
                if self.sock:
                    try: self.sock.shutdown(socket.SHUT_RDWR)
                    except: pass
                    self.sock.close()
            except: pass
            self.sock=None
    def write(self, data: bytes):
        with self.lock:
            try:
                if self.mode=="serial" and self.ser: self.ser.write(data); return True
                if self.mode=="tcp" and self.sock: self.sock.sendall(data); return True
                return False
            except Exception as e: log_line(f"[LINK] write error: {e}"); return False
    def read(self, maxlen=128)->bytes:
        with self.lock:
            try:
                if self.mode=="serial" and self.ser: return self.ser.read(maxlen)
                if self.mode=="tcp" and self.sock:
                    try: return self.sock.recv(maxlen)
                    except socket.timeout: return b""
            except Exception as e: log_line(f"[LINK] read error: {e}")
        return b""

class PSTAsciiTCP:
    def __init__(self, app, host="0.0.0.0", port=PST_TCP_PORT):
        self.app=app; self.host=host; self.port=port
        self.sock=None; self.running=False; self.clients=set()
    def start(self):
        if self.running: return True
        try:
            self.sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host,self.port)); self.sock.listen(5); self.running=True
            threading.Thread(target=self._accept_loop, daemon=True).start()
            log_line(f"[PST TCP] listening {self.host}:{self.port}"); return True
        except Exception as e: log_line(f"[PST TCP] start error: {e}"); return False
    def stop(self):
        self.running=False
        try:
            if self.sock:
                try: self.sock.shutdown(socket.SHUT_RDWR)
                except: pass
                self.sock.close()
        except: pass
        self.sock=None
        for c in list(self.clients):
            try: c.close()
            except: pass
        self.clients.clear()
        log_line("[PST TCP] stopped.")
    def _accept_loop(self):
        while self.running:
            try:
                c,addr=self.sock.accept(); self.clients.add(c); log_line(f"[PST TCP] client {addr} connected")
                threading.Thread(target=self._client_loop, args=(c,addr), daemon=True).start()
            except Exception: time.sleep(0.05)
    def _client_loop(self, c, addr):
        buf=""; c.settimeout(0.5)
        try:
            while self.running:
                try: data=c.recv(4096)
                except socket.timeout: continue
                except Exception: break
                if not data: break
                try: text=data.decode("ascii", errors="ignore")
                except: continue
                buf+=text
                while "\r" in buf or "\n" in buf:
                    i = buf.find("\r") if "\r" in buf else buf.find("\n")
                    line, buf = buf[:i], buf[i+1:]
                    self._handle_line(line.strip(), c)
        finally:
            try: c.close()
            except: pass
            try: self.clients.remove(c)
            except: pass
            log_line(f"[PST TCP] client {addr} disconnected")
    def _handle_line(self, line, sock):
        if not line: return
        log_line(f"[PST TCP] << {line}")
        up=line.upper().replace(",", " ").replace(";", " ").strip()
        if up in ("C","CP","ST"):
            reply=f"+AZ={self.app.current_az.get():.1f} +EL={self.app.current_el.get():.1f}\r"
            try: sock.sendall(reply.encode("ascii"))
            except: pass
            log_line(f"[PST TCP] >> {reply.strip()}"); return
        if up in ("STOP","S"):
            self.app.send_stop()
            try: sock.sendall(b"OK\r")
            except: pass
            log_line("[PST TCP] STOP acknowledged"); return
        az=el=None
        try:
            parts=up.split()
            for p in parts:
                if p.startswith("AZ"): az=float(p[2:])
                if p.startswith("EL"): el=float(p[2:])
        except Exception: log_line(f"[PST TCP] parse error: {line}"); return
        if az is None and el is None:
            log_line(f"[PST TCP] unrecognized: {line}"); return
        if az is not None: az=round(az,1)
        if el is not None: el=round(el,1)
        self.app.set_rotor_position(az,el)
        reply=f"+AZ={self.app.current_az.get():.1f} +EL={self.app.current_el.get():.1f}\r"
        try: sock.sendall(reply.encode("ascii"))
        except: pass
        log_line(f"[PST TCP] >> {reply.strip()}")

class App:
    def __init__(self, root):
        self.root=root; root.title(APP_TITLE)
        self.config=configparser.ConfigParser(); self._load_config()
        root.geometry(self.config.get("ui","geometry", fallback="1280x900"))

        self.link=RotorLink(); self.rot2bin=ROT2ProgBinary()
        self.current_az=tk.DoubleVar(value=0.0); self.current_el=tk.DoubleVar(value=0.0)
        self.mode_var=tk.StringVar(value=self.config.get("conn","mode", fallback="serial"))
        self.ascii_fallback=tk.BooleanVar(value=self.config.getboolean("prefs","ascii_fallback", fallback=False))
        self.debug_hex=tk.BooleanVar(value=self.config.getboolean("prefs","debug_hex", fallback=False))
        self.forward_host=tk.StringVar(value=self.config.get("forward","host", fallback="127.0.0.1"))
        self.forward_port=tk.IntVar(value=self.config.getint("forward","port", fallback=UDP_FORWARD_PORT))
        self.xy_log_enabled=tk.BooleanVar(value=self.config.getboolean("prefs","xy_log", fallback=True))
        self.observer_lat=tk.DoubleVar(value=self.config.getfloat("observer","lat", fallback=52.2297))
        self.observer_lon=tk.DoubleVar(value=self.config.getfloat("observer","lon", fallback=21.0122))

        self.sat_name=tk.StringVar(value=self.config.get("tle","name", fallback="ISS (ZARYA)"))
        self.tle1=tk.StringVar(value=self.config.get("tle","l1", fallback=""))
        self.tle2=tk.StringVar(value=self.config.get("tle","l2", fallback=""))
        self.tle_date_str=tk.StringVar(value=self.config.get("tle","date", fallback="n/d"))

        self.program_version_url=self.config.get("update","program_version_url", fallback="https://example.com/version.txt")
        self.program_update_page=self.config.get("update","program_update_page", fallback="https://www.qrz.com/db/SP1JMF")
        self.tle_sources=["https://www.amsat.org/amsat/ftp/keps/current/nasa.all","https://celestrak.org/NORAD/elements/amateur.txt"]

        self.plot_mode=tk.StringVar(value=self.config.get("ui","plot_mode", fallback="orbital"))
        self.show_time=tk.BooleanVar(value=self.config.getboolean("ui","show_time", fallback=True))
        self._orbital_anim_running=False

        self._build_gui()
        self._refresh_ports()

        self._stop_poll=False; self.poll_thread=None; self._start_poll()
        self.pst_tcp=PSTAsciiTCP(self); self.pst_tcp.start()

        self._last_tle_update=None
        threading.Thread(target=self._tle_auto_update_loop, daemon=True).start()
        threading.Thread(target=self._program_update_check_once, daemon=True).start()

        self._all_sat_names=self._load_tle_names()
        self._set_sat_list(self._all_sat_names)
        self._log(f"[INFO] Załadowano {len(self._all_sat_names)} satelitów z {TLE_OFFLINE_FILE}")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_gui(self):
        menubar=tk.Menu(self.root)
        helpm=tk.Menu(menubar, tearoff=0)
        helpm.add_command(label="O programie", command=self.show_about)
        helpm.add_command(label="Strona AMSAT", command=lambda: webbrowser.open("https://www.amsat.org"))
        helpm.add_command(label="Strona SP1JMF (QRZ)", command=lambda: webbrowser.open("https://www.qrz.com/db/SP1JMF"))
        menubar.add_cascade(label="Pomoc", menu=helpm)
        self.root.config(menu=menubar)

        top=ttk.Frame(self.root); top.pack(fill="x", padx=6, pady=6)
        connf=ttk.LabelFrame(top, text="SPID MD-03 (ROT2PROG) – Połączenie"); connf.pack(side="left", fill="y", padx=6, pady=4)
        ttk.Radiobutton(connf, text="USB (Serial)", variable=self.mode_var, value="serial").grid(row=0,column=0,sticky="w")
        ttk.Radiobutton(connf, text="TCP (port 23)", variable=self.mode_var, value="tcp").grid(row=0,column=1,sticky="w")
        self.port_combo=ttk.Combobox(connf, width=18, state="readonly"); self.port_combo.grid(row=1,column=0,padx=4,pady=2)
        ttk.Button(connf, text="Odśwież porty", command=self._refresh_ports).grid(row=1,column=1,padx=4)
        ttk.Label(connf, text="TCP host:").grid(row=2,column=0,sticky="e"); self.tcp_host=ttk.Entry(connf, width=16); self.tcp_host.insert(0, self.config.get("conn","tcp_host", fallback="192.168.1.150")); self.tcp_host.grid(row=2,column=1,sticky="w")
        ttk.Label(connf, text="TCP port:").grid(row=2,column=2,sticky="e"); self.tcp_port=ttk.Entry(connf, width=8); self.tcp_port.insert(0, str(self.config.getint("conn","tcp_port", fallback=23))); self.tcp_port.grid(row=2,column=3,sticky="w")
        ttk.Checkbutton(connf, text="ASCII fallback", variable=self.ascii_fallback).grid(row=0,column=2,sticky="w")
        ttk.Checkbutton(connf, text="Pokaż HEX RX/TX", variable=self.debug_hex).grid(row=0,column=3,sticky="w")
        ttk.Button(connf, text="Połącz", command=self.connect).grid(row=3,column=0,pady=4)
        ttk.Button(connf, text="Rozłącz", command=self.disconnect).grid(row=3,column=1,pady=4)

        ctrl=ttk.LabelFrame(top, text="Sterowanie rotorem"); ctrl.pack(side="left", fill="y", padx=6, pady=4)
        ttk.Label(ctrl, text="AZ [°]:").grid(row=0,column=0,sticky="e"); self.az_entry=ttk.Entry(ctrl, width=8); self.az_entry.grid(row=0,column=1)
        ttk.Label(ctrl, text="EL [°]:").grid(row=1,column=0,sticky="e"); self.el_entry=ttk.Entry(ctrl, width=8); self.el_entry.grid(row=1,column=1)
        ttk.Button(ctrl, text="Ustaw", command=self.gui_set).grid(row=2,column=0,columnspan=2,pady=3, sticky="ew")

        sf=ttk.Frame(ctrl); sf.grid(row=3, column=0, columnspan=2, pady=6, sticky="ew")
        self.stop_btn = tk.Button(sf, text="🛑 STOP ROTOR", command=self.send_stop, bg="#cc0000", fg="white", font=("Segoe UI", 12, "bold"))
        self.stop_btn.pack(fill="x")

        ttk.Label(ctrl, text="Aktualny AZ:").grid(row=4,column=0,sticky="e"); ttk.Label(ctrl, textvariable=self.current_az, width=8).grid(row=4,column=1,sticky="w")
        ttk.Label(ctrl, text="Aktualny EL:").grid(row=5,column=0,sticky="e"); ttk.Label(ctrl, textvariable=self.current_el, width=8).grid(row=5,column=1,sticky="w")

        orbit=ttk.LabelFrame(top, text="TLE / Śledzenie satelity"); orbit.pack(side="left", fill="y", padx=6, pady=4)
        ttk.Label(orbit, text="Szer (°):").grid(row=0,column=0,sticky="e"); self.lat_entry=ttk.Entry(orbit,width=10); self.lat_entry.insert(0,str(self.observer_lat.get())); self.lat_entry.grid(row=0,column=1)
        ttk.Label(orbit, text="Dł (°):").grid(row=1,column=0,sticky="e"); self.lon_entry=ttk.Entry(orbit,width=10); self.lon_entry.insert(0,str(self.observer_lon.get())); self.lon_entry.grid(row=1,column=1)

        ttk.Label(orbit, text="Satelita:").grid(row=2,column=0,sticky="e")
        self.sat_combo = ttk.Combobox(orbit, width=26)
        self.sat_combo.grid(row=2,column=1,columnspan=2, sticky="w")
        self.sat_combo.bind("<KeyRelease>", self._sat_autocomplete)
        self.sat_combo.bind("<<ComboboxSelected>>", self._on_sat_selected)

        ttk.Label(orbit, text="TLE L1:").grid(row=3,column=0,sticky="e"); self.tle1_entry=ttk.Entry(orbit,width=36,textvariable=self.tle1); self.tle1_entry.grid(row=3,column=1,columnspan=2)
        ttk.Label(orbit, text="TLE L2:").grid(row=4,column=0,sticky="e"); self.tle2_entry=ttk.Entry(orbit,width=36,textvariable=self.tle2); self.tle2_entry.grid(row=4,column=1,columnspan=2)
        ttk.Button(orbit, text="Aktualizuj TLE (online)", command=self.gui_update_tle).grid(row=3,column=3,padx=4)

        ttk.Button(orbit, text="Start śledzenia", command=self.start_tracking).grid(row=5,column=0,pady=4)
        ttk.Button(orbit, text="Stop", command=self.stop_tracking).grid(row=5,column=1,pady=4)
        ttk.Button(orbit, text="Sprawdź aktualizacje programu", command=self.check_program_update_now).grid(row=6,column=0,columnspan=2,pady=4)

        right=ttk.Frame(self.root); right.pack(fill="both", expand=True, padx=6, pady=6)
        plotf=ttk.LabelFrame(right, text="Wizualizacja"); plotf.pack(fill="both", expand=True, padx=4, pady=4)
        mode_row=ttk.Frame(plotf); mode_row.pack(fill="x", padx=4, pady=2)
        ttk.Label(mode_row, text="Tryb:").pack(side="left")
        ttk.Radiobutton(mode_row, text="Mapa orbity", variable=self.plot_mode, value="orbital", command=self.redraw_plot).pack(side="left", padx=6)
        ttk.Radiobutton(mode_row, text="Az/El vs czas", variable=self.plot_mode, value="azel", command=self.redraw_plot).pack(side="left", padx=6)

        self.fig=Figure(figsize=(8.6,5.8), dpi=100)
        self.ax=self.fig.add_subplot(111)
        self.canvas=FigureCanvasTkAgg(self.fig, master=plotf)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        fwd=ttk.LabelFrame(right, text="Forwarding UDP (wyjście)"); fwd.pack(fill="x", padx=4, pady=4)
        ttk.Label(fwd, text="Host:").grid(row=0,column=0,sticky="e"); self.fwd_host_entry=ttk.Entry(fwd, textvariable=self.forward_host, width=18); self.fwd_host_entry.grid(row=0,column=1,sticky="w")
        ttk.Label(fwd, text="Port:").grid(row=0,column=2,sticky="e"); self.fwd_port_entry=ttk.Entry(fwd, textvariable=self.forward_port, width=8); self.fwd_port_entry.grid(row=0,column=3,sticky="w")

        self.log=tk.Text(self.root, height=8, state="disabled"); self.log.pack(fill="x", padx=6, pady=6)

        status_frame=ttk.Frame(self.root); status_frame.pack(fill="x", padx=6, pady=(0,6))
        self.status_left=ttk.Label(status_frame, text=f"Ver: {APP_VERSION} | TLE: {self.tle_date_str.get()} | AZ: 0.0° | EL: 0.0°")
        self.status_left.pack(side="left")
        self.status_color_lbl=tk.Label(status_frame, text="  Status: OK  ", fg="white", bg="#2e7d32"); self.status_color_lbl.pack(side="right")

        self._schedule_status_update()

    def _refresh_ports(self):
        ports=[p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"]=ports
        if ports:
            try: self.port_combo.current(0)
            except: pass

    def connect(self):
        if self.mode_var.get()=="serial":
            port=self.port_combo.get()
            if not port: messagebox.showwarning("Port","Wybierz port COM"); return
            ok,err=self.link.connect_serial(port, BAUD)
        else:
            host=self.tcp_host.get().strip() or "192.168.1.150"
            try: port=int(self.tcp_port.get() or "23")
            except: port=23
            ok,err=self.link.connect_tcp(host, port)
        if not ok:
            messagebox.showerror("Połączenie", err); self._log(err); self._set_status("offline"); return
        self._log("Połączono z rotorem."); self._set_status("ok")

    def disconnect(self):
        self.link.disconnect(); self._log("Rozłączono."); self._set_status("offline")

    def _start_poll(self):
        if not (hasattr(self,"poll_thread") and self.poll_thread and self.poll_thread.is_alive()):
            self._stop_poll=False
            self.poll_thread=threading.Thread(target=self._poll_loop, daemon=True); self.poll_thread.start()

    def _poll_loop(self):
        buf=b""
        while not self._stop_poll:
            try:
                chunk=self.link.read(128)
                if chunk:
                    buf+=chunk
                    if self.debug_hex.get(): self._log(f"[RX] {to_hex(chunk)}")
                    while len(buf)>=12:
                        idx=buf.find(b"\x57\x03")
                        if idx<0: buf=buf[-2:]; break
                        if idx>0: buf=buf[idx:]
                        frame=buf[:12]
                        parsed=self.rot2bin.parse_status(frame)
                        if parsed:
                            az,el=parsed
                            self.root.after(0, lambda a=az: self.current_az.set(round(a,1)))
                            self.root.after(0, lambda e=el: self.current_el.set(round(e,1)))
                            self._log(f"[STAT] AZ={az:.1f} EL={el:.1f}")
                            self._forward_xy(az, el, tag="ROTOR")
                            buf=buf[12:]
                        else:
                            buf=buf[1:]
                time.sleep(POLL_SEC)
            except Exception as e:
                self._log(f"Poll error: {e}"); time.sleep(POLL_SEC)

    def set_rotor_position(self, az=None, el=None):
        if az is None: az=self.current_az.get()
        if el is None: el=self.current_el.get()
        az=round(az%360.0,1); el=round(clamp(el,0,180),1)
        if self.ascii_fallback.get():
            cmd=f"AZ{az:06.1f} EL{el:06.1f}\r".encode("ascii")
            ok=self.link.write(cmd)
            self._log(f"[TX ASCII] AZ={az:.1f} EL={el:.1f}"); return ok
        frame=self.rot2bin.build_set_position(az, el); ok=self.link.write(frame)
        self._log(f"[TX BIN] {to_hex(frame)}"); return ok

    def gui_set(self):
        try: az=float(self.az_entry.get()); el=float(self.el_entry.get())
        except ValueError: messagebox.showerror("Dane","Podaj liczby az/el"); return
        self.set_rotor_position(az, el)

    def send_stop(self):
        frame=self.rot2bin.build_stop(); ok=self.link.write(frame)
        self._log("[STOP] Komenda zatrzymania"); self._log(f"[STOP] -> TX {to_hex(frame)}"); return ok

    def _schedule_status_update(self):
        def _tick():
            try:
                self.status_left.config(text=f"Ver: {APP_VERSION} | TLE: {self.tle_date_str.get() or 'n/d'} | AZ: {self.current_az.get():.1f}° | EL: {self.current_el.get():.1f}°")
            finally:
                self.root.after(500, _tick)
        _tick()

    def _set_status(self, kind):
        if kind=="ok": self.status_color_lbl.config(text="  Status: OK  ", bg="#2e7d32", fg="white")
        elif kind=="update": self.status_color_lbl.config(text="  Status: UPDATE  ", bg="#f9a825", fg="black")
        elif kind=="offline": self.status_color_lbl.config(text="  Status: OFFLINE  ", bg="#616161", fg="white")
        else: self.status_color_lbl.config(text="  Status: ERROR  ", bg="#b71c1c", fg="white")

    def _get_tle_text(self, force_online=False):
        if not force_online and os.path.exists(TLE_OFFLINE_FILE):
            try:
                with open(TLE_OFFLINE_FILE,"r",encoding="utf-8") as f:
                    txt=f.read()
                    if len(txt.strip())>50: return txt, "offline"
            except: pass
        if force_online:
            for url in self.tle_sources:
                try:
                    self._log(f"Pobieram TLE: {url}")
                    r=requests.get(url, timeout=12); r.raise_for_status()
                    if len(r.text.strip())>100: return r.text, url
                except Exception as e: self._log(f"TLE źródło błąd: {e}")
        return f"""{self.sat_name.get()}
{self.tle1.get()}
{self.tle2.get()}""", "fields"

    def gui_update_tle(self):
        threading.Thread(target=self._update_tle_bg, daemon=True).start()

    def _update_tle_bg(self):
        txt,src=self._get_tle_text(force_online=True)
        self._refresh_sat_list_from_text(txt)
        name=self.sat_name.get().strip().upper(); lines=txt.splitlines(); found=False
        for i in range(len(lines)-2):
            if name and name in lines[i].upper():
                l1=lines[i+1].strip(); l2=lines[i+2].strip()
                if l1.startswith("1 ") and l2.startswith("2 "):
                    self.tle1.set(l1); self.tle2.set(l2); self.tle_date_str.set(datetime.utcnow().strftime("%Y-%m-%d"))
                    self._save_config(); self._log(f"Zaktualizowano TLE dla {name} (źródło: {src})."); found=True; break
        if not found: self._log("Nie znaleziono dopasowania w pobranych TLE.")
        self._set_status("ok"); self.redraw_plot()

    def _tle_auto_update_loop(self):
        while True:
            try:
                now=datetime.utcnow()
                if (self._last_tle_update is None) or (now - self._last_tle_update >= timedelta(hours=AUTO_UPDATE_TLE_H)):
                    self._get_tle_text(force_online=True)
                    self._last_tle_update=now; self._set_status("update")
                time.sleep(300)
            except Exception as e: self._log(f"TLE auto error: {e}"); self._set_status("error"); time.sleep(600)

    def _program_update_check_once(self): self.check_program_update_now(silent=True)
    def check_program_update_now(self, silent=False):
        try:
            url=self.program_version_url
            if not url or url.startswith("https://example.com"):
                if not silent: self._log("URL wersji programu nie ustawiony w config.ini [update].")
                return
            r=requests.get(url, timeout=8); r.raise_for_status()
            latest=r.text.strip().splitlines()[0].strip()
            if latest and latest!=APP_VERSION:
                self._set_status("update")
                msg=f"Dostępna nowa wersja: v{latest} (Twoja: v{APP_VERSION}). Otworzyć stronę?"
                if messagebox.askyesno("Aktualizacja programu", msg):
                    webbrowser.open(self.program_update_page or url)
            else:
                if not silent: messagebox.showinfo("Aktualizacja programu", "Masz najnowszą wersję.")
                self._set_status("ok")
        except Exception as e:
            if not silent: messagebox.showwarning("Aktualizacja", f"Błąd sprawdzania: {e}")
            self._set_status("offline")

    def start_tracking(self):
        name=self.sat_name.get().strip()
        if not name:
            messagebox.showwarning("Brak satelity", "Najpierw wybierz satelitę z listy!"); return
        l1=self.tle1.get().strip(); l2=self.tle2.get().strip()
        try:
            if not (l1 and l2):
                if not os.path.exists(TLE_OFFLINE_FILE):
                    raise FileNotFoundError(f"Brak pliku {TLE_OFFLINE_FILE}")
                tle=tlefile.read(TLE_OFFLINE_FILE, name)
                l1, l2 = tle.line1.strip(), tle.line2.strip()
            l1, l2 = tle_fix_pair(l1, l2)  # napraw TLE
            self.orb=Orbital(name, line1=l1, line2=l2)
        except Exception as e:
            self._log(f"[ERROR] Błąd orbital dla {name}: {e}")
            self._log(traceback.format_exc())
            messagebox.showerror("Błąd orbital", f"Nie udało się zainicjować śledzenia dla:\n{name}\n\nSzczegóły: {e}")
            return
        self._log(f"Start śledzenia: {name}")
        self._start_orbital_anim()
        self.redraw_plot()

    def stop_tracking(self):
        self._log("Stop śledzenia."); self.orb=None; self._stop_orbital_anim(); self.redraw_plot()

    def _start_orbital_anim(self):
        if self._orbital_anim_running: return
        self._orbital_anim_running=True
        def tick():
            if not self._orbital_anim_running: return
            try: self.redraw_plot()
            finally: self.root.after(5000, tick)
        tick()

    def _stop_orbital_anim(self):
        self._orbital_anim_running=False

    def compute_groundtrack(self, minutes=90, step_seconds=30):
        if not hasattr(self,"orb") or self.orb is None: return [],[],[],[],[]
        now=datetime.utcnow().replace(tzinfo=pytz.UTC)
        times=[now+timedelta(seconds=s) for s in range(-45*60, 45*60+1, step_seconds)]
        lons,lats,azs,els=[],[],[],[]
        for t in times:
            try:
                lon,lat,_=self.orb.get_lonlatalt(t)
                if lon is not None and lon>180: lon-=360
                lons.append(lon); lats.append(lat)
                az,el,_=self.orb.get_observer_look(self.observer_lon.get(), self.observer_lat.get(), 0, t)
                azs.append(az); els.append(el)
            except:
                lons.append(float('nan')); lats.append(float('nan')); azs.append(float('nan')); els.append(float('nan'))
        return times,lons,lats,azs,els

    def _draw_orbital_view(self):
        ax=self.fig.add_subplot(111, projection=ccrs.PlateCarree()) if CARTOPY_AVAILABLE else self.fig.add_subplot(111)
        if CARTOPY_AVAILABLE:
            ax.set_global(); ax.coastlines('110m'); ax.add_feature(cfeature.BORDERS, linewidth=0.3)
            gl=ax.gridlines(draw_labels=True); gl.top_labels=False; gl.right_labels=False
        else:
            ax.set_xlim(-180,180); ax.set_ylim(-90,90); ax.grid(True); ax.set_xlabel("Długość [°]"); ax.set_ylabel("Szerokość [°]")
        if hasattr(self,"orb") and self.orb is not None:
            times,lons,lats,azs,els=self.compute_groundtrack(90,30)
            if CARTOPY_AVAILABLE: ax.plot(lons,lats, linestyle='-', marker=None, transform=ccrs.PlateCarree())
            else: ax.plot(lons,lats, linestyle='-')
            try:
                lon,lat,_=self.orb.get_lonlatalt(datetime.utcnow().replace(tzinfo=pytz.UTC))
                if lon>180: lon-=360
                if CARTOPY_AVAILABLE: ax.plot(lon,lat, marker='o', markersize=6, transform=ccrs.PlateCarree())
                else: ax.plot(lon,lat, marker='o', markersize=6)
            except: pass
        if CARTOPY_AVAILABLE: ax.plot(self.observer_lon.get(), self.observer_lat.get(), marker='s', markersize=6, transform=ccrs.PlateCarree())
        else: ax.plot(self.observer_lon.get(), self.observer_lat.get(), marker='s', markersize=6)
        ax.set_title("Mapa orbity (v5.1c)")

    def _draw_azel(self):
        ax=self.fig.add_subplot(111); ax.grid(True); ax.set_xlabel("Czas [min od teraz]"); ax.set_ylabel("Kąt [°]")
        if hasattr(self,"orb") and self.orb is not None:
            now=datetime.utcnow().replace(tzinfo=pytz.UTC); times=[now+timedelta(minutes=i) for i in range(0,61,1)]
            xs=[(t-now).total_seconds()/60.0 for t in times]
            azs,els=[],[]
            for t in times:
                try: az,el,_=self.orb.get_observer_look(self.observer_lon.get(), self.observer_lat.get(), 0, t)
                except: az,el=float('nan'),float('nan')
                azs.append(az); els.append(el)
            ax.plot(xs, azs, label="Azymut [°]"); ax.plot(xs, els, label="Elewacja [°]"); ax.legend()
        ax.set_title("Azymut / Elewacja vs czas")

    def redraw_plot(self):
        self.fig.clf()
        if self.plot_mode.get()=="azel": self._draw_azel()
        else: self._draw_orbital_view()
        try: self.canvas.draw()
        except: pass

    def _forward_xy(self, az, el, tag="FWD"):
        try:
            x,y=azel_to_xy(az,el); host=self.forward_host.get().strip(); port=int(self.forward_port.get())
            if host and port>0:
                msg=f"X={x:.6f} Y={y:.6f} AZ={az:.1f} EL={el:.1f}"
                s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.sendto(msg.encode("utf-8"), (host,port)); s.close()
                self._log(f"[{tag}] UDP {host}:{port} :: {msg}")
            if self.xy_log_enabled.get():
                with open(XY_LOG_FILE,"a",encoding="utf-8") as f:
                    f.write(f"{datetime.utcnow().isoformat()}Z,{az:.1f},{el:.1f},{x:.6f},{y:.6f}\n")
        except Exception as e: self._log(f"forward_xy error: {e}")

    def _log(self, msg):
        log_line(msg)
        try:
            self.log.config(state="normal"); self.log.insert("end", msg+"\n"); self.log.see("end"); self.log.config(state="disabled")
        except: pass

    def _load_tle_names(self):
        if not os.path.exists(TLE_OFFLINE_FILE): return ["(wszystkie)"]
        try:
            lines=open(TLE_OFFLINE_FILE,"r",encoding="utf-8").read().splitlines()
            names=[lines[i].strip() for i in range(0,len(lines),3) if lines[i].strip()]
            return ["(wszystkie)"]+names
        except: return ["(wszystkie)"]

    def _set_sat_list(self, names):
        self.sat_combo["values"]=names
        if names:
            saved=self.sat_name.get().strip()
            try: idx=names.index(saved) if saved in names else 0
            except: idx=0
            self.sat_combo.current(idx)

    def _sat_autocomplete(self, event=None):
        text=self.sat_combo.get().strip().upper()
        if not hasattr(self,"_all_sat_names") or not self._all_sat_names: return
        if not text: self._set_sat_list(self._all_sat_names); return
        filt=[n for n in self._all_sat_names if text in n.upper()]
        self._set_sat_list(filt if filt else self._all_sat_names)

    def _on_sat_selected(self, event=None):
        name=self.sat_combo.get().strip()
        self.sat_name.set(name)
        if name=="(wszystkie)": return
        if not os.path.exists(TLE_OFFLINE_FILE): return
        try:
            lines=open(TLE_OFFLINE_FILE,"r",encoding="utf-8").read().splitlines()
            for i in range(len(lines)-2):
                if name.upper()==lines[i].strip().upper():
                    l1=lines[i+1].strip(); l2=lines[i+2].strip()
                    self.tle1.set(l1); self.tle2.set(l2)
                    self._log(f"[TLE] Załadowano {name}")
                    self.tle_date_str.set(datetime.utcnow().strftime("%Y-%m-%d"))
                    self._set_status("ok"); self.redraw_plot()
                    break
        except Exception as e:
            self._log(f"[TLE] Błąd odczytu: {e}")

    def _refresh_sat_list_from_text(self, tle_text):
        try:
            lines=tle_text.splitlines(); names=[]
            for i in range(0,len(lines)-2,3):
                nm=lines[i].strip()
                l1=lines[i+1].strip() if i+1<len(lines) else ""
                l2=lines[i+2].strip() if i+2<len(lines) else ""
                if nm and l1.startswith("1 ") and l2.startswith("2 "): names.append(nm)
            if names:
                self._all_sat_names=["(wszystkie)"]+names
                self._set_sat_list(self._all_sat_names)
                self._log(f"[INFO] Załadowano {len(names)} satelitów (online)")
        except Exception as e:
            self._log(f"[TLE] refresh list error: {e}")

    def _load_config(self):
        try:
            if os.path.exists(CFG_FILE): self.config.read(CFG_FILE, encoding="utf-8")
        except Exception as e: print("Config read error:", e, file=sys.stderr)

    def _save_config(self):
        try:
            for sec in ("ui","conn","prefs","forward","observer","tle","update"):
                if not self.config.has_section(sec): self.config.add_section(sec)
            self.config.set("ui","geometry", self.root.winfo_geometry()); self.config.set("ui","plot_mode", self.plot_mode.get()); self.config.set("ui","show_time", "1" if self.show_time.get() else "0")
            self.config.set("conn","mode", self.mode_var.get()); self.config.set("conn","tcp_host", getattr(self,"tcp_host","").get() if hasattr(self,"tcp_host") else ""); self.config.set("conn","tcp_port", getattr(self,"tcp_port","").get() if hasattr(self,"tcp_port") else "23")
            self.config.set("prefs","ascii_fallback", "1" if self.ascii_fallback.get() else "0"); self.config.set("prefs","debug_hex", "1" if self.debug_hex.get() else "0"); self.config.set("prefs","xy_log","1" if self.xy_log_enabled.get() else "0")
            self.config.set("forward","host", self.forward_host.get()); self.config.set("forward","port", str(self.forward_port.get()))
            self.config.set("observer","lat", str(self.observer_lat.get())); self.config.set("observer","lon", str(self.observer_lon.get()))
            self.config.set("tle","name", self.sat_name.get()); self.config.set("tle","l1", self.tle1.get()); self.config.set("tle","l2", self.tle2.get()); self.config.set("tle","date", self.tle_date_str.get())
            if not self.config.has_section("update"): self.config.add_section("update")
            self.config.set("update","program_version_url", self.config.get("update","program_version_url", fallback="https://example.com/version.txt"))
            self.config.set("update","program_update_page", self.config.get("update","program_update_page", fallback="https://www.qrz.com/db/SP1JMF"))
            with open(CFG_FILE,"w",encoding="utf-8") as f: self.config.write(f)
        except Exception as e: print("Config save error:", e, file=sys.stderr)

    def show_about(self):
        win=tk.Toplevel(self.root); win.title("About – SPID XY Rotator Controller"); win.resizable(False, False)
        ttk.Label(win, text="SPID XY ROTATOR CONTROLLER BY SP1JMF", font=("Segoe UI", 11, "bold")).grid(row=0, column=0, columnspan=3, pady=(10,6), padx=10)
        ttk.Label(win, text=f"Version {APP_VERSION} (2025)").grid(row=1, column=0, columnspan=3, pady=(0,10), padx=10)
        text=("Control software for SPID MD-03 rotator\n"
              "Protocol: ROT2PROG (binary + ASCII fallback)\n"
              "Includes PSTrotator bridge, UDP forwarding,\n"
              "and AMSAT satellite tracking with Orbital View.")
        ttk.Label(win, text=text, justify="left").grid(row=2, column=0, columnspan=3, padx=10)
        ttk.Button(win, text="🌐 Otwórz stronę AMSAT", command=lambda: webbrowser.open("https://www.amsat.org")).grid(row=3, column=0, pady=10, padx=10, sticky="ew")
        ttk.Button(win, text="🛰️ Otwórz stronę SP1JMF", command=lambda: webbrowser.open("https://www.qrz.com/db/SP1JMF")).grid(row=3, column=1, pady=10, padx=10, sticky="ew")
        ttk.Button(win, text="OK", command=win.destroy).grid(row=3, column=2, pady=10, padx=10, sticky="e")

    def on_close(self):
        try: self.pst_tcp.stop()
        except: pass
        self._stop_poll=True; self._stop_orbital_anim()
        try: self.link.disconnect()
        except: pass
        self._save_config()
        try: self.root.destroy()
        except: pass

class SplashScreen(tk.Toplevel):
    def __init__(self, parent, image_path=SPLASH_PNG, duration=3000):
        super().__init__(parent)
        self.overrideredirect(True)
        self.duration=duration; self.start_time=time.time()
        self.w=1920; self.h=1080
        if PIL_AVAILABLE and os.path.exists(image_path):
            try:
                img=Image.open(image_path).resize((self.w,self.h))
                self.photo=ImageTk.PhotoImage(img)
                lbl=tk.Label(self, image=self.photo, borderwidth=0, highlightthickness=0); lbl.pack()
            except Exception:
                self._fallback_canvas()
        else:
            self._fallback_canvas()
        self.canvas=tk.Canvas(self, width=960, height=20, bg="#d0d7e1", highlightthickness=0, bd=0)
        self.canvas.place(relx=0.5, rely=0.92, anchor="center")
        self.rect=self.canvas.create_rectangle(0,0,0,20, fill="#3378ff", width=0)
        self.update_idletasks()
        sw,sh=self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry(f"{self.w}x{self.h}+{(sw-self.w)//2}+{(sh-self.h)//2}")
        self.lift()
        self.after(15, self._animate)
    def _fallback_canvas(self):
        c=tk.Canvas(self, width=self.w, height=self.h, bg="#e8f0ff", highlightthickness=0); c.pack()
        c.create_text(self.w/2, self.h/2, text="SPID XY ROTATOR CONTROLLER BY SP1JMF", font=("Segoe UI",36,"bold"), fill="#1e3d6b")
    def _animate(self):
        elapsed=(time.time()-self.start_time)*1000.0
        prog=min(max(elapsed/self.duration,0.0),1.0)
        self.canvas.coords(self.rect, 0,0, 960*prog, 20)
        if prog<1.0: self.after(15, self._animate)
        else: self.destroy()

def main():
    root=tk.Tk(); root.withdraw()
    splash=SplashScreen(root, "splash_spid.png", duration=3000)
    def _start():
        try:
            app=App(root)
        except Exception as e:
            messagebox.showerror("Błąd startu", str(e)); raise
        root.deiconify()
    root.after(3050,_start)
    root.mainloop()

if __name__=="__main__":
    main()
