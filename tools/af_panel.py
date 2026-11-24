#!/usr/bin/env python3  
# tools/af_panel.py — панель настроек RP2040_audio_dsp.  
# Зависимости: pip install pyserial sounddevice. GUI — tkinter (стандартная поставка).  
#  
# Протокол (прошивка schema=1):  
#   version      -> "# version: NAME VER (schema=N store=N)"  
#   params dump  -> "# params begin count=N" / "P|label|type|cur|banked" /  
#                   "# S|section" / "# params end"  
#   state dump   -> "# state begin" / "R|name|val" / "# state end"  
#   <label> <v>  -> "# OK ..." | "# ERR ..."  
#   load N/save N-> банки 1..7  
  
import queue    
import threading    
import time    
import wave    
import tkinter as tk    
from tkinter import ttk, messagebox    
  
try:    
    import serial    
    from serial.tools import list_ports    
except ImportError:    
    raise SystemExit("Нужен pyserial: pip install pyserial")    
  
try:        
    import sounddevice as sd    
    import numpy as np    
except ImportError:        
    sd = None   # мониторинг и спектр недоступны, панель работает без звука          
  
try:  
    import lameenc  
except ImportError:  
    lameenc = None   # нет — запись пойдёт в WAV       
  
SCHEMA_SUPPORTED = 1    
BAUD = 115200    
  
# Группировка параметров по блокам аудиотракта.  
# (заголовок рамки, метка-флажок включения или None, [метки параметров]).  
# Порядок рамок = порядок в списке; метки не из списка попадут в "Прочее".  
# (заголовок рамки, метка-флажок или None, [метки], колонка 0..3).  
# Колонки 0-1 = RX, 2-3 = TX; рамки внутри колонки pack'уются сверху вниз.  
GROUPS = [      
    ("Полоса приёма", "rx band",      
     ["band lo", "band hi"], 0),      
    ("АРУ / лимитер", "agc",      
     ["gain", "agc ref", "agc max", "agc min", "lim ceil"], 0),      
    ("CW-фильтр / декодер", "cw",      
     ["cw f0", "cw bw", "cw noise", "cw dec", "cw wpm", "cw sql"], 0),      
    ("RX shelf / exciter", "rx shelf on",      
     ["rx shelf", "rx shelf f", "rx shelf q", "rx exc", "rx excmix"], 0),      
    ("Шумоподавление NR", "nr",      
     ["nr depth", "nr makeup", "nr mu"], 1),      
    ("Автонотч NT", "nt",      
     ["nt mu"], 1),      
    ("RTTY", "rtty",      
     ["rtty scan", "rtty rsid", "rtty rev", "rtty dbg", "rtty ctr", "rtty sql"], 1),      
    ("RX presence", "rx pres",      
     ["rx presmax", "rx pres f", "rx pres q"], 1),      
    ("RX EQ (3 полосы)", "rxeq on",      
     ["rxeq lo", "rxeq mid", "rxeq hi", "rxeq lo f", "rxeq mid f", "rxeq hi f"], 1),      
    ("TX полоса / АРУ", "tx band on",      
     ["tx band lo", "tx band hi", "tx agc on", "tx agc", "tx agcmax",      
      "tx agc max", "tx agc tgt", "tx gate on", "tx gate", "tx gatehold",      
      "tx gatehyst", "tx gate hys", "tx gate hld", "tx gfloor"], 2),      
    ("TX компрессор", "tx comp on",      
     ["tx comp", "tx 2band", "tx split", "tx lr2", "tx cthr", "tx cthr lo",      
      "tx cthr hi", "tx soft", "tx ap"], 2),      
    ("TX дакинг / разборчивость", "tx duck",      
     ["tx duckmax", "tx intel", "tx intel tgt", "tx intel cut", "tx intel bst"], 2),      
    ("TX pres / shelf / exc", "tx pres",      
     ["tx presmax", "tx pres f", "tx pres q", "tx shelf on",      
      "tx shelf", "tx shelf f", "tx shelf q", "tx exc", "tx excmix"], 3),      
    ("TX нотчи", "tx notch",      
     ["tx notch f1", "tx notch f2"], 3),      
    ("TX EQ (3 полосы)", "txeq on",        
     ["txeq lo", "txeq mid", "txeq hi", "txeq lo f", "txeq mid f", "txeq hi f"], 3),        
    ("Прочее RX", None,          
     ["voice vol", "nb", "nb thr"], 0),            
    ("TX Клип", "tx clip on",        
     ["tx clip"], 3),        
    ("Прочее", None, [], 2),   # нераспределённые метки — в колонку 3 (под TX дакинг)          
]          
  
# Runtime-режимы: НЕ банковые, команда "<name> on|off" выполняется немедленно.  
# Состояние читается из state dump (R|name|val). Порядок = порядок в рамке.  
# (имя в state dump R|name|, команда CLI). Имя и команда различаются,  
# где метка AF_PARAMS содержит пробел ("rx pres" -> R|rxpres|).  
RUNTIME_MODES = [        
    ("nt",      "nt"),        
    ("nr",      "nr"),        
    ("nb",      "nb"),        
    ("bypass",  "bypass"),        
    ("tx",      "tx"),        
    ("cw",      "cw"),        
    ("rtty",    "rtty"),        
    ("ramp",    "ramp"),        
    ("tt",      "tt"),        
    ("st",      "st"),        
]        
  
  
# ---------------------------------------------------------------------------  
# Serial worker: поток чтения -> очередь строк в GUI  
# ---------------------------------------------------------------------------  
class SerialWorker:  
    def __init__(self):  
        self.ser = None  
        self.rxq = queue.Queue()      # строки от платы  
        self.alive = False  
        self._lock = threading.Lock()  
  
    def open(self, port):  
        self.close()  
        self.ser = serial.Serial(port, BAUD, timeout=0.1)  
        self.alive = True  
        threading.Thread(target=self._reader, daemon=True).start()  
  
    def close(self):  
        self.alive = False  
        if self.ser:  
            try:  
                self.ser.close()  
            except Exception:  
                pass  
            self.ser = None  
  
    def _reader(self):  
        buf = b""  
        while self.alive and self.ser:  
            try:  
                data = self.ser.read(256)  
            except Exception:  
                self.rxq.put(("__disconnected__",))  
                return  
            if not data:  
                continue  
            buf += data  
            while b"\n" in buf:  
                line, buf = buf.split(b"\n", 1)  
                self.rxq.put(("line", line.decode("utf-8", "replace").rstrip("\r")))  
  
    def send(self, text):  
        with self._lock:  
            if self.ser:  
                self.ser.write((text + "\r\n").encode("utf-8"))  
  
  
# ---------------------------------------------------------------------------  
# GUI  
# ---------------------------------------------------------------------------  
class Panel(tk.Tk):  
    def __init__(self):  
        super().__init__()  
        self.title("RP2040 audio DSP by RU0AOG 2026 — панель настроек")  
        self.geometry("1150x810")    
  
        self.w = SerialWorker()  
        self.params = {}          # label -> {"type":..., "var":...}  
        self.schema_ok = False  
        self._dump_lines = []     # сбор строк дампа  
        self._dump_active = False  
        self._dump_count = -1  
        self._dump_got = 0  
        self._state_lines = []  
        self._state_active = False  
        self._pending = []        # очередь команд после connect      
        self.audio_in = None    # sd.InputStream при активном мониторинге  
        self.audio_out = None   # sd.OutputStream  
        self.audio_q = queue.Queue(maxsize=50)  # блоки вход->выход  
        self.audio_vol = 0.3      # громкость монитора 0.0..1.0 (читается из audio-потока)      
        self.rec_stream = None    # sd.InputStream при активной записи      
        self.rec_file = None      # wave.Wave_write или file для MP3  
        self.rec_enc = None       # lameenc.Encoder при записи в MP3  
        self.rec_lock = threading.Lock()      
        self.spec_q = queue.Queue(maxsize=8)   # блоки float32 для FFT    
        self.spec_fs = 44100    
        self.spec_on = False        
        self.spec_buf = None    
        self.spec_peak = -80.0      # peak-hold уровень, дБ  
        self.spec_peak_f = 0.0      # частота последнего пика, Гц    
        self.spec_hold = None       # послесвечение: max по бинам с затуханием (np.array)      
        self.spec_last = None       # (freqs, db) последнего кадра — для кнопки "Скрин"      
        self.snr_avg = None         # EMA-сглаженный SNR (~3 кадра)    
        self.snr_disp = None        # отображаемое значение, обновляется раз в 1 с    
        self.snr_next_ms = 0        # время следующего обновления цифр SNR и M/S    
        self.ms_avg = None          # EMA-сглаженные (mark, space) RTTY    
        self.ms_disp = None         # отображаемые (mark, space), раз в 1 с    
        self.thd_avg = None         # EMA THD, дБ    
        self.sinad_avg = None       # EMA SINAD, дБ    
        self.ts_disp = None         # отображаемые (thd, sinad), раз в 1 с    
        self.spec_full = False      # спектр развёрнут на всё окно (dblclick)    
        self.cmd_hist = []          # история введённых команд (стрелки вверх/вниз)  
        self.cmd_hist_i = 0         # позиция в истории; len = после последней  
        self.protocol("WM_DELETE_WINDOW", self._on_close)    
  
        self._build_top()      
        # Строка состояния (state dump) — на всю ширину внизу окна.  
        # pack side="bottom" ДО paned — иначе paned съест всё место.  
        ttk.Label(self, textvariable=self.state_var,  
                  anchor="w", foreground="#555").pack(  
                  side="bottom", fill="x", padx=6, pady=(0, 2))  
        # Горизонтальный сплит: слева таблица параметров, справа лог на всю высоту.    
        self.paned = ttk.PanedWindow(self, orient="horizontal")    
        self.paned.pack(fill="both", expand=True)    
        self._build_table()    
        self._build_bottom()      
        self.after(50, self._poll_rx)      
        self.after(400, self._auto_start)   # автовыбор порта/микрофона + старт  
  
    # --- layout -------------------------------------------------------------  
    def _build_top(self):  
        f = ttk.Frame(self)  
        f.pack(fill="x", padx=6, pady=4)  
        ttk.Label(f, text="Порт:").pack(side="left")  
        self.port_var = tk.StringVar()  
        self.port_cb = ttk.Combobox(f, textvariable=self.port_var, width=12,    
                                    state="readonly",    
                                    values=self._ports())    
        self.port_cb.pack(side="left", padx=4)      
        self.btn_conn = ttk.Button(f, text="Подключить", command=self._connect)       
        self.btn_conn.pack(side="left", padx=8)    
        ttk.Button(f, text="Обновить", command=self._refresh_ports).pack(side="left")    
        self.status_var = tk.StringVar(value="не подключено")  
        ttk.Label(f, textvariable=self.status_var).pack(side="left", padx=8)  
  
        bf = ttk.Frame(self)  
        bf.pack(fill="x", padx=6)  
        ttk.Label(bf, text="Банк:").pack(side="left")  
        for n in range(1, 8):  
            ttk.Button(bf, text=f"load {n}",  
                       command=lambda n=n: self._cmd(f"load {n}", redump=True)  
                       ).pack(side="left", padx=1)  
            ttk.Button(bf, text=f"save {n}",    
                       command=lambda n=n: self._cmd(f"save {n}")    
                       ).pack(side="left", padx=1)    
        self.state_var = tk.StringVar(value="")      
  
        # Рамка "Режимы" — runtime-флажки из state dump, не сохраняются в банк.  
        mf = ttk.LabelFrame(self, text="Режимы (runtime, не в банке)")  
        mf.pack(fill="x", padx=6, pady=(4, 0))  
        self.mode_vars = {}          # name -> IntVar  
        self._mode_updating = False  # подавить command при обновлении из дампа  
        for i, (name, cmd) in enumerate(RUNTIME_MODES):      
            var = tk.IntVar(value=0)      
            ttk.Checkbutton(      
                mf, text=cmd, variable=var,      
                command=lambda c=cmd, v=var: self._mode_set(c, v.get())      
            ).grid(row=i // 5, column=i % 5, sticky="w", padx=4)
            self.mode_vars[name] = var      
  
        # Аудиомонитор: UAC-устройство платы -> динамики ПК (справа в рамке).  
        self.audio_var = tk.StringVar()  
        self.audio_cb = ttk.Combobox(mf, textvariable=self.audio_var,  
                                     width=28, state="readonly",  
                                     values=self._audio_inputs())  
        self.audio_cb.grid(row=0, column=6, rowspan=2, padx=(16, 4), sticky="e")  
        self.audio_btn = ttk.Button(mf, text="Монитор",    
                                    command=self._audio_toggle)    
        self.audio_btn.grid(row=0, column=7, rowspan=2, padx=4, sticky="e")    
        # Громкость монитора (аналог volumeSlider/waveOutSetVolume в RP2040_Control)  
        self.audio_scale = ttk.Scale(  
            mf, from_=0, to=100, orient="horizontal", length=90,  
            command=lambda v: setattr(self, "audio_vol", float(v) / 100.0))  
        self.audio_scale.set(30)    
        self.audio_scale.grid(row=0, column=8, rowspan=2, padx=(0, 4), sticky="e")    
        self.rec_btn = ttk.Button(mf, text="Запись",    
                                  command=self._rec_toggle)    
        self.rec_btn.grid(row=0, column=9, rowspan=2, padx=4, sticky="e")    
        self.shot_btn = ttk.Button(mf, text="Скрин",  
                                   command=self._spec_shot)  
        self.shot_btn.grid(row=0, column=10, rowspan=2, padx=4, sticky="e")  
        if sd is None:      
            self.audio_cb.configure(state="disabled")      
            self.audio_btn.configure(state="disabled")      
            self.audio_scale.configure(state="disabled")    
            self.rec_btn.configure(state="disabled")  
        mf.columnconfigure(6, weight=1)   # прижать аудио-блок вправо      
  
    def _build_table(self):      
        self.tbl_wrap = ttk.Frame(self.paned)      
        self.paned.add(self.tbl_wrap, weight=3)      
        self.canvas = tk.Canvas(self.tbl_wrap, highlightthickness=0)      
        sb = ttk.Scrollbar(self.tbl_wrap, orient="vertical", command=self.canvas.yview)      
        self.tbl = ttk.Frame(self.canvas)      
        self.tbl.bind("<Configure>",    
                      lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))    
        self.canvas.create_window((0, 0), window=self.tbl, anchor="nw")    
        self.canvas.configure(yscrollcommand=sb.set)    
        self.canvas.pack(side="left", fill="both", expand=True)    
        sb.pack(side="right", fill="y")    
        # 4 вертикальные колонки: рамки pack'уются сверху вниз без "дыр" grid'а.  
        self.columns = []  
        for c in range(4):  
            cf = ttk.Frame(self.tbl)  
            cf.grid(row=0, column=c, sticky="new", padx=2)  
            self.columns.append(cf)  
        # Колесо — только над таблицей; bind_all перехватывал бы и лог.      
        self.canvas.bind("<Enter>",      
            lambda _e: self.canvas.bind_all("<MouseWheel>",      
                lambda e: self.canvas.yview_scroll(-e.delta // 120, "units")))      
        self.canvas.bind("<Leave>",      
            lambda _e: self.canvas.unbind_all("<MouseWheel>"))      
  
    def _build_bottom(self):        
        # Правая панель: спектр сверху, лог ниже + командная строка + статус.      
        self.right_fr = ttk.Frame(self.paned)        
        self.paned.add(self.right_fr, weight=1)        
        # Спектр входа UAC: текущий срез FFT 0..6 кГц / 0..-80 дБ,    
        # маркеры band lo/hi. Работает, пока включён монитор.    
        # Двойной щелчок — развернуть/свернуть на всё окно.    
        self.spec_fr = ttk.LabelFrame(self.right_fr, text="Спектр (0-6 кГц)")      
        self.spec_fr.pack(fill="x", padx=(0, 6), pady=(4, 0))      
        self.spec_cv = tk.Canvas(self.spec_fr, height=280, bg="#101010",      
                                 highlightthickness=0)      
        self.spec_cv.pack(fill="x", padx=4, pady=4)      
        self.spec_cv.bind("<Double-Button-1>", lambda _e: self._spec_toggle_full())  
        # height ограничивает высоту; bg — светло-серый фон лога.  
        # state="normal" + блокировка ввода: текст можно выделять и  
        # копировать (Ctrl+C / ПКМ-выделение), но не редактировать.  
        log_wrap = ttk.Frame(self.right_fr)  
        log_wrap.pack(fill="both", expand=True, padx=(0, 6), pady=4)  
        self.log = tk.Text(log_wrap, width=42, height=14,  
                           bg="#f0f0f0")  
        log_sb = ttk.Scrollbar(log_wrap, orient="vertical",  
                               command=self.log.yview)  
        self.log.configure(yscrollcommand=log_sb.set)  
        self.log.pack(side="left", fill="both", expand=True)  
        log_sb.pack(side="right", fill="y")  
        self.log.bind("<Key>", lambda _e: "break")          # read-only    
        for ev in ("<<Paste>>", "<<Cut>>", "<<Clear>>"):    
            self.log.bind(ev, lambda _e: "break")    
        self.log.bind("<Button-3>", self._log_menu)         # ПКМ — меню копирования  
        self.cmd_fr = ttk.Frame(self.right_fr)          
        self.cmd_fr.pack(fill="x", padx=(0, 6), pady=(0, 2))          
        self.cmd_var = tk.StringVar()          
        e = ttk.Entry(self.cmd_fr, textvariable=self.cmd_var)          
        e.pack(side="left", fill="x", expand=True)        
        e.bind("<Return>", lambda _e: self._send_manual())  
        # ПКМ — вставить из буфера (как Ctrl+V); стрелки — история команд.  
        e.bind("<Button-3>", lambda _e: (e.event_generate("<<Paste>>"), "break")[1])  
        e.bind("<Up>",   lambda _e: self._cmd_hist_move(-1))  
        e.bind("<Down>", lambda _e: self._cmd_hist_move(+1))  
        self.cmd_entry = e    
        ttk.Button(self.cmd_fr, text="Отправить", command=self._send_manual).pack(side="left", padx=4)             
        ttk.Button(self.cmd_fr, text="Очистить", command=self._log_clear).pack(side="left")      
        ttk.Button(self.cmd_fr, text="Скопировать", command=self._log_copy).pack(side="left", padx=(4, 0))      
  
    # --- helpers ------------------------------------------------------------  
    def _ports(self):  
        return [p.device for p in list_ports.comports()]  
  
    def _refresh_ports(self):  
        self.port_cb["values"] = self._ports()  

    def _auto_start(self):  
        # Автостарт: первый порт кроме COM1 + USB-микрофон -> connect + монитор.  
        # Порт: пропускаем COM1 (системный), берём первый оставшийся.  
        if self.w.ser is None:  
            ports = self._ports()  
            port = next((p for p in ports if p.upper() != "COM1"), None)  
            if port:  
                self.port_var.set(port)  
                self._connect()  
            else:  
                self._log("# auto: COM-порт не найден (кроме COM1)")  
        # Микрофон: первое устройство с "usb" в имени.  
        if sd is not None and self.audio_in is None:  
            devs = self.audio_cb["values"]  
            mic = next((d for d in devs if "usb" in d.lower()), None)  
            if mic:  
                self.audio_var.set(mic)  
                self._audio_toggle()  
            else:  
                self._log("# auto: USB-микрофон не найден")  

    def _log(self, s):      
        self.log.insert("end", s + "\n")      
        self.log.see("end")      
      
    def _log_clear(self):      
        self.log.delete("1.0", "end")      
    
    def _log_copy(self):      
        # Весь лог в буфер обмена.      
        self.clipboard_clear()      
        self.clipboard_append(self.log.get("1.0", "end-1c"))      
  
    def _log_menu(self, event):  
        # ПКМ в логе: меню копирования. "Копировать" активна только  
        # при наличии выделения.  
        m = tk.Menu(self, tearoff=0)  
        has_sel = bool(self.log.tag_ranges("sel"))  
        m.add_command(label="Копировать",  
                      command=self._log_copy_sel,  
                      state="normal" if has_sel else "disabled")  
        m.add_command(label="Копировать всё", command=self._log_copy)  
        m.tk_popup(event.x_root, event.y_root)  
        return "break"  
  
    def _log_copy_sel(self):  
        # Выделенный фрагмент лога в буфер обмена.  
        try:  
            txt = self.log.get("sel.first", "sel.last")  
        except tk.TclError:  
            return  
        self.clipboard_clear()  
        self.clipboard_append(txt)  
    def _audio_inputs(self):        
        # Входные устройства (UAC-плата видна как микрофонный вход).      
        # Только WASAPI: MME/DirectSound дают виртуальные дубли    
        # ("Первичный драйвер записи", "Стерео микшер", "Переназначение...").    
        if sd is None:        
            return []        
        try:        
            wasapi = {i for i, h in enumerate(sd.query_hostapis())    
                      if "WASAPI" in h["name"]}    
            if not wasapi:  
                # Не Windows (Linux/Mac) — WASAPI нет, берём все входы.  
                return [f"{i}: {d['name']}"  
                        for i, d in enumerate(sd.query_devices())  
                        if d["max_input_channels"] > 0]  
            return [f"{i}: {d['name']}"    
                    for i, d in enumerate(sd.query_devices())    
                    if d["max_input_channels"] > 0    
                    and d["hostapi"] in wasapi]    
        except Exception:        
            return []        

    def _probe_fs(self, idx, kind, candidates):  
        # Первая частота, которую устройство реально принимает.  
        for fs in dict.fromkeys(candidates):  
            try:  
                if kind == "in":  
                    sd.check_input_settings(device=idx, samplerate=fs)  
                else:  
                    sd.check_output_settings(device=idx, samplerate=fs)  
                return fs  
            except Exception:  
                continue  
        return None  

    def _audio_toggle(self):    
        if self.audio_in is not None:    
            self._audio_stop()  
            self._log("# audio: монитор выкл")    
            return    
        sel = self.audio_var.get()    
        if not sel:    
            messagebox.showerror("Аудио", "Выберите аудиоустройство")    
            return    
        idx = int(sel.split(":")[0])        
        try:        
            info = sd.query_devices(idx)        
            ch = min(2, int(info["max_input_channels"])) or 1        
            # WASAPI shared mode принимает только частоту, настроенную  
            # в Windows для устройства — перебираем кандидатов.  
            fs_in = self._probe_fs(idx, "in",  
                                   (44100, 48000, int(info["default_samplerate"])))  
            out_def = sd.query_devices(sd.default.device[1])  
            fs_out = self._probe_fs(sd.default.device[1], "out",  
                                    (fs_in, 44100, 48000,  
                                     int(out_def["default_samplerate"])))  
            if fs_in is None or fs_out is None:  
                raise RuntimeError("нет поддерживаемой частоты "  
                                   f"(in={fs_in} out={fs_out})")  
            fs = fs_in  
            blk = fs // 50  
            if fs_out != fs_in:  
                self._log(f"# audio: внимание fs_in={fs_in} != fs_out={fs_out}"  
                          " — возможен сдвиг тона")  
  
            # Как в RP2040_Control: вход и выход — НЕЗАВИСИМЫЕ потоки  
            # (waveIn + waveOut/WAVE_MAPPER), буферы копируются через очередь.  
            # Duplex sd.Stream не используем — PortAudio требует один host API  
            # для входа и выхода и падает с PaErrorCode -9993.  
            def in_cb(indata, frames, time_, status):    
                try:    
                    self.audio_q.put_nowait(indata.copy())    
                except queue.Full:    
                    pass            # выход не успевает — блок теряем    
                try:    
                    self.spec_q.put_nowait(indata[:, 0].copy())  # канал 0 -> FFT  
                except queue.Full:    
                    pass    
  
            def out_cb(outdata, frames, time_, status):  
                try:  
                    data = self.audio_q.get_nowait()  
                    n = min(len(data), len(outdata))  
                    outdata[:n] = data[:n] * self.audio_vol  
                    outdata[n:] = 0  
                except queue.Empty:  
                    outdata[:] = 0  
  
            self.audio_out = sd.OutputStream(  
                samplerate=fs_out, channels=ch, dtype="float32",  
                blocksize=fs_out // 50, callback=out_cb)  # выход по умолчанию  
            self.audio_in = sd.InputStream(  
                device=idx, samplerate=fs_in, channels=ch,  
                dtype="float32", blocksize=blk, callback=in_cb)  
            self.audio_out.start()    
            self.audio_in.start()    
            self.spec_fs = fs_in    
            self.spec_on = True    
            self._spec_draw()    
            self._log(f"# audio: {info['name']} fs={fs} ch={ch}")    
        except Exception as ex:      
            self._audio_stop()  
            messagebox.showerror("Аудио", str(ex))      
            return      
        self.audio_btn.configure(text="Стоп")    
        self._log(f"# audio: монитор <- {sel}")    
  
    def _audio_stop(self):  
        for attr in ("audio_in", "audio_out"):  
            st = getattr(self, attr)  
            if st is not None:  
                try:  
                    st.stop()  
                    st.close()  
                except Exception:  
                    pass  
                setattr(self, attr, None)  
        while not self.audio_q.empty():    
            try:    
                self.audio_q.get_nowait()    
            except queue.Empty:    
                break    
        self.spec_on = False        
        self.spec_buf = None    
        self.spec_peak = -80.0      
        self.spec_peak_f = 0.0    
        self.spec_hold = None    
        self.snr_avg = None      
        self.snr_disp = None    
        self.snr_next_ms = 0    
        self.ms_avg = None    
        self.ms_disp = None    
        self.thd_avg = None    
        self.sinad_avg = None    
        self.ts_disp = None    
        while not self.spec_q.empty():        
            try:    
                self.spec_q.get_nowait()    
            except queue.Empty:    
                break    
        if hasattr(self, "spec_cv"):    
            self.spec_cv.delete("all")    
        self.audio_btn.configure(text="Монитор")    

    def _rec_toggle(self):  
        if self.rec_stream is not None:  
            self._rec_stop()  
            self._log("# rec: стоп")  
            return  
        sel = self.audio_var.get()  
        if not sel:  
            messagebox.showerror("Запись", "Выберите аудиоустройство")  
            return  
        idx = int(sel.split(":")[0])    
        try:    
            info = sd.query_devices(idx)    
            ch = min(2, int(info["max_input_channels"])) or 1    
            # Как в мониторе: WASAPI принимает только частоту, настроенную  
            # в Windows — перебираем кандидатов (иначе PaErrorCode -9997).  
            fs = self._probe_fs(idx, "in",  
                                (44100, 48000, int(info["default_samplerate"])))  
            if fs is None:  
                raise RuntimeError("нет поддерживаемой частоты записи")  
            if lameenc is not None:  
                # MP3: lameenc кодирует int16 PCM на лету, 64 кбит/с mono  
                # (~0.5 МБ/мин против ~5.5 МБ/мин у WAV). Stereo -> mono.  
                fname = time.strftime("rec_%Y%m%d_%H%M%S.mp3")  
                enc = lameenc.Encoder()  
                enc.set_bit_rate(64)  
                enc.set_in_sample_rate(fs)  
                enc.set_channels(1)  
                enc.set_quality(2)  
                self.rec_enc = enc  
                self.rec_file = open(fname, "wb")  
  
                def rec_cb(indata, frames, time_, status):  
                    with self.rec_lock:  
                        if self.rec_file is not None and self.rec_enc is not None:  
                            pcm = indata[:, 0] if indata.ndim > 1 else indata  
                            data = self.rec_enc.encode(pcm.tobytes())  
                            if data:  
                                self.rec_file.write(data)  
            else:  
                fname = time.strftime("rec_%Y%m%d_%H%M%S.wav")      
                wf = wave.open(fname, "wb")    
                wf.setnchannels(ch)    
                wf.setsampwidth(2)              # int16    
                wf.setframerate(fs)    
    
                def rec_cb(indata, frames, time_, status):    
                    # int16 -> сырые байты в WAV; запись короткая, блокировка не мешает    
                    with self.rec_lock:    
                        if self.rec_file is not None:    
                            self.rec_file.writeframes(indata.tobytes())    
    
            # до стрима: callback увидит файл сразу, а _rec_stop закроет при ошибке    
            self.rec_stream = sd.InputStream(    
                device=idx, samplerate=fs, channels=ch,    
                dtype="int16", blocksize=fs // 50,    
                callback=rec_cb)    
            self.rec_stream.start()    
        except Exception as ex:    
            self._rec_stop()    
            messagebox.showerror("Запись", str(ex))  
            return  
        self.rec_btn.configure(text="Стоп rec")    
        fmt = "mp3 64k" if lameenc is not None else "wav (lameenc не найден)"  
        self._log(f"# rec: -> {fname} [{fmt}] <- {sel}")    
  
    def _rec_stop(self):  
        if self.rec_stream is not None:  
            try:  
                self.rec_stream.stop()  
                self.rec_stream.close()  
            except Exception:  
                pass  
            self.rec_stream = None  
        with self.rec_lock:    
            if self.rec_file is not None:    
                try:  
                    if self.rec_enc is not None:  
                        tail = self.rec_enc.flush()   # хвост MP3-фреймов  
                        if tail:  
                            self.rec_file.write(tail)  
                    self.rec_file.close()    
                except Exception:    
                    pass    
                self.rec_file = None    
                self.rec_enc = None  
        self.rec_btn.configure(text="Запись")    

    SPEC_NFFT      = 4096    # окно FFT обычное;  ~10.8 Гц/бин при fs=44100  
    SPEC_NFFT_FULL = 4096    # развёрнутый спектр; ~5.4 Гц/бин  

    def _spec_toggle_full(self):    
        # Двойной щелчок по спектру: развернуть на всё окно / свернуть.    
        self.spec_full = not self.spec_full    
        self.spec_hold = None   # длина бинов сменится — пересоздать  
        if self.spec_full:    
            self.paned.forget(self.tbl_wrap)          # скрыть таблицу параметров    
            self.log.master.pack_forget()             # скрыть лог (обёртка со скроллом)    
            self.cmd_fr.pack_forget()                 # и командную строку  
            self.spec_fr.pack(fill="both", expand=True,  
                              padx=(0, 6), pady=(4, 0))  
            self.spec_cv.pack(fill="both", expand=True, padx=4, pady=4)  
        else:    
            self.spec_cv.pack(fill="x", padx=4, pady=4)    
            self.spec_fr.pack(fill="x", padx=(0, 6), pady=(4, 0))    
            self.log.master.pack(fill="both", expand=True, padx=(0, 6), pady=4)    
            self.cmd_fr.pack(fill="x", padx=(0, 6), pady=(0, 2))  
            self.paned.insert(0, self.tbl_wrap, weight=3)  # таблица слева  

    def _spec_draw(self):    
        # Перерисовка ~12 раз/с: сетка дБ/Гц + маркеры ПФ + полилиния спектра.  
        # Фиксированные диапазоны: 0..6 кГц по X, 0..-80 дБ по Y.    
        if not self.spec_on:    
            return    
        cv = self.spec_cv        
        nfft = self.SPEC_NFFT_FULL if self.spec_full else self.SPEC_NFFT  
        # Блоки из in_cb идут по fs//50 (~882 при 44100) — меньше NFFT.    
        # Сливаем всё из очереди в накопитель и берём последние nfft.    
        try:        
            while True:    
                blk = self.spec_q.get_nowait()    
                self.spec_buf = blk if self.spec_buf is None \
                    else np.concatenate((self.spec_buf, blk))    
        except queue.Empty:        
            pass    
        if self.spec_buf is not None and len(self.spec_buf) > 4 * nfft:    
            self.spec_buf = self.spec_buf[-2 * nfft:]  # не растим хвост    
        cv.delete("all")    
        w = max(cv.winfo_width(), 100)    
        h = max(cv.winfo_height(), 60)    
        fmax = 6000    
        plot_h = h - 14                     # нижние 14 px — подписи частот    
        for db in range(-10, -80, -10):     # сетка уровней, шаг 10 дБ, -80 внизу        
            y = -db / 80.0 * plot_h        
            cv.create_line(0, y, w, y, fill="#303030")      
            cv.create_text(2, y, text=str(db), anchor="nw", fill="#888",      
                           font=("TkDefaultFont", 7))      
        f = 1000    
        while f < fmax:                     # сетка частот, шаг 1 кГц    
            x = f / fmax * w    
            cv.create_line(x, 0, x, plot_h, fill="#303030")    
            cv.create_text(x, h - 12, text=f"{f // 1000}k", anchor="n",    
                           fill="#888", font=("TkDefaultFont", 7))    
            f += 1000    
        # Маркеры полосы ПФ — только когда фильтр в тракте (rx band on)  
        p_band = self.params.get("rx band")  
        band_on = True  
        if p_band:  
            try:  
                band_on = (int(p_band["var"].get()) == 1)  
            except (ValueError, tk.TclError):  
                band_on = True  
        lo = hi = None    
        if band_on:      
            for lbl in ("band lo", "band hi"):  # маркеры полосы ПФ              
                p = self.params.get(lbl)        
                if p:        
                    try:        
                        fx = float(p["var"].get())        
                        if lbl == "band lo":  
                            lo = fx  
                        else:  
                            hi = fx  
                        if 0 < fx < fmax:        
                            x = fx / fmax * w        
                            cv.create_line(x, 0, x, plot_h,        
                                           fill="#ff5555", dash=(3, 2))        
                    except (ValueError, tk.TclError):        
                        pass  
            if lo is not None and hi is not None:        
                cv.create_rectangle(w - 120, 2, w - 2, 16, fill="#101010",        
                                    outline="")   # подложка под подпись        
                cv.create_text(w - 4, 4, text=f"{lo:.0f}–{hi:.0f} Гц",        
                               fill="#ff5555", anchor="ne",        
                               font=("TkDefaultFont", 8))          
        if self.spec_buf is not None and len(self.spec_buf) >= nfft:        
            win = np.hanning(nfft)    
            seg = self.spec_buf[-nfft:]    
            seg = seg - np.mean(seg)    
            mag = np.abs(np.fft.rfft(seg * win))    
            db = 20 * np.log10(mag / (win.sum() / 2) + 1e-9)   # компенсация ~6 дБ окна    
            freqs = np.fft.rfftfreq(nfft, 1.0 / self.spec_fs)         
            m = (freqs >= 50) & (freqs <= fmax)   # DC и <50 Гц не рисуем      
            dbm = np.clip(db[m], -80.0, 0.0)      
            self.spec_last = (freqs[m].copy(), dbm.copy())   # кадр для "Скрин"  
            # Послесвечение: per-bin max с затуханием ~1.5 дБ/кадр (~18 дБ/с).  
            # Рисуется тёмно-зелёным ПОД основной линией.  
            if self.spec_hold is None or len(self.spec_hold) != len(dbm):    
                self.spec_hold = np.full(len(dbm), -80.0)    
            self.spec_hold = np.maximum(dbm, self.spec_hold - 1.5)  
            pts_h = []  
            for x, y in zip(freqs[m] / fmax * w,    
                            np.clip(-self.spec_hold, 0, 80) / 80.0 * plot_h):    
                pts_h += [x, y]  
            if len(pts_h) >= 4:  
                cv.create_line(*pts_h, fill="#0a5a0a", width=1, smooth=True)  
            pts = []      
            for x, y in zip(freqs[m] / fmax * w,        
                            np.clip(-dbm, 0, 80) / 80.0 * plot_h):        
                pts += [x, y]      
            if len(pts) >= 4:        
                cv.create_line(*pts, fill="#00ff00", width=1, smooth=True)    
            # Peak-hold: жёлтый пунктир на уровне максимума, затухание  
            # 0.5 дБ/кадр (~6 дБ/с). Частота — у последнего пробившего пика.  
            mpk = m   # <50 Гц уже отрезаны маской отрисовки  
            pk = int(np.argmax(db[mpk]))    
            pk_db = float(db[mpk][pk])    
            if pk_db >= self.spec_peak:    
                self.spec_peak = pk_db    
                self.spec_peak_f = float(freqs[mpk][pk])    
            else:  
                self.spec_peak = max(-80.0, self.spec_peak - 0.5)    
            py = np.clip(-self.spec_peak, 0, 80) / 80.0 * plot_h        
            cv.create_line(0, py, w, py, fill="#ffff00", dash=(2, 4))      
            cv.create_rectangle(w - 62, 20, w - 2, 48, fill="#101010",        
                                outline="")   # подложка под подписи        
            cv.create_text(w - 4, 22, text=f"{self.spec_peak:.0f} dB",        
                           fill="#ffff00", anchor="ne",        
                           font=("TkDefaultFont", 8))        
            cv.create_text(w - 4, 36, text=f"{self.spec_peak_f:.0f} Гц",        
                           fill="#ffff00", anchor="ne",        
                           font=("TkDefaultFont", 8))        
  
            # SNR: пик к медианному шуму. Полоса измерения — ПФ с отступом    
            # 10% от краёв (скаты фильтра не считаем сигналом); если фильтр    
            # выключен или границ нет — вся полоса отрисовки 0..6 кГц.    
            fm = freqs[m]        
            dbu = db[m]                       # необрезанный — для медианы шума    
            if lo is not None and hi is not None and hi > lo:        
                pad = 0.1 * (hi - lo)        
                sm = (fm >= lo + pad) & (fm <= hi - pad)        
            else:        
                sm = np.ones(len(fm), dtype=bool)        
            # Шум — медиана по полосе БЕЗ зоны пика ±100 Гц: юбка      
            # лепестка и гармоники не должны завышать оценку шума.      
            i_pk = int(np.argmax(dbu))      
            nz = sm & (np.abs(fm - fm[i_pk]) > 100.0)      
            if nz.any():      
                snr = float(dbu[i_pk] - np.median(dbu[nz]))      
            elif sm.any():      
                snr = float(dbu[sm].max() - np.median(dbu[sm]))      
            else:      
                snr = float(dbu.max() - np.median(dbu))           
            # EMA только на число: спектр мгновенный, подпись не пляшет    
            if self.snr_avg is None:    
                self.snr_avg = snr    
            else:    
                self.snr_avg = 0.3 * snr + 0.7 * self.snr_avg   # ~3 кадра    
            now_ms = int(time.monotonic() * 1000)    
            tick = False    
            if self.snr_disp is None or now_ms >= self.snr_next_ms:      
                self.snr_disp = self.snr_avg      
                self.snr_next_ms = now_ms + 1000          # цифра — раз в 1 с      
                tick = True    
            cv.create_rectangle(w - 92, 52, w - 2, 66, fill="#101010", outline="")        
            cv.create_text(w - 4, 54, text=f"SNR {self.snr_disp:.0f} дБ",        
                           fill="#00ccff", anchor="ne",        
                           font=("TkDefaultFont", 8))                          
  
            # THD / SINAD — второй столбец, от мгновенного пика (тот же      
            # бин, что цифра в правом верхнем углу).      
            magm = mag[m]        
            i0 = int(np.argmax(magm))        
            f0 = fm[i0]        
            # Полоса измерения — ПФ целиком (band lo..hi); без фильтра —      
            # вся полоса отрисовки. Отступ 10% нужен был только медиане SNR.    
            if lo is not None and hi is not None and hi > lo:    
                bm = (fm >= lo) & (fm <= hi)    
            else:    
                bm = np.ones(len(fm), dtype=bool)    
            # Сигнальная маска: от пика в обе стороны, пока бины выше    
            # медианы шума +6 дБ — весь лепесток с юбкой считается сигналом,    
            # а не шумом (фиксированные ±60 Гц резали широкий лепесток).    
            noise_db = float(np.median(dbu[bm])) if bm.any() else -80.0    
            thr = noise_db + 6.0    
            sig_m = np.zeros(len(fm), dtype=bool)    
            sig_m[i0] = True    
            j = i0 - 1    
            while j >= 0 and dbu[j] > thr:      # влево от пика    
                sig_m[j] = True    
                j -= 1    
            j = i0 + 1    
            while j < len(fm) and dbu[j] > thr:  # вправо от пика    
                sig_m[j] = True    
                j += 1    
            p_sig = float(np.sum(magm[sig_m] ** 2))        
            # THD: гармоники 2f0..Nf0 в полосе bm, вне сигнальной маски    
            # (гармоника, попавшая внутрь лепестка, не считается).    
            p_harm = 0.0        
            k = 2        
            while k * f0 <= fm[-1]:        
                ih = int(np.argmin(np.abs(fm - k * f0)))        
                if bm[ih] and not sig_m[ih]:    
                    h_lo, h_hi = max(0, ih - 1), min(len(magm), ih + 2)        
                    p_harm += float(np.sum(magm[h_lo:h_hi] ** 2))        
                k += 1        
            thd_db = 10.0 * np.log10((p_harm + 1e-12) / (p_sig + 1e-12))        
            # SINAD = сигнал / (шум+искажения) в полосе ПФ.    
            nd_m = bm & ~sig_m        
            p_nd = float(np.sum(magm[nd_m] ** 2)) if nd_m.any() else 0.0        
            sinad_db = 10.0 * np.log10((p_sig + 1e-12) / (p_nd + 1e-12))               
            if self.thd_avg is None:    
                self.thd_avg, self.sinad_avg = thd_db, sinad_db    
            else:    
                self.thd_avg = 0.3 * thd_db + 0.7 * self.thd_avg    
                self.sinad_avg = 0.3 * sinad_db + 0.7 * self.sinad_avg    
            if self.ts_disp is None or tick:    
                self.ts_disp = (self.thd_avg, self.sinad_avg)    
            cv.create_rectangle(w - 92, 70, w - 2, 98, fill="#101010", outline="")        
            cv.create_text(w - 4, 72, text=f"THD {self.ts_disp[0]:.0f} дБ",        
                           fill="#00ccff", anchor="ne",        
                           font=("TkDefaultFont", 8))        
            cv.create_text(w - 4, 86, text=f"SINAD {self.ts_disp[1]:.0f} дБ",        
                           fill="#00ccff", anchor="ne",        
                           font=("TkDefaultFont", 8))                        
  
            # Частоты режимов — справа внизу, над подписями сетки.      
            if self.mode_vars.get("rtty") and self.mode_vars["rtty"].get():      
                # RTTY: два сильнейших пика с разносом >= 100 Гц -> mark/space.      
                i1 = int(np.argmax(dbm))      
                far = np.abs(fm - fm[i1]) >= 100.0      
                if far.any():      
                    i2 = int(np.argmax(np.where(far, dbm, -80.0)))      
                    f_mark, f_space = max(fm[i1], fm[i2]), min(fm[i1], fm[i2])      
                    # EMA ~3 кадра на обе частоты — как у SNR  
                    if self.ms_avg is None:  
                        self.ms_avg = (f_mark, f_space)  
                    else:  
                        self.ms_avg = (0.3 * f_mark + 0.7 * self.ms_avg[0],  
                                       0.3 * f_space + 0.7 * self.ms_avg[1])  
                    # Цифры — раз в 1 с, на том же тике, что и SNR    
                    if self.ms_disp is None or tick:    
                        self.ms_disp = self.ms_avg    
                    cv.create_rectangle(w - 92, plot_h - 30, w - 2, plot_h - 2,      
                                        fill="#101010", outline="")      
                    cv.create_text(w - 4, plot_h - 28,      
                                   text=f"M {self.ms_disp[0]:.0f} Гц",      
                                   fill="#ffaa00", anchor="ne",      
                                   font=("TkDefaultFont", 8))      
                    cv.create_text(w - 4, plot_h - 14,      
                                   text=f"S {self.ms_disp[1]:.0f} Гц",      
                                   fill="#ffaa00", anchor="ne",      
                                   font=("TkDefaultFont", 8))       
            elif self.mode_vars.get("cw") and self.mode_vars["cw"].get():    
                # CW: частота тона = частота пика спектра.    
                cv.create_rectangle(w - 92, plot_h - 16, w - 2, plot_h - 2,    
                                    fill="#101010", outline="")    
                cv.create_text(w - 4, plot_h - 14,    
                               text=f"CW {self.spec_peak_f:.0f} Гц",    
                               fill="#ffaa00", anchor="ne",    
                               font=("TkDefaultFont", 8))    
        self.after(80, self._spec_draw)            

    def _spec_shot(self):  
        # Снимок спектра в высоком разрешении: перерисовка последнего  
        # кадра через Pillow в 1920x1080. Без Pillow — fallback на  
        # postscript виджета (вектор, конвертируется в PDF/PNG).  
        fname = time.strftime("spec_%Y%m%d_%H%M%S")  
        try:  
            from PIL import Image, ImageDraw  
        except ImportError:  
            Image = None  
        if Image is not None and self.spec_last is not None:    
            W, H, fmax = 1920, 1080, 6000    
            plot_h = H - 40    
            img = Image.new("RGB", (W, H), "#101010")    
            dr = ImageDraw.Draw(img)    
            # TrueType с кириллицей; без него — дефолтный bitmap (мелкий, без рус.)    
            try:
                from PIL import ImageFont
                fnt = ImageFont.truetype("arial.ttf", 22)
                fnt_sm = ImageFont.truetype("arial.ttf", 18)
            except Exception:    
                try:    
                    fnt = ImageFont.truetype("DejaVuSans.ttf", 22)    
                    fnt_sm = ImageFont.truetype("DejaVuSans.ttf", 18)    
                except Exception:    
                    fnt = fnt_sm = ImageFont.load_default()    
            def dash_line(x0, y0, x1, y1, fill, dash=8, gap=6, width=2):    
                # PIL не умеет пунктир — рисуем штрихи вручную (вертикаль)    
                y = y0    
                while y < y1:    
                    dr.line([(x0, y), (x1, min(y + dash, y1))],    
                            fill=fill, width=width)    
                    y += dash + gap    
            for dbv in range(-10, -80, -10):          
                y = -dbv / 80.0 * plot_h          
                dr.line([(0, y), (W, y)], fill="#303030")                 
            for f in range(1000, fmax, 1000):      
                x = f / fmax * W      
                dr.line([(x, 0), (x, plot_h)], fill="#303030")       
            p_band = self.params.get("rx band")  
            band_on = True  
            if p_band:  
                try:  
                    band_on = (int(p_band["var"].get()) == 1)  
                except (ValueError, tk.TclError):  
                    band_on = True  
            if band_on:  
                lo = hi = None  
                for lbl in ("band lo", "band hi"):  
                    p = self.params.get(lbl)  
                    if p:  
                        try:  
                            fx = float(p["var"].get())  
                            if lbl == "band lo":  
                                lo = fx  
                            else:  
                                hi = fx  
                            if 0 < fx < fmax:    
                                x = fx / fmax * W    
                                dash_line(x, 0, x, plot_h, "#7a2a2a")      
                        except (ValueError, tk.TclError):    
                            pass    
                if lo is not None and hi is not None:          
                    t = f"{lo:.0f}–{hi:.0f} Гц"        
                    tw = dr.textlength(t, font=fnt)        
                    dr.rectangle([W - 18 - tw, 4, W - 6, 34], fill="#101010")        
                    dr.text((W - 10 - tw, 8), t, fill="#ff5555", font=fnt)                  
            fr, dbv = self.spec_last    
            # послесвечение — тёмно-зелёным под живой линией  
            if self.spec_hold is not None and len(self.spec_hold) == len(dbv):  
                hpts = [(float(f / fmax * W),    
                         float(np.clip(-d, 0, 80) / 80.0 * plot_h))    
                        for f, d in zip(fr, self.spec_hold)]  
                if len(hpts) >= 2:  
                    dr.line(hpts, fill="#0a5a0a", width=2)  
            pts = [(float(f / fmax * W),      
                    float(np.clip(-d, 0, 80) / 80.0 * plot_h))      
                   for f, d in zip(fr, dbv)]    
            if len(pts) >= 2:    
                dr.line(pts, fill="#00ff00", width=3)    
            py = np.clip(-self.spec_peak, 0, 80) / 80.0 * plot_h      
            x = 0    
            while x < W:   # горизонтальный пунктир    
                dr.line([(x, py), (min(x + 8, W), py)], fill="#8a8a20", width=2)      
                x += 14    
            t1 = f"{self.spec_peak:.0f} дБ"      
            t2 = f"{self.spec_peak_f:.0f} Гц"      
            tw = max(dr.textlength(t1, font=fnt),    
                     dr.textlength(t2, font=fnt))    
            dr.rectangle([W - 18 - tw, 40, W - 6, 102], fill="#101010")      
            dr.text((W - 10 - dr.textlength(t1, font=fnt), 44),        
                    t1, fill="#ffff00", font=fnt)        
            dr.text((W - 10 - dr.textlength(t2, font=fnt), 76),          
                    t2, fill="#ffff00", font=fnt)           
            # Подписи сетки — последними, поверх всех линий, на подложках  
            for dbv in range(-10, -80, -10):      
                y = -dbv / 80.0 * plot_h      
                t = f"{dbv} дБ"        
                tw2 = dr.textlength(t, font=fnt_sm)  
                dr.rectangle([4, y, 12 + tw2, y + 24], fill="#101010")  
                dr.text((6, y + 2), t, fill="#888888", font=fnt_sm)  
            for fk in range(1000, fmax, 1000):    
                x = fk / fmax * W    
                t = f"{fk // 1000} кГц"    
                tw2 = dr.textlength(t, font=fnt_sm)    
                dr.rectangle([x + 2, plot_h + 4, x + 8 + tw2, plot_h + 30],    
                             fill="#101010")    
                dr.text((x + 4, plot_h + 6), t, fill="#888888", font=fnt_sm)  
            # SNR — слева вверху под подписью полосы, шрифтом маркеров (fnt)      
            if self.snr_avg is not None:          
                t = f"SNR {self.snr_avg:.0f} дБ"          
                tw2 = dr.textlength(t, font=fnt)          
                dr.rectangle([W - 18 - tw2, 108, W - 6, 138], fill="#101010")          
                dr.text((W - 10 - tw2, 112), t, fill="#00ccff", font=fnt)             
            # THD / SINAD — вторым столбцом, как на экране  
            if self.ts_disp is not None:  
                t1 = f"THD {self.ts_disp[0]:.0f} дБ"  
                t2 = f"SINAD {self.ts_disp[1]:.0f} дБ"  
                tw = max(dr.textlength(t1, font=fnt),  
                         dr.textlength(t2, font=fnt))  
                dr.rectangle([W - 18 - tw, 144, W - 6, 206], fill="#101010")      
                dr.text((W - 10 - dr.textlength(t1, font=fnt), 148),      
                        t1, fill="#00ccff", font=fnt)      
                dr.text((W - 10 - dr.textlength(t2, font=fnt), 180),      
                        t2, fill="#00ccff", font=fnt)          
            # Частоты режимов — справа внизу, над подписями сетки  
            if self.mode_vars.get("rtty") and self.mode_vars["rtty"].get():  
                i1 = int(np.argmax(dbv))  
                far = np.abs(fr - fr[i1]) >= 100.0  
                if far.any():  
                    i2 = int(np.argmax(np.where(far, dbv, -80.0)))    
                    f_mark, f_space = max(fr[i1], fr[i2]), min(fr[i1], fr[i2])    
                    if self.ms_avg is not None:    
                        f_mark, f_space = self.ms_avg   # сглаженные, как на экране    
                    t1 = f"M {f_mark:.0f} Гц"      
                    t2 = f"S {f_space:.0f} Гц"    
                    tw = max(dr.textlength(t1, font=fnt),    
                             dr.textlength(t2, font=fnt))    
                    dr.rectangle([W - 16 - tw, plot_h - 64, W - 6, plot_h - 4],    
                                 fill="#101010")    
                    dr.text((W - 10 - dr.textlength(t1, font=fnt),    
                             plot_h - 60), t1, fill="#ffaa00", font=fnt)    
                    dr.text((W - 10 - dr.textlength(t2, font=fnt),    
                             plot_h - 32), t2, fill="#ffaa00", font=fnt)    
            elif self.mode_vars.get("cw") and self.mode_vars["cw"].get():    
                t = f"CW {self.spec_peak_f:.0f} Гц"    
                tw2 = dr.textlength(t, font=fnt)    
                dr.rectangle([W - 16 - tw2, plot_h - 36, W - 6, plot_h - 4],    
                             fill="#101010")    
                dr.text((W - 10 - tw2, plot_h - 32), t,    
                        fill="#ffaa00", font=fnt)    
            try:      
                img.save(fname + ".png")      
                self._log(f"# скрин: {fname}.png ({W}x{H})")  
            except Exception as e:  
                self._log(f"# скрин не удался: {e}")  
            return  
        # Fallback: postscript виджета (векторная графика).  
        try:  
            self.spec_cv.update()  
            self.spec_cv.postscript(file=fname + ".ps", colormode="color")  
            self._log(f"# скрин: {fname}.ps (postscript; для PNG нужен Pillow)")  
        except tk.TclError as e:  
            self._log(f"# скрин не удался: {e}")  

    def _on_close(self):        
        # Освободить аудио и порт, иначе процесс может повиснуть.        
        self._rec_stop()        
        self._audio_stop()      
        self.w.close()        
        self.destroy()        

    def _connect(self):  
        if self.w.ser:          
            self.w.close()            
            self._rec_stop()        
            self._audio_stop()      
            self.btn_conn["text"] = "Подключить"      
            self.status_var.set("не подключено")    
            return    
        port = self.port_var.get()  
        if not port:  
            messagebox.showerror("Порт", "Выберите COM-порт")  
            return  
        try:  
            self.w.open(port)  
        except Exception as ex:  
            messagebox.showerror("Порт", str(ex))  
            return  
        self.btn_conn["text"] = "Отключить"  
        self.status_var.set(f"{port}: handshake…")  
        self._pending = ["version", "params dump", "state dump"]  
        self._send_next()  
  
    def _send_next(self):  
        if self._pending:  
            cmd = self._pending.pop(0)  
            self._log("> " + cmd)  
            self.w.send(cmd)  
  
    def _cmd(self, cmd, redump=False):    
        self._log("> " + cmd)    
        self.w.send(cmd)    
        if redump:    
            # Ответ на load — "# OK ...", не фрейм: _send_next по нему не сработает.    
            # Ставим дампы в очередь и запускаем её сами — иначе зависнут.    
            self._pending += ["params dump", "state dump"]    
            self.after(300, self._send_next)   # дать плате ответить на load    
  
    def _send_manual(self):    
        cmd = self.cmd_var.get().strip()    
        if cmd:    
            self._cmd(cmd)    
            if not self.cmd_hist or self.cmd_hist[-1] != cmd:  # без дублей подряд  
                self.cmd_hist.append(cmd)  
                del self.cmd_hist[:-100]                       # не более 100  
            self.cmd_hist_i = len(self.cmd_hist)  
            self.cmd_var.set("")    
  
    def _cmd_hist_move(self, d):  
        # Стрелки вверх/вниз по истории команд; Down за последней — пустая строка.  
        if not self.cmd_hist:  
            return "break"  
        self.cmd_hist_i = min(max(self.cmd_hist_i + d, 0), len(self.cmd_hist))  
        self.cmd_var.set(self.cmd_hist[self.cmd_hist_i]  
                         if self.cmd_hist_i < len(self.cmd_hist) else "")  
        self.cmd_entry.icursor("end")   # курсор в конец подставленной строки  
        return "break"  
  
    # --- rx processing --------------------------------------------------------  
    def _poll_rx(self):  
        try:  
            while True:  
                item = self.w.rxq.get_nowait()  
                if item[0] == "__disconnected__":  
                    self.status_var.set("порт потерян")  
                    self.btn_conn["text"] = "Подключить"  
                    continue  
                self._on_line(item[1])  
        except queue.Empty:  
            pass  
        self.after(50, self._poll_rx)  
  
    def _on_line(self, line):  
        self._log(line)  
  
        # --- params dump frame ---  
        if line.startswith("# params begin"):  
            self._dump_active = True  
            self._dump_lines = []  
            self._dump_got = 0  
            self._dump_count = -1  
            if "count=" in line:  
                self._dump_count = int(line.split("count=")[1])  
            return  
        if self._dump_active:  
            if line.startswith("# params end"):  
                self._dump_active = False  
                if self._dump_count >= 0 and self._dump_got != self._dump_count:  
                    self._log(f"# дамп обрезан: {self._dump_got}/{self._dump_count} — повтор")  
                    self.w.send("params dump")  
                else:  
                    self._build_params(self._dump_lines)  
                    self._send_next()  
                return  
            if line.startswith("P|"):  
                self._dump_got += 1  
            self._dump_lines.append(line)  
            return  
  
        # --- state dump frame ---  
        if line.startswith("# state begin"):  
            self._state_active = True  
            self._state_lines = []  
            return  
        if self._state_active:  
            if line.startswith("# state end"):  
                self._state_active = False  
                self._show_state(self._state_lines)  
                self._send_next()  
                return  
            self._state_lines.append(line)  
            return  
  
        # --- version ---  
        if line.startswith("# version:"):  
            self._on_version(line)  
            self._send_next()  
            return  
  
        # --- ответ на set: обновить cur в таблице ---  
        if line.startswith("# OK ") and "=" in line:  
            self._refresh_param(line[5:])  
  
    def _on_version(self, line):  
        # "# version: RP2040_audio_dsp 2026.11.24 (schema=1 store=11)"  
        try:  
            schema = int(line.split("schema=")[1].split()[0].rstrip(")"))  
        except Exception:  
            schema = -1  
        self.schema_ok = (schema == SCHEMA_SUPPORTED)  
        ver = line.split(":", 1)[1].strip()  
        if self.schema_ok:  
            self.status_var.set(f"подключено: {ver}")  
        else:  
            self.status_var.set(f"{ver} — schema={schema}, read-only")  
            messagebox.showwarning(  
                "Версия",  
                f"schema={schema}, поддерживается {SCHEMA_SUPPORTED}.\n"  
                "Панель в режиме read-only.")  
  
    # --- params table ---------------------------------------------------------  
    def _build_params(self, lines):      
        for cf in self.columns:    
            for wdg in cf.winfo_children():      
                wdg.destroy()      
        self.params.clear()      
    
        # Разобрать дамп: label -> (type, cur)    
        parsed = {}    
        for ln in lines:    
            if ln.startswith("P|"):    
                _, label, ptype, cur, _b = (ln.split("|") + ["", "", "", ""])[:5]    
                parsed[label] = (ptype, cur)    
    
        # Раскладка по рамкам; нераспределённые — в "Прочее".  
        # Рамки pack'уются в свою колонку сверху вниз — без "дыр" grid'а.  
        used = set()      
        for title, en_label, members, col in GROUPS:      
            fr = ttk.LabelFrame(self.columns[col], text=title)      
            r = 0      
            # Флажок включения блока — первой строкой      
            if en_label and en_label in parsed:      
                self._add_param(fr, r, en_label, *parsed[en_label])      
                used.add(en_label)      
                r += 1      
            for label in members:        
                if label == en_label or label not in parsed:        
                    continue        
                self._add_param(fr, r, label, *parsed[label])        
                used.add(label)        
                r += 1        
            if title == "Автонотч NT":  
                # Ручной нотч — runtime-флажок рядом с автонотчем  
                # (не банковый параметр: команда rxnotch on|off немедленно,  
                # состояние из state dump R|rxnotch|).  
                var = tk.IntVar(value=0)  
                ttk.Checkbutton(  
                    fr, text="rxnotch", variable=var,  
                    command=lambda v=var: self._mode_set("rxnotch", v.get())  
                ).grid(row=r, column=0, columnspan=2, sticky="w", padx=4)  
                self.mode_vars["rxnotch"] = var  
                r += 1  
            if title == "Прочее":               
                # Нераспределённые метки — одним столбцом, чтобы не растягивать    
                # колонку по ширине (двухстолбцовая раскладка ломала рамки выше).    
                for label in parsed:    
                    if label not in used:    
                        self._add_param(fr, r, label, *parsed[label])    
                        used.add(label)    
                        r += 1    
            if r == 0:      
                fr.destroy()          # пустая рамка — не показывать      
                continue      
            fr.pack(fill="x", padx=2, pady=3)      
    
    def _add_param(self, parent, row, label, ptype, cur, col0=0):      
        ttk.Label(parent, text=label).grid(row=row, column=col0, sticky="w", padx=4)      
        if ptype == "onoff":      
            var = tk.IntVar(value=int(cur))      
            ttk.Checkbutton(      
                parent, variable=var,      
                command=lambda l=label, v=var: self._set(l, v.get())      
            ).grid(row=row, column=col0 + 1, sticky="w")      
        else:      
            var = tk.StringVar(value=cur)      
            e = ttk.Entry(parent, textvariable=var, width=9)      
            e.grid(row=row, column=col0 + 1, sticky="w")      
            e.bind("<Return>",      
                   lambda _e, l=label, v=var: self._set(l, v.get()))      
            e.bind("<FocusOut>",      
                   lambda _e, l=label, v=var: self._set(l, v.get()))      
        hint = {"x100": "x100", "gain": "Q8", "u32": "u32"}.get(ptype, "")      
        if hint:      
            ttk.Label(parent, text=hint, foreground="#888").grid(      
                row=row, column=col0 + 2, sticky="w")      
        self.params[label] = {"type": ptype, "var": var}      
  
    def _set(self, label, val):    
        if not self.schema_ok:    
            self._log(f"# read-only: {label} не отправлено")    
            return    
        # onoff-параметры: прошивка принимает только on|off, не 0|1  
        # (cli_param_try в af_cli.cpp возвращает false на числе ->  
        # строка уходит в cli_exec и даёт ERR).  
        p = self.params.get(label)  
        if p and p["type"] == "onoff":  
            val = "on" if str(val) in ("1", "on") else "off"  
        self._cmd(f"{label} {val}")    
  
    def _refresh_param(self, ok_text):  
        # "# OK band lo=100" -> обновить var, если метка совпала  
        if "=" not in ok_text:  
            return  
        label, val = ok_text.rsplit("=", 1)  
        label = label.strip()  
        val = val.strip().split()[0]  
        p = self.params.get(label)  
        if p:  
            try:  
                p["var"].set(int(float(val)))  
            except ValueError:  
                p["var"].set(val)  
  
    def _mode_set(self, cmd, val):    
        # Флажок runtime-режима: команда немедленно, в банк не пишется.    
        if self._mode_updating:    
            return                      # изменение пришло из дампа, не от пользователя    
        if not self.schema_ok:    
            self._log(f"# read-only: {cmd} не отправлено")    
            return    
        self._cmd(f"{cmd} {'on' if val else 'off'}")    
        # Перечитать живое состояние — плата могла отклонить (nr при nt on и т.п.)  
        self._pending.append("state dump")  
        self.after(300, self._send_next)  
  
    def _show_state(self, lines):    
        parts = []    
        for ln in lines:    
            if ln.startswith("R|"):    
                _, name, val = (ln.split("|") + ["", ""])[:3]    
                parts.append(f"{name}={val}")  
                # Синхронизировать флажки рамки "Режимы" с живым состоянием  
                if name in self.mode_vars:  
                    self._mode_updating = True  
                    try:  
                        self.mode_vars[name].set(1 if val == "1" else 0)  
                    finally:  
                        self._mode_updating = False  
        self.state_var.set("  ".join(parts))    
  
  
if __name__ == "__main__":    
    Panel().mainloop()  
