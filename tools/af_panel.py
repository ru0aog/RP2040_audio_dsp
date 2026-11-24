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
  
SCHEMA_SUPPORTED = 1    
BAUD = 115200    
DUMP_TIMEOUT_S = 10.0    
  
# Группировка параметров по блокам аудиотракта.  
# (заголовок рамки, метка-флажок включения или None, [метки параметров]).  
# Порядок рамок = порядок в списке; метки не из списка попадут в "Прочее".  
# (заголовок рамки, метка-флажок или None, [метки], колонка 0..3).  
# Колонки 0-1 = RX, 2-3 = TX; рамки внутри колонки pack'уются сверху вниз.  
GROUPS = [    
    ("Полоса приёма", None,    
     ["band lo", "band hi"], 0),    
    ("АРУ / лимитер", "agc",    
     ["gain", "agc ref", "agc max", "agc min", "lim ceil"], 0),    
    ("CW-фильтр / декодер", "cw",    
     ["cw f0", "cw bw", "cw noise", "cw dec", "cw wpm", "cw sql"], 0),    
    ("RX shelf / exciter", None,    
     ["rx shelf", "rx shelf f", "rx shelf q", "rx exc", "rx excmix"], 0),    
    ("Шумоподавление NR", "nr",    
     ["nr depth", "nr makeup", "nr mu"], 1),    
    ("Автонотч NT", "nt",    
     ["nt mu"], 1),    
    ("RTTY", "rtty",    
     ["rtty scan", "rtty rsid", "rtty rev", "rtty dbg", "rtty ctr", "rtty sql"], 1),    
    ("RX presence", "rx pres",    
     ["rx presmax", "rx pres f", "rx pres q"], 1),    
    ("RX EQ (3 полосы)", None,    
     ["rxeq lo", "rxeq mid", "rxeq hi", "rxeq lo f", "rxeq mid f", "rxeq hi f"], 1),    
    ("TX полоса / АРУ", "tx",    
     ["tx band lo", "tx band hi", "tx agc", "tx agcmax",    
      "tx gate", "tx gatehold", "tx gatehyst", "tx gfloor"], 2),    
    ("TX компрессор", "tx 2band",    
     ["tx split", "tx lr2", "tx cthr", "tx cthr lo", "tx cthr hi",    
      "tx soft", "tx ap"], 2),    
    ("TX дакинг / разборчивость", "tx duck",    
     ["tx duckmax", "tx intel", "tx intel tgt", "tx intel cut", "tx intel bst"], 2),    
    ("TX presence / shelf / exciter", "tx pres",    
     ["tx presmax", "tx pres f", "tx pres q",    
      "tx shelf", "tx shelf f", "tx shelf q", "tx exc", "tx excmix"], 3),    
    ("TX нотчи", "tx notch",    
     ["tx notch f1", "tx notch f2"], 3),    
    ("TX EQ (3 полосы)", None,      
     ["txeq lo", "txeq mid", "txeq hi", "txeq lo f", "txeq mid f", "txeq hi f"], 3),      
    ("Прочее RX", None,        
     ["voice vol", "nb", "nb thr"], 0),          
    ("Прочее TX", None,      
     ["tx clip", "tx comp", "tx agc max", "tx agc tgt",      
      "tx gate hys", "tx gate hld"], 3),      
    ("Прочее", None, [], 2),   # нераспределённые метки — в колонку 3 (под TX дакинг)        
]        
  
# Runtime-режимы: НЕ банковые, команда "<name> on|off" выполняется немедленно.  
# Состояние читается из state dump (R|name|val). Порядок = порядок в рамке.  
# (имя в state dump R|name|, команда CLI). Имя и команда различаются,  
# где метка AF_PARAMS содержит пробел ("rx pres" -> R|rxpres|).  
RUNTIME_MODES = [    
    ("nt",      "nt"),    
    ("nr",      "nr"),    
    ("bypass",  "bypass"),    
    ("cw",      "cw"),    
    ("nb",      "nb"),    
    ("rxnotch", "rxnotch"),    
    ("rxpres",  "rx pres"),    
    ("rxexc",   "rx exc"),    
    ("rtty",    "rtty"),    
    ("ramp",    "ramp"),    
    ("tx",      "tx"),    
    ("tt",      "tt"),    
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
        self.geometry("1130x800")    
  
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
        self.rec_file = None      # wave.Wave_write    
        self.rec_lock = threading.Lock()    
        self.spec_q = queue.Queue(maxsize=8)   # блоки float32 для FFT    
        self.spec_fs = 44100    
        self.spec_on = False        
        self.spec_buf = None    
        self.spec_peak = -100.0     # peak-hold уровень, дБ  
        self.spec_peak_f = 0.0      # частота последнего пика, Гц  
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
  
    # --- layout -------------------------------------------------------------  
    def _build_top(self):  
        f = ttk.Frame(self)  
        f.pack(fill="x", padx=6, pady=4)  
        ttk.Label(f, text="Порт:").pack(side="left")  
        self.port_var = tk.StringVar()  
        self.port_cb = ttk.Combobox(f, textvariable=self.port_var, width=12,  
                                    values=self._ports())  
        self.port_cb.pack(side="left", padx=4)  
        ttk.Button(f, text="Обновить", command=self._refresh_ports).pack(side="left")  
        self.btn_conn = ttk.Button(f, text="Подключить", command=self._connect)  
        self.btn_conn.pack(side="left", padx=8)  
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
            ).grid(row=i // 6, column=i % 6, sticky="w", padx=6, pady=1)      
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
        if sd is None:      
            self.audio_cb.configure(state="disabled")      
            self.audio_btn.configure(state="disabled")      
            self.audio_scale.configure(state="disabled")    
            self.rec_btn.configure(state="disabled")  
        mf.columnconfigure(6, weight=1)   # прижать аудио-блок вправо      
  
    def _build_table(self):    
        wrap = ttk.Frame(self.paned)    
        self.paned.add(wrap, weight=3)    
        self.canvas = tk.Canvas(wrap, highlightthickness=0)    
        sb = ttk.Scrollbar(wrap, orient="vertical", command=self.canvas.yview)    
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
        right = ttk.Frame(self.paned)      
        self.paned.add(right, weight=1)      
        # Спектр входа UAC: текущий срез FFT 0..6 кГц / 0..-100 дБ,  
        # маркеры band lo/hi. Работает, пока включён монитор.  
        sf = ttk.LabelFrame(right, text="Спектр (0-6 кГц)")    
        sf.pack(fill="x", padx=(0, 6), pady=(4, 0))    
        self.spec_cv = tk.Canvas(sf, height=280, bg="#101010",    
                                 highlightthickness=0)    
        self.spec_cv.pack(fill="x", padx=4, pady=4)    
        # height ограничивает высоту; bg — светло-серый фон лога.    
        self.log = tk.Text(right, width=42, height=14, state="disabled",    
                           bg="#f0f0f0")            
        self.log.pack(fill="both", expand=True, padx=(0, 6), pady=4)        
        f = ttk.Frame(right)      
        f.pack(fill="x", padx=(0, 6), pady=(0, 2))      
        self.cmd_var = tk.StringVar()      
        e = ttk.Entry(f, textvariable=self.cmd_var)      
        e.pack(side="left", fill="x", expand=True)      
        e.bind("<Return>", lambda _e: self._send_manual())      
        ttk.Button(f, text="Отправить", command=self._send_manual).pack(side="left", padx=4)           
        ttk.Button(f, text="Очистить", command=self._log_clear).pack(side="left")    
        ttk.Button(f, text="Скопировать", command=self._log_copy).pack(side="left", padx=(4, 0))    
  
    # --- helpers ------------------------------------------------------------  
    def _ports(self):  
        return [p.device for p in list_ports.comports()]  
  
    def _refresh_ports(self):  
        self.port_cb["values"] = self._ports()  
  
    def _log(self, s):    
        self.log.configure(state="normal")    
        self.log.insert("end", s + "\n")    
        self.log.see("end")    
        self.log.configure(state="disabled")    
    
    def _log_clear(self):    
        self.log.configure(state="normal")    
        self.log.delete("1.0", "end")    
        self.log.configure(state="disabled")    
    
    def _log_copy(self):    
        # Весь лог в буфер обмена (Text в disabled — читаем напрямую).    
        self.clipboard_clear()    
        self.clipboard_append(self.log.get("1.0", "end-1c"))    
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
        self.spec_peak = -100.0  
        self.spec_peak_f = 0.0  
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
  
            self.rec_file = wf            # до стрима: callback увидит файл сразу,  
                                          # а _rec_stop закроет его при ошибке  
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
        self._log(f"# rec: -> {fname} <- {sel}")  
  
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
                    self.rec_file.close()  
                except Exception:  
                    pass  
                self.rec_file = None  
        self.rec_btn.configure(text="Запись")  

    SPEC_NFFT = 4096   # окно FFT;  ~10.8 Гц/бин при fs=44100 
  
    def _spec_draw(self):    
        # Перерисовка ~12 раз/с: сетка дБ/Гц + маркеры ПФ + полилиния спектра.  
        # Фиксированные диапазоны: 0..6 кГц по X, 0..-100 дБ по Y.  
        if not self.spec_on:    
            return    
        cv = self.spec_cv      
        # Блоки из in_cb идут по fs//50 (~882 при 44100) — меньше SPEC_NFFT.  
        # Сливаем всё из очереди в накопитель и берём последние SPEC_NFFT.  
        try:      
            while True:  
                blk = self.spec_q.get_nowait()  
                self.spec_buf = blk if self.spec_buf is None \
                    else np.concatenate((self.spec_buf, blk))  
        except queue.Empty:      
            pass  
        if self.spec_buf is not None and len(self.spec_buf) > 4 * self.SPEC_NFFT:  
            self.spec_buf = self.spec_buf[-2 * self.SPEC_NFFT:]  # не растим хвост  
        cv.delete("all")    
        w = max(cv.winfo_width(), 100)    
        h = max(cv.winfo_height(), 60)    
        fmax = 6000    
        plot_h = h - 14                     # нижние 14 px — подписи частот    
        for db in (-20, -40, -60, -80):     # сетка уровней, 0 дБ сверху    
            y = -db / 100.0 * plot_h    
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
        if band_on:  
            for lbl in ("band lo", "band hi"):  # маркеры полосы ПФ      
                p = self.params.get(lbl)      
                if p:      
                    try:      
                        fx = float(p["var"].get())      
                        if 0 < fx < fmax:      
                            x = fx / fmax * w      
                            cv.create_line(x, 0, x, plot_h,      
                                           fill="#ff5555", dash=(3, 2))      
                    except (ValueError, tk.TclError):      
                        pass      
        if self.spec_buf is not None and len(self.spec_buf) >= self.SPEC_NFFT:      
            win = np.hanning(self.SPEC_NFFT)      
            mag = np.abs(np.fft.rfft(self.spec_buf[-self.SPEC_NFFT:] * win))      
            db = 20 * np.log10(mag / (self.SPEC_NFFT / 2) + 1e-9)    
            freqs = np.fft.rfftfreq(self.SPEC_NFFT, 1.0 / self.spec_fs)      
            m = (freqs >= 50) & (freqs <= fmax)   # DC и <50 Гц не рисуем  
            pts = []    
            for x, y in zip(freqs[m] / fmax * w,    
                            np.clip(-db[m], 0, 100) / 100.0 * plot_h):    
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
                self.spec_peak = max(-100.0, self.spec_peak - 0.5)  
            py = np.clip(-self.spec_peak, 0, 100) / 100.0 * plot_h  
            cv.create_line(0, py, w, py, fill="#ffff00", dash=(2, 4))  
            cv.create_text(w - 4, 4, text=f"{self.spec_peak:.0f} dB",  
                           fill="#ffff00", anchor="ne",  
                           font=("TkDefaultFont", 8))  
            cv.create_text(w - 4, 18, text=f"{self.spec_peak_f:.0f} Гц",  
                           fill="#ffff00", anchor="ne",  
                           font=("TkDefaultFont", 8))  
        self.after(80, self._spec_draw)      

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
            self.cmd_var.set("")  
  
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
