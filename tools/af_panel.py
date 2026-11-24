#!/usr/bin/env python3  
# tools/af_panel.py — панель настроек RP2040_audio_dsp.  
# Зависимости: pip install pyserial. GUI — tkinter (стандартная поставка).  
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
import tkinter as tk  
from tkinter import ttk, messagebox  
  
try:  
    import serial  
    from serial.tools import list_ports  
except ImportError:  
    raise SystemExit("Нужен pyserial: pip install pyserial")  
  
SCHEMA_SUPPORTED = 1  
BAUD = 115200  
DUMP_TIMEOUT_S = 10.0  
  
  
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
        self.title("RP2040 audio DSP — панель настроек")  
        self.geometry("980x640")  
  
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
  
        self._build_top()  
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
        ttk.Label(bf, textvariable=self.state_var).pack(side="right")  
  
    def _build_table(self):  
        wrap = ttk.Frame(self)  
        wrap.pack(fill="both", expand=True, padx=6, pady=4)  
        self.canvas = tk.Canvas(wrap, highlightthickness=0)  
        sb = ttk.Scrollbar(wrap, orient="vertical", command=self.canvas.yview)  
        self.tbl = ttk.Frame(self.canvas)  
        self.tbl.bind("<Configure>",  
                      lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))  
        self.canvas.create_window((0, 0), window=self.tbl, anchor="nw")  
        self.canvas.configure(yscrollcommand=sb.set)  
        self.canvas.pack(side="left", fill="both", expand=True)  
        sb.pack(side="right", fill="y")  
        # Колесо — только над таблицей; bind_all перехватывал бы и лог.    
        self.canvas.bind("<Enter>",    
            lambda _e: self.canvas.bind_all("<MouseWheel>",    
                lambda e: self.canvas.yview_scroll(-e.delta // 120, "units")))    
        self.canvas.bind("<Leave>",    
            lambda _e: self.canvas.unbind_all("<MouseWheel>"))    
  
    def _build_bottom(self):  
        f = ttk.Frame(self)  
        f.pack(fill="x", padx=6, pady=4)  
        self.cmd_var = tk.StringVar()  
        e = ttk.Entry(f, textvariable=self.cmd_var)  
        e.pack(side="left", fill="x", expand=True)  
        e.bind("<Return>", lambda _e: self._send_manual())  
        ttk.Button(f, text="Отправить", command=self._send_manual).pack(side="left", padx=4)  
        self.log = tk.Text(self, height=8, state="disabled")  
        self.log.pack(fill="x", padx=6, pady=(0, 6))  
  
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
  
    def _connect(self):  
        if self.w.ser:  
            self.w.close()  
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
        for wdg in self.tbl.winfo_children():  
            wdg.destroy()  
        self.params.clear()  
  
        row = 0  
        for ln in lines:  
            if ln.startswith("# S|"):  
                ttk.Label(self.tbl, text=ln[4:],  
                          font=("TkDefaultFont", 10, "bold")).grid(  
                    row=row, column=0, columnspan=3, sticky="w", pady=(8, 2))  
                row += 1  
                continue  
            if not ln.startswith("P|"):  
                continue  
            _, label, ptype, cur, banked = (ln.split("|") + ["", "", "", ""])[:5]  
            ttk.Label(self.tbl, text=label).grid(row=row, column=0, sticky="w")  
  
            if ptype == "onoff":  
                var = tk.IntVar(value=int(cur))  
                wdg = ttk.Checkbutton(  
                    self.tbl, variable=var,  
                    command=lambda l=label, v=var: self._set(l, v.get()))  
                wdg.grid(row=row, column=1, sticky="w")  
            else:  
                var = tk.StringVar(value=cur)  
                e = ttk.Entry(self.tbl, textvariable=var, width=10)  
                e.grid(row=row, column=1, sticky="w")  
                e.bind("<Return>",  
                       lambda _e, l=label, v=var: self._set(l, v.get()))  
                e.bind("<FocusOut>",  
                       lambda _e, l=label, v=var: self._set(l, v.get()))  
            note = {"x100": "x100", "gain": "Q8", "u32": "u32"}.get(ptype, "")  
            ttk.Label(self.tbl, text=note, foreground="#888").grid(  
                row=row, column=2, sticky="w", padx=4)  
            self.params[label] = {"type": ptype, "var": var}  
            row += 1  
  
    def _set(self, label, val):  
        if not self.schema_ok:  
            self._log(f"# read-only: {label} не отправлено")  
            return  
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
  
    def _show_state(self, lines):  
        parts = []  
        for ln in lines:  
            if ln.startswith("R|"):  
                _, name, val = (ln.split("|") + ["", ""])[:3]  
                parts.append(f"{name}={val}")  
        self.state_var.set("  ".join(parts))  
  
  
if __name__ == "__main__":    
    Panel().mainloop()  
