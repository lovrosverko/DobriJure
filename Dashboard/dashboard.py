import customtkinter as ctk
import asyncio
import threading
import json
import math
import time
from bleak import BleakScanner, BleakClient
from PIL import Image, ImageTk, ImageDraw

# --- Konfiguracija ---
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

# UUID-ovi za HC-02 (ISSC) - Preuzeto iz postojeceg koda
UART_RX_CHAR_UUID = "49535343-1e4d-4bd9-ba61-23c647249616" # Notify
UART_TX_CHAR_UUID = "49535343-8841-43f4-a8d4-ecbe34729bb3" # Write

ROBOT_IMAGE_PATH = "slike/tlocrtRobota.png"

class App(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("WSC 2026 - Mission Control")
        self.geometry("1400x900")

        # --- Podatkovni Model ---
        self.data_lock = threading.Lock()
        self.data = {
            "enc_l": 0, "enc_r": 0, "spd": 0.0, "head": 0.0,
            "bat": 0.0, "status": "Ready", "tof": 0
        }
        
        # --- Misija i Učenje ---
        self.misija_koraci = [] # Lista rjecnika {"cmd": ..., "val": ...}
        self.ucenje_active = False
        self.last_pose = {"x": 0, "y": 0, "theta": 0}

        # --- BLE Infrastruktura ---
        self.ble_client = None
        self.loop = None
        self.is_connected = False
        self.rx_buffer = ""
        
        # Pokretanje Async Loop-a u zasebnom threadu
        self.thread = threading.Thread(target=self.start_async_loop, daemon=True)
        self.thread.start()

        # --- Inicijalizacija UI ---
        self.setup_ui()
        
        # --- UI Update Loop ---
        self.update_ui_loop()

        # --- Key Bindings for Teach-in ---
        self.bind("<KeyPress-w>", lambda e: self.send_manual("w"))
        self.bind("<KeyPress-s>", lambda e: self.send_manual("s"))
        self.bind("<KeyPress-a>", lambda e: self.send_manual("a"))
        self.bind("<KeyPress-d>", lambda e: self.send_manual("d"))
        self.bind("<KeyRelease-w>", lambda e: self.send_manual("stop"))
        self.bind("<KeyRelease-s>", lambda e: self.send_manual("stop"))
        self.bind("<KeyRelease-a>", lambda e: self.send_manual("stop"))
        self.bind("<KeyRelease-d>", lambda e: self.send_manual("stop"))
        self.bind("<space>", lambda e: self.snapshot_pose())

    def setup_ui(self):
        # Glavni TabView
        self.tabview = ctk.CTkTabview(self)
        self.tabview.pack(fill="both", expand=True, padx=10, pady=10)

        self.tab1 = self.tabview.add("Nadzorna Ploča")
        self.tab2 = self.tabview.add("Podešavanje")
        self.tab3 = self.tabview.add("Planer Misije")
        self.tab4 = self.tabview.add("Učenje")

        self.setup_tab1()
        self.setup_tab2()
        self.setup_tab3()
        self.setup_tab4()

    # ---------------- TAB 1: NADZORNA PLOČA ----------------
    def setup_tab1(self):
        self.tab1.grid_columnconfigure(0, weight=1) # Lijevo (Kompas)
        self.tab1.grid_columnconfigure(1, weight=2) # Sredina (Kamera)
        self.tab1.grid_columnconfigure(2, weight=1) # Desno (Kontrole)
        self.tab1.grid_rowconfigure(0, weight=1)

        # -- Lijevo: Kompas i Status --
        frame_left = ctk.CTkFrame(self.tab1)
        frame_left.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        ctk.CTkLabel(frame_left, text="ORIJENTACIJA", font=("Arial", 16, "bold")).pack(pady=10)
        self.compass_canvas = ctk.CTkCanvas(frame_left, width=200, height=200, bg="#2b2b2b", highlightthickness=0)
        self.compass_canvas.pack(pady=10)
        
        self.lbl_head = ctk.CTkLabel(frame_left, text="SMJER: 0.0°", font=("Arial", 18))
        self.lbl_head.pack(pady=5)
        
        ctk.CTkLabel(frame_left, text="TELEMETRIJA", font=("Arial", 16, "bold")).pack(pady=(30, 10))
        self.lbl_enc = ctk.CTkLabel(frame_left, text="Enc: L:0 | R:0")
        self.lbl_enc.pack()
        self.lbl_spd = ctk.CTkLabel(frame_left, text="Brzina: 0.0 m/s")
        self.lbl_spd.pack()

        # -- Sredina: Kamera --
        frame_center = ctk.CTkFrame(self.tab1)
        frame_center.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        ctk.CTkLabel(frame_center, text="KAMERA UŽIVO", font=("Arial", 16, "bold")).pack(pady=10)
        
        # Placeholder frame
        self.camera_frame = ctk.CTkFrame(frame_center, fg_color="black")
        self.camera_frame.pack(fill="both", expand=True, padx=10, pady=10)
        ctk.CTkLabel(self.camera_frame, text="NEMA VIDEO SIGNALA", text_color="gray").place(relx=0.5, rely=0.5, anchor="center")

        # -- Desno: Kontrole Veze --
        frame_right = ctk.CTkFrame(self.tab1)
        frame_right.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        
        ctk.CTkLabel(frame_right, text="KONTROLA ROBOTA", font=("Arial", 16, "bold")).pack(pady=10)
        
        self.lbl_connection = ctk.CTkLabel(frame_right, text="NIJE POVEZANO", text_color="red", font=("Arial", 14, "bold"))
        self.lbl_connection.pack(pady=10)
        
        ctk.CTkButton(frame_right, text="POVEŽI (HC-02)", command=self.trigger_connect).pack(pady=10)
        
        ctk.CTkButton(frame_right, text="START", fg_color="green", height=60, font=("Arial", 20, "bold"), 
                      command=lambda: self.send_json({"cmd": "start"})).pack(pady=(40, 10), fill="x", padx=20)
                      
        ctk.CTkButton(frame_right, text="STOP", fg_color="red", height=60, font=("Arial", 20, "bold"), 
                      command=lambda: self.send_json({"cmd": "stop"})).pack(pady=10, fill="x", padx=20)

    # ---------------- TAB 2: PODEŠAVANJE ----------------
    def setup_tab2(self):
        frame_pid = ctk.CTkFrame(self.tab2)
        frame_pid.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkLabel(frame_pid, text="PID Konfiguracija", font=("Arial", 14, "bold")).pack(pady=5)
        
        # Grid za unose
        grid = ctk.CTkFrame(frame_pid, fg_color="transparent")
        grid.pack(pady=5)
        
        ctk.CTkLabel(grid, text="Kp").grid(row=0, column=0, padx=5)
        ctk.CTkLabel(grid, text="Ki").grid(row=0, column=1, padx=5)
        ctk.CTkLabel(grid, text="Kd").grid(row=0, column=2, padx=5)
        
        self.entry_kp = ctk.CTkEntry(grid, width=60); self.entry_kp.grid(row=1, column=0, padx=5)
        self.entry_ki = ctk.CTkEntry(grid, width=60); self.entry_ki.grid(row=1, column=1, padx=5)
        self.entry_kd = ctk.CTkEntry(grid, width=60); self.entry_kd.grid(row=1, column=2, padx=5)
        
        ctk.CTkButton(frame_pid, text="Pošalji PID", command=self.send_pid).pack(pady=10)

        # Ostale postavke
        frame_tools = ctk.CTkFrame(self.tab2)
        frame_tools.pack(fill="x", padx=10, pady=10)
        
        ctk.CTkButton(frame_tools, text="Kalibriraj IMU", command=lambda: self.send_json({"cmd": "cal_imu"})).pack(side="left", padx=10, pady=10)
        ctk.CTkButton(frame_tools, text="Spremi u EEPROM", command=lambda: self.send_json({"cmd": "save_eeprom"})).pack(side="left", padx=10, pady=10)

    # ---------------- TAB 3: PLANER MISIJE ----------------
    def setup_tab3(self):
        # Lijevo: Lista misije
        self.scroll_misija = ctk.CTkScrollableFrame(self.tab3, label_text="Lista Koraka")
        self.scroll_misija.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        
        # Desno: Dodavanje
        frame_controls = ctk.CTkFrame(self.tab3)
        frame_controls.pack(side="right", fill="y", padx=10, pady=10)
        
        ctk.CTkLabel(frame_controls, text="Dodaj Korak", font=("Arial", 14, "bold")).pack(pady=10)
        
        self.combo_cmd = ctk.CTkOptionMenu(frame_controls, values=["VOZI", "ROTIRAJ", "RUKA", "PAUZA"])
        self.combo_cmd.pack(pady=5)
        
        self.entry_val = ctk.CTkEntry(frame_controls, placeholder_text="Vrijednost (cm/stup)")
        self.entry_val.pack(pady=5)
        
        ctk.CTkButton(frame_controls, text="Dodaj", command=self.add_mission_step).pack(pady=10)
        ctk.CTkButton(frame_controls, text="Obriši Zadnji", command=self.remove_last_step, fg_color="orange").pack(pady=5)
        
        ctk.CTkButton(frame_controls, text="UPLOAD MISIJE", command=self.upload_mission, fg_color="blue", height=40).pack(pady=(30, 5))

    # ---------------- TAB 4: UČENJE ----------------
    def setup_tab4(self):
        info_frame = ctk.CTkFrame(self.tab4)
        info_frame.pack(fill="both", expand=True, padx=20, pady=20)
        
        ctk.CTkLabel(info_frame, text="NAČIN RADA: UČENJE", font=("Arial", 24, "bold")).pack(pady=20)
        
        ctk.CTkLabel(info_frame, text="Koristi WASD tipke za vožnju robota.", font=("Arial", 16)).pack(pady=10)
        ctk.CTkLabel(info_frame, text="Pritisni SPACE za spremanje trenutne 'Ključne Točke' (Pose Snapshot).", font=("Arial", 16)).pack(pady=10)
        
        self.lbl_ucenje_status = ctk.CTkLabel(info_frame, text="Zadnja točka: Nema", text_color="yellow", font=("Arial", 14))
        self.lbl_ucenje_status.pack(pady=20)
        
        # Novi gumbi za Eye-in-Hand
        ctk.CTkButton(info_frame, text="Testiraj Pozu Vožnja", fg_color="purple", 
                      command=lambda: self.send_json({"cmd": "ruka", "val": "voznja"})).pack(pady=10)
        
        self.lbl_tof = ctk.CTkLabel(info_frame, text="TOF Udaljenost: 0 mm", font=("Arial", 16, "bold"))
        self.lbl_tof.pack(pady=5)

        ctk.CTkButton(info_frame, text="Resetiraj Poziciju Robota", command=lambda: self.send_json({"cmd": "reset_pose"})).pack(pady=10)

    # ---------------- LOGIKA ----------------
    def send_pid(self):
        try:
            p = float(self.entry_kp.get())
            i = float(self.entry_ki.get())
            d = float(self.entry_kd.get())
            self.send_json({"cmd": "set_pid", "p": p, "i": i, "d": d})
        except ValueError:
            print("Neispravne PID vrijednosti!")

    def add_mission_step(self):
        cmd = self.combo_cmd.get()
        val = self.entry_val.get()
        
        mapping = {"VOZI": "straight", "ROTIRAJ": "turn", "RUKA": "arm", "PAUZA": "wait"}
        
        step_text = f"{len(self.misija_koraci)+1}. {cmd} {val}"
        lbl = ctk.CTkLabel(self.scroll_misija, text=step_text, anchor="w")
        lbl.pack(fill="x", padx=5)
        
        self.misija_koraci.append({"cmd": mapping[cmd], "val": val})

    def remove_last_step(self):
        if self.misija_koraci:
            self.misija_koraci.pop()
            widgets = self.scroll_misija.winfo_children()
            if widgets:
                widgets[-1].destroy()

    def upload_mission(self):
        # Šalje cijelu listu (ili korak po korak, ovisno o arduinu, ovdje šaljemo niz)
        # Arduino vjerojatno nema buffer za cijelu misiju odjednom ako je dugacka.
        # Za sada saljemo kao posebnu komandu "mission_load" pa niz.
        # Ovdje pojednostavljeno:
        print("Upload misije:", json.dumps(self.misija_koraci))
        for step in self.misija_koraci:
            self.send_json(step)
            time.sleep(0.1) # Mali delay da se ne pretrpa buffer

    def send_manual(self, key):
        # Šaljemo samo ako smo u Tab 4
        if self.tabview.get() != "Učenje":
            return
            
        cmd_map = {
            "w": {"l": 100, "r": 100},
            "s": {"l": -100, "r": -100},
            "a": {"l": -80, "r": 80},
            "d": {"l": 80, "r": -80},
            "stop": {"l": 0, "r": 0}
        }
        
        target = cmd_map.get(key)
        if target:
            self.send_json({"cmd": "manual", "l": target["l"], "r": target["r"]})

    def snapshot_pose(self):
        if self.tabview.get() == "Učenje":
            self.send_json({"cmd": "get_pose"})
            self.lbl_ucenje_status.configure(text="Zahtjev za poziciju poslan...", text_color="orange")

    # ---------------- BLE & THREADING ----------------
    def start_async_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def trigger_connect(self):
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.connect_ble(), self.loop)

    async def connect_ble(self):
        self.update_status("Skeniram...", "orange")
        devices = await BleakScanner.discover()
        target = None
        for d in devices:
            if d.name and "HC-02" in d.name:
                target = d
                break
        
        if not target:
            self.update_status("HC-02 nije pronađen", "red")
            return

        self.update_status("Povezivanje...", "orange")
        self.ble_client = BleakClient(target.address)
        
        try:
            await self.ble_client.connect()
            self.is_connected = True
            self.update_status("POVEZANO", "green")
            await self.ble_client.start_notify(UART_RX_CHAR_UUID, self.notification_handler)
        except Exception as e:
            self.is_connected = False
            self.update_status(f"Greška: {e}", "red")

    def notification_handler(self, sender, data):
        try:
            chunk = data.decode('utf-8', errors='ignore')
            self.rx_buffer += chunk
            
            if "\n" in self.rx_buffer:
                lines = self.rx_buffer.split("\n")
                
                for line in lines[:-1]:
                    line = line.strip()
                    if line.startswith("{") and line.endswith("}"):
                        self.process_json(line)
                            
                self.rx_buffer = lines[-1]
                
        except Exception as e:
            print(f"RX Error: {e}")

    def process_json(self, json_str):
        try:
            js = json.loads(json_str)
            
            # Ako je snapshot odgovor
            if "pose_x" in js:
                self.handle_snapshot(js)
            
            # Ažuriraj glavni model
            with self.data_lock:
                self.data.update(js)
                
        except json.JSONDecodeError:
            pass

    def handle_snapshot(self, js):
        x = js.get("pose_x", 0)
        y = js.get("pose_y", 0)
        th = js.get("pose_th", 0)
        
        info = f"Spremljeno: X:{x:.1f} Y:{y:.1f} Th:{th:.1f}°"
        # Update UI thread-safe way via variable or sync mechanism?
        # Tkinter handles variable updates loosely, but for safety lets update label in update_ui loop
        # For now, just print to status label via update_ui_loop check logic or direct configure (CTk is somewhat thread safe)
        self.lbl_ucenje_status.configure(text=info, text_color="green")
        
        # Ovdje bi se dodalo u misiju
        # ... logika za deltu ...

    def send_json(self, payload):
        if not self.is_connected: return
        json_str = json.dumps(payload)
        data = (json_str + "\n").encode('utf-8')
        asyncio.run_coroutine_threadsafe(self.write_ble(data), self.loop)

    async def write_ble(self, data):
        if self.ble_client and self.is_connected:
            try:
                await self.ble_client.write_gatt_char(UART_TX_CHAR_UUID, data)
                print(f"TX: {data}")
            except Exception as e:
                print(f"TX Fail: {e}")

    def update_status(self, text, color):
        self.lbl_connection.configure(text=text, text_color=color)

    def update_ui_loop(self):
        with self.data_lock:
            h = self.data.get('head', 0.0)
            spd = self.data.get('spd', 0.0)
            el = self.data.get('enc_l', 0)
            er = self.data.get('enc_r', 0)

        self.lbl_head.configure(text=f"SMJER: {h:.1f}°")
        self.lbl_spd.configure(text=f"Brzina: {spd:.2f} m/s")
        self.lbl_enc.configure(text=f"Enc: L:{el} | R:{er}")
        
        # Azuriranje TOF-a
        tof = self.data.get('tof', 0)
        if hasattr(self, 'lbl_tof'):
            self.lbl_tof.configure(text=f"TOF Udaljenost: {tof} mm")
        
        self.draw_compass(h)
        self.after(100, self.update_ui_loop)

    def draw_compass(self, head):
        self.compass_canvas.delete("all")
        w, h = 200, 200
        cx, cy = w/2, h/2
        r = 80
        self.compass_canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline="white", width=2)
        
        angle_rad = math.radians(head - 90)
        tx = cx + (r-10) * math.cos(angle_rad)
        ty = cy + (r-10) * math.sin(angle_rad)
        self.compass_canvas.create_line(cx, cy, tx, ty, fill="red", width=3, arrow="last")
        self.compass_canvas.create_text(cx, cy-r-15, text="N", fill="white")

if __name__ == "__main__":
    app = App()
    app.mainloop()
