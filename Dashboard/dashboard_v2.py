import customtkinter as ctk
import tkinter as tk
from tkinter import messagebox
import threading
import socket
import serial
import serial.tools.list_ports
import time
import requests
from PIL import Image, ImageTk
from io import BytesIO
import os

# Configuration
DEFAULT_IP = "192.168.0.7"

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

class RobotController:
    def __init__(self):
        self.ser = None
        self.connected = False
        self.lock = threading.Lock()
        self.running = False
        self.on_telemetry_callback = None

    def connect(self, port, baud=115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.connected = True
            self.running = True
            threading.Thread(target=self.read_loop, daemon=True).start()
            return True
        except Exception as e:
            print(f"Connection Error: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.ser:
            self.ser.close()
            self.connected = False

    def send_command(self, cmd):
        if self.connected and self.ser:
            with self.lock:
                try:
                    self.ser.write((cmd + "\n").encode())
                    print(f"TX: {cmd}")
                except Exception as e:
                    print(f"TX Error: {e}")

    def read_loop(self):
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("STATUS:"):
                        # STATUS:cm,pL,pR,armIdx,usF,usB,usL,usR,ind
                        parts = line.split(":")[1].split(",")
                        if len(parts) >= 9:
                            data = {
                                "cm": parts[0], "pL": parts[1], "pR": parts[2],
                                "arm": parts[3],
                                "usF": parts[4], "usB": parts[5], "usL": parts[6], "usR": parts[7],
                                "ind": parts[8]
                            }
                            if self.on_telemetry_callback:
                                self.on_telemetry_callback(data)
                    elif line and not line.startswith("DEBUG"): # Filter junk
                        print(f"RX: {line}")
            except Exception as e:
                print(f"RX Error: {e}")
                time.sleep(1)

class MJPEGViewer(ctk.CTkLabel):
    def __init__(self, master, url, **kwargs):
        super().__init__(master, text="Waiting for Stream...", **kwargs)
        self.url = url
        self.running = False
        self.thread = None
        self.frame_count = 0
        
    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.update_stream)
            self.thread.daemon = True
            self.thread.start()
            
    def stop(self):
        self.running = False

    def update_stream(self):
        print(f"Connecting to {self.url}...")
        while self.running:
            try:
                stream = requests.get(self.url, stream=True, timeout=5)
                if stream.status_code == 200:
                    print("Stream Connected!")
                    bytes_data = bytes()
                    for chunk in stream.iter_content(chunk_size=1024):
                        if not self.running: break
                        bytes_data += chunk
                        a = bytes_data.find(b'\xff\xd8') # JPEG Start
                        b = bytes_data.find(b'\xff\xd9') # JPEG End
                        if a != -1 and b != -1:
                            jpg = bytes_data[a:b+2]
                            bytes_data = bytes_data[b+2:] 
                            try:
                                image = Image.open(BytesIO(jpg))
                                # Resize to fit label
                                photo = ctk.CTkImage(image.resize((320, 240)), size=(320, 240))
                                self.configure(image=photo, text="")
                            except Exception as e:
                                print(f"Frame Decode Error: {e}")
                                pass
                else:
                    self.configure(text=f"Status: {stream.status_code}")
                    time.sleep(1)
            except Exception as e:
                print(f"Stream Error: {e}")
                import traceback
                traceback.print_exc()
                self.configure(text=f"Reconnecting... ({e})")
                time.sleep(1) # Wait before retry

class DashboardApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("WSC 2026 Mission Control")
        self.geometry("1000x700")
        
        self.robot = RobotController()
        
        # Layout
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        
        # Sidebar
        self.sidebar = ctk.CTkFrame(self, width=200, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        
        self.logo_label = ctk.CTkLabel(self.sidebar, text="WSC ROBOT", font=ctk.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))
        
        # Connection Box
        self.conn_frame = ctk.CTkFrame(self.sidebar)
        self.conn_frame.grid(row=1, column=0, padx=10, pady=10)
        self.refresh_ports()
        
        self.btn_connect = ctk.CTkButton(self.conn_frame, text="Connect BLE/UART", command=self.toggle_connection)
        self.btn_connect.pack(pady=5)
        
        # IP Address Input
        ctk.CTkLabel(self.sidebar, text="Nicla Vision IP:", font=ctk.CTkFont(size=12, weight="bold")).grid(row=2, column=0, padx=20, pady=(10, 0))
        self.entry_ip = ctk.CTkEntry(self.sidebar)
        self.entry_ip.grid(row=3, column=0, padx=20, pady=(0, 20))
        self.entry_ip.insert(0, DEFAULT_IP)
        
        # Tabs
        self.tabview = ctk.CTkTabview(self)
        self.tabview.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        
        self.tab_calib = self.tabview.add("Kalibracija")
        self.tab_manual = self.tabview.add("Učenje (Manual)")
        self.tab_auto = self.tabview.add("Autonomno")
        
        # State
        self.manual_speed = 100
        
        self.setup_calibration_tab()
        self.setup_manual_tab()
        self.setup_auto_tab()
        
        # Hooks
        self.robot.on_telemetry_callback = self.update_telemetry
        self.ui_updater()

        # Key bindings for Manual Drive
        self.bind("<KeyPress>", self.on_key_press)
        
    def show_image_popup(self, jpg_data):
        try:
            top = ctk.CTkToplevel(self)
            top.title("Snapshot")
            top.geometry("400x300")
            
            image = Image.open(BytesIO(jpg_data))
            photo = ctk.CTkImage(image, size=(320, 240))
            
            lbl = ctk.CTkLabel(top, text="", image=photo)
            lbl.pack(expand=True, fill="both")
        except Exception as e:
            print(f"Popup error: {e}")

    def refresh_ports(self):
        # Simple combo box for ports (Mockup)
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.combo_ports = ctk.CTkComboBox(self.conn_frame, values=ports if ports else ["No Ports"])
        self.combo_ports.pack(pady=5)

    def toggle_connection(self):
        if not self.robot.connected:
            port = self.combo_ports.get()
            if self.robot.connect(port):
                self.btn_connect.configure(text="Disconnect", fg_color="red")
            else:
                messagebox.showerror("Error", "Could not connect")
        else:
            self.robot.disconnect()
            self.btn_connect.configure(text="Connect", fg_color="blue")

    def setup_calibration_tab(self):
        # 3 Columns
        frame = self.tab_calib
        col1 = ctk.CTkFrame(frame)
        col1.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        col2 = ctk.CTkFrame(frame)
        col2.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        col3 = ctk.CTkFrame(frame)
        col3.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        
        # --- COLUMN 1: LOGIKA & MOTORI ---
        ctk.CTkLabel(col1, text="PID & Motori", font=("Arial", 16, "bold")).pack(pady=5)
        
        # PID
        self.entry_kp = self.create_input(col1, "Kp:")
        self.entry_ki = self.create_input(col1, "Ki:")
        self.entry_kd = self.create_input(col1, "Kd:")
        
        ctk.CTkLabel(col1, text="Motori Config", font=("Arial", 12, "bold")).pack(pady=(10,5))
        self.entry_pulses = self.create_input(col1, "Pulse/cm:")
        self.entry_pulses.insert(0, "40.0")
        self.entry_speed = self.create_input(col1, "Brzina:")
        self.entry_speed.insert(0, "100")
        
        ctk.CTkButton(col1, text="Spremi Config (PID/Motor)", command=self.save_config).pack(pady=10)

        # --- COLUMN 2: VIZIJA ---
        ctk.CTkLabel(col2, text="Kamera & Boje", font=("Arial", 16, "bold")).pack(pady=5)
        
        # Video Stream 
        self.video_viewer = MJPEGViewer(col2, "", width=320, height=240, fg_color="black")
        self.video_viewer.pack(pady=5)
        ctk.CTkButton(col2, text="Start Stream", command=self.start_video_stream).pack(pady=5)
        
        # Color Sliders
        ctk.CTkLabel(col2, text="Kalibracija Boja").pack()
        self.combo_obj = ctk.CTkComboBox(col2, values=["BOCA", "LIMENKA", "SPUZVA"])
        self.combo_obj.pack(pady=5)
        
        # LAB Inputs
        self.lab_vars = {}
        lab_labels = ["L Min", "L Max", "A Min", "A Max", "B Min", "B Max"]
        lab_defaults = [30, 100, -64, -8, -32, 32] 
        
        self.lab_frame = ctk.CTkFrame(col2)
        self.lab_frame.pack(pady=5)
        
        for i, label in enumerate(lab_labels):
            lbl = ctk.CTkLabel(self.lab_frame, text=label, width=40)
            lbl.grid(row=i//2, column=(i%2)*2, padx=2)
            entry = ctk.CTkEntry(self.lab_frame, width=50)
            entry.grid(row=i//2, column=(i%2)*2+1, padx=2)
            entry.insert(0, str(lab_defaults[i]))
            self.lab_vars[label] = entry

        ctk.CTkButton(col2, text="Pošalji Boje (SET)", command=self.send_color).pack(pady=5)
        
        # --- COLUMN 3: ROBOTSKA RUKA ---
        ctk.CTkLabel(col3, text="Manipulator", font=("Arial", 16, "bold")).pack(pady=5)
        
        # Presets
        self.preset_names = [
            "Parkiraj", "Voznja", "Uzmi_Boca", "Uzmi_Limenka", "Uzmi_Spuzva",
            "Spremi_1", "Spremi_2", "Spremi_3",
            "Iz_Sprem_1", "Iz_Sprem_2", "Iz_Sprem_3",
            "Dostava_1", "Dostava_2", "Dostava_3", "Extra"
        ]
        self.combo_preset = ctk.CTkComboBox(col3, values=self.preset_names)
        self.combo_preset.pack(pady=5)
        
        row_preset = ctk.CTkFrame(col3)
        row_preset.pack(pady=5)
        ctk.CTkButton(row_preset, text="Učitaj Preset", width=80, command=self.load_preset).pack(side="left", padx=5)
        ctk.CTkButton(row_preset, text="Spremi Preset", width=80, fg_color="orange", command=self.save_preset).pack(side="left", padx=5)
        
        # Sliders for 6 Servos
        self.sliders = []
        self.servo_entries = [] # To update text on preset load
        servo_names = ["Baza", "Rame", "Lakat", "Zglob", "Rot", "Hvat"]
        
        for i in range(6):
            max_angle = 360 if i == 0 else 180
            self.create_servo_control(col3, i, servo_names[i], max_angle)

    def create_servo_control(self, parent, idx, name, max_angle=180):
        f = ctk.CTkFrame(parent)
        f.pack(fill="x", padx=5, pady=2)
        
        ctk.CTkLabel(f, text=f"{idx}-{name}", width=50).pack(side="left")
        
        entry = ctk.CTkEntry(f, width=50)
        entry.pack(side="right", padx=2)
        
        slider = ctk.CTkSlider(f, from_=0, to=max_angle, number_of_steps=max_angle)
        slider.set(90)
        entry.insert(0, "90")
        slider.pack(side="left", fill="x", expand=True, padx=5)
        
        # Callbacks
        def on_slide(val):
            entry.delete(0, "end")
            entry.insert(0, f"{int(val)}")
            self.send_servo(idx, val)
            
        def on_entry(event):
            try:
                val = float(entry.get())
                if 0 <= val <= max_angle:
                    slider.set(val)
                    self.send_servo(idx, val)
            except: pass

        slider.configure(command=on_slide)
        entry.bind("<Return>", on_entry)
        
        self.sliders.append(slider)
        self.servo_entries.append(entry)

    def send_servo(self, ch, val):
        self.robot.send_command(f"SERVO:{ch},{val:.1f}")
        
    def send_color(self):
        obj = self.combo_obj.get()
        vals = []
        labels = ["L Min", "L Max", "A Min", "A Max", "B Min", "B Max"]
        try:
            for l in labels:
                vals.append(self.lab_vars[l].get())
            
            # Construct: SET:TYPE,lmin,lmax,amin,amax,bmin,bmax
            cmd = f"SET:{obj},{','.join(vals)}"
            print(f"Sending Color Config: {cmd}")
            self.robot.send_command(cmd)
        except Exception as e:
            messagebox.showerror("Error", f"Invalid Values: {e}")

    def setup_manual_tab(self):
        frame = self.tab_manual
        
        # Grid layout: 2 Columns
        left_col = ctk.CTkFrame(frame)
        left_col.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        right_col = ctk.CTkFrame(frame)
        right_col.pack(side="right", fill="both", expand=True, padx=5, pady=5)
        
        # --- LEFT: Telemetry & Controls ---
        ctk.CTkLabel(left_col, text="Telemetrija Uživo", font=("Arial", 16, "bold")).pack(pady=5)
        
        # Telemetry Labels
        self.lbl_dist = ctk.CTkLabel(left_col, text="Distance: 0.0 cm")
        self.lbl_dist.pack()
        self.lbl_enc = ctk.CTkLabel(left_col, text="Enc: L=0 R=0")
        self.lbl_enc.pack()
        self.lbl_arm = ctk.CTkLabel(left_col, text="Arm Preset: -")
        self.lbl_arm.pack()
        
        ctk.CTkLabel(left_col, text="--- Senzori ---").pack(pady=(5,0))
        self.lbl_us_fb = ctk.CTkLabel(left_col, text="Front: 0 | Back: 0")
        self.lbl_us_fb.pack()
        self.lbl_us_lr = ctk.CTkLabel(left_col, text="Left: 0 | Right: 0")
        self.lbl_us_lr.pack()
        self.lbl_induct = ctk.CTkLabel(left_col, text="Inductive: False", text_color="gray")
        self.lbl_induct.pack()
        
        ctk.CTkLabel(left_col, text="--- Kontrole ---").pack(pady=(10,5))
        
        # Speed Control
        f_spd = ctk.CTkFrame(left_col)
        f_spd.pack(fill="x", padx=5, pady=2)
        ctk.CTkLabel(f_spd, text="Brzina (+/-):").pack(side="left")
        self.entry_man_speed = ctk.CTkEntry(f_spd, width=60)
        self.entry_man_speed.pack(side="right", padx=5)
        self.entry_man_speed.insert(0, str(self.manual_speed))
        
        # Arm Preset Selector for Manual Tab
        ctk.CTkLabel(left_col, text="Ruka:").pack()
        self.man_preset_combo = ctk.CTkComboBox(left_col, values=self.preset_names)
        self.man_preset_combo.pack(pady=2)
        ctk.CTkButton(left_col, text="Izvrši & Snimi Ruku", fg_color="blue", command=self.exec_record_arm).pack(pady=5)
        
        # Snapshot Button
        ctk.CTkButton(left_col, text="📸 Uslikaj Sliku", command=self.take_snapshot).pack(pady=10)
        self.lbl_snapshot_status = ctk.CTkLabel(left_col, text="")
        self.lbl_snapshot_status.pack()

        # --- RIGHT: Mission Recorder ---
        ctk.CTkLabel(right_col, text="Snimač Misije (Numpad)", font=("Arial", 14)).pack(pady=5)
        instr = "8/2/4/6 = Vozi | Space = Waypoint | Ruka = Gumb"
        ctk.CTkLabel(right_col, text=instr, font=("Arial", 10)).pack()
        
        self.waypoints_list = ctk.CTkTextbox(right_col, width=400, height=300)
        self.waypoints_list.pack(pady=10, fill="both", expand=True)
        self.waypoints_list.insert("1.0", "--- Čekanje Poteza ---\n")
        self.waypoints_list.configure(state="disabled") # Read only

        ctk.CTkButton(right_col, text="Spremi Misiju (misija.txt)", command=self.save_mission).pack(pady=5)

    def update_telemetry(self, data):
        # Callback from thread, careful with GUI updates. 
        # CTK is mostly thread safe but best to use after? 
        # Actually CTK/Tkinter is NOT thread safe. We must queue.
        # Quick hack: direct update usually fails or crashes eventually.
        # Setup 'after' loop in main thread is better.
        # But let's try direct update protected by try-except for now or use a variable.
        self.latest_telemetry = data

    def ui_updater(self):
        if hasattr(self, 'latest_telemetry') and self.latest_telemetry:
            d = self.latest_telemetry
            # Update Labels
            self.lbl_dist.configure(text=f"Distance: {d['cm']} cm")
            self.lbl_enc.configure(text=f"Enc: L={d['pL']} R={d['pR']}")
            self.lbl_arm.configure(text=f"Arm Preset: {d['arm']}")
            self.lbl_us_fb.configure(text=f"Front: {d['usF']} | Back: {d['usB']}")
            self.lbl_us_lr.configure(text=f"Left: {d['usL']} | Right: {d['usR']}")
            
            # Auto Tab Updates
            if hasattr(self, 'lbl_auto_dist'):
                self.lbl_auto_dist.configure(text=f"Distance: {d['cm']} cm | Enc: {d['pL']}/{d['pR']}")
                self.lbl_auto_sensors.configure(text=f"US: F={d['usF']} B={d['usB']} L={d['usL']} R={d['usR']} | I={d['ind']}")
            
        self.after(100, self.ui_updater)

    def exec_record_arm(self):
        preset = self.man_preset_combo.get()
        # 1. Execute
        try:
             idx = self.preset_names.index(preset)
             self.robot.send_command(f"LOAD_PRESET:{idx}")
             # 2. Record
             self.log_mission_step(f"ARM:{preset}")
        except: pass

    def take_snapshot(self):
        # Fetch single frame
        ip = self.entry_ip.get()
        url = f"http://{ip}:8080" # Root returns MJPEG stream usually...
        # Nicla script sends MJPEG. We can just open, read one frame, close.
        threading.Thread(target=self._snapshot_thread, args=(url,), daemon=True).start()
        
    def _snapshot_thread(self, url):
        try:
            self.lbl_snapshot_status.configure(text="Fetching...")
            stream = requests.get(url, stream=True, timeout=2)
            if stream.status_code == 200:
                bytes_data = bytes()
                for chunk in stream.iter_content(chunk_size=1024):
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')
                    b = bytes_data.find(b'\xff\xd9')
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        # Show in popup
                        self.show_image_popup(jpg)
                        break
            self.lbl_snapshot_status.configure(text="Done")
        except Exception as e:
            self.lbl_snapshot_status.configure(text="Error")
            print(e)
            
    def show_image_popup(self, jpg_data):
        # We need to run this in main thread? 
        # CTK Toplevel
        pass # To be implemented or handled via queue
        
    def log_mission_step(self, text):
        self.waypoints_list.configure(state="normal")
        self.waypoints_list.insert("end", text + "\n")
        self.waypoints_list.see("end")
        self.waypoints_list.configure(state="disabled")

    def setup_auto_tab(self):
        frame = self.tab_auto
        ctk.CTkButton(frame, text="Učitaj Misiju", command=self.load_mission).pack(pady=10)
        ctk.CTkButton(frame, text="START MISIJA", fg_color="green", height=50, command=self.start_mission).pack(pady=10)
        self.log_box = ctk.CTkTextbox(frame, width=500, height=300)
        self.log_box.pack(pady=10)

    def create_input(self, parent, label):
        f = ctk.CTkFrame(parent, fg_color="transparent")
        f.pack(fill="x", padx=10, pady=2)
        ctk.CTkLabel(f, text=label, width=30).pack(side="left")
        e = ctk.CTkEntry(f)
        e.pack(side="right", expand=True, fill="x")
        return e

    # Logic
    def start_video_stream(self):
        ip = self.entry_ip.get()
        url = f"http://{ip}:8080"
        self.video_viewer.url = url
        self.video_viewer.start()

    def save_config(self):
        # 1. PID
        kp = self.entry_kp.get()
        ki = self.entry_ki.get()
        kd = self.entry_kd.get()
        self.robot.send_command(f"PID:{kp},{ki},{kd}")
        time.sleep(0.1)
        
        # 2. Motor
        puls = self.entry_pulses.get()
        spd = self.entry_speed.get()
        self.robot.send_command(f"SET_MOTOR:{puls},{spd}")
        


    def save_preset(self):
        name = self.combo_preset.get()
        try:
             idx = self.preset_names.index(name)
             self.robot.send_command(f"SAVE_PRESET:{idx}")
        except ValueError:
             messagebox.showerror("Error", "Unknown Preset Name")

    def load_preset(self):
        name = self.combo_preset.get()
        try:
             idx = self.preset_names.index(name)
             self.robot.send_command(f"LOAD_PRESET:{idx}")
        except ValueError:
             messagebox.showerror("Error", "Unknown Preset Name")




    def start_mission(self):
        self.robot.send_command("MODE:AUTO")

    def on_key_press(self, event):
        # Numpad logic
        if self.tabview.get() != "Učenje (Manual)": return
        
        key = event.keysym
        speed = 100
        cmd = ""
        
        if key == 'KP_Up' or key == '8': cmd = f"MAN:FWD,{speed}"
        elif key == 'KP_Down' or key == '2': cmd = f"MAN:BCK,{speed}"
        elif key == 'KP_Left' or key == '4': cmd = f"MAN:LFT,{speed}"
        elif key == 'KP_Right' or key == '6': cmd = f"MAN:RGT,{speed}"
        elif key == 'space': 
            self.waypoints_list.insert("end", f"Waypoint Saved\n")
            return
            
        if cmd: self.robot.send_command(cmd)

    def save_mission(self):
        try:
            with open("misija.txt", "w", encoding="utf-8") as f:
                f.write(self.waypoints_list.get("1.0", "end"))
                messagebox.showinfo("Saved", "Mission saved to misija.txt")
        except Exception as e:
            messagebox.showerror("Error", f"Could not save: {e}")

    def load_mission(self):
        try:
            with open("misija.txt", "r", encoding="utf-8") as f:
                content = f.read()
                self.log_box.delete("1.0", "end")
                self.log_box.insert("end", content)
        except:
            messagebox.showerror("Error", "No mission file found")

    def setup_auto_tab(self):
        frame = self.tab_auto
        
        # Layout: Left (Stream/Telemetry) vs Right (Mission Control)
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        
        left_col = ctk.CTkFrame(frame)
        left_col.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        right_col = ctk.CTkFrame(frame)
        right_col.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        # --- LEFT: Stream & Telemetry ---
        ctk.CTkLabel(left_col, text="Kamera Uživo", font=("Arial", 16, "bold")).pack(pady=5)
        
        # MJPEG Viewer
        # We need the IP. But IP might change? We bind it dynamically or reload?
        # For now, create it, but URL set on Connect/Update.
        self.stream_viewer = MJPEGViewer(left_col, url=f"http://{DEFAULT_IP}:8080", width=400, height=300)
        self.stream_viewer.pack(pady=5)
        ctk.CTkButton(left_col, text="Osvježi Stream (IP)", command=self.refresh_stream).pack(pady=2)

        ctk.CTkLabel(left_col, text="--- Telemetrija ---").pack(pady=10)
        # Duplicate labels for Auto Tab (or reuse? Tkinter widgets can't be in two places)
        # We create new labels and update both in ui_updater
        self.lbl_auto_dist = ctk.CTkLabel(left_col, text="Distance: 0.0 cm")
        self.lbl_auto_dist.pack()
        self.lbl_auto_sensors = ctk.CTkLabel(left_col, text="Sensors: -")
        self.lbl_auto_sensors.pack()
        self.lbl_auto_status = ctk.CTkLabel(left_col, text="Robot Status: IDLE", text_color="orange")
        self.lbl_auto_status.pack(pady=5)

        # --- RIGHT: Mission Control ---
        ctk.CTkLabel(right_col, text="Učitana Misija", font=("Arial", 16)).pack(pady=5)
        
        self.log_box = ctk.CTkTextbox(right_col, width=400, height=300)
        self.log_box.pack(pady=5, fill="both", expand=True)
        
        btn_row = ctk.CTkFrame(right_col, fg_color="transparent")
        btn_row.pack(pady=10)
        
        ctk.CTkButton(btn_row, text="Učitaj (misija.txt)", command=self.load_mission).pack(side="left", padx=5)
        ctk.CTkButton(btn_row, text="POKRENI MISIJU", fg_color="green", command=self.start_mission).pack(side="left", padx=5)
        ctk.CTkButton(btn_row, text="STOP", fg_color="red", command=self.stop_mission).pack(side="left", padx=5)

        self.mission_running = False

    def refresh_stream(self):
         ip = self.entry_ip.get()
         self.stream_viewer.url = f"http://{ip}:8080"
         self.stream_viewer.running = False # Restart logic? 
         # MJPEGViewer wrapper resets if URL changes? Need to verify class.
         # Assuming simple restart:
         self.stream_viewer.stop()
         self.stream_viewer.start()

    def start_mission(self):
        if self.mission_running: return
        self.mission_running = True
        threading.Thread(target=self.run_mission_thread, daemon=True).start()

    def stop_mission(self):
        self.mission_running = False
        self.robot.send_command("MAN:STOP") # or STANI
        self.lbl_auto_status.configure(text="STATUS: STOPPED", text_color="red")

    def run_mission_thread(self):
        self.lbl_auto_status.configure(text="STATUS: RUNNING", text_color="green")
        lines = self.log_box.get("1.0", "end").splitlines()
        
        for i, line in enumerate(lines):
            if not self.mission_running: break
            line = line.strip()
            if not line or line.startswith("---") or line.startswith("#"): continue
            
            # Highlight current line (Simulated by selecting?)
            # self.log_box.tag_add("current", ...) - too complex for threading.
            print(f"Executing: {line}")
            self.lbl_auto_status.configure(text=f"Exec: {line}")
            
            parts = line.split(":") 
            cmd_type = parts[0]
            
            if cmd_type == "MOVE":
                target = int(float(parts[1])) # Handle "50.0"
                cmd = f"MOVE:{target}" # Robot logic expects MOVE:cm
                self.robot.send_command(cmd)
                
                # Wait for completion (Telemetry monitoring)
                # Logic: Wait for cm to resets (0), then wait for cm >= target-tolerance
                time.sleep(0.5) # Give it time to reset
                start_time = time.time()
                while self.mission_running:
                    # Timeout safety
                    if time.time() - start_time > 10: break 
                    
                    try:
                        curr = float(self.latest_telemetry.get('cm', 0))
                        # Check if reached (allow 2cm error)
                        if abs(curr) >= abs(target) - 2:
                            break
                    except: pass
                    time.sleep(0.1)
                
                # Stop explicitly? MOVE logic stops itself.
                time.sleep(0.5) # Settle

            elif cmd_type == "ARM":
                preset_name = parts[1] # "Uzmi Bocu"
                # Need to map back to index? Or usage "LOAD_PRESET:idx" in recorder?
                # Recorder wrote "ARM:Uzmi Bocu". 
                # We need mapping.
                try:
                    idx = self.preset_names.index(preset_name)
                    self.robot.send_command(f"LOAD_PRESET:{idx}")
                    time.sleep(2.0) # Robot arm movement time
                except:
                    print(f"Unknown Preset: {preset_name}")
            
            elif cmd_type == "WAIT":
                ms = int(parts[1])
                time.sleep(ms / 1000.0)

        self.mission_running = False
        self.lbl_auto_status.configure(text="STATUS: DONE", text_color="blue")

if __name__ == "__main__":
    app = DashboardApp()
    app.mainloop()
