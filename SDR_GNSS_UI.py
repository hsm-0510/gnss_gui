import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import subprocess, threading, re, shlex, os, time, socket, signal
import matplotlib.pyplot as plt
from PIL import Image, ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from queue import Queue, Empty
import serial.tools.list_ports
from ctypes import *
import ctypes as ct

libsdr = cdll.LoadLibrary('C:/Hasem/Work/Hasem/2025/Task 12 12_Jun PocketSDR Testing/PocketSDR/lib/win32/libsdr.so')

class ToolTip:
    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tip_window = None
        self.widget.bind("<Enter>", self.show_tip)
        self.widget.bind("<Leave>", self.hide_tip)

    def show_tip(self, event):
        if self.tip_window or not self.text:
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + 20
        self.tip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        label = tk.Label(tw, text=self.text, background="yellow", relief=tk.SOLID, borderwidth=1, foreground="black")
        label.pack()

    def hide_tip(self, event):
        if self.tip_window:
            self.tip_window.destroy()
            self.tip_window = None

class PocketSDRGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SDR Based GNSS Receiver")
        self.root.geometry("1280x720")
        self.root.minsize(800, 600)
        self.root.config(highlightbackground="#151b23")
        
        # Initialization
        self.update_interval = 500  # milliseconds
        self.ui_update_scheduled = False
        self.running = False
        self.nmea_running = False
        self.process = None
        self.log_file = None
        self.log_file_path = ""
        self.sat_data_buffer = {}
        self.nmea_socket = None
        self.output_queue = Queue()
        self.ser = None
        self.is_connected = False
        self.data = ""
        self.old_time = ""
        
        # IMPORTANT: Update this path to the actual location of pocket_trk.exe
#        self.base_path = "C:/Users/sting/Downloads/PocketSDR-master/PocketSDR-master/bin"
        self.base_path = os.path.dirname(os.path.realpath(__file__))        
        # Style configuration
        style = ttk.Style()
        style.theme_use('default')
        style.configure("TFrame", background="#dcdad5")
        style.configure("TButton", font=("Helvetica", 10, "bold"), background="#ccdaff", foreground="black")
        style.configure("Treeview", font=("Helvetica", 10), background="#151b23", foreground="black")
        style.configure("Treeview.Heading", font=("Helvetica", 12, "bold"), background="#ccdaff", foreground="black")
        style.configure("TLabelframe.Label", font=("Helvetica", 16, "bold"), background="#dcdad5", foreground="black")

        # Top frame for logo and heading
        self.top_frame = ttk.Frame(self.root, style="TFrame", padding="10")
        self.top_frame.grid(row=0, column=0, sticky="we")
        try:
            logo_path = os.path.join(self.base_path, "SRT_Logo.png")
            logo_image = Image.open(logo_path).resize((150, 150), Image.Resampling.LANCZOS)
            self.logo_photo = ImageTk.PhotoImage(logo_image)
            self.logo_label = ttk.Label(self.top_frame, image=self.logo_photo)
            self.logo_label.grid(row=0, column=0, padx=50, pady=10)
        except Exception as e:
            print(f"Logo loading failed: {e}")
        
        self.heading = ttk.Label(self.top_frame, text="SDR-BASED GNSS RECEIVER", font=("Helvetica", 50, "bold"), anchor='center')
        self.heading.grid(row=0, column=1, padx=10, pady=10, sticky="ew")
        self.top_frame.columnconfigure(1, weight=1)

        # Main frame
        self.main_frame = ttk.Frame(self.root, style="TFrame", padding="10")
        self.main_frame.grid(row=1, column=0, sticky="nsew")

        # Command frame
        self.cmd_frame = ttk.LabelFrame(self.main_frame, text="Controls", padding="10")
        self.cmd_frame.grid(row=0, column=0, columnspan=2, sticky="we", pady=5)
        
        self.start_button = ttk.Button(self.cmd_frame, text="Start", command=self.start_pocket_sdr)
        self.start_button.grid(row=0, column=0, padx=5, pady=5)
        
        self.stop_button = ttk.Button(self.cmd_frame, text="Stop", command=self.stop_pocket_sdr, state="disabled")
        self.stop_button.grid(row=0, column=1, padx=5, pady=5)
        
        self.log_button = ttk.Button(self.cmd_frame, text="Save Log File", command=self.select_log_file)
        self.log_button.grid(row=0, column=2, padx=5, pady=5)

        # Create the label for the dropdown
        self.dropdown_label = ttk.Label(self.cmd_frame, text="Transmit NMEA:", font=("Helvetica", 10, "bold"))
        self.dropdown_label.grid(row=0, column=3, padx=5, pady=5, sticky='e')  # 'e' aligns it to the right

        
        # Create the dropdown (combobox)
        self.dropdown = ttk.Combobox(self.cmd_frame, state='readonly')
        self.dropdown.set("Select COM Port...")
        self.dropdown.grid(row=0, column=4, padx=5, pady=5)
        self.dropdown.bind("<FocusIn>", self.update_dropdown)
        
        
        
        # Create the label for the dropdown
        self.hz_label = ttk.Label(self.cmd_frame, text="Output Rate (Hz):", font=("Helvetica", 10, "bold"))
        self.hz_label.grid(row=0, column=5, padx=5, pady=5, sticky='e')  # 'e' aligns it to the right

        # Create the dropdown (combobox)
        self.hz = ttk.Combobox(self.cmd_frame, state='readonly')
        self.hz.set("Select Output Rate...")
        self.hz.grid(row=0, column=6, padx=5, pady=5)
        self.hz.bind("<FocusIn>", self.update_hz)
        
        # Create the label for the dropdown
        self.baud_label = ttk.Label(self.cmd_frame, text="Baudrate:", font=("Helvetica", 10, "bold"))
        self.baud_label.grid(row=0, column=7, padx=5, pady=5, sticky='e')  # 'e' aligns it to the right

        # Create the dropdown (combobox)
        self.baud = ttk.Combobox(self.cmd_frame, state='readonly')
        self.baud.set("Select Baudrate...")
        self.baud.grid(row=0, column=8, padx=5, pady=5)
        self.baud.bind("<FocusIn>", self.update_baud)
        
        self.connect_button = ttk.Button(self.cmd_frame, text="Start Transmission", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=9, padx=5, pady=5)

        ToolTip(self.start_button, "Start the SDR process")
        ToolTip(self.stop_button, "Stop the SDR process")
        ToolTip(self.log_button, "Choose where to save satellite data")
        ToolTip(self.dropdown, "Choose which COM port to transmit NMEA")
        ToolTip(self.connect_button, "Connect/Disconnect COM port to transmit NMEA")
        ToolTip(self.hz, "Select NMEA output rate (Hz)")
        ToolTip(self.baud, "Select COM port baudrate")

        # Data frame
        self.data_frame = ttk.Frame(self.main_frame)
        self.data_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")

        # Status frame
        self.status_frame = ttk.LabelFrame(self.data_frame, text="Position & Status", padding="10", width=500)
        self.status_frame.grid(row=0, column=1, sticky="nws", padx=5, pady=5)
        self.status_frame.grid_propagate(False)
        
        labels = ["Time", "Latitude", "Longitude", "Altitude", "Fix", "Buffer", "Search", "Lock", "Velocity"]
        self.status_labels = {}
        for i, label in enumerate(labels):
            self.status_labels[label] = ttk.Label(self.status_frame, text=f"{label}: ", font=("Helvetica", 9, "bold"), width=67, background="#f0f0f0", foreground="black")
            self.status_labels[label].grid(row=i, column=0, sticky="w", pady=1)

        # Satellite table
        self.tree_frame = ttk.LabelFrame(self.data_frame, text="Satellite Data", padding="10")
        self.tree_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
#        columns = ("CH", "RF", "SAT", "SIG", "PRN", "LOCK(s)", "C/N0", "COFF(ms)", "DOP(Hz)", "ADR(cyc)", "SYNC", "NAV", "ERR", "LOL", "FEC")
        columns = ("CH", "SAT", "SIG", "PRN", "LOCK(s)", "C/N0", "COFF(ms)", "DOP(Hz)", "NAV",  "LOL", "FEC")
        self.tree = ttk.Treeview(self.tree_frame, columns=columns, show="headings", height=10)
        
        column_widths = [90, 90, 100, 100, 90, 120, 120, 140, 140, 140, 120, 100, 100, 100, 100]
        for col, width in zip(columns, column_widths):
            self.tree.heading(col, text=col)
            self.tree.column(col, width=width, anchor="center")
        self.tree.grid(row=0, column=0, sticky="nsew")
        
        self.tree.tag_configure("oddrow", background="#f9f9f9")
        self.tree.tag_configure("evenrow", background="#e6f3ff")
        
        scrollbar = ttk.Scrollbar(self.tree_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.grid(row=0, column=1, sticky="ns")
        
        self.initial_column_widths = column_widths
        self.initial_total_width = sum(column_widths)
        self.tree.bind("<Configure>", self.on_treeview_resize)

        # Plot frame
        self.plot_frame = ttk.LabelFrame(self.main_frame, text="C/N0 Plot", padding="10")
        self.plot_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=5)
        
        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Status bar
        self.status_bar = ttk.Label(self.root, text="Ready", relief="sunken", anchor="w")
        self.status_bar.grid(row=2, column=0, sticky="we")
        
        # Grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        self.main_frame.columnconfigure(0, weight=1)
        self.main_frame.rowconfigure(1, weight=2)
        self.main_frame.rowconfigure(2, weight=1)
        self.data_frame.columnconfigure(0, weight=2)
        self.data_frame.columnconfigure(1, weight=1)
        self.data_frame.rowconfigure(0, weight=1)
        self.tree_frame.columnconfigure(0, weight=1)
        self.tree_frame.rowconfigure(0, weight=1)

        # Menu
        menubar = tk.Menu(self.root)
        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="Save Log File", command=self.select_log_file)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.destroy)
        menubar.add_cascade(label="File", menu=file_menu)
        
        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(label="About", command=self.show_about)
        menubar.add_cascade(label="Help", menu=help_menu)
        self.root.config(menu=menubar)
        
    def update_baud(self, event=None):
        self.baud['values'] = ['9600', '19200', '38400', '57600', '115200']
    
    def toggle_connection(self):
        if self.is_connected:
            self.close_com_port()
            self.connect_button.config(text="Stopped")
        else:
            self.connect_com()
            self.connect_button.config(text="Transmitting")
        self.is_connected = not self.is_connected
    
    def close_com_port(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"COM port closed")
        else:
            print("No active COM port to close")
    
    def send_data_to_com(self, com_port, baud_rate, data, hz):
        try:
            with serial.Serial(com_port, int('9600'), timeout=1) as self.ser:
                while True:
                    print(f"COM: {com_port}; Baud: {baud_rate}; Freq: {hz}")
                    self.ser.write(data.encode('utf-8'))
                    if hz == '0.5':
                        time.sleep(2)
                    elif hz == '1':
                        time.sleep(1)
                    elif hz == '10':
                        time.sleep(0.1)
                    else:
                        time.sleep(1)
        except Exception as e:
            print(f"Error: {e}")
    
    def connect_com(self):
        selected_port = self.dropdown.get()
        selectedHz = self.hz.get()
        selectedBaud = self.baud.get()
        
        if selected_port == "No COM Ports Found":
            print("Please select a valid COM port.")
            return
        
        #nmea_data = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
        
        
#         self.transmit_thread = threading.Thread(target=self.send_data_to_com, args=(selected_port, selectedBaud, self.data, selectedHz), daemon=True)
#         self.transmit_thread.start()
        threading.Thread(target=self.send_data_to_com, args=(selected_port, selectedBaud, self.data, selectedHz), daemon=True).start()
        print(f"Started sending data to {selected_port}")
    
    def update_hz(self, event=None):
        self.hz['values'] = ['0.5', '1', '10']
        #self.hz.current(0)
    
    def update_dropdown(self, event=None):
        available_ports = self.find_com_ports()
        if available_ports:
            self.dropdown['values'] = available_ports
            #self.dropdown.current(0)
        else:
            self.dropdown['values'] = ['No COM Ports Found']
    
    def find_com_ports(self):
        # Get a list of available COM ports
        ports = serial.tools.list_ports.comports()

        # Return the device names of the available ports
        return [port.device for port in ports]
    
    def on_treeview_resize(self, event):
        total_width = self.tree.winfo_width() - 20
        for col, initial_width in zip(self.tree["columns"], self.initial_column_widths):
            new_width = max(50, int((initial_width / self.initial_total_width) * total_width))
            self.tree.column(col, width=new_width)

    def show_about(self):
        messagebox.showinfo("About", "SDR Based GNSS Receiver\nVersion 1.1\nDeveloped by StingRay Team")

    def select_log_file(self):
        self.log_file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Save Log File As"
        )
        if self.log_file_path:
            self.status_bar.config(text=f"Log file selected: {os.path.basename(self.log_file_path)}")
        else:
            self.status_bar.config(text="No log file selected")

    def validate_command(self, cmd):
        executable = cmd[0].lstrip("./").rstrip(".exe")
        for exe_name in [executable, f"{executable}.exe"]:
            full_path = os.path.join(self.base_path, exe_name)
            if os.path.isfile(full_path):
                cmd[0] = full_path
                return True, ""
        return False, f"Executable '{executable}' not found at {self.base_path}. Please verify the path."

    def start_pocket_sdr(self):
        if self.running:
            return

#        cmd = shlex.split("./pocket_trk -sig L1CA -prn 1-32 -nmea :4400")
        cmd = shlex.split("./pocket_trk -sig L1CA -prn 1-32 -sig G1CA -prn -7-6/1-27 -sig E1B -prn 1-36 -sig E5AI -prn 1-36 -sig E5BI -prn 1-36 -sig B1I -prn 1-63 -sig B1CD -prn 1-63 -sig B2AD -prn 1-63 -sig B2I -prn 1-63 -sig B2BI -prn 1-63 -sig B3I -prn 1-63 -nmea :4400")
        #cmd = shlex.split("./pocket_trk -sig L1CA -prn 1-32 -sig G1CA -prn -7-6/1-27 -sig E1B -prn 1-36 -sig B1I -prn 1-63 -nmea :4400")
        is_valid, error_msg = self.validate_command(cmd)
        if not is_valid:
            messagebox.showerror("Command Error", error_msg)
            self.status_bar.config(text=f"Error: {error_msg}")
            return

        if self.log_file_path:
            try:
                self.log_file = open(self.log_file_path, 'a', encoding='utf-8')
                self.log_file.write(f"--- Logging Started: {time.ctime()} ---\n")
                self.log_file.flush()
            except Exception as e:
                messagebox.showerror("Log File Error", f"Cannot open log file:\n{e}")
                self.log_file = None
                return

        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        self.clear_data()
        self.status_bar.config(text="Starting SDR process...")

        self.running = True
        self.nmea_running = True

        self.thread = threading.Thread(target=self.read_pocket_sdr, args=(cmd,), daemon=True)
        self.nmea_thread = threading.Thread(target=self.read_nmea_data, daemon=True)
        self.thread.start()
        self.nmea_thread.start()

        self.root.after(self.update_interval, self.process_queue)


    def stop_pocket_sdr(self):
        if not self.running:
            return
        self.running = False
        self.nmea_running = False
        if self.process:
            try:
                print("ENDING THE PROCESS")
                #self.process.send_signal(signal.CTRL_BREAK_EVENT)
                #os.system("taskkill /im pocket_trk.exe /f")
                self.process.kill()
                #self.process.terminate()
                print("ENDED IT")
                #self.process.kill()
                #self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
            finally:
                self.process = None
        
        if self.nmea_socket:
            self.nmea_socket.close()
            self.nmea_socket = None
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.status_bar.config(text="SDR process stopped")

    def read_pocket_sdr(self, cmd):
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdin=subprocess.DEVNULL,
                text=True,
                cwd=self.base_path,
                creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
            )

            stderr_thread = threading.Thread(target=self.read_stderr, daemon=True)
            stderr_thread.start()

            while self.running:
                line = self.process.stdout.readline()
                if not line and self.process.poll() is not None:
                    break
                if line:
                    cleaned_line = line.strip()
                    if self.log_file:
                        try:
                            self.log_file.write(cleaned_line + '\n')
                            self.log_file.flush()
                        except Exception as e:
                            print(f"[Log Write Error] {e}")
                    self.output_queue.put(("stdout", cleaned_line))
        except Exception as e:
            print(f"[SDR Error] {e}")
            self.output_queue.put(("error", f"SDR error: {e}"))
        finally:
            if self.log_file:
                try:
                    self.log_file.write(f"--- Logging Stopped: {time.ctime()} ---\n")
                    self.log_file.close()
                except Exception as e:
                    print(f"[Log Close Error] {e}")
            self.log_file = None
            self.running = False


    def read_stderr(self):
        while self.running:
            line = self.process.stderr.readline()
            if not line:
                break
            if self.log_file:
                try:
                    self.log_file.write(f"STDERR: {line}")
                    self.log_file.flush()
                    print(f"Logged stderr: {line.strip()}")
                except Exception as e:
                    print(f"Stderr log error: {e}")
            self.output_queue.put(("stderr", line))

    def process_queue(self):
        if not self.running:
            return
        try:
            while True:
                try:
                    type_, line = self.output_queue.get_nowait()
                    cleaned_line = re.sub(r'\033\[[0-9;]*[mA]', '', line).strip()
                    if not cleaned_line or "CH  RF  SAT  SIG" in cleaned_line:
                        continue
                    if re.match(r"\d{4}-\d{2}-\d{2}", cleaned_line):
                        self.parse_position_status(cleaned_line)
                    #elif re.match(r"\s*\d+\s+\d+\s+G\d+", cleaned_line):
                    elif re.match(r"\s*\d+\s+\d+\s+[A-Z]\d+", cleaned_line):
                        data = self.parse_satellite_data(cleaned_line)
                        if data:
                            #self.sat_data_buffer[data[2]] = data  # Use SAT ID as key
                            unique_key = (data[0], data[2], data[3])  # CH, SAT, SIG
                            self.sat_data_buffer[unique_key] = data

                            if not self.ui_update_scheduled:
                                self.ui_update_scheduled = True
                                self.root.after(0, self.update_ui)
                except Empty:
                    break
        except Exception as e:
            print(f"Queue processing error: {e}")
        finally:
            if self.running:
                self.root.after(self.update_interval, self.process_queue)

    def parse_position_status(self, line):
        parts = line.split()
        if len(parts) >= 12:
#             self.status_labels["Time"].config(text=f"Time: {parts[0]} {parts[1]}")
#             self.status_labels["Latitude"].config(text=f"Latitude: {parts[2]}")
#             self.status_labels["Longitude"].config(text=f"Longitude: {parts[3]}")
#             self.status_labels["Altitude"].config(text=f"Altitude: {parts[4]}")
#             self.status_labels["Fix"].config(text=f"Fix: {parts[5]}")
#             self.status_labels["Buffer"].config(text=f"Buffer: {parts[8]}")
#             self.status_labels["Search"].config(text=f"Search: {parts[10]}")
#             self.status_labels["Lock"].config(text=f"Lock: {parts[12]} {parts[13]}")
            if parts[2] != '0.00000000' and parts[1] != self.old_time:
                self.status_labels["Time"].config(text=f"Time: {parts[0]} {parts[1]}")
                self.status_labels["Latitude"].config(text=f"Latitude: {parts[2]}", foreground="black", background="#00ff00")
                self.status_labels["Longitude"].config(text=f"Longitude: {parts[3]}", foreground="black", background="#00ff00")
                self.status_labels["Altitude"].config(text=f"Altitude: {parts[4]}", foreground="black", background="#00ff00")
                self.status_labels["Fix"].config(text=f"Fix: {parts[5]}")
                self.status_labels["Buffer"].config(text=f"Buffer: {parts[8]}")
                self.status_labels["Search"].config(text=f"Search: {parts[10]}")
                self.status_labels["Lock"].config(text=f"Lock: {parts[12]} {parts[13]}")
                self.old_time = parts[1]
            else:
                self.status_labels["Time"].config(text=f"Time: {parts[0]} {parts[1]}")
                self.status_labels["Latitude"].config(text=f"Latitude: {parts[2]}")
                self.status_labels["Longitude"].config(text=f"Longitude: {parts[3]}")
                self.status_labels["Altitude"].config(text=f"Altitude: {parts[4]}")
                self.status_labels["Fix"].config(text=f"Fix: {parts[5]}")
                self.status_labels["Buffer"].config(text=f"Buffer: {parts[8]}")
                self.status_labels["Search"].config(text=f"Search: {parts[10]}")
                self.status_labels["Lock"].config(text=f"Lock: {parts[12]} {parts[13]}")
                self.old_time = parts[1]

    def parse_satellite_data(self, line):
        try:
            parts = line.strip().split()

            if len(parts) < 16:
                print(f"[Skip] Incomplete line ({len(parts)} parts): {line.strip()}")
                return None

            ch = parts[0]          # CH
            sat = parts[2]         # SAT
            sig = parts[3]         # SIG
            prn = parts[4]         # PRN
            lock = parts[5]        # LOCK(s)
            cn0 = parts[6]         # C/N0 (dB-Hz) ← ✔️ ONLY this should be used for plotting
            coff = parts[8]        # COFF(ms) (skip parts[7])
            dop = parts[9]         # DOP(Hz)
            nav = parts[12]        # NAV
            lol = parts[14]        # LOL
            fec = parts[15]        # FEC

            return (ch, sat, sig, prn, lock, cn0, coff, dop, nav, lol, fec)
        except Exception as e:
            print(f"[Error] {e} in line: {line.strip()}")
            return None

    def read_nmea_data(self):
        max_retries = 5
        retry_delay = 2  # seconds
        for attempt in range(max_retries):
            try:
                self.nmea_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.nmea_socket.connect(("localhost", 4400))
                print("Connected to NMEA socket on localhost:4400")
                while self.nmea_running:
                    self.data = self.nmea_socket.recv(1024).decode('ascii', errors='ignore')
                    print(self.data)
                    if not self.data:
                        print("No NMEA data received, breaking")
                        break
                    print(f"Received NMEA data: {self.data}")  # Debug print
                    for sentence in self.data.splitlines():
                        if sentence.startswith(("$GPRMC", "$GNRMC")):
                            print(f"Processing NMEA sentence: {sentence}")  # Debug print
                            self.parse_nmea_sentence(sentence)
                    return self.data
                break  # Exit retry loop on success
            except ConnectionRefusedError as e:
                print(f"Connection attempt {attempt + 1} failed: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    print(f"NMEA error: Failed to connect after {max_retries} attempts: {e}")
                    self.output_queue.put(("error", f"NMEA error: Failed to connect after {max_retries} attempts: {e}"))
            except Exception as e:
                print(f"NMEA error: {e}")
                self.output_queue.put(("error", f"NMEA error: {e}"))
                break
#             finally:
#                 if self.nmea_socket:
#                     self.nmea_socket.close()
#                     print("NMEA socket closed")

    def parse_nmea_sentence(self, sentence):
        parts = sentence.split(',')
        if len(parts) >= 8 and parts[2] == 'A':
            try:
                speed_knots = float(parts[7])
                speed_mps = speed_knots * 0.514444
                self.root.after(0, lambda: self.status_labels["Velocity"].config(text=f"Velocity: {speed_mps:.2f} m/s"))
            except (ValueError, IndexError):
                self.root.after(0, lambda: self.status_labels["Velocity"].config(text="Velocity: N/A"))

    def update_ui(self):
        self.update_table()
        self.update_plot()
        self.ui_update_scheduled = False

    def update_table(self):
        # Use composite key (CH, SAT, SIG) to uniquely identify each satellite signal
        current_items = {
            (item["values"][0], item["values"][1], item["values"][2]): iid
            for iid in self.tree.get_children()
            for item in [self.tree.item(iid)]
        }

        sat_data_list = list(self.sat_data_buffer.values())

        for i, data in enumerate(sat_data_list):
            tag = "evenrow" if i % 2 == 0 else "oddrow"
            key = (data[0], data[1], data[2])  # CH, SAT, SIG

            if key in current_items:
                self.tree.item(current_items[key], values=data, tags=(tag,))
                del current_items[key]
            else:
                self.tree.insert("", "end", values=data, tags=(tag,))

        # Remove any old entries not in the new sat_data
        for iid in current_items.values():
            self.tree.delete(iid)

        # Scroll to latest item
        if self.tree.get_children():
            self.tree.see(self.tree.get_children()[-1])


    def update_plot(self):
        if not self.sat_data_buffer:
            return

        # Constellation classification
        constellations = {
            "GPS": {"L1CA"},
            "Galileo": {"E1B", "E5AI", "E5BI"},
            "BeiDou": {"B1I", "B1CD", "B2AD", "B2I", "B2BI", "B3I"},
            "GLONASS": {"G1CA"}
        }
        colors = {
            "GPS": "skyblue",
            "Galileo": "green",
            "BeiDou": "orange",
            "GLONASS": "red"
        }

        grouped_data = {k: [] for k in constellations}
        for data in self.sat_data_buffer.values():
            sig = data[2]
            for constellation, signals in constellations.items():
                if sig in signals:
                    grouped_data[constellation].append((f"{data[1]}", float(data[5])))
                    break

        self.ax.clear()
        bar_handles = []
        for constellation, sats in grouped_data.items():
            if sats:
                labels, cn0_values = zip(*sats)
                bar = self.ax.bar(labels, cn0_values, label=constellation, color=colors[constellation])
                bar_handles.append(bar)

        self.ax.set_xlabel("Satellite", fontsize=10)
        self.ax.set_ylabel("C/N0 (dB-Hz)", fontsize=10)
        self.ax.set_title("Carrier-to-Noise Ratio per Satellite", fontsize=14)
        self.ax.grid(True)
        self.ax.legend()
        #self.ax.set_xticklabels(self.ax.get_xticklabels(), rotation=45, fontsize=10)

        self.canvas.draw()


    def clear_data(self):
        self.sat_data_buffer.clear()
        self.tree.delete(*self.tree.get_children())
        for label in self.status_labels.values():
            label.config(text=f"{label.cget('text').split(':')[0]}: ")
        self.ax.clear()
        self.canvas.draw()

    def destroy(self):
        self.stop_pocket_sdr()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = PocketSDRGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.destroy)
    root.mainloop()

if __name__ == "__main__":
    main()