import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading, re, os, time, socket, traceback, random, sys
import matplotlib.pyplot as plt
from PIL import Image, ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from queue import Queue, Empty
import serial.tools.list_ports
from datetime import datetime
import numpy as np
from geopy.distance import geodesic
import pygetwindow as gw

# NOTE: This version removes the pocket_trk subprocess entirely and
# reads BOTH streams from TCP sockets:
#   - NMEA stream on localhost:4848
#   - Tracking/"stderr" stream on localhost:6868
# It then updates the GUI exactly like before but fed by these sockets.

stop_window = "Administrator: C:\\WINDOWS\\system32\\cmd.exe"

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
        label = tk.Label(tw, text=self.text, background="yellow", relief=tk.SOLID, borderwidth=1, foreground="black", font=(12))
        label.pack()

    def hide_tip(self, event):
        if self.tip_window:
            self.tip_window.destroy()
            self.tip_window = None

class PocketSDRGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SDR Based GNSS Receiver")
        self.cmd_window_title = "PocketSDR CMD"
        self.root.state('zoomed')
        self.root.resizable(True, True)
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        window_width = int(screen_width * 0.8)
        window_height = int(screen_height * 0.8)
        print(f"{window_width}x{window_height}")
        #self.root.geometry("1280x720")
        self.root.minsize(800, 600)
        self.root.config(highlightbackground="#151b23")
        self.root = root
        
        # --- NEW: TCP-only configuration ---
        self.nmea_host = "127.0.0.1"
        self.nmea_port = 4848
        self.track_host = "127.0.0.1"
        self.track_port = 6868
        
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
        self.data_adjust = ""
        self.old_time = ""
        self.velocity = 0
        self.acceleration = 0
        self.jerk = 0
        self.old_velocity = 0
        self.old_acceleration = 0
        self.old_jerk = 0
        self.tcpport = 4444
        self.current_utc_seconds = 0
        self.max_jerk = 0
        self.velocity_ms = 0
        self.first_time = 0
        self.nmeaStatus = 0
        self.pvtStatus = 0
        self.nmeaStarted = 0
        cmd_font = 16
        self.parts = ""
        self.lat = []
        self.lon = []
        self.cep_status = 0
        self.max_samples = 1000
        self.vrms = 0
        self.cep = 0
        self.velocity_ar = []
        self.open_sockets = []
        
        # IMPORTANT: Update this path to the actual location of pocket_trk.exe
#        self.base_path = "C:/Users/sting/Downloads/PocketSDR-master/PocketSDR-master/bin"
        if getattr(sys, 'frozen', False):
            base_path = os.path.dirname(sys.executable)
        else:
            base_path = os.path.dirname(os.path.abspath(__file__))
        self.base_path = base_path
        
        
        # Style configuration
        style = ttk.Style()
        style.theme_use('default')
        style.configure("TFrame", background="#dcdad5")
        style.configure("TButton", font=("Helvetica", cmd_font, "bold"), background="#ccdaff", foreground="black")
        style.configure("Treeview", font=("Helvetica", cmd_font), background="#151b23", foreground="black")
        style.configure("Treeview.Heading", font=("Helvetica", cmd_font, "bold"), background="#ccdaff", foreground="black")
        style.configure("TLabelframe.Label", font=("Helvetica", cmd_font, "bold"), background="#dcdad5", foreground="black")
        style.configure("TCombobox", font=("Helvetica", cmd_font))

        # Top frame for logo and heading
        self.top_frame = ttk.Frame(self.root, style="TFrame", padding="10")
        self.top_frame.grid(row=0, column=0, sticky="we")
        try:
            logo_path = os.path.join(self.base_path, "SRT_Logo_3.png")
            logo_image = Image.open(logo_path).resize((160, 140), Image.Resampling.LANCZOS)
            self.logo_photo = ImageTk.PhotoImage(logo_image)
            self.logo_label = ttk.Label(self.top_frame, image=self.logo_photo)
            self.logo_label.grid(row=0, column=0, padx=5, pady=0)
        except Exception as e:
            print(f"Logo loading failed: {e}")
        
        self.heading = ttk.Label(self.top_frame, text="SDR-BASED GNSS RECEIVER", font=("Helvetica", 40, "bold"), anchor='center')
        self.heading.grid(row=0, column=1, pady=10, sticky="ew")
        self.top_frame.columnconfigure(1, weight=1)

        # Main frame
        self.main_frame = ttk.Frame(self.root, style="TFrame", padding="10")
        self.main_frame.grid(row=1, column=0, sticky="nsew")

        # Command frame
        self.cmd_frame = ttk.LabelFrame(self.main_frame, text="Controls", padding="10")
        self.cmd_frame.grid(row=0, column=0, columnspan=2, sticky="we", pady=5)
        
        self.start_button = ttk.Button(self.cmd_frame, text="Start", command=self.start_tcp_readers)
        self.start_button.grid(row=0, column=0, padx=5, pady=5, sticky="we")
        
        self.stop_button = ttk.Button(self.cmd_frame, text="Stop", command=self.stop_all, state="disabled")
        self.stop_button.grid(row=1, column=0, padx=5, pady=5)
        
        self.log_button = ttk.Button(self.cmd_frame, text="Save Log", command=self.select_log_file)
        self.log_button.grid(row=1, column=3, padx=5, pady=5, sticky="we")
        
        self.cep_button = ttk.Button(self.cmd_frame, text="Reset CEP", command=self.reset_cep)
        self.cep_button.grid(row=1, column=7, padx=5, pady=5, sticky="we")

        # Create the label for the dropdown
        self.dropdown_label = ttk.Label(self.cmd_frame, text="Transmit NMEA:", font=("Helvetica", cmd_font, "bold"))
        self.dropdown_label.grid(row=0, column=1, padx=5, pady=5, sticky="we")  # 'e' aligns it to the right
        
        # Create the dropdown (combobox)
        self.dropdown = ttk.Combobox(self.cmd_frame, state='readonly', font=("Helvetica", 14), width=17)
        self.dropdown.set("Select COM Port...")
        self.dropdown.grid(row=0, column=2, padx=5, pady=5, sticky="we")
        self.dropdown.bind("<Button-1>", self.update_dropdown)
        
        # Create the label for the dropdown
        self.hz_label = ttk.Label(self.cmd_frame, text="Output Rate (Hz):", font=("Helvetica", cmd_font, "bold"))
        self.hz_label.grid(row=1, column=1, padx=5, pady=5, sticky="we")  # 'e' aligns it to the right

        # Create the dropdown (combobox)
        self.hz = ttk.Combobox(self.cmd_frame, state='readonly', font=("Helvetica", 14), width=17)
        self.hz.set("Select Output Rate...")
        self.hz.grid(row=1, column=2, padx=5, pady=5, sticky="we")
        self.hz.bind("<Button-1>", self.update_hz)
        
        # Create the label for the dropdown
        self.baud_label = ttk.Label(self.cmd_frame, text="Baudrate:", font=("Helvetica", cmd_font, "bold"))
        self.baud_label.grid(row=0, column=3, padx=5, pady=5, sticky="we")  # 'e' aligns it to the right

        # Create the dropdown (combobox)
        self.baud = ttk.Combobox(self.cmd_frame, state='readonly', font=("Helvetica", 14), width=17)
        self.baud.set("Select Baudrate...")
        self.baud.grid(row=0, column=4, padx=5, pady=5, sticky="w")
        self.baud.bind("<Button-1>", self.update_baud)
        
        self.connect_button = ttk.Button(self.cmd_frame, text="Start Transmission", command=self.toggle_connection)
        self.connect_button.grid(row=1, column=4, padx=5, pady=5)
        
        labelAll = ["Time", "Latitude", "Longitude", "Altitude", "CEP", "RMS Velocity"]
        self.state_labels = {}
        for i, label in enumerate(labelAll):
            self.state_labels[label] = ttk.Label(
                self.cmd_frame,
                text=f"{label}: ",
                font=("Helvetica", cmd_font, "bold"),
                foreground="black",
            )
        self.state_labels["Time"].grid(row=1, column=6, sticky="we", padx=5, pady=5)
        self.state_labels["Latitude"].grid(row=0, column=5, sticky="we", padx=5, pady=5)
        self.state_labels["Longitude"].grid(row=1, column=5, sticky="we", padx=5, pady=5)
        self.state_labels["Altitude"].grid(row=0, column=6, sticky="we", padx=5, pady=5)
        self.state_labels["CEP"].grid(row=0, column=7, sticky="we", padx=5, pady=5)
        self.state_labels["RMS Velocity"].grid(row=0, column=8, sticky="we", padx=5, pady=5)

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
        
        #Paned Window
        self.paned_window = ttk.PanedWindow(self.main_frame, orient=tk.VERTICAL)
        self.paned_window.grid(row=1, column=0, columnspan=2, rowspan=2, sticky="nsew")

        # Satellite table
        self.tree_frame = ttk.LabelFrame(self.paned_window, text="Satellite Data", padding="10")
        self.tree_frame.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)
        self.tree_frame = ttk.LabelFrame(self.paned_window, text="Satellite Data", padding="10")       
#        columns = ("CH", "RF", "SAT", "SIG", "PRN", "LOCK(s)", "C/N0", "COFF(ms)", "DOP(Hz)", "ADR(cyc)", "SYNC", "NAV", "ERR", "LOL", "FEC")
        columns = ("CH", "SAT", "SIG", "PRN", "LOCK(s)", "C/N0", "COFF(ms)", "DOP(Hz)", "NAV",  "LOL", "FEC")
        self.tree = ttk.Treeview(self.tree_frame, columns=columns, show="headings", height=35)
        
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
        
        # Add tree_frame to paned window
        self.paned_window.add(self.tree_frame, weight=2)
        
        # Tabs for Plot and Position & Status
        self.tab_control = ttk.Notebook(self.paned_window)
        self.tab_control.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=5)

        # C/N0 Plot tab
        self.plot_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.plot_tab, text="C/N0 Plot")

        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_tab)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        #Adding X and Y Labels
        self.ax.set_title("Carrier-to-Noise Ratio per Satellite", fontsize=14, color="black")
        self.ax.set_xlabel("Satellite", fontsize=9, color="black")  # Set x-axis label
        self.ax.set_ylabel("C/N0 (dB-Hz)", fontsize=9, color="black")  # Set y-axis label

        # Position & Status tab
        self.status_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.status_tab, text="Position & Status")

        self.status_frame = ttk.Frame(self.status_tab)
        self.status_frame.pack(fill="both", expand=True, padx=10, pady=10)

        labels = ["Time", "Latitude", "Longitude", "Altitude", "Fix", "Buffer", "Search", "Lock", "Velocity", "Acceleration", "Jerk", "CEP", "RMS Velocity"]
        self.status_labels = {}
        for i, label in enumerate(labels):
            self.status_labels[label] = ttk.Label(
                self.status_frame,
                text=f"{label}: ",
                font=("Helvetica", 16, "bold"),
                background="#f0f0f0",
                foreground="black",
                width=300
            )
            self.status_labels[label].grid(row=i, column=0, sticky="w", pady=1)
        
        self.kinematic_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.kinematic_tab, text="Kinematic Plots")
        
        self.fig1, self.ax1 = plt.subplots(3, 1, figsize=(6, 7))
        self.canvas2 = FigureCanvasTkAgg(self.fig1, master=self.kinematic_tab)
        self.canvas2.get_tk_widget().pack(fill="both", expand=True)
        
        # --- Matplotlib plot styling ---
        TEXT_COLOR = 'black'
        PLOT_BACKGROUND_COLOR = 'white'
        VELOCITY_LINE_COLOR = '#007ACC' # SkyBlue
        ACCELERATION_LINE_COLOR = '#228B22' # PaleGreen
        JERK_LINE_COLOR = '#FF8C00'     # Gold
        self.fig1.set_facecolor(PLOT_BACKGROUND_COLOR)
        for ax1_item in self.ax1:
            ax1_item.set_facecolor(PLOT_BACKGROUND_COLOR)
            ax1_item.tick_params(axis='x', colors=TEXT_COLOR)
            ax1_item.tick_params(axis='y', colors=TEXT_COLOR)
            ax1_item.xaxis.label.set_color(TEXT_COLOR)
            ax1_item.yaxis.label.set_color(TEXT_COLOR)
            ax1_item.title.set_color(TEXT_COLOR)
            ax1_item.grid(True, color='#444444', linestyle=':', linewidth=0.5)
            ax1_item.spines['bottom'].set_color('#888888')
            ax1_item.spines['top'].set_color('#888888')
            ax1_item.spines['right'].set_color('#888888')
            ax1_item.spines['left'].set_color('#888888')
            #ax1_item.legend(labelcolor='linecolor')
        
        # Set specific line colors for the plots
        self.vel_line, = self.ax1[0].plot([], [], label='Velocity', color=VELOCITY_LINE_COLOR)
        self.acc_line, = self.ax1[1].plot([], [], label='Acceleration', color=ACCELERATION_LINE_COLOR)
        self.jerk_line, = self.ax1[2].plot([], [], label='Jerk', color=JERK_LINE_COLOR)
        
        #self.ax1[0].set_xlabel('Time (s)', fontsize=8)
        self.ax1[0].set_ylabel('Velocity\n(m/s)', fontsize=12, rotation=0, labelpad=40)
        #self.ax1[1].set_xlabel('Time (s)', fontsize=8)
        self.ax1[1].set_ylabel('Acceleration\n(m/s^2)', fontsize=12, rotation=0,  labelpad=40)
        self.ax1[2].set_xlabel('Time (s)', fontsize=12)
        self.ax1[2].set_ylabel('Jerk\n(m/s^3)', fontsize=12, rotation=0,  labelpad=40)
        
        #if self.current_utc_seconds != 0:
        self.canvas2.draw()
        
        self.plot_data = {'time': [], 'velocity': [], 'acceleration': [], 'jerk': []}
        
        ##################################################################################################
        
        self.mean_error_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.mean_error_tab, text="CEP / RMS Plots")
        
        self.fig2, self.ax2 = plt.subplots(2, 1, figsize=(6, 7))
        self.canvas3 = FigureCanvasTkAgg(self.fig2, master=self.mean_error_tab)
        self.canvas3.get_tk_widget().pack(fill="both", expand=True)
        
        # --- Matplotlib plot styling ---
        TEXT_COLOR = 'black'
        PLOT_BACKGROUND_COLOR = 'white'
        CEP_LINE_COLOR = '#007ACC' # SkyBlue
        RMS_VELOCITY_LINE_COLOR = '#228B22' # PaleGreen
        self.fig2.set_facecolor(PLOT_BACKGROUND_COLOR)
        for ax2_item in self.ax2:
            ax2_item.set_facecolor(PLOT_BACKGROUND_COLOR)
            ax2_item.tick_params(axis='x', colors=TEXT_COLOR)
            ax2_item.tick_params(axis='y', colors=TEXT_COLOR)
            ax2_item.xaxis.label.set_color(TEXT_COLOR)
            ax2_item.yaxis.label.set_color(TEXT_COLOR)
            ax2_item.title.set_color(TEXT_COLOR)
            ax2_item.grid(True, color='#444444', linestyle=':', linewidth=0.5)
            ax2_item.spines['bottom'].set_color('#888888')
            ax2_item.spines['top'].set_color('#888888')
            ax2_item.spines['right'].set_color('#888888')
            ax2_item.spines['left'].set_color('#888888')
            #ax1_item.legend(labelcolor='linecolor')
        
        # Set specific line colors for the plots
        self.cep_line, = self.ax2[0].plot([], [], label='CEP', color=CEP_LINE_COLOR)
        self.vrms_line, = self.ax2[1].plot([], [], label='RMS Velocity', color=RMS_VELOCITY_LINE_COLOR)
        
        #self.ax1[0].set_xlabel('Time (s)', fontsize=8)
        self.ax2[0].set_ylabel('Circular Error\nProbability (m)', fontsize=12, rotation=0, labelpad=40)
        self.ax2[1].set_ylabel('RMS Velocity\n(m/s)', fontsize=12, rotation=0,  labelpad=40)
        self.ax2[1].set_xlabel('Time (s)', fontsize=8)
        
        #if self.current_utc_seconds != 0:
        self.canvas3.draw()
        
        self.plot_cep_err = {'time': [], 'cep': [], 'vrms': []}
        ##################################################################################################
        
        # Add tab control to paned window
        self.paned_window.add(self.tab_control, weight=1)
        
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

    # ---------------------- NEW: TCP-only lifecycle ----------------------
    def start_tcp_readers(self):
        # Reset state similar to old start
        self.first_time = 0
        self.statusGGA = 0
        self.running = True
        self.nmea_running = True
        self.last_processed_time = None
        self.old_velocity = 0
        self.old_acceleration = 0
        self.nmeaStatus = 0
        self.state = 0
        self.last_processed_time = 0
        self.current_utc_seconds = 0
        self.pvtStatus = 0
        self.cep_status = 0
        self.lat = []
        self.lon = []
        self.vrms = 0
        self.cep = 0
        self.velocity_ar = []

        self.sat_data_buffer.clear()
        self.update_table()
        self.clear_data()
        with self.output_queue.mutex:
            self.output_queue.queue.clear()

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
        self.status_bar.config(text=f"Connecting to NMEA {self.nmea_host}:{self.nmea_port} and Tracking {self.track_host}:{self.track_port} ...")

        self.stop_event = threading.Event()
        # Start NMEA reader
        self.nmea_thread = threading.Thread(target=self.read_nmea_data_socket, daemon=True)
        self.nmea_thread.start()
        # Start tracking/"stderr" reader
        self.track_thread = threading.Thread(target=self.read_tracking_socket, daemon=True)
        self.track_thread.start()

        self.root.after(self.update_interval, self.process_queue)

    def stop_all(self):
        if hasattr(self, 'stop_event'):
            self.stop_event.set()

        # Join threads
        for th_name in ('nmea_thread', 'track_thread'):
            if hasattr(self, th_name):
                th = getattr(self, th_name)
                if th and th.is_alive():
                    th.join(timeout=2)

        # Close sockets
        for s in (self.nmea_socket, self.track_socket):
            try:
                if s:
                    s.close()
            except Exception:
                pass
        self.nmea_socket = None
        self.track_socket = None

        # Reset state flags
        self.running = False
        self.nmea_running = False
        self.nmeaStatus = 0
        self.statusGGA = 0
        self.first_time = 0
        self.last_processed_time = 0
        self.current_utc_seconds = 0
        self.pvtStatus = 0
        self.cep_status = 0
        self.lat = []
        self.lon = []
        self.vrms = 0
        self.cep = 0
        self.velocity_ar = []

        if self.log_file:
            try:
                self.log_file.write(f"--- Logging Stopped: {time.ctime()} ---\n")
                self.log_file.close()
            except Exception as e:
                print(f"[Log Close Error] {e}")
            self.log_file = None

        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.status_bar.config(text="TCP readers stopped")

    # ---------------------- TCP readers ----------------------
    def read_tracking_socket(self):
        """Read tracking/status (previously stderr) lines from track_port and push into queue."""
        max_retries = 10
        retry_delay = 1.5
        for attempt in range(max_retries):
            if self.stop_event.is_set():
                return
            try:
                self.track_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.track_socket.settimeout(10)
                self.track_socket.connect((self.track_host, self.track_port))
                self.status_bar.config(text=f"Connected tracking on {self.track_host}:{self.track_port}")
                buffer = ""
                while not self.stop_event.is_set():
                    chunk = self.track_socket.recv(4096)
                    if not chunk:
                        break
                    try:
                        text = chunk.decode('utf-8', errors='ignore')
                    except Exception:
                        text = chunk.decode('ascii', errors='ignore')
                    buffer += text
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line is None:
                            continue
                        self.output_queue.put(("stderr", line))
                break
            except (ConnectionRefusedError, socket.timeout) as e:
                print(f"[Track sock] attempt {attempt+1} failed: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    self.output_queue.put(("error", f"Tracking socket failed after {max_retries} attempts: {e}"))
            except Exception as e:
                print(f"[Track sock] error: {e}")
                self.output_queue.put(("error", f"Tracking socket error: {e}"))
                break
            finally:
                try:
                    if self.track_socket:
                        self.track_socket.close()
                except Exception:
                    pass
                self.track_socket = None

    def read_nmea_data_socket(self):
        """Read NMEA sentences from nmea_port and feed kinematic/position pipeline."""
        self.old_velocity = 0
        self.old_acceleration = 0
        self.state = 0
        self.last_processed_time = None
        self.max_jerk = float('-inf')

        max_retries = 10
        retry_delay = 1.5
        for attempt in range(max_retries):
            if self.stop_event.is_set():
                return
            try:
                self.nmea_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.nmea_socket.settimeout(10)
                self.nmea_socket.connect((self.nmea_host, self.nmea_port))
                self.status_bar.config(text=f"Connected NMEA on {self.nmea_host}:{self.nmea_port}")
                buffer = ""
                while not self.stop_event.is_set():
                    data = self.nmea_socket.recv(2048)
                    if not data:
                        break
                    text = data.decode('ascii', errors='ignore')
                    buffer += text
                    while '\n' in buffer and not self.stop_event.is_set():
                        lineNMEA, buffer = buffer.split('\n', 1)
                        lineNMEA = lineNMEA.strip()
                        if not lineNMEA:
                            continue
                        if self.nmeaStatus == 0:
                            _, _, _, self.first_time = self.parse_gpgga(lineNMEA)
                            self.process_line(lineNMEA)
                            self.nmeaStatus = 1
                        else:
                            self.process_line(lineNMEA)
                break
            except (ConnectionRefusedError, socket.timeout) as e:
                print(f"[NMEA sock] attempt {attempt+1} failed: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    self.output_queue.put(("error", f"NMEA socket failed after {max_retries} attempts: {e}"))
            except Exception as e:
                print(f"[NMEA sock] error: {e}")
                traceback.print_exc()
                self.output_queue.put(("error", f"NMEA socket error: {e}"))
                break
            finally:
                try:
                    if self.nmea_socket:
                        self.nmea_socket.close()
                except Exception:
                    pass
                self.nmea_socket = None

    # ---------------------- Existing helpers (mostly unchanged) ----------------------
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

    def send_data_to_com(self, com_port, baud_rate, hz):
        try:
            import serial
            with serial.Serial(com_port, int('9600'), timeout=1) as self.ser:
                while True:
                    self.nmea_adjustment()
                    print(f"COM: {com_port}; Baud: {baud_rate}; Freq: {hz}")
                    print(f"SENDING: {self.data_adjust}")
                    self.ser.write(self.data_adjust.encode('utf-8'))
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

    def nmea_adjustment(self):
        # Adjust SNR in GSV and recompute checksum
        data_adjust = ""
        lines = self.data.strip().splitlines() if self.data else []
        for line in lines:
            if 'GSV' in line:
                line_wo_checksum = line.split('*')[0]
                fields = line_wo_checksum.split(',')
                i = 4
                while i+3 < len(fields):
                    fields[i+3] = self.adjust_snr(fields[i+3])
                    i += 4
                new_body = ','.join(fields)
                checksum = 0
                for c in new_body[1:]:
                    checksum ^= ord(c)
                data_adjust += f"{new_body}*{checksum:02X}\r\n"
                if self.log_file:
                    try:
                        self.log_file.write(f"{new_body}*{checksum:02X}\r\n")
                        self.log_file.flush()
                    except Exception as e:
                        print(f"[Log Write Error] {e}")
            else:
                data_adjust += line + "\r\n"
                if self.log_file:
                    try:
                        self.log_file.write(line + "\r\n")
                        self.log_file.flush()
                    except Exception as e:
                        print(f"[Log Write Error] {e}")
        self.data_adjust = data_adjust

    def adjust_snr(self, snr_str):
        try:
            snr = float(snr_str)
            delta = random.randint(-1, 1)
            new_snr = snr + delta
            new_snr = max(0, min(new_snr, 99))
            return str(int(new_snr))
        except ValueError:
            return snr_str

    def connect_com(self):
        selected_port = self.dropdown.get()
        selectedHz = self.hz.get()
        selectedBaud = self.baud.get()
        if selected_port == "No COM Ports Found":
            print("Please select a valid COM port.")
            return
        threading.Thread(target=self.send_data_to_com, args=(selected_port, selectedBaud, selectedHz), daemon=True).start()
        print(f"Started sending data to {selected_port}")

    def update_hz(self, event=None):
        self.hz['values'] = ['0.5', '1', '10']

    def update_dropdown(self, event=None):
        available_ports = self.find_com_ports()
        if available_ports:
            self.dropdown['values'] = available_ports
        else:
            self.dropdown['values'] = ['No COM Ports Found']

    def find_com_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def on_treeview_resize(self, event):
        total_width = self.tree.winfo_width() - 20
        for col, initial_width in zip(self.tree["columns"], self.initial_column_widths):
            new_width = max(50, int((initial_width / self.initial_total_width) * total_width))
            self.tree.column(col, width=new_width)

    def show_about(self):
        messagebox.showinfo("About", "SDR Based GNSS Receiver\nVersion 1.1 (TCP-only)\nDeveloped by StingRay Team")

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

    # ---------------------- Processing pipeline ----------------------
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
                    elif re.match(r"\s*\d+\s+\d+\s+[A-Z]\d+", cleaned_line):
                        data = self.parse_satellite_data(cleaned_line)
                        if data:
                            unique_key = (data[0], data[2], data[3])  # CH, SAT, SIG
                            self.sat_data_buffer[unique_key] = data
                            if not self.ui_update_scheduled:
                                self.ui_update_scheduled = True
                                self.root.after(0, self.update_ui)
                except Empty:
                    break
        except Exception as e:
            print(f"Queue processing error: {e}")
            traceback.print_exc()
        finally:
            if self.running:
                self.root.after(self.update_interval, self.process_queue)

    def parse_position_status(self, line):
        self.parts = line.split()
        if len(self.parts) >= 12:
            if self.parts[2] != '0.00000000' and self.parts[1] != self.old_time:
                self.pvtStatus = 1
                self.status_labels["Time"].config(text=f"Time: {self.parts[0]} {self.parts[1]}")
                self.status_labels["Latitude"].config(text=f"Latitude: {self.parts[2]}", foreground="#6FAE2B")
                self.status_labels["Longitude"].config(text=f"Longitude: {self.parts[3]}", foreground="#6FAE2B")
                self.status_labels["Altitude"].config(text=f"Altitude: {self.parts[4]} m", foreground="#6FAE2B")
                self.state_labels["Time"].config(text=f"Time: {self.parts[0]} {self.parts[1]}")
                self.state_labels["Latitude"].config(text=f"Latitude: {self.parts[2]}", foreground="#6FAE2B")
                self.state_labels["Longitude"].config(text=f"Longitude: {self.parts[3]}", foreground="#6FAE2B")
                self.state_labels["Altitude"].config(text=f"Altitude: {self.parts[4]} m", foreground="#6FAE2B")
                self.status_labels["Fix"].config(text=f"Fix: {self.parts[5]}")
                self.status_labels["Buffer"].config(text=f"Buffer: {self.parts[8]}")
                self.status_labels["Search"].config(text=f"Search: {self.parts[10]}")
                self.status_labels["Lock"].config(text=f"Lock: {self.parts[12]} {self.parts[13]}")
                self.cep = self.calculate_cep()
                self.status_labels["CEP"].config(text=f"CEP: {self.cep:.2f} m")
                self.state_labels["CEP"].config(text=f"CEP: {self.cep:.2f} m")
                self.vrms = self.calculate_vrms()
                self.status_labels["RMS Velocity"].config(text=f"RMS Velocity: {self.vrms:.2f} m/s")
                self.state_labels["RMS Velocity"].config(text=f"RMS Velocity: {self.vrms:.2f} m/s")
                self.update_cep_err_plot(self.cep, self.vrms, (self.current_utc_seconds - self.first_time))
                self.old_time = self.parts[1]
            else:
                self.status_labels["Time"].config(text=f"Time: {self.parts[0]} {self.parts[1]}")
                self.status_labels["Latitude"].config(text=f"Latitude: {self.parts[2]}")
                self.status_labels["Longitude"].config(text=f"Longitude: {self.parts[3]}")
                self.status_labels["Altitude"].config(text=f"Altitude: {self.parts[4]} m")
                self.state_labels["Time"].config(text=f"Time: {self.parts[0]} {self.parts[1]}")
                self.state_labels["Latitude"].config(text=f"Latitude: {self.parts[2]}")
                self.state_labels["Longitude"].config(text=f"Longitude: {self.parts[3]}")
                self.state_labels["Altitude"].config(text=f"Altitude: {self.parts[4]} m")
                self.status_labels["Fix"].config(text=f"Fix: {self.parts[5]}")
                self.status_labels["Buffer"].config(text=f"Buffer: {self.parts[8]}")
                self.status_labels["Search"].config(text=f"Search: {self.parts[10]}")
                self.status_labels["Lock"].config(text=f"Lock: {self.parts[12]} {self.parts[13]}")
                self.cep = self.calculate_cep()
                self.status_labels["CEP"].config(text=f"CEP: {self.cep:.2f} m")
                self.state_labels["CEP"].config(text=f"CEP: {self.cep:.2f} m")
                self.update_cep_err_plot(self.cep, self.vrms, (self.current_utc_seconds - self.first_time))
                self.old_time = self.parts[1]

    def parse_satellite_data(self, line):
        try:
            parts = line.strip().split()
            if len(parts) < 16:
                print(f"[Skip] Incomplete line ({len(parts)} parts): {line.strip()}")
                return None
            ch = parts[0]
            sat = parts[2]
            sig = parts[3]
            prn = parts[4]
            lock = parts[5]
            cn0 = parts[6]
            coff = parts[8]
            dop = parts[9]
            nav = parts[12]
            lol = parts[14]
            fec = parts[15]
            return (ch, sat, sig, prn, lock, cn0, coff, dop, nav, lol, fec)
        except Exception as e:
            print(f"[Error] {e} in line: {line.strip()}")
            return None

    def read_nmea_data(self):
        # Deprecated in TCP-only build, kept for compatibility if needed
        pass

    def process_line(self, line_to_process):
        # Update time from GGA
        print("INSIDE PROCESS LINE FUNCTION")
        print(f"First Time: {self.first_time}, Current UTC: {self.current_utc_seconds}, Status GGA: {getattr(self, 'statusGGA', 0)}")
        timing = 0
        while getattr(self, 'statusGGA', 0) == 0 and not self.stop_event.is_set():
            if "GGA" in line_to_process:
                self.statusGGA = 1
                _, _, _, self.first_time = self.parse_gpgga(line_to_process)
            else:
                break
        if "GGA" in line_to_process and self.statusGGA == 1 and not self.stop_event.is_set():
            _, _, _, time_from_gga = self.parse_gpgga(line_to_process)
            if time_from_gga is not None:
                self.current_utc_seconds = time_from_gga
                timing = self.current_utc_seconds - self.first_time
                print(f"GxGGA processed. Current UTC Seconds: {timing:.3f}")
            return
        # Velocity from RMC/VTG
        if "RMC" in line_to_process and (self.state == 0 or self.state == 2) and not self.stop_event.is_set():
            self.state = 2
            self.velocity = self.parse_grmc(line_to_process)
        elif "VTG" in line_to_process and (self.state == 0 or self.state == 1) and not self.stop_event.is_set():
            self.state = 1
            self.velocity = self.parse_gnvtg(line_to_process)
        if self.velocity is not None and self.current_utc_seconds is not None and not self.stop_event.is_set():
            if self.last_processed_time is not None:
                delta_time = self.current_utc_seconds - self.last_processed_time
                if delta_time > 0.001:
                    self.velocity_ms = float(self.velocity) * 0.5144444
                    self.acceleration = (self.velocity_ms - self.old_velocity) / delta_time
                    self.jerk = (self.acceleration - self.old_acceleration) / delta_time
                    self.max_jerk = max(self.max_jerk, self.jerk)
                    self.update_kinematic_display(self.velocity_ms, self.acceleration, self.jerk)
                    self.update_kinematic(self.velocity_ms, self.acceleration, self.jerk, (self.current_utc_seconds - self.first_time))
                    self.old_velocity = self.velocity_ms
                    self.old_acceleration = self.acceleration
                    self.last_processed_time = self.current_utc_seconds
            else:
                self.velocity_ms = float(self.velocity) * 0.5144444
                self.update_kinematic_display(self.velocity_ms, 0.0, 0.0)
                self.update_kinematic(self.velocity_ms, 0.0, 0.0, self.current_utc_seconds)
                self.old_velocity = self.velocity_ms
                self.old_acceleration = 0.0
                self.last_processed_time = self.current_utc_seconds
        if self.current_utc_seconds != 0 and not self.stop_event.is_set():
            self.canvas2.draw()

    def parse_gnvtg(self, sentence):
        parts = sentence.split(',')
        if len(parts) < 8:
            return None
        velocity = parts[7]
        return velocity

    def parse_grmc(self, sentence):
        parts = sentence.split(',')
        if len(parts) < 12:
            return None
        speed_knots = parts[7]
        return speed_knots

    def parse_gpgga(self, sentence):
        parts = sentence.split(',')
        if len(parts) < 10:
            return None, None, None, None
        utc_time = parts[1]
        lat = parts[2]
        lat_dir = parts[3]
        lon = parts[4]
        lon_dir = parts[5]
        alt = parts[9]
        if utc_time:
            try:
                utc_time_obj = datetime.strptime(utc_time, "%H%M%S.%f")
                utc_seconds = utc_time_obj.hour * 3600 + utc_time_obj.minute * 60 + utc_time_obj.second + utc_time_obj.microsecond / 1e6
            except ValueError:
                try:
                    utc_time_obj = datetime.strptime(utc_time, "%H%M%S")
                    utc_seconds = utc_time_obj.hour * 3600 + utc_time_obj.minute * 60 + utc_time_obj.second
                except ValueError:
                    utc_seconds = None
        else:
            utc_seconds = None
        return lat, lon, alt, utc_seconds

    def calculate_cep(self):
        self.max_samples = 1000
        if len(self.lat) >= self.max_samples:
            self.lat.pop(0)
            self.lon.pop(0)
        # parts updated by parse_position_status -> parts[2], parts[3]
        try:
            self.lat.append(float(self.parts[2]))
            self.lon.append(float(self.parts[3]))
        except Exception:
            return 0
        if len(self.lat) < 10:
            return 0
        mean_lat = np.mean(self.lat)
        mean_lon = np.mean(self.lon)
        errors = [geodesic((lat, lon), (mean_lat, mean_lon)).meters for lat, lon in zip(self.lat, self.lon)]
        cep = np.percentile(errors, 50)
        return cep

    def reset_cep(self):
        print("CEP Data Reset")
        self.lat.clear()
        self.lon.clear()
        self.velocity_ar.clear()
        self.calculate_cep()
        self.calculate_vrms()
        self.plot_cep_err = {'time': [], 'cep': [], 'vrms': []}
        self.cep_line.set_data([], [])
        self.vrms_line.set_data([], [])
        for ax2 in self.ax2:
            ax2.relim()
            ax2.autoscale_view()
        self.canvas3.draw()
        return "CEP reset complete."

    def calculate_vrms(self):
        self.max_samples = 1000
        if len(self.velocity_ar) >= self.max_samples:
            self.velocity_ar.pop(0)
        self.velocity_ar.append(self.velocity_ms)
        vrms = np.sqrt(np.mean(np.square(self.velocity_ar))) if self.velocity_ar else 0
        return vrms

    def update_cep_err_plot(self, cep, vrms, time_s):
        if not self.plot_cep_err['time']:
            self.plot_cep_err['initial_time'] = time_s
        relative_time = time_s - self.plot_cep_err['initial_time']
        self.plot_cep_err['time'].append(relative_time)
        self.plot_cep_err['cep'].append(cep)
        self.plot_cep_err['vrms'].append(vrms)
        max_points = 500
        if len(self.plot_cep_err['time']) > max_points:
            for key in self.plot_cep_err:
                if key != 'initial_time':
                    self.plot_cep_err[key] = self.plot_cep_err[key][-max_points:]
            self.plot_cep_err['initial_time'] = self.plot_cep_err['time'][0]
            self.plot_cep_err['time'] = [t - self.plot_cep_err['initial_time'] for t in self.plot_cep_err['time']]
        self.cep_line.set_data(self.plot_cep_err['time'], self.plot_cep_err['cep'])
        self.vrms_line.set_data(self.plot_cep_err['time'], self.plot_cep_err['vrms'])
        self.ax2[0].set_ylabel('Circular Error\nProbability (m)', fontsize=12, rotation=0, labelpad=40)
        self.ax2[1].set_ylabel('RMS Velocity\n(m/s)', fontsize=12, rotation=0,  labelpad=40)
        self.ax2[1].set_xlabel('Time (s)', fontsize=12)
        for ax2 in self.ax2:
            ax2.relim()
            ax2.autoscale_view()
            ax2.grid(True)
        if self.current_utc_seconds != 0:
            self.canvas3.draw()

    def update_kinematic(self, velocity, acceleration, jerk, time_s):
        if not self.plot_data['time']:
            self.plot_data['initial_time'] = time_s
        relative_time = time_s - self.plot_data['initial_time']
        self.plot_data['time'].append(relative_time)
        self.plot_data['velocity'].append(velocity)
        self.plot_data['acceleration'].append(acceleration)
        self.plot_data['jerk'].append(jerk)
        max_points = 500
        if len(self.plot_data['time']) > max_points:
            for key in self.plot_data:
                if key != 'initial_time':
                    self.plot_data[key] = self.plot_data[key][-max_points:]
            self.plot_data['initial_time'] = self.plot_data['time'][0]
            self.plot_data['time'] = [t - self.plot_data['initial_time'] for t in self.plot_data['time']]
        self.vel_line.set_data(self.plot_data['time'], self.plot_data['velocity'])
        self.acc_line.set_data(self.plot_data['time'], self.plot_data['acceleration'])
        self.jerk_line.set_data(self.plot_data['time'], self.plot_data['jerk'])
        self.ax1[0].set_ylabel('Velocity\n(m/s)', fontsize=12, rotation=0, labelpad=40)
        self.ax1[1].set_ylabel('Acceleration\n(m/s^2)', fontsize=12, rotation=0,  labelpad=40)
        self.ax1[2].set_xlabel('Time (s)', fontsize=12)
        self.ax1[2].set_ylabel('Jerk\n(m/s^3)', fontsize=12, rotation=0,  labelpad=40)
        for ax1 in self.ax1:
            ax1.relim()
            ax1.autoscale_view()
            ax1.grid(True)
        if self.current_utc_seconds != 0:
            self.canvas2.draw()

    def update_kinematic_display(self, velocity, acceleration, jerk):
        self.root.after(0, lambda: self.status_labels["Velocity"].config(text=f"Velocity: {velocity:.2f} m/s"))
        self.root.after(0, lambda: self.status_labels["Acceleration"].config(text=f"Acceleration: {acceleration:.2f} m/s^2"))
        self.root.after(0, lambda: self.status_labels["Jerk"].config(text=f"Jerk: {jerk:.2f} m/s^3"))
        if self.current_utc_seconds != 0:
            self.canvas2.draw()

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
        current_items = {
            (item["values"][0], item["values"][1], item["values"][2]): iid
            for iid in self.tree.get_children()
            for item in [self.tree.item(iid)]
        }
        sat_data_list = list(self.sat_data_buffer.values())
        for i, data in enumerate(sat_data_list):
            tag = "evenrow" if i % 2 == 0 else "oddrow"
            key = (data[0], data[1], data[2])
            if key in current_items:
                self.tree.item(current_items[key], values=data, tags=(tag,))
                del current_items[key]
            else:
                self.tree.insert("", "end", values=data, tags=(tag,))
        for iid in current_items.values():
            self.tree.delete(iid)

    def update_plot(self):
        if not self.sat_data_buffer:
            return
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
                    try:
                        cn0_value = float(data[5])
                        grouped_data[constellation].append((f"{data[1]}", cn0_value))
                    except:
                        continue
                    break
        self.ax.clear()
        for constellation, sats in grouped_data.items():
            if sats:
                labels, cn0_values = zip(*sats)
                self.ax.bar(labels, cn0_values, label=constellation, color=colors[constellation])
        self.ax.set_title("Carrier-to-Noise Ratio per Satellite", fontsize=14, color="black")
        self.ax.set_xlabel("Satellite", fontsize=9, color="black")
        self.ax.set_ylabel("C/N0 (dB-Hz)", fontsize=9, color="black")
        self.ax.grid(True)
        self.ax.legend()
        self.canvas.draw()

    def clear_data(self):
        self.state_labels["Time"].config(text=f"Time:", foreground="black")
        self.state_labels["Latitude"].config(text=f"Latitude:", foreground="black")
        self.state_labels["Longitude"].config(text=f"Longitude:", foreground="black")
        self.state_labels["Altitude"].config(text=f"Altitude:", foreground="black")
        self.status_labels["Latitude"].config(text=f"Latitude:", foreground="black")
        self.status_labels["Longitude"].config(text=f"Longitude:", foreground="black")
        self.status_labels["Altitude"].config(text=f"Altitude:", foreground="black")
        self.status_labels["CEP"].config(text=f"CEP:")
        self.state_labels["CEP"].config(text=f"CEP:")
        self.status_labels["RMS Velocity"].config(text=f"RMS Velocity:")
        self.state_labels["RMS Velocity"].config(text=f"RMS Velocity:")
        self.sat_data_buffer.clear()
        self.tree.delete(*self.tree.get_children())
        for label in self.status_labels.values():
            label.config(text=f"{label.cget('text').split(':')[0]}: ")
        self.ax.clear()
        self.canvas.draw()
        if self.current_utc_seconds != 0:
            self.canvas2.draw()
        self.plot_data = {'time': [], 'velocity': [], 'acceleration': [], 'jerk': []}
        self.vel_line.set_data([], [])
        self.acc_line.set_data([], [])
        self.jerk_line.set_data([], [])
        for ax in self.ax1:
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw()
        self.plot_cep_err = {'time': [], 'cep': [], 'vrms': []}
        self.cep_line.set_data([], [])
        self.vrms_line.set_data([], [])
        for ax2 in self.ax2:
            ax2.relim()
            ax2.autoscale_view()
        self.canvas3.draw()

    def destroy(self):
        self.stop_all()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = PocketSDRGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.destroy)
    root.mainloop()

if __name__ == "__main__":
    main()
