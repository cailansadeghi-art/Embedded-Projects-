import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import time

BAUD = 9600

def get_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Control Pannel")
        self.geometry("900x550")

        self.ser = None
        self.reader_running = False

        style = ttk.Style(self)
        aggieM = '#500000'
        style.theme_use("clam")
        style.configure("TFrame", background=aggieM)
        style.configure("TLabelframe", background=aggieM, foreground="white")
        style.configure("TLabelframe.Label", background=aggieM, foreground="white")
        style.configure("TLabel", background=aggieM, foreground="white")
        style.configure("TButton", padding=8)

        top = ttk.Frame(self, padding=10)
        top.pack(side="top", fill="x")
        ttk.Label(top, text="ROBOTIC ARM UI 2026", font=("Segoe UI", 16, "bold")).pack(side="left")

        serial_bar = ttk.Frame(top)
        serial_bar.pack(side="right")

        ttk.Label(serial_bar, text="Port:").pack(side="left", padx=(0, 6))

        self.port_var = tk.StringVar()
        self.port_menu = ttk.Combobox(serial_bar, textvariable=self.port_var, values=get_ports(),
                                      width=12, state="readonly")
        self.port_menu.pack(side="left")
        if self.port_menu["values"]:
            self.port_var.set(self.port_menu["values"][0])

        ttk.Button(serial_bar, text="Refresh", command=self.refresh_ports).pack(side="left", padx=6)
        ttk.Button(serial_bar, text="Connect", command=self.connect).pack(side="left", padx=6)
        ttk.Button(serial_bar, text="Disconnect", command=self.disconnect).pack(side="left", padx=6)
        ttk.Button(serial_bar, text="Clear Log", command=self.clear_log).pack(pady=5)


        main = ttk.Frame(self, padding=10)
        main.pack(side="top", fill="both", expand=True)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=2)
        main.rowconfigure(0, weight=1)

        left = ttk.LabelFrame(main, text="Controls", padding=10)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        right = ttk.LabelFrame(main, text="Log", padding=10)
        right.grid(row=0, column=1, sticky="nsew")

        self.log = tk.Text(right, wrap="word")
        self.log.pack(fill="both", expand=True)
        self.write_log("GUI loaded. (Serial not connected)\n")

        self.status_var = tk.StringVar(value="Status: Disconnected")
        status = ttk.Label(self, textvariable=self.status_var, anchor="w", padding=(10, 6))
        status.pack(side="bottom", fill="x")

        # Buttons (make sure Arduino has matching commands)
        self.add_button(left, "FIST", "Making Fist", "FIST")
        self.add_button(left, "FUNNY", "Making Funny", "FUNNY")
        self.add_button(left, "TAMU", "Saying TAMU in ASL", "TAMU")
        self.add_button(left, "BLOOD", "Throwing up the gang signs", "BLOOD")
        self.add_button(left, "ANGRY", "Showing the middle finger", "ANGRY")
        self.add_button(left, "SIMON_SAYS", "Playing Simon Says", "SIMON_SAYS")
        self.add_button(left, "RESET", "Resetting...", "RESET")

        # Clean shutdown
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def add_button(self, parent, label, log_text, cmd):
        ttk.Button(parent, text=label, command=lambda: self.on_press(log_text, cmd)).pack(fill="x", pady=5)

    def on_press(self, log_text, cmd):
        self.write_log(f"{log_text}\n")
        self.send(cmd)

    def write_log(self, text):
        self.log.insert("end", text)
        self.log.see("end")

    def refresh_ports(self):
        ports = get_ports()
        self.port_menu["values"] = ports
        self.port_var.set(ports[0] if ports else "")
        self.write_log(f"Ports refreshed: {ports}\n")
        
    def clear_log(self):
        self.log.delete("1.0", tk.END)


    def connect(self):
        port = self.port_var.get()
        if not port:
            self.write_log("No COM port selected.\n")
            return

        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.1)
            # Opening serial often resets Arduino — give it time to boot and print "Nano Ready"
            time.sleep(2.0)
            self.ser.reset_input_buffer()

            self.status_var.set(f"Status: Connected to {port} @ {BAUD}")
            self.write_log(f"Connected to {port}\n")

            self.reader_running = True
            threading.Thread(target=self.serial_reader, daemon=True).start()

        except Exception as e:
            self.ser = None
            self.status_var.set("Status: Disconnected")
            self.write_log(f"Connect failed: {e}\n")

    def disconnect(self):
        self.reader_running = False
        time.sleep(0.1)
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
        self.status_var.set("Status: Disconnected")
        self.write_log("Disconnected.\n")

    def send(self, cmd):
        if not self.ser:
            self.write_log(f"(Not sent — not connected) > {cmd}\n")
            return
        try:
            self.ser.write((cmd + "\n").encode("utf-8"))
            self.write_log(f"> {cmd}\n")
        except Exception as e:
            self.write_log(f"Send failed: {e}\n")

    def serial_reader(self):
        # Continuously read anything Arduino prints and push into the log
        while self.reader_running and self.ser:
            try:
                line = self.ser.readline()
                if line:
                    text = line.decode(errors="ignore").rstrip("\r\n")
                    # Tkinter UI updates must run on main thread:
                    self.after(0, lambda t=text: self.write_log(f"< {t}\n"))
            except:
                break

    def on_close(self):
        self.disconnect()
        self.destroy()

if __name__ == "__main__":
    App().mainloop()
