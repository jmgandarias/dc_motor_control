# Small GUI tool to trigger the ESP32 step experiment and save CSV data.
# - Opens a serial port, sends a single '1' byte to start the experiment on the ESP32.
# - Reads lines of the form "PWM;pos;vel;time" and collects rows.
# - Stops on an "END" line and saves collected data to a CSV file when requested.
#
# Comments have been added to clarify structure, threading, serial handling and CSV saving.

import sys
import time
import threading
import queue
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import csv
from datetime import datetime
import os

# Default serial settings and reader sleep interval
DEFAULT_BAUD = 500000  # Must match Serial.begin() on ESP32
READ_THREAD_SLEEP_S = 0.01  # Sleep between polls in the reader thread


class ESP32StepGUI(tk.Tk):
    """Main GUI application window for running the ESP32 step experiment."""

    def __init__(self):
        super().__init__()
        self.title("ESP32 Step Experiment (CSV)")
        self.geometry("720x576")

        # Serial and thread state
        self.ser = None  # serial.Serial object when open
        self.read_thread = None  # background thread that reads serial
        self.stop_event = threading.Event()  # signals reader thread to stop
        self.collecting = False  # True while collecting experiment data
        self.lines_queue = queue.Queue()  # UI thread queue for log lines

        # In-memory buffer of parsed data rows (thread-safe via data_lock)
        # Row format: [PWM(int), pos(float), vel(float), time_ms(float)]
        self.data_rows = []
        self.data_lock = threading.Lock()

        # Build UI and start periodic queue processing
        self._build_ui()
        self.after(100, self._process_lines_queue)

        self.ready_seen = False

    # -------------------- UI --------------------
    def _build_ui(self):
        """Create and lay out all UI widgets."""
        pad = {'padx': 8, 'pady': 6}

        # Connection frame
        conn = ttk.LabelFrame(self, text="Connection")
        conn.pack(fill='x', **pad)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, sticky='w', **pad)
        self.port_cmb = ttk.Combobox(conn, width=36, state='readonly', values=self._list_ports())
        self.port_cmb.grid(row=0, column=1, sticky='w', **pad)

        self.refresh_btn = ttk.Button(conn, text="Refresh", command=self._refresh_ports)
        self.refresh_btn.grid(row=0, column=2, sticky='w', **pad)

        ttk.Label(conn, text="Baud:").grid(row=1, column=0, sticky='w', **pad)
        self.baud_cmb = ttk.Combobox(conn, width=12, state='readonly',
                                    values=[9600, 19200, 38400, 57600, 115200, 230400, 250000, 460800, 500000 , 921600, 1000000, 2000000])
        # If you run your ESP32 at 2,000,000 baud by default, do:
        # self.baud_cmb.set("2000000")
        self.baud_cmb.set(str(DEFAULT_BAUD))
        self.baud_cmb.grid(row=1, column=1, sticky='w', **pad)

        # New: Connect / Disconnect buttons
        self.connect_btn = ttk.Button(conn, text="Connect", command=self._on_connect)
        self.connect_btn.grid(row=0, column=3, sticky='w', **pad)
        self.disconnect_btn = ttk.Button(conn, text="Disconnect", command=self._on_disconnect, state='disabled')
        self.disconnect_btn.grid(row=0, column=4, sticky='w', **pad)

        # --- Output frame (add this) ---
        out = ttk.LabelFrame(self, text="Output")
        out.pack(fill='x', **pad)

        ttk.Label(out, text="CSV file:").grid(row=0, column=0, sticky='w', **pad)

        # The actual entry the rest of the code expects:
        self.file_entry = ttk.Entry(out, width=50)
        self.file_entry.grid(row=0, column=1, sticky='we', **pad)
        out.columnconfigure(1, weight=1)

        # Sensible default path: ~/Documents if it exists, else CWD, with timestamped name.
        default_name = datetime.now().strftime("esp32_step_%Y%m%d_%H%M%S.csv")
        docs = os.path.expanduser("~/Documents")
        default_dir = docs if os.path.isdir(docs) else os.getcwd()
        self.file_entry.insert(0, os.path.join(default_dir, default_name))

        # Browse button already wired to _browse_outfile()
        browse_btn = ttk.Button(out, text="Browse…", command=self._browse_outfile)
        browse_btn.grid(row=0, column=2, **pad)

        # Controls frame
        ctrl = ttk.LabelFrame(self, text="Controls")
        ctrl.pack(fill='x', **pad)

        # Start is disabled until connected
        self.start_btn = ttk.Button(ctrl, text="Start Experiment", command=self._on_start, state='disabled')
        self.start_btn.grid(row=0, column=0, **pad)

        self.stop_btn = ttk.Button(ctrl, text="Stop & Save", command=self._on_stop_and_save, state='disabled')
        self.stop_btn.grid(row=0, column=1, **pad)

        self.exit_btn = ttk.Button(ctrl, text="Exit", command=self._on_exit)
        self.exit_btn.grid(row=0, column=2, **pad)

        # Status text and log pane
        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(self, textvariable=self.status_var).pack(anchor="w", **pad)

        self.log = tk.Text(self, height=9, wrap="none", state="disabled")
        self.log.pack(fill="both", expand=True, **pad)

    # -------------------- Serial helpers --------------------
    def _list_ports(self):
        """Return a list of human-readable port strings for the combobox."""
        ports = serial.tools.list_ports.comports()
        return [f"{p.device} - {p.description}" for p in ports]

    def _refresh_ports(self):
        ports = self._list_ports()
        self.port_cmb['values'] = ports
        if len(ports) == 1:
            self.port_cmb.set(ports[0])
        self._log("Ports refreshed.")

    def _open_serial(self):
        if self.ser and self.ser.is_open:
            return True
        sel = self.port_cmb.get()
        if not sel:
            messagebox.showwarning("Select port", "Please select a serial port first.")
            return False
        port = sel.split(" - ")[0]
        try:
            self.ser = serial.Serial(
                port=port, baudrate=int(self.baud_cmb.get()), timeout=0.05
            )

            # Optional: pulse DTR to reset many ESP32 boards
            try:
                self.ser.dtr = False
                time.sleep(0.05)
                self.ser.dtr = True
            except Exception:
                pass

            # Clear any old data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            time.sleep(0.2)  # small settle time
            self._log(f"Opened {port} @ {self.ser.baudrate} baud.")
            return True
        except Exception as e:
            messagebox.showerror("Serial error", f"Could not open port:\n{e}")
            return False

    def _close_serial(self):
        """Close serial port if open (safe to call multiple times)."""
        if self.ser:
            try:
                self.ser.close()
                self._log("Serial port closed.")
            except Exception:
                pass
            self.ser = None

    # -------------------- Actions --------------------
    def _on_start(self):

        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Not connected", "Connect first.")
            return
        if not self.ready_seen:
            # Either warn or wait briefly
            self.status_var.set("Waiting for READY…")
            self._log("Device not READY yet; delaying send 1 by 800 ms.")
            self.after(800, self._send_start_trigger)  # send a bit later
        else:
            self._send_start_trigger()

    def _send_start_trigger(self):
        # Reset buffer, send '1'
        with self.data_lock:
            self.data_rows = []
        self.collecting = True
        self.start_btn['state'] = 'disabled'
        self.stop_btn['state'] = 'normal'
        self.status_var.set("Experiment running…")
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(b'1')
            self.ser.flush()
            self._log("Sent start trigger: '1'")
        except Exception as e:
            messagebox.showerror("Serial error", f"Failed to write start byte:\n{e}")
            self._on_stop_and_save(force_save=False)

    def _on_stop_and_save(self, force_save=True):
        """Stop collecting and optionally save the collected data to CSV."""
        self.collecting = False
        self.stop_btn["state"] = "disabled"
        self.start_btn["state"] = "normal"
        saved = False
        if force_save:
            saved = self._save_csv()
        self.status_var.set("Saved and ready." if saved else "Ready.")

    def _on_exit(self):
        """Gracefully exit the application: stop thread, close serial and destroy window."""
        self.stop_event.set()
        self.collecting = False
        try:
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join(timeout=1.0)
        except Exception:
            pass
        self._close_serial()
        self.destroy()

    def _on_connect(self):
        """Open the selected serial port and start the background reader."""
        if not self._open_serial():
            return

        # Start the reader thread if not already running
        if self.read_thread is None or not self.read_thread.is_alive():
            self.stop_event.clear()
            self.read_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.read_thread.start()

        # Clear any stale data
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass

        self._log(f"Connecting to {self.ser.port} @ {self.ser.baudrate} baud.")
        self.status_var.set("Connected. Ready to start.")
        # Update UI states
        self.connect_btn['state'] = 'disabled'
        self.disconnect_btn['state'] = 'normal'
        self.start_btn['state'] = 'normal'
        self.stop_btn['state'] = 'disabled'


    def _on_disconnect(self):
        """Stop reader, close the port, and update UI."""
        self.collecting = False
        self.stop_event.set()
        try:
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join(timeout=1.0)
        except Exception:
            pass
        self.read_thread = None

        self._close_serial()

        self.status_var.set("Disconnected.")
        self._log("Disconnected.")
        # Update UI states
        self.connect_btn['state'] = 'normal'
        self.disconnect_btn['state'] = 'disabled'
        self.start_btn['state'] = 'disabled'
        self.stop_btn['state'] = 'disabled'

    # -------------------- Reader & parsing --------------------
    def _reader_loop(self):
        """Background thread: read raw bytes from serial, split into lines and hand to handler.
        Uses a small local buffer to cope with partial reads and CRLF/newline handling.
        """
        buffer = b""
        while not self.stop_event.is_set():
            try:
                if self.ser and self.ser.is_open:
                    chunk = self.ser.read(1024)
                    if chunk:
                        buffer += chunk
                        # Process all complete lines available in buffer
                        while b"\n" in buffer:
                            line, buffer = buffer.split(b"\n", 1)
                            # Decode line robustly and strip whitespace
                            line = line.strip().decode(errors="ignore")
                            if line:
                                self._handle_line(line)
                    else:
                        # No data this iteration: sleep briefly to avoid busy-looping
                        time.sleep(READ_THREAD_SLEEP_S)
                else:
                    # Serial not open: sleep longer and retry
                    time.sleep(0.2)
            except Exception as e:
                # Forward read errors to UI log and continue
                self.lines_queue.put(("log", f"[Serial read error] {e}"))
                time.sleep(0.2)

    def _handle_line(self, line: str):
        """Process a single text line from the ESP32.
        Expected format: 'PWM;pos;vel;time' or the special 'END' marker."""
        # Echo every line in the log (UI thread will display)
        self.lines_queue.put(("log", line))

        if line.strip() == "READY":
                self.ready_seen = True
                # Optionally auto-enable Start:
                self.after(0, lambda: self.status_var.set("Connected. READY from device."))
                return

        # If ESP32 signals end of experiment with "END", stop & save on main thread
        if line == "END":
            # Use after() to ensure UI actions run in main thread
            self.after(0, lambda: self._on_stop_and_save(force_save=True))
            return

        # If not currently collecting, ignore parsed data
        if not self.collecting:
            return

        # Parse semicolon-separated fields
        parts = line.split(";")
        if len(parts) != 4:
            return  # ignore malformed lines

        try:
            # Accept integer or float-looking PWM, be tolerant to formatting
            pwm = int(float(parts[0]))
            pos = float(parts[1])
            vel = float(parts[2])
            tms = float(parts[3])  # milliseconds reported by ESP32
        except ValueError:
            return  # ignore parse errors

        # Append row to shared buffer under lock
        with self.data_lock:
            self.data_rows.append([pwm, pos, vel, tms])

    

    # -------------------- Save CSV --------------------
    def _save_csv(self):
        """Write collected rows to CSV file. Returns True on success."""
        with self.data_lock:
            rows = list(self.data_rows)

        if not rows:
            self._log("No data to save.")
            return False

        out_path = self.file_entry.get().strip()
        if not out_path:
            messagebox.showwarning(
                "Missing path", "Please specify an output .csv filename."
            )
            return False
        if not out_path.lower().endswith(".csv"):
            out_path += ".csv"

        out_dir = os.path.dirname(out_path) or "."
        os.makedirs(out_dir, exist_ok=True)

        try:
            # Write header + data rows
            with open(out_path, mode="w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["PWM", "pos_rad", "vel_rad_per_s", "time_ms"])
                writer.writerows(rows)
            self._log(f"Saved {len(rows)} rows to:\n{out_path}")
            return True
        except Exception as e:
            messagebox.showerror("Save error", f"Failed to save CSV:\n{e}")
            return False

    # -------------------- UI helpers --------------------
    def _browse_outfile(self):
        """Open file-save dialog and update file_entry with chosen path."""
        initial = self.file_entry.get()
        if not initial.lower().endswith(".csv"):
            initial += ".csv"
        file = filedialog.asksaveasfilename(
            title="Choose output .csv file",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=os.path.basename(initial),
            initialdir=(
                os.path.dirname(initial) if os.path.dirname(initial) else os.getcwd()
            ),
        )
        if file:
            self.file_entry.delete(0, tk.END)
            self.file_entry.insert(0, file)

    def _process_lines_queue(self):
        """Periodically called on the main thread to drain the lines queue and append to the log widget."""
        try:
            while True:
                kind, payload = self.lines_queue.get_nowait()
                if kind == "log":
                    self._log(payload)
        except queue.Empty:
            pass
        # Re-schedule
        self.after(100, self._process_lines_queue)

    def _log(self, message: str):
        """Append a line to the read-only log text widget (thread-safe via queue)."""
        self.log.configure(state="normal")
        self.log.insert("end", message + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")


if __name__ == "__main__":
    app = ESP32StepGUI()
    app.mainloop()
