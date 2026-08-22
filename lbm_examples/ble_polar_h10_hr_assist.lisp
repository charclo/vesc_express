; Polar H10 heart rate -> proportional VESC motor assist
;
; Connects to a Polar H10 chest strap over the standard BLE Heart Rate
; Service (works with most other chest/wrist HRMs too, since it's a
; generic BLE GATT profile, not something Polar specific) and continuously
; adjusts the current-rel sent to a VESC controller over CAN.
;
; The assist is a simple proportional controller around a target heart
; rate: it gives MORE assist while your heart rate is ABOVE hr-target
; (helping bring your effort back down) and LESS assist while it's below
; (so you have to work a bit harder to get there). The net effect is the
; motor tries to hold you at a roughly constant heart rate/effort level
; regardless of hills, headwind, etc. - the same idea some cardiac-rehab
; and fitness e-bikes use. If you'd rather have it work the other way
; around (less assist at high effort, e.g. to encourage easing off), just
; flip the sign of hr-gain below.
;
; This script only depends on the generic (ble-client-*) building blocks
; and (canset-current-rel), so it should work unmodified against any BLE
; device that implements the standard Heart Rate Service - not just the
; Polar H10.
;
; --- Setup -------------------------------------------------------------
; 1. In VESC Tool, set BLE mode to "Scripting" (App Settings -> Ble) and
;    load this script (or add it as a startup script) - this also starts
;    the normal BLE GATT server used by VESC Tool/the mobile app, so
;    that keeps working alongside this.
; 2. Make sure the Polar H10 isn't already connected to a phone/watch -
;    most BLE devices only accept one connection at a time.
; 3. Leave hr-addr as nil for the first run: the script scans, prints
;    every device it sees whose name contains "Polar", and connects to
;    the first match. Once you see its address printed, copy it into
;    hr-addr below so future boots connect directly instead of scanning
;    (faster and more reliable).
; 4. Set motor-can-id and tune hr-target/hr-gain/assist-min/assist-max to
;    taste.

; --- User configuration -------------------------------------------------

; Builds a byte array (as used for BLE addresses/UUIDs/payloads below) out
; of a list of byte values - e.g. (make-bytes (list 0xaa 0xbb)).
(defun make-bytes-h (buf lst i)
    (if (not lst)
        buf
        (progn
            (bufset-u8 buf i (car lst))
            (make-bytes-h buf (cdr lst) (+ i 1))
        )
    )
)
(defun make-bytes (lst)
    (make-bytes-h (buf-resize "" nil (length lst) 'copy) lst 0)
)

; CAN id of the VESC controlling the motor. Use (can-scan) from a REPL to
; find it if you don't already know it.
(def motor-can-id 0)

; Set this to the strap's address once you know it, e.g:
; (def hr-addr (make-bytes (list 170 187 204 221 238 255)))
(def hr-addr nil)
; 0 = public, 1 = random. The Polar H10 (like most modern BLE devices)
; uses a random address - only change this if you know your device uses
; a public one.
(def hr-addr-type 1)

; Target heart rate (bpm) the assist tries to hold you at.
(def hr-target 140.0)

; How much current-rel to add/remove per bpm of deviation from hr-target.
(def hr-gain 0.012)

; Assist is always clamped to this range.
(def assist-min 0.0)
(def assist-max 0.6)

; If no heart rate reading has arrived for this many seconds (strap lost
; contact, out of range, disconnected, ...), assist falls back to
; assist-min as a safety measure.
(def hr-timeout-s 5.0)

; --- Heart Rate Service (standard BLE GATT service, 0x180D/0x2A37) -----

(def hr-service 0x180d)
(def hr-char 0x2a37)

(def hr-conn nil)
; ATT handle of the heart rate characteristic on the current connection,
; as returned by ble-client-subscribe - notifications are tagged with
; this, not with hr-char (their UUID).
(def hr-char-handle nil)
(def hr-last-update (systime))

(defun clampf (x lo hi)
    (cond
        ((< x lo) lo)
        ((> x hi) hi)
        (t x)
    )
)

; Heart Rate Measurement value, per the BT SIG Heart Rate Service spec:
;   byte 0: flags (bit 0 clear = uint8 bpm, set = uint16 bpm, LE)
;   byte 1(-2): heart rate value
;   (energy expended / rr-intervals may follow, ignored here)
(defun parse-hr (data)
    (if (= 0 (bitwise-and (bufget-u8 data 0) 1))
        (bufget-u8 data 1)
        (bufget-u16 data 1 'little-endian)
    )
)

(defun apply-hr (hr)
    (def hr-last-update (systime))
    (var assist (clampf (+ assist-min (* hr-gain (- hr hr-target))) assist-min assist-max))
    (canset-current-rel motor-can-id assist)
    (print (list "hr" hr "assist" assist))
)

; Runs as its own process (doesn't touch the mailbox), independently
; enforcing the "no recent reading -> fall back to assist-min" safety
; limit even if something upstream (BLE stack, the strap itself, ...)
; stops delivering data without an explicit disconnect event.
(defun watchdog-loop ()
    (loopwhile t
        (progn
            (sleep 1.0)
            (if (and hr-conn (> (secs-since hr-last-update) hr-timeout-s))
                (canset-current-rel motor-can-id assist-min)
                nil
            )
        )
    )
)

; Scans for a Polar H10 (or any HRM whose name contains "Polar") and
; returns its address, or nil if none was seen within the scan window.
; Must run on the process registered with event-register-handler, since
; that's the only process ble-scan events are ever delivered to.
(defun scan-for-hr ()
    (print "Scanning for a BLE heart rate monitor advertising as \"Polar\"...")
    (event-enable 'event-ble-scan)
    (ble-client-scan-start 15)
    (var found nil)
    (loopwhile (and (not found) (ble-client-scanning))
        (recv-to 0.5
            ((event-ble-scan (? a) (? rssi) (? name))
                (if (and name (>= (str-find name "Polar") 0))
                    (progn
                        (print (list "Found" name "at" a "rssi" rssi))
                        (setq found a)
                    )
                    nil
                )
            )
            (timeout nil)
        )
    )
    found
)

; Blocking: scans (if needed), connects, and subscribes to the heart rate
; characteristic. Leaves hr-conn set on success.
(defun connect-hr ()
    (var addr (if hr-addr hr-addr (scan-for-hr)))
    (if (not addr)
        (progn
            (print "No Polar H10 found. Move it closer, wake it up (put it on / wet the electrodes), and make sure it isn't already connected elsewhere - retrying.")
            (sleep 2.0)
        )
        (progn
            (print (list "Connecting to" addr "..."))
            (var conn (ble-client-connect addr hr-addr-type))
            (if conn
                (let ((handle (ble-client-subscribe conn hr-service hr-char)))
                    (if handle
                        (progn
                            (print (list "Connected and subscribed, conn-handle" conn))
                            (def hr-conn conn)
                            (def hr-char-handle handle)
                            (def hr-last-update (systime))
                        )
                        (progn
                            (print "Could not subscribe to the heart rate characteristic.")
                            (ble-client-disconnect conn)
                            (sleep 2.0)
                        )
                    )
                )
                (progn
                    (print "Connection failed, retrying...")
                    (sleep 2.0)
                )
            )
        )
    )
)

(defun app-loop ()
    (loopwhile t
        (if (not hr-conn)
            (connect-hr)
            (recv
                ((event-ble-client-data (? conn) (? handle) (? data))
                    (if (and (eq conn hr-conn) (= handle hr-char-handle))
                        (apply-hr (parse-hr data))
                        nil
                    )
                )
                ((event-ble-client-connect (? conn) (? connected))
                    (if (and (eq conn hr-conn) (not connected))
                        (progn
                            (print "Heart rate strap disconnected, reconnecting...")
                            (canset-current-rel motor-can-id assist-min)
                            (def hr-conn nil)
                            (def hr-char-handle nil)
                        )
                        nil
                    )
                )
                (_ nil)
            )
        )
    )
)

(ble-start-app)
(event-register-handler (spawn app-loop))
(event-enable 'event-ble-client-connect)
(event-enable 'event-ble-client-data)
(spawn watchdog-loop)
