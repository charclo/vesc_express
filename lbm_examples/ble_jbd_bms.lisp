; JBD / Xiaoxiang / Overkill Solar "Smart BMS" over Bluetooth
;
; Reads pack voltage, current, state of charge, charge-FET status and
; per-cell voltages from a cheap JBD-protocol Bluetooth BMS - sold under
; many names (JBD, Xiaoxiang, Overkill Solar, and a long list of
; unbranded AliExpress/Alibaba listings; if the vendor's phone app is
; called something like "JBD Tools" or "XiaoXiang BMS", this is the right
; protocol) - and feeds the values into VESC Express' native BMS values
; (set-bms-val), so they show up in VESC Tool's normal BMS page exactly
; like a CAN-connected VESC BMS would.
;
; This script is READ-ONLY monitoring - it doesn't affect motor control
; by itself. Combine it with your own logic (e.g. cut assist below a SOC
; threshold, using (get-bms-val 'bms-soc) and (canset-current-rel ...))
; if you want that.
;
; The JBD protocol is widely used, but firmware differs a bit between
; vendors/clones (e.g. some report only 1 balance-status word where
; others report 2, or differ on capacity units). The layout below is the
; commonly documented one; if the numbers you get back look wrong, print
; `info`/`cells` in poll-bms and compare the raw bytes against the
; comments, then adjust the offsets for your unit. bms-soh and the
; accumulated Ah/Wh counters aren't set here since JBD's basic-info frame
; doesn't report them directly - extend parse-info/poll-bms if your unit
; does (e.g. via the design-capacity field) and you want them filled in.
;
; --- Setup ---------------------------------------------------------
; 1. Set BLE mode to "Scripting" in VESC Tool and load this script.
; 2. Make sure the BMS isn't already connected to its own phone app.
; 3. Leave bms-addr nil for the first run: the script scans for any
;    device whose name contains "bms" (case insensitive) and connects to
;    the first match - unreliable, since advertised names vary a lot
;    between clones. It's much more reliable to find the address once
;    with a generic BLE scanner app (e.g. nRF Connect) and hardcode it
;    into bms-addr below, the same way as in the Polar H10 example
;    script.

; --- User configuration -------------------------------------------------

; Builds a byte array (as used for BLE addresses/payloads below) out of a
; list of byte values - e.g. (make-bytes (list 0xaa 0xbb)).
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

; Set this to the BMS's address once you know it, e.g:
; (def bms-addr (make-bytes (list 170 187 204 221 238 255)))
(def bms-addr nil)
; 0 = public, 1 = random - most of these boards use a public address, but
; try 1 if connecting with 0 doesn't work.
(def bms-addr-type 0)

(def poll-interval-s 2.0)

; --- JBD protocol --------------------------------------------------

; Service/characteristics used by essentially all JBD-protocol BMS BLE
; modules (this is the 16 bit alias of the vendor's 128 bit UUID, which
; ble-client-* accepts directly).
(def bms-service 0xff00)
(def bms-char-write 0xff02)
(def bms-char-notify 0xff01)

; Read requests. Frame layout is [0xDD 0xA5 cmd len ...payload checksum-hi
; checksum-lo 0x77], with checksum = (0x10000 - (cmd + len + sum(payload)))
; & 0xFFFF. Both of these reads have no payload, so the checksum is fixed
; and the frames below can just be sent as-is.
(def bms-req-info  (make-bytes (list 0xDD 0xA5 0x03 0x00 0xFF 0xFD 0x77)))  ; pack info
(def bms-req-cells (make-bytes (list 0xDD 0xA5 0x04 0x00 0xFF 0xFC 0x77)))  ; per-cell voltages

(def bms-conn nil)
; ATT handle of the notify characteristic on the current connection, as
; returned by ble-client-subscribe - notifications are tagged with this,
; not with bms-char-notify (its UUID).
(def bms-notify-handle nil)
; Bytes of the response frame currently being reassembled, as a plain
; list (notifications are typically ~20 bytes/packet, but JBD response
; frames are often longer than that and arrive split across several).
(def bms-rx-buf nil)

(defun minf (a b) (if (< a b) a b))
(defun maxf (a b) (if (> a b) a b))

(defun chunk->list-h (arr i acc)
    (if (< i 0)
        acc
        (chunk->list-h arr (- i 1) (cons (bufget-u8 arr i) acc))
    )
)
; Converts a raw notification byte-array into a list of byte values, in
; order - lets us reassemble/parse without needing to pre-size an array.
(defun chunk->list (arr) (chunk->list-h arr (- (buflen arr) 1) nil))

(defun u8@ (buf i) (ix buf i))
(defun u16be@ (buf i) (+ (* 256 (ix buf i)) (ix buf (+ i 1))))
(defun i16be@ (buf i)
    (var v (u16be@ buf i))
    (if (>= v 32768) (- v 65536) v)
)

(defun frame-len (buf) (+ 7 (u8@ buf 3))) ; header(4) + payload + checksum(2) + end(1)
(defun frame-complete? (buf) (and (>= (length buf) 4) (>= (length buf) (frame-len buf))))

; Blocking: writes `req`, then waits (up to timeout-s) for a complete
; response frame whose command byte matches req's, reassembling it from
; however many notification packets it takes. Returns the full frame (as
; a byte list) on success, or nil on timeout, a mismatched response, or a
; disconnect. Must run on the process registered with
; event-register-handler.
(defun bms-request (conn req timeout-s)
    (def bms-rx-buf nil)
    (if (ble-client-write conn bms-service bms-char-write req)
        (bms-wait-frame conn (bufget-u8 req 2) timeout-s)
        nil
    )
)

(defun bms-wait-frame (conn cmd timeout-s)
    (var t0 (systime))
    (var result nil)
    (loopwhile (and (not result) (< (secs-since t0) timeout-s))
        (recv-to (- timeout-s (secs-since t0))
            ((event-ble-client-data (? c) (? handle) (? data))
                (if (and (eq c conn) (= handle bms-notify-handle))
                    (progn
                        (def bms-rx-buf (append bms-rx-buf (chunk->list data)))
                        (if (and (frame-complete? bms-rx-buf) (= (u8@ bms-rx-buf 1) cmd))
                            (setq result bms-rx-buf)
                            nil
                        )
                    )
                    nil
                )
            )
            ((event-ble-client-connect (? c) (? connected))
                (if (and (eq c conn) (not connected)) (setq result 'disconnected) nil)
            )
            (timeout nil)
        )
    )
    (if (eq result 'disconnected) nil result)
)

; --- Pack info (cmd 0x03) response layout, offsets from frame start ---
;  4: total voltage,      u16 be, 0.01V
;  6: current,            i16 be, 0.01A (sign convention varies by unit -
;                          flip if charge/discharge read backwards)
;  8: residual capacity,  u16 be, 0.01Ah
; 10: nominal capacity,   u16 be, 0.01Ah
; 12: cycle count,        u16 be
; 14: production date,    u16 be (unused)
; 16: balance status low, u16 be (bitmask, cells 1-16, unused here)
; 18: balance status high,u16 be (bitmask, cells 17-32, unused here)
; 20: protection status,  u16 be (bitmask, unused here)
; 22: software version,   u8 (unused)
; 23: RSOC,               u8, %
; 24: FET control status, u8 (bit0 = charge FET on, bit1 = discharge FET on)
; 25: cell count,         u8
; 26: NTC count,          u8
; 27+: NTC temps,         u16 be each, 0.1K units (subtract 273.15 after /10)

(defun parse-v-tot (buf) (/ (u16be@ buf 4) 100.0))
(defun parse-i-in (buf) (/ (i16be@ buf 6) 100.0))
(defun parse-rsoc (buf) (u8@ buf 23))
(defun parse-fet-status (buf) (u8@ buf 24))
(defun parse-cell-count (buf) (u8@ buf 25))
(defun parse-ntc-count (buf) (u8@ buf 26))
(defun parse-ntc-temp (buf i) (- (/ (u16be@ buf (+ 27 (* i 2))) 10.0) 273.15))

(defun parse-ntc-max-h (buf i n best)
    (if (>= i n)
        best
        (parse-ntc-max-h buf (+ i 1) n (maxf best (parse-ntc-temp buf i)))
    )
)
(defun parse-ntc-max (buf)
    (var n (parse-ntc-count buf))
    (if (> n 0) (parse-ntc-max-h buf 0 n -100.0) 0.0)
)

; --- Cell voltages (cmd 0x04) response layout ---
;  4+: cell voltages, u16 be each, mV, one per cell (count = len / 2)

(defun bms-apply-cells-h (buf n i vmin vmax)
    (if (>= i n)
        (progn
            (set-bms-val 'bms-v-cell-min vmin)
            (set-bms-val 'bms-v-cell-max vmax)
        )
        (let ((v (/ (u16be@ buf (+ 4 (* i 2))) 1000.0)))
            (progn
                (set-bms-val 'bms-v-cell i v)
                (bms-apply-cells-h buf n (+ i 1) (minf vmin v) (maxf vmax v))
            )
        )
    )
)
(defun bms-apply-cells (buf)
    (var n (/ (u8@ buf 3) 2))
    (set-bms-val 'bms-cell-num n)
    (bms-apply-cells-h buf n 0 5.0 0.0)
)

(defun poll-bms (conn)
    (var info (bms-request conn bms-req-info 3.0))
    (if info
        (progn
            (set-bms-val 'bms-v-tot (parse-v-tot info))
            (set-bms-val 'bms-i-in (parse-i-in info))
            (set-bms-val 'bms-soc (parse-rsoc info))
            (set-bms-val 'bms-temp-cell-max (parse-ntc-max info))
            (set-bms-val 'bms-chg-allowed (if (= 0 (bitwise-and (parse-fet-status info) 1)) 0 1))
            (print (list "bms v" (parse-v-tot info) "i" (parse-i-in info) "soc" (parse-rsoc info)))
        )
        (print "BMS: no response to info request")
    )
    (var cells (bms-request conn bms-req-cells 3.0))
    (if cells
        (bms-apply-cells cells)
        (print "BMS: no response to cell voltage request")
    )
)

(defun scan-for-bms ()
    (print "Scanning for a BLE BMS (name containing \"bms\", case insensitive)...")
    (event-enable 'event-ble-scan)
    (ble-client-scan-start 15)
    (var found nil)
    (loopwhile (and (not found) (ble-client-scanning))
        (recv-to 0.5
            ((event-ble-scan (? a) (? rssi) (? name))
                (if (and name (>= (str-find (str-to-lower name) "bms") 0))
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

(defun connect-bms ()
    (var addr (if bms-addr bms-addr (scan-for-bms)))
    (if (not addr)
        (progn
            (print "No BMS found. Make sure it's powered, in range, and not already connected to its phone app - retrying.")
            (sleep 2.0)
        )
        (progn
            (print (list "Connecting to" addr "..."))
            (var conn (ble-client-connect addr bms-addr-type))
            (if conn
                (let ((handle (ble-client-subscribe conn bms-service bms-char-notify)))
                    (if handle
                        (progn
                            (print (list "Connected and subscribed, conn-handle" conn))
                            (def bms-conn conn)
                            (def bms-notify-handle handle)
                        )
                        (progn
                            (print "Could not subscribe to the BMS notify characteristic.")
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
        (if (not bms-conn)
            (connect-bms)
            (progn
                (poll-bms bms-conn)
                (if (not (ble-client-connected bms-conn))
                    (progn
                        (print "BMS disconnected, reconnecting...")
                        (def bms-conn nil)
                        (def bms-notify-handle nil)
                    )
                    (sleep poll-interval-s)
                )
            )
        )
    )
)

(ble-start-app)
(event-register-handler (spawn app-loop))
(event-enable 'event-ble-client-connect)
(event-enable 'event-ble-client-data)
