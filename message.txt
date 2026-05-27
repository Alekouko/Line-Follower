from machine import Pin, PWM
import time

# --- ΚΙΝΗΤΗΡΕΣ ---
in1 = Pin(2, Pin.OUT)  # Αριστερό πίσω
in2 = Pin(3, Pin.OUT)  # Αριστερό μπροστά
in3 = Pin(4, Pin.OUT)  # Δεξί μπροστά
in4 = Pin(9, Pin.OUT)  # Δεξί πίσω

pwm_a = PWM(Pin(6)); pwm_a.freq(1000)  # ENA
pwm_b = PWM(Pin(7)); pwm_b.freq(1000)  # ENB

# --- ΣΥΝΤΕΛΕΣΤΗΣ ΒΑθΜΟΝΟΜΗΣΗΣ (CALIBRATION) ---
# Επειδή το δεξί μοτέρ είναι πιο γρήγορο, πολλαπλασιάζουμε την ταχύτητά του με 0.83
# για να μειωθεί η ισχύς του και να πηγαίνει το ρομπότ ομοιόμορφα στην ευθεία.
RIGHT_MOTOR_CALIBRATION = 0.83  

# --- ΤΑΧΥΤΗΤΕΣ ---
BASE_SPEED  = 30000  # Κοινή βασική ταχύτητα για την ευθεία
TURN_SPEED  = 18000  # Ταχύτητα για την επιτόπου στροφή στο Recovery (αν παρασύρεται, μείωσέ το σε 15000)

# --- PID ΠΑΡΑΜΕΤΡΟΙ ---
Kp = 1500.0
Ki = 0.0
Kd = 10000.0

# --- SENSORS ---
sensor_L = Pin(10, Pin.IN)  # Αριστερός
sensor_M = Pin(11, Pin.IN)  # Μεσαίος
sensor_R = Pin(13, Pin.IN)  # Δεξιός

def set_motors(left_speed, right_speed):
    # Εφαρμογή του calibration ΜΟΝΟ στο δεξί μοτέρ για εξίσωση των δυνάμεων
    right_speed = right_speed * RIGHT_MOTOR_CALIBRATION

    # Διαχείριση Αριστερού Μοτέρ (Μπροστά / Πίσω)
    if left_speed >= 0:
        in1.value(0); in2.value(1)
    else:
        in1.value(1); in2.value(0)
        left_speed = abs(left_speed)
        
    # Διαχείριση Δεξιού Μοτέρ (Μπροστά / Πίσω)
    if right_speed >= 0:
        in3.value(1); in4.value(0)
    else:
        in3.value(0); in4.value(1)
        right_speed = abs(right_speed)
    
    # Περιορισμός τιμών PWM (0 έως 65535) για προστασία
    left_duty = max(0, min(65535, int(left_speed)))
    right_duty = max(0, min(65535, int(right_speed)))
    
    pwm_a.duty_u16(left_duty)
    pwm_b.duty_u16(right_duty)

def stop():
    in1.value(0); in2.value(0)
    in3.value(0); in4.value(0)
    pwm_a.duty_u16(0)
    pwm_b.duty_u16(0)

# --- ΜΕΤΑΒΛΗΤΕΣ PID & ΜΝΗΜΗΣ ---
last_direction = None 
previous_error = 0
integral = 0

print("Εκκίνηση σε 2 δευτερόλεπτα...")
time.sleep(2)

while True:
    # Ανάγνωση αισθητήρων (1 = Μαύρο/Γραμμή, 0 = Λευκό)
    L = sensor_L.value()
    M = sensor_M.value()
    R = sensor_R.value()
    
    print(f"L:{L} M:{M} R:{R} -> ", end="")
    
    # 1. ΠΕΡΙΠΤΩΣΗ: Όλα μαύρα (Διασταύρωση / Στοπ)
    if L == 1 and M == 1 and R == 1:
        print("Διασταύρωση / Στοπ")
        stop()
        break 

    # 2. ΠΕΡΙΠΤΩΣΗ: Όλα λευκά (Χάθηκε η γραμμή!) - Ενεργητικό Recovery με Pivot Turn
    elif L == 0 and M == 0 and R == 0:
        integral = 0  
        previous_error = 0
        if last_direction == "left":
            print("Χάθηκε η γραμμή! Επιτόπου στροφή ΑΡΙΣΤΕΡΑ...")
            # Αριστερός τροχός ανάποδα, δεξιός μπροστά
            set_motors(-TURN_SPEED, TURN_SPEED)  
        elif last_direction == "right":
            print("Χάθηκε η γραμμή! Επιτόπου στροφή ΔΕΞΙΑ...")
            # Αριστερός τροχός μπροστά, δεξιός ανάποδα
            set_motors(TURN_SPEED, -TURN_SPEED)  
        else:
            print("Χάθηκε η γραμμή χωρίς ιστορικό -> Στοπ")
            stop()
            break

    # 3. ΠΕΡΙΠΤΩΣΗ: Κανονική λειτουργία PID
    else:
        if M == 1 and L == 0 and R == 0:
            error = 0
        elif L == 1 and M == 0 and R == 0:
            error = -2
            last_direction = "left"
        elif R == 1 and M == 0 and L == 0:
            error = 2
            last_direction = "right"
        elif L == 1 and M == 1 and R == 0:
            error = -1
            last_direction = "left"
        elif R == 1 and M == 1 and L == 0:
            error = 1
            last_direction = "right"
        else:
            error = previous_error

        # Αλγόριθμος PID
        integral += error
        integral = max(-10, min(10, integral)) # Anti-windup
        
        derivative = error - previous_error
        correction = (Kp * error) + (Ki * integral) + (Kd * derivative)
        previous_error = error

        # Εφαρμογή της διόρθωσης πάνω στην κοινή BASE_SPEED
        left_speed  = BASE_SPEED + correction
        right_speed = BASE_SPEED - correction

        print(f"Err:{error} | Cor:{correction:.1f} | L_Spd:{int(left_speed)} R_Spd:{int(right_speed)}")
        set_motors(left_speed, right_speed)

    time.sleep_ms(10)