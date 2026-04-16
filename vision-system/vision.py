# Dokumentasi Program Vision sebelumnya
import os
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
import time
import math
import datetime
from collections import deque

# ============================================================
# KONFIGURASI ARENA & GRID (DALAM CM)
# ============================================================
GRID_CELL_CM = 5   # Default: 15cm grid
AVAILABLE_GRID_SIZES = [15.0, 10.0, 5.0]

ARENA_W_CM = 200.0  # 2 meter
ARENA_H_CM = 150.0  # 1.5 meter

def calculate_grid_dimensions():
    """Calculate grid dimensions based on current cell size"""
    global GRID_COLS, GRID_ROWS
    GRID_COLS = int(ARENA_W_CM / GRID_CELL_CM)
    GRID_ROWS = int(ARENA_H_CM / GRID_CELL_CM)
    return GRID_COLS, GRID_ROWS

GRID_COLS = int(ARENA_W_CM / GRID_CELL_CM)
GRID_ROWS = int(ARENA_H_CM / GRID_CELL_CM)

# ============================================================
# PARAMETER SWARM / FOLLOWER
# ============================================================
R_IN_CM = 35.0
R_OUT_CM = 45.0
ANCHOR_DIST_CM = 15.0
ANCHOR_BETA = 0.28
ANCHOR_MAX_STEP = 8.0
THETA_REF_BETA = 0.35

LEADER_YAW_OFFSET_RAD = -math.pi / 2.0
FOLLOW_YAW_OFFSET_RAD = -math.pi / 2.0

# ============================================================
# ID ArUco & Corner Markers
# ============================================================
LEADER_ID = 1
FOLLOWER_ID = 2
ARUCO_SIZE_CM = 9.0
OFFSET_CM = ARUCO_SIZE_CM / 2.0

CORNER_MARKERS = {
    10: (0.0 + OFFSET_CM, 0.0 + OFFSET_CM),           # Bottom-left
    11: (ARENA_W_CM - OFFSET_CM, 0.0 + OFFSET_CM),    # Bottom-right
    12: (ARENA_W_CM - OFFSET_CM, ARENA_H_CM - OFFSET_CM),  # Top-right
    13: (0.0 + OFFSET_CM, ARENA_H_CM - OFFSET_CM),    # Top-left
}

# ============================================================
# PATH FILE OUTPUT
# ============================================================
root_dir = os.environ.get("LOCALAPPDATA", ".")
POSE_DIR = os.path.join(root_dir, "RobotData")
os.makedirs(POSE_DIR, exist_ok=True)

LEADER_POSE_FILE = os.path.join(POSE_DIR, "leader_pose.txt")
FOLLOWER_POSE_FILE = os.path.join(POSE_DIR, "follower_pose.txt")
OBSTACLE_FILE = os.path.join(POSE_DIR, "obstacle.txt")
PPG_FILE = os.path.join(POSE_DIR, "pixels_per_grid.txt")
ANCHOR_FILE = os.path.join(POSE_DIR, "leader_anchor.txt")
LEADER_PATH_FILE = os.path.join(POSE_DIR, "leader_path.txt")
GRID_CONFIG_FILE = os.path.join(POSE_DIR, "grid_config.txt")

# ============================================================
# NEW: FILE UNTUK GOAL BIRU
# ============================================================
GOAL_BLUE_FILE = os.path.join(POSE_DIR, "goal_blue.txt")

# ============================================================
# GLOBAL STATE
# ============================================================
leader_path = []
follower_path = []
show_masks = False
show_blue_mask = False  # NEW: toggle untuk mask biru

last_leader_pix = None
last_follower_pix = None
last_leader_world = None
last_follower_world = None

follower_mode = "IDLE"

_prev_theta_leader = None
_prev_theta_ref = None
_leader_hist_cm = deque(maxlen=5)
_anchor_filt_cm = None

# Homografi
H_PIX2WORLD = None
H_WORLD2PIX = None

_prev_ppg = -1
_last_ppg_write = 0.0

# NEW: State untuk deteksi goal biru
_last_blue_goal_cm = None  # (x_cm, y_cm) - posisi goal biru terakhir
_blue_goal_stable_count = 0  # Counter untuk stabilitas deteksi
BLUE_GOAL_MIN_STABLE = 5  # Minimal frame stabil sebelum ditulis ke file
_last_blue_write_time = 0.0  # Throttle penulisan file

# NEW: Print jarak leader-follower (throttle)
_last_dist_print_time = 0.0
DIST_PRINT_INTERVAL_S = 0.5  # max 2 Hz


# ============================================================
# DICTIONARY ARUCO
# ============================================================
DICT_LIST = {
    "1": aruco.DICT_4X4_50,
    "2": aruco.DICT_4X4_100,
    "3": aruco.DICT_4X4_1000,
    "4": aruco.DICT_6X6_250,
    "5": aruco.DICT_7X7_250,
}

# ============================================================
# UTILITY FUNCTIONS
# ============================================================

def load_hsv_settings_yaml(filename: str):
    if os.path.exists(filename):
        with open(filename, "r") as f:
            try:
                return yaml.safe_load(f)
            except Exception:
                return {}
    return {}

def save_hsv_settings_yaml(filename: str, settings: dict):
    try:
        with open(filename, "w") as f:
            yaml.dump(settings, f)
    except Exception as e:
        print(f"[WARN] Gagal menyimpan HSV settings: {e}")

def get_trackbar_values(window_name: str):
    h_min = cv2.getTrackbarPos("H_min", window_name)
    s_min = cv2.getTrackbarPos("S_min", window_name)
    v_min = cv2.getTrackbarPos("V_min", window_name)
    h_max = cv2.getTrackbarPos("H_max", window_name)
    s_max = cv2.getTrackbarPos("S_max", window_name)
    v_max = cv2.getTrackbarPos("V_max", window_name)
    return np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max])

def setup_trackbar(window_name: str, settings: dict, default_range: dict):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 600, 220)
    values = settings.get(window_name, default_range)
    cv2.createTrackbar("H_min", window_name, values.get("H_min", 0), 179, lambda x: None)
    cv2.createTrackbar("S_min", window_name, values.get("S_min", 0), 255, lambda x: None)
    cv2.createTrackbar("V_min", window_name, values.get("V_min", 0), 255, lambda x: None)
    cv2.createTrackbar("H_max", window_name, values.get("H_max", 179), 179, lambda x: None)
    cv2.createTrackbar("S_max", window_name, values.get("S_max", 255), 255, lambda x: None)
    cv2.createTrackbar("V_max", window_name, values.get("V_max", 255), 255, lambda x: None)

# ============================================================
# I/O ATOMIC
# ============================================================

def save_atomic(filepath: str, content: str):
    tmp = filepath + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(content)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, filepath)
    except PermissionError:
        print(f"[WARN] Gagal menulis {filepath} (locked).")

def save_pose(filepath: str, x_cm: float, y_cm: float, theta_rad: float):
    save_atomic(filepath, f"{x_cm:.3f},{y_cm:.3f},{theta_rad:.6f}\n")

def save_obstacles(obstacles_list):
    lines = ""
    for x_cm, y_cm in obstacles_list:
        lines += f"{x_cm:.3f},{y_cm:.3f}\n"
    save_atomic(OBSTACLE_FILE, lines)

def save_anchor(x_cm: float, y_cm: float):
    save_atomic(ANCHOR_FILE, f"{x_cm:.3f},{y_cm:.3f}\n")

def save_leader_pos_to_file(pos_cm):
    x_cm, y_cm = pos_cm
    with open(LEADER_PATH_FILE, "a") as f:
        f.write(f"{x_cm:.3f},{y_cm:.3f}\n")

def write_ppg(ppg: int):
    global _last_ppg_write, _prev_ppg
    now = time.time()
    if now - _last_ppg_write < 0.5:
        return
    _last_ppg_write = now
    _prev_ppg = ppg
    save_atomic(PPG_FILE, str(int(ppg)))

def print_leader_follower_distance(dist_cm: float):
    global _last_dist_print_time
    now = time.time()
    if now - _last_dist_print_time < DIST_PRINT_INTERVAL_S:
        return
    _last_dist_print_time = now
    print(f"[DIST] Leader-Follower = {dist_cm:.1f} cm")

# ============================================================
# NEW: SAVE GOAL BIRU
# ============================================================

def save_blue_goal(x_cm: float, y_cm: float):
    """Simpan posisi goal biru ke file (format: x_cm,y_cm)"""
    global _last_blue_write_time
    
    now = time.time()
    # Throttle: jangan tulis terlalu sering (max 2 Hz)
    if now - _last_blue_write_time < 0.5:
        return
    
    _last_blue_write_time = now
    save_atomic(GOAL_BLUE_FILE, f"{x_cm:.3f},{y_cm:.3f}\n")
    print(f"[BLUE GOAL] Saved: ({x_cm:.1f}, {y_cm:.1f}) cm")

def clear_blue_goal():
    """Hapus file goal biru (ketika tidak terdeteksi)"""
    try:
        if os.path.exists(GOAL_BLUE_FILE):
            os.remove(GOAL_BLUE_FILE)
            print("[BLUE GOAL] File cleared")
    except Exception:
        pass

# ============================================================
# GRID CONFIG
# ============================================================

def load_grid_config():
    global GRID_CELL_CM, GRID_COLS, GRID_ROWS
    try:
        if os.path.exists(GRID_CONFIG_FILE):
            with open(GRID_CONFIG_FILE, "r") as f:
                content = f.read().strip()
                if content:
                    new_size = float(content)
                    if new_size in AVAILABLE_GRID_SIZES:
                        GRID_CELL_CM = new_size
                        GRID_COLS, GRID_ROWS = calculate_grid_dimensions()
                        print(f"[GRID CONFIG] Loaded: {GRID_CELL_CM}cm -> {GRID_COLS}x{GRID_ROWS} grid")
                        return True
    except Exception as e:
        print(f"[GRID CONFIG] Error loading: {e}")
    return False

def save_grid_config(cell_size_cm: float):
    save_atomic(GRID_CONFIG_FILE, f"{cell_size_cm:.1f}\n")
    print(f"[GRID CONFIG] Saved: {cell_size_cm}cm")

# ============================================================
# GEOMETRI & SUDUT
# ============================================================

def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a

def smooth_angle(prev: float, new: float, alpha: float = 0.3) -> float:
    if prev is None:
        return new
    delta = wrap_pi(new - prev)
    return wrap_pi(prev + alpha * delta)

def calculate_theta(p1, p2):
    dx = p2[0] - p1[0]
    dy = p1[1] - p2[1]
    rad = math.atan2(dy, dx)
    deg = math.degrees(rad)
    if deg < 0:
        deg += 360
    return deg

def lp_point(prev, new, beta: float):
    if prev is None:
        return new
    return (prev[0] + beta * (new[0] - prev[0]),
            prev[1] + beta * (new[1] - prev[1]))

def limit_step(prev, new, max_step: float):
    if prev is None:
        return new
    dx = new[0] - prev[0]
    dy = new[1] - prev[1]
    dist = math.hypot(dx, dy)
    if dist <= max_step or dist == 0:
        return new
    scale = max_step / dist
    return (prev[0] + dx * scale, prev[1] + dy * scale)

# ============================================================
# HOMOGRAFI PIXEL <-> WORLD (CM)
# ============================================================

def update_homography(corner_pixels: dict):
    global H_PIX2WORLD, H_WORLD2PIX

    if not all(mid in corner_pixels for mid in CORNER_MARKERS):
        return

    img_pts = []
    wrd_pts = []
    for mid in CORNER_MARKERS:
        (u, v) = corner_pixels[mid]
        img_pts.append([u, v])
        wrd_pts.append([CORNER_MARKERS[mid][0], CORNER_MARKERS[mid][1]])

    img_pts = np.float32(img_pts)
    wrd_pts = np.float32(wrd_pts)

    H, _ = cv2.findHomography(img_pts, wrd_pts, method=0)
    if H is None:
        print("[H] Gagal menghitung homografi.")
        return

    H_PIX2WORLD = H
    try:
        H_WORLD2PIX = np.linalg.inv(H)
    except np.linalg.LinAlgError:
        H_WORLD2PIX = None
        print("[H] Homografi singular.")
        return

    print("[H] Homografi diperbarui.")

    mid_y = ARENA_H_CM * 0.5
    p0 = world_to_pixel(0.0, mid_y)
    p1 = world_to_pixel(GRID_CELL_CM, mid_y)
    if p0 is not None and p1 is not None:
        ppg = int(round(math.hypot(p1[0] - p0[0], p1[1] - p0[1])))
        if ppg > 0:
            print(f"[PPG] pixels_per_grid = {ppg}")
            write_ppg(ppg)

def pixel_to_world(x_pix: int, y_pix: int):
    if H_PIX2WORLD is None:
        return None, None
    vec = np.array([x_pix, y_pix, 1.0], dtype=np.float32)
    w = H_PIX2WORLD @ vec
    if abs(w[2]) < 1e-6:
        return None, None
    xw = w[0] / w[2]
    yw = w[1] / w[2]
    return float(xw), float(yw)

def world_to_pixel(x_cm: float, y_cm: float):
    if H_WORLD2PIX is None:
        return None
    vec = np.array([x_cm, y_cm, 1.0], dtype=np.float32)
    p = H_WORLD2PIX @ vec
    if abs(p[2]) < 1e-6:
        return None
    u = p[0] / p[2]
    v = p[1] / p[2]
    return (int(round(u)), int(round(v)))

# ============================================================
# DRAWING UTILITIES
# ============================================================

def draw_arrow(frame, start_point, end_point, color=(0, 255, 255), thickness=2, tip_length=0.3):
    cv2.arrowedLine(frame, start_point, end_point, color, thickness, tipLength=tip_length)

def draw_world_grid(frame):
    if H_WORLD2PIX is None:
        return

    for r in range(GRID_ROWS + 1):
        y_cm = r * GRID_CELL_CM
        p0 = world_to_pixel(0.0, y_cm)
        p1 = world_to_pixel(ARENA_W_CM, y_cm)
        if p0 is not None and p1 is not None:
            cv2.line(frame, p0, p1, (200, 200, 200), 1, cv2.LINE_AA)

    for c in range(GRID_COLS + 1):
        x_cm = c * GRID_CELL_CM
        p0 = world_to_pixel(x_cm, 0.0)
        p1 = world_to_pixel(x_cm, ARENA_H_CM)
        if p0 is not None and p1 is not None:
            cv2.line(frame, p0, p1, (200, 200, 200), 1, cv2.LINE_AA)

# ============================================================
# ARUCO DETECTION & POSE
# ============================================================

def detect_aruco_markers_core(frame, corners, ids):
    global leader_path, follower_path
    global last_leader_pix, last_follower_pix
    global last_leader_world, last_follower_world
    global follower_mode
    global _prev_theta_leader, _prev_theta_ref, _leader_hist_cm, _anchor_filt_cm

    if ids is None:
        return frame

    aruco.drawDetectedMarkers(frame, corners, ids)

    detected = {}
    corner_pixels = {}

    for i, mid_arr in enumerate(ids):
        mid = int(mid_arr[0])
        pts = corners[i][0]
        cx_pix = int(np.mean(pts[:, 0]))
        cy_pix = int(np.mean(pts[:, 1]))
        detected[mid] = (cx_pix, cy_pix, pts)

        if mid in CORNER_MARKERS:
            corner_pixels[mid] = (cx_pix, cy_pix)

    if len(corner_pixels) == len(CORNER_MARKERS):
        update_homography(corner_pixels)

    for mid, (cx_pix, cy_pix, pts) in detected.items():
        color = (255, 0, 0) if mid == LEADER_ID else (0, 0, 255)
        cv2.circle(frame, (cx_pix, cy_pix), 5, color, -1)

        if mid not in (LEADER_ID, FOLLOWER_ID):
            continue

        x_cm, y_cm = pixel_to_world(cx_pix, cy_pix)
        if x_cm is None or y_cm is None:
            continue

        tl = pts[0]
        tr = pts[1]
        vx_img = tr[0] - tl[0]
        vy_img = tr[1] - tl[1]

        theta = math.atan2(-vy_img, vx_img)
        if mid == LEADER_ID:
            theta = wrap_pi(theta + LEADER_YAW_OFFSET_RAD)
            theta = smooth_angle(_prev_theta_leader, theta, alpha=0.30)
            _prev_theta_leader = theta
        else:
            theta = wrap_pi(theta + FOLLOW_YAW_OFFSET_RAD)
            theta = smooth_angle(None, theta, alpha=0.30)

        if mid == LEADER_ID:
            save_pose(LEADER_POSE_FILE, x_cm, y_cm, theta)
            last_leader_world = (x_cm, y_cm)
            last_leader_pix = (cx_pix, cy_pix)
            leader_path.append(last_leader_pix)
            save_leader_pos_to_file((x_cm, y_cm))
        else:
            save_pose(FOLLOWER_POSE_FILE, x_cm, y_cm, theta)
            last_follower_world = (x_cm, y_cm)
            last_follower_pix = (cx_pix, cy_pix)
            follower_path.append(last_follower_pix)

        L = 30
        theta_img = -theta
        end_x = int(cx_pix + L * math.cos(theta_img))
        end_y = int(cy_pix + L * math.sin(theta_img))
        # draw_arrow(frame, (cx_pix, cy_pix), (end_x, end_y))

        if mid == LEADER_ID:
            _leader_hist_cm.append((x_cm, y_cm))

            theta_ref = None
            if len(_leader_hist_cm) >= 3:
                x1, y1 = _leader_hist_cm[-3]
                x3, y3 = _leader_hist_cm[-1]
                dx = x3 - x1
                dy = y3 - y1
                if abs(dx) + abs(dy) > 1e-4:
                    theta_ref = math.atan2(dy, dx)
            if theta_ref is None:
                theta_ref = theta

            theta_ref = smooth_angle(_prev_theta_ref, theta_ref, alpha=THETA_REF_BETA)
            _prev_theta_ref = theta_ref

            OFFSET_CM2 = -1.5
            ax_raw = x_cm - OFFSET_CM2 * math.cos(theta_ref)
            ay_raw = y_cm - OFFSET_CM2 * math.sin(theta_ref)
            ax_lim, ay_lim = limit_step(_anchor_filt_cm, (ax_raw, ay_raw), ANCHOR_MAX_STEP)
            _anchor_filt_cm = lp_point(_anchor_filt_cm, (ax_lim, ay_lim), ANCHOR_BETA)

            save_anchor(_anchor_filt_cm[0], _anchor_filt_cm[1])

            apix = world_to_pixel(_anchor_filt_cm[0], _anchor_filt_cm[1])
            if apix is not None:
                cv2.circle(frame, apix, 5, (0, 255, 0), -1)
                cv2.line(frame, (cx_pix, cy_pix), apix, (0, 200, 0), 1)

                outer = world_to_pixel(x_cm + R_OUT_CM, y_cm)
                if outer is not None:
                    radius_px = int(round(math.hypot(outer[0] - cx_pix, outer[1] - cy_pix)))
                    cv2.circle(frame, (cx_pix, cy_pix), radius_px, (0, 255, 255), 2)

        if mid == FOLLOWER_ID and last_leader_world is not None:
            fx, fy = x_cm, y_cm
            lx, ly = last_leader_world
            dist_cm = math.hypot(fx - lx, fy - ly)
            print_leader_follower_distance(dist_cm)


            if follower_mode == "FOLLOW":
                if dist_cm >= R_OUT_CM:
                    follower_mode = "CATCH_UP"
            else:
                if dist_cm <= R_IN_CM:
                    follower_mode = "FOLLOW"

            try:
                if os.path.exists(ANCHOR_FILE):
                    with open(ANCHOR_FILE, "r") as f:
                        line = f.readlines()[-1].strip()
                    ax_cm, ay_cm = [float(t) for t in line.split(",")]
                    apix = world_to_pixel(ax_cm, ay_cm)
                    if apix is not None:
                        draw_arrow(frame, (cx_pix, cy_pix), apix)
            except Exception:
                pass

    return frame

# ============================================================
# NEW: DETEKSI WARNA BIRU UNTUK GOAL
# ============================================================

def detect_blue_goal(frame, hsv, lower_blue, upper_blue):
    """
    Deteksi objek biru terbesar sebagai goal.
    Return: (x_cm, y_cm, x_pix, y_pix) atau None jika tidak terdeteksi
    """
    global _last_blue_goal_cm, _blue_goal_stable_count
    
    # Buat mask untuk warna biru
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Morphological operations untuk bersihkan noise
    kernel = np.ones((5, 5), np.uint8)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
    mask_blue = cv2.dilate(mask_blue, kernel, iterations=1)
    
    # Cari contours
    contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        # Tidak ada biru terdeteksi
        _blue_goal_stable_count = 0
        return mask_blue, None
    
    # Cari contour terbesar
    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)
    
    # Filter area minimum (untuk menghindari noise)
    MIN_BLUE_AREA = 200  # pixel^2
    if area < MIN_BLUE_AREA:
        _blue_goal_stable_count = 0
        return mask_blue, None
    
    # Hitung centroid
    M = cv2.moments(largest_contour)
    if M["m00"] == 0:
        return mask_blue, None
    
    cx_pix = int(M["m10"] / M["m00"])
    cy_pix = int(M["m01"] / M["m00"])
    
    # Konversi ke koordinat dunia (cm)
    x_cm, y_cm = pixel_to_world(cx_pix, cy_pix)
    
    if x_cm is None or y_cm is None:
        return mask_blue, None
    
    # Validasi: harus di dalam arena
    if x_cm < 0 or x_cm > ARENA_W_CM or y_cm < 0 or y_cm > ARENA_H_CM:
        return mask_blue, None
    
    # Cek stabilitas (posisi tidak berubah drastis)
    if _last_blue_goal_cm is not None:
        dist = math.hypot(x_cm - _last_blue_goal_cm[0], y_cm - _last_blue_goal_cm[1])
        if dist < 10.0:  # Dalam radius 10cm dari posisi sebelumnya
            _blue_goal_stable_count += 1
        else:
            _blue_goal_stable_count = 1
    else:
        _blue_goal_stable_count = 1
    
    _last_blue_goal_cm = (x_cm, y_cm)
    
    # Gambar di frame
    x, y, w, h = cv2.boundingRect(largest_contour)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 1)  # Biru tebal
    cv2.circle(frame, (cx_pix, cy_pix), 8, (255, 0, 0), -1)  # Center point
    
    
    
    # Gambar crosshair di goal
    cv2.line(frame, (cx_pix - 15, cy_pix), (cx_pix + 15, cy_pix), (255, 0, 0), 2)
    cv2.line(frame, (cx_pix, cy_pix - 15), (cx_pix, cy_pix + 15), (255, 0, 0), 2)
    
    return mask_blue, (x_cm, y_cm, cx_pix, cy_pix, area)

# ============================================================
# MAIN LOOP
# ============================================================

def detect_colors():
    global show_masks, show_blue_mask, leader_path, last_leader_pix
    global GRID_CELL_CM, GRID_COLS, GRID_ROWS
    global _blue_goal_stable_count
    
    # Coba beberapa index kamera
    cap = None
    for cam_idx in [0, 1, 2]:
        cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            print(f"[CAMERA] Opened camera index {cam_idx}")
            break
        cap.release()
    
    if cap is None or not cap.isOpened():
        print("❌ Kamera gagal dibuka.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    settings_file = "hsv_settings.yaml"
    saved_settings = load_hsv_settings_yaml(settings_file)

    # Trackbar untuk KUNING (obstacle)
    setup_trackbar("Trackbars Yellow", saved_settings, {
        "H_min": 20, "S_min": 120, "V_min": 120,
        "H_max": 30, "S_max": 255, "V_max": 255
    })

    # NEW: Trackbar untuk BIRU (goal)
    setup_trackbar("Trackbars Blue", saved_settings, {
        "H_min": 100, "S_min": 120, "V_min": 70,
        "H_max": 130, "S_max": 255, "V_max": 255
    })

    # ArUco detector
    current_dict_key = "3"
    dictionary = aruco.getPredefinedDictionary(DICT_LIST[current_dict_key])
    try:
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, parameters)
    except AttributeError:
        parameters = aruco.DetectorParameters_create()
        detector = None

    prev_time = time.time()
    frame_count = 0

    load_grid_config()
    print(f"[VISION] Starting with grid: {GRID_CELL_CM}cm ({GRID_COLS}x{GRID_ROWS})")

    open(LEADER_PATH_FILE, "w").close()
    
    print("\n" + "="*50)
    print("VISION SYSTEM WITH BLUE GOAL DETECTION")
    print("="*50)
    print("Keyboard shortcuts:")
    print("  'q' - Quit")
    print("  'm' - Toggle yellow mask")
    print("  'b' - Toggle blue mask")
    print("  'g' - Cycle grid size")
    print("  's' - Save screenshot")
    print("  'r' - Reset leader path")
    print("  'c' - Clear blue goal file")
    print("  '1-5' - Change ArUco dictionary")
    print("="*50 + "\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        now = time.time()
        fps = 1.0 / max(now - prev_time, 1e-6)
        prev_time = now

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ========== 1) DETEKSI OBSTACLE KUNING ==========
        lower_yellow, upper_yellow = get_trackbar_values("Trackbars Yellow")
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        small_kernel = np.ones((3, 3), np.uint8)
        mask_temp = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, small_kernel)
        mask_temp = cv2.erode(mask_temp, small_kernel, iterations=1)
        mask_temp = cv2.dilate(mask_temp, small_kernel, iterations=1)

        contours_temp, _ = cv2.findContours(mask_temp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        avg_area = (sum(cv2.contourArea(c) for c in contours_temp) /
                    max(len(contours_temp), 1) if contours_temp else 0)

        if avg_area < 800:
            kernel = np.ones((3, 3), np.uint8)
            mask_yellow = cv2.erode(mask_temp, kernel, iterations=2)
            mask_yellow = cv2.dilate(mask_yellow, kernel, iterations=2)

        else:
            kernel = np.ones((4, 4), np.uint8)
            mask_yellow = cv2.erode(mask_temp, kernel, iterations=1)
            mask_yellow = cv2.dilate(mask_temp, kernel, iterations=2)
            mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)

        contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        obstacles_with_area = []

        for cnt in contours_yellow:
            area = cv2.contourArea(cnt)
            if area < 50:
                continue
            x, y, wbox, hbox = cv2.boundingRect(cnt)
            cx_pix = x + wbox // 2
            cy_pix = y + hbox // 2

            x_cm, y_cm = pixel_to_world(cx_pix, cy_pix)
            if x_cm is None or y_cm is None:
                continue

            obstacles_with_area.append((x_cm, y_cm, area))
            cv2.rectangle(frame, (x, y), (x + wbox, y + hbox), (0, 255, 255), 2)

        def dedupe_obstacles(obs_list, min_dist_cm=15.0):
            if len(obs_list) <= 1:
                return [(x, y) for x, y, _ in obs_list]
            deduped = []
            for x, y, area in obs_list:
                replaced = False
                for i, (ex, ey, earea) in enumerate(deduped):
                    dist = math.hypot(x - ex, y - ey)
                    if dist < min_dist_cm:
                        if area > earea:
                            deduped[i] = (x, y, area)
                        replaced = True
                        break
                if not replaced:
                    deduped.append((x, y, area))
            return [(x, y) for x, y, _ in deduped]

        obstacles_cm = dedupe_obstacles(obstacles_with_area, min_dist_cm=GRID_CELL_CM)
        print("raw:", len(obstacles_with_area), "deduped:", len(obstacles_cm))
        
        if frame_count % 5 == 0 and obstacles_cm:
            save_obstacles(obstacles_cm)

        # ========== 2) NEW: DETEKSI GOAL BIRU ==========
        lower_blue, upper_blue = get_trackbar_values("Trackbars Blue")
        mask_blue, blue_result = detect_blue_goal(frame, hsv, lower_blue, upper_blue)
        
        if blue_result is not None:
            x_cm, y_cm, cx_pix, cy_pix, area = blue_result
            
            # Simpan ke file jika sudah stabil
            if _blue_goal_stable_count >= BLUE_GOAL_MIN_STABLE:
                save_blue_goal(x_cm, y_cm)
                
                

        # ========== 3) DETEKSI ARUCO ==========
        if frame_count % 3 == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            try:
                parameters = aruco.DetectorParameters()
                detector = aruco.ArucoDetector(
                    aruco.getPredefinedDictionary(DICT_LIST[current_dict_key]),
                    parameters
                )
                corners, ids, _ = detector.detectMarkers(gray)
            except AttributeError:
                corners, ids, _ = aruco.detectMarkers(
                    gray,
                    aruco.getPredefinedDictionary(DICT_LIST[current_dict_key]),
                    parameters=None
                )

            if ids is not None:
                frame = detect_aruco_markers_core(frame, corners, ids)

        # ========== 4) VISUALISASI TRAIL LEADER & GRID ==========
        for i in range(1, len(leader_path)):
            cv2.line(frame, leader_path[i - 1], leader_path[i], (0, 255, 0), 2)

        # if len(leader_path) >= 2:
        #     draw_arrow(frame, leader_path[-2], leader_path[-1])

        draw_world_grid(frame)

        # ========== 5) INFO TEXT ==========
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        hom_status = "READY" if H_PIX2WORLD is not None else "CALIBRATING..."
        cv2.putText(frame, f"Homography: {hom_status}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.putText(frame, f"Grid: {GRID_CELL_CM:.0f}cm ({GRID_COLS}x{GRID_ROWS})", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        
        # Status goal biru
        if blue_result is not None:
            x_cm, y_cm, _, _, area = blue_result
            status = f"BLUE GOAL: ({x_cm:.0f}, {y_cm:.0f}) cm "
            color = (230, 100, 20) 
        else:
            status = "BLUE GOAL: Not detected"
            color = (0, 0, 255)
        cv2.putText(frame, status, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # ========== 6) SHOW WINDOWS ==========
        cv2.imshow("Vision - Blue Goal Detection", frame)
        
        if show_masks:
            cv2.imshow("Mask Yellow (Obstacles)", mask_yellow)
        else:
            try:
                if cv2.getWindowProperty("Mask Yellow (Obstacles)", cv2.WND_PROP_VISIBLE) >= 1:
                    cv2.destroyWindow("Mask Yellow (Obstacles)")
            except Exception:
                pass
        
        if show_blue_mask:
            cv2.imshow("Mask Blue (Goal)", mask_blue)
        else:
            try:
                if cv2.getWindowProperty("Mask Blue (Goal)", cv2.WND_PROP_VISIBLE) >= 1:
                    cv2.destroyWindow("Mask Blue (Goal)")
            except Exception:
                pass

        # ========== 7) KEYBOARD INPUT ==========
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            # Save HSV settings sebelum keluar
            current_settings = {
                "Trackbars Yellow": {
                    "H_min": cv2.getTrackbarPos("H_min", "Trackbars Yellow"),
                    "S_min": cv2.getTrackbarPos("S_min", "Trackbars Yellow"),
                    "V_min": cv2.getTrackbarPos("V_min", "Trackbars Yellow"),
                    "H_max": cv2.getTrackbarPos("H_max", "Trackbars Yellow"),
                    "S_max": cv2.getTrackbarPos("S_max", "Trackbars Yellow"),
                    "V_max": cv2.getTrackbarPos("V_max", "Trackbars Yellow"),
                },
                "Trackbars Blue": {
                    "H_min": cv2.getTrackbarPos("H_min", "Trackbars Blue"),
                    "S_min": cv2.getTrackbarPos("S_min", "Trackbars Blue"),
                    "V_min": cv2.getTrackbarPos("V_min", "Trackbars Blue"),
                    "H_max": cv2.getTrackbarPos("H_max", "Trackbars Blue"),
                    "S_max": cv2.getTrackbarPos("S_max", "Trackbars Blue"),
                    "V_max": cv2.getTrackbarPos("V_max", "Trackbars Blue"),
                }
            }
            save_hsv_settings_yaml(settings_file, current_settings)
            print("[HSV] Settings saved to hsv_settings.yaml")
            break
            
        elif key == ord('m'):
            show_masks = not show_masks
            print(f"[MASK] Yellow mask: {'ON' if show_masks else 'OFF'}")
            
        elif key == ord('b'):
            show_blue_mask = not show_blue_mask
            print(f"[MASK] Blue mask: {'ON' if show_blue_mask else 'OFF'}")
            
        elif key == ord('g'):
            current_idx = AVAILABLE_GRID_SIZES.index(GRID_CELL_CM) if GRID_CELL_CM in AVAILABLE_GRID_SIZES else 0
            next_idx = (current_idx + 1) % len(AVAILABLE_GRID_SIZES)
            GRID_CELL_CM = AVAILABLE_GRID_SIZES[next_idx]
            GRID_COLS, GRID_ROWS = calculate_grid_dimensions()
            save_grid_config(GRID_CELL_CM)
            print(f"[GRID] Changed to {GRID_CELL_CM}cm -> {GRID_COLS}x{GRID_ROWS} grid")
            
        elif key == ord('s'):
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            os.makedirs("captures", exist_ok=True)
            filename = os.path.join("captures", f"frame_{timestamp}.png")
            cv2.imwrite(filename, frame)
            print(f"[SAVE] Frame saved: {filename}")
            
        elif key == ord('r'):
            leader_path.clear()
            last_leader_pix = None
            open(LEADER_PATH_FILE, "w").close()
            print("[RESET] Leader path cleared")
            
        elif key == ord('c'):
            clear_blue_goal()
            _blue_goal_stable_count = 0
            print("[CLEAR] Blue goal file cleared")
            
        elif chr(key) in DICT_LIST:
            current_dict_key = chr(key)
            print(f"[ARUCO] Dictionary changed to: {current_dict_key}")

    cap.release()
    cv2.destroyAllWindows()

# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    detect_colors()
