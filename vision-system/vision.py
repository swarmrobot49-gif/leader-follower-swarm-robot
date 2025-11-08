import cv2
import numpy as np
import socket
import yaml
import os
import time
import math
import datetime
import json
import cv2.aruco as aruco
from collections import deque

# =====================================================
# Konfigurasi GRID (fix): 13 x 10 dengan 15 cm per sel
# =====================================================
GRID_COLS = 13
GRID_ROWS = 10
GRID_CELL_CM = 15  # jarak antar garis grid (cm)

# Variabel Global
leader_path = []
follower_path = []
show_masks = False
last_leader_pos = None
last_follower_pos = None

# Nilai default lama (tidak lagi dipakai untuk konversi cm↔px)
PIXEL_PER_CM = 25.3 / 10.0

DETECTION_RADIUS_CM = 40
follower_should_follow = False
follower_mode = "IDLE"  # bisa IDLE, FOLLOW, atau CATCH_UP

LEADER_YAW_OFFSET_RAD = -math.pi/2
FOLLOW_YAW_OFFSET_RAD = -math.pi/2

_prev_theta_leader = None
_prev_theta_follower = None

POSE_DIR = os.path.join(os.environ["LOCALAPPDATA"], "RobotData")
os.makedirs(POSE_DIR, exist_ok=True)       # folder penyimpanan
LEADER_POSE_FILE   = os.path.join(POSE_DIR, "leader_pose.txt")
FOLLOWER_POSE_FILE = os.path.join(POSE_DIR, "follower_pose.txt")
OBSTACLE_FILE      = os.path.join(POSE_DIR, "obstacles.txt")
PPG_FILE           = os.path.join(POSE_DIR, "pixels_per_grid.txt")  # untuk GUI

# Mapping dictionary ArUco
DICT_LIST = {
    "1": aruco.DICT_4X4_50,
    "2": aruco.DICT_4X4_100,
    "3": aruco.DICT_4X4_1000,
    "4": aruco.DICT_6X6_250,
    "5": aruco.DICT_7X7_250,
}

# ---------- Utilitas HSV ----------
def save_hsv_settings_yaml(filename, data):
    with open(filename, 'w') as f:
        yaml.dump(data, f)

def load_hsv_settings_yaml(filename):
    if os.path.exists(filename):
        with open(filename, 'r') as f:
            return yaml.safe_load(f)
    return {}

def get_trackbar_values(window_name):
    h_min = cv2.getTrackbarPos("H_min", window_name)
    s_min = cv2.getTrackbarPos("S_min", window_name)
    v_min = cv2.getTrackbarPos("V_min", window_name)
    h_max = cv2.getTrackbarPos("H_max", window_name)
    s_max = cv2.getTrackbarPos("S_max", window_name)
    v_max = cv2.getTrackbarPos("V_max", window_name)
    return np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max])

def setup_trackbar(window_name, settings, default_range):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 600, 220)
    values = settings.get(window_name, default_range)
    cv2.createTrackbar("H_min", window_name, values.get("H_min", 0), 179, lambda x: None)
    cv2.createTrackbar("S_min", window_name, values.get("S_min", 0), 255, lambda x: None)
    cv2.createTrackbar("V_min", window_name, values.get("V_min", 0), 255, lambda x: None)
    cv2.createTrackbar("H_max", window_name, values.get("H_max", 179), 179, lambda x: None)
    cv2.createTrackbar("S_max", window_name, values.get("S_max", 255), 255, lambda x: None)
    cv2.createTrackbar("V_max", window_name, values.get("V_max", 255), 255, lambda x: None)

# ---------- I/O pose & obstacle (atomic) ----------
def save_pose_atomic(filepath, x_cm, y_cm, theta_rad, retries=10, sleep_s=0.02):
    tmp = filepath + ".tmp"
    content = f"{x_cm:.3f},{y_cm:.3f},{theta_rad:.6f}\n"
    for _ in range(retries):
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(content)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, filepath)   # atomic swap di Windows 10+
            return
        except PermissionError:
            time.sleep(sleep_s)
    print(f"[WARN] Gagal menulis {filepath} (locked).")

def save_obstacles_atomic(obstacles_list, retries=10, sleep_s=0.02):
    tmp = OBSTACLE_FILE + ".tmp"
    content = ""
    for obs in obstacles_list:
        if isinstance(obs, (list, tuple)) and len(obs) >= 2:
            content += f"{obs[0]:.3f},{obs[1]:.3f}\n"
        else:
            continue
    for _ in range(retries):
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(content)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, OBSTACLE_FILE)
            return
        except PermissionError:
            time.sleep(sleep_s)
    print(f"[WARN] Gagal menulis {OBSTACLE_FILE} (locked).")

def save_pose(filepath, x_cm, y_cm, theta_rad):
    save_pose_atomic(filepath, x_cm, y_cm, theta_rad)

# ---------- Sudut & geometri ----------
def wrap_pi(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def smooth_angle(prev, new, alpha=0.3):
    if prev is None:
        return new
    delta = wrap_pi(new - prev)
    return wrap_pi(prev + alpha * delta)

def calculate_theta(p1, p2):
    dx = p2[0] - p1[0]
    dy = p1[1] - p2[1]  # Y terbalik (gambar)
    radians = math.atan2(dy, dx)
    degrees = math.degrees(radians)
    if degrees < 0:
        degrees += 360
    return degrees

# ---------- Grid geometry (13x10 @ 15cm) ----------
_prev_ppg = -1
_last_ppg_write = 0.0

def _write_ppg(ppg: int, path: str = PPG_FILE):
    global _last_ppg_write
    now = time.time()
    if now - _last_ppg_write < 0.5:
        return
    try:
        with open(path, "w") as f:
            f.write(str(int(ppg)))
        _last_ppg_write = now
    except Exception as e:
        print(f"[WARN] gagal menulis {path}: {e}")

def compute_grid_geometry(frame_w: int, frame_h: int):
    """
    Hitung geometri grid agar:
    - semua area 13x10 sel (masing2 15cm) masuk ke frame
    - sel square (pakai min skala X/Y)
    Return: (px_per_cm, pixels_per_grid, off_x, off_y, grid_w_px, grid_h_px)
    """
    arena_w_cm = GRID_COLS * GRID_CELL_CM
    arena_h_cm = GRID_ROWS * GRID_CELL_CM
    px_per_cm_x = frame_w / float(arena_w_cm)
    px_per_cm_y = frame_h / float(arena_h_cm)
    px_per_cm   = min(px_per_cm_x, px_per_cm_y)

    pixels_per_grid = int(round(GRID_CELL_CM * px_per_cm))
    pixels_per_grid = max(4, min(pixels_per_grid, max(frame_w, frame_h)))

    grid_w_px = GRID_COLS * pixels_per_grid
    grid_h_px = GRID_ROWS * pixels_per_grid
    off_x = (frame_w - grid_w_px) // 2
    off_y = (frame_h - grid_h_px) // 2
    return px_per_cm, pixels_per_grid, off_x, off_y, grid_w_px, grid_h_px

def draw_fixed_grid(frame, pixels_per_grid, off_x, off_y, grid_w_px, grid_h_px,
                    line_color=(200,200,200), thickness=1):
    if pixels_per_grid < 2:
        return
    # horizontal
    for r in range(GRID_ROWS + 1):
        y = off_y + r * pixels_per_grid
        cv2.line(frame, (off_x, y), (off_x + grid_w_px, y), line_color, thickness, cv2.LINE_AA)
    # vertical
    for c in range(GRID_COLS + 1):
        x = off_x + c * pixels_per_grid
        cv2.line(frame, (x, off_y), (x, off_y + grid_h_px), line_color, thickness, cv2.LINE_AA)

# ---------- Gambar util ----------
def draw_arrow(frame, start_point, end_point, color=(0, 0, 255), thickness=2, tip_length=0.3):
    cv2.arrowedLine(frame, start_point, end_point, color, thickness, tipLength=tip_length)

def save_leader_pos_to_file(pos_cm, filepath=os.path.join(POSE_DIR, "leader_path.txt")):
    with open(filepath, "a") as f:
        f.write(f"{pos_cm[0]:.2f},{pos_cm[1]:.2f}\n")

# ---------- ArUco ----------
def detect_aruco_markers_core(frame, corners, ids,
                              px_per_cm, off_x, off_y, grid_h_px,
                              detection_radius_px):
    global leader_path, last_leader_pos, follower_mode

    if ids is None:
        return frame, {}

    aruco.drawDetectedMarkers(frame, corners, ids)
    aruco_centers = {}

    for i, marker_id in enumerate(ids):
        marker_id_str = str(marker_id[0])
        pts = corners[i][0].astype(int)

        center_x_pixel = int(np.mean(pts[:, 0]))
        center_y_pixel = int(np.mean(pts[:, 1]))

        # Konversi ke cm dalam KOORDINAT ARENA (origin di kiri-bawah grid)
        center_x_cm = (center_x_pixel - off_x) / px_per_cm
        center_y_cm = (off_y + grid_h_px - center_y_pixel) / px_per_cm
        aruco_centers[marker_id_str] = (center_x_cm, center_y_cm)

        # Leader (ID=1)
        if marker_id_str == "1":
            # Sudut: pakai orientasi sisi TL->TR
            pts_f = corners[i][0]
            tl = pts_f[0]; tr = pts_f[1]
            vx_img = tr[0] - tl[0]
            vy_img = tr[1] - tl[1]

            theta_arena = math.atan2(-vy_img, vx_img)  # balik sumbu Y
            theta_arena = wrap_pi(theta_arena + LEADER_YAW_OFFSET_RAD)
            global _prev_theta_leader
            theta_arena = smooth_angle(_prev_theta_leader, theta_arena, alpha=0.30)
            _prev_theta_leader = theta_arena

            save_pose(LEADER_POSE_FILE, center_x_cm, center_y_cm, theta_arena)

            # Gambar panah arah di frame (konversi balik ke gambar: Y ke bawah)
            L = 30
            theta_img_draw = -theta_arena
            end_x = int(center_x_pixel + L * math.cos(theta_img_draw))
            end_y = int(center_y_pixel + L * math.sin(theta_img_draw))
            cv2.arrowedLine(frame, (center_x_pixel, center_y_pixel), (end_x, end_y),
                            (0, 255, 255), 2, tipLength=0.35)

            leader_pos_pixels = (center_x_pixel, center_y_pixel)
            leader_path.append(leader_pos_pixels)
            last_leader_pos = leader_pos_pixels
            save_leader_pos_to_file((center_x_cm, center_y_cm))

            cv2.circle(frame, leader_pos_pixels, detection_radius_px, (0, 255, 255), 2)

        # Follower (ID=2)
        elif marker_id_str == "2":
            follower_pos_pixels = (center_x_pixel, center_y_pixel)
            follower_path.append(follower_pos_pixels)
            last_follower_pos = follower_pos_pixels

            # Sudut follower
            pts_f = corners[i][0]
            tl = pts_f[0]; tr = pts_f[1]
            vx_img = tr[0] - tl[0]
            vy_img = tr[1] - tl[1]

            theta_arena = math.atan2(-vy_img, vx_img)  # balik sumbu Y
            theta_arena = wrap_pi(theta_arena + FOLLOW_YAW_OFFSET_RAD)
            global _prev_theta_follower
            theta_arena = smooth_angle(_prev_theta_follower, theta_arena, alpha=0.30)
            _prev_theta_follower = theta_arena

            save_pose(FOLLOWER_POSE_FILE, center_x_cm, center_y_cm, theta_arena)

            L = 30
            theta_img_draw = -theta_arena
            end_x = int(center_x_pixel + L * math.cos(theta_img_draw))
            end_y = int(center_y_pixel + L * math.sin(theta_img_draw))
            cv2.arrowedLine(frame, (center_x_pixel, center_y_pixel), (end_x, end_y),
                            (0, 255, 255), 2, tipLength=0.35)

            # Mode follow/catch-up pakai radius pada PIXEL
            if last_leader_pos is not None:
                dx = follower_pos_pixels[0] - last_leader_pos[0]
                dy = follower_pos_pixels[1] - last_leader_pos[1]
                distance = math.hypot(dx, dy)

                if distance <= detection_radius_px:
                    follower_mode = "FOLLOW"
                    target = last_leader_pos
                else:
                    follower_mode = "CATCH_UP"
                    # Ambil target dari leader_path.txt (dalam CM), konversi ke PX + offset
                    try:
                        with open(os.path.join(POSE_DIR, "leader_path.txt"), "r") as f:
                            lines = f.readlines()
                            if lines:
                                last_line = lines[-1].strip().split(",")
                                leader_x_cm = float(last_line[0])
                                leader_y_cm = float(last_line[1])
                                leader_x_px = int(off_x + leader_x_cm * px_per_cm)
                                leader_y_px = int(off_y + (GRID_ROWS * GRID_CELL_CM * px_per_cm) - leader_y_cm * px_per_cm)
                                target = (leader_x_px, leader_y_px)
                            else:
                                target = None
                    except FileNotFoundError:
                        target = None

                if target is not None:
                    arahkan_follower_ke_jalur(frame, follower_pos_pixels, [target])

        # Titik marker
        color = (255, 0, 0) if marker_id_str == "1" else (0, 0, 255)
        cv2.circle(frame, (center_x_pixel, center_y_pixel), 5, color, -1)

    return frame, aruco_centers

def arahkan_follower_ke_jalur(frame, follower_pos, path_points):
    if not path_points or not follower_pos:
        return
    titik_terdekat = min(path_points, key=lambda p: math.hypot(p[0] - follower_pos[0], p[1] - follower_pos[1]))
    draw_arrow(frame, follower_pos, titik_terdekat, color=(0, 255, 255), thickness=2, tip_length=0.4)
    theta = calculate_theta(follower_pos, titik_terdekat)
    print(f"[Follower ➜ Target] Mode: {follower_mode}, Titik: {titik_terdekat}, Theta: {theta:.1f}°")

# ---------- Main loop ----------
def detect_colors():
    global show_masks, last_leader_pos

    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("❌ Kamera gagal dibuka.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    settings_file = "hsv_settings.yaml"
    saved_settings = load_hsv_settings_yaml(settings_file)

    setup_trackbar("Trackbars Yellow", saved_settings, {
        "H_min": 20, "S_min": 120, "V_min": 120,
        "H_max": 30, "S_max": 255, "V_max": 255
    })

    kernel = np.ones((5, 5), np.uint8)
    prev_time = time.time()
    frame_count = 0

    # ArUco dictionary
    current_dict_key = "3"
    dictionary = aruco.getPredefinedDictionary(DICT_LIST[current_dict_key])
    try:
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, parameters)
    except AttributeError:
        parameters = aruco.DetectorParameters_create()
        detector = None

    # Loop utama
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]

        # Geometri grid untuk frame saat ini
        px_per_cm, pixels_per_grid, off_x, off_y, grid_w_px, grid_h_px = compute_grid_geometry(w, h)

        # sinkron ke GUI jika berubah
        global _prev_ppg
        if pixels_per_grid != _prev_ppg:
            print(f"[GRID] {GRID_COLS}x{GRID_ROWS} @ {GRID_CELL_CM}cm -> {pixels_per_grid}px/sel (frame {w}x{h})")
            _prev_ppg = pixels_per_grid
            _write_ppg(pixels_per_grid)

        # radius deteksi dalam PIXEL
        detection_radius_px = int(DETECTION_RADIUS_CM * px_per_cm)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_count += 1
        current_time = time.time()
        fps = 1.0 / (current_time - prev_time)
        prev_time = current_time

        # --- Deteksi warna kuning (obstacle) ---
        lower_yellow, upper_yellow = get_trackbar_values("Trackbars Yellow")
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        small_kernel = np.ones((3,3), np.uint8)
        mask_temp = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, small_kernel)
        mask_temp = cv2.erode(mask_temp, small_kernel, iterations=1)
        mask_temp = cv2.dilate(mask_temp, small_kernel, iterations=1)

        contours_temp, _ = cv2.findContours(mask_temp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        avg_area = sum(cv2.contourArea(c) for c in contours_temp) / max(len(contours_temp), 1) if contours_temp else 0

        if avg_area < 800:
            kernel = np.ones((3,3), np.uint8)
            mask_yellow = cv2.erode(mask_temp, kernel, iterations=2)
            mask_yellow = cv2.dilate(mask_yellow, kernel, iterations=2)
        else:
            kernel = np.ones((4,4), np.uint8)
            mask_yellow = cv2.erode(mask_temp, kernel, iterations=1)
            mask_yellow = cv2.dilate(mask_yellow, kernel, iterations=2)
            mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)

        contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        obstacles_with_area = []  # (x_cm, y_cm, area)
        for cnt in contours_yellow:
            area = cv2.contourArea(cnt)
            if area > 10:
                x, y, wbox, hbox = cv2.boundingRect(cnt)
                cx_pix = x + wbox // 2
                cy_pix = y + hbox // 2

                # Konversi ke cm pakai geometri grid (origin kiri-bawah)
                cx_cm = (cx_pix - off_x) / px_per_cm
                cy_cm = (off_y + grid_h_px - cy_pix) / px_per_cm

                obstacles_with_area.append((cx_cm, cy_cm, area))
                cv2.rectangle(frame, (x, y), (x + wbox, y + hbox), (0, 255, 255), 2)

        # de-dupe obstacles (cm)
        def dedupe_obstacles(obstacles_with_area, min_dist_cm=15.0):
            if len(obstacles_with_area) <= 1:
                return [(x, y) for x, y, _ in obstacles_with_area]
            deduped = []
            for x, y, area in obstacles_with_area:
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

        obstacles_cm = dedupe_obstacles(obstacles_with_area, min_dist_cm=15.0)

        # Simpan obstacles (tiap 5 frame)
        if frame_count % 5 == 0:
            save_obstacles_atomic(obstacles_cm)

        # Label obstacle
        for cnt in contours_yellow:
            if cv2.contourArea(cnt) > 100:
                x, y, wbox, hbox = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + wbox, y + hbox), (0, 255, 255), 2)
                #cv2.putText(frame, "Obstacle", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # --- Deteksi ArUco tiap 3 frame ---
        if frame_count % 3 == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if detector:
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)
            if ids is not None:
                frame, _ = detect_aruco_markers_core(frame, corners, ids,
                                                     px_per_cm, off_x, off_y, grid_h_px,
                                                     detection_radius_px)

        # Jalur leader
        for i in range(1, len(leader_path)):
            cv2.line(frame, leader_path[i - 1], leader_path[i], (0, 255, 0), 2)

        if len(leader_path) >= 2:
            theta = calculate_theta(leader_path[-2], leader_path[-1])
            draw_arrow(frame, leader_path[-2], leader_path[-1], color=(0, 255, 255), thickness=2, tip_length=0.4)
            cv2.putText(frame, f"Theta: {theta:.1f} deg", (leader_path[-1][0] + 10, leader_path[-1][1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(frame, "Orientasi Leader", (leader_path[-1][0] + 10, leader_path[-1][1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # --- Gambar GRID setiap frame (pakai geometri fix 13x10 @ 15cm) ---
        draw_fixed_grid(frame, pixels_per_grid, off_x, off_y, grid_w_px, grid_h_px)

        # FPS
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Tampilkan
        cv2.imshow("Frame", frame)
        if show_masks:
            cv2.imshow("Mask Yellow", mask_yellow)
        else:
            if cv2.getWindowProperty("Mask Yellow", cv2.WND_PROP_VISIBLE) >= 1:
                cv2.destroyWindow("Mask Yellow")

        # Kontrol keyboard
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('m'):
            show_masks = not show_masks
        elif key == ord('s'):
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join("captures", f"frame_{timestamp}.png")
            os.makedirs("captures", exist_ok=True)
            cv2.imwrite(filename, frame)
            print(f"Frame disimpan sebagai {filename}")
        elif key == ord('r'):
            leader_path.clear()
            last_leader_pos = None
            open(os.path.join(POSE_DIR, "leader_path.txt"), "w").close()
            print("Reset berhasil: Jalur leader dihapus, posisi direset.")
        elif chr(key) in DICT_LIST:
            current_dict_key = chr(key)
            dictionary = aruco.getPredefinedDictionary(DICT_LIST[current_dict_key])
            try:
                parameters = aruco.DetectorParameters()
                detector = aruco.ArucoDetector(dictionary, parameters)
            except AttributeError:
                parameters = aruco.DetectorParameters_create()
                detector = None
            print(f"Ganti dictionary: {current_dict_key} ({DICT_LIST[current_dict_key]})")

    cap.release()
    cv2.destroyAllWindows()

# Main
if __name__ == "__main__":
    detect_colors()