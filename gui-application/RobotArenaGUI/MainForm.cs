using System.Globalization;  // Untuk memastikan parsing angka desimal (koma vs titik) benar
using System.IO;           // Untuk membaca dan menulis data stream dengan mudah
using System.Net.Sockets; // Komponen utama untuk networking (TcpClient)
using System.Text;         // Untuk mengubah string menjadi byte dan sebaliknya
using System.Threading;
using System.Net;
using Newtonsoft.Json;  // Tambah di using
using Newtonsoft.Json.Linq;
using System.Timers;

namespace RobotArenaGUI
{
    public enum CellType { Free, Obstacle, Start, Goal, PathNode }
    public partial class MainForm : Form
    {
        // === Konstanta Arena & Grid ===
        private const float ARENA_WIDTH_REAL_CM = 200.0f;  // 2 meter
        private const float ARENA_HEIGHT_REAL_CM = 150.0f; // 1.5 meter
        private const float ROBOT_WIDTH_CM = 15.0f;
        private const float ROBOT_HEIGHT_CM = 15.0f;
        private bool isPaintingObstacles = false;
        // private const float GRID_CELL_SIZE_REAL_CM = 15.0f; // Ukuran sel grid fisik
        private float[] availableGridCellSizesCm = { 15.0f, 10.0f, 5.0f };
        private int currentGridSizeIndex = 0; // Indeks untuk availableGridCellSizesCm
        private float currentGridCellSizeCm;  // Ukuran sel grid saat ini

        private static readonly string PoseDir =
    Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "RobotData");
        private static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        private static readonly string LEADER_POSE_PATH = Path.Combine(PoseDir, "leader_pose.txt");
        private static readonly string FOLLOWER_POSE_PATH = Path.Combine(PoseDir, "follower_pose.txt");

        private float leaderX = 0, leaderY = 0, leaderTh = 0;
        private float followerX = 0, followerY = 0, followerTh = 0;
        private bool haveLeaderPose = false, haveFollowerPose = false;

        private const string VISION_SERVER_IP = "localhost"; 
        private const int VISION_SERVER_PORT = 6666;

        // === Streamer SETLEADER ===
        private const string FOLLOWER_IP = "10.159.150.23"; 
        private const int FOLLOWER_PORT = 6060;               
        private TcpClient followerClient;
        private StreamWriter followerWriter;
        private CancellationTokenSource setleaderCts;

        private volatile bool latestLeaderHas = false;
        private volatile float latestLeaderX, latestLeaderY, latestLeaderTh;
        private System.Windows.Forms.Timer poseTimer;

        private int gridOffsetX = 0;
        private int gridOffsetY = 0;
            bool _leaderSetposSent = false;

        private float pixelPerCm = 2.0f;
        private int setposCounter = 0;

        private TcpClient leaderClient;
        private StreamWriter leaderWriter;

        private static readonly string OBSTACLE_FILE_PATH = Path.Combine(PoseDir, "obstacles.txt");
        private System.Timers.Timer obstacleUpdateTimer;
        private List<PointF> currentVisionObstacles = new List<PointF>();  // Cache posisi cm



        private PointF robotFixedPhysicalStartCm = new PointF(22.5f, 22.5f);

        private int gridCols;
        private int gridRows;
        private float cellSizePixelsFloat; // Ukuran sel dalam pixel di GUI, akan dihitung

        private const string ROBOT_IP_ADDRESS = "10.159.150.9"; // <<< GANTI DENGAN IP ESP8266 ANDA!
        private const int ROBOT_PORT = 5555;

        // === Status Grid ===

        private CellType[,] gridData; // Array 2D untuk menyimpan status setiap sel

        // === Posisi Robot & Goal ===
        // private Point robotStartGridPos = new Point(2, 2); // Posisi awal robot (kolom 2, baris 2)
        private Point robotStartGridPos;
        private Point? currentGoalGridPos = null;          // Posisi goal saat ini, nullable

        // === Pathfinding & Waypoints ===
        private List<Point> calculatedPath = null; // Menyimpan path hasil Dijkstra

        // === Mode UI ===
        private bool obstacleModeActive = false;

        public MainForm()
        {
            InitializeComponent();
            
            toggleObstacleModeButton.Click += ToggleObstacleModeButton_Click;
            clearObstaclesButton.Click += ClearObstaclesButton_Click;
            sendWaypointsButton.Click += SendWaypointsButton_Click;
            changeGridButton.Click += changeGridButton_Click;
        }

        private CellType[,] CreateInflatedGridForPathfinding()
        {
            // Hitung berapa banyak sel yang ditempati robot
            // Math.Ceiling penting jika ukuran robot tidak pas kelipatan ukuran sel
            int robotFootprintWidthCells = (int)Math.Ceiling(ROBOT_WIDTH_CM / currentGridCellSizeCm);
            int robotFootprintHeightCells = (int)Math.Ceiling(ROBOT_HEIGHT_CM / currentGridCellSizeCm);

            // Jari-jari inflasi dalam sel dari pusat robot.
            // Jika robot 3 sel lebar, pusatnya di sel tengah, dan ia meluas 1 sel ke kiri & kanan.
            int inflationRadiusCol = (robotFootprintWidthCells - 1) / 2;
            int inflationRadiusRow = (robotFootprintHeightCells - 1) / 2;
            // Untuk footprint genap (misal 2 sel), pembagian integer ini akan membulatkan ke bawah,
            // yang berarti inflasi mungkin perlu penanganan khusus atau kita asumsikan pusat robot
            // selalu di salah satu sel. Untuk 3x3, ini akan menjadi 1.

            var inflatedGrid = new CellType[gridCols, gridRows];

            for (int r_center = 0; r_center < gridRows; r_center++) // Iterasi setiap sel (calon posisi pusat robot)
            {
                for (int c_center = 0; c_center < gridCols; c_center++)
                {
                    // Awalnya anggap sel ini aman untuk pusat robot
                    inflatedGrid[c_center, r_center] = CellType.Free;

                    // Periksa apakah menempatkan pusat robot di (c_center, r_center) akan menyebabkan
                    // salah satu bagian dari footprint robot menabrak obstacle ASLI.
                    bool collisionDetected = false;
                    for (int dr = -inflationRadiusRow; dr <= inflationRadiusRow; dr++) // Iterasi footprint robot
                    {
                        for (int dc = -inflationRadiusCol; dc <= inflationRadiusCol; dc++)
                        {
                            int actualCellCol = c_center + dc; // Kolom sel yang ditempati oleh bagian robot
                            int actualCellRow = r_center + dr; // Baris sel yang ditempati oleh bagian robot

                            if (IsWithinGridBounds(actualCellCol, actualCellRow))
                            {
                                // Periksa terhadap gridData ASLI
                                if (gridData[actualCellCol, actualCellRow] == CellType.Obstacle)
                                {
                                    collisionDetected = true;
                                    break; // Cukup satu bagian robot menabrak obstacle
                                }
                            }
                            else // Jika bagian dari footprint robot keluar dari arena
                            {
                                collisionDetected = true; // Anggap sebagai kolisi (pusat robot tidak boleh di sini)
                                break;
                            }
                        }
                        if (collisionDetected) break;
                    }

                    if (collisionDetected)
                    {
                        // Jika ada kolisi, maka pusat robot tidak boleh berada di (c_center, r_center)
                        inflatedGrid[c_center, r_center] = CellType.Obstacle;
                    }
                }
            }

            // Pastikan posisi Start dan Goal tidak secara tidak sengaja terhapus oleh inflasi
            // jika mereka valid di peta asli. Dijkstra akan gagal jika Start/Goal adalah obstacle.
            // Ini penting: Jika start/goal Anda sangat dekat dengan obstacle sehingga setelah inflasi
            // posisi tersebut menjadi tidak valid, path tidak akan ditemukan.
            if (IsWithinGridBounds(robotStartGridPos.X, robotStartGridPos.Y))
            {
                // Hanya set Start jika sel aslinya bukan obstacle
                if (gridData[robotStartGridPos.X, robotStartGridPos.Y] != CellType.Obstacle)
                {
                    inflatedGrid[robotStartGridPos.X, robotStartGridPos.Y] = CellType.Start;
                }
                else
                {
                    // Jika start awal ternyata di dalam obstacle asli, ini masalah desain.
                    // Untuk sekarang, biarkan pathfinder gagal.
                    UpdateStatusLabel("ERROR: Posisi start robot berada di dalam obstacle!");
                }
            }

            if (currentGoalGridPos.HasValue && IsWithinGridBounds(currentGoalGridPos.Value.X, currentGoalGridPos.Value.Y))
            {
                // Hanya set Goal jika sel aslinya bukan obstacle
                if (gridData[currentGoalGridPos.Value.X, currentGoalGridPos.Value.Y] != CellType.Obstacle)
                {
                    inflatedGrid[currentGoalGridPos.Value.X, currentGoalGridPos.Value.Y] = CellType.Goal;
                }
                else if (inflatedGrid[currentGoalGridPos.Value.X, currentGoalGridPos.Value.Y] == CellType.Obstacle)
                {
                    // Jika goal jatuh di area inflasi, berarti tidak bisa dicapai dengan aman
                    UpdateStatusLabel("WARNING: Posisi goal mungkin tidak bisa dicapai karena ukuran robot!");
                }
            }
            return inflatedGrid;
        }

        private void MainForm_Load(object sender, EventArgs e)
        {
            currentGridCellSizeCm = availableGridCellSizesCm[currentGridSizeIndex];
            changeGridButton.Text = $"Grid {currentGridCellSizeCm:F0}x{currentGridCellSizeCm:F0}";

            InitializeArenaAndGrid(); // Panggil untuk setup awal
            poseTimer = new System.Windows.Forms.Timer();
            poseTimer.Interval = 100; // 20 Hz
            poseTimer.Tick += PoseTimer_Tick;
            poseTimer.Start();
            EnsureLeaderConnection();
            
            UpdateStatusLabel("Vision obstacles monitoring started.");
            System.Diagnostics.Debug.WriteLine("[GUI INIT] Obstacle timer started (200ms interval).");

            if (TryReadPose(LEADER_POSE_PATH, out var x, out var y, out var th))
            {
                leaderWriter.WriteLine($"SETPOS {x.ToString(CI)},{y.ToString(CI)},{th.ToString(CI)}"); // pakai SETPOS_DEG jika perlu
                _leaderSetposSent = true;
            }

            setleaderCts = new CancellationTokenSource();
            _ = Task.Run(() => SetleaderSenderLoop(setleaderCts.Token));

            UpdateStatusLabel("Form loaded. Grid initialized.");
            sendWaypointsButton.Enabled = false;

            System.Diagnostics.Debug.WriteLine($"arenaPictureBox.Visible after Load init: {this.arenaPictureBox.Visible}");
            // arenaPi
            {
                string _path;
                int ppg = LoadPixelsPerGrid(out _path, -1);
                if (ppg > 0) UpdateStatusLabel($"Pixel-per-grid sinkron: {ppg}px/sel (sumber: {_path})");
            }


        }


        private void InitializeArenaAndGrid()
        {
            System.Diagnostics.Debug.WriteLine($"InitializeArenaAndGrid started. Current cell size: {currentGridCellSizeCm} cm");

            if (arenaPictureBox.ClientSize.Width == 0 || arenaPictureBox.ClientSize.Height == 0)
            {
                System.Diagnostics.Debug.WriteLine("WARNING: PictureBox ClientSize is zero in InitializeArenaAndGrid! Aborting grid init for now.");
                // Mungkin PictureBox belum sepenuhnya di-layout.
                // Ini bisa jadi masalah jika dipanggil terlalu dini (misalnya dari konstruktor sebelum InitializeComponent).
                return;
            }
            System.Diagnostics.Debug.WriteLine($"PictureBox ClientSize: W={arenaPictureBox.ClientSize.Width}, H={arenaPictureBox.ClientSize.Height}");

            gridCols = (int)Math.Floor(ARENA_WIDTH_REAL_CM / currentGridCellSizeCm);
            gridRows = (int)Math.Floor(ARENA_HEIGHT_REAL_CM / currentGridCellSizeCm);

            if (gridCols <= 0 || gridRows <= 0)
            {
                System.Diagnostics.Debug.WriteLine($"ERROR: Calculated gridCols ({gridCols}) or gridRows ({gridRows}) is zero or negative. Aborting grid init.");
                cellSizePixelsFloat = 0; // Set tidak valid
                return;
            }

            float pixelPerCmX = (float)arenaPictureBox.ClientSize.Width / ARENA_WIDTH_REAL_CM;
            float pixelPerCmY = (float)arenaPictureBox.ClientSize.Height / ARENA_HEIGHT_REAL_CM;
            float pixelPerCm = Math.Min(pixelPerCmX, pixelPerCmY);
            cellSizePixelsFloat = currentGridCellSizeCm * pixelPerCm;
            // --- OVERRIDE dari vision: pakai pixels_per_grid.txt kalau ada ---
            string srcPath;
            int ppg = LoadPixelsPerGrid(out srcPath, -1);
            if (ppg > 0)
            {
                // Samakan cell size GUI dengan vision (mis. 45 px per sel)
                cellSizePixelsFloat = ppg;
                System.Diagnostics.Debug.WriteLine($"[GRID] Using pixels_per_grid from vision: {ppg} px/sel (src: {srcPath})");
            }
            else
            {
                System.Diagnostics.Debug.WriteLine($"[GRID] pixels_per_grid.txt not found -> keep computed cellSizePixelsFloat = {cellSizePixelsFloat:F1}px");
            }


            if (cellSizePixelsFloat <= 0)
            {
                System.Diagnostics.Debug.WriteLine($"ERROR: cellSizePixels calculated as {cellSizePixelsFloat}. Aborting grid init.");
                return;
            }

            System.Diagnostics.Debug.WriteLine($"Calculated: pixelPerCm={pixelPerCm:F3}, cellSizePixels={cellSizePixelsFloat}, gridCols={gridCols}, gridRows={gridRows}");


            float totalGridWidthPixelsFloat = gridCols * cellSizePixelsFloat;
            float totalGridHeightPixelsFloat = gridRows * cellSizePixelsFloat;
            gridOffsetX = (int)Math.Round((arenaPictureBox.ClientSize.Width - totalGridWidthPixelsFloat) / 2.0f);
            gridOffsetY = (int)Math.Round((arenaPictureBox.ClientSize.Height - totalGridHeightPixelsFloat) / 2.0f);
            System.Diagnostics.Debug.WriteLine($"Calculated: OffsetX={gridOffsetX}, OffsetY={gridOffsetY}");

            gridData = new CellType[gridCols, gridRows];
            for (int c = 0; c < gridCols; c++)
            {
                for (int r = 0; r < gridRows; r++)
                {
                    gridData[c, r] = CellType.Free;
                }
            }

            // Kalkulasi dan set robotStartGridPos
            if (currentGridCellSizeCm == 15.0f)
            {
                robotStartGridPos = new Point(0, 0);
            }
            else
            {
                int startCol = (int)(robotFixedPhysicalStartCm.X / currentGridCellSizeCm);
                int startRow = (int)(robotFixedPhysicalStartCm.Y / currentGridCellSizeCm);
                robotStartGridPos = new Point(startCol, startRow);
            }

            if (IsWithinGridBounds(robotStartGridPos.X, robotStartGridPos.Y))
            {
                gridData[robotStartGridPos.X, robotStartGridPos.Y] = CellType.Start;
            }
            else
            {
                robotStartGridPos = new Point(Math.Min(1, gridCols > 0 ? gridCols - 1 : 0), Math.Min(1, gridRows > 0 ? gridRows - 1 : 0));
                if (IsWithinGridBounds(robotStartGridPos.X, robotStartGridPos.Y))
                {
                    gridData[robotStartGridPos.X, robotStartGridPos.Y] = CellType.Start;
                }
            }
            System.Diagnostics.Debug.WriteLine($"robotStartGridPos set to: ({robotStartGridPos.X},{robotStartGridPos.Y})");

            currentGoalGridPos = null;
            ClearPathAndButton();

            arenaPictureBox.MouseDown += ArenaPictureBox_MouseDown;
            arenaPictureBox.MouseMove += ArenaPictureBox_MouseMove;
            arenaPictureBox.MouseUp += ArenaPictureBox_MouseUp;
            arenaPictureBox.MouseClick += ArenaPictureBox_MouseClick_ForGoalSetting;

            // Hapus re-attachment dari sini jika Anda mengandalkan designer atau satu kali di Load
            arenaPictureBox.Paint -= ArenaPictureBox_Paint;
            arenaPictureBox.Paint += ArenaPictureBox_Paint;
            arenaPictureBox.Invalidate();


        }

        private bool IsWithinGridBounds(int c, int r)
        {
            return c >= 0 && c < gridCols && r >= 0 && r < gridRows;
        }

        // --- Event Handler untuk Menggambar Arena ---
        private void ArenaPictureBox_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.Clear(Color.WhiteSmoke);

            if (cellSizePixelsFloat <= 0 || gridData == null) return;

            for (int c = 0; c < gridCols; c++)
            {
                for (int r = 0; r < gridRows; r++)
                {
                    // Hitung posisi piksel Y dengan membalik sumbu Y
                    float pixelX = gridOffsetX + c * cellSizePixelsFloat;
                    float pixelY = gridOffsetY + ((gridRows - 1) - r) * cellSizePixelsFloat;

                    RectangleF cellRectF = new RectangleF(pixelX, pixelY, cellSizePixelsFloat, cellSizePixelsFloat);

                    Brush cellBrush = Brushes.White;
                    switch (gridData[c, r])
                    {
                        case CellType.Obstacle: cellBrush = Brushes.DarkGray; break;
                        case CellType.Start: cellBrush = Brushes.Green; break;
                        case CellType.Goal: cellBrush = Brushes.Red; break;
                        case CellType.PathNode: cellBrush = Brushes.LightBlue; break;
                    }
                    g.FillRectangle(cellBrush, cellRectF); // Menggambar area sel
                    g.DrawRectangle(Pens.LightGray, (int)cellRectF.X, (int)cellRectF.Y, (int)cellRectF.Width, (int)cellRectF.Height); // Menggambar garis grid
                }
            }
        }

        // --- Event Handler untuk Klik Mouse di Arena ---
        private void ArenaPictureBox_MouseClick_ForGoalSetting(object sender, MouseEventArgs e)
        {
            if (obstacleModeActive) return;

            Point? gridPos = PixelToGrid(e.X, e.Y);
            if (!gridPos.HasValue) return;

            int c = gridPos.Value.X;
            int r = gridPos.Value.Y;

            // 2. Pastikan klik berada di dalam batas grid yang valid
            if (!IsWithinGridBounds(c, r))
            {
                return; // Klik di luar area grid yang digambar
            }

            // 3. Logika untuk menempatkan Goal baru:
            //    Hanya bisa menempatkan Goal di sel yang 'Free' atau bekas 'PathNode'.
            //    Dan pastikan sel tersebut bukan posisi Start robot.
            if ((gridData[c, r] == CellType.Free || gridData[c, r] == CellType.PathNode) &&
                !(c == robotStartGridPos.X && r == robotStartGridPos.Y))
            {
                // A. Hapus visual Goal lama dari gridData (ubah kembali menjadi Free)
                if (currentGoalGridPos.HasValue)
                {
                    Point oldGoal = currentGoalGridPos.Value;
                    if (IsWithinGridBounds(oldGoal.X, oldGoal.Y) && gridData[oldGoal.X, oldGoal.Y] == CellType.Goal)
                    {
                        // Hanya ubah jadi Free jika bukan Start juga (meskipun logika di atas sudah mencegah Goal = Start)
                        if (!oldGoal.Equals(robotStartGridPos))
                        {
                            gridData[oldGoal.X, oldGoal.Y] = CellType.Free;
                        }
                    }
                }

                // B. Hapus visual Path lama dari gridData dan reset data path
                //    Ini juga akan menonaktifkan tombol 'sendWaypointsButton'
                ClearPathAndButton(); // Ini adalah fungsi helper yang sudah kita buat:
                                      // ClearPathFromGrid();
                                      // calculatedPath = null;
                                      // sendWaypointsButton.Enabled = false;

                // C. Set Goal baru
                currentGoalGridPos = new Point(c, r);
                gridData[c, r] = CellType.Goal; // Tandai sel baru sebagai Goal di data grid
                UpdateStatusLabel($"Goal diatur di: ({c},{r}). Menghitung path...");

                // D. Hitung dan tampilkan path baru
                FindAndDisplayPath();

                // E. Minta PictureBox untuk digambar ulang untuk menampilkan perubahan
                arenaPictureBox.Invalidate();
            }
            else if (c == robotStartGridPos.X && r == robotStartGridPos.Y)
            {
                UpdateStatusLabel("Tidak bisa mengatur Goal di posisi Start robot.");
            }
            else if (gridData[c, r] == CellType.Obstacle)
            {
                UpdateStatusLabel("Tidak bisa mengatur Goal di atas Obstacle.");
            }
        }


        private void ClearPathFromGrid()
        {
            if (calculatedPath != null)
            {
                foreach (Point p_path in calculatedPath)
                {
                    if (IsWithinGridBounds(p_path.X, p_path.Y))
                    {
                        // Hanya ubah PathNode menjadi Free, jangan timpa Start atau Goal yang mungkin ada di path
                        if (gridData[p_path.X, p_path.Y] == CellType.PathNode)
                        {
                            gridData[p_path.X, p_path.Y] = CellType.Free;
                        }
                    }
                }
            }
        }

        private void FindAndDisplayPath()
        {
            if (!currentGoalGridPos.HasValue)
            {
                sendWaypointsButton.Enabled = false;
                return;
            }
            ClearPathFromGrid(); // Membersihkan visualisasi path lama dari gridData

            // 1. Buat peta yang sudah di-inflate untuk pathfinding
            CellType[,] pathfindingGrid = CreateInflatedGridForPathfinding();

            // 2. Panggil Dijkstra menggunakan pathfindingGrid
            // Pastikan start dan goal valid di pathfindingGrid
            // (CreateInflatedGridForPathfinding sudah mencoba mempertahankan Start/Goal jika valid)

            if (pathfindingGrid[robotStartGridPos.X, robotStartGridPos.Y] == CellType.Obstacle)
            {
                UpdateStatusLabel($"Posisi Start ({robotStartGridPos.X},{robotStartGridPos.Y}) tidak valid karena ukuran robot & obstacle.");
                sendWaypointsButton.Enabled = false;
                arenaPictureBox.Invalidate(); // Mungkin ingin menggambar inflated grid untuk debug
                return;
            }
            if (pathfindingGrid[currentGoalGridPos.Value.X, currentGoalGridPos.Value.Y] == CellType.Obstacle &&
                gridData[currentGoalGridPos.Value.X, currentGoalGridPos.Value.Y] != CellType.Goal) // Jangan error jika goal itu sendiri jadi obstacle
            {
                UpdateStatusLabel($"Posisi Goal ({currentGoalGridPos.Value.X},{currentGoalGridPos.Value.Y}) tidak valid karena ukuran robot & obstacle.");
                sendWaypointsButton.Enabled = false;
                arenaPictureBox.Invalidate();
                return;
            }

            calculatedPath = DijkstraPathfinder.FindPath(pathfindingGrid, // Gunakan peta inflasi
                                                        robotStartGridPos,
                                                        currentGoalGridPos.Value,
                                                        gridCols, gridRows);

            // Visualisasi: Tetap menggambar path di atas gridData asli
            // Peta inflasi hanya digunakan untuk perhitungan path.
            if (calculatedPath != null && calculatedPath.Any())
            {
                foreach (Point p_path in calculatedPath)
                {
                    if (IsWithinGridBounds(p_path.X, p_path.Y))
                    {
                        if (gridData[p_path.X, p_path.Y] == CellType.Free)
                        {
                            gridData[p_path.X, p_path.Y] = CellType.PathNode;
                        }
                    }
                }
                // Pastikan Goal tetap terlihat sebagai Goal di gridData untuk visualisasi
                if (currentGoalGridPos.HasValue && IsWithinGridBounds(currentGoalGridPos.Value.X, currentGoalGridPos.Value.Y))
                {
                    gridData[currentGoalGridPos.Value.X, currentGoalGridPos.Value.Y] = CellType.Goal;
                }


                UpdateStatusLabel($"Path ditemukan ke ({currentGoalGridPos.Value.X},{currentGoalGridPos.Value.Y}). Siap dikirim.");
                sendWaypointsButton.Enabled = true;
            }
            else
            {
                UpdateStatusLabel($"Path tidak ditemukan ke ({currentGoalGridPos.Value.X},{currentGoalGridPos.Value.Y}) (mungkin terhalang ukuran robot).");
                sendWaypointsButton.Enabled = false;
            }
            arenaPictureBox.Invalidate(); // Gambar ulang arena dengan path baru (di atas gridData asli)
        }


        // --- Event Handler untuk Tombol ---
        private void ToggleObstacleModeButton_Click(object sender, EventArgs e)
        {
            obstacleModeActive = !obstacleModeActive;
            isPaintingObstacles = false; // Selalu reset status melukis saat ganti mode

            if (obstacleModeActive)
            {
                toggleObstacleModeButton.Text = "Mode Obstacle: ON";
                UpdateStatusLabel("Mode Obstacle AKTIF. Tekan & geser mouse untuk buat/hapus obstacle.");
            }
            else
            {
                toggleObstacleModeButton.Text = "Mode Obstacle: OFF";
                UpdateStatusLabel("Mode Obstacle NONAKTIF. Klik sel untuk set Goal.");
            }
        }

        private bool TryReadPose(string path, out float x, out float y, out float th)
        {
            x = y = th = 0f;
            try
            {
                if (!File.Exists(path)) return false;
                var line = File.ReadAllText(path).Trim();
                if (string.IsNullOrWhiteSpace(line)) return false;
                var p = line.Split(',');
                if (p.Length < 3) return false;
                x = float.Parse(p[0], CultureInfo.InvariantCulture);
                y = float.Parse(p[1], CultureInfo.InvariantCulture);
                th = float.Parse(p[2], CultureInfo.InvariantCulture); // rad
                return true;
            }
            catch { return false; }
        }

        private void PoseTimer_Tick(object sender, EventArgs e)
        {
            // 1) Baca file pose (overwrite setiap deteksi)
            haveLeaderPose = TryReadPose(LEADER_POSE_PATH, out leaderX, out leaderY, out leaderTh);
            haveFollowerPose = TryReadPose(FOLLOWER_POSE_PATH, out followerX, out followerY, out followerTh);
            ReadAndUpdateObstacles();
            setposCounter++;
            if (setposCounter >= 5) // kalau timer 100 ms ? 0.5 s
            {
                setposCounter = 0;
                EnsureFollowerConnection();
                // followerX, followerY, followerTh = dari TryReadPoseFile(FOLLOWER_POSE_PATH, ...)
                followerWriter?.WriteLine(
                    $"SETPOS {followerX.ToString(CultureInfo.InvariantCulture)} " +
                    $"{followerY.ToString(CultureInfo.InvariantCulture)} " +
                    $"{followerTh.ToString(CultureInfo.InvariantCulture)}");
                followerWriter?.Flush();
            }
            // 2) Update �Start� (posisi robot) di grid dari follower atau leader � tergantung kebutuhan
            //    Misal: gunakan follower sebagai posisi robot yang sedang jalan:
            if (haveLeaderPose)
            {
                int newGridX = (int)Math.Floor(leaderX / currentGridCellSizeCm);
                int newGridY = (int)Math.Floor(leaderY / currentGridCellSizeCm);

                // Hapus start lama
                if (IsWithinGridBounds(robotStartGridPos.X, robotStartGridPos.Y) &&
                    gridData[robotStartGridPos.X, robotStartGridPos.Y] == CellType.Start)
                {
                    gridData[robotStartGridPos.X, robotStartGridPos.Y] = CellType.Free;
                }

                // Set start baru
                robotStartGridPos = new Point(
                    Math.Max(0, Math.Min(gridCols - 1, newGridX)),
                    Math.Max(0, Math.Min(gridRows - 1, newGridY))
                );
                gridData[robotStartGridPos.X, robotStartGridPos.Y] = CellType.Start;

                arenaPictureBox.Invalidate();
            }


            // 3) Stream SETLEADER ke follower (rad, cm)
            if (haveLeaderPose)
            {

                if (followerWriter != null)
                {
                    try
                    {
                        followerWriter.WriteLine($"SETLEADER {leaderX.ToString(CultureInfo.InvariantCulture)} {leaderY.ToString(CultureInfo.InvariantCulture)} {leaderTh.ToString(CultureInfo.InvariantCulture)}");
                        // kirim START sekali bila dibutuhkan:
                        // followerWriter.WriteLine("START");
                        followerWriter.Flush();
                    }
                    catch
                    {
                        // Rekoneksi nanti otomatis di tick berikutnya
                        CleanupFollowerConnection();
                    }
                  }
            }
        }

        // Event handler timer:
        

        // Helper: Bandingkan list (sederhana, toleransi 1cm)
        private bool AreObstaclesEqual(List<PointF> list1, List<PointF> list2)
        {
            if (list1.Count != list2.Count)
            {
                System.Diagnostics.Debug.WriteLine($"[EQUAL CHECK] Count mismatch: {list1.Count} vs {list2.Count}");
                return false;
            }
            for (int i = 0; i < list1.Count; i++)
            {
                float dx = Math.Abs(list1[i].X - list2[i].X);
                float dy = Math.Abs(list1[i].Y - list2[i].Y);
                float dist = (float)Math.Sqrt(dx * dx + dy * dy);
                if (dist > 1.0f)  // Toleransi 1 cm
                {
                    System.Diagnostics.Debug.WriteLine($"[EQUAL CHECK] Point {i} mismatch: dist={dist:F1} cm");
                    return false;
                }
            }
            return true;
        }

        // Fungsi update grid: Convert cm ke grid cell, set Obstacle
        private void UpdateGridFromVisionObstacles()
        {
            System.Diagnostics.Debug.WriteLine("[GRID UPDATE] Starting: Clearing old obstacles and setting new vision ones.");

            // 1. Clear SEMUA Obstacle (reset grid ke Free, termasuk manual – vision prioritas)
            for (int c = 0; c < gridCols; c++)
            {
                for (int r = 0; r < gridRows; r++)
                {
                    if (gridData[c, r] == CellType.Obstacle)
                    {
                        gridData[c, r] = CellType.Free;  // Clear semua obstacle
                    }
                }
            }

            // 2. Set vision obstacles baru (dengan inflate kecil untuk test)
            HashSet<Point> uniqueCells = new HashSet<Point>();
            int inflateRadius = 0;  // Mulai 0 untuk test tepat (tanpa inflate). Nanti naik ke 1 jika ok.
            foreach (PointF obsCm in currentVisionObstacles)
            {
                // Convert cm ke grid (bottom-left origin, X kanan+, Y atas+? Tunggu konfirmasi Y)
                int c = (int)Math.Floor(obsCm.X / currentGridCellSizeCm);
                int r = (int)Math.Floor(obsCm.Y / currentGridCellSizeCm);  // Jika Y invert salah, ganti ke (gridRows - 1 - floor(...))
                uniqueCells.Add(new Point(c, r));
                // Clamp ke bounds (hindari overflow)
                c = Math.Max(0, Math.Min(c, gridCols - 1));
                r = Math.Max(0, Math.Min(r, gridRows - 1));

                

                // Inflate loop
                for (int dc = -inflateRadius; dc <= inflateRadius; dc++)
                {
                    for (int dr = -inflateRadius; dr <= inflateRadius; dr++)
                    {
                        int targetC = c + dc;
                        int targetR = r + dr;
                        if (IsWithinGridBounds(targetC, targetR))
                        {
                            uniqueCells.Add(new Point(targetC, targetR));
                        }
                    }
                }
                System.Diagnostics.Debug.WriteLine($"[GRID UPDATE] Vision obs at cm({obsCm.X:F1}, {obsCm.Y:F1}) -> cell ({c}, {r})");
            }
            foreach (Point cell in uniqueCells)
            {
                gridData[cell.X, cell.Y] = CellType.Obstacle;
            }

            System.Diagnostics.Debug.WriteLine($"[GRID UPDATE] Set {uniqueCells.Count} unique cells from {currentVisionObstacles.Count} obs.");
        }

        private void EnsureFollowerConnection()
        {
            if (followerClient != null && followerClient.Connected && followerWriter != null) return;
            try
            {
                CleanupFollowerConnection();
                followerClient = new TcpClient();
                followerClient.NoDelay = true;
                var ar = followerClient.BeginConnect(FOLLOWER_IP, FOLLOWER_PORT, null, null);
                if (!ar.AsyncWaitHandle.WaitOne(500)) throw new Exception("Timeout connect");
                followerClient.EndConnect(ar);
                followerWriter = new StreamWriter(followerClient.GetStream(), new UTF8Encoding(false)) { AutoFlush = true };

                // optional: aktifkan gerak
                followerWriter.WriteLine("START");
                followerWriter.Flush();
            }
            catch
            {
                CleanupFollowerConnection();
            }
        }
        // --- sinkron pixel-per-grid dari vision ---
        private int LoadPixelsPerGrid(out string usedPath, int fallback = -1)
        {
            // Cari di beberapa lokasi umum:
            string[] candidates =
            {
                System.IO.Path.Combine(PoseDir, "pixels_per_grid.txt"),           // %LocalAppData%\RobotData\pixels_per_grid.txt
                System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "pixels_per_grid.txt"),
                "pixels_per_grid.txt"
            };

            foreach (var p in candidates)
            {
                try
                {
                    if (System.IO.File.Exists(p))
                    {
                        var txt = System.IO.File.ReadAllText(p).Trim();
                        if (int.TryParse(txt, System.Globalization.NumberStyles.Integer, CultureInfo.InvariantCulture, out int v) && v > 0)
                        {
                            usedPath = p;
                            return v;
                        }
                    }
                }
                catch { /* ignore and try next */ }
            }
            usedPath = null!;
            return fallback;
        }


        private void CleanupFollowerConnection()
        {
            try { followerWriter?.Dispose(); } catch { }
            try { followerClient?.Close(); } catch { }
            followerWriter = null;
            followerClient = null;
        }




        private void ClearObstaclesButton_Click(object sender, EventArgs e)
        {
            for (int c = 0; c < gridCols; c++)
            {
                for (int r = 0; r < gridRows; r++)
                {
                    // Hanya bersihkan obstacle, jangan timpa Start atau Goal yang mungkin ada
                    if (gridData[c, r] == CellType.Obstacle)
                    {
                        gridData[c, r] = CellType.Free;
                    }
                }
            }

            ClearPathAndButton(); // Hapus path yang mungkin jadi valid/invalid & nonaktifkan tombol

            // Goal mungkin masih ada, tidak perlu dihapus saat clear obstacle saja
            // Jika ingin goal juga hilang, tambahkan logika penghapusan currentGoalGridPos di sini

            arenaPictureBox.Invalidate();
            UpdateStatusLabel("Semua obstacle dibersihkan. Silakan set Goal baru atau buat obstacle.");
        }

        private async void SendWaypointsButton_Click(object sender, EventArgs e)
        {
            if (calculatedPath == null || !calculatedPath.Any())
            {
                MessageBox.Show("Tidak ada path untuk dikirim. Tentukan Goal terlebih dahulu.",
                    "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            // lock UI
            sendWaypointsButton.Enabled = false;
            toggleObstacleModeButton.Enabled = false;
            clearObstaclesButton.Enabled = false;
            changeGridButton.Enabled = false;
            updatePositionButton.Enabled = false;
            UpdateStatusLabel("Mengirim misi ke robot...");

            try
            {
                var CI = System.Globalization.CultureInfo.InvariantCulture;

                // 1) Ambil pose dari kamera (fallback ke grid kalau belum siap)
                var (ok, x_cm, y_cm, theta_rad) = await GetPoseFromVisionSystem();
                var startPoseCm = ok
                    ? new PointF(x_cm, y_cm)
                    : new PointF(
                        robotStartGridPos.X * currentGridCellSizeCm + currentGridCellSizeCm / 2f,
                        robotStartGridPos.Y * currentGridCellSizeCm + currentGridCellSizeCm / 2f
                      );
                float startThetaDeg = ok ? theta_rad * 180.0f / (float)Math.PI : 90.0f; // jika perlu derajat

                // 2) Pastikan konek ke Leader
                EnsureLeaderConnection();

                // 3) (HANYA SEKALI per run) sinkron odom leader dari kamera
                if (!_leaderSetposSent && ok)
                {
                    const bool USE_DEGREES = false; // true jika firmware leader masih pakai derajat
                    if (USE_DEGREES)
                        leaderWriter.WriteLine($"SETPOS_DEG {x_cm.ToString(CI)},{y_cm.ToString(CI)},{startThetaDeg.ToString(CI)}");
                    else
                        leaderWriter.WriteLine($"SETPOS {x_cm.ToString(CI)},{y_cm.ToString(CI)},{theta_rad.ToString(CI)}");
                    _leaderSetposSent = true;
                    await Task.Delay(10);
                }

                // 4) Reset misi & kirim waypoint grid
                leaderWriter.WriteLine("CLEAR");
                await Task.Delay(10);

                foreach (var wp in calculatedPath) // List<Point>
                {
                    leaderWriter.WriteLine($"{wp.X},{wp.Y}"); // jika firmware butuh Y-atas, ubah ke rTop = (gridRows-1)-wp.Y
                    await Task.Delay(5);
                }

                // 5) Mulai eksekusi
                leaderWriter.WriteLine("EXECUTE");
                UpdateStatusLabel("Misi dikirim. Leader mulai bergerak.");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error saat mengirim ke robot: {ex.Message}",
                    "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                UpdateStatusLabel("Error komunikasi dengan robot.");
                ResetForNewGoal();
            }
            finally
            {
                // unlock UI
                toggleObstacleModeButton.Enabled = true;
                clearObstaclesButton.Enabled = true;
                changeGridButton.Enabled = true;
                updatePositionButton.Enabled = true;
            }
        }

        private void ReadAndUpdateObstacles()
        {
            if (!File.Exists(OBSTACLE_FILE_PATH))
            {
                System.Diagnostics.Debug.WriteLine("[POSE TIMER] Obstacle file not found.");
                return;
            }

            try
            {
                string[] lines = File.ReadAllLines(OBSTACLE_FILE_PATH);
                List<PointF> newObstacles = new List<PointF>();

                foreach (string line in lines)
                {
                    if (string.IsNullOrWhiteSpace(line)) continue;
                    string[] parts = line.Split(',');
                    if (parts.Length == 2 && float.TryParse(parts[0], NumberStyles.Float, CI, out float xCm) &&
                        float.TryParse(parts[1], NumberStyles.Float, CI, out float yCm))
                    {
                        newObstacles.Add(new PointF(xCm, yCm));
                        System.Diagnostics.Debug.WriteLine($"[OBSTACLE READ] Parsed: ({xCm:F1}, {yCm:F1}) cm");
                    }
                }

                System.Diagnostics.Debug.WriteLine($"[POSE TIMER] Parsed {newObstacles.Count} obstacles");

                if (!AreObstaclesEqual(currentVisionObstacles, newObstacles))
                {
                    currentVisionObstacles = newObstacles;
                    UpdateGridFromVisionObstacles();  // Sudah ada dari sebelumnya
                    this.Invoke(new Action(() =>
                    {
                        arenaPictureBox.Invalidate();
                        arenaPictureBox.Refresh();
                        UpdateStatusLabel($"Vision obstacles updated: {newObstacles.Count} detected.");
                    }));
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[OBSTACLE ERROR] {ex.Message}");
            }
        }

        // Ganti nama fungsi atau buat fungsi baru dan kaitkan dengan MouseClick

        private void EnsureLeaderConnection()
        {
            if (leaderClient != null && leaderClient.Connected && leaderWriter != null) return;
            leaderClient?.Close();
            leaderClient = new TcpClient { NoDelay = true };
            var ar = leaderClient.BeginConnect(ROBOT_IP_ADDRESS, ROBOT_PORT, null, null);
            if (!ar.AsyncWaitHandle.WaitOne(1000)) throw new Exception("Timeout connect Leader");
            leaderClient.EndConnect(ar);
            leaderWriter = new StreamWriter(leaderClient.GetStream(), new UTF8Encoding(false)) { AutoFlush = true };
        }
        private void ResetForNewGoal()
        {
            ClearPathFromGrid(); // Bersihkan visual path

            // Jika misi sebelumnya memiliki goal (artinya berhasil sampai atau ditargetkan)
            if (currentGoalGridPos.HasValue)
            {
                // Posisi start BARU adalah posisi goal LAMA.
                // Hapus visual start lama (hijau)
                if (IsWithinGridBounds(robotStartGridPos.X, robotStartGridPos.Y))
                    gridData[robotStartGridPos.X, robotStartGridPos.Y] = CellType.Free;

                // Update posisi start ke posisi goal terakhir
                robotStartGridPos = currentGoalGridPos.Value;

                // Gambar start baru di posisi goal lama
                if (IsWithinGridBounds(robotStartGridPos.X, robotStartGridPos.Y))
                    gridData[robotStartGridPos.X, robotStartGridPos.Y] = CellType.Start;

                // Kosongkan goal saat ini
                currentGoalGridPos = null;
            }

            calculatedPath = null;
            sendWaypointsButton.Enabled = false; // Tombol kirim nonaktif sampai path baru dibuat
            arenaPictureBox.Invalidate(); //
        }

        private async Task SetleaderSenderLoop(CancellationToken token)
        {
            const int sendHz = 15; // 10�20 Hz cukup
            var sendDelay = TimeSpan.FromMilliseconds(1000.0 / sendHz);
            var retryDelay = TimeSpan.FromSeconds(1);

            while (!token.IsCancellationRequested)
            {
                try
                {
                    // Pastikan koneksi
                    if (followerClient == null || !followerClient.Connected || followerWriter == null)
                    {
                        CleanupFollowerConnection();
                        followerClient = new TcpClient();
                        var connectTask = followerClient.ConnectAsync(FOLLOWER_IP, FOLLOWER_PORT);
                        var done = await Task.WhenAny(connectTask, Task.Delay(1000, token));
                        if (done != connectTask) throw new TimeoutException("Connect timeout");
                        followerWriter = new StreamWriter(followerClient.GetStream(), new UTF8Encoding(false)) { AutoFlush = true };

                        // START sekali (opsional)
                        await followerWriter.WriteLineAsync("START");
                    }

                    // Kirim periodik
                    if (latestLeaderHas)
                    {
                        var line = string.Create(System.Globalization.CultureInfo.InvariantCulture, $"SETLEADER {latestLeaderX} {latestLeaderY} {latestLeaderTh}");
                        await followerWriter.WriteLineAsync(line);
                    }

                    await Task.Delay(sendDelay, token);
                }
                catch (Exception)
                {
                    // putus? bersihkan & coba lagi
                    CleanupFollowerConnection();
                    try { await Task.Delay(retryDelay, token); } catch { }
                }
            }
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            try { setleaderCts?.Cancel(); } catch { }
            CleanupFollowerConnection();
            base.OnFormClosing(e);
        }






        private void UpdateStatusLabel(string message)
        {
            statusLabel.Text = message;
        }

        private void toggleObstacleModeButton_Click_1(object sender, EventArgs e)
        {

        }

        private void ArenaPictureBox_MouseDown(object sender, MouseEventArgs e)
        {
            if (obstacleModeActive) // Hanya jika dalam mode obstacle
            {
                isPaintingObstacles = true; // Mulai mode melukis
                PaintObstacleAtMousePosition(e.X, e.Y); // Langsung gambar/hapus di sel pertama
            }
        }
        private void ArenaPictureBox_MouseMove(object sender, MouseEventArgs e)
        {
            if (obstacleModeActive && isPaintingObstacles) // Hanya jika mode obstacle & sedang melukis
            {
                PaintObstacleAtMousePosition(e.X, e.Y);
            }
        }
        private void ArenaPictureBox_MouseUp(object sender, MouseEventArgs e)
        {
            if (obstacleModeActive && isPaintingObstacles)
            {
                isPaintingObstacles = false; // Selesai melukis
            }
        }
        private void PaintObstacleAtMousePosition(int mouseX, int mouseY)
        {
            Point? gridPos = PixelToGrid(mouseX, mouseY);
            if (!gridPos.HasValue) return;

            int c = gridPos.Value.X;
            int r = gridPos.Value.Y;

            Point currentCellPoint = new Point(c, r);

            // Jangan ubah sel Start atau Goal menjadi obstacle
            if (currentCellPoint.Equals(robotStartGridPos) ||
                (currentGoalGridPos.HasValue && currentCellPoint.Equals(currentGoalGridPos.Value)))
            {
                return;
            }

            // Hanya ubah sel menjadi OBSTACLE jika sel tersebut saat ini FREE atau merupakan PATHNODE lama.
            // Jika sel tersebut sudah OBSTACLE, JANGAN lakukan apa-apa (tidak di-toggle menjadi Free).
            if (gridData[c, r] == CellType.Free || gridData[c, r] == CellType.PathNode)
            {
                gridData[c, r] = CellType.Obstacle; // Set menjadi obstacle

                // Jika ada path yang sudah ada, path tersebut kemungkinan besar tidak valid lagi
                // karena ada obstacle baru. Kita perlu membersihkannya.
                if (calculatedPath != null && calculatedPath.Any())
                {
                    ClearPathAndButton(); // Fungsi helper yang sudah kita buat
                }
                arenaPictureBox.Invalidate(); // Gambar ulang arena karena ada perubahan
            }
            // Jika gridData[c,r] sudah CellType.Obstacle, tidak ada yang dilakukan.
        }

        // Fungsi helper baru untuk membersihkan path dan menonaktifkan tombol
        private void ClearPathAndButton()
        {
            ClearPathFromGrid(); // Fungsi yang sudah ada untuk membersihkan visual path
            calculatedPath = null; // Hapus data path
            sendWaypointsButton.Enabled = false; // Nonaktifkan tombol kirim
                                                 // Tidak perlu Invalidate() di sini jika PaintObstacleAtMousePosition sudah memanggilnya
        }

        private void changeGridButton_Click(object sender, EventArgs e)
        {
            currentGridSizeIndex = (currentGridSizeIndex + 1) % availableGridCellSizesCm.Length;
            currentGridCellSizeCm = availableGridCellSizesCm[currentGridSizeIndex];
            changeGridButton.Text = $"Grid {currentGridCellSizeCm:F0}x{currentGridCellSizeCm:F0}";

            InitializeArenaAndGrid(); // Ini akan memanggil Refresh di akhirnya

            arenaPictureBox.Invalidate(); // Mungkin tidak perlu jika InitializeArenaAndGrid sudah panggil Refresh
            // arenaPictureBox.Refresh();    // Juga tidak perlu jika InitializeArenaAndGrid sudah panggil Refresh
            System.Diagnostics.Debug.WriteLine($"Grid changed. (InitializeArenaAndGrid called Refresh).");
            UpdateStatusLabel($"Ukuran grid diubah menjadi {currentGridCellSizeCm}x{currentGridCellSizeCm} cm. Klik arena untuk set Goal.");
        }

        private void btnUpdateFromCamera_Click(object sender, EventArgs e)
        {
            if (!TryReadPose(LEADER_POSE_PATH, out var lx, out var ly, out var lth))
            {
                MessageBox.Show(
                    "Gagal membaca leader_pose.txt.\nPastikan finalVision.py sedang berjalan dan file diperbarui.",
                    "Error Membaca File", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            // ====== Update grid dari cm -> sel ======
            // Misal cell size kamu 15 cm:
            float cellCm = currentGridCellSizeCm; // pakai variabel yang sudah ada di project-mu
            int gx = (int)Math.Floor(lx / cellCm);
            int gy = (int)Math.Floor(ly / cellCm);

            // Clamp ke batas grid
            gx = Math.Max(0, Math.Min(gridCols - 1, gx));
            gy = Math.Max(0, Math.Min(gridRows - 1, gy));

            // Bersihkan start lama lalu set titik baru (jika kamu pakai CellType.Start)
            if (IsWithinGridBounds(robotStartGridPos.X, robotStartGridPos.Y) &&
                gridData[robotStartGridPos.X, robotStartGridPos.Y] == CellType.Start)
            {
                gridData[robotStartGridPos.X, robotStartGridPos.Y] = CellType.Free;
            }
            robotStartGridPos = new Point(gx, gy);
            gridData[gx, gy] = CellType.Start;

            // Simpan pose leader untuk dipakai mengirim SETLEADER
            leaderX = lx; leaderY = ly; leaderTh = lth;

            latestLeaderX = lx; latestLeaderY = ly; latestLeaderTh = lth;
            latestLeaderHas = true;

            // Refresh gambar
            arenaPictureBox.Invalidate();

            // ====== (Opsional) Kirim SETLEADER ke Follower ======
            // Pastikan kamu sudah punya koneksi followerClient/followerWriter (lihat langkah 3)
            try
            {
                EnsureFollowerConnection(); // lihat fungsi di bawah
                followerWriter?.WriteLine(
                    $"SETLEADER {leaderX.ToString(CultureInfo.InvariantCulture)} " +
                    $"{leaderY.ToString(CultureInfo.InvariantCulture)} " +
                    $"{leaderTh.ToString(CultureInfo.InvariantCulture)}");
                followerWriter?.Flush();
            }
            catch { /* biarkan diam-diam, biar tidak ganggu UI */ }
        }


        private async void button1_Click(object sender, EventArgs e)
        {
            if (!TryReadPose(LEADER_POSE_PATH, out var lx, out var ly, out var lth))
            {
                MessageBox.Show(
                    "Gagal membaca leader_pose.txt.\nPastikan finalVision.py sedang berjalan dan file diperbarui.",
                    "Error Membaca File", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            // ====== Update grid dari cm -> sel ======
            // Misal cell size kamu 15 cm:
            float cellCm = currentGridCellSizeCm; // pakai variabel yang sudah ada di project-mu
            int gx = (int)Math.Floor(lx / cellCm);
            int gy = (int)Math.Floor(ly / cellCm);

            // Clamp ke batas grid
            gx = Math.Max(0, Math.Min(gridCols - 1, gx));
            gy = Math.Max(0, Math.Min(gridRows - 1, gy));

            // Bersihkan start lama lalu set titik baru (jika kamu pakai CellType.Start)
            if (IsWithinGridBounds(robotStartGridPos.X, robotStartGridPos.Y) &&
                gridData[robotStartGridPos.X, robotStartGridPos.Y] == CellType.Start)
            {
                gridData[robotStartGridPos.X, robotStartGridPos.Y] = CellType.Free;
            }
            robotStartGridPos = new Point(gx, gy);
            gridData[gx, gy] = CellType.Start;

            // Simpan pose leader untuk dipakai mengirim SETLEADER
            leaderX = lx; leaderY = ly; leaderTh = lth;

            // Refresh gambar
            arenaPictureBox.Invalidate();

            // ====== (Opsional) Kirim SETLEADER ke Follower ======
            // Pastikan kamu sudah punya koneksi followerClient/followerWriter (lihat langkah 3)
            try
            {
                EnsureFollowerConnection(); // lihat fungsi di bawah
                followerWriter?.WriteLine(
                    $"SETLEADER {leaderX.ToString(CultureInfo.InvariantCulture)} " +
                    $"{leaderY.ToString(CultureInfo.InvariantCulture)} " +
                    $"{leaderTh.ToString(CultureInfo.InvariantCulture)}");
                followerWriter?.Flush();
            }
            catch { /* biarkan diam-diam, biar tidak ganggu UI */ }
        }
        private async Task<(bool success, float x, float y, float theta)> GetPoseFromVisionSystem()
        {
            try
            {
                // 'using' akan otomatis menutup koneksi setelah selesai
                using (var client = new TcpClient())
                {
                    // Hubungkan ke server Python di localhost:9999
                    await client.ConnectAsync(VISION_SERVER_IP, VISION_SERVER_PORT);

                    using (var stream = client.GetStream())
                    using (var reader = new StreamReader(stream, Encoding.UTF8))
                    using (var writer = new StreamWriter(stream, new UTF8Encoding(false)) { AutoFlush = true })
                    {
                        // 1. Kirim perintah GET_POSE
                        await writer.WriteLineAsync("GET_POSE");

                        // 2. Baca respons dari server
                        string response = await reader.ReadLineAsync();

                        // 3. Parse respons
                        if (response != null && response.StartsWith("POSE:"))
                        {
                            // Ambil bagian setelah "POSE:", yaitu "X,Y,THETA"
                            string data = response.Substring(5);
                            string[] parts = data.Split(',');

                            if (parts.Length == 3)
                            {
                                // Gunakan InvariantCulture untuk memastikan '.' dibaca sebagai desimal
                                float x = float.Parse(parts[0], CultureInfo.InvariantCulture);
                                float y = float.Parse(parts[1], CultureInfo.InvariantCulture);
                                float theta = float.Parse(parts[2], CultureInfo.InvariantCulture);
                                float arenaH = ARENA_HEIGHT_REAL_CM;   // 150.0 by default
                                y = MathF.Abs(y - arenaH);
                                return (true, x, y, theta); // Kembalikan data yang berhasil diparse
                            }
                        }

                        // Jika respons tidak sesuai format atau null
                        return (false, 0, 0, 0);
                    }
                }
            }
            catch (Exception ex)
            {
                // Log error untuk debugging
                System.Diagnostics.Debug.WriteLine($"Error Koneksi Sistem Visi: {ex.Message}");
                return (false, 0, 0, 0); // Kembalikan indikasi kegagalan
            }
        }

        private void MainForm_Shown(object sender, EventArgs e)
        {
            System.Diagnostics.Debug.WriteLine("MainForm_Shown event triggered.");
            if (arenaPictureBox != null)
            {
                System.Diagnostics.Debug.WriteLine($"In Shown: arenaPictureBox.IsHandleCreated = {arenaPictureBox.IsHandleCreated}"); // <-- TAMBAHKAN INI
                System.Diagnostics.Debug.WriteLine($"In Shown: arenaPictureBox.Visible = {arenaPictureBox.Visible}"); // <-- TAMBAHKAN INI JUGA
                System.Diagnostics.Debug.WriteLine($"In Shown: arenaPictureBox.Width = {arenaPictureBox.Width}, Height = {arenaPictureBox.Height}");


                arenaPictureBox.Refresh();
                System.Diagnostics.Debug.WriteLine("Refresh called from MainForm_Shown.");
            }
            else
            {
                System.Diagnostics.Debug.WriteLine("ERROR: arenaPictureBox is NULL in MainForm_Shown!");
            }
            // InitializeArenaAndGrid(); // Panggil dari sini
            // UpdateStatusLabel("Klik arena untuk set Goal, atau aktifkan Mode Obstacle."); // Ini sudah ada di Load, bisa juga di sini jika mau
            // sendWaypointsButton.Enabled = false; // Juga sudah di Load

            // Panggil Invalidate di sini setelah InitializeArenaAndGrid yang mungkin juga memanggilnya.
            // Ini untuk memastikan permintaan repaint ada setelah form benar-benar tampil.
            if (arenaPictureBox != null && arenaPictureBox.IsHandleCreated && arenaPictureBox.Visible)
            {
                arenaPictureBox.Refresh();
                System.Diagnostics.Debug.WriteLine("Refresh called from MainForm_Shown.");
            }
        }
        // Di MainForm.cs

        private async Task<bool> SendMissionToRobotAsync(PointF startPoseCm, float startThetaDeg, List<Point> waypoints)
        {
            try
            {
                using (var client = new TcpClient())
                {
                    // Coba hubungkan dengan timeout singkat
                    var connectTask = client.ConnectAsync(ROBOT_IP_ADDRESS, ROBOT_PORT);
                    if (await Task.WhenAny(connectTask, Task.Delay(3000)) != connectTask)
                    {
                        // Timeout
                        throw new Exception("Connection to robot timed out.");
                    }

                    // Jika berhasil terhubung
                    using (var stream = client.GetStream())
                    using (var reader = new StreamReader(stream, Encoding.UTF8))
                    using (var writer = new StreamWriter(stream, new UTF8Encoding(false)) { AutoFlush = true })
                    {
                        // Tunggu pesan selamat datang dari ESP (opsional tapi bagus)
                        string welcomeMsg = await reader.ReadLineAsync();
                        System.Diagnostics.Debug.WriteLine($"ROBOT (Welcome): {welcomeMsg}");

                        // --- URUTAN PENGIRIMAN PERINTAH ---

                        // 1. Kirim CLEAR
                        await writer.WriteLineAsync("CLEAR");
                        string clearResponse = await reader.ReadLineAsync(); // Baca feedback "Received: [CLEAR]"
                        System.Diagnostics.Debug.WriteLine($"ROBOT (Feedback): {clearResponse}");
                        clearResponse = await reader.ReadLineAsync(); // Baca feedback "Waypoints cleared..."
                        System.Diagnostics.Debug.WriteLine($"ROBOT (Feedback): {clearResponse}");

                        // 2. Kirim SETPOS
                        string setposCmd = $"SETPOS {startPoseCm.X.ToString(CultureInfo.InvariantCulture)},{startPoseCm.Y.ToString(CultureInfo.InvariantCulture)},{startThetaDeg.ToString(CultureInfo.InvariantCulture)}";
                        await writer.WriteLineAsync(setposCmd);
                        await reader.ReadLineAsync(); // Baca feedback "Received: [SETPOS...]"
                        await reader.ReadLineAsync(); // Baca feedback "Odometry set to..."

                        // 3. Kirim semua Waypoints
                        foreach (Point wp in waypoints)
                        {
                            string wpCmd = $"{wp.X},{wp.Y}";
                            await writer.WriteLineAsync(wpCmd);
                            await reader.ReadLineAsync(); // Baca feedback "Received: [X,Y]"
                            await reader.ReadLineAsync(); // Baca feedback "Stored waypoint #..."
                        }

                        // 4. Kirim EXECUTE
                        await writer.WriteLineAsync("EXECUTE");
                        await reader.ReadLineAsync(); // Baca feedback "Received: [EXECUTE]"
                        await reader.ReadLineAsync(); // Baca feedback "Executing..."


                        // 5. Tunggu Konfirmasi Misi Selesai
                        UpdateStatusLabel("Misi berjalan... Menunggu konfirmasi selesai.");
                        while (client.Connected)
                        {
                            // Set timeout untuk setiap pembacaan agar tidak menunggu selamanya
                            var readTask = reader.ReadLineAsync();
                            if (await Task.WhenAny(readTask, Task.Delay(120000)) != readTask) // Timeout 2 menit
                            {
                                throw new Exception("Did not receive completion signal from robot (timeout).");
                            }

                            string response = await readTask;
                            if (response == null)
                            { // Klien terputus
                                throw new Exception("Connection to robot lost.");
                            }

                            UpdateStatusLabel($"Robot: {response}");
                            System.Diagnostics.Debug.WriteLine($"ROBOT (Runtime): {response}");

                            if (response.Contains("ALL_WAYPOINTS_REACHED"))
                            {
                                MessageBox.Show("Robot telah mencapai tujuan akhir!", "Misi Selesai", MessageBoxButtons.OK, MessageBoxIcon.Information);
                                ResetForNewGoal(); // Panggil reset GUI setelah misi benar-benar selesai
                                return true; // Misi sukses
                            }
                        }
                        return false; // Jika loop selesai karena koneksi terputus
                    }
                }
            }
            catch (Exception ex)
            {
                // Log error untuk debugging dan kembalikan false
                System.Diagnostics.Debug.WriteLine($"Error during robot mission: {ex.Message}");
                System.Diagnostics.Debug.WriteLine($"!!! KONEKSI GAGAL: {ex.GetType().Name} - {ex.Message}");
                System.Diagnostics.Debug.WriteLine($"--- Stack Trace ---");
                System.Diagnostics.Debug.WriteLine(ex.StackTrace);
                return false;
            }
        }



        private void MainForm_Paint(object sender, PaintEventArgs e)
        {
            System.Diagnostics.Debug.WriteLine("--- MainForm_Paint event triggered! ---");
        }

        private Point? PixelToGrid(int mouseX, int mouseY)
        {
            if (cellSizePixelsFloat <= 0) return null;

            // Hitung kolom (c) dengan memperhitungkan offset
            int c = (int)Math.Floor((mouseX - gridOffsetX) / cellSizePixelsFloat);

            // Hitung baris (r) dengan logika terbalik untuk Y
            int r_from_top = (int)Math.Floor((mouseY - gridOffsetY) / cellSizePixelsFloat);
            int r = (gridRows - 1) - r_from_top; // Membalikkan: 0 sekarang di bawah

            // Kembalikan koordinat grid jika valid
            if (IsWithinGridBounds(c, r))
            {
                return new Point(c, r);
            }
            return null; // Klik di luar area grid
        }

        private void arenaPictureBox_Click(object sender, EventArgs e)
        {

        }

        private void sendWaypointsButton_Click_1(object sender, EventArgs e)
        {

        }
    }
}