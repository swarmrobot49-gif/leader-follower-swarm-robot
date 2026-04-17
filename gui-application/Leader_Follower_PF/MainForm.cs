using System.Globalization;  // Untuk memastikan parsing angka desimal (koma vs titik) benar
using System.IO;           // Untuk membaca dan menulis data stream dengan mudah
using System.Net.Sockets; // Komponen utama untuk networking (TcpClient)
using System.Text;         // Untuk mengubah string menjadi byte dan sebaliknya
using System.Threading;
using System.Net;
using Newtonsoft.Json;  // Tambah di using
using Newtonsoft.Json.Linq;
using System.Timers;
using System.Drawing;
using System.Collections.Generic;

namespace HybridDijkstraPotentialField
{
    public enum CellType { Free, Obstacle, Start, Goal, PathNode }
    public enum PathPlanningMode
    {
        HybridDijkstraPF = 0, // Dijkstra + PF smoothing
        DijkstraOnly = 1,           // Original
        PurePotentialField = 2,     // Pure PF
        PurePSO = 3,                // Pure PSO
        PureReactivePF = 4

    }

    public partial class MainForm : Form
    {
        // === Konstanta Arena & Grid ===
        private const float ARENA_WIDTH_REAL_CM = 200.0f;  // 2 meter
        private const float ARENA_HEIGHT_REAL_CM = 150.0f; // 1.5 meter
        private const float ROBOT_WIDTH_CM = 15.0f;
        private const float ROBOT_HEIGHT_CM = 15.0f;

        // Physical size of yellow obstacles detected by vision system
        // Adjust this if your actual obstacles are different size
        private const float VISION_OBSTACLE_SIZE_CM = 15.0f;  // 15cm x 15cm typical

        private bool isPaintingObstacles = false;
        // private const float GRID_CELL_SIZE_REAL_CM = 15.0f; // Ukuran sel grid fisik
        private float[] availableGridCellSizesCm = { 15.0f, 10.0f, 5.0f };
        private int currentGridSizeIndex = 2; // Indeks untuk availableGridCellSizesCm
        private float currentGridCellSizeCm;  // Ukuran sel grid saat ini

        private PathPlanningMode currentMode = PathPlanningMode.PureReactivePF; // Default hybrid
        private PotentialFieldPlanner pfPlanner;
        private PSOPathPlanner psoPlanner;
        private System.Diagnostics.Stopwatch pathPlanningStopwatch = new System.Diagnostics.Stopwatch();

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
        private const string FOLLOWER_IP = "10.234.70.9";
        private const int FOLLOWER_PORT = 6060;
        private TcpClient followerClient;
        private StreamWriter followerWriter;
        private CancellationTokenSource setleaderCts;

        #region BLUE GOAL VARIABLES - PASTE AFTER OTHER VARIABLE DECLARATIONS

        // === BLUE GOAL AUTO-DETECTION ===
        private static readonly string BLUE_GOAL_FILE_PATH = Path.Combine(PoseDir, "goal_blue.txt");

        private bool enableBlueGoalAutoDetect = true;  // Toggle untuk enable/disable auto-detect
        private CheckBox chkEnableBlueGoal;            // UI checkbox
        private Label lblBlueGoalStatus;               // UI status label
        private PointF? lastBlueGoalCm = null;         // Posisi goal biru terakhir (cm)
        private DateTime lastBlueGoalFileTime = DateTime.MinValue;  // Timestamp file terakhir
        private long _lastBlueGoalReadMs = 0;          // Throttle reading
        private const int BLUE_GOAL_READ_PERIOD_MS = 200;  // Baca file max 5 Hz

        #endregion

        // ============================================================
        // SECTION 2: TAMBAHKAN METHOD-METHOD BARU INI
        // ============================================================

        #region BLUE GOAL METHODS - PASTE AS NEW METHODS IN MainForm CLASS


        private volatile bool latestLeaderHas = false;
        private volatile float latestLeaderX, latestLeaderY, latestLeaderTh;
        private System.Windows.Forms.Timer poseTimer;

        private ReactivePotentialFieldController leaderReactivePF;
        private FollowerReactivePFController followerReactivePF;
        private bool isReactivePFMode = false;
        private const bool ENABLE_LEGACY_FOLLOWER_STREAM = false; // set true jika ingin kembali ke follower2 legacy
        private PointF lastForceVector = PointF.Empty;
        private bool showForceVisualization = true;

        // UI untuk Reactive PF
        private GroupBox reactivePFGroupBox;
        private Button btnStartReactivePF;
        private Button btnStopReactivePF;
        private Label lblReactivePFStatus;
        private Label lblForceVisualization;

        // Tuning controls
        private GroupBox tuningGroupBox;
        private TrackBar trackAttractiveGain;
        private TrackBar trackRepulsiveGain;
        private TrackBar trackObstacleRange;
        private Label lblAttractiveGain;
        private Label lblRepulsiveGain;
        private Label lblObstacleRange;


        private int gridOffsetX = 0;
        private int gridOffsetY = 0;
        bool _leaderSetposSent = false;

        private int _poseTickBusy = 0;
        private DateTime _leaderPoseWriteUtc = DateTime.MinValue;
        private DateTime _followerPoseWriteUtc = DateTime.MinValue;
        private DateTime _obstaclesWriteUtc = DateTime.MinValue;
        private long _lastObstacleReadMs = 0;
        private const int OBSTACLE_READ_PERIOD_MS = 250; // baca obstacle max 4 Hz (cukup banget)
        private long _lastStatusMs = 0;

        // Di bagian deklarasi variabel (sekitar line 60-100)
        private volatile float safeFollowerX = 0, safeFollowerY = 0, safeFollowerTh = 0;
        private volatile bool haveSafeFollowerPose = false;

        private long _lastForceUiMs = 0;
        // === PF scenario logging (1 folder per START) ===
        private readonly object _pfLogLock = new object();
        private StreamWriter _pfLogWriter = null;
        private System.Diagnostics.Stopwatch _pfLogSw = null;
        private string _pfScenarioDir = null;
        private int _pfLogRowCount = 0;


        private List<Point> globalDijkstraPath = null;

        private float pixelPerCm = 2.0f;
        private int setposCounter = 0;

        private TcpClient leaderClient;
        private StreamWriter leaderWriter;
        private TelemetryLogger telemetryLogger;

        private static readonly string OBSTACLE_FILE_PATH = Path.Combine(PoseDir, "obstacles.txt");
        private System.Timers.Timer obstacleUpdateTimer;
        private List<PointF> currentVisionObstacles = new List<PointF>();  // Cache posisi cm
        private HashSet<Point> manualObstacles = new HashSet<Point>();

        private PointF robotFixedPhysicalStartCm = new PointF(22.5f, 22.5f);

        private int gridCols;
        private int gridRows;
        private float cellSizePixelsFloat; // Ukuran sel dalam pixel di GUI, akan dihitung

        private const string ROBOT_IP_ADDRESS = "10.234.70.23";
        private const int ROBOT_PORT = 5555;

        // === Status Grid ===

        private CellType[,] gridData;

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
            this.DoubleBuffered = true;
            arenaPanel.DoubleBuffered(true);   // ganti arenaPanel sesuai nama panel gambar kamu


            toggleObstacleModeButton.Click += ToggleObstacleModeButton_Click;
            clearObstaclesButton.Click += ClearObstaclesButton_Click;
            sendWaypointsButton.Click += SendWaypointsButton_Click;
            changeGridButton.Click += changeGridButton_Click;

            algorithmComboBox.SelectedIndexChanged += AlgorithmComboBox_SelectedIndexChanged;
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

            // Save initial grid config for vision system
            SaveGridConfigToFile(currentGridCellSizeCm);

            InitializeArenaAndGrid(); // Panggil untuk setup awal
            InitializeBlueGoalUI();
            InitializeTelemetryLogger();
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
            if (ENABLE_LEGACY_FOLLOWER_STREAM)
            {
                setleaderCts = new CancellationTokenSource();
                _ = Task.Run(() => SetleaderSenderLoop(setleaderCts.Token));
            }
            UpdateStatusLabel("Form loaded. Grid initialized.");
            sendWaypointsButton.Enabled = false;

            System.Diagnostics.Debug.WriteLine($"arenaPictureBox.Visible after Load init: {this.arenaPictureBox.Visible}");
            // arenaPi
            {
                string _path;
                int ppg = LoadPixelsPerGrid(out _path, -1);
                if (ppg > 0) UpdateStatusLabel($"Pixel-per-grid sinkron: {ppg}px/sel (sumber: {_path})");
            }
            InitializeReactivePF();

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

            // === PERBAIKAN: Hitung pixel per sel agar SELURUH grid fit di PictureBox ===
            float pixelsPerCellX = (float)arenaPictureBox.ClientSize.Width / (float)gridCols;
            float pixelsPerCellY = (float)arenaPictureBox.ClientSize.Height / (float)gridRows;
            cellSizePixelsFloat = Math.Min(pixelsPerCellX, pixelsPerCellY);

            // --- DISABLED: Override dari vision causing grid to shrink ---
            // Vision's pixels_per_grid is calculated for camera resolution,
            // NOT for GUI PictureBox size. Always use GUI-calculated value.
            /*
            string srcPath;
            int ppg = LoadPixelsPerGrid(out srcPath, -1);
            if (ppg > 0)
            {
                float maxCellSizeToFit = Math.Min(pixelsPerCellX, pixelsPerCellY);

                if (ppg <= maxCellSizeToFit * 1.1f)
                {
                    cellSizePixelsFloat = ppg;
                    System.Diagnostics.Debug.WriteLine($"[GRID] Using pixels_per_grid from vision: {ppg} px/sel (src: {srcPath})");
                }
                else
                {
                    System.Diagnostics.Debug.WriteLine($"[GRID] Vision ppg ({ppg}) too large, using calculated: {cellSizePixelsFloat:F1}px");
                }
            }
            else
            {
                System.Diagnostics.Debug.WriteLine($"[GRID] pixels_per_grid.txt not found -> using calculated cellSizePixelsFloat = {cellSizePixelsFloat:F1}px");
            }
            */

            System.Diagnostics.Debug.WriteLine($"[GRID] Using GUI-calculated cellSizePixelsFloat = {cellSizePixelsFloat:F1}px (vision override disabled)");


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
            lastBlueGoalCm = null;
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
            manualObstacles.Clear(); // Reset manual obstacles saat grid di-reinitialize
            System.Diagnostics.Debug.WriteLine("[MANUAL OBSTACLE] Cleared on grid reinitialize.");
            UpdateGridFromVisionObstacles();  // ← ADD THIS LINE!
            System.Diagnostics.Debug.WriteLine("[GRID INIT] Vision obstacles repopulated.");

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
            DrawBlueGoalIndicator(g);
            DrawReactivePFVisualization(g);
            DrawFollowerOverlay(g);

        }

        // --- Event Handler untuk Klik Mouse di Arena ---
        private void ArenaPictureBox_MouseClick_ForGoalSetting(object sender, MouseEventArgs e)
        {
            if (obstacleModeActive) return;

            if (enableBlueGoalAutoDetect)
            {
                UpdateStatusLabel("⚠️ Blue goal auto-detect is ON. Uncheck to set goal manually.");
                return;
            }

            Point? gridPos = PixelToGrid(e.X, e.Y);
            if (!gridPos.HasValue) return;

            int c = gridPos.Value.X;
            int r = gridPos.Value.Y;

            // 2. Pastikan klik berada di dalam batas grid yang valid
            if (!IsWithinGridBounds(c, r))
            {
                return; // Klik di luar area grid yang digambar
            }
            // === REACTIVE PF MODE: set goal langsung ke controller (tanpa FindAndDisplayPath) ===
            if (isReactivePFMode)
            {
                // Update visual goal saja (opsional, biar kelihatan di GUI)
                if (currentGoalGridPos.HasValue)
                {
                    Point oldGoal = currentGoalGridPos.Value;
                    if (IsWithinGridBounds(oldGoal.X, oldGoal.Y) && gridData[oldGoal.X, oldGoal.Y] == CellType.Goal)
                        gridData[oldGoal.X, oldGoal.Y] = CellType.Free;
                }

                currentGoalGridPos = new Point(c, r);
                gridData[c, r] = CellType.Goal;

                // Set goal ke Reactive PF (pakai helper yang sudah ada di controller)
                leaderReactivePF?.SetGoalFromGrid(c, r, currentGridCellSizeCm);

                // Status ringan (tanpa "Menghitung path...")
                float xCm = c * currentGridCellSizeCm + currentGridCellSizeCm / 2f;
                float yCm = r * currentGridCellSizeCm + currentGridCellSizeCm / 2f;
                UpdateStatusLabel($"Reactive PF goal: ({xCm:F1},{yCm:F1}) cm");

                arenaPictureBox.Invalidate();
                return;
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

            // ====== MAIN PATH PLANNING LOGIC ======
            pathPlanningStopwatch.Restart();



            switch (currentMode)
            {
                case PathPlanningMode.DijkstraOnly:
                    // Original Dijkstra (tetap ada untuk comparison)
                    calculatedPath = DijkstraPathfinderNoDiagObst.FindPath(
                        pathfindingGrid,
                        robotStartGridPos,
                        currentGoalGridPos.Value,
                        gridCols, gridRows
                    );
                    break;

                case PathPlanningMode.HybridDijkstraPF:
                    // === HYBRID APPROACH (RECOMMENDED) ===

                    // Step 1: Dijkstra untuk global path
                    List<Point> dijkstraPath = DijkstraPathfinderNoDiagObst.FindPath(
                        pathfindingGrid,
                        robotStartGridPos,
                        currentGoalGridPos.Value,
                        gridCols, gridRows
                    );

                    if (dijkstraPath != null && dijkstraPath.Any())
                    {
                        globalDijkstraPath = dijkstraPath;  // <-- simpan

                        if (pfPlanner == null)
                        {
                            pfPlanner = new PotentialFieldPlanner(
                                pathfindingGrid,
                                gridCols,
                                gridRows,
                                currentGridCellSizeCm
                            );
                        }

                        List<PointF> smoothedPathCm = pfPlanner.SmoothPath(
                            dijkstraPath,
                            currentVisionObstacles
                        );

                        calculatedPath = ConvertCmPathToGrid(smoothedPathCm);
                        calculatedPath = RemoveUnsafeDiagonals(calculatedPath, pathfindingGrid);
                    }
                    else
                    {
                        calculatedPath = null;
                    }
                    break;

                case PathPlanningMode.PurePSO:
                    // === PURE PSO ===
                    if (psoPlanner == null)
                    {
                        psoPlanner = new PSOPathPlanner(
                            pathfindingGrid,
                            gridCols,
                            gridRows,
                            currentGridCellSizeCm
                        );
                    }

                    calculatedPath = psoPlanner.FindPath(
                        robotStartGridPos,
                        currentGoalGridPos.Value
                    );
                    break;

                case PathPlanningMode.PurePotentialField:
                    // === PURE POTENTIAL FIELD ===
                    if (pfPlanner == null)
                    {
                        pfPlanner = new PotentialFieldPlanner(
                            pathfindingGrid,
                            gridCols,
                            gridRows,
                            currentGridCellSizeCm
                        );
                    }

                    calculatedPath = pfPlanner.FindPath(
                        robotStartGridPos,
                        currentGoalGridPos.Value
                    );
                    break;
            }

            pathPlanningStopwatch.Stop();

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
                float pathLength = CalculatePathLength(calculatedPath);
                float smoothness = CalculatePathSmoothness(calculatedPath);


                UpdateStatusLabel(
                    $"Path found! Method: {currentMode}, " +
                    $"Waypoints: {calculatedPath.Count}, " +
                    $"Length: {pathLength:F1} cm, " +
                    $"Computation: {pathPlanningStopwatch.ElapsedMilliseconds} ms"
                );
                sendWaypointsButton.Enabled = true;
            }
            else
            {
                UpdateStatusLabel($"Path tidak ditemukan menggunakan {currentMode}.");
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

        private void SaveGridConfigToFile(float cellSizeCm)
        {
            try
            {
                string gridConfigPath = Path.Combine(PoseDir, "grid_config.txt");
                string content = cellSizeCm.ToString("F1", CultureInfo.InvariantCulture);

                // Atomic write using temp file
                string tempPath = gridConfigPath + ".tmp";
                File.WriteAllText(tempPath, content);
                File.Move(tempPath, gridConfigPath, true); // Overwrite if exists

                System.Diagnostics.Debug.WriteLine($"[GRID CONFIG] Saved to file: {cellSizeCm}cm");
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[GRID CONFIG] Error saving: {ex.Message}");
            }
        }

        private void PoseTimer_Tick(object sender, EventArgs e)
        {
            if (System.Threading.Interlocked.Exchange(ref _poseTickBusy, 1) == 1) return;
            try
            {
                // 1) Baca file pose (overwrite setiap deteksi)
                haveLeaderPose = TryReadPose(LEADER_POSE_PATH, out leaderX, out leaderY, out leaderTh);
                haveFollowerPose = TryReadPose(FOLLOWER_POSE_PATH, out followerX, out followerY, out followerTh);

                ReadAndUpdateObstacles();
                UpdateGoalFromBlueDetection();

                // Cache leader pose untuk streamer legacy (kalau dipakai)
                if (haveLeaderPose)
                {
                    latestLeaderX = leaderX;
                    latestLeaderY = leaderY;
                    latestLeaderTh = leaderTh;
                    latestLeaderHas = true;
                }
                if (haveFollowerPose)
                {
                    safeFollowerX = followerX;
                    safeFollowerY = followerY;
                    safeFollowerTh = followerTh;
                    haveSafeFollowerPose = true;
                }

                // 2) Update Start di grid (posisi robot yang sedang jalan) - gunakan leader pose
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

                // =========================================================
                // PF MODE: MainForm tidak boleh kirim SETPOS/SETLEADER ke follower.
                // =========================================================
                if (isReactivePFMode)
                {
                    setposCounter = 0;
                    UpdateReactivePFFromVision();

                    // NEW: tetap repaint supaya overlay follower/leader ikut update dari pose file
                    if (haveLeaderPose || haveFollowerPose) arenaPictureBox.Invalidate();

                    return;
                }


                // =========================================================
                // NON-PF MODE (legacy follower2): kirim SETPOS + SETLEADER
                // =========================================================
                if (ENABLE_LEGACY_FOLLOWER_STREAM)
                {
                    setposCounter++;
                    if (setposCounter >= 5) // timer 100 ms => 0.5 s
                    {
                        setposCounter = 0;
                        EnsureFollowerConnection();

                        if (haveFollowerPose)
                        {
                            followerWriter?.WriteLine(
                                $"SETPOS {followerX.ToString(CultureInfo.InvariantCulture)} " +
                                $"{followerY.ToString(CultureInfo.InvariantCulture)} " +
                                $"{followerTh.ToString(CultureInfo.InvariantCulture)}");
                            followerWriter?.Flush();
                        }
                    }

                    // Stream SETLEADER ke follower (rad, cm)
                    if (haveLeaderPose && followerWriter != null)
                    {
                        try
                        {
                            followerWriter.WriteLine(
                                $"SETLEADER {leaderX.ToString(CultureInfo.InvariantCulture)} " +
                                $"{leaderY.ToString(CultureInfo.InvariantCulture)} " +
                                $"{leaderTh.ToString(CultureInfo.InvariantCulture)}");
                            followerWriter.Flush();
                        }
                        catch
                        {
                            // Rekoneksi nanti otomatis di tick berikutnya
                            CleanupFollowerConnection();
                        }
                    }
                }



                UpdateReactivePFFromVision();
            }
            finally
            {
                System.Threading.Interlocked.Exchange(ref _poseTickBusy, 0);
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
            System.Diagnostics.Debug.WriteLine("[GRID UPDATE] Starting: Updating vision obstacles while preserving manual ones.");

            // 1. Clear all obstacles first
            for (int c = 0; c < gridCols; c++)
            {
                for (int r = 0; r < gridRows; r++)
                {
                    if (gridData[c, r] == CellType.Obstacle)
                    {
                        gridData[c, r] = CellType.Free;
                    }
                }
            }

            // 2. Restore manual obstacles FIRST (prioritas manual)
            foreach (Point manualObs in manualObstacles)
            {
                if (IsWithinGridBounds(manualObs.X, manualObs.Y))
                {
                    gridData[manualObs.X, manualObs.Y] = CellType.Obstacle;
                }
            }

            System.Diagnostics.Debug.WriteLine($"[GRID UPDATE] Restored {manualObstacles.Count} manual obstacles.");

            // 3. Add vision obstacles with proper inflation based on physical size
            HashSet<Point> visionCells = new HashSet<Point>();

            // CRITICAL FIX: Calculate inflation radius based on physical obstacle size
            // Calculate how many cells the obstacle should occupy
            // For 15cm grid: 15/15 = 1 cell (radius 0) - just center point
            // For 10cm grid: 15/10 = 1.5 cells (radius 1) - 3x3 block
            // For 5cm grid:  15/5  = 3 cells (radius 1) - 3x3 block
            int inflateRadius = (int)Math.Ceiling((VISION_OBSTACLE_SIZE_CM / currentGridCellSizeCm) / 2.0f) - 1;
            inflateRadius = Math.Max(0, inflateRadius); // Ensure non-negative

            System.Diagnostics.Debug.WriteLine($"[GRID UPDATE] Using inflation radius = {inflateRadius} cells for {currentGridCellSizeCm}cm grid (physical obstacle: {VISION_OBSTACLE_SIZE_CM}cm)");

            foreach (PointF obsCm in currentVisionObstacles)
            {
                int c = (int)Math.Floor(obsCm.X / currentGridCellSizeCm);
                int r = (int)Math.Floor(obsCm.Y / currentGridCellSizeCm);

                // Clamp ke bounds
                c = Math.Max(0, Math.Min(c, gridCols - 1));
                r = Math.Max(0, Math.Min(r, gridRows - 1));

                // Inflate loop to create NxN block representing physical obstacle size
                for (int dc = -inflateRadius; dc <= inflateRadius; dc++)
                {
                    for (int dr = -inflateRadius; dr <= inflateRadius; dr++)
                    {
                        int targetC = c + dc;
                        int targetR = r + dr;
                        if (IsWithinGridBounds(targetC, targetR))
                        {
                            visionCells.Add(new Point(targetC, targetR));
                        }
                    }
                }
                System.Diagnostics.Debug.WriteLine($"[GRID UPDATE] Vision obs at cm({obsCm.X:F1}, {obsCm.Y:F1}) -> cell ({c}, {r}) inflated to {(2 * inflateRadius + 1)}x{(2 * inflateRadius + 1)} block");
            }

            // Set vision obstacles di grid
            foreach (Point cell in visionCells)
            {
                if (IsWithinGridBounds(cell.X, cell.Y))
                {
                    gridData[cell.X, cell.Y] = CellType.Obstacle;
                }
            }

            System.Diagnostics.Debug.WriteLine($"[GRID UPDATE] Set {visionCells.Count} vision cells + {manualObstacles.Count} manual obstacles.");
        }

        private void DrawPFLegend(Graphics g)
        {
            // Position legend di pojok kanan atas
            int legendX = 10;
            int legendY = arenaPictureBox.Height - 100;
            int legendWidth = 270;
            int legendHeight = 90;

            // Background box dengan semi-transparent white
            using (Brush bgBrush = new SolidBrush(Color.FromArgb(220, 255, 255, 255)))
            {
                g.FillRectangle(bgBrush, legendX, legendY, legendWidth, legendHeight);
            }

            // Border
            using (Pen borderPen = new Pen(Color.Gray, 2))
            {
                g.DrawRectangle(borderPen, legendX, legendY, legendWidth, legendHeight);
            }

            // Title
            using (Font titleFont = new Font("Segoe UI", 10, FontStyle.Bold))
            {
                g.DrawString("Legend - Panah Robot:", titleFont, Brushes.Black, legendX + 10, legendY + 8);
            }

            using (Font textFont = new Font("Segoe UI", 9))
            {
                // === Blue Arrow (Heading) ===
                int arrowY1 = legendY + 35;

                // Draw blue arrow sample
                Point arrowStart1 = new Point(legendX + 15, arrowY1);
                Point arrowEnd1 = new Point(legendX + 55, arrowY1);
                using (Pen bluePen = new Pen(Color.Blue, 2))
                {
                    bluePen.EndCap = System.Drawing.Drawing2D.LineCap.ArrowAnchor;
                    g.DrawLine(bluePen, arrowStart1, arrowEnd1);
                }

                // Label
                g.DrawString("= Robot's Heading", textFont, Brushes.Black, legendX + 65, arrowY1 - 8);

                // === Magenta Arrow (Force Vector) ===
                int arrowY2 = legendY + 60;

                // Draw magenta arrow sample
                Point arrowStart2 = new Point(legendX + 15, arrowY2);
                Point arrowEnd2 = new Point(legendX + 55, arrowY2);
                using (Pen magentaPen = new Pen(Color.Magenta, 3))
                {
                    magentaPen.EndCap = System.Drawing.Drawing2D.LineCap.ArrowAnchor;
                    g.DrawLine(magentaPen, arrowStart2, arrowEnd2);
                }

                // Label
                g.DrawString("= Total Forces", textFont, Brushes.Black, legendX + 65, arrowY2 - 8);
            }
        }

        private void InitializeBlueGoalUI()
        {
            // GroupBox untuk Blue Goal
            GroupBox blueGoalGroupBox = new GroupBox
            {
                Text = "🔵 Blue Goal Detection",
                Location = new Point(10, 760),  // Sesuaikan posisi sesRuai layout Anda
                Size = new Size(280, 80),
                Font = new Font("Segoe UI", 9, FontStyle.Bold)
            };

            // Checkbox untuk enable/disable auto-detect
            chkEnableBlueGoal = new CheckBox
            {
                Text = "Auto-detect Blue Goal from Vision",
                Location = new Point(10, 22),
                Size = new Size(250, 22),
                Checked = enableBlueGoalAutoDetect,
                Font = new Font("Segoe UI", 9)
            };
            chkEnableBlueGoal.CheckedChanged += (s, e) =>
            {
                enableBlueGoalAutoDetect = chkEnableBlueGoal.Checked;
                if (enableBlueGoalAutoDetect)
                {
                    //UpdateStatusLabel("🔵 Blue goal auto-detection ENABLED");
                }
                else
                {
                    //UpdateStatusLabel("Blue goal auto-detection DISABLED - Click arena to set goal manually");
                    // Reset blue goal state
                    lastBlueGoalCm = null;
                    //UpdateBlueGoalStatusLabel("Blue Goal: Disabled", Color.Gray);
                }
            };

            // Label untuk status blue goal
            lblBlueGoalStatus = new Label
            {
                Text = "Blue Goal: Waiting for detection...",
                Location = new Point(10, 48),
                Size = new Size(260, 20),
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 9)
            };

            //blueGoalGroupBox.Controls.Add(chkEnableBlueGoal);
            //blueGoalGroupBox.Controls.Add(lblBlueGoalStatus);

            //this.Controls.Add(blueGoalGroupBox);
            //blueGoalGroupBox.BringToFront();

            System.Diagnostics.Debug.WriteLine("[BLUE GOAL] UI Initialized");
        }

        /// <summary>
        /// Baca posisi goal biru dari file yang ditulis oleh vision system
        /// </summary>
        private bool TryReadBlueGoal(out float x_cm, out float y_cm)
        {
            x_cm = 0;
            y_cm = 0;

            try
            {
                if (!File.Exists(BLUE_GOAL_FILE_PATH))
                    return false;

                // Cek apakah file sudah diupdate (berdasarkan timestamp)
                var fileInfo = new FileInfo(BLUE_GOAL_FILE_PATH);
                if (fileInfo.LastWriteTimeUtc <= lastBlueGoalFileTime)
                    return false;  // File belum berubah, skip

                lastBlueGoalFileTime = fileInfo.LastWriteTimeUtc;

                // Baca isi file
                string content = File.ReadAllText(BLUE_GOAL_FILE_PATH).Trim();
                if (string.IsNullOrWhiteSpace(content))
                    return false;

                // Parse format: "x_cm,y_cm"
                string[] parts = content.Split(',');
                if (parts.Length < 2)
                    return false;

                x_cm = float.Parse(parts[0], CultureInfo.InvariantCulture);
                y_cm = float.Parse(parts[1], CultureInfo.InvariantCulture);

                // Validasi: harus di dalam arena
                if (x_cm < 0 || x_cm > ARENA_WIDTH_REAL_CM ||
                    y_cm < 0 || y_cm > ARENA_HEIGHT_REAL_CM)
                {
                    System.Diagnostics.Debug.WriteLine($"[BLUE GOAL] Out of bounds: ({x_cm}, {y_cm})");
                    return false;
                }

                return true;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[BLUE GOAL] Read error: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Update goal dari deteksi biru (dipanggil di PoseTimer_Tick)
        /// </summary>
        private void UpdateGoalFromBlueDetection()
        {
            if (!enableBlueGoalAutoDetect)
                return;

            // Throttle: jangan baca terlalu sering
            long now = Environment.TickCount64;
            if (now - _lastBlueGoalReadMs < BLUE_GOAL_READ_PERIOD_MS)
                return;
            _lastBlueGoalReadMs = now;

            // Coba baca file goal biru
            if (!TryReadBlueGoal(out float x_cm, out float y_cm))
            {
                // File tidak ada, tidak berubah, atau invalid
                if (lastBlueGoalCm.HasValue)
                {
                    // Goal sebelumnya ada tapi sekarang hilang
                    // Jangan langsung hapus goal - biarkan goal terakhir tetap aktif
                    // Hanya update status
                    //UpdateBlueGoalStatusLabel("Blue Goal: Detection lost (keeping last position)", Color.Orange);
                }
                return;
            }

            // Cek apakah posisi berubah signifikan (> 5cm dari posisi terakhir)
            bool positionChanged = !lastBlueGoalCm.HasValue ||
                Math.Sqrt(Math.Pow(x_cm - lastBlueGoalCm.Value.X, 2) +
                         Math.Pow(y_cm - lastBlueGoalCm.Value.Y, 2)) > 3.0f;

            if (!positionChanged)
            {
                // Posisi sama, tidak perlu update
                return;
            }

            // Simpan posisi baru
            lastBlueGoalCm = new PointF(x_cm, y_cm);

            // Konversi cm ke grid cell
            int gridX = (int)Math.Floor(x_cm / currentGridCellSizeCm);
            int gridY = (int)Math.Floor(y_cm / currentGridCellSizeCm);

            // Clamp ke bounds grid
            gridX = Math.Max(0, Math.Min(gridCols - 1, gridX));
            gridY = Math.Max(0, Math.Min(gridRows - 1, gridY));

            Point newGoalGrid = new Point(gridX, gridY);

            // Validasi: jangan set goal di posisi robot
            if (newGoalGrid.Equals(robotStartGridPos))
            {
                UpdateBlueGoalStatusLabel($"Blue Goal: ({x_cm:F0}, {y_cm:F0}) cm - At robot position!", Color.Orange);
                return;
            }

            // Validasi: jangan set goal di obstacle
            if (IsWithinGridBounds(gridX, gridY) && gridData[gridX, gridY] == CellType.Obstacle)
            {
                UpdateBlueGoalStatusLabel($"Blue Goal: ({x_cm:F0}, {y_cm:F0}) cm - On obstacle!", Color.Orange);
                return;
            }

            // Set goal baru menggunakan method yang sudah ada
            SetGoalFromBlueDetection(gridX, gridY, x_cm, y_cm);

            UpdateBlueGoalStatusLabel($"Blue Goal: ({x_cm:F0}, {y_cm:F0}) cm → Grid({gridX}, {gridY})", Color.Green);

            System.Diagnostics.Debug.WriteLine($"[BLUE GOAL] Auto-set: ({x_cm:F1}, {y_cm:F1}) cm → Grid({gridX}, {gridY})");
        }

        /// <summary>
        /// Set goal dari deteksi biru
        /// </summary>
        private void SetGoalFromBlueDetection(int gridX, int gridY, float xCm, float yCm)
        {
            // Validasi bounds
            if (!IsWithinGridBounds(gridX, gridY))
                return;

            // Jangan set di posisi start
            if (gridX == robotStartGridPos.X && gridY == robotStartGridPos.Y)
                return;

            // Jangan set di obstacle
            if (gridData[gridX, gridY] == CellType.Obstacle)
                return;

            // Hapus goal lama dari grid
            if (currentGoalGridPos.HasValue)
            {
                Point oldGoal = currentGoalGridPos.Value;
                if (IsWithinGridBounds(oldGoal.X, oldGoal.Y) &&
                    gridData[oldGoal.X, oldGoal.Y] == CellType.Goal)
                {
                    gridData[oldGoal.X, oldGoal.Y] = CellType.Free;
                }
            }

            // Clear path lama
            ClearPathAndButton();

            // Set goal baru
            currentGoalGridPos = new Point(gridX, gridY);
            gridData[gridX, gridY] = CellType.Goal;

            // Handle berdasarkan mode
            if (isReactivePFMode)
            {
                // Mode Reactive PF: set goal langsung ke controller
                leaderReactivePF?.SetGoalFromGrid(gridX, gridY, currentGridCellSizeCm);
                UpdateStatusLabel($"🔵 Reactive PF goal (Blue): ({xCm:F1}, {yCm:F1}) cm");
            }
            else
            {
                // Mode path planning: hitung path
                UpdateStatusLabel($"🔵 Goal set from Blue at ({gridX},{gridY}). Calculating path...");
                FindAndDisplayPath();
            }

            // Refresh display
            arenaPictureBox.Invalidate();
        }

        /// <summary>
        /// Update label status blue goal (thread-safe)
        /// </summary>
        private void UpdateBlueGoalStatusLabel(string text, Color color)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => UpdateBlueGoalStatusLabel(text, color)));
                return;
            }

            if (lblBlueGoalStatus != null)
            {
                lblBlueGoalStatus.Text = text;
                lblBlueGoalStatus.ForeColor = color;
            }
        }

        /// <summary>
        /// Gambar indikator blue goal di arena
        /// Panggil di ArenaPictureBox_Paint setelah menggambar grid
        /// </summary>
        private void DrawBlueGoalIndicator(Graphics g)
        {
            if (!enableBlueGoalAutoDetect || !lastBlueGoalCm.HasValue)
                return;

            // Konversi posisi cm ke pixel
            Point pix = WorldToPixelRPF(lastBlueGoalCm.Value.X, lastBlueGoalCm.Value.Y);

            // Gambar lingkaran biru di sekitar goal
            using (Pen bluePen = new Pen(Color.Blue, 3))
            {
                g.DrawEllipse(bluePen, pix.X - 22, pix.Y - 22, 44, 44);
            }

            // Gambar lingkaran dalam (filled)
            using (Brush blueBrush = new SolidBrush(Color.FromArgb(80, 0, 0, 255)))
            {
                g.FillEllipse(blueBrush, pix.X - 18, pix.Y - 18, 36, 36);
            }

            

            // Gambar crosshair
            using (Pen crossPen = new Pen(Color.DarkBlue, 2))
            {
                g.DrawLine(crossPen, pix.X - 15, pix.Y, pix.X + 15, pix.Y);
                g.DrawLine(crossPen, pix.X, pix.Y - 15, pix.X, pix.Y + 15);
            }

            // Gambar koordinat
            using (Font coordFont = new Font("Arial", 8))
            {
                string coordText = $"({lastBlueGoalCm.Value.X:F0}, {lastBlueGoalCm.Value.Y:F0})";
                g.DrawString(coordText, coordFont, Brushes.DarkBlue, pix.X + 25, pix.Y - 8);
            }
        }

        #endregion

        private void EnsureFollowerConnection()
        {
            // Legacy follower2 streaming ONLY.
            // Saat Reactive PF mode aktif, jangan pernah pegang koneksi follower dari MainForm.
            if (isReactivePFMode) return;


            if (!ENABLE_LEGACY_FOLLOWER_STREAM) return;
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

                // optional: aktifkan gerak (legacy mode)
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
            // Clear dari grid
            for (int c = 0; c < gridCols; c++)
            {
                for (int r = 0; r < gridRows; r++)
                {
                    if (gridData[c, r] == CellType.Obstacle)
                    {
                        gridData[c, r] = CellType.Free;
                    }
                }
            }

            // Clear manual obstacles tracking
            manualObstacles.Clear(); // TAMBAH INI!
            System.Diagnostics.Debug.WriteLine("[MANUAL OBSTACLE] All manual obstacles cleared.");

            ClearPathAndButton();

            arenaPictureBox.Invalidate();
            UpdateStatusLabel("Semua obstacle dibersihkan. Silakan set Goal baru atau buat obstacle.");
        }


        private void ClearManualObstaclesOnly()
        {
            // Remove only manual obstacles from grid
            foreach (Point manualObs in manualObstacles)
            {
                if (IsWithinGridBounds(manualObs.X, manualObs.Y) &&
                    gridData[manualObs.X, manualObs.Y] == CellType.Obstacle)
                {
                    gridData[manualObs.X, manualObs.Y] = CellType.Free;
                }
            }

            manualObstacles.Clear();
            System.Diagnostics.Debug.WriteLine("[MANUAL OBSTACLE] Manual obstacles cleared, vision obstacles preserved.");

            arenaPictureBox.Invalidate();
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

                if (telemetryLogger != null && !telemetryLogger.IsLogging)
                {
                    bool logStarted = telemetryLogger.StartLogging(leaderClient);

                    if (logStarted)
                    {
                        // Tambahkan metadata ke log
                        telemetryLogger.AddNote($"Algorithm: {currentMode}");
                        telemetryLogger.AddNote($"Grid size: {currentGridCellSizeCm}cm");
                        telemetryLogger.AddNote($"Waypoints: {calculatedPath.Count}");
                        telemetryLogger.AddNote($"Start pose: ({x_cm:F2}, {y_cm:F2}, {theta_rad:F3} rad)");

                        if (currentGoalGridPos.HasValue)
                        {
                            telemetryLogger.AddNote($"Goal grid: ({currentGoalGridPos.Value.X}, {currentGoalGridPos.Value.Y})");
                        }
                    }
                }

                // 5) Mulai eksekusi
                leaderWriter.WriteLine("EXECUTE");

                await Task.Delay(100); // Tunggu sebentar untuk ensure semua terkirim

                //if (leaderWriter != null)
                //{
                //    leaderWriter.Close();
                //    leaderWriter = null;
                //}
                //if (leaderClient != null)
                //{
                //    leaderClient.Close();
                //    leaderClient = null;
                //}
                UpdateStatusLabel("Misi dikirim. Leader mulai bergerak.");
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error saat mengirim ke robot: {ex.Message}",
                "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                UpdateStatusLabel("❌ Error komunikasi dengan robot.");

                // Stop logging jika error (flag: mission failed)
                if (telemetryLogger != null && telemetryLogger.IsLogging)
                {
                    telemetryLogger.StopLogging(missionSuccess: false);
                }

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
            // Throttle: jangan baca tiap tick
            long now = Environment.TickCount64;
            if (now - _lastObstacleReadMs < OBSTACLE_READ_PERIOD_MS) return;
            _lastObstacleReadMs = now;

            if (!File.Exists(OBSTACLE_FILE_PATH)) return;

            try
            {
                // Lebih ringan dari ReadAllLines (lebih sedikit alloc)
                List<PointF> newObstacles = new List<PointF>();

                foreach (var line in File.ReadLines(OBSTACLE_FILE_PATH))
                {
                    if (string.IsNullOrWhiteSpace(line)) continue;

                    string[] parts = line.Split(',');
                    if (parts.Length == 2 &&
                        float.TryParse(parts[0], NumberStyles.Float, CI, out float xCm) &&
                        float.TryParse(parts[1], NumberStyles.Float, CI, out float yCm))
                    {
                        newObstacles.Add(new PointF(xCm, yCm));
                    }
                }

                // Hanya kalau berubah
                if (!AreObstaclesEqual(currentVisionObstacles, newObstacles))
                {
                    currentVisionObstacles = newObstacles;

                    // Kalau lagi Reactive PF, biasanya cukup update list obstacle saja.
                    // Jangan update grid + replan (ini sumber lag besar).
                    if (!isReactivePFMode)
                    {
                        UpdateGridFromVisionObstacles();

                        // Replan itu berat -> throttle juga (opsional, tapi sangat disarankan)
                        if (currentMode == PathPlanningMode.HybridDijkstraPF &&
                            globalDijkstraPath != null && globalDijkstraPath.Count > 1 &&
                            haveLeaderPose && currentGoalGridPos.HasValue)
                        {
                            ReplanLocalWithPotentialField();
                        }
                    }

                    // Jangan Refresh (blocking). Cukup Invalidate.
                    arenaPictureBox.Invalidate();

                    // Update label jangan tiap event (throttle 5 Hz)
                    if (now - _lastStatusMs > 200)
                    {
                        _lastStatusMs = now;
                        UpdateStatusLabel($"Vision obstacles updated: {newObstacles.Count} detected.");
                    }
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
            const int sendHz = 15; // 10–20 Hz cukup
            var sendDelay = TimeSpan.FromMilliseconds(1000.0 / sendHz);
            var retryDelay = TimeSpan.FromSeconds(1);

            while (!token.IsCancellationRequested)
            {
                // PF mode takeover => stop loop legacy ini dan lepas koneksi
                if (isReactivePFMode)
                {
                    CleanupFollowerConnection();
                    break;
                }

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
                    if (latestLeaderHas && followerWriter != null)
                    {
                        var line = string.Create(CultureInfo.InvariantCulture, $"SETLEADER {latestLeaderX} {latestLeaderY} {latestLeaderTh}");
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
            CleanupReactivePF();

            // Stop telemetry logging jika masih aktif (flag: mission cancelled)
            if (telemetryLogger != null && telemetryLogger.IsLogging)
            {
                telemetryLogger.AddNote("Application closing - mission cancelled");
                telemetryLogger.StopLogging(missionSuccess: false);  // File akan dihapus
            }

            // Cleanup connections
            try { setleaderCts?.Cancel(); } catch { }
            CleanupFollowerConnection();

            try { leaderWriter?.Close(); } catch { }
            try { leaderClient?.Close(); } catch { }

            base.OnFormClosing(e);
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

            // Tambahkan ke manual obstacles tracking
            if (gridData[c, r] == CellType.Free || gridData[c, r] == CellType.PathNode)
            {
                gridData[c, r] = CellType.Obstacle;
                manualObstacles.Add(currentCellPoint); // BARIS BARU!

                System.Diagnostics.Debug.WriteLine($"[MANUAL OBSTACLE] Added at ({c}, {r}). Total: {manualObstacles.Count}");

                if (calculatedPath != null && calculatedPath.Any())
                {
                    ClearPathAndButton();
                }
                arenaPictureBox.Invalidate();
            }
        }

        private void InitializeTelemetryLogger()
        {
            try
            {
                string telemetryLogDir = Path.Combine(PoseDir, "TelemetryLogs");
                telemetryLogger = new TelemetryLogger(telemetryLogDir);

                // Subscribe ke events
                telemetryLogger.OnStatusChanged += (msg) => {
                    if (InvokeRequired)
                        Invoke(new Action(() => UpdateStatusLabel(msg)));
                    else
                        UpdateStatusLabel(msg);
                };

                telemetryLogger.OnTelemetryReceived += (telLine) => {
                    // Optional: log ke debug console
                    System.Diagnostics.Debug.WriteLine($"[TEL] {telLine}");
                };

                // Event untuk auto-stop saat mission selesai
                telemetryLogger.OnMissionCompleted += () => {
                    if (InvokeRequired)
                    {
                        Invoke(new Action(() => {
                            HandleMissionCompleted();
                        }));
                    }
                    else
                    {
                        HandleMissionCompleted();
                    }
                };

                System.Diagnostics.Debug.WriteLine("[TELEMETRY] Logger initialized (auto mode)");
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[TELEMETRY] Init failed: {ex.Message}");
            }
        }

        private void HandleMissionCompleted()
        {
            System.Diagnostics.Debug.WriteLine("[MISSION] ALL_WAYPOINTS_REACHED detected!");

            // Stop logging (flag: mission success)
            if (telemetryLogger != null && telemetryLogger.IsLogging)
            {
                telemetryLogger.StopLogging(missionSuccess: true);
            }

            // Tampilkan notifikasi ke user
            UpdateStatusLabel($"✅ Mission completed! Log saved: Session Test #{telemetryLogger.CurrentTestNumber}");

            // Optional: Tutup koneksi ke robot (atau biarkan terbuka untuk mission berikutnya)
            // CloseLeaderConnection();

            // Optional: Reset untuk goal baru
            // ResetForNewGoal();
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

            // Save grid config to file for vision system to sync
            SaveGridConfigToFile(currentGridCellSizeCm);
            UpdateGridFromVisionObstacles();
            InitializeArenaAndGrid(); // Ini akan memanggil Refresh di akhirnya

            arenaPictureBox.Invalidate(); // Mungkin tidak perlu jika InitializeArenaAndGrid sudah panggil Refresh
            // arenaPictureBox.Refresh();    // Juga tidak perlu jika InitializeArenaAndGrid sudah panggil Refresh
            System.Diagnostics.Debug.WriteLine($"Grid changed. (InitializeArenaAndGrid called Refresh).");
            UpdateStatusLabel($"Ukuran grid diubah menjadi {currentGridCellSizeCm}x{currentGridCellSizeCm} cm. Klik arena untuk set Goal.");
        }

        private void btnUpdateFromCamera_Click(object sender, EventArgs e)
        {
            // Read leader pose from camera (via vision system)
            if (!TryReadPose(LEADER_POSE_PATH, out var lx, out var ly, out var lth))
            {
                MessageBox.Show(
                    "Gagal membaca leader_pose.txt.\nPastikan vision.py sedang berjalan dan file diperbarui.",
                    "Error Membaca File", MessageBoxButtons.OK, MessageBoxIcon.Error);
                UpdateStatusLabel("❌ Gagal membaca pose dari kamera.");
                return;
            }

            if (TryReadPose(LEADER_POSE_PATH, out var x, out var y, out var th))
            {
                leaderWriter.WriteLine($"SETPOS {x.ToString(CI)},{y.ToString(CI)},{th.ToString(CI)}"); // pakai SETPOS_DEG jika perlu
                _leaderSetposSent = true;
            }

            // ====== 1. Update visual grid position (start marker) ======
            float cellCm = currentGridCellSizeCm;
            int gx = (int)Math.Floor(lx / cellCm);
            int gy = (int)Math.Floor(ly / cellCm);

            // Clamp ke batas grid
            gx = Math.Max(0, Math.Min(gridCols - 1, gx));
            gy = Math.Max(0, Math.Min(gridRows - 1, gy));

            // Bersihkan start lama lalu set titik baru
            if (IsWithinGridBounds(robotStartGridPos.X, robotStartGridPos.Y) &&
                gridData[robotStartGridPos.X, robotStartGridPos.Y] == CellType.Start)
            {
                gridData[robotStartGridPos.X, robotStartGridPos.Y] = CellType.Free;
            }
            robotStartGridPos = new Point(gx, gy);
            gridData[gx, gy] = CellType.Start;

            // Update global leader pose variables
            leaderX = lx;
            leaderY = ly;
            leaderTh = lth;
            latestLeaderX = lx;
            latestLeaderY = ly;
            latestLeaderTh = lth;
            latestLeaderHas = true;

            // Refresh visual
            arenaPictureBox.Invalidate();

            // ====== 2. Send SETPOS to LEADER robot (update odometry) ======
            try
            {
                EnsureLeaderConnection();

                if (leaderWriter != null)
                {
                    // Send SETPOS command with radian (same as startup)
                    string setposCmd = $"SETPOS {lx.ToString(CultureInfo.InvariantCulture)} " +
                                      $"{ly.ToString(CultureInfo.InvariantCulture)} " +
                                      $"{lth.ToString(CultureInfo.InvariantCulture)}";

                    leaderWriter.WriteLine(setposCmd);
                    leaderWriter.Flush();

                    System.Diagnostics.Debug.WriteLine($"[UPDATE BTN] Sent to leader: {setposCmd}");

                    // Success feedback
                    UpdateStatusLabel($"✅ Odometry leader diupdate: ({lx:F1}, {ly:F1}, {lth:F3} rad)");
                    MessageBox.Show(
                        $"Odometry robot leader berhasil diupdate dari kamera!\n\n" +
                        $"Posisi: ({lx:F2} cm, {ly:F2} cm)\n" +
                        $"Orientasi: {lth:F3} rad ({lth * 180 / Math.PI:F1}°)",
                        "Update Sukses",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Information);
                }
                else
                {
                    throw new Exception("Leader connection not established");
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[UPDATE BTN] Error sending SETPOS to leader: {ex.Message}");
                UpdateStatusLabel("❌ Gagal mengirim SETPOS ke leader.");
                MessageBox.Show(
                    $"Gagal mengirim SETPOS ke robot leader.\n\n" +
                    $"Error: {ex.Message}\n\n" +
                    $"Pastikan:\n" +
                    $"1. Robot leader terhubung ke WiFi\n" +
                    $"2. IP address benar: {ROBOT_IP_ADDRESS}:{ROBOT_PORT}\n" +
                    $"3. Firmware leader sedang berjalan",
                    "Error Koneksi",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error);
            }
        }

        private void ReplanLocalWithPotentialField()
        {
            if (currentMode != PathPlanningMode.HybridDijkstraPF)
                return;
            if (globalDijkstraPath == null || globalDijkstraPath.Count < 2)
                return;
            if (!haveLeaderPose)
                return;

            // 1) SELALU ambil grid terbaru yang sudah ter-inflate + vision obstacles
            var pathfindingGrid = CreateInflatedGridForPathfinding();

            // 2) Rebuild pfPlanner agar pakai grid terbaru
            pfPlanner = new PotentialFieldPlanner(
                pathfindingGrid,
                gridCols,
                gridRows,
                currentGridCellSizeCm
            );

            // 3) Posisi robot sekarang (cm -> grid)
            int curX = (int)Math.Floor(leaderX / currentGridCellSizeCm);
            int curY = (int)Math.Floor(leaderY / currentGridCellSizeCm);
            Point currentCell = new Point(
                Math.Max(0, Math.Min(gridCols - 1, curX)),
                Math.Max(0, Math.Min(gridRows - 1, curY))
            );

            // 4) Cari index path global yang paling dekat
            int nearestIdx = 0;
            float bestDistSq = float.MaxValue;
            for (int i = 0; i < globalDijkstraPath.Count; i++)
            {
                var p = globalDijkstraPath[i];
                float dx = p.X - currentCell.X;
                float dy = p.Y - currentCell.Y;
                float d2 = dx * dx + dy * dy;
                if (d2 < bestDistSq)
                {
                    bestDistSq = d2;
                    nearestIdx = i;
                }
            }

            // 5) Sub-goal beberapa langkah di depan
            int lookahead = 4;
            int goalIdx = Math.Min(globalDijkstraPath.Count - 1, nearestIdx + lookahead);
            Point localGoalCell = globalDijkstraPath[goalIdx];

            // 6) PF lokal: currentCell -> localGoalCell
            List<Point> localPfPath = pfPlanner.FindPath(currentCell, localGoalCell);
            if (localPfPath == null || localPfPath.Count < 2)
                return;

            // 7) Smooth + convert ke grid
            List<PointF> localCmPath = pfPlanner.SmoothPath(localPfPath, currentVisionObstacles);
            List<Point> localGridPath = ConvertCmPathToGrid(localCmPath);

            // 8) Jahit ke global path
            var newGlobal = new List<Point>();
            for (int i = 0; i < nearestIdx; i++)
                newGlobal.Add(globalDijkstraPath[i]);

            foreach (var p in localGridPath)
            {
                if (newGlobal.Count == 0 || !newGlobal[newGlobal.Count - 1].Equals(p))
                    newGlobal.Add(p);
            }

            for (int i = goalIdx + 1; i < globalDijkstraPath.Count; i++)
                newGlobal.Add(globalDijkstraPath[i]);

            globalDijkstraPath = newGlobal;
            calculatedPath = newGlobal;   // ini yang kelihatan di GUI & siap dikirim ke robot

            System.Diagnostics.Debug.WriteLine($"[LOCAL PF] Replan, new path length = {newGlobal.Count}");
        }



        private void Button_MouseEnter(object sender, EventArgs e)
        {
            if (sender is Button btn)
            {
                // Simpan warna asli jika belum ada
                if (btn.Tag == null)
                {
                    btn.Tag = btn.BackColor;
                }

                // Buat warna lebih terang saat hover
                btn.BackColor = LightenColor(btn.BackColor, 20);
            }
        }

        private void Button_MouseLeave(object sender, EventArgs e)
        {
            if (sender is Button btn && btn.Tag is Color originalColor)
            {
                // Kembalikan ke warna asli
                btn.BackColor = originalColor;
            }
        }

        private Color LightenColor(Color color, int amount)
        {
            return Color.FromArgb(
                Math.Min(255, color.R + amount),
                Math.Min(255, color.G + amount),
                Math.Min(255, color.B + amount)
            );
        }

        // Method untuk update status label yang sudah ada
        private void UpdateStatusLabel(string message)
        {
            if (InvokeRequired)
            {
                Invoke(new Action<string>(UpdateStatusLabel), message);
                return;
            }

            // Update status strip label instead
            if (statusStripLabel != null)
            {
                // Tambahkan icon sesuai jenis pesan
                string icon = "ℹ️";
                if (message.Contains("Error") || message.Contains("Gagal"))
                    icon = "❌";
                else if (message.Contains("Success") || message.Contains("Selesai"))
                    icon = "✓";
                else if (message.Contains("Warning") || message.Contains("Peringatan"))
                    icon = "⚠️";
                else if (message.Contains("Robot") || message.Contains("Kirim"))
                    icon = "🤖";

                statusStripLabel.Text = $"{icon} {message}";
            }

            // Keep the old statusLabel updated for backward compatibility
            if (statusLabel != null)
            {
                statusLabel.Text = message;
            }
        }

        private List<Point> RemoveUnsafeDiagonals(List<Point> path, CellType[,] grid)
        {
            if (path == null || path.Count < 2) return path;

            List<Point> safePath = new List<Point>();
            safePath.Add(path[0]); // Start point

            for (int i = 1; i < path.Count; i++)
            {
                Point prev = safePath[safePath.Count - 1];
                Point current = path[i];

                bool isDiagonal = (prev.X != current.X && prev.Y != current.Y);

                if (isDiagonal)
                {
                    // Cek apakah ada obstacle di sekitar diagonal
                    bool hasNearbyObstacle = false;

                    // Cek 3x3 area di sekitar kedua titik
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        for (int dy = -1; dy <= 1; dy++)
                        {
                            int checkX1 = prev.X + dx;
                            int checkY1 = prev.Y + dy;
                            int checkX2 = current.X + dx;
                            int checkY2 = current.Y + dy;

                            if (IsWithinGridBounds(checkX1, checkY1) &&
                                grid[checkX1, checkY1] == CellType.Obstacle)
                            {
                                hasNearbyObstacle = true;
                                break;
                            }

                            if (IsWithinGridBounds(checkX2, checkY2) &&
                                grid[checkX2, checkY2] == CellType.Obstacle)
                            {
                                hasNearbyObstacle = true;
                                break;
                            }
                        }
                        if (hasNearbyObstacle) break;
                    }

                    if (hasNearbyObstacle)
                    {
                        // Ganti diagonal dengan 2 langkah orthogonal (L-shape)
                        // Pilih jalur yang lebih aman
                        Point intermediate1 = new Point(prev.X, current.Y);
                        Point intermediate2 = new Point(current.X, prev.Y);

                        bool route1Safe = IsWithinGridBounds(intermediate1.X, intermediate1.Y) &&
                                          grid[intermediate1.X, intermediate1.Y] != CellType.Obstacle;
                        bool route2Safe = IsWithinGridBounds(intermediate2.X, intermediate2.Y) &&
                                          grid[intermediate2.X, intermediate2.Y] != CellType.Obstacle;

                        if (route1Safe)
                        {
                            safePath.Add(intermediate1);
                        }
                        else if (route2Safe)
                        {
                            safePath.Add(intermediate2);
                        }
                        // Jika keduanya tidak aman, tetap gunakan diagonal (fallback)
                    }
                }

                safePath.Add(current);
            }

            return safePath;
        }
        private List<Point> ConvertCmPathToGrid(List<PointF> pathCm)
        {
            if (pathCm == null || pathCm.Count == 0)
                return new List<Point>();

            List<Point> gridPath = new List<Point>();

            System.Diagnostics.Debug.WriteLine($"\n[ConvertCmToGrid] === CONVERSION START ===");
            System.Diagnostics.Debug.WriteLine($"Input: {pathCm.Count} cm waypoints");
            System.Diagnostics.Debug.WriteLine($"Cell size: {currentGridCellSizeCm} cm");

            // Convert first waypoint
            PointF firstCm = pathCm[0];
            int firstX = (int)Math.Floor(firstCm.X / currentGridCellSizeCm);
            int firstY = (int)Math.Floor(firstCm.Y / currentGridCellSizeCm);
            firstX = Math.Max(0, Math.Min(gridCols - 1, firstX));
            firstY = Math.Max(0, Math.Min(gridRows - 1, firstY));

            Point lastGridPoint = new Point(firstX, firstY);
            gridPath.Add(lastGridPoint);

            System.Diagnostics.Debug.WriteLine($"  [0] Cm({firstCm.X:F1}, {firstCm.Y:F1}) -> Grid({firstX}, {firstY})");

            // Process each subsequent waypoint
            for (int i = 1; i < pathCm.Count; i++)
            {
                PointF currentCm = pathCm[i];

                // Convert to grid coordinates
                int currentX = (int)Math.Floor(currentCm.X / currentGridCellSizeCm);
                int currentY = (int)Math.Floor(currentCm.Y / currentGridCellSizeCm);
                currentX = Math.Max(0, Math.Min(gridCols - 1, currentX));
                currentY = Math.Max(0, Math.Min(gridRows - 1, currentY));

                Point currentGridPoint = new Point(currentX, currentY);

                System.Diagnostics.Debug.WriteLine($"  [{i}] Cm({currentCm.X:F1}, {currentCm.Y:F1}) -> Grid({currentX}, {currentY})");

                // CRITICAL: Check if there's a gap between last and current waypoint
                int dx = Math.Abs(currentX - lastGridPoint.X);
                int dy = Math.Abs(currentY - lastGridPoint.Y);
                int maxDist = Math.Max(dx, dy);

                if (maxDist > 1)
                {
                    // GAP DETECTED! Need interpolation
                    System.Diagnostics.Debug.WriteLine($"      ⚠️ GAP detected: distance = {maxDist} cells");
                    System.Diagnostics.Debug.WriteLine($"      → Interpolating from ({lastGridPoint.X},{lastGridPoint.Y}) to ({currentX},{currentY})");

                    // Use Bresenham's line algorithm to fill the gap
                    List<Point> interpolated = BresenhamLine(lastGridPoint, currentGridPoint);

                    // Add interpolated points (skip first as it's already in path)
                    for (int j = 1; j < interpolated.Count; j++)
                    {
                        Point interpPoint = interpolated[j];

                        // Safety check: skip if obstacle
                        if (IsWithinGridBounds(interpPoint.X, interpPoint.Y) &&
                            gridData[interpPoint.X, interpPoint.Y] != CellType.Obstacle)
                        {
                            gridPath.Add(interpPoint);
                            System.Diagnostics.Debug.WriteLine($"        + Interpolated: ({interpPoint.X}, {interpPoint.Y})");
                        }
                        else
                        {
                            System.Diagnostics.Debug.WriteLine($"        ✗ Skipped obstacle at ({interpPoint.X}, {interpPoint.Y})");
                        }
                    }

                    lastGridPoint = currentGridPoint;
                }
                else if (currentGridPoint != lastGridPoint)
                {
                    // No gap, adjacent cells
                    if (IsWithinGridBounds(currentX, currentY) &&
                        gridData[currentX, currentY] != CellType.Obstacle)
                    {
                        gridPath.Add(currentGridPoint);
                        lastGridPoint = currentGridPoint;
                        System.Diagnostics.Debug.WriteLine($"      ✓ Added (adjacent)");
                    }
                    else
                    {
                        System.Diagnostics.Debug.WriteLine($"      ✗ Skipped (obstacle)");
                    }
                }
                else
                {
                    System.Diagnostics.Debug.WriteLine($"      ↔ Duplicate, skipped");
                }
            }

            System.Diagnostics.Debug.WriteLine($"[ConvertCmToGrid] === RESULT: {gridPath.Count} waypoints ===\n");
            return gridPath;
        }

        private List<Point> BresenhamLine(Point from, Point to)
        {
            List<Point> points = new List<Point>();

            int x0 = from.X, y0 = from.Y;
            int x1 = to.X, y1 = to.Y;

            int dx = Math.Abs(x1 - x0);
            int dy = Math.Abs(y1 - y0);

            int sx = x0 < x1 ? 1 : -1;
            int sy = y0 < y1 ? 1 : -1;

            int err = dx - dy;

            while (true)
            {
                points.Add(new Point(x0, y0));

                if (x0 == x1 && y0 == y1)
                    break;

                int e2 = 2 * err;

                if (e2 > -dy)
                {
                    err -= dy;
                    x0 += sx;
                }

                if (e2 < dx)
                {
                    err += dx;
                    y0 += sy;
                }
            }

            return points;
        }

        private float CalculatePathLength(List<Point> path)
        {
            if (path == null || path.Count < 2)
                return 0;

            float totalLength = 0;
            for (int i = 1; i < path.Count; i++)
            {
                float dx = (path[i].X - path[i - 1].X) * currentGridCellSizeCm;
                float dy = (path[i].Y - path[i - 1].Y) * currentGridCellSizeCm;
                totalLength += (float)Math.Sqrt(dx * dx + dy * dy);
            }

            return totalLength;
        }

        private float CalculatePathSmoothness(List<Point> path)
        {
            if (path == null || path.Count < 3)
                return 0;

            float totalAngleChange = 0;

            for (int i = 1; i < path.Count - 1; i++)
            {
                // Vector 1: from path[i-1] to path[i]
                float v1x = path[i].X - path[i - 1].X;
                float v1y = path[i].Y - path[i - 1].Y;

                // Vector 2: from path[i] to path[i+1]
                float v2x = path[i + 1].X - path[i].X;
                float v2y = path[i + 1].Y - path[i].Y;

                // Calculate angle between vectors
                float dot = v1x * v2x + v1y * v2y;
                float mag1 = (float)Math.Sqrt(v1x * v1x + v1y * v1y);
                float mag2 = (float)Math.Sqrt(v2x * v2x + v2y * v2y);

                if (mag1 > 0.001f && mag2 > 0.001f)
                {
                    float cosAngle = dot / (mag1 * mag2);
                    cosAngle = Math.Max(-1.0f, Math.Min(1.0f, cosAngle)); // Clamp
                    float angle = (float)Math.Acos(cosAngle);
                    totalAngleChange += angle;
                }
            }

            // Return average angle change (lower = smoother)
            return totalAngleChange / (path.Count - 2);
        }

        private void InitializePathPlanningUI()
        {
            // Contoh: tambahkan combo box
            ComboBox algorithmSelector = new ComboBox();
            algorithmSelector.Items.AddRange(new object[] {
                "Dijkstra Only",
                "Hybrid (Dijkstra + PF)", // Default
                "Pure PSO",
                "Pure Potential Field"
            });
            algorithmSelector.SelectedIndex = 1; // Hybrid default
            algorithmSelector.SelectedIndexChanged += (s, e) =>
            {
                currentMode = (PathPlanningMode)algorithmSelector.SelectedIndex;

                // Re-calculate path if goal already set
                if (currentGoalGridPos.HasValue)
                {
                    FindAndDisplayPath();
                }
            };

            // Add to form...
        }

        private void RunComparisonStudy()
        {
            if (!currentGoalGridPos.HasValue)
            {
                MessageBox.Show("Set goal position first!");
                return;
            }

            var results = new Dictionary<string, PathMetrics>();
            CellType[,] pathfindingGrid = CreateInflatedGridForPathfinding();

            // Test 1: Dijkstra
            pathPlanningStopwatch.Restart();
            var dijkstraPath = DijkstraPathfinderNoDiagObst.FindPath(
                pathfindingGrid, robotStartGridPos, currentGoalGridPos.Value, gridCols, gridRows
            );
            pathPlanningStopwatch.Stop();
            results["Dijkstra"] = new PathMetrics
            {
                Path = dijkstraPath,
                ComputationTimeMs = pathPlanningStopwatch.ElapsedMilliseconds,
                PathLength = CalculatePathLength(dijkstraPath),
                Smoothness = CalculatePathSmoothness(dijkstraPath)
            };

            // Test 2: PSO
            psoPlanner = new PSOPathPlanner(pathfindingGrid, gridCols, gridRows, currentGridCellSizeCm);
            pathPlanningStopwatch.Restart();
            var psoPath = psoPlanner.FindPath(robotStartGridPos, currentGoalGridPos.Value);
            pathPlanningStopwatch.Stop();
            results["PSO"] = new PathMetrics
            {
                Path = psoPath,
                ComputationTimeMs = pathPlanningStopwatch.ElapsedMilliseconds,
                PathLength = CalculatePathLength(psoPath),
                Smoothness = CalculatePathSmoothness(psoPath)
            };

            // Test 3: Potential Field
            pfPlanner = new PotentialFieldPlanner(pathfindingGrid, gridCols, gridRows, currentGridCellSizeCm);
            pathPlanningStopwatch.Restart();
            var pfPath = pfPlanner.FindPath(robotStartGridPos, currentGoalGridPos.Value);
            pathPlanningStopwatch.Stop();
            results["Potential Field"] = new PathMetrics
            {
                Path = pfPath,
                ComputationTimeMs = pathPlanningStopwatch.ElapsedMilliseconds,
                PathLength = CalculatePathLength(pfPath),
                Smoothness = CalculatePathSmoothness(pfPath)
            };

            // Display results
            DisplayComparisonResults(results);
        }

        private class PathMetrics
        {
            public List<Point> Path;
            public long ComputationTimeMs;
            public float PathLength;
            public float Smoothness;
        }

        // === PATH VALIDATION HELPER ===
        // Pastikan path berangkat dari start, berakhir di goal,
        // dan tidak melewati obstacle (menggunakan grid yang sudah di-inflate).
        private bool IsPathValid(List<Point> path, CellType[,] grid, Point start, Point goal)
        {
            if (path == null || path.Count < 2)
                return false;

            // Start & goal harus cocok
            if (path[0] != start)
                return false;

            if (path[path.Count - 1] != goal)
                return false;

            // Setiap waypoint harus di dalam grid dan bukan obstacle
            foreach (Point p in path)
            {
                if (!IsWithinGridBounds(p.X, p.Y))
                    return false;

                if (grid[p.X, p.Y] == CellType.Obstacle)
                    return false;
            }

            // Cek setiap segmen apakah melintasi obstacle
            for (int i = 1; i < path.Count; i++)
            {
                if (SegmentHitsObstacle(path[i - 1], path[i], grid))
                    return false;
            }

            return true;
        }

        // Bresenham line. Return true kalau ada obstacle yang dilewati segmen.
        private bool SegmentHitsObstacle(Point from, Point to, CellType[,] grid)
        {
            int dx = Math.Abs(to.X - from.X);
            int dy = Math.Abs(to.Y - from.Y);
            int x = from.X;
            int y = from.Y;
            int n = 1 + dx + dy;
            int x_inc = (to.X > from.X) ? 1 : -1;
            int y_inc = (to.Y > from.Y) ? 1 : -1;
            int error = dx - dy;
            dx *= 2;
            dy *= 2;

            for (; n > 0; --n)
            {
                if (!IsWithinGridBounds(x, y))
                    return true; // keluar arena = dianggap tabrak obstacle

                if (grid[x, y] == CellType.Obstacle)
                    return true;

                if (error > 0)
                {
                    x += x_inc;
                    error -= dy;
                }
                else
                {
                    y += y_inc;
                    error += dx;
                }
            }

            return false;
        }


        private void DisplayComparisonResults(Dictionary<string, PathMetrics> results)
        {
            string report = "=== PATH PLANNING COMPARISON ===\n\n";

            foreach (var kvp in results)
            {
                string algoName = kvp.Key;
                PathMetrics metrics = kvp.Value;

                report += $"{algoName}:\n";
                report += $"  Waypoints: {metrics.Path?.Count ?? 0}\n";
                report += $"  Path Length: {metrics.PathLength:F2} cm\n";
                report += $"  Smoothness: {metrics.Smoothness:F3} rad\n";
                report += $"  Computation: {metrics.ComputationTimeMs} ms\n";
                report += $"  Success: {(metrics.Path != null ? "Yes" : "No")}\n\n";
            }

            MessageBox.Show(report, "Algorithm Comparison");

            // Optionally save to file for paper
            System.IO.File.WriteAllText("comparison_results.txt", report);
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
            if (ENABLE_LEGACY_FOLLOWER_STREAM && !isReactivePFMode)
            {
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

        #region REACTIVE POTENTIAL FIELD METHODS

        private void InitializeReactivePF()
        {
            System.Diagnostics.Debug.WriteLine("[REACTIVE PF] Initializing...");

            // Dispose existing controllers (if any)
            try { leaderReactivePF?.Dispose(); } catch { }
            try { followerReactivePF?.Dispose(); } catch { }

            // Initialize leader controller
            leaderReactivePF = new ReactivePotentialFieldController();
            leaderReactivePF.SetArenaSize(ARENA_WIDTH_REAL_CM, ARENA_HEIGHT_REAL_CM);

            // *** PENTING: Set callback untuk kirim command via leaderWriter ***
            leaderReactivePF.SendCommandCallback = (cmd) => {
                try
                {
                    // Pastikan koneksi ada
                    EnsureLeaderConnection();

                    if (leaderWriter != null)
                    {
                        leaderWriter.WriteLine(cmd);
                        leaderWriter.Flush();
                        System.Diagnostics.Debug.WriteLine($"[RPF TX] {cmd}");
                    }
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"[RPF TX ERROR] {ex.Message}");
                }
            };

            // Subscribe events
            leaderReactivePF.OnStatusChanged += (msg) => {
                if (InvokeRequired)
                    Invoke(new Action(() => UpdateReactivePFStatus(msg)));
                else
                    UpdateReactivePFStatus(msg);
            };

            leaderReactivePF.OnVelocityCommand += (v, w, fx, fy) => {
                lastForceVector = new PointF(fx, fy);

                // === PF logging row ===
                LogPFRow(v, w, fx, fy);

                if (InvokeRequired)
                    Invoke(new Action(() => arenaPictureBox.Invalidate()));
                else
                    arenaPictureBox.Invalidate();
            };


            leaderReactivePF.OnGoalReached += () => {
                if (InvokeRequired)
                    Invoke(new Action(() => HandleReactivePFGoalReached()));
                else
                    HandleReactivePFGoalReached();
            };

            leaderReactivePF.OnDebugMessage += (msg) => {
                System.Diagnostics.Debug.WriteLine($"[RPF] {msg}");
            };

            // === INIT FOLLOWER REACTIVE PF ===
            followerReactivePF = new FollowerReactivePFController(FOLLOWER_IP, FOLLOWER_PORT);
            followerReactivePF.OnDebugMessage += (m) => System.Diagnostics.Debug.WriteLine("[FRPF] " + m);
            followerReactivePF.OnStatusChanged += (m) => System.Diagnostics.Debug.WriteLine("[FRPF] " + m);
            followerReactivePF.OnVelocityCommand += (v, w) =>
                System.Diagnostics.Debug.WriteLine($"[FRPF CMD] v={v:F2} w={w:F3}");

            // Create UI
            CreateReactivePFUI();

            System.Diagnostics.Debug.WriteLine("[REACTIVE PF] Initialized successfully");
        }


        private void CreateReactivePFUI()
        {
            // ===== CONTROL GROUP =====
            reactivePFGroupBox = new GroupBox
            {
                Text = "🎯 Reactive Potential Field Control",
                Location = new Point(10, 450),
                Size = new Size(280, 130),
                Visible = false,
                Font = new Font("Segoe UI", 9, FontStyle.Bold)
            };

            btnStartReactivePF = new Button
            {
                Text = "▶ START",
                Location = new Point(10, 25),
                Size = new Size(100, 35),
                BackColor = Color.LightGreen,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("Segoe UI", 9, FontStyle.Bold)
            };
            btnStartReactivePF.Click += BtnStartReactivePF_Click;

            btnStopReactivePF = new Button
            {
                Text = "■ STOP",
                Location = new Point(120, 25),
                Size = new Size(80, 35),
                BackColor = Color.LightCoral,
                FlatStyle = FlatStyle.Flat,
                Enabled = false,
                Font = new Font("Segoe UI", 9, FontStyle.Bold)
            };
            btnStopReactivePF.Click += BtnStopReactivePF_Click;

            lblReactivePFStatus = new Label
            {
                Text = "Status: Idle - Set goal lalu klik START",
                Location = new Point(10, 65),
                Size = new Size(260, 20),
                ForeColor = Color.DarkBlue
            };

            lblForceVisualization = new Label
            {
                Text = "Force: (0.0, 0.0)",
                Location = new Point(10, 85),
                Size = new Size(260, 20),
                ForeColor = Color.DarkGreen
            };

            CheckBox chkShowForce = new CheckBox
            {
                //Text = "Show Force Vector",
                Location = new Point(10, 105),
                Size = new Size(150, 20),
                Checked = true
            };
            chkShowForce.CheckedChanged += (s, e) => {
                showForceVisualization = chkShowForce.Checked;
                arenaPictureBox.Invalidate();
            };



            reactivePFGroupBox.Controls.AddRange(new Control[] {
                btnStartReactivePF, btnStopReactivePF,
                lblReactivePFStatus, lblForceVisualization, chkShowForce
            });

            // ===== TUNING GROUP =====
            tuningGroupBox = new GroupBox
            {
                Text = "⚙ PF Tuning Parameters",
                Location = new Point(10, 590),
                Size = new Size(280, 160),
                Visible = false,
                Font = new Font("Segoe UI", 9, FontStyle.Bold)
            };

            // Attractive Gain
            Label lblAttrTitle = new Label { Text = "Attractive Gain:", Location = new Point(10, 22), Size = new Size(100, 18), Font = new Font("Segoe UI", 8) };
            trackAttractiveGain = new TrackBar
            {
                Location = new Point(10, 40),
                Size = new Size(180, 30),
                Minimum = 5,
                Maximum = 50,
                Value = 12,
                TickFrequency = 5
            };
            lblAttractiveGain = new Label { Text = "1.2", Location = new Point(195, 45), Size = new Size(50, 18), Font = new Font("Segoe UI", 9, FontStyle.Bold), ForeColor = Color.DarkBlue };
            trackAttractiveGain.ValueChanged += (s, e) => {
                float val = trackAttractiveGain.Value / 10.0f;
                lblAttractiveGain.Text = val.ToString("F1");
                if (leaderReactivePF != null) leaderReactivePF.AttractiveGain = val;
            };

            // Repulsive Gain
            Label lblRepTitle = new Label { Text = "Repulsive Gain:", Location = new Point(10, 70), Size = new Size(100, 18), Font = new Font("Segoe UI", 8) };
            trackRepulsiveGain = new TrackBar
            {
                Location = new Point(10, 88),
                Size = new Size(180, 30),
                Minimum = 200,
                Maximum = 2000,
                Value = 800,
                TickFrequency = 200
            };
            lblRepulsiveGain = new Label { Text = "800", Location = new Point(195, 93), Size = new Size(50, 18), Font = new Font("Segoe UI", 9, FontStyle.Bold), ForeColor = Color.DarkRed };
            trackRepulsiveGain.ValueChanged += (s, e) => {
                lblRepulsiveGain.Text = trackRepulsiveGain.Value.ToString();
                if (leaderReactivePF != null) leaderReactivePF.RepulsiveGain = trackRepulsiveGain.Value;
            };

            // Obstacle Range
            Label lblRangeTitle = new Label { Text = "Obstacle Range (cm):", Location = new Point(10, 118), Size = new Size(130, 18), Font = new Font("Segoe UI", 8) };
            trackObstacleRange = new TrackBar
            {
                Location = new Point(10, 133),
                Size = new Size(180, 30),
                Minimum = 15,
                Maximum = 80,
                Value = 35,
                TickFrequency = 10
            };
            lblObstacleRange = new Label { Text = "35", Location = new Point(195, 138), Size = new Size(50, 18), Font = new Font("Segoe UI", 9, FontStyle.Bold), ForeColor = Color.DarkOrange };
            trackObstacleRange.ValueChanged += (s, e) => {
                lblObstacleRange.Text = trackObstacleRange.Value.ToString();
                if (leaderReactivePF != null) leaderReactivePF.ObstacleInfluenceRange = trackObstacleRange.Value;
            };

            tuningGroupBox.Controls.AddRange(new Control[] {
                lblAttrTitle, trackAttractiveGain, lblAttractiveGain,
                lblRepTitle, trackRepulsiveGain, lblRepulsiveGain,
                lblRangeTitle, trackObstacleRange, lblObstacleRange
            });

            // Add to form
            this.Controls.Add(reactivePFGroupBox);
            this.Controls.Add(tuningGroupBox);

            // Bring to front
            reactivePFGroupBox.BringToFront();
            tuningGroupBox.BringToFront();
        }

        private void BtnStartReactivePF_Click(object sender, EventArgs e)
        {
            if (!currentGoalGridPos.HasValue)
            {
                MessageBox.Show("Silakan set Goal terlebih dahulu dengan mengklik arena.",
                    "Goal Belum Diset", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            // TAKEOVER MODE DULU (biar PoseTimer dan streamer legacy berhenti kirim ke follower)
            isReactivePFMode = true;
            setposCounter = 0;

            // Stop streamer legacy & lepas koneksi follower (biar follower PF bisa connect)
            try { setleaderCts?.Cancel(); } catch { }
            CleanupFollowerConnection();

            // Safety: pastikan controller sudah di-init
            if (leaderReactivePF == null || followerReactivePF == null)
                InitializeReactivePF();

            // Set goal ke controller (leader)
            leaderReactivePF.SetGoalFromGrid(
                currentGoalGridPos.Value.X,
                currentGoalGridPos.Value.Y,
                currentGridCellSizeCm
            );

            // Update obstacles untuk leader & follower
            var obs = currentVisionObstacles ?? new List<PointF>();
            leaderReactivePF.UpdateObstacles(obs);
            followerReactivePF?.UpdateObstacles(obs);

            // Start follower PF dulu
            followerReactivePF?.StartControl();

            // Kirim START ke leader (firmware) jika diperlukan
            try
            {
                EnsureLeaderConnection();
                if (leaderWriter != null)
                {
                    leaderWriter.WriteLine("START");
                    leaderWriter.Flush();
                    System.Diagnostics.Debug.WriteLine("[RPF] Sent START command to leader");
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[RPF] Error sending START: {ex.Message}");
            }
            StartNewPFScenarioLogging();

            // Start leader PF loop
            leaderReactivePF.StartControl();

            // UI update
            btnStartReactivePF.Enabled = false;
            btnStopReactivePF.Enabled = true;
            toggleObstacleModeButton.Enabled = false;
            clearObstaclesButton.Enabled = false;
            changeGridButton.Enabled = false;

            UpdateStatusLabel("🚀 Reactive PF Control STARTED");
        }


        private void BtnStopReactivePF_Click(object sender, EventArgs e)
        {
            // Stop controllers
            try { leaderReactivePF?.StopControl(); } catch { }
            try { followerReactivePF?.StopControl(); } catch { }
            StopPFScenarioLogging();

            // Kirim STOP tambahan via leaderWriter
            try
            {
                if (leaderWriter != null)
                {
                    leaderWriter.WriteLine("SETVEL 0 0");
                    leaderWriter.Flush();
                }
            }
            catch { }

            // Kembali ke mode legacy
            isReactivePFMode = false;
            setposCounter = 0;
            // Restart streamer legacy (opsional)
            if (ENABLE_LEGACY_FOLLOWER_STREAM)
            {
                try
                {
                    if (setleaderCts == null || setleaderCts.IsCancellationRequested)
                    {
                        setleaderCts = new CancellationTokenSource();
                        _ = Task.Run(() => SetleaderSenderLoop(setleaderCts.Token));
                    }
                }
                catch { }
            }

            btnStartReactivePF.Enabled = true;
            btnStopReactivePF.Enabled = false;
            toggleObstacleModeButton.Enabled = true;
            clearObstaclesButton.Enabled = true;
            changeGridButton.Enabled = true;

            arenaPictureBox.Invalidate();
            UpdateStatusLabel("⏹ Reactive PF Control STOPPED");
        }

        // ============================================================
        // PF SCENARIO LOGGING
        // ============================================================

        private void StartNewPFScenarioLogging()
        {
            try
            {
                Directory.CreateDirectory(PoseDir);

                string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string baseDir = Path.Combine(PoseDir, $"scenario_{ts}");
                string dir = baseDir;

                int idx = 1;
                while (Directory.Exists(dir))
                {
                    dir = $"{baseDir}_{idx:00}";
                    idx++;
                }

                Directory.CreateDirectory(dir);
                _pfScenarioDir = dir;

                string csvPath = Path.Combine(_pfScenarioDir, "pf_log.csv");

                lock (_pfLogLock)
                {
                    _pfLogWriter?.Dispose();
                    _pfLogWriter = new StreamWriter(csvPath, false, new UTF8Encoding(false)) { AutoFlush = false };
                    _pfLogRowCount = 0;
                    _pfLogSw = System.Diagnostics.Stopwatch.StartNew();

                    _pfLogWriter.WriteLine(
                        "t_ms," +
                        "leader_x_cm,leader_y_cm,leader_th_rad," +
                        "follower_x_cm,follower_y_cm,follower_th_rad," +
                        "goal_x_cm,goal_y_cm," +
                        "fx,fy,v_cmd,w_cmd," +
                        "dist_goal_cm," +
                        "obst_count,min_obst_dist_cm," +
                        "k_att,k_rep,obst_range_cm"
                    );
                }

                WriteScenarioMetadata();
                SnapshotScenarioFiles("start");

                System.Diagnostics.Debug.WriteLine($"[PF LOG] Started: {csvPath}");
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[PF LOG] Start error: {ex.Message}");
            }
        }

        private void StopPFScenarioLogging()
        {
            try
            {
                SnapshotScenarioFiles("stop");

                lock (_pfLogLock)
                {
                    _pfLogWriter?.Flush();
                    _pfLogWriter?.Dispose();
                    _pfLogWriter = null;

                    _pfLogSw?.Stop();
                    _pfLogSw = null;
                    _pfLogRowCount = 0;
                }

                System.Diagnostics.Debug.WriteLine($"[PF LOG] Stopped. Folder: {_pfScenarioDir}");
                _pfScenarioDir = null;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[PF LOG] Stop error: {ex.Message}");
            }
        }

        private void WriteScenarioMetadata()
        {
            try
            {
                if (leaderReactivePF == null) return;
                if (string.IsNullOrEmpty(_pfScenarioDir)) return;

                PointF goal = leaderReactivePF.HasGoal ? leaderReactivePF.GetGoalPosition() : new PointF(float.NaN, float.NaN);

                string metaPath = Path.Combine(_pfScenarioDir, "params.txt");
                using (var sw = new StreamWriter(metaPath, false, new UTF8Encoding(false)))
                {
                    sw.WriteLine($"timestamp={DateTime.Now:yyyy-MM-dd HH:mm:ss}");
                    sw.WriteLine($"arena_width_cm={ARENA_WIDTH_REAL_CM.ToString(CI)}");
                    sw.WriteLine($"arena_height_cm={ARENA_HEIGHT_REAL_CM.ToString(CI)}");
                    sw.WriteLine($"grid_cell_size_cm={currentGridCellSizeCm.ToString(CI)}");
                    sw.WriteLine($"goal_x_cm={goal.X.ToString(CI)}");
                    sw.WriteLine($"goal_y_cm={goal.Y.ToString(CI)}");
                    sw.WriteLine($"k_att={leaderReactivePF.AttractiveGain.ToString(CI)}");
                    sw.WriteLine($"k_rep={leaderReactivePF.RepulsiveGain.ToString(CI)}");
                    sw.WriteLine($"obstacle_influence_range_cm={leaderReactivePF.ObstacleInfluenceRange.ToString(CI)}");
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[PF LOG] Meta error: {ex.Message}");
            }
        }

        private void SnapshotScenarioFiles(string tag)
        {
            try
            {
                if (string.IsNullOrEmpty(_pfScenarioDir)) return;

                void CopyIfExists(string src, string dstName)
                {
                    if (File.Exists(src))
                    {
                        File.Copy(src, Path.Combine(_pfScenarioDir, dstName), true);
                    }
                }

                CopyIfExists(OBSTACLE_FILE_PATH, $"obstacles_{tag}.txt");
                CopyIfExists(LEADER_POSE_PATH, $"leader_pose_{tag}.txt");
                CopyIfExists(FOLLOWER_POSE_PATH, $"follower_pose_{tag}.txt");
                CopyIfExists(Path.Combine(PoseDir, "grid_config.txt"), $"grid_config_{tag}.txt");
                CopyIfExists(Path.Combine(PoseDir, "pixels_per_grid.txt"), $"pixels_per_grid_{tag}.txt");
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[PF LOG] Snapshot error: {ex.Message}");
            }
        }

        private void LogPFRow(float v, float w, float fx, float fy)
        {
            StreamWriter writer;
            System.Diagnostics.Stopwatch sw;

            lock (_pfLogLock)
            {
                writer = _pfLogWriter;
                sw = _pfLogSw;
            }

            if (writer == null || sw == null) return;
            if (leaderReactivePF == null) return;

            long tMs = sw.ElapsedMilliseconds;

            PointF goal = leaderReactivePF.HasGoal ? leaderReactivePF.GetGoalPosition() : new PointF(float.NaN, float.NaN);

            float dxg = goal.X - leaderX;
            float dyg = goal.Y - leaderY;
            float distGoal = (float)Math.Sqrt(dxg * dxg + dyg * dyg);

            int obstCount = 0;
            float minObs = float.NaN;

            try
            {
                var obs = currentVisionObstacles;
                if (obs != null && obs.Count > 0)
                {
                    obstCount = obs.Count;
                    float best = float.PositiveInfinity;

                    for (int i = 0; i < obs.Count; i++)
                    {
                        float dx = obs[i].X - leaderX;
                        float dy = obs[i].Y - leaderY;
                        float d = (float)Math.Sqrt(dx * dx + dy * dy);
                        if (d < best) best = d;
                    }

                    minObs = best;
                }
            }
            catch { }

            // === FIX: Pakai safe variables untuk follower ===
            float logFollowerX = haveSafeFollowerPose ? safeFollowerX : 0f;
            float logFollowerY = haveSafeFollowerPose ? safeFollowerY : 0f;
            float logFollowerTh = haveSafeFollowerPose ? safeFollowerTh : 0f;

            string line =
                $"{tMs}," +
                $"{leaderX.ToString("F2", CI)},{leaderY.ToString("F2", CI)},{leaderTh.ToString("F4", CI)}," +
                $"{logFollowerX.ToString("F2", CI)},{logFollowerY.ToString("F2", CI)},{logFollowerTh.ToString("F4", CI)}," +
                $"{goal.X.ToString("F2", CI)},{goal.Y.ToString("F2", CI)}," +
                $"{fx.ToString("F3", CI)},{fy.ToString("F3", CI)}," +
                $"{v.ToString("F3", CI)},{w.ToString("F3", CI)}," +
                $"{distGoal.ToString("F2", CI)}," +
                $"{obstCount}," +
                $"{(float.IsNaN(minObs) ? "" : minObs.ToString("F2", CI))}," +
                $"{leaderReactivePF.AttractiveGain.ToString("F3", CI)}," +
                $"{leaderReactivePF.RepulsiveGain.ToString("F3", CI)}," +
                $"{leaderReactivePF.ObstacleInfluenceRange.ToString("F1", CI)}";

            lock (_pfLogLock)
            {
                if (_pfLogWriter == null) return;

                _pfLogWriter.WriteLine(line);
                _pfLogRowCount++;

                if (_pfLogRowCount % 10 == 0)
                    _pfLogWriter.Flush();
            }
        }


        private void HandleReactivePFGoalReached()
        {

            MessageBox.Show("🎉 Robot telah mencapai Goal!", "Goal Reached",
                MessageBoxButtons.OK, MessageBoxIcon.Information);

            BtnStopReactivePF_Click(null, null);
            ResetForNewGoal();
        }

        private void UpdateReactivePFStatus(string msg)
        {
            if (lblReactivePFStatus != null)
            {
                lblReactivePFStatus.Text = $"Status: {msg}";
            }
            UpdateStatusLabel(msg);
        }

        private void UpdateReactivePFFromVision()
        {
            if (!isReactivePFMode) return;
            if (leaderReactivePF == null) return;

            // Update leader pose
            if (haveLeaderPose)
            {
                leaderReactivePF.UpdateLeaderPose(leaderX, leaderY, leaderTh);
                followerReactivePF?.UpdateLeaderPose(leaderX, leaderY, leaderTh);
            }

            // Update follower pose
            if (haveFollowerPose)
            {
                followerReactivePF?.UpdateFollowerPose(followerX, followerY, followerTh);
            }

            // Update obstacles
            // Update obstacles (termasuk kosong -> clear obstacles lama)
            var obs = currentVisionObstacles ?? new List<PointF>();
            leaderReactivePF.UpdateObstacles(obs);
            followerReactivePF?.UpdateObstacles(obs);


            // Update force display
            if (lblForceVisualization != null)
            {
                long now = Environment.TickCount64;
                if (now - _lastForceUiMs > 200) // 5 Hz
                {
                    _lastForceUiMs = now;
                    var force = leaderReactivePF.GetLastTotalForce();
                    lblForceVisualization.Text = $"Force: ({force.X:F1}, {force.Y:F1})";
                }
            }

        }

        private void HandleReactivePFModeSelection()
        {
            bool isReactivePF = (currentMode == PathPlanningMode.PureReactivePF);

            // Show/hide Reactive PF UI
            if (reactivePFGroupBox != null) reactivePFGroupBox.Visible = isReactivePF;
            //if (tuningGroupBox != null) tuningGroupBox.Visible = isReactivePF;

            // Hide standard path UI when in reactive mode
            if (sendWaypointsButton != null) sendWaypointsButton.Visible = !isReactivePF;

            if (isReactivePF)
            {
                UpdateStatusLabel("Mode: Pure Reactive PF. Set goal lalu klik START.");

                // Clear pre-computed path
                ClearPathFromGrid();
                calculatedPath = null;
            }
        }
        // Di MainForm.cs


        private void DrawReactivePFVisualization(Graphics g)
        {
            if (!isReactivePFMode || !showForceVisualization) return;
            if (leaderReactivePF == null) return;

            DrawPFLegend(g);

            // Draw goal marker
            if (leaderReactivePF.HasGoal)
            {
                PointF goalPos = leaderReactivePF.GetGoalPosition();
                Point goalPix = WorldToPixelRPF(goalPos.X, goalPos.Y);

                // Outer ring
                using (Pen goalPen = new Pen(Color.Red, 3))
                {
                    g.DrawEllipse(goalPen, goalPix.X - 18, goalPix.Y - 18, 36, 36);
                }

                // Inner circle
                g.FillEllipse(Brushes.OrangeRed, goalPix.X - 10, goalPix.Y - 10, 20, 20);

                // Distance label
                float dist = leaderReactivePF.GetDistanceToGoal();
                using (Font font = new Font("Arial", 9, FontStyle.Bold))
                {
                    g.DrawString($"{dist:F0}cm", font, Brushes.Red, goalPix.X + 22, goalPix.Y - 8);
                }
            }

            // ========== LEADER REPULSIVE ZONE (for follower avoidance) ==========
            // Draw leader's repulsive influence zone FIRST (behind leader robot)
            if (haveLeaderPose && followerReactivePF != null && followerReactivePF.IsRunning)
            {
                Point leaderPix = WorldToPixelRPF(leaderX, leaderY);

                // Get follower's LeaderAvoidanceRange parameter
                // Get follower's LeaderAvoidanceRange parameter (PHYSICAL)
                float leaderAvoidRange = followerReactivePF.LeaderAvoidanceRange;

                // === SEPARATE: Visual radius (for display only) ===
                float VISUAL_SCALE_FACTOR = 0.6f;  // 60% dari physical range (tuning parameter)
                float visualRadius = leaderAvoidRange * VISUAL_SCALE_FACTOR;  // e.g., 25cm * 0.6 = 15cm
                int leaderRadiusPix = (int)(visualRadius * GetPixelsPerCmRPF());

                // Draw leader's repulsive zone (semi-transparent blue) - VISUAL ONLY
                using (Brush leaderZoneBrush = new SolidBrush(Color.FromArgb(35, 0, 100, 255)))
                {
                    g.FillEllipse(leaderZoneBrush, leaderPix.X - leaderRadiusPix, leaderPix.Y - leaderRadiusPix,
                        leaderRadiusPix * 2, leaderRadiusPix * 2);
                }

                // Label for leader zone
                using (Font font = new Font("Arial", 8, FontStyle.Regular))
                {
                    string zoneLabel = $"Leader Zone: {leaderAvoidRange:F0}cm";
                    SizeF textSize = g.MeasureString(zoneLabel, font);
                    PointF labelPos = new PointF(leaderPix.X - textSize.Width / 2, leaderPix.Y - leaderRadiusPix - 15);

                    // Background rectangle for better readability
                    g.FillRectangle(Brushes.White, labelPos.X - 2, labelPos.Y - 2, textSize.Width + 4, textSize.Height + 4);
                    g.DrawString(zoneLabel, font, Brushes.DarkBlue, labelPos);
                }
            }

            // Draw force vector on robot (LEADER)
            if (haveLeaderPose && lastForceVector != PointF.Empty)
            {
                Point robotPix = WorldToPixelRPF(leaderX, leaderY);

                // Force vector (magenta arrow)
                float forceScale = 2.5f;
                int endX = robotPix.X + (int)(lastForceVector.X * forceScale);
                int endY = robotPix.Y - (int)(lastForceVector.Y * forceScale); // Y inverted

                using (Pen forcePen = new Pen(Color.Magenta, 3))
                {
                    forcePen.EndCap = System.Drawing.Drawing2D.LineCap.ArrowAnchor;
                    g.DrawLine(forcePen, robotPix, new Point(endX, endY));
                }

                // Robot heading (blue arrow)
                float headLen = 30;
                int headEndX = robotPix.X + (int)(headLen * Math.Cos(leaderTh));
                int headEndY = robotPix.Y - (int)(headLen * Math.Sin(leaderTh));

                using (Pen headPen = new Pen(Color.Blue, 2))
                {
                    headPen.EndCap = System.Drawing.Drawing2D.LineCap.ArrowAnchor;
                    g.DrawLine(headPen, robotPix, new Point(headEndX, headEndY));
                }

                // Robot body circle (LEADER - Green with "L" label)
                g.FillEllipse(Brushes.LimeGreen, robotPix.X - 12, robotPix.Y - 12, 24, 24);
                g.DrawEllipse(Pens.DarkGreen, robotPix.X - 12, robotPix.Y - 12, 24, 24);

                // Label "L" for Leader
                using (Font font = new Font("Arial", 10, FontStyle.Bold))
                using (StringFormat sf = new StringFormat { Alignment = StringAlignment.Center, LineAlignment = StringAlignment.Center })
                {
                    g.DrawString("L", font, Brushes.White, new RectangleF(robotPix.X - 12, robotPix.Y - 12, 24, 24), sf);
                }
            }

            // Draw obstacles (simple markers WITHOUT large zones to reduce clutter)
            var obstacles = leaderReactivePF.GetObstacles();

            foreach (var obs in obstacles)
            {
                Point obsPix = WorldToPixelRPF(obs.X, obs.Y);

                // Obstacle center only (smaller, less prominent)
                g.FillEllipse(Brushes.DarkRed, obsPix.X - 6, obsPix.Y - 6, 12, 12);
                g.DrawEllipse(Pens.Red, obsPix.X - 6, obsPix.Y - 6, 12, 12);
            }

            // Wall follow indicator
            if (leaderReactivePF.IsWallFollowing)
            {
                using (Font font = new Font("Arial", 11, FontStyle.Bold))
                {
                    g.FillRectangle(Brushes.Orange, 5, 5, 180, 25);
                    g.DrawString("⚠ WALL FOLLOW MODE", font, Brushes.White, 10, 8);
                }
            }
        }

        // === NEW: draw follower overlay on top of the grid ===
        private void DrawFollowerOverlay(Graphics g)
        {
            if (!haveFollowerPose) return;

            // Convert follower pose (cm) -> pixel
            Point fpix = WorldToPixelRPF(followerX, followerY);

            // Heading arrow
            float headLen = 26f;
            int hx = fpix.X + (int)(headLen * Math.Cos(followerTh));
            int hy = fpix.Y - (int)(headLen * Math.Sin(followerTh)); // Y inverted (same style as leader)

            using (Pen headPen = new Pen(Color.CadetBlue, 2))
            {
                headPen.EndCap = System.Drawing.Drawing2D.LineCap.ArrowAnchor;
                g.DrawLine(headPen, fpix, new Point(hx, hy));
            }

            // Body
            g.FillEllipse(Brushes.DeepSkyBlue, fpix.X - 10, fpix.Y - 10, 20, 20);
            g.DrawEllipse(Pens.Navy, fpix.X - 10, fpix.Y - 10, 20, 20);

            // Label "F"
            using (Font font = new Font("Arial", 8, FontStyle.Bold))
            using (StringFormat sf = new StringFormat { Alignment = StringAlignment.Center, LineAlignment = StringAlignment.Center })
            {
                g.DrawString("F", font, Brushes.White, new RectangleF(fpix.X - 10, fpix.Y - 10, 20, 20), sf);
            }
        }


        private Point WorldToPixelRPF(float xCm, float yCm)
        {
            float pixPerCm = cellSizePixelsFloat / currentGridCellSizeCm;

            int pixX = gridOffsetX + (int)(xCm * pixPerCm);
            int pixY = gridOffsetY + (int)((ARENA_HEIGHT_REAL_CM - yCm) * pixPerCm);

            return new Point(pixX, pixY);
        }

        private float GetPixelsPerCmRPF()
        {
            return cellSizePixelsFloat / currentGridCellSizeCm;
        }

        private void CleanupReactivePF()
        {
            try
            {
                isReactivePFMode = false;
                leaderReactivePF?.Dispose();
                followerReactivePF?.Dispose();
            }
            catch { }
        }
        #endregion
        private void arenaPictureBox_Click(object sender, EventArgs e)
        {

        }

        // ===== NEW: Event handler untuk algorithm selection =====
        private void AlgorithmComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            // Update mode berdasarkan ComboBox selection
            currentMode = (PathPlanningMode)algorithmComboBox.SelectedIndex;
            HandleReactivePFModeSelection();

            System.Diagnostics.Debug.WriteLine($"Algorithm changed to: {currentMode}");

            // Re-calculate path jika goal sudah di-set
            if (currentGoalGridPos.HasValue)
            {
                UpdateStatusLabel($"Switching to {currentMode}. Re-calculating path...");
                FindAndDisplayPath();
            }
            else
            {
                UpdateStatusLabel($"Algorithm set to {currentMode}. Set a goal to see the path.");
            }
        }
        // =========================================================
    }
}