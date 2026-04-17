using System;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Globalization;

namespace HybridDijkstraPotentialField
{
    /// <summary>
    /// Kelas untuk logging data telemetry dari Leader Robot ke file.
    /// Mode AUTO: Start saat Send Waypoints, Stop saat robot sampai goal atau mission gagal.
    /// File baru dibuat setiap kali GUI dijalankan ulang.
    /// </summary>
    public class TelemetryLogger
    {
        private StreamWriter logWriter;
        private string currentLogPath;
        private volatile bool isLogging;
        private Thread listenerThread;
        private volatile bool shouldStop;
        
        private TcpClient robotClient;
        private StreamReader robotReader;
        
        private readonly string logDirectory;
        private int sessionTestCounter;  // Counter untuk session saat ini
        
        // Event untuk update UI
        public event Action<string> OnTelemetryReceived;
        public event Action<string> OnStatusChanged;
        public event Action OnMissionCompleted;  // Event saat ALL_WAYPOINTS_REACHED terdeteksi
        
        public bool IsLogging => isLogging;
        public string CurrentLogPath => currentLogPath;
        public int CurrentTestNumber => sessionTestCounter;
        
        public TelemetryLogger(string baseLogDirectory)
        {
            logDirectory = baseLogDirectory;
            Directory.CreateDirectory(logDirectory);
            
            // Session ini dimulai dari 1, akan increment setiap kali StartLogging dipanggil
            sessionTestCounter = 1;
        }
        
        /// <summary>
        /// Mulai logging telemetry baru. Otomatis dipanggil sebelum EXECUTE.
        /// </summary>
        public bool StartLogging(TcpClient client)
        {
            if (isLogging)
            {
                OnStatusChanged?.Invoke("⚠️ Logging sudah berjalan!");
                return false;
            }
            
            try
            {
                robotClient = client;
                
                // Buat nama file dengan format: Session_X_YYYYMMDD_HHMMSS.csv
                // X = nomor test dalam session ini (1, 2, 3, ...)
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string filename = $"Session_{sessionTestCounter}_{timestamp}.csv";
                currentLogPath = Path.Combine(logDirectory, filename);
                
                // Buat file dan tulis header CSV
                logWriter = new StreamWriter(currentLogPath, false, Encoding.UTF8);
                logWriter.WriteLine("# Leader Robot Odometry Log");
                logWriter.WriteLine($"# Session Test Number: {sessionTestCounter}");
                logWriter.WriteLine($"# Start Time: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
                logWriter.WriteLine($"# Format: TEL,timestamp_ms,x_cm,y_cm,theta_deg,targetX_cm,targetY_cm,distToTarget_cm,state");
                logWriter.WriteLine("# State: IDLE=0, TURNING=1, MOVING=2, GOAL_REACHED=3");
                logWriter.WriteLine("#");
                logWriter.WriteLine("timestamp_ms,x_cm,y_cm,theta_deg,targetX_cm,targetY_cm,distToTarget_cm,state");
                logWriter.Flush();
                
                isLogging = true;
                shouldStop = false;
                
                // Start listener thread
                listenerThread = new Thread(TelemetryListenerLoop);
                listenerThread.IsBackground = true;
                listenerThread.Start();
                
                OnStatusChanged?.Invoke($"✅ Logging started: Session Test #{sessionTestCounter} → {filename}");
                
                return true;
            }
            catch (Exception ex)
            {
                OnStatusChanged?.Invoke($"❌ Error starting log: {ex.Message}");
                CleanupResources();
                return false;
            }
        }
        
        /// <summary>
        /// Stop logging dan tutup file
        /// </summary>
        /// <param name="missionSuccess">True jika robot sampai goal, False jika dibatalkan</param>
        public void StopLogging(bool missionSuccess = true)
        {
            if (!isLogging) return;
            
            shouldStop = true;
            isLogging = false;
            
            // Tunggu thread selesai (max 2 detik)
            if (listenerThread != null && listenerThread.IsAlive)
            {
                listenerThread.Join(2000);
            }
            
            CleanupResources(missionSuccess);
            
            if (missionSuccess)
            {
                // Increment counter untuk test berikutnya HANYA jika berhasil
                sessionTestCounter++;
                OnStatusChanged?.Invoke($"✅ Logging stopped (SUCCESS). File saved: {Path.GetFileName(currentLogPath)}");
            }
            else
            {
                // Mission gagal/dibatalkan - hapus file yang tidak lengkap
                try
                {
                    if (File.Exists(currentLogPath))
                    {
                        File.Delete(currentLogPath);
                        OnStatusChanged?.Invoke($"⚠️ Logging cancelled. Incomplete log file deleted.");
                    }
                }
                catch
                {
                    OnStatusChanged?.Invoke($"⚠️ Logging stopped (CANCELLED). File may be incomplete: {Path.GetFileName(currentLogPath)}");
                }
            }
        }
        
        private void CleanupResources(bool missionSuccess = true)
        {
            try
            {
                if (logWriter != null)
                {
                    logWriter.WriteLine($"# End Time: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
                    logWriter.WriteLine($"# Mission Status: {(missionSuccess ? "SUCCESS - Goal Reached" : "CANCELLED - Incomplete")}");
                    logWriter.Flush();
                    logWriter.Close();
                    logWriter.Dispose();
                    logWriter = null;
                }
            }
            catch { }
            
            // Jangan tutup robotClient/robotReader karena masih dipakai untuk command
            robotReader = null;
        }
        
        private void TelemetryListenerLoop()
        {
            try
            {
                if (robotClient == null || !robotClient.Connected)
                {
                    OnStatusChanged?.Invoke("❌ Robot not connected!");
                    return;
                }
                
                robotReader = new StreamReader(robotClient.GetStream(), Encoding.UTF8);
                
                while (!shouldStop && isLogging && robotClient.Connected)
                {
                    try
                    {
                        // Baca line dari robot (non-blocking dengan timeout)
                        if (robotClient.Available > 0)
                        {
                            string line = robotReader.ReadLine();
                            
                            if (!string.IsNullOrEmpty(line))
                            {
                                ProcessTelemetryLine(line);
                            }
                        }
                        else
                        {
                            Thread.Sleep(10); // Small delay jika tidak ada data
                        }
                    }
                    catch (IOException)
                    {
                        // Connection lost
                        break;
                    }
                }
            }
            catch (Exception ex)
            {
                OnStatusChanged?.Invoke($"❌ Listener error: {ex.Message}");
            }
            finally
            {
                isLogging = false;
            }
        }
        
        private void ProcessTelemetryLine(string line)
        {
            // Deteksi mission completion dari robot
            if (line.Contains("ALL_WAYPOINTS_REACHED"))
            {
                System.Diagnostics.Debug.WriteLine("[TELEMETRY] Mission completed detected!");
                
                // Tambahkan note ke log file
                if (logWriter != null)
                {
                    try
                    {
                        logWriter.WriteLine($"# MISSION COMPLETED: {DateTime.Now:HH:mm:ss}");
                        logWriter.Flush();
                    }
                    catch { }
                }
                
                // Trigger event untuk MainForm
                OnMissionCompleted?.Invoke();
                return;
            }
            
            // Format dari robot: TEL,timestamp,x,y,theta,targetX,targetY,distToTarget,state
            if (!line.StartsWith("TEL,"))
            {
                // Bukan telemetry data, mungkin feedback lain dari robot
                return;
            }
            
            try
            {
                // Buang prefix "TEL," dan split
                string data = line.Substring(4);
                
                // Tulis langsung ke file (sudah dalam format CSV)
                if (logWriter != null)
                {
                    logWriter.WriteLine(data);
                    logWriter.Flush(); // Force write ke disk
                }
                
                // Trigger event untuk update UI (optional)
                OnTelemetryReceived?.Invoke(line);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"[TELEMETRY] Parse error: {ex.Message}");
            }
        }
        
        /// <summary>
        /// Tambahkan note/comment ke file log
        /// </summary>
        public void AddNote(string note)
        {
            if (isLogging && logWriter != null)
            {
                try
                {
                    logWriter.WriteLine($"# NOTE [{DateTime.Now:HH:mm:ss}]: {note}");
                    logWriter.Flush();
                }
                catch { }
            }
        }
    }
}
