namespace HybridDijkstraPotentialField
{
    partial class MainForm
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            components = new System.ComponentModel.Container();
            arenaPictureBox = new PictureBox();
            toggleObstacleModeButton = new Button();
            clearObstaclesButton = new Button();
            sendWaypointsButton = new Button();
            statusLabel = new Label();
            changeGridButton = new Button();
            updatePositionButton = new Button();
            followerUpdateTimer = new System.Windows.Forms.Timer(components);
            algorithmComboBox = new ComboBox();
            controlPanel = new Panel();
            algorithmLabel = new Label();
            titleLabel = new Label();
            statusStrip = new StatusStrip();
            statusStripLabel = new ToolStripStatusLabel();
            arenaPanel = new Panel();
            ((System.ComponentModel.ISupportInitialize)arenaPictureBox).BeginInit();
            controlPanel.SuspendLayout();
            statusStrip.SuspendLayout();
            arenaPanel.SuspendLayout();
            SuspendLayout();
            // 
            // arenaPictureBox
            // 
            arenaPictureBox.BackColor = Color.White;
            arenaPictureBox.BorderStyle = BorderStyle.FixedSingle;
            arenaPictureBox.Dock = DockStyle.Fill;
            arenaPictureBox.Location = new Point(10, 10);
            arenaPictureBox.Margin = new Padding(4, 5, 4, 5);
            arenaPictureBox.Name = "arenaPictureBox";
            arenaPictureBox.Size = new Size(983, 775);
            arenaPictureBox.TabIndex = 0;
            arenaPictureBox.TabStop = false;
            arenaPictureBox.Click += arenaPictureBox_Click;
            arenaPictureBox.Paint += ArenaPictureBox_Paint;
            // 
            // toggleObstacleModeButton
            // 
            toggleObstacleModeButton.BackColor = Color.FromArgb(52, 152, 219);
            toggleObstacleModeButton.Cursor = Cursors.Hand;
            toggleObstacleModeButton.FlatAppearance.BorderSize = 0;
            toggleObstacleModeButton.FlatStyle = FlatStyle.Flat;
            toggleObstacleModeButton.Font = new Font("Segoe UI Semibold", 10F, FontStyle.Bold);
            toggleObstacleModeButton.ForeColor = Color.White;
            toggleObstacleModeButton.Location = new Point(15, 100);
            toggleObstacleModeButton.Margin = new Padding(4, 5, 4, 5);
            toggleObstacleModeButton.Name = "toggleObstacleModeButton";
            toggleObstacleModeButton.Size = new Size(230, 50);
            toggleObstacleModeButton.TabIndex = 2;
            toggleObstacleModeButton.Text = "🚧 Mode Obstacle: OFF";
            toggleObstacleModeButton.UseVisualStyleBackColor = false;
            toggleObstacleModeButton.Click += toggleObstacleModeButton_Click_1;
            toggleObstacleModeButton.MouseEnter += Button_MouseEnter;
            toggleObstacleModeButton.MouseLeave += Button_MouseLeave;
            // 
            // clearObstaclesButton
            // 
            clearObstaclesButton.BackColor = Color.FromArgb(231, 76, 60);
            clearObstaclesButton.Cursor = Cursors.Hand;
            clearObstaclesButton.FlatAppearance.BorderSize = 0;
            clearObstaclesButton.FlatStyle = FlatStyle.Flat;
            clearObstaclesButton.Font = new Font("Segoe UI Semibold", 10F, FontStyle.Bold);
            clearObstaclesButton.ForeColor = Color.White;
            clearObstaclesButton.Location = new Point(15, 160);
            clearObstaclesButton.Margin = new Padding(4, 5, 4, 5);
            clearObstaclesButton.Name = "clearObstaclesButton";
            clearObstaclesButton.Size = new Size(230, 50);
            clearObstaclesButton.TabIndex = 3;
            clearObstaclesButton.Text = "🗑️ Bersihkan Obstacle";
            clearObstaclesButton.UseVisualStyleBackColor = false;
            clearObstaclesButton.MouseEnter += Button_MouseEnter;
            clearObstaclesButton.MouseLeave += Button_MouseLeave;
            // 
            // sendWaypointsButton
            // 
            sendWaypointsButton.BackColor = Color.FromArgb(46, 204, 113);
            sendWaypointsButton.Cursor = Cursors.Hand;
            sendWaypointsButton.FlatAppearance.BorderSize = 0;
            sendWaypointsButton.FlatStyle = FlatStyle.Flat;
            sendWaypointsButton.Font = new Font("Segoe UI Semibold", 10F, FontStyle.Bold);
            sendWaypointsButton.ForeColor = Color.White;
            sendWaypointsButton.Location = new Point(15, 220);
            sendWaypointsButton.Margin = new Padding(4, 5, 4, 5);
            sendWaypointsButton.Name = "sendWaypointsButton";
            sendWaypointsButton.Size = new Size(230, 50);
            sendWaypointsButton.TabIndex = 4;
            sendWaypointsButton.Text = "🚀 Kirim ke Robot";
            sendWaypointsButton.UseVisualStyleBackColor = false;
            sendWaypointsButton.Click += SendWaypointsButton_Click;
            sendWaypointsButton.MouseEnter += Button_MouseEnter;
            sendWaypointsButton.MouseLeave += Button_MouseLeave;
            // 
            // statusLabel
            // 
            statusLabel.Dock = DockStyle.Bottom;
            statusLabel.Location = new Point(0, 827);
            statusLabel.Margin = new Padding(4, 0, 4, 0);
            statusLabel.Name = "statusLabel";
            statusLabel.Size = new Size(1263, 38);
            statusLabel.TabIndex = 5;
            statusLabel.Text = "Selamat datang! Klik arena untuk set GOAL";
            statusLabel.Visible = false;
            // 
            // changeGridButton
            // 
            changeGridButton.BackColor = Color.FromArgb(155, 89, 182);
            changeGridButton.Cursor = Cursors.Hand;
            changeGridButton.FlatAppearance.BorderSize = 0;
            changeGridButton.FlatStyle = FlatStyle.Flat;
            changeGridButton.Font = new Font("Segoe UI Semibold", 10F, FontStyle.Bold);
            changeGridButton.ForeColor = Color.White;
            changeGridButton.Location = new Point(15, 280);
            changeGridButton.Margin = new Padding(4, 5, 4, 5);
            changeGridButton.Name = "changeGridButton";
            changeGridButton.Size = new Size(230, 50);
            changeGridButton.TabIndex = 6;
            changeGridButton.Text = "📐 Grid: 15x15";
            changeGridButton.UseVisualStyleBackColor = false;
            changeGridButton.Click += changeGridButton_Click;
            changeGridButton.MouseEnter += Button_MouseEnter;
            changeGridButton.MouseLeave += Button_MouseLeave;
            // 
            // updatePositionButton
            // 
            updatePositionButton.BackColor = Color.FromArgb(241, 196, 15);
            updatePositionButton.Cursor = Cursors.Hand;
            updatePositionButton.FlatAppearance.BorderSize = 0;
            updatePositionButton.FlatStyle = FlatStyle.Flat;
            updatePositionButton.Font = new Font("Segoe UI Semibold", 10F, FontStyle.Bold);
            updatePositionButton.ForeColor = Color.White;
            updatePositionButton.Location = new Point(15, 340);
            updatePositionButton.Margin = new Padding(4, 5, 4, 5);
            updatePositionButton.Name = "updatePositionButton";
            updatePositionButton.Size = new Size(230, 50);
            updatePositionButton.TabIndex = 7;
            updatePositionButton.Text = "📷 Update dari Kamera";
            updatePositionButton.UseVisualStyleBackColor = false;
            updatePositionButton.Click += button1_Click;
            updatePositionButton.MouseEnter += Button_MouseEnter;
            updatePositionButton.MouseLeave += Button_MouseLeave;
            // 
            // followerUpdateTimer
            // 
            followerUpdateTimer.Enabled = true;
            followerUpdateTimer.Interval = 500;
            // 
            // algorithmComboBox
            // 
            algorithmComboBox.BackColor = Color.White;
            algorithmComboBox.DropDownStyle = ComboBoxStyle.DropDownList;
            algorithmComboBox.FlatStyle = FlatStyle.Flat;
            algorithmComboBox.Font = new Font("Segoe UI", 9.75F);
            algorithmComboBox.FormattingEnabled = true;
            algorithmComboBox.Items.AddRange(new object[] { "Hybrid (Dijkstra + PF)", "Dijkstra Only", "Pure Potential Field", "Pure PSO", "Real-time Reactive PF" });
            algorithmComboBox.Location = new Point(15, 450);
            algorithmComboBox.Name = "algorithmComboBox";
            algorithmComboBox.Size = new Size(230, 36);
            algorithmComboBox.TabIndex = 8;
            algorithmComboBox.SelectedIndexChanged += AlgorithmComboBox_SelectedIndexChanged;
            // 
            // controlPanel
            // 
            controlPanel.BackColor = Color.FromArgb(44, 62, 80);
            controlPanel.Controls.Add(algorithmLabel);
            controlPanel.Controls.Add(titleLabel);
            controlPanel.Controls.Add(toggleObstacleModeButton);
            controlPanel.Controls.Add(algorithmComboBox);
            controlPanel.Controls.Add(clearObstaclesButton);
            controlPanel.Controls.Add(updatePositionButton);
            controlPanel.Controls.Add(sendWaypointsButton);
            controlPanel.Controls.Add(changeGridButton);
            controlPanel.Dock = DockStyle.Left;
            controlPanel.Location = new Point(0, 0);
            controlPanel.Name = "controlPanel";
            controlPanel.Padding = new Padding(10);
            controlPanel.Size = new Size(260, 827);
            controlPanel.TabIndex = 9;
            // 
            // algorithmLabel
            // 
            algorithmLabel.AutoSize = true;
            algorithmLabel.Font = new Font("Segoe UI Semibold", 9.75F, FontStyle.Bold);
            algorithmLabel.ForeColor = Color.White;
            algorithmLabel.Location = new Point(15, 420);
            algorithmLabel.Name = "algorithmLabel";
            algorithmLabel.Size = new Size(140, 28);
            algorithmLabel.TabIndex = 10;
            algorithmLabel.Text = "⚙️ Algoritma:";
            // 
            // titleLabel
            // 
            titleLabel.Dock = DockStyle.Top;
            titleLabel.Font = new Font("Segoe UI", 14F, FontStyle.Bold);
            titleLabel.ForeColor = Color.FromArgb(236, 240, 241);
            titleLabel.Location = new Point(10, 10);
            titleLabel.Name = "titleLabel";
            titleLabel.Padding = new Padding(5);
            titleLabel.Size = new Size(240, 70);
            titleLabel.TabIndex = 9;
            titleLabel.Text = "🤖 Path Planning\r\nControl Panel";
            titleLabel.TextAlign = ContentAlignment.MiddleCenter;
            // 
            // statusStrip
            // 
            statusStrip.BackColor = Color.FromArgb(52, 73, 94);
            statusStrip.ImageScalingSize = new Size(24, 24);
            statusStrip.Items.AddRange(new ToolStripItem[] { statusStripLabel });
            statusStrip.Location = new Point(260, 795);
            statusStrip.Name = "statusStrip";
            statusStrip.Padding = new Padding(1, 0, 16, 0);
            statusStrip.Size = new Size(1003, 32);
            statusStrip.TabIndex = 10;
            statusStrip.Text = "statusStrip1";
            // 
            // statusStripLabel
            // 
            statusStripLabel.Font = new Font("Segoe UI", 9F);
            statusStripLabel.ForeColor = Color.White;
            statusStripLabel.Name = "statusStripLabel";
            statusStripLabel.Size = new Size(368, 25);
            statusStripLabel.Text = "✓ Selamat datang! Klik arena untuk set GOAL";
            // 
            // arenaPanel
            // 
            arenaPanel.BackColor = Color.FromArgb(236, 240, 241);
            arenaPanel.Controls.Add(arenaPictureBox);
            arenaPanel.Dock = DockStyle.Fill;
            arenaPanel.Location = new Point(260, 0);
            arenaPanel.Name = "arenaPanel";
            arenaPanel.Padding = new Padding(10);
            arenaPanel.Size = new Size(1003, 795);
            arenaPanel.TabIndex = 11;
            // 
            // MainForm
            // 
            AutoScaleDimensions = new SizeF(10F, 25F);
            AutoScaleMode = AutoScaleMode.Font;
            BackColor = Color.FromArgb(236, 240, 241);
            ClientSize = new Size(1263, 865);
            Controls.Add(arenaPanel);
            Controls.Add(statusStrip);
            Controls.Add(controlPanel);
            Controls.Add(statusLabel);
            Margin = new Padding(4, 5, 4, 5);
            MinimumSize = new Size(1285, 920);
            Name = "MainForm";
            StartPosition = FormStartPosition.CenterScreen;
            Text = "Hybrid Dijkstra & Potential Field - Path Planning System";
            Load += MainForm_Load;
            Shown += MainForm_Shown;
            Paint += MainForm_Paint;
            ((System.ComponentModel.ISupportInitialize)arenaPictureBox).EndInit();
            controlPanel.ResumeLayout(false);
            controlPanel.PerformLayout();
            statusStrip.ResumeLayout(false);
            statusStrip.PerformLayout();
            arenaPanel.ResumeLayout(false);
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private PictureBox arenaPictureBox;
        private Button toggleObstacleModeButton;
        private Button clearObstaclesButton;
        private Button sendWaypointsButton;
        private Label statusLabel;
        private Button changeGridButton;
        private Button updatePositionButton;
        private System.Windows.Forms.Timer followerUpdateTimer;
        private ComboBox algorithmComboBox;
        private Panel controlPanel;
        private Label titleLabel;
        private Label algorithmLabel;
        private StatusStrip statusStrip;
        private ToolStripStatusLabel statusStripLabel;
        private Panel arenaPanel;
    }
}