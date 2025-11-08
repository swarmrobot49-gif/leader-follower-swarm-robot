namespace RobotArenaGUI
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
            ((System.ComponentModel.ISupportInitialize)arenaPictureBox).BeginInit();
            SuspendLayout();
            // 
            // arenaPictureBox
            // 
            arenaPictureBox.BorderStyle = BorderStyle.FixedSingle;
            arenaPictureBox.Location = new Point(284, 20);
            arenaPictureBox.Margin = new Padding(4, 5, 4, 5);
            arenaPictureBox.Name = "arenaPictureBox";
            arenaPictureBox.Size = new Size(835, 600);
            arenaPictureBox.TabIndex = 0;
            arenaPictureBox.TabStop = false;
            arenaPictureBox.Click += arenaPictureBox_Click;
            arenaPictureBox.Paint += ArenaPictureBox_Paint;
            // 
            // toggleObstacleModeButton
            // 
            toggleObstacleModeButton.Location = new Point(40, 20);
            toggleObstacleModeButton.Margin = new Padding(4, 5, 4, 5);
            toggleObstacleModeButton.Name = "toggleObstacleModeButton";
            toggleObstacleModeButton.Size = new Size(184, 85);
            toggleObstacleModeButton.TabIndex = 2;
            toggleObstacleModeButton.Text = "Mode Obstacle: OFF";
            toggleObstacleModeButton.UseVisualStyleBackColor = true;
            toggleObstacleModeButton.Click += toggleObstacleModeButton_Click_1;
            // 
            // clearObstaclesButton
            // 
            clearObstaclesButton.Location = new Point(40, 132);
            clearObstaclesButton.Margin = new Padding(4, 5, 4, 5);
            clearObstaclesButton.Name = "clearObstaclesButton";
            clearObstaclesButton.Size = new Size(184, 78);
            clearObstaclesButton.TabIndex = 3;
            clearObstaclesButton.Text = "Bersihkan Obstacle";
            clearObstaclesButton.UseVisualStyleBackColor = true;
            // 
            // sendWaypointsButton
            // 
            sendWaypointsButton.Location = new Point(40, 235);
            sendWaypointsButton.Margin = new Padding(4, 5, 4, 5);
            sendWaypointsButton.Name = "sendWaypointsButton";
            sendWaypointsButton.Size = new Size(184, 90);
            sendWaypointsButton.TabIndex = 4;
            sendWaypointsButton.Text = "Kirim Waypoint ke Robot";
            sendWaypointsButton.UseVisualStyleBackColor = true;
            sendWaypointsButton.Click += SendWaypointsButton_Click;
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
            // 
            // changeGridButton
            // 
            changeGridButton.Location = new Point(40, 350);
            changeGridButton.Margin = new Padding(4, 5, 4, 5);
            changeGridButton.Name = "changeGridButton";
            changeGridButton.Size = new Size(184, 77);
            changeGridButton.TabIndex = 6;
            changeGridButton.Text = "Grid 15x15";
            changeGridButton.UseVisualStyleBackColor = true;
            changeGridButton.Click += changeGridButton_Click;
            // 
            // updatePositionButton
            // 
            updatePositionButton.Location = new Point(40, 453);
            updatePositionButton.Margin = new Padding(4, 5, 4, 5);
            updatePositionButton.Name = "updatePositionButton";
            updatePositionButton.Size = new Size(184, 75);
            updatePositionButton.TabIndex = 7;
            updatePositionButton.Text = "Update Posisi Robot dari Kamera";
            updatePositionButton.UseVisualStyleBackColor = true;
            updatePositionButton.Click += button1_Click;
            // 
            // followerUpdateTimer
            // 
            followerUpdateTimer.Enabled = true;
            followerUpdateTimer.Interval = 500;
            // 
            // MainForm
            // 
            AutoScaleDimensions = new SizeF(10F, 25F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1263, 865);
            Controls.Add(updatePositionButton);
            Controls.Add(changeGridButton);
            Controls.Add(statusLabel);
            Controls.Add(sendWaypointsButton);
            Controls.Add(clearObstaclesButton);
            Controls.Add(toggleObstacleModeButton);
            Controls.Add(arenaPictureBox);
            Margin = new Padding(4, 5, 4, 5);
            Name = "MainForm";
            Text = "Form1";
            Load += MainForm_Load;
            Shown += MainForm_Shown;
            Paint += MainForm_Paint;
            ((System.ComponentModel.ISupportInitialize)arenaPictureBox).EndInit();
            ResumeLayout(false);
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
    }
}
