namespace SuperDelaunay.UI
{
    partial class DebuggerForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
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
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle1 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle2 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle3 = new System.Windows.Forms.DataGridViewCellStyle();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.label1 = new System.Windows.Forms.Label();
            this.buttonInsertAll = new System.Windows.Forms.Button();
            this.buttonInsertOne = new System.Windows.Forms.Button();
            this.buttonInsertRange = new System.Windows.Forms.Button();
            this.trackBar1 = new System.Windows.Forms.TrackBar();
            this.buttonReset = new System.Windows.Forms.Button();
            this.checkBoxVertexOrthoballs = new System.Windows.Forms.CheckBox();
            this.checkBoxVertexNewTriangles = new System.Windows.Forms.CheckBox();
            this.checkBoxVertexWeight = new System.Windows.Forms.CheckBox();
            this.checkBoxVertexBadTriangles = new System.Windows.Forms.CheckBox();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.textBoxTriangulationInfo = new System.Windows.Forms.TextBox();
            this.checkBoxTriangulationOrthoballs = new System.Windows.Forms.CheckBox();
            this.checkBoxTriangulationWeights = new System.Windows.Forms.CheckBox();
            this.groupBoxTimer = new System.Windows.Forms.GroupBox();
            this.dataGridView1 = new System.Windows.Forms.DataGridView();
            this.splitContainer1 = new System.Windows.Forms.SplitContainer();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.flowLayoutPanel1 = new System.Windows.Forms.FlowLayoutPanel();
            this.checkBoxTriangulationVertices = new System.Windows.Forms.CheckBox();
            this.checkBoxTriangulationEdges = new System.Windows.Forms.CheckBox();
            this.groupBox1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar1)).BeginInit();
            this.groupBox3.SuspendLayout();
            this.groupBoxTimer.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.dataGridView1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).BeginInit();
            this.splitContainer1.Panel1.SuspendLayout();
            this.splitContainer1.Panel2.SuspendLayout();
            this.splitContainer1.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.flowLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.buttonInsertAll);
            this.groupBox1.Controls.Add(this.buttonInsertOne);
            this.groupBox1.Controls.Add(this.buttonInsertRange);
            this.groupBox1.Controls.Add(this.trackBar1);
            this.groupBox1.Controls.Add(this.buttonReset);
            this.groupBox1.Dock = System.Windows.Forms.DockStyle.Top;
            this.groupBox1.Location = new System.Drawing.Point(4, 4);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(550, 77);
            this.groupBox1.TabIndex = 1;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Controls";
            // 
            // label1
            // 
            this.label1.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.label1.Location = new System.Drawing.Point(168, 53);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(375, 13);
            this.label1.TabIndex = 5;
            this.label1.Text = "label1";
            // 
            // buttonInsertAll
            // 
            this.buttonInsertAll.Location = new System.Drawing.Point(87, 48);
            this.buttonInsertAll.Name = "buttonInsertAll";
            this.buttonInsertAll.Size = new System.Drawing.Size(75, 23);
            this.buttonInsertAll.TabIndex = 2;
            this.buttonInsertAll.Text = "Insert all";
            this.buttonInsertAll.UseVisualStyleBackColor = true;
            this.buttonInsertAll.Click += new System.EventHandler(this.buttonInsertAll_Click);
            // 
            // buttonInsertOne
            // 
            this.buttonInsertOne.Location = new System.Drawing.Point(6, 48);
            this.buttonInsertOne.Name = "buttonInsertOne";
            this.buttonInsertOne.Size = new System.Drawing.Size(75, 23);
            this.buttonInsertOne.TabIndex = 1;
            this.buttonInsertOne.Text = "Insert one";
            this.buttonInsertOne.UseVisualStyleBackColor = true;
            this.buttonInsertOne.Click += new System.EventHandler(this.buttonInsertOne_Click);
            // 
            // buttonInsertRange
            // 
            this.buttonInsertRange.Location = new System.Drawing.Point(87, 19);
            this.buttonInsertRange.Name = "buttonInsertRange";
            this.buttonInsertRange.Size = new System.Drawing.Size(75, 23);
            this.buttonInsertRange.TabIndex = 4;
            this.buttonInsertRange.Text = "Insert 1";
            this.buttonInsertRange.UseVisualStyleBackColor = true;
            this.buttonInsertRange.Click += new System.EventHandler(this.buttonInsertRange_Click);
            // 
            // trackBar1
            // 
            this.trackBar1.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.trackBar1.Location = new System.Drawing.Point(164, 19);
            this.trackBar1.Name = "trackBar1";
            this.trackBar1.Size = new System.Drawing.Size(379, 45);
            this.trackBar1.TabIndex = 3;
            this.trackBar1.Scroll += new System.EventHandler(this.trackBar1_Scroll);
            // 
            // buttonReset
            // 
            this.buttonReset.Location = new System.Drawing.Point(6, 19);
            this.buttonReset.Name = "buttonReset";
            this.buttonReset.Size = new System.Drawing.Size(75, 23);
            this.buttonReset.TabIndex = 0;
            this.buttonReset.Text = "Reset";
            this.buttonReset.UseVisualStyleBackColor = true;
            this.buttonReset.Click += new System.EventHandler(this.buttonReset_Click);
            // 
            // checkBoxVertexOrthoballs
            // 
            this.checkBoxVertexOrthoballs.AutoSize = true;
            this.checkBoxVertexOrthoballs.Location = new System.Drawing.Point(280, 3);
            this.checkBoxVertexOrthoballs.Name = "checkBoxVertexOrthoballs";
            this.checkBoxVertexOrthoballs.Size = new System.Drawing.Size(73, 17);
            this.checkBoxVertexOrthoballs.TabIndex = 3;
            this.checkBoxVertexOrthoballs.Text = "Orthoballs";
            this.checkBoxVertexOrthoballs.UseVisualStyleBackColor = true;
            this.checkBoxVertexOrthoballs.CheckedChanged += new System.EventHandler(this.checkBoxChanged);
            // 
            // checkBoxVertexNewTriangles
            // 
            this.checkBoxVertexNewTriangles.AutoSize = true;
            this.checkBoxVertexNewTriangles.Location = new System.Drawing.Point(184, 3);
            this.checkBoxVertexNewTriangles.Name = "checkBoxVertexNewTriangles";
            this.checkBoxVertexNewTriangles.Size = new System.Drawing.Size(90, 17);
            this.checkBoxVertexNewTriangles.TabIndex = 4;
            this.checkBoxVertexNewTriangles.Text = "New triangles";
            this.checkBoxVertexNewTriangles.UseVisualStyleBackColor = true;
            this.checkBoxVertexNewTriangles.CheckedChanged += new System.EventHandler(this.checkBoxChanged);
            // 
            // checkBoxVertexWeight
            // 
            this.checkBoxVertexWeight.AutoSize = true;
            this.checkBoxVertexWeight.Checked = true;
            this.checkBoxVertexWeight.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBoxVertexWeight.Location = new System.Drawing.Point(3, 3);
            this.checkBoxVertexWeight.Name = "checkBoxVertexWeight";
            this.checkBoxVertexWeight.Size = new System.Drawing.Size(82, 17);
            this.checkBoxVertexWeight.TabIndex = 0;
            this.checkBoxVertexWeight.Text = "Next weight";
            this.checkBoxVertexWeight.UseVisualStyleBackColor = true;
            this.checkBoxVertexWeight.CheckedChanged += new System.EventHandler(this.checkBoxChanged);
            // 
            // checkBoxVertexBadTriangles
            // 
            this.checkBoxVertexBadTriangles.AutoSize = true;
            this.checkBoxVertexBadTriangles.Checked = true;
            this.checkBoxVertexBadTriangles.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBoxVertexBadTriangles.Location = new System.Drawing.Point(91, 3);
            this.checkBoxVertexBadTriangles.Name = "checkBoxVertexBadTriangles";
            this.checkBoxVertexBadTriangles.Size = new System.Drawing.Size(87, 17);
            this.checkBoxVertexBadTriangles.TabIndex = 1;
            this.checkBoxVertexBadTriangles.Text = "Bad triangles";
            this.checkBoxVertexBadTriangles.UseVisualStyleBackColor = true;
            this.checkBoxVertexBadTriangles.CheckedChanged += new System.EventHandler(this.checkBoxChanged);
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.textBoxTriangulationInfo);
            this.groupBox3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.groupBox3.Location = new System.Drawing.Point(0, 0);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(550, 146);
            this.groupBox3.TabIndex = 2;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Info";
            // 
            // textBoxTriangulationInfo
            // 
            this.textBoxTriangulationInfo.BackColor = System.Drawing.SystemColors.Window;
            this.textBoxTriangulationInfo.Dock = System.Windows.Forms.DockStyle.Fill;
            this.textBoxTriangulationInfo.Font = new System.Drawing.Font("Microsoft Sans Serif", 8F);
            this.textBoxTriangulationInfo.Location = new System.Drawing.Point(3, 16);
            this.textBoxTriangulationInfo.Multiline = true;
            this.textBoxTriangulationInfo.Name = "textBoxTriangulationInfo";
            this.textBoxTriangulationInfo.ReadOnly = true;
            this.textBoxTriangulationInfo.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.textBoxTriangulationInfo.Size = new System.Drawing.Size(544, 127);
            this.textBoxTriangulationInfo.TabIndex = 4;
            this.textBoxTriangulationInfo.WordWrap = false;
            // 
            // checkBoxTriangulationOrthoballs
            // 
            this.checkBoxTriangulationOrthoballs.AutoSize = true;
            this.checkBoxTriangulationOrthoballs.Checked = true;
            this.checkBoxTriangulationOrthoballs.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBoxTriangulationOrthoballs.Location = new System.Drawing.Point(430, 3);
            this.checkBoxTriangulationOrthoballs.Name = "checkBoxTriangulationOrthoballs";
            this.checkBoxTriangulationOrthoballs.Size = new System.Drawing.Size(73, 17);
            this.checkBoxTriangulationOrthoballs.TabIndex = 6;
            this.checkBoxTriangulationOrthoballs.Text = "Orthoballs";
            this.checkBoxTriangulationOrthoballs.UseVisualStyleBackColor = true;
            this.checkBoxTriangulationOrthoballs.CheckedChanged += new System.EventHandler(this.checkBoxChanged);
            // 
            // checkBoxTriangulationWeights
            // 
            this.checkBoxTriangulationWeights.AutoSize = true;
            this.checkBoxTriangulationWeights.Checked = true;
            this.checkBoxTriangulationWeights.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBoxTriangulationWeights.Location = new System.Drawing.Point(359, 3);
            this.checkBoxTriangulationWeights.Name = "checkBoxTriangulationWeights";
            this.checkBoxTriangulationWeights.Size = new System.Drawing.Size(65, 17);
            this.checkBoxTriangulationWeights.TabIndex = 4;
            this.checkBoxTriangulationWeights.Text = "Weights";
            this.checkBoxTriangulationWeights.UseVisualStyleBackColor = true;
            this.checkBoxTriangulationWeights.CheckedChanged += new System.EventHandler(this.checkBoxChanged);
            // 
            // groupBoxTimer
            // 
            this.groupBoxTimer.Controls.Add(this.dataGridView1);
            this.groupBoxTimer.Dock = System.Windows.Forms.DockStyle.Fill;
            this.groupBoxTimer.Location = new System.Drawing.Point(0, 0);
            this.groupBoxTimer.Name = "groupBoxTimer";
            this.groupBoxTimer.Size = new System.Drawing.Size(550, 459);
            this.groupBoxTimer.TabIndex = 0;
            this.groupBoxTimer.TabStop = false;
            this.groupBoxTimer.Text = "Timer";
            // 
            // dataGridView1
            // 
            this.dataGridView1.AllowUserToAddRows = false;
            this.dataGridView1.AllowUserToDeleteRows = false;
            this.dataGridView1.AllowUserToResizeRows = false;
            this.dataGridView1.AutoSizeColumnsMode = System.Windows.Forms.DataGridViewAutoSizeColumnsMode.AllCells;
            this.dataGridView1.BackgroundColor = System.Drawing.SystemColors.Window;
            this.dataGridView1.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            dataGridViewCellStyle1.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft;
            dataGridViewCellStyle1.BackColor = System.Drawing.SystemColors.Window;
            dataGridViewCellStyle1.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            dataGridViewCellStyle1.ForeColor = System.Drawing.SystemColors.ControlText;
            dataGridViewCellStyle1.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle1.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            dataGridViewCellStyle1.WrapMode = System.Windows.Forms.DataGridViewTriState.False;
            this.dataGridView1.DefaultCellStyle = dataGridViewCellStyle1;
            this.dataGridView1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.dataGridView1.Location = new System.Drawing.Point(3, 16);
            this.dataGridView1.Name = "dataGridView1";
            this.dataGridView1.ReadOnly = true;
            dataGridViewCellStyle2.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft;
            dataGridViewCellStyle2.BackColor = System.Drawing.SystemColors.Control;
            dataGridViewCellStyle2.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            dataGridViewCellStyle2.ForeColor = System.Drawing.SystemColors.WindowText;
            dataGridViewCellStyle2.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle2.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            dataGridViewCellStyle2.WrapMode = System.Windows.Forms.DataGridViewTriState.True;
            this.dataGridView1.RowHeadersDefaultCellStyle = dataGridViewCellStyle2;
            this.dataGridView1.RowHeadersVisible = false;
            dataGridViewCellStyle3.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft;
            this.dataGridView1.RowsDefaultCellStyle = dataGridViewCellStyle3;
            this.dataGridView1.Size = new System.Drawing.Size(544, 440);
            this.dataGridView1.TabIndex = 1;
            // 
            // splitContainer1
            // 
            this.splitContainer1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.splitContainer1.Location = new System.Drawing.Point(4, 146);
            this.splitContainer1.Name = "splitContainer1";
            this.splitContainer1.Orientation = System.Windows.Forms.Orientation.Horizontal;
            // 
            // splitContainer1.Panel1
            // 
            this.splitContainer1.Panel1.Controls.Add(this.groupBox3);
            // 
            // splitContainer1.Panel2
            // 
            this.splitContainer1.Panel2.Controls.Add(this.groupBoxTimer);
            this.splitContainer1.Size = new System.Drawing.Size(550, 611);
            this.splitContainer1.SplitterDistance = 146;
            this.splitContainer1.SplitterWidth = 6;
            this.splitContainer1.TabIndex = 3;
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.flowLayoutPanel1);
            this.groupBox4.Dock = System.Windows.Forms.DockStyle.Top;
            this.groupBox4.Location = new System.Drawing.Point(4, 81);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(550, 65);
            this.groupBox4.TabIndex = 5;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Preview";
            // 
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.Controls.Add(this.checkBoxVertexWeight);
            this.flowLayoutPanel1.Controls.Add(this.checkBoxVertexBadTriangles);
            this.flowLayoutPanel1.Controls.Add(this.checkBoxVertexNewTriangles);
            this.flowLayoutPanel1.Controls.Add(this.checkBoxVertexOrthoballs);
            this.flowLayoutPanel1.Controls.Add(this.checkBoxTriangulationWeights);
            this.flowLayoutPanel1.Controls.Add(this.checkBoxTriangulationOrthoballs);
            this.flowLayoutPanel1.Controls.Add(this.checkBoxTriangulationVertices);
            this.flowLayoutPanel1.Controls.Add(this.checkBoxTriangulationEdges);
            this.flowLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLayoutPanel1.Location = new System.Drawing.Point(3, 16);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(544, 46);
            this.flowLayoutPanel1.TabIndex = 0;
            // 
            // checkBoxTriangulationVertices
            // 
            this.checkBoxTriangulationVertices.AutoSize = true;
            this.checkBoxTriangulationVertices.Checked = true;
            this.checkBoxTriangulationVertices.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBoxTriangulationVertices.Location = new System.Drawing.Point(3, 26);
            this.checkBoxTriangulationVertices.Name = "checkBoxTriangulationVertices";
            this.checkBoxTriangulationVertices.Size = new System.Drawing.Size(64, 17);
            this.checkBoxTriangulationVertices.TabIndex = 7;
            this.checkBoxTriangulationVertices.Text = "Vertices";
            this.checkBoxTriangulationVertices.UseVisualStyleBackColor = true;
            this.checkBoxTriangulationVertices.CheckedChanged += new System.EventHandler(this.checkBoxChanged);
            // 
            // checkBoxTriangulationEdges
            // 
            this.checkBoxTriangulationEdges.AutoSize = true;
            this.checkBoxTriangulationEdges.Checked = true;
            this.checkBoxTriangulationEdges.CheckState = System.Windows.Forms.CheckState.Checked;
            this.checkBoxTriangulationEdges.Location = new System.Drawing.Point(73, 26);
            this.checkBoxTriangulationEdges.Name = "checkBoxTriangulationEdges";
            this.checkBoxTriangulationEdges.Size = new System.Drawing.Size(56, 17);
            this.checkBoxTriangulationEdges.TabIndex = 8;
            this.checkBoxTriangulationEdges.Text = "Edges";
            this.checkBoxTriangulationEdges.UseVisualStyleBackColor = true;
            this.checkBoxTriangulationEdges.CheckedChanged += new System.EventHandler(this.checkBoxChanged);
            // 
            // DebuggerForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.ClientSize = new System.Drawing.Size(558, 761);
            this.Controls.Add(this.splitContainer1);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.groupBox1);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.SizableToolWindow;
            this.Name = "DebuggerForm";
            this.Padding = new System.Windows.Forms.Padding(4);
            this.ShowIcon = false;
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "Delaunay Debugger";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.LiveSolverForm_FormClosing);
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.LiveSolverForm_FormClosed);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar1)).EndInit();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBoxTimer.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.dataGridView1)).EndInit();
            this.splitContainer1.Panel1.ResumeLayout(false);
            this.splitContainer1.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.splitContainer1)).EndInit();
            this.splitContainer1.ResumeLayout(false);
            this.groupBox4.ResumeLayout(false);
            this.flowLayoutPanel1.ResumeLayout(false);
            this.flowLayoutPanel1.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Button buttonReset;
        private System.Windows.Forms.Button buttonInsertOne;
        private System.Windows.Forms.Button buttonInsertAll;
        private System.Windows.Forms.CheckBox checkBoxVertexBadTriangles;
        private System.Windows.Forms.CheckBox checkBoxVertexWeight;
        private System.Windows.Forms.CheckBox checkBoxVertexOrthoballs;
        private System.Windows.Forms.TextBox textBoxTriangulationInfo;
        private System.Windows.Forms.CheckBox checkBoxTriangulationOrthoballs;
        private System.Windows.Forms.CheckBox checkBoxTriangulationWeights;
        private System.Windows.Forms.CheckBox checkBoxVertexNewTriangles;
        private System.Windows.Forms.GroupBox groupBoxTimer;
        private System.Windows.Forms.SplitContainer splitContainer1;
        private System.Windows.Forms.DataGridView dataGridView1;
        private System.Windows.Forms.Button buttonInsertRange;
        private System.Windows.Forms.TrackBar trackBar1;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.CheckBox checkBoxTriangulationVertices;
        private System.Windows.Forms.CheckBox checkBoxTriangulationEdges;
    }
}