using SuperDelaunay.GH;
using SuperDelaunay;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Rhino.Geometry; 

namespace SuperDelaunay.UI
{
    public partial class DebuggerForm : Form
    {
        #region Fields
        private Point3d[] _points;
        private double[] _weights;
        private Plane _plane;
        private int _currentIndex;
        private MouseCallback _mouse;
        private Conduit _conduit;
        #endregion

        #region Properties
        public Comp_Delaunay2D Component { get; private set; }
        public Delaunay2d Instance { get { return Component.DelaunayInstance; } }
        public bool HasInstance { get { return Instance != null; } }

        public bool ShowNextVertexWeight { get { return checkBoxVertexWeight.Checked; } }
        public bool ShowNextVertexBadTriangles { get { return checkBoxVertexBadTriangles.Checked; } }
        public bool ShowNextVertexNewTriangles { get { return checkBoxVertexNewTriangles.Checked; } }
        public bool ShowNextVertexOrthoballs { get { return checkBoxVertexOrthoballs.Checked; } }
        public bool ShowTriangulationWeights { get { return checkBoxTriangulationWeights.Checked; } }
        public bool ShowTriangulationOrthoballs { get { return checkBoxTriangulationOrthoballs.Checked; } }
        public bool ShowTriangulationVertices { get { return checkBoxTriangulationVertices.Checked; } }
        public bool ShowTriangulationEdges { get { return checkBoxTriangulationEdges.Checked; } }
        #endregion

        #region Constructors
        public DebuggerForm()
        {
            InitializeComponent();
            Focus();
            _mouse = new MouseCallback(this);
            _mouse.Enabled = true;
            _conduit = new Conduit(this);
            _conduit.Enabled = true;
        }
        public DebuggerForm(Comp_Delaunay2D comp) : this()
        {
            Component = comp;
            Reset(); 
        }
        #endregion

        #region Methods
        public void InsertPoints(List<Point3d> points, List<double> weights)
        {
            if(!IsSameInstance(points, weights))
            {
                _points = new Point3d[points.Count];
                _weights = new double[points.Count];
                var minW = double.MaxValue;
                var maxW = double.MinValue;
                for (int i = 0; i < points.Count; i++)
                {
                    var weight = weights[i];
                    _points[i] = points[i];
                    _weights[i] = weights[i];
                    if (weight < minW)
                        minW = weight;
                    if (weight > maxW)
                        maxW = weight;
                }
                if((maxW-minW) != 0.0)
                    Array.Sort(_weights, _points); 
                _plane = Instance.Plane.IsValid ? Instance.Plane : Utils.FitPlane(points);
                trackBar1.Minimum = 1;
                trackBar1.Maximum = _points.Length;
                trackBar1.Value = 1; 
            }
            UpdateInfo();
        }
        public bool IsSameInstance(List<Point3d> points, List<double> weights)
        {
            if (_points == null || _points.Length != points.Count)
                return false;
            var pts = points.ToArray();
            var wgs = weights.ToArray();
            Array.Sort(wgs, pts);
            for (int i = 0; i < pts.Length; i++)
            {
                if (_points[i] != pts[i])
                    return false;
                if (_weights[i] != wgs[i])
                    return false;
            }
            return true;
        }
       
        public void ExpirePreview()
        {
            Component.ExpirePreview(true);
        }
        public void ExpireSolution()
        {
            Component.ExpireSolution(true);
        }

        public Vertex CurrentVertex()
        {
            if (!HasInstance || _points == null || _currentIndex >= _points.Length)
                return null;

            var pt = _points[_currentIndex];
            _plane.ClosestParameter(pt, out double s, out double t);
            return new Vertex(_currentIndex, new Point2d(s, t), _weights[_currentIndex], pt);
        }
      
        public void Reset()
        {
            buttonInsertAll.Enabled = buttonInsertOne.Enabled = buttonInsertRange.Enabled = true;
            _currentIndex = 0;
            Component.DelaunayInstance = null;
            UpdateInfo();
            Rhino.RhinoDoc.ActiveDoc.Views.Redraw();
        } 
        public void Ended()
        {
            buttonInsertAll.Enabled = buttonInsertOne.Enabled = buttonInsertRange.Enabled = false;
        }
        
        public void InsertOne()
        {
            if (!HasInstance)
                return;
            Instance.Insert(_points[_currentIndex],_weights[_currentIndex] );
            _currentIndex++;
            UpdateInfo();
            if (_currentIndex >= _points.Length)
                Ended();
        }
        public void InsertAll()
        {
            if (!HasInstance)
                return;
            Instance.Insert(_points.ToArray(), _weights.ToArray());
            UpdateInfo();
            Ended();
        }
        public void InsertRange()
        {
            if (!HasInstance)
                return;
            var rem = _points.Length - _currentIndex;
            var count = Math.Min(rem, trackBar1.Value); 
            Instance.Insert(
                _points.Skip(_currentIndex).Take(count).ToArray(), 
                _weights.Skip(_currentIndex).Take(count).ToArray());
            _currentIndex += count;
            UpdateInfo();
            if (_currentIndex >= _points.Length)
                Ended();
        }
   
        public void UpdateTriangulationInfo()
        {
            var sb = new StringBuilder();
            if (HasInstance)
            {
                sb.AppendLine($"Next vertex:");
                sb.AppendLine("  " + CurrentVertex()?.ToInfo());
                sb.AppendLine(); 
                sb.AppendLine($"Vertices: {Instance.VerticesCount}");
                foreach (var v in Instance.Vertices)
                    sb.AppendLine("  " + v.ToInfo());
                sb.AppendLine();
                sb.AppendLine($"Triangles: {Instance.TrianglesCount}");
                foreach (var t in Instance.Triangles)
                    sb.AppendLine("  " + t.ToInfo());
                sb.AppendLine();
                var infVert = Instance.GetAllVertices().Except(Instance.Vertices).ToArray();
                sb.AppendLine($"Infinite vertices: {infVert.Length}");
                foreach (var v in infVert)
                    sb.AppendLine("  " + v.ToInfo());
                sb.AppendLine();
                var infTri = Instance.GetAllTriangles().Except(Instance.Triangles).ToArray();
                sb.AppendLine($"Infinite triangles: {infTri.Length}");
                foreach (var v in infTri)
                    sb.AppendLine("  " + v.ToInfo());
            }
            
            textBoxTriangulationInfo.Text = sb.ToString();
        }
        public void UpdateTimer()
        {
            dataGridView1.DataSource = null;
            dataGridView1.Columns.Clear();
            dataGridView1.Rows.Clear();
            if (!HasInstance || Instance._timer == null)
                return;
            dataGridView1.DataSource = Instance._timer.ToDataTable( );
        }
        public void UpdateInfo()
        {
            if (_points != null)
                label1.Text = $"{_points.Length - _currentIndex} remaining vertices.";
            else
                label1.Text = "";
            UpdateTriangulationInfo();
            UpdateTimer();
        }

        public IEnumerable<Triangle> GetSelectedTriangles()
        {
            return _mouse.SelectedTriangles;
        }
        #endregion
         
        #region Handlers
        #region Buttons
        private void buttonReset_Click(object sender, EventArgs e)
        {
            Reset();
            UpdateInfo();
            ExpireSolution();
            if (HasInstance)
            {
                var bb = new Box(Plane.WorldXY, _points);
                bb.MakeValid();
                Instance.SetInfiniteTriangulation(Plane.WorldXY, bb); 
                UpdateInfo();
            }
            Focus();
        }

        private void buttonInsertOne_Click(object sender, EventArgs e)
        {
            InsertOne();
            ExpireSolution();
            Focus();
        }

        private void buttonInsertAll_Click(object sender, EventArgs e)
        {
            InsertAll();
            ExpireSolution();
            Focus();
        }

        private void buttonInsertRange_Click(object sender, EventArgs e)
        {
            InsertRange();
            ExpireSolution();
            Focus();
        }

        private void trackBar1_Scroll(object sender, EventArgs e)
        {
            buttonInsertRange.Text = $"Insert {trackBar1.Value}";
        }
        private void checkBoxChanged(object sender, EventArgs e)
        {
            ExpirePreview();
        }
        #endregion
         
        private void LiveSolverForm_FormClosed(object sender, FormClosedEventArgs e)
        { 
            ExpireSolution();
        }
        private void LiveSolverForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            Component.Controller = null;
            _mouse.Enabled = false;
            _conduit.Enabled = false; 
        }
        #endregion 

        public class MouseCallback : Rhino.UI.MouseCallback
        {
            private DebuggerForm _form;
            public List<Triangle> SelectedTriangles { get; set; }
            
            public MouseCallback(DebuggerForm form) : base()
            {
                _form = form;
                SelectedTriangles = new List<Triangle>();
            }

            protected override void OnMouseMove(Rhino.UI.MouseCallbackEventArgs e)
            {
                //base.OnMouseMove(e);
                var cnt = SelectedTriangles.Count;
                SelectedTriangles.Clear();
                var del = _form.Instance;
                if (del == null)
                    return;
             
                 var line = e.View.ActiveViewport.ClientToWorld(e.ViewportPoint); 
             
                var ts = del.GetAllTriangles().ToArray();
             
                foreach (var t in ts) 
                {
                   
                    var pln = t.GetPlane();
                    Rhino.Geometry.Intersect.Intersection.LinePlane(line, pln, out double lt);
                    var pt = pln.ClosestPoint(line.PointAt(lt));
                    if (Utils.IsPointInTriangle(t.A.Position3d, t.B.Position3d, t.C.Position3d, pt))
                    {
                        SelectedTriangles.Add(t); 
                        break;
                    }
                       
                }
                if (SelectedTriangles.Count > 0 || SelectedTriangles.Count != cnt)
                    e.View.Redraw();
            } 
        }

        public class Conduit : Rhino.Display.DisplayConduit
        {
            private DebuggerForm _form;
            public Conduit(DebuggerForm form) : base()
            {
                _form = form;
            }
            protected override void CalculateBoundingBox(Rhino.Display.CalculateBoundingBoxEventArgs e)
            {
                base.CalculateBoundingBox(e);
                if (!_form.HasInstance)
                    return;
               e.IncludeBoundingBox(_form.Instance.BoundingBox.BoundingBox);
            }
            protected override void DrawOverlay(Rhino.Display.DrawEventArgs e)
            {
                base.DrawOverlay(e);

                _form.Component.Draw(e.Display, Color.Blue);
            }
        }

    }
}
