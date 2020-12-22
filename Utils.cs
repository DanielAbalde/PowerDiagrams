using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace SuperDelaunay
{
    public static class Utils
    {
        public static double RhinoTolerance
        {
            get
            {
                var doc = Rhino.RhinoDoc.ActiveDoc;
                if (doc != null)
                    return doc.ModelAbsoluteTolerance;
                return 1e-4;
            }
        }
        public static void Print(object obj)
        {
            Rhino.RhinoApp.WriteLine(obj.ToString());
        }
        public static void Print(string format, params string[] args)
        {
            Rhino.RhinoApp.WriteLine(string.Format(format, args));
        }
        public static void Bake(object obj)
        {
            if (obj == null)
                return;
            if (obj is GeometryBase g)
            {
                Rhino.RhinoDoc.ActiveDoc.Objects.Add(g);
            }
            else if (obj is Point3d p)
            {
                Rhino.RhinoDoc.ActiveDoc.Objects.AddPoint(p);
            }
            else if (obj is Line ln)
            {
                Rhino.RhinoDoc.ActiveDoc.Objects.AddLine(ln);
            }
            else if (obj is Box bb)
            {
                Rhino.RhinoDoc.ActiveDoc.Objects.AddBox(bb);
            }
            else if (obj is Polyline pln)
            {
                Rhino.RhinoDoc.ActiveDoc.Objects.AddPolyline(pln);
            }
            else if (obj is Triangle t)
            {
                Rhino.RhinoDoc.ActiveDoc.Objects.AddPolyline(t.ToPolyline());
            }
            else if (obj is Edge e)
            {
                Rhino.RhinoDoc.ActiveDoc.Objects.AddLine(e.ToLine());
            }
            else if (obj is Vertex v)
            {
                Rhino.RhinoDoc.ActiveDoc.Objects.AddPoint(v.Position3d);
            }
            else if (obj is System.Collections.IEnumerable c)
            {
                foreach (var o in c)
                    Bake(o);
            }
            else
            {
                throw new NotImplementedException();
            }
            Rhino.RhinoDoc.ActiveDoc.Views.Redraw();
        }
        public static IEnumerable<T> GetEnumValues<T>()
        {
            return Enum.GetValues(typeof(T)).Cast<T>();
        }
        public static IEnumerable<T> Repeat<T>(T data, int times)
        {
            for (int i = 0; i < times; i++)
                yield return data;
        }
        public static List<T> ShiftList<T>(List<T> list, int offset, bool wrap = true)
        {
            if (list == null)
                throw new ArgumentNullException("list is null");
            int count = list.Count;
            if (count == 0)
                throw new ArgumentException("list is empty");
            if (offset == 0)
                return list;

            int start = offset;
            if (start < 0)
                while (start < 0)
                    start += count;
            else if (start >= count)
                while (start >= count)
                    start -= count;

            int size = count;
            if (!wrap)
                size -= start;

            List<T> shiftedList = new List<T>();
            for (int i = start; i < start + size; i++)
                shiftedList.Add(list[i % count]);

            return shiftedList;
        }
        public static Plane FitPlane(IEnumerable<Point3d> points, bool averageOrigin = false)
        {
            if (points == null || !points.Any())
                return Plane.Unset;
             
            var pts = points.Where(p => p.IsValid).ToArray();

            if (pts.Length == 1)
            {
                if (averageOrigin)
                    return new Plane(pts[0], Vector3d.ZAxis);
                else
                    return Plane.WorldXY;
            }
            else if (pts.Length == 2)
            {
                if (averageOrigin)
                    return new Plane((pts[0] + pts[1]) / 2.0, Vector3d.ZAxis);
                else
                    return Plane.WorldXY;
            }
            else if (pts.Length == 3)
            {
                var pln = new Plane(pts[0], pts[1], pts[2]);
                if (averageOrigin)
                    pln.Origin = (pts[0] + pts[1] + pts[2]) / 2.0;
                return pln;
            }

            if (Plane.FitPlaneToPoints(pts, out Plane plane) == PlaneFitResult.Failure)
                return Plane.Unset;

            var angTol = 0.0017453292519943296;
            if (plane.ZAxis.IsParallelTo(Vector3d.ZAxis, angTol) != 0)
            {
                plane = new Plane(plane.Origin, Vector3d.ZAxis);
            }
            else
            {
                if (plane.ZAxis.IsParallelTo(Vector3d.YAxis, angTol) != 0)
                {
                    plane = new Plane(plane.Origin, Vector3d.YAxis);
                }
                else
                {
                    if (plane.ZAxis.IsParallelTo(Vector3d.XAxis, angTol) != 0)
                    {
                        plane = new Plane(plane.Origin, Vector3d.XAxis);
                    }
                }
            }

            if (averageOrigin)
            {
                plane.Origin = Centroid(pts);
            }

            return plane;
        }
        public static Point3d Centroid(IEnumerable<Point3d> points)
        {
            if (points == null || !points.Any())
                return Point3d.Unset;

            Point3d center = new Point3d();
            foreach (Point3d pt in points)
                center += pt;
            center /= points.Count();
            return center;
        }
        public static bool IsPointInTriangle(Point3d pa, Point3d pb, Point3d pc, Point3d pt)
        {
            bool SameSide(Point3d p1, Point3d p2, Point3d pta, Point3d ptb)
            {
                return Vector3d.CrossProduct(ptb - pta, p1 - pta) * Vector3d.CrossProduct(ptb - pta, p2 - pta) >= 0;
            }

            return SameSide(pt, pa, pb, pc) && SameSide(pt, pc, pa, pb) && SameSide(pt, pb, pc, pa)
                 && Math.Abs((pa - pt) * Vector3d.CrossProduct(pa - pb, pa - pc)) <= 0;
        }
        public static Point3d Orthocenter(Point3d pa, Point3d pb, Point3d pc, double wa = 0.0, double wb = 0.0, double wc = 0.0)
        {
            var vab = pb - pa;
            var dab = vab.Length;
            vab.Unitize();
            var vac = pc - pa;
            var dac = vac.Length;
            vac.Unitize();
            var normal = Vector3d.CrossProduct(vab, vac);
            var perpab = Vector3d.CrossProduct(vab, normal);
            var xab = (dab + (wa * wa - wb * wb) / dab) / 2.0;
            var lab = new Line(pa + vab * xab, perpab, 1);
            var perpac = Vector3d.CrossProduct(vac, normal);
            var xac = (dac + (wa * wa - wc * wc) / dac) / 2.0;
            var lac = new Line(pa + vac * xac, perpac, 1);
            Rhino.Geometry.Intersect.Intersection.LineLine(lab, lac, out double ta, out _);
            return lab.PointAt(ta);
        }
        public static Point3d SnapPoint(Point3d point, double snapping)
        {
            return new Point3d(
                snapping * Math.Round(point.X / snapping),
                snapping * Math.Round(point.Y / snapping),
                snapping * Math.Round(point.Z / snapping));
        }
        public static Point2d SnapPoint(Point2d point, double snapping)
        {
            return new Point2d(
                snapping * Math.Round(point.X / snapping),
                snapping * Math.Round(point.Y / snapping));
        }
        public static bool IsCounterClockwise(Point3d a, Point3d b, Point3d reference, Vector3d normal)
        {
            return normal * Vector3d.CrossProduct(a - reference, b - reference) > 0;
        }
        public static T[] SortByRadial<T>(T[] objects, Plane plane, Func<T, Point3d> objectToPoint)
        {
            double[] angles0 = new double[objects.Length];
            double pi2 = 2 * Math.PI;
            for (int i = 0; i < objects.Length; i++)
            {
                plane.ClosestParameter(objectToPoint(objects[i]), out double u, out double v);
                angles0[i] = (Math.Atan2(v, u) + pi2) % pi2;
            }
            Array.Sort(angles0, objects);
            return objects;
        }

        public static double TriangleQuality(Point3d pa, Point3d pb, Point3d pc)
        {
            var ab = pb - pa;
            var ac = pc - pa;
            var bc = pc - pb;
            var aveLen = (ab.Length + ac.Length + bc.Length) / 3.0;
            var idealArea = 0.5 * (aveLen * aveLen);
            var realArea = 0.5 * Vector3d.CrossProduct(ab, ac).Length;
            return realArea / idealArea;
        }
        
        public static bool LineLineIntersects(Line line0, Line line1)
        {
            return LineLineIntersects(line0, line1, out _, out _);


        }
        public static bool LineLineIntersects(Line line0, Line line1, out double t0, out double t1)
        {
            t0 = t1 = double.NaN;
            var v0 = line0.Direction;
            var v1 = line1.Direction;
            var v2 = line0.From - line1.From;

            var d21 = v2 * v1;
            var d10 = v1 * v0;
            var d20 = v2 * v0;
            var d11 = v1 * v1;
            var d00 = v0 * v0;

            var denom = d00 * d11 - d10 * d10;
            if (Math.Abs(denom) < 1e-12)
                return false;

            var numer = d21 * d10 - d20 * d11;
            t0 = numer / denom;
            t1 = (d21 + d10 * t0) / d11;
            return (t0 >= 0.0 && t0 <= 1.0 && t1 >= 0.0 && t1 <= 1.0);

        }

        public static GH_Structure<GH_Integer> ToTopologyTree(IEnumerable<Edge> edges, int iteration)
        {
            var tree = new GH_Structure<GH_Integer>();
            if (edges == null)
                return tree;
            int i = 0;
            foreach (var e in edges)
            {
                tree.AppendRange(new GH_Integer[] { new GH_Integer(e.A.Index), new GH_Integer(e.B.Index) }, new GH_Path(iteration, i));
                i++;
            }
            return tree;
        }
        public static GH_Structure<GH_Integer> ToTopologyTree(IEnumerable<Triangle> triangles, int iteration)
        {
            var tree = new GH_Structure<GH_Integer>();
            if (triangles == null)
                return tree;
            int i = 0;
            foreach (var t in triangles)
            {
                tree.AppendRange(new GH_Integer[] { new GH_Integer(t.A.Index), new GH_Integer(t.B.Index), new GH_Integer(t.C.Index) }, new GH_Path(iteration, i));
                i++;
            }
            return tree;
        }
        public static GH_Structure<GH_Integer> ToTopologyTree(IEnumerable<IEnumerable<Vertex>> vertices, int iteration)
        {
            var tree = new GH_Structure<GH_Integer>();
            if (vertices == null)
                return tree;
            int i = 0;
            foreach (var vs in vertices)
            {
                tree.AppendRange(vs.Select(v => new GH_Integer(v.Index)), new GH_Path(iteration, i));
                i++;
            }
            return tree;
        }
        public static GH_Structure<GH_Integer> ToTopologyTree(IEnumerable<IEnumerable<Edge>> edges, int iteration)
        {
            var tree = new GH_Structure<GH_Integer>();
            if (edges == null)
                return tree;
            int i = 0;

            foreach (var es in edges)
            {
                int j = 0;
                foreach (var e in es)
                {
                    tree.AppendRange(new GH_Integer[] { new GH_Integer(e.A.Index), new GH_Integer(e.B.Index) },
                        new GH_Path(iteration, i, j));

                    j++;
                }
                i++;
            }
            return tree;
        }


    }
     
    /// <summary>
    /// Meassure the runtime using nested meassurements
    /// </summary>
    /// <remarks>This class belongs to Peacock.dll and is still under development, so it is marked as internal.</remarks>
    /// <example>
    /// <code>
    /// var rt = new RunningTime("Test");
    /// rt.Start();
    /// rt.Start("First Section");
    /// // Do something.
    /// rt.Start("Second Section");
    /// rt.Start("Second Section", "Subsection A");
    /// // Do something.
    /// rt.End("Second Section", "Subsection A");
    /// rt.Start("Second Section", "Subsection B");
    /// // Do something.
    /// rt.End("Second Section", "Subsection B");
    /// rt.End("Second Section");
    /// rt.End("First Section");
    /// rt.End();
    /// </code>
    /// </example>
    internal class RunningTime
    {
        #region Fields  
        private List<DateTime> _starts;
        private List<DateTime> _ends;
        private List<TimeSpan> _ellapsed;
        private Dictionary<string, RunningTime> _internalSections;
        #endregion

        #region Properties
        public string Name { get; private set; }
        public List<string> SectionNames { get { return _internalSections.Keys.ToList(); } }
        public List<RunningTime> Sections { get { return _internalSections.Values.ToList(); } }
        public bool IsRunning { get; private set; }
        public RunningTime Owner { get; private set; }
        public int NestedLevel
        {
            get
            {
                int level = 0;
                var o = Owner;
                while (o != null)
                {
                    level++;
                    o = o.Owner;
                }
                return level;
            }
        }
        public string FullName
        {
            get
            {
                var fn = Name;
                var owner = Owner;
                while (owner != null)
                {
                    fn = string.Concat(owner.Name, "-", fn);
                    owner = owner.Owner;
                }
                return fn;
            }
        }
        public bool IsLastOfOwner
        {
            get
            {
                return Owner == null || Owner.SectionNames.IndexOf(Name) == Owner.SectionNames.Count - 1;
            }
        }
        #endregion

        #region Constructors 
        public RunningTime(string name = "")
        {
            Name = string.IsNullOrEmpty(name) ? "Unnamed" : name;
            _starts = new List<DateTime>();
            _ends = new List<DateTime>();
            _ellapsed = new List<TimeSpan>();
            _internalSections = new Dictionary<string, RunningTime>();
        }
        public RunningTime(RunningTime other)
        {
            Name = other.Name;
            _starts = new List<DateTime>(other._starts);
            _ends = new List<DateTime>(other._ends);
            _ellapsed = new List<TimeSpan>(other._ellapsed);
            _internalSections = new Dictionary<string, RunningTime>(other._internalSections);
            Owner = other.Owner;
        }
        #endregion

        #region Methods 
        public static RunningTime GetSection(RunningTime timer, bool createIfNotExists, bool throwException, params string[] section)
        {

            if (section == null || section.Length == 0 || string.IsNullOrEmpty(section.FirstOrDefault()))
                return timer;
            var name = section.FirstOrDefault();
            if (timer._internalSections.ContainsKey(name))
            {
                if (section.Length < 2)
                    return timer._internalSections[name];
                return GetSection(timer._internalSections[name], createIfNotExists, throwException, section.Skip(1).ToArray());
            }
            else
            {
                if (createIfNotExists)
                {
                    var parent = timer;
                    foreach (var s in section)
                    {
                        var child = new RunningTime(s);
                        child.Owner = parent;
                        parent._internalSections.Add(s, child);
                        parent = child;
                    }
                    return parent;
                }
                else
                {
                    if (throwException)
                    {
                        throw new ArgumentException("section", string.Format("\"{0}\" doesn't exists.", string.Join(" - ", section)));
                    }
                    else
                    {
                        return null;
                    }

                }
            }
        }

        public void Reset(params string[] section)
        {
            var s = GetSection(this, true, true, section);
            s._starts.Clear();
            s._ends.Clear();
            s._ellapsed.Clear();
            foreach (var r in s._internalSections.Values)
                r.Reset();
        }

        public void Start(params string[] section)
        {
            var s = GetSection(this, true, true, section);
            if (!s.IsRunning)
            {
                s._starts.Add(DateTime.Now);
                s.IsRunning = true;
            }
        }
        public void End(params string[] section)
        {
            var s = GetSection(this, false, true, section);

            if (!s.IsRunning)
                return;
            foreach (var r in s._internalSections.Values)
                r.End();
            s._ends.Add(DateTime.Now);
            s._ellapsed.Add(s._ends[s._ends.Count - 1] - s._starts[s._starts.Count - 1]);
            s.IsRunning = false;
        }

        public void AddProcess(Action action, params string[] section)
        {
            Start(section);
            action();
            End(section);
        }

        private void Traverse(Action<RunningTime> action)
        {
            action(this);
            foreach (var c in _internalSections.Values)
                c.Traverse(action);
        }


        #region Result
        public List<TimeSpan> GetSpans(params string[] section)
        {
            var s = GetSection(this, false, true, section);
            var spans = new List<TimeSpan>(s._ellapsed);
            if (s.IsRunning && s._starts.Count > s._ends.Count)
            {
                spans.Add(DateTime.Now - s._starts[s._starts.Count - 1]);
            }
            return spans;
        }
        public double[] GetResult(int decimals = 4, params string[] section)
        {
            var min = double.MaxValue;
            var max = double.MinValue;
            var ave = 0.0;
            var spans = GetSpans(section);
            if (spans.Count == 0)
                return new double[] { 0, 0, 0, 0, 0 };
            if (spans.Count == 1)
            {
                var ms = (double)spans[0].Milliseconds;
                if (ms == 0)
                    ms = spans[0].TotalMilliseconds;
                return new double[] { ms, ms, ms, ms, 1 }; 
            }
             
            foreach (var ts in spans)
            {
                var ms = (double)ts.Milliseconds;
                if (ms == 0)
                    ms = ts.TotalMilliseconds;
                if (ms < min)
                    min = ms;
                if (ms > max)
                    max = ms;
                ave += ms;
            }
            var tot = ave;
            ave /= spans.Count;
            return new double[] {
        Math.Round(tot, decimals),
        Math.Round(ave, decimals),
        Math.Round(min, decimals),
        Math.Round(max, decimals),
        spans.Count };
        }

        public System.Data.DataTable ToDataTable(bool indent = true)
        {
            var t = new System.Data.DataTable(Name);
            t.Columns.Add("Name", typeof(string));
            t.Columns.Add("Total ms", typeof(double));
            t.Columns.Add("Average ms", typeof(double));
            t.Columns.Add("Min ms", typeof(double));
            t.Columns.Add("Max ms", typeof(double));
            t.Columns.Add("Count", typeof(int));
            Traverse(r =>
            {
                var rs = r.GetFormated(indent);
                t.Rows.Add(rs.Select(d => {
                    if (d is double dd)
                        return Math.Round(dd, 3);
                    else
                        return d;
                }).ToArray());
            });
            return t;
        }
        public string GetNestedSectionNames(bool indent = true)
        {
            var sb = new System.Text.StringBuilder();
            Traverse(s => sb.AppendLine(s.GetNestedSectionName(indent)));
            return sb.ToString();
        }
        private static string GetNestedSymbol(RunningTime timer)
        {
            var sym = "";
            if (timer.Owner == null)
                return sym;
            sym = " ";
            var owner = timer.Owner;
            if (timer.IsLastOfOwner)
                sym = string.Concat(sym, "└─ ");
            else
                sym = string.Concat(sym, "├─ ");

            while (owner != null)
            {
                if (owner.Owner == null)
                    break;
                if (owner.IsLastOfOwner)
                    sym = string.Concat("   ", sym);
                else
                    sym = string.Concat(" │ ", sym);
                owner = owner.Owner;
            }

            return sym;
        }
        private string GetNestedSectionName(bool indent = true)
        {
            if (indent)
            { 
                return string.Concat( GetNestedSymbol(this), Name);
            }
            else
            {
                return FullName;
            }

        }
        public object[] GetFormated(bool indent, params string[] section)
        {
            var s = GetSection(this, false, true, section);
            var n = s.GetNestedSectionName(indent);
            var r = s.GetResult();
            return new object[] { n, r[0], r[1], r[2], r[3], r[4] };
        }

        public string GetFormatedDetailedMilliseconds(bool indent = true)
        {
            var log = new List<object[]>();
            Traverse(r => log.Add(r.GetFormated(indent)));
            var sb = new System.Text.StringBuilder();
            sb.AppendLine(string.Format(
            "{0,-20} | {1,-10} | {2,-10} | {3,-10} | {4,-10} | {5,-5}",
            "Name", "Total ms", "Ave ms", "Min ms", "Max ms", "Count"));
            foreach (var l in log)
                sb.AppendLine(string.Format(
                  "{0,-20} | {1,-10} | {2,-10} | {3,-10} | {4,-10} | {5,-5}",
                  l[0], l[1], l[2], l[3], l[4], l[5]));
            return sb.ToString();
        }
        public override string ToString()
        {
            return GetFormatedDetailedMilliseconds();
        }
        #endregion

        #endregion
    }
}
