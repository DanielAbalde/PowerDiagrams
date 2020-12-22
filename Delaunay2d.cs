using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using static SuperDelaunay.Utils;

namespace SuperDelaunay
{
    /// <summary>
    /// Weighted delaunay triangulation in 2d, composed by <see cref="Vertex"/> and <see cref="Triangle"/>.
    /// </summary> 
    /// <remarks>Using a modified version of the Bowyer-Watson algorithm.</remarks>
    /// <example>
    /// <code>
    /// <![CDATA[
    /// var D = new Delaunay2D(points, weights, Plane.WorldXY);
    /// var edges = D.GetEdges(out IEnumerable<Line> lines);
    /// var voronoi = D.ToVoronoi(curveBounds); 
    /// ]]>
    /// </code>
    /// </example>
    public class Delaunay2d
    {
        #region Fields
        private HashSet<Vertex> _vertices;
        private HashSet<Triangle> _triangles;
        private HashSet<Vertex> _infiniteVertices;
        private HashSet<Triangle> _infiniteTriangles;
        private Plane _plane;
        private Box _boundingBox;
        private Box _infiniteBoundingBox; 
        private DelaunayMetric _metric;
        private bool _weighted;
        #endregion

        #region Properties
        /// <summary>
        /// Vertices of this Delaunay.
        /// </summary>
        /// <remarks>This list does not reference the original internal list to avoid modifying it from outside.</remarks>
        public List<Vertex> Vertices
        {
            get
            {
                return _vertices.ToList();
            }
        }
        /// <summary>
        /// Number of vertices.
        /// </summary>
        public int VerticesCount
        {
            get
            {
                return _vertices.Count;
            }
        }
        /// <summary>
        /// Triangles of this Delaunay.
        /// </summary>
        /// <remarks>This list does not reference the original internal list to avoid modifying it from outside.</remarks>
        public List<Triangle> Triangles
        {
            get
            {
                return _triangles.ToList();
            }
        }
        /// <summary>
        /// Number of triangles.
        /// </summary>
        public int TrianglesCount
        {
            get
            {
                return _triangles.Count;
            }
        }
        /// <summary>
        /// Delaunay plane.
        /// </summary>
        public Plane Plane { get { return _plane; } }
        /// <summary>
        /// Delaunay bounding box.
        /// </summary>
        public Box BoundingBox { get { return _boundingBox; } } 
        #endregion

        #region Constructors
        /// <summary>
        /// Create an empty instance.
        /// </summary>
        public Delaunay2d()
        {
            _vertices = new HashSet<Vertex>();
            _triangles = new HashSet<Triangle>();
            _infiniteVertices = new HashSet<Vertex>();
            _infiniteTriangles = new HashSet<Triangle>();
            _boundingBox = Box.Empty; 
        }
        /// <summary>
        /// Create an instance from points. All weights are zero and the plane is the best fit plane.
        /// </summary>
        /// <param name="points">Points to triangulate.</param>
        /// See <see cref="Delaunay2d(Point3d[], double[], Plane, DelaunayMetric)"/> to use the main constructor.
        public Delaunay2d(Point3d[] points) : this(points, points.Select(p => 0.0).ToArray(), FitPlane(points)) { }
        /// <summary>
        /// Create an instance from points and weights. The plane is the best fit plane.
        /// </summary>
        /// <param name="points">Points to triangulate.</param>
        /// <param name="weights">Weights for each point.</param>
        /// See <see cref="Delaunay2d(Point3d[], double[], Plane, DelaunayMetric)"/> to use the main constructor.
        public Delaunay2d(Point3d[] points, double[] weights) : this(points, weights, FitPlane(points)) { }
        /// <summary>
        /// Create an instance using weighted points and plane.
        /// </summary>
        /// <param name="points">Points to triangulate.</param>
        /// <param name="weights">Weights for each point.</param>
        /// <param name="plane">Plane to define the two-dimensionality.</param>
        /// <param name="metric">Experimental for debbugin. DELETE.</param>
        /// <exception cref="ArgumentNullException">Thrown when the list of points or weights or the plane is null.</exception>
        /// <exception cref="ArgumentException">Thrown when the number of points is not equal the number of weights.</exception>
        /// <exception cref="ArgumentException">Thrown when the number of points is less than 3.</exception>
        public Delaunay2d(Point3d[] points, double[] weights, Plane plane) : this(points, weights, plane, DelaunayMetric.Power) { }
        internal Delaunay2d(Point3d[] points, double[] weights, Plane plane, DelaunayMetric metric = DelaunayMetric.Power)
        {
            if (points == null)
                throw new ArgumentNullException(nameof(points));
            if (points.Length < 3)
                throw new ArgumentException("points.Length must be >= 3.");
            if (weights == null)
                throw new ArgumentNullException(nameof(weights));
            if (points.Length != weights.Length)
                throw new ArgumentException("The number of points must be equal the number of weights.");
            if (plane == Plane.Unset || !plane.IsValid)
                throw new ArgumentNullException("plane is unset or invalid.");

            _vertices = new HashSet<Vertex>();
            _triangles = new HashSet<Triangle>();
            _infiniteVertices = new HashSet<Vertex>();
            _infiniteTriangles = new HashSet<Triangle>();
            _plane = plane;
            _boundingBox = Box.Empty; 
            _metric = metric;
            Insert(points, weights);

        }
        /// <summary>
        /// Create one instance by copying another.
        /// </summary>
        /// <param name="other">Another non-null instance to be cloned.</param>
        /// <remarks>Use this constructor to duplicate this class.</remarks>
        public Delaunay2d(Delaunay2d other)
        {
            if (other == null)
                throw new ArgumentNullException("other");
            _vertices = new HashSet<Vertex>();
            _triangles = new HashSet<Triangle>();
            _plane = other._plane;
            _boundingBox = other._boundingBox;
            _infiniteBoundingBox = other._infiniteBoundingBox; 
            _infiniteVertices = new HashSet<Vertex>();
            _infiniteTriangles = new HashSet<Triangle>(); 
            _metric = other._metric;
            var dic = new Dictionary<int, Vertex>();
            foreach (var v in other.GetAllVertices())
            {
                var n = new Vertex(v);
                if (!dic.ContainsKey(n.GetHashCode()))
                {
                    AddVertex(n);
                    dic.Add(v.GetHashCode(), n);
                }
                else
                {
                    throw new Exception("Duplicated vertex " + dic[n.GetHashCode()]);
                }

            }
            foreach (var t in other.GetAllTriangles())
            {
                var a = dic[t.A.GetHashCode()];
                var b = dic[t.B.GetHashCode()];
                var c = dic[t.C.GetHashCode()];
                AddTriangle(new Triangle(a, b, c));
            }
        }
        #endregion

        #region Methods
        #region Topology 
        private bool AddVertex(Vertex v)
        {
            if (v.IsInfinite)
                return _infiniteVertices.Add(v);
            else
                return _vertices.Add(v);
        }
        private bool RemoveVertex(Vertex v)
        {
            return _vertices.Remove(v) || _infiniteVertices.Remove(v);
        }
        private bool AddTriangle(Triangle t)
        {
            foreach (var v in t.Vertices)
                v.AddTriangle(t);
            if (t.IsInfinite)
                return _infiniteTriangles.Add(t);
            else
                return _triangles.Add(t);

        }
        private bool RemoveTriangle(Triangle t)
        {
            foreach (var v in t.Vertices)
                v.RemoveTriangle(t);
            if (t.IsInfinite)
            {
                return _infiniteTriangles.Remove(t);
            }
            else
            {
                return _triangles.Remove(t);
            }
        }
        /// <summary>
        /// Remove all vertices and triangles.
        /// </summary>
        public void Clear()
        {
            _vertices.Clear();
            _infiniteVertices.Clear();
            _triangles.Clear();
            _infiniteTriangles.Clear();
        } 
        #endregion

        #region Solver
        /// <summary>
        /// Inserts new weighted points to the triangulation.
        /// </summary>
        /// <param name="points">New points to be added.</param>
        /// <param name="weights">Weights per points</param> 
        /// <remarks>If the plane is unset or not valid, the best fit plane will be used.</remarks>
        /// <exception cref="ArgumentNullException">Thrown when the list of points or weights is null.</exception>
        /// <exception cref="ArgumentException">Thrown when the number of points is not equal the number of weights or when some point or weight is not valid.</exception>
        public void Insert(Point3d[] points, double[] weights)
        {
            if (points == null)
                throw new ArgumentNullException(nameof(points));
            if (weights == null)
                throw new ArgumentNullException(nameof(weights));
            if (points.Length != weights.Length)
                throw new ArgumentException("The number of points must be equal the number of weights.");

            //StartTimer();
            //StartTimer("Insert all");
            UpdatePlane(points);

            var vertices = new Vertex[points.Length];
            int vc = _vertices.Count;
            var minW = double.MaxValue;
            var maxW = double.MinValue;
            for (int i = 0; i < points.Length; i++)
            {
                var point = points[i]; 
                if(!point.IsValid)
                    throw new ArgumentException($"Point {i} is not valid.");
                var weight = weights[i];
                if(double.IsNaN(weight) || double.IsInfinity(weight))
                    throw new ArgumentException($"Weight {i} is not valid.");
                if (weight < minW)
                    minW = weight;
                if (weight > maxW)
                    maxW = weight;
                _plane.RemapToPlaneSpace(point, out Point3d pt);
                vertices[i] = new Vertex(vc + i, new Point2d(pt.X, pt.Y), weight, points[i]);
            }
            _weighted = (maxW - minW) != 0.0;
            UpdateBoundingBox(vertices);
            //EndTimer("Insert all");

            Triangulate(vertices);

            //EndTimer();
        }
        /// <summary>
        /// Inserts a new weighted point to the triangulation.
        /// <para>If needed, it updates the plane, bounding box, infinite vertices and triangles, so consider using <see cref="Insert(List{Point3d}, List{double})"/> if you want to include multiple points.</para>
        /// </summary>
        /// <param name="point">Vertex position.</param>
        /// <param name="weight">Vertex weight.</param> 
        /// <exception cref="ArgumentException"> Throw when point is not valid or when weight is NaN or infinity.</exception>
        public void Insert(Point3d point, double weight)
        {
            if (!point.IsValid)
                throw new ArgumentException("Point is not valid.");
            if(double.IsNaN(weight) || double.IsInfinity(weight))
                throw new ArgumentException("Weight is not valid.");

            //StartTimer();
           // StartTimer("Insert one");
            UpdatePlane(point);

            _plane.RemapToPlaneSpace(point, out Point3d pt);
            var vertex = new Vertex(_vertices.Count, new Point2d(pt.X, pt.Y), weight, point);

            UpdateBoundingBox(vertex);
            //EndTimer("Insert one");

            Triangulate(vertex);

            //EndTimer();
        }
        private void Triangulate(params Vertex[] vertices)
        {
            /*
             * Bowyer-Watson algorithm
             * https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm
             * 
             * Pseudocode:
             * create bounding triangulation
             * for each vertex to insert
             *   find bad triangles where circumcircle contains vertex
             *   extract boundary polygon from bad triangles
             *   remove bad triangles from triangulation
             *   for each edge in polygon join with vertex to insert this triangle
             * remove bounding triangulation
             * 
             * 
             * In order to adapt it to the weighted delaunay for arbitrary weights,
             * I have edited the algorithm in my own way:
             * Sort the vertices by weight at the beginning and return them to the original position at the end.
             * On the bad triangles that contain other vertices in their orthocircle,
             * I include the neighbouring triangles that share two edges,
             * eliminate those that prevent the creation of a convex polygon
             * and keep the largest connected group.
             * Flip the edges if in the triangles that share it
             * the direction between the connected orthocentres is opposite to the direction of their centres.
             */

            if (vertices == null || vertices.Length == 0)
                return;

            //StartTimer("Triangulate");

            if (_weighted)
                vertices = vertices.OrderBy(v => v.Weight).ToArray();

            foreach (var vertex in vertices)
            {
                //StartTimer("Triangulate", "Add vertex");
                AddVertex(vertex);
                //EndTimer("Triangulate", "Add vertex");

                //StartTimer("Triangulate", "Find bad triangles");
                var badTriangles = FindBadTriangles(vertex);
                //EndTimer("Triangulate", "Find bad triangles");
                if (!badTriangles.Any())
                    continue;
            
                //StartTimer("Triangulate", "Polygonize");
                var newTriangles = Polygonize(badTriangles, vertex);
                //EndTimer("Triangulate", "Polygonize");

                //StartTimer("Triangulate", "Remove bad triangles");
                foreach (var t in badTriangles)
                    RemoveTriangle(t);
                //EndTimer("Triangulate", "Remove bad triangles");

               // StartTimer("Triangulate", "Add new triangles"); 
                foreach (var t in newTriangles) 
                    AddTriangle(t);
                //EndTimer("Triangulate", "Add new triangles");

                FlipEdges(vertex.GetConnectedVertices().SelectMany(v => v.GetConnectedEdges()));
 
            }
       
            //StartTimer("Triangulate", "Sort vertices");
            SortVertices();
            //EndTimer("Triangulate", "Sort vertices");
            //EndTimer("Triangulate");
        
        }

        internal HashSet<Triangle> FindBadTriangles(Vertex vertex)
        { 
            var triangles = FindFirstBadTriangles();

            if (triangles.Count == 0 || !_weighted)
                return triangles;

            if (triangles.Count == 1)
            {
                if (!triangles.First().IsInsideTriangle(vertex.Position3d))
                    triangles.Remove(triangles.First());
                return triangles;
            }
             
            var edgeAdj = new Dictionary<Edge, List<Triangle>>(); 
            foreach(var t in triangles)
            {
                foreach(var e in t.GetEdges())
                {
                    if (edgeAdj.ContainsKey(e))
                        edgeAdj[e].Add(t);
                    else
                        edgeAdj.Add(e, new List<Triangle>() { t }); 
                }
            }

            //IncludeNeighbors();
            RemoveSelfIntersecting();
            GetBiggestGroup();

            return triangles;

            HashSet<Triangle> FindFirstBadTriangles()
            {
                var set = new HashSet<Triangle>();
            

                //StartTimer("Triangulate", "Find bad triangles", "First");
                     
                foreach (var t in _infiniteTriangles)
                {
                    if (t.IsInsideOrthoball(vertex, _metric))
                    {
                        set.Add(t);
                    }
                }
                foreach (var t in _triangles)
                {
                    if (t.IsInsideOrthoball(vertex, _metric))
                    {
                        set.Add(t);
                    }
                }

                ///EndTimer("Triangulate", "Find bad triangles", "First");

                //StartTimer("Triangulate", "Find bad triangles", "First");
                //StartTimer("Triangulate", "Find bad triangles", "IsBadTriangle", "First");
                //Triangle first = GetAllTriangles().FirstOrDefault(t => t.IsInsideOrthoball(vertex, _metric));
                //EndTimer("Triangulate", "Find bad triangles", "IsBadTriangle", "First");
                //if (first == null)
                //    return set;
                //StartTimer("Triangulate", "Find bad triangles", "IsBadTriangle", "Rest");
                //set.Add(first);
                //var visited = new HashSet<Triangle>();
                //visited.Add(first);
                //var queue = new Queue<Triangle>(first.GetSharedVertexTriangles());
                //while (queue.Count > 0)
                //{
                //    var t = queue.Dequeue();
                //    visited.Add(t);
                //    if (t.IsInsideOrthoball(vertex, _metric))
                //    {
                //        set.Add(t);
                //        foreach (var s in t.GetSharedVertexTriangles())
                //        {
                //            if (!visited.Contains(s))
                //            {
                //                queue.Enqueue(s);
                //            }
                //        }
                //    }
                //}
                //EndTimer("Triangulate", "Find bad triangles", "IsBadTriangle", "Rest");
                //EndTimer("Triangulate", "Find bad triangles", "IsBadTriangle");

                return set;


            }

            void TraverseTriangles(Func<Triangle, HashSet<Triangle>, int, bool> action)
            {
                var remaining = new HashSet<Triangle>(triangles);
                bool Traverse(Triangle triangle, int iteration) 
                { 
                    if (!remaining.Remove(triangle) || !action(triangle, remaining, iteration))
                        return false;
                    foreach(var edge in triangle.GetEdges())
                    {
                        var adj = edgeAdj[edge];
                        if (adj.Count == 2)
                        {
                            if (adj[0] == triangle) 
                                Traverse(adj[1], iteration);
                            else if (adj[1] == triangle)
                                Traverse(adj[0], iteration); 
                        }
                    }
                    return true;
                }

                int iter = 0;
                while (remaining.Count > 0)
                {
                    Traverse(remaining.First(), iter);
                    iter++;
                }
            }

            void IncludeNeighbors()
            {
                //StartTimer("Triangulate", "Find bad triangles", "IncludeNeighbors");
                var queue = new Queue<KeyValuePair<Edge, List<Triangle>>>(edgeAdj);
                var visited = new HashSet<Triangle>();
                while(queue.Count > 0)
                { 
                    var adj = queue.Dequeue();
                    if (adj.Value.Count > 1)
                        continue;
                    var triangle = adj.Value[0];
                    if (visited.Contains(triangle))
                        continue;
                    visited.Add(triangle);
                    foreach ( var t in triangle.GetSharedEdgeTriangles())
                    {
                        if (triangles.Contains(t))
                            continue;
                        int counter = 0;
                        var te = t.GetEdges();
                        foreach (var e in te)
                        {
                            if (edgeAdj.ContainsKey(e))
                                counter++;
                            if (counter == 2)
                                break;
                        }
                        if(counter >= 2)
                        {
                            triangles.Add(t);
                            foreach (var e in te)
                            {
                                if (edgeAdj.ContainsKey(e))
                                {
                                    edgeAdj[e].Add(t); 
                                } 
                                else
                                {
                                    var l = new List<Triangle>() { t };
                                    edgeAdj.Add(e, l); 
                                    queue.Enqueue(new KeyValuePair<Edge, List<Triangle>>(e, l));
                                }
                                   
                            }
                        }
                    } 
                }
                //EndTimer("Triangulate", "Find bad triangles", "IncludeNeighbors");
            } 
            void GetBiggestGroup()
            {
            
                if (triangles.Count <= 1)
                    return;
                //StartTimer("Triangulate", "Find bad triangles", "GetBiggestGroup");

                //var groups = new List<HashSet<Triangle>>();
                //groups.Add(new HashSet<Triangle>());
                //var remaining = new HashSet<Triangle>(triangles);

                //GroupTriangles(remaining.First());

                //while (remaining.Count > 0)
                //{
                //    groups.Add(new HashSet<Triangle>());
                //    GroupTriangles(remaining.First());
                //}

                //EndTimer("Triangulate", "Find bad triangles", "GetBiggestGroup");
                //if (groups.Count > 1)
                //{
                //    triangles = new HashSet<Triangle>(groups.OrderBy(g => g.Count).Last());
                //}

                //void GroupTriangles(Triangle triangle)
                //{
                //    groups.Last().Add(triangle);
                //    remaining.Remove(triangle);
                //    var except = new List<Triangle>() { triangle };
                //    foreach (var edge in triangle.GetEdges())
                //    {
                //        var adj = edgeAdj[edge];
                //        if (adj.Count == 2)
                //        {
                //            var next = adj.Except(except).FirstOrDefault();
                //            if (next != null && remaining.Contains(next))
                //            {
                //                GroupTriangles(next);
                //            }
                //        }
                //    }
                //}

                var groups = new List<HashSet<Triangle>>();
                    TraverseTriangles((t, r, i) =>
                    {
                        if (i >= groups.Count)
                            groups.Add(new HashSet<Triangle>());
                        groups.Last().Add(t);
                        return true;
                    });

                    if (groups.Count > 1)
                    {
                        triangles = new HashSet<Triangle>(groups.OrderBy(g => g.Count).Last());
                    }
                //EndTimer("Triangulate", "Find bad triangles", "GetBiggestGroup");
            }
            void RemoveSelfIntersecting()
            { 
                //StartTimer("Triangulate", "Find bad triangles", "RemoveSelfIntersecting");
                //var vertices = new HashSet<Vertex>(edgeAdj.Where(kv => kv.Value.Count == 1).SelectMany(e => e.Key.Vertices));
                //var hset = new HashSet<Triangle>();
                //var removed = new HashSet<Triangle>();
                //var queue = new Queue<Triangle>(triangles);
                //while(queue.Count > 0)
                //{ 
                //    var triangle = queue.Dequeue();
                //    var edges = triangle.GetEdges();
                //    var remove = false;
                //    foreach (var edge in edges)
                //    {
                //        foreach (var v in vertices.Except(edge.Vertices))
                //        {
                //            if (IsPointInTriangle(edge.A.Position3d, vertex.Position3d, edge.B.Position3d, v.Position3d))
                //            {  
                //                removed.Add(triangle);
                               
                //                foreach(var e in edges)
                //                {
                //                    edgeAdj[e].Remove(triangle);
                //                    foreach (var t in edgeAdj[e])
                //                    { 
                //                        if (t == triangle  || queue.Contains(t) || removed.Contains(t)) 
                //                            continue;
                //                        hset.Remove(t);
                //                        queue.Enqueue(t);
                //                    }
                //                }
                //                vertices = new HashSet<Vertex>(edgeAdj.Where(kv => kv.Value.Count == 1).SelectMany(e => e.Key.Vertices));

                //                remove = true;
                //                break;
                //            }
                //        }
                //        if (remove)
                //            break;
                //    }
                //    if(!remove)
                //        hset.Add(triangle);
                //}
                //EndTimer("Triangulate", "Find bad triangles", "RemoveSelfIntersecting");
                //triangles = hset;
                 
                bool IsBadVertex(Vertex v)
                { 
                    var ln = new Line(v.Position3d, vertex.Position3d); 
                    foreach (var kvp in edgeAdj)
                    { 
                        if(kvp.Value.Count == 1 && !kvp.Key.Contains(v))
                        {  
                            if (LineLineIntersects(ln, kvp.Key.ToLine()))
                            {
                                return true;
                            }
                        }
                    }
                    return false;
                }

                bool HasBadVertices()
                {
                    var vertices = edgeAdj
                        .Where(kv => kv.Value.Count == 1)
                        .SelectMany(kv => kv.Key.Vertices)
                        .Distinct();
                    foreach(var v in vertices)
                    { 
                        if (IsBadVertex(v))
                        {
                            bool removed = false;
                            foreach(var t in v.GetAllTriangles())
                            {
                                if (triangles.Contains(t))
                                {
                                    triangles.Remove(t);
                                    foreach (var e in t.GetEdges())
                                    {
                                        if (edgeAdj[e].Count > 1)
                                        {
                                            edgeAdj[e].Remove(t);
                                        }
                                        else
                                        {
                                            edgeAdj.Remove(e);
                                        } 
                                    }
                                    removed = true;
                                }
                            }
                            if (removed)
                                return true;
                        }
                    }
                    return false;
                }

                //StartTimer("Triangulate", "Find bad triangles", "RemoveSelfIntersecting3");
                while (HasBadVertices())
                {
                    if (triangles.Count <= 1)
                    {
                        break;
                    }
                } 
                //EndTimer("Triangulate", "Find bad triangles", "RemoveSelfIntersecting3"); 
            }
        }
        internal IEnumerable<Triangle> Polygonize(IEnumerable<Triangle> badTriangles, Vertex vertex)
        {
            return badTriangles
                    .SelectMany(t => t.GetEdges())
                    .GroupBy(e => e)
                    .Where(g => g.Count() == 1 && !g.First().Contains(vertex))
                    .Select(g => g.First()) 
                    .Select(e => new Triangle(e.A, e.B, vertex));
        }
        internal void FlipEdgesSLOW(IEnumerable<Edge> edges)
        {
            bool IsBadEdge(Triangle t0, Triangle t1, out double distance)
            {
                var n0 = t0.GetCenter() - t1.GetCenter();
                var n1 = t0.Orthocenter - t1.Orthocenter;
                distance = Math.Abs(n1.Length);
                return n0 * n1 < 0;
            }

            //StartTimer("Triangulate", "Flip connected edges");

            var queue = new SortedDictionary<double, Edge>();
            var flipped = new Dictionary<Edge, double[]>();
            void Enqueue(Edge edge)
            {
                if (queue.ContainsValue(edge))
                { 
                    return;
                }
                var key = -(edge.A.WeightSquared + edge.B.WeightSquared);
                //var key = -Math.Max(edge.A.WeightSquared, edge.B.WeightSquared);
                //var key = -(edge.A.WeightSquared + edge.B.WeightSquared) / 2.0;
                while (queue.ContainsKey(key))
                    key -= 1e-3;
                queue.Add(key, edge);
            }
            Edge Dequeue()
            {
                var key = queue.Keys.First();
                var edge = queue[key];
                queue.Remove(key);
                return edge;
            }

            //StartTimer("Triangulate", "Flip connected edges", "Enqueue");
            foreach (var e in edges.Distinct())
                Enqueue(e);
            //EndTimer("Triangulate", "Flip connected edges", "Enqueue");
           
            //StartTimer("Triangulate", "Flip connected edges", "While");
            var maxIters = (int)Math.Pow(queue.Count, 3);
            while (queue.Count > 0 && maxIters-- > 0)
            {
                var edge = Dequeue();
                var shared = edge.GetConnnectedTriangles().ToArray();
                if (shared.Length != 2 || (flipped.ContainsKey(edge) && flipped[edge][1] > 1) || shared[0].GetNormal() * shared[1].GetNormal() < 0)
                    continue;
                var ov0 = shared[0].GetOppositeVertex(edge);
                var ov1 = shared[1].GetOppositeVertex(edge);
                if (ov0.GetNormal() * ov1.GetNormal() < 0)
                    continue;
                if (IsBadEdge(shared[0], shared[1], out double d))
                { 
                    var contains = flipped.ContainsKey(edge);
                    if (contains && flipped[edge][0] > d )
                        continue;
                    if (contains)
                    {
                        flipped[edge][0] = d;
                        flipped[edge][1]++;
                    }
                    else
                    {
                        flipped.Add(edge, new double[] { d, 1 });
                    }
                    foreach (var t in shared)
                        RemoveTriangle(t);
                    var t0 = new Triangle(ov0, edge.A, ov1);
                    var t1 = new Triangle(ov1, edge.B, ov0);
                    AddTriangle(t0);
                    AddTriangle(t1);
                    foreach (var e in ov0.GetConnectedEdges().Concat(ov1.GetConnectedEdges()).Distinct())
                        if (!flipped.ContainsKey(edge) || flipped[edge][1] < 2)
                            Enqueue(e);

                }
            }
           
            //EndTimer("Triangulate", "Flip connected edges", "While");

           // EndTimer("Triangulate", "Flip connected edges");
        }
        internal void FlipEdges(IEnumerable<Edge> edges)
        {
            //StartTimer("Triangulate", "Flip connected edges 3");
            var set = new HashSet<Edge>(edges);
            var visited = new HashSet<Edge>();
            while(set.Count > 0)
            { 
                //StartTimer("Triangulate", "Flip connected edges 3", "1");
                var edge = set.First();
                set.Remove(edge);
                if (!visited.Add(edge))
                    continue;

                //EndTimer("Triangulate", "Flip connected edges 3", "1");
                //StartTimer("Triangulate", "Flip connected edges 3", "2");
                var shared = edge.GetConnnectedTriangles().ToArray();
                //EndTimer("Triangulate", "Flip connected edges 3", "2");
                if (shared.Length != 2)
                    continue;
                var t0 = shared[0];
                var t1 = shared[1];
                //if (t0.GetNormal() * t1.GetNormal() < 0)
                //    continue;
                //StartTimer("Triangulate", "Flip connected edges 3", "3");
                var n0 = t0.GetCenter() - t1.GetCenter();
                var n1 = t0.Orthocenter - t1.Orthocenter;
                //EndTimer("Triangulate", "Flip connected edges 3", "3");
                if (n0 * n1 < 0)
                {
                    //StartTimer("Triangulate", "Flip connected edges 3", "4");
                    var ov0 = t0.GetOppositeVertex(edge);
                    var ov1 = t1.GetOppositeVertex(edge);
                    //EndTimer("Triangulate", "Flip connected edges 3", "4");
                    //if (ov0.GetNormal() * ov1.GetNormal() < 0)
                    //    continue;
                    //StartTimer("Triangulate", "Flip connected edges 3", "5");
                    foreach (var t in shared)
                        RemoveTriangle(t);
                    var t2 = new Triangle(ov0, edge.A, ov1);
                    var t3 = new Triangle(ov1, edge.B, ov0);
                    AddTriangle(t2);
                    AddTriangle(t3);
                    //EndTimer("Triangulate", "Flip connected edges 3", "5");
                   // StartTimer("Triangulate", "Flip connected edges 3", "6");
                    //foreach (var e in ov0.GetConnectedEdges().Concat(ov1.GetConnectedEdges()))
                    //    if (!visited.Contains(e))
                    //        set.Add(e);
                    var newEdges = new Edge[] {
                        new Edge(ov0, edge.A), new Edge(ov0, edge.B),
                        new Edge(ov1, edge.A), new Edge(ov1, edge.B)
                    };
                    foreach (var e in newEdges)
                    {
                        if (visited.Contains(e))
                            visited.Remove(e);
                        set.Add(e);
                    }
                      
                    //EndTimer("Triangulate", "Flip connected edges 3", "6");
                }
            }
            //EndTimer("Triangulate", "Flip connected edges 3");
        }

        private void SortVertices()
        {
            if (_weighted)
                _vertices = new HashSet<Vertex>(_vertices.OrderBy(v => v.Index));
        }
        private void UpdatePlane(params Point3d[] points)
        {
            if (!_plane.IsValid || _vertices.Count <= 3)
            {
                var pts = points.ToList();
                foreach (var v in _vertices)
                    pts.Add(v.Position3d);
                _plane = FitPlane(pts);
            }
        }
        private void UpdateBoundingBox(params Vertex[] vertices)
        {
            var previous = _boundingBox;
            if (vertices.Length > 1)
            {
                if (_boundingBox.IsValid)
                {
                    _boundingBox = new Box(_plane, vertices.Select(v => v.Position3d));
                }
                else
                {
                    foreach (var v in vertices)
                    {
                        var x = _boundingBox.X;
                        x.Grow(v.Position3d.X);
                        _boundingBox.X = x;
                        var y = _boundingBox.Y;
                        y.Grow(v.Position3d.Y);
                        _boundingBox.Y = y;
                        var z = _boundingBox.Z;
                        z.Grow(v.Position3d.Z);
                        _boundingBox.Z = z;
                    }
                }
                
            }
            else
            {
                var v = vertices[0];
                if (!_boundingBox.Contains(v.Position3d))
                {
                    if (_boundingBox.IsValid)
                    {
                        var x = _boundingBox.X;
                        x.Grow(v.Position3d.X);
                        _boundingBox.X = x;
                        var y = _boundingBox.Y;
                        y.Grow(v.Position3d.Y);
                        _boundingBox.Y = y;
                        var z = _boundingBox.Z;
                        z.Grow(v.Position3d.Z);
                        _boundingBox.Z = z;
                    }
                    else
                    {
                        var dir = _plane.PointAt(1, 1, 1) - _plane.Origin;
                        _boundingBox = new Box(_plane, new Point3d[] { v.Position3d - dir, v.Position3d + dir });
                    } 
                }
            }

            _boundingBox.MakeValid();

            if (!_boundingBox.EpsilonEquals(previous, 1e-8))
            {
                if (_infiniteVertices.Count == 0)
                    AddBoundingRectangle();
                else
                    UpdateBoundingRectangle();
            }
          
        }
        #endregion

        #region Debugger
        internal RunningTime _timer;
        internal void StartTimer(params string[] section)
        {
#if DEBUG 
            if (_timer == null)
            {
                _timer = new RunningTime("Delaunay2D");
            }
            _timer.Start(section);
#endif
        }
        internal void EndTimer(params string[] section)
        {
#if DEBUG
            if (_timer == null)
                return;

            _timer.End(section);
#endif
        }
        #endregion

        #region Infinite
        internal void SetInfiniteTriangulation(Plane plane, Box bb)
        {
            if (!plane.IsValid || !bb.IsValid)
                throw new ArgumentNullException();
            _plane = plane;
            _boundingBox = bb;
            if (_infiniteVertices.Count == 0)
                AddBoundingRectangle();
            else
                UpdateBoundingRectangle();
        }
        private Vertex CreateInfiniteVertex(int index, Point3d point)
        {
            if (index >= 0)
                throw new ArgumentException("index must be less than 0 for infinite vertices");
            _plane.ClosestParameter(point, out double s, out double t);

            return new Vertex(index, new Point2d(s, t), 0, point);
        }
        private void AddBoundingRectangle()
        { 
            var m = 100 * _boundingBox.BoundingBox.Diagonal.Length;
            _infiniteBoundingBox = _boundingBox;
            _infiniteBoundingBox.Inflate(m); 
            var v0 = CreateInfiniteVertex(-1, _infiniteBoundingBox.PointAt(0.0, 0.0, 0.5));
            var v1 = CreateInfiniteVertex(-2, _infiniteBoundingBox.PointAt(0.5, 0.0, 0.5));
            var v2 = CreateInfiniteVertex(-3, _infiniteBoundingBox.PointAt(1.0, 0.0, 0.5));
            var v3 = CreateInfiniteVertex(-4, _infiniteBoundingBox.PointAt(1.0, 0.5, 0.5));
            var v4 = CreateInfiniteVertex(-5, _infiniteBoundingBox.PointAt(1.0, 1.0, 0.5));
            var v5 = CreateInfiniteVertex(-6, _infiniteBoundingBox.PointAt(0.5, 1.0, 0.5));
            var v6 = CreateInfiniteVertex(-7, _infiniteBoundingBox.PointAt(0.0, 1.0, 0.5));
            var v7 = CreateInfiniteVertex(-8, _infiniteBoundingBox.PointAt(0.0, 0.5, 0.5));
            AddVertex(v0);
            AddVertex(v1);
            AddVertex(v2);
            AddVertex(v3);
            AddVertex(v4);
            AddVertex(v5);
            AddVertex(v6);
            AddVertex(v7);
            var t0 = new Triangle(v0, v1, v7);
            var t1 = new Triangle(v1, v2, v3);
            var t2 = new Triangle(v3, v4, v5);
            var t3 = new Triangle(v5, v6, v7);
            var t4 = new Triangle(v7, v1, v5);
            var t5 = new Triangle(v1, v3, v5);
            AddTriangle(t0);
            AddTriangle(t1);
            AddTriangle(t2);
            AddTriangle(t3);
            AddTriangle(t4);
            AddTriangle(t5);
        }
        private void UpdateBoundingRectangle()
        {
            var m = 100 * Math.Max(_boundingBox.X.Length, _boundingBox.Y.Length);
            _infiniteBoundingBox = _boundingBox;
            _infiniteBoundingBox.Inflate(m);
            var newVertices = new Vertex[] {
            CreateInfiniteVertex(-1, _infiniteBoundingBox.PointAt(0.0, 0.0, 0.5)),
            CreateInfiniteVertex(-2, _infiniteBoundingBox.PointAt(0.5, 0.0, 0.5)),
            CreateInfiniteVertex(-3, _infiniteBoundingBox.PointAt(1.0, 0.0, 0.5)),
            CreateInfiniteVertex(-4, _infiniteBoundingBox.PointAt(1.0, 0.5, 0.5)),
            CreateInfiniteVertex(-5, _infiniteBoundingBox.PointAt(1.0, 1.0, 0.5)),
            CreateInfiniteVertex(-6, _infiniteBoundingBox.PointAt(0.5, 1.0, 0.5)),
            CreateInfiniteVertex(-7, _infiniteBoundingBox.PointAt(0.0, 1.0, 0.5)),
            CreateInfiniteVertex(-8, _infiniteBoundingBox.PointAt(0.0, 0.5, 0.5))};

            foreach (var n in newVertices)
            {
                var iv = _infiniteVertices.First(v => v.Index == n.Index);
                iv.Position = n.Position;
                iv.Position3d = n.Position3d;
            }
            var ts = _infiniteTriangles.ToArray();
            _infiniteTriangles.Clear();
            foreach (var t in ts)
            {
                t.UpdateOrthoball();
                _infiniteTriangles.Add(t);

            }

        }
        #endregion

        #region Transform
        /// <summary>
        /// Apply a transform to this delaunay. It transform the plane, the bounding box and all the vertex positions.
        /// </summary>
        /// <param name="xform">Transform to apply.</param>
        public void Transform(Rhino.Geometry.Transform xform)
        {
            _plane.Transform(xform);
            _boundingBox.Transform(xform);
            _infiniteBoundingBox.Transform(xform);
            foreach (var v in _vertices)
            {
                var p = v.Position3d;
                p.Transform(xform);
                _plane.ClosestParameter(p, out double s, out double t);
                v.Position3d = p;
                v.Position = new Point2d(s, t);
            }

            foreach (var v in _infiniteVertices)
            {
                var p = v.Position3d;
                p.Transform(xform);
                _plane.ClosestParameter(p, out double s, out double t);
                v.Position3d = p;
                v.Position = new Point2d(s, t);
            }
        }
        #endregion

        #region To
        #region Vertices 
        /// <summary>
        /// Get all the vertices, including the infinite vertices.
        /// </summary> 
        /// <remarks>If you are not interested in the infinite vertices, use <see cref="Vertices"/> instead.</remarks>
        public IEnumerable<Vertex> GetAllVertices()
        {
            return _vertices.Concat(_infiniteVertices);
        }
        /// <summary>
        /// Get an array of vertex positions.
        /// </summary>
        /// <returns>For each vertex, its 3d position.</returns>
        public Point3d[] ToPoint3dArray()
        {
            return _vertices.Select(v => v.Position3d).ToArray();
        }
        /// <summary>
        /// Get an array of vertex weights.
        /// </summary>
        /// <returns>For each vertex, its weight.</returns>
        public double[] ToWeightArray()
        {
            return _vertices.Select(v => v.Weight).ToArray();
        }
        /// <summary>
        /// True if all vertices are at a distance from the plane less than the Rhino absolute tolerance.
        /// </summary>
        /// <returns>True if points are co-planar.</returns>
        public bool IsPlanar()
        {
            var tol = RhinoTolerance;
            foreach (var v in _vertices)
                if (Math.Abs(_plane.DistanceTo(v.Position3d)) > tol)
                    return false;
            return true;
        }
        #endregion
        #region Edges
        /// <summary>
        /// Get the triangulation edges.
        /// </summary> 
        public IEnumerable<Edge> GetEdges()
        {
            return _triangles.SelectMany(t => t.GetEdges()).Distinct();
        }
        /// <summary>
        /// Get the triangulation edges.
        /// </summary>
        /// <param name="lines">For each edge, its line geometry.</param> 
        public IEnumerable<Edge> GetEdges(out IEnumerable<Line> lines)
        {
            var edges = GetEdges();
            lines = edges.Select(e => e.ToLine());
            return edges;
        }
        /// <summary>
        /// Get the triangulation edges concatenated with the infinite edges.
        /// </summary> 
        public IEnumerable<Edge> GetAllEdges()
        {
            return _triangles.SelectMany(t => t.GetEdges())
                .Concat(_infiniteTriangles.SelectMany(t => t.GetEdges()))
                .Distinct();
        } 
        #endregion
        #region Triangles
        /// <summary>
        /// Gets the collection of triangles concatenated with the infinite triangles.
        /// </summary> 
        /// <remarks>The infinite triangles are only useful for debugging reasons.</remarks>
        public IEnumerable<Triangle> GetAllTriangles()
        {
            return _triangles.Concat(_infiniteTriangles);
        }
        /// <summary>
        /// Gets the collection of triangles as a polyline array.
        /// </summary> 
        public Polyline[] ToTriangleArray()
        {
            return _triangles.Select(t => t.ToPolyline()).ToArray();
        }
        /// <summary>
        /// Converts the triangulation to mesh.
        /// </summary> 
        public Mesh ToMesh()
        {
            var mesh = new Mesh();
            mesh.Vertices.AddVertices(ToPoint3dArray());
            mesh.Faces.AddFaces(_triangles.Select(t => t.ToMeshFace()));
            mesh.RebuildNormals();
            mesh.Normals.ComputeNormals();
            mesh.FaceNormals.ComputeFaceNormals();
            return mesh;
        }
        /// <summary>
        /// Converts the triangulation edges to a mesh.
        /// </summary>
        /// <param name="radius">The radius of edge.</param>
        /// <param name="divisionFactor">The number of divisions along the edge is the length of the edge multiplied by this value.</param>
        public Mesh ToThickenerPrimal(double radius, double divisionFactor = 0.0)
        {
            int SegmentCount(double distance)
            {
                if (divisionFactor <= 0.0 || double.IsNaN(divisionFactor) || double.IsInfinity(divisionFactor))
                    return 1;
                return (int)Math.Max(1, Math.Floor(distance * divisionFactor));
            }
        
            Mesh mesh = new Mesh();
            foreach (var tr in _triangles)
            { 
                var vertices = tr.Vertices;
                var points = new Point3d[3];
                var normals = new Vector3d[3];
                var nexts = new Vector3d[3];
                var prevs = new Vector3d[3];
                var avers = new Vector3d[3];
                var lengths = new double[3];
                var maxLength = new double[3];
                var aveNor = Vector3d.Zero;
                var externals = new bool[3];
                for (int i = 0; i < 3; i++)
                {
                    var v = vertices[i];
                    points[i] = v.Position3d;
                    normals[i] = v.GetNormal();
                    aveNor += normals[i];
                }              
                for (int i = 0; i < 3; i++)
                {
                    var i0 = (i - 1 + 3) % 3;
                    var i1 = (i + 1) % 3; 
                    var v0 = points[i0] - points[i];
                    v0.Unitize();
                    prevs[i] = v0;
                    var v1 = points[i1] - points[i];
                    lengths[i] = v1.Length;
                    v1.Unitize();
                    nexts[i] = v1;
                    var v2 = v0+v1;;
                    v2.Unitize();
                    avers[i] = v2;
                    if (normals[i] * aveNor < 0)
                        normals[i] *= -1;
                }
                for (int i = 0; i < 3; i++)
                {
                    var i1 = (i + 1) % 3;
                    Utils.LineLineIntersects(
                        new Line(points[i], avers[i], 1),
                        new Line(points[i1], avers[i1], 1),
                        out double t0, out double t1);
                    maxLength[i] = t0; 
                    externals[i] = new Edge(vertices[i], vertices[i1]).GetConnnectedTriangles().Where(t => !t.IsInfinite).Count() == 1;
                }
                   
                for (int j = 0; j < 3; j++)
                {
                    var j0 = (j - 1 + 3) % 3;
                    var j1 = (j + 1) % 3; 
                    var prev = points[j0];
                    var curr = points[j];
                    var next = points[j1];
                    var ang0 = Math.Sin(Vector3d.VectorAngle(nexts[j], avers[j]));
                    var ave0 = avers[j] * radius / ang0;
                    var ave0Len = ave0.Length;
                    var max0 = maxLength[j];
                    if (ave0Len > max0)
                        ave0 *= max0 / ave0Len;
                    var ang1 = Math.Sin(Vector3d.VectorAngle(nexts[j1], avers[j1]));
                    var ave1 = avers[j1] * radius / ang1;
                    var ave1Len = ave1.Length;
                    var max1 = maxLength[j1];
                    if (ave1Len > max1)
                        ave1 *= max1 / ave1Len;

                    var currNor = normals[j];
                    var pbot0 = curr - currNor * radius;
                    var pmid0 = curr + ave0;
                    var ptop0 = curr + currNor * radius;
                     
                    var nextNor = normals[j1];
                    var pbot1 = next - nextNor * radius; 
                    var pmid1 = next + ave1;
                   var ptop1 = next + nextNor * radius;

                    var m = new Mesh();
                    var segments = SegmentCount(lengths[j]);
                    for (int k = 0; k <= segments; k++)
                    {
                        var t = k / (double)segments;
                        var t1 = 1.0 - t;
                        m.Vertices.Add(t * pbot0 + t1 * pbot1);
                        m.Vertices.Add(t * pmid0 + t1 * pmid1);
                        m.Vertices.Add(t * ptop0 + t1 * ptop1);
                        if (k < segments)
                        {
                            int k3 = k * 3;
                            m.Faces.AddFace(k3, k3 + 3, k3 + 4, k3 + 1);
                            m.Faces.AddFace(k3 + 1, k3 + 4, k3 + 5, k3 + 2);
                        }
                    }
                    
                    if (externals[j])
                    {
                        var m2 = new Mesh();
                        Vector3d dir0 = Vector3d.Zero;
                        if ( externals[j0])
                        {
                            dir0 = -avers[j] * radius / ang0;
                        }
                        else
                        {
                            var va = vertices[j];
                            var v0 = va.GetConnectedVertices()
                                .Where(v => v.IsExternal)
                                .Except(vertices)
                                .OrderBy(v => v.Position3d.DistanceToSquared(va.Position3d))
                                .First();
                            var vec = v0.Position3d - va.Position3d;
                            vec.Unitize();
                            dir0 = vec + nexts[j];
                            dir0 *=  -(radius / Math.Sin(Vector3d.VectorAngle(nexts[j], dir0))) / dir0.Length;
                        }
                        Vector3d dir1 = Vector3d.Zero;
                        if (externals[j1])
                        {
                            dir1 = -avers[j1] * radius / ang1;
                        }
                        else
                        {
                            var va = vertices[j1];
                            var v0 = va.GetConnectedVertices()
                                .Where(v => v.IsExternal)
                                .Except(vertices)
                                .OrderBy(v => v.Position3d.DistanceToSquared(va.Position3d))
                                .First();
                            var vec = v0.Position3d - va.Position3d; vec.Unitize();
                            dir1 = vec + prevs[j1];
                            dir1 *= -(radius / Math.Sin(Vector3d.VectorAngle(prevs[j1], dir1))) / dir1.Length;
                        }
                        var pmid2 = curr + dir0;
                        var pmid3 = next + dir1;
                        for (int k = 0; k <= segments; k++)
                        {
                            var t = k / (double)segments;
                            var t1 = 1.0 - t;
                            m2.Vertices.Add(t1 * pbot0 + t * pbot1);
                            m2.Vertices.Add(t1 * pmid2 + t * pmid3);
                            m2.Vertices.Add(t1 * ptop0 + t * ptop1);
                            if (k < segments)
                            {
                                int k3 = k * 3;
                                m2.Faces.AddFace(k3, k3 + 3, k3 + 4, k3 + 1);
                                m2.Faces.AddFace(k3 + 1, k3 + 4, k3 + 5, k3 + 2);
                            }
                        } 
                        m.Append(m2);
                    }

                    mesh.Append(m);
                }
                 
            }

            mesh.Vertices.CombineIdentical(true, true);
            mesh.Weld(Math.PI);
            mesh.Compact();
            mesh.RebuildNormals();  
            return mesh;
        }
        #endregion
        #region Voronoi        
        /// <summary>
        /// Gets the voronoi cell of each vertex.
        /// </summary> 
        public IEnumerable<Polyline> ToVoronoi()
        {
            //return _vertices.Select(v => v.ToVoronoiCell(out _));
            var voronoi = new Polyline[_vertices.Count];
            System.Threading.Tasks.Parallel.ForEach(_vertices, (v, s, i) =>
            {
                voronoi[i] = v.ToVoronoiCell(out _);
            });
            return voronoi;
        }
        /// <summary>
        /// Gets the voronoi cell of each vertex.
        /// </summary>
        /// <param name="bounds">The polyline bounds to intersect the cells.</param> 
        public IEnumerable<Polyline> ToVoronoi(Curve bounds)
        {
            Polyline pln = null;
            if (bounds != null)
            {
                if (!bounds.IsClosed)
                    throw new ArgumentException("bounds must be closed");
                bounds.MakeDeformable();
                bounds.Transform(Rhino.Geometry.Transform.PlanarProjection(Plane));
                if (!bounds.TryGetPolyline(out pln))
                {
                    var s = Math.Min((0.1 * _boundingBox.Area / _vertices.Count) / 6, bounds.GetLength() / 20);
                    pln = bounds.ToPolyline(s, Rhino.RhinoDoc.ActiveDoc.ModelAngleToleranceRadians, 0.75 * s, 1.25 * s).ToPolyline();
                }
            }

           // return _vertices.Select(v => v.ToVoronoiCell(out _, pln));
            var voronoi = new Polyline[_vertices.Count];
            System.Threading.Tasks.Parallel.ForEach(_vertices, (v, s, i) =>
            {
                voronoi[i] = v.ToVoronoiCell(out _, pln);
            });
            return voronoi;
        }
        /// <summary>
        /// Converts the voronoi diagram to a mesh.
        /// </summary>
        /// <param name="radius">The radius of edge.</param>
        /// <param name="divisionFactor">The number of divisions along the edge is the length of the edge multiplied by this value.</param>
        /// <param name="bounds">The polyline bounds to intersect the cells.</param> 
        public Mesh ToThickenerDual(double radius, double divisionFactor = 0.0, Curve bounds = null)
        {
            int SegmentCount(double distance)
            {
                if (divisionFactor <= 0.0 || double.IsNaN(divisionFactor) || double.IsInfinity(divisionFactor))
                    return 1;
                return (int)Math.Max(1, Math.Floor(distance * divisionFactor));
            }
            Mesh mesh = new Mesh();
            Polyline pln = null;
            if (bounds != null)
            {
                if (!bounds.IsClosed)
                    throw new ArgumentException("bounds must be closed");
                bounds.MakeDeformable();
                bounds.Transform(Rhino.Geometry.Transform.PlanarProjection(Plane));
                if (!bounds.TryGetPolyline(out pln))
                {
                    var s = Math.Min((0.1 * _boundingBox.Area / _vertices.Count) / 6, bounds.GetLength() / 20);
                    pln = bounds.ToPolyline(s, Rhino.RhinoDoc.ActiveDoc.ModelAngleToleranceRadians, 0.75 * s, 1.25 * s).ToPolyline();
                }
              
            }
            foreach (var v in _vertices)
            {
                var cell = v.ToVoronoiCell(out List<Tuple<Point3d, Vector3d, int>> cellOrientation, pln);
                if (cell == null)
                    continue;
                
                int count = cellOrientation.Count;
                for (int j = 0; j < count; j++)
                {
                    var prevO = cellOrientation[(j - 1 + count) % count];
                    var currO = cellOrientation[j];
                    var nextO = cellOrientation[(j + 1) % count];
                    var next2O = cellOrientation[(j + 2) % count];

                    var prev = prevO.Item1;
                    var curr = currO.Item1;
                    var next = nextO.Item1;
                    var next2 = next2O.Item1;

                    var vprev = prev - curr;
                    var vnext = next - curr;
                    var vnext2 = next2 - next;
                    var length = vnext.Length;
                    vnext.Unitize();
                    vprev.Unitize();
                    vnext2.Unitize();
                    var currNor = currO.Item2;
                    var perp0 = Vector3d.CrossProduct(currNor, vnext);
                    var vmid0 = (vprev + vnext) / 2.0;
                    if (vmid0.IsZero)
                        vmid0 = perp0;
                    vmid0.Unitize();
                    if (vmid0 * perp0 < 0)
                        vmid0 *= -1;
                    var sin0 = Math.Sin(Vector3d.VectorAngle(vprev, vnext) / 2.0);
                    if (sin0 == 0)
                        sin0 = 1;
                    var pbot0 = curr - currNor * radius;
                    var pmid0 = curr + vmid0 * (radius / sin0);
                    var ptop0 = curr + currNor * radius;

                    var nextNor = nextO.Item2;
                    var perp1 = Vector3d.CrossProduct(nextNor, vnext2);
                    var vmid1 = (-vnext + vnext2) / 2.0;
                    if (vmid1.IsZero)
                        vmid1 = perp1;
                    vmid1.Unitize();
                    if (vmid1 * perp1 < 0)
                        vmid1 *= -1;
                    var sin1 = Math.Sin(Vector3d.VectorAngle(-vnext, vnext2) / 2.0);
                    if (sin1 == 0)
                        sin1 = 1;
                    var pbot1 = next - nextNor * radius;
                    var pmid1 = next + vmid1 * (radius / sin1);
                    var ptop1 = next + nextNor * radius;

                    var m = new Mesh();
                    int segments = SegmentCount(length);
                    for (int k = 0; k <= segments; k++)
                    {
                        var t = k / (double)segments;
                        var t1 = 1.0 - t;
                        m.Vertices.Add(t * ptop0 + t1 * ptop1);
                        m.Vertices.Add(t * pmid0 + t1 * pmid1);
                        m.Vertices.Add(t * pbot0 + t1 * pbot1);
                        if (k < segments)
                        {
                            int k3 = k * 3;
                            m.Faces.AddFace(k3, k3 + 3, k3 + 4, k3 + 1);
                            m.Faces.AddFace(k3 + 1, k3 + 4, k3 + 5, k3 + 2);
                        }
                    }
                    mesh.Append(m);
                }

                if (cellOrientation.Any(co => co.Item3 > 0))
                {
                    var coi = cellOrientation.FirstOrDefault(co => co.Item3 == 1);
                    if (coi != null)
                    {
                        var id = cellOrientation.IndexOf(coi);
                        var currO = cellOrientation[id];
                        var prevO = cellOrientation[(id - 1 + count) % count];
                        var prev2O = cellOrientation[(id - 2 + count) % count];
                        var nextO = cellOrientation[(id + 1) % count];
                        var next2O = cellOrientation[(id + 2) % count];

                        var curr = currO.Item1;
                        var prev = prevO.Item1;
                        var prev2 = prev2O.Item1;
                        var next = nextO.Item1;
                        var next2 = next2O.Item1;

                        var vprev = prev - curr;
                        var vnext = next - curr;
                        var vprev2 = prev2 - prev;
                        var vnext2 = next2 - next;
                        var length0 = vprev.Length;
                        var length1 = vnext.Length;
                        vprev.Unitize();
                        vnext.Unitize();
                        vprev2.Unitize();
                        vnext2.Unitize();
                        var ave = -(vprev + vnext) / 2;
                        if (ave.IsZero)
                            ave = -(vprev2 + vnext2) / 2;
                        ave.Unitize();
                        var preNor = prevO.Item2;
                        var nor = currO.Item2;
                        var nextNor = nextO.Item2;
                        var perp0 = Vector3d.CrossProduct(nor, vprev);
                        var perp1 = Vector3d.CrossProduct(nor, vnext);
                        if (ave * perp0 < 0)
                            ave *= -1;

                        var cos0 = Math.Abs(Math.Cos(Vector3d.VectorAngle(-vprev2, perp0)));
                        if (cos0 == 0)
                            cos0 = 1;
                        var cos1 = Math.Abs(Math.Cos(Vector3d.VectorAngle(vnext2, perp1)));
                        if (cos1 == 0)
                            cos1 = 1;
                        var sin = Math.Sin(Vector3d.VectorAngle(vprev, vnext) / 2.0);
                        if (sin == 0)
                            sin = 1;

                        var pbot0 = prev - preNor * radius;
                        var pmid0 = prev - vprev2 * (radius / cos0);
                        var ptop0 = prev + preNor * radius;
                        var pbot = curr - nor * radius;
                        var pmid = curr + ave * (radius / sin);
                        var ptop = curr + nor * radius;
                        var pbot1 = next - nextNor * radius;
                        var pmid1 = next - vnext2 * (radius / cos1);
                        var ptop1 = next + nextNor * radius;

                        var m = new Mesh();
                        var segment0 = SegmentCount(length0);
                        for (int k = 0; k <= segment0; k++)
                        {
                            var t = k / (double)segment0;
                            var t1 = 1.0 - t;
                            m.Vertices.Add(t * pbot0 + t1 * pbot);
                            m.Vertices.Add(t * pmid0 + t1 * pmid);
                            m.Vertices.Add(t * ptop0 + t1 * ptop);
                            if (k < segment0)
                            {
                                int k3 = k * 3;
                                m.Faces.AddFace(k3, k3 + 3, k3 + 4, k3 + 1);
                                m.Faces.AddFace(k3 + 1, k3 + 4, k3 + 5, k3 + 2);
                            }
                        }
                        mesh.Append(m);
                        m = new Mesh();
                        var segment1 = SegmentCount(length1);
                        for (int k = 0; k <= segment1; k++)
                        {
                            var t = k / (double)segment1;
                            var t1 = 1.0 - t;
                            m.Vertices.Add(t * pbot + t1 * pbot1);
                            m.Vertices.Add(t * pmid + t1 * pmid1);
                            m.Vertices.Add(t * ptop + t1 * ptop1);
                            if (k < segment1)
                            {
                                int k3 = k * 3;
                                m.Faces.AddFace(k3, k3 + 3, k3 + 4, k3 + 1);
                                m.Faces.AddFace(k3 + 1, k3 + 4, k3 + 5, k3 + 2);
                            }
                        }
                        mesh.Append(m);
                    }
                    else
                    {
                        var coi2 = cellOrientation.Where(co => co.Item3 == 2).ToArray();
                        if (coi2 != null && coi2.Length > 1)
                        {
                            for (int i = 0; i < coi2.Length - 1; i++)
                            {

                                var id = cellOrientation.IndexOf(coi2[i]);
                                var currO = cellOrientation[id];
                                var prevO = cellOrientation[(id - 1 + count) % count];
                                var nextO = cellOrientation[(id + 1) % count];
                                var nextO2 = cellOrientation[(id + 2) % count];
                                var curr = currO.Item1;
                                var prev = prevO.Item1;
                                var next = nextO.Item1;
                                var next2 = nextO2.Item1;

                                var vprev = prev - curr;
                                var vnext = next - curr;
                                var vnext2 = next2 - next;
                                var length = vnext.Length;
                                vnext.Unitize();
                                vprev.Unitize();
                                vnext2.Unitize();
                                var preNor = prevO.Item2;
                                var nor = currO.Item2;
                                var nextNor = nextO.Item2;
                                var perp = Vector3d.CrossProduct(nor, vnext);
                                var dir0 = vprev;
                                if (i > 0)
                                {
                                    dir0 = -(vprev + vnext) / 2;
                                    if (dir0.IsZero)
                                        dir0 = perp;
                                    dir0.Unitize();
                                    if (dir0 * perp < 0)
                                        dir0 *= -1;
                                }
                                var dir1 = vnext2;
                                if (i < coi2.Length - 2)
                                {
                                    dir1 = -(vnext2 - vnext) / 2;
                                    if (dir1.IsZero)
                                        dir1 = perp;
                                    dir1.Unitize();
                                    if (dir1 * perp < 0)
                                        dir1 *= -1;
                                }
                                var pbot = curr - nor * radius;
                                var pmid = curr - dir0 * (radius / Math.Cos(Vector3d.VectorAngle(dir0, perp)));
                                var ptop = curr + nor * radius;
                                var pbot1 = next - nextNor * radius;
                                var pmid1 = next - dir1 * (radius / Math.Cos(Vector3d.VectorAngle(dir1, perp)));
                                var ptop1 = next + nextNor * radius;

                                var m = new Mesh();
                                var segments = SegmentCount(length);
                                for (int k = 0; k <= segments; k++)
                                {
                                    var t = k / (double)segments;
                                    var t1 = 1.0 - t;
                                    m.Vertices.Add(t * pbot + t1 * pbot1);
                                    m.Vertices.Add(t * pmid + t1 * pmid1);
                                    m.Vertices.Add(t * ptop + t1 * ptop1);
                                    if (k < segments)
                                    {
                                        int k3 = k * 3;
                                        m.Faces.AddFace(k3, k3 + 3, k3 + 4, k3 + 1);
                                        m.Faces.AddFace(k3 + 1, k3 + 4, k3 + 5, k3 + 2);
                                    }
                                }
                                mesh.Append(m);
                            }

                        }
                    }

                }
            }
            mesh.Vertices.CombineIdentical(true, true);
            mesh.Weld(Math.PI);
            mesh.Normals.ComputeNormals();
            mesh.FaceNormals.ComputeFaceNormals();
            mesh.UnifyNormals();
            return mesh;
        }
        #endregion
        #region ConvexHull        
        /// <summary>
        /// Constructs the convex hull, the smallest convex shape that contains all vertices.
        /// </summary> 
        /// <returns>The convex hull edges.</returns>
        public IEnumerable<Edge> ToConvexHull()
        {
            var edges = new Queue<Edge>(GetEdges().Where(e => e.A.IsExternal && e.B.IsExternal));
            if (edges.Count == 0)
                return new List<Edge>();
            var chain = new List<Edge>() { edges.Dequeue() };
            var exitIter = Math.Pow(edges.Count, 3);
            while (edges.Count > 0 && exitIter-- > 0)
            {
                var edge = edges.Dequeue();
                if (edge.Contains(chain.First().A))
                {
                    chain.Insert(0, edge);
                }
                else if (edge.Contains(chain.Last().B))
                {
                    chain.Add(edge);
                }
                else
                {
                    edges.Enqueue(edge);
                }
            }
            return chain;
        }
        /// <summary>
        /// Constructs the convex hull, the smallest convex shape that contains all vertices.
        /// </summary>
        /// <param name="pln">The joinned edges polyline.</param> 
        /// <returns>The convex hull edges.</returns>
        public IEnumerable<Edge> ToConvexHull(out Polyline pln)
        {
            var chain = ToConvexHull();
            pln = new Polyline(chain.Select(e => e.A.Position3d));
            if (pln.Count > 2)
                pln.Add(pln[0]);
            return chain;
        }
        #endregion
        #region AlphaShape        
        /// <summary>
        /// Constructs the alpha shape, the bounding polygons where for each edge there is no point within the circle formed by its ends and a given radius.
        /// </summary>
        /// <param name="radius">The radius of the alpha ball.</param>
        /// <param name="polylines">The polyline shapes.</param>
        /// <returns>For each closed shape, its vertices.</returns> 
        public IEnumerable<IEnumerable<Vertex>> ToAlphaShape(double radius, out List<Polyline> polylines)
        {
            return ToAlphaShape(e => radius, out polylines);
        }
        /// <summary>
        /// Constructs the alpha shape, the bounding polygons where for each edge there is no point within the circle formed by its ends and a given radius.
        /// </summary>
        /// <param name="radiusFunc">The radius function per edge of the alpha ball.</param>
        /// <param name="polylines">The polyline shapes.</param>
        /// <returns>For each closed shape, its vertices.</returns> 
        /// <see cref="ToAlphaShape(double, out List{Polyline})"/>
        public IEnumerable<IEnumerable<Vertex>> ToAlphaShape(Func<Edge, double> radiusFunc, out List<Polyline> polylines)
        {
            bool AlphaShapeCondition(Edge edge, double radius)
            {
                radius = Math.Max(radius, 0.01);
                var x1 = edge.A.Position.X;
                var y1 = edge.A.Position.Y;
                var x2 = edge.B.Position.X;
                var y2 = edge.B.Position.Y;
                var xa = 0.5 * (x2 - x1);
                var ya = 0.5 * (y2 - y1);
                var x0 = x1 + xa;
                var y0 = y1 + ya;
                var a = Math.Sqrt(xa * xa + ya * ya);
                var b = Math.Sqrt(radius * radius - a * a);
                var c0 = new Point2d(x0 + (b * ya) / a, y0 - (b * xa) / a);
                var c1 = new Point2d(x0 - (b * ya) / a, y0 + (b * xa) / a);
                bool nope0 = false;
                bool nope1 = false;
                foreach (var nei in edge.A.GetConnectedVertices().Concat(edge.B.GetConnectedVertices()).Distinct())
                {
                    if (nei == edge.A || nei == edge.B)
                        continue;
                    if (!nope0 && nei.Position.DistanceTo(c0) < radius)
                    {
                        nope0 = true;
                        continue;
                    }
                    if (!nope1 && nei.Position.DistanceTo(c1) < radius)
                    {
                        nope1 = true;
                        continue;
                    }
                    if (nope0 && nope1)
                        return false;
                }
                return true;
            }

            polylines = new List<Polyline>();
            var edgeArray = GetEdges().Where(e => AlphaShapeCondition(e, radiusFunc(e))).ToArray();

            var adjacency = new Dictionary<int, List<int>>();
            var toFix = new HashSet<int>();
            for (int i = 0; i < edgeArray.Length; i++)
            {
                var ei = edgeArray[i];
                if (!adjacency.ContainsKey(i))
                    adjacency.Add(i, new List<int>());
                for (int j = i + 1; j < edgeArray.Length; j++)
                {
                    var ej = edgeArray[j];
                    if (!adjacency.ContainsKey(j))
                        adjacency.Add(j, new List<int>());
                    if (ei.Contains(ej.A) || ei.Contains(ej.B))
                    {
                        adjacency[i].Add(j);
                        adjacency[j].Add(i);
                        if (adjacency[i].Count > 3)
                            toFix.Add(i);
                        if (adjacency[j].Count > 3)
                            toFix.Add(j);
                    }
                }
            }
            var invalid = new bool[edgeArray.Length];
            foreach (var i in toFix.OrderBy(j => adjacency[j].Count).Reverse())
            {
                var edge = edgeArray[i];
                var adj = adjacency[i];
                var valence = adj.Count;
                if (valence == 4)
                {
                    var exit = false;
                    for (int j = 0; j < adj.Count; j++)
                    {
                        var eji = adj[j];
                        var ej = edgeArray[eji];

                        for (int k = j + 1; k < adj.Count; k++)
                        {
                            var eki = adj[k];
                            var ek = edgeArray[eki];
                            if (
                              (ej.A == ek.A && edge.Contains(ej.B) && edge.Contains(ek.B)) ||
                              (ej.B == ek.B && edge.Contains(ej.A) && edge.Contains(ek.A)) ||
                              (ej.A == ek.B && edge.Contains(ej.B) && edge.Contains(ek.A)) ||
                              (ej.B == ek.A && edge.Contains(ej.A) && edge.Contains(ek.B)))
                            {
                                invalid[eji] = true;
                                invalid[eki] = true;
                                adj.Remove(eji);
                                adj.Remove(eki);
                                adjacency[eji].Remove(i);
                                adjacency[eki].Remove(i);
                                exit = true;
                                break;
                            }
                        }
                        if (exit)
                            break;
                    }
                    if (!exit)
                    {
                        invalid[i] = adj.Where(a => !invalid[a]).Count() == 4;
                    }
                }
                else if (valence == 5)
                {
                    var exit = false;
                    for (int j = 0; j < adj.Count; j++)
                    {
                        var eji = adj[j];
                        var ej = edgeArray[eji];
                        for (int k = j + 1; k < adj.Count; k++)
                        {
                            var eki = adj[k];
                            var ek = edgeArray[eki];
                            if (
                              (ej.A == ek.A && edge.Contains(ej.B) && edge.Contains(ek.B)) ||
                              (ej.B == ek.B && edge.Contains(ej.A) && edge.Contains(ek.A)) ||
                              (ej.A == ek.B && edge.Contains(ej.B) && edge.Contains(ek.A)) ||
                              (ej.B == ek.A && edge.Contains(ej.A) && edge.Contains(ek.B)))
                            {
                                invalid[eji] = true;
                                invalid[eki] = true;
                                adj.Remove(eji);
                                adj.Remove(eki);
                                adjacency[eji].Remove(i);
                                adjacency[eki].Remove(i);
                                invalid[i] = false;
                                exit = true;
                                break;
                            }
                        }
                        if (exit)
                            break;
                    }
                }
            }
             
            var chains = new List<List<Edge>>();
            var currentPolyline = new Polyline();
            var currentChain = new List<Edge>();

            var queue = new Queue<int>();
            var currentIndex = 0;
            var counter = edgeArray.Length;
            while (currentIndex < edgeArray.Length && (invalid[currentIndex] || adjacency[currentIndex].All(a => invalid[a])) && counter > 0)
            {
                invalid[currentIndex] = true;
                currentIndex++;
                counter--;
            }

            if (currentIndex >= edgeArray.Length)
                return null;

            queue.Enqueue(currentIndex);

            do
            {
                currentIndex = queue.Dequeue();
                if (invalid[currentIndex])
                    continue;
                var currentEdge = edgeArray[currentIndex];

                if (currentPolyline.Count == 0)
                {
                    currentPolyline.Add(currentEdge.A.Position3d);
                    currentPolyline.Add(currentEdge.B.Position3d);
                    currentChain.Add(currentEdge);
                }
                else
                {

                    if (currentEdge.A.Position3d == currentPolyline.Last())
                    {
                        currentPolyline.Add(currentEdge.B.Position3d);
                        currentChain.Add(currentEdge);
                    }
                    else if (currentEdge.B.Position3d == currentPolyline.Last())
                    {
                        currentPolyline.Add(currentEdge.A.Position3d);
                        currentChain.Add(currentEdge);
                    }
                    else if (currentEdge.A.Position3d == currentPolyline.First())
                    {
                        currentPolyline.Insert(0, currentEdge.B.Position3d);
                        currentChain.Insert(0, currentEdge);
                    }
                    else if (currentEdge.B.Position3d == currentPolyline.First())
                    {
                        currentPolyline.Insert(0, currentEdge.A.Position3d);
                        currentChain.Insert(0, currentEdge);
                    }

                }
                invalid[currentIndex] = true;
                if (currentChain.Count > 2 && currentChain.First().IsConnectedTo(currentChain.Last()))
                {
                    polylines.Add(currentPolyline);
                    chains.Add(currentChain);
                    currentPolyline = new Polyline();
                    currentChain = new List<Edge>();
                    currentIndex = edgeArray.Length - 1;
                    while (currentIndex > -1 && (invalid[currentIndex] || adjacency[currentIndex].All(a => invalid[a])))
                    {
                        invalid[currentIndex] = true;
                        currentIndex--;
                    }
                    if (currentIndex == -1)
                        break;
                    queue.Enqueue(currentIndex);
                }
                else
                {
                    foreach (var id in adjacency[currentIndex].Where(a => !invalid[a]))
                        queue.Enqueue(id);
                }
            }
            while (queue.Count > 0);

            if (polylines.Count == 0)
            {
                polylines.Add(currentPolyline);
                chains.Add(currentChain);
            }

            var vertices = new HashSet<Vertex>[chains.Count];
            for (int i = 0; i < chains.Count; i++)
            {
                var set = new HashSet<Vertex>();
                var chain = chains[i];
                var lastEdge = chain[0];
                if(chain.Count == 1)
                {
                    set.Add(lastEdge.A);
                    set.Add(lastEdge.B);
                }
                else
                {
                    if (chain[1].Contains(lastEdge.B))
                    {
                        set.Add(lastEdge.A);
                        set.Add(lastEdge.B);
                    }
                    else
                    {
                        set.Add(lastEdge.B);
                        set.Add(lastEdge.A);
                    }
            
                }
            
                for (int j = 1; j < chain.Count; j++)
                {
                    var e = chain[j];
                    if (lastEdge.Contains(e.A))
                    {
                        set.Add(e.B);
                    }
                    else
                    {
                        set.Add(e.A);
                    }
                    lastEdge = e;
                }
                vertices[i] = set;
            }
            return vertices;
        }
        #endregion
        #region BetaSkeleton          
        /// <summary>
        /// Constructs the beta skeleton graph.
        /// </summary>
        /// <param name="beta">Beta factor.<para>If beta &lt; 1, it uses the intersection of the two beta circles with radius edge length / beta.</para> <para>If beta = 1, the diameter of the beta circle is the edge length (Gabriel Graph).</para> <para>If beta &gt; 1, it uses the union of the two beta circles with radius edge length * beta.</para></param>
        /// <param name="lines">The resulting lines.</param>
        /// <returns>The collection of beta skeleton edges.</returns>
        /// <see cref="ToBetaSkeleton(Func{Edge, double}, out IEnumerable{Line})"/>
        public IEnumerable<Edge> ToBetaSkeleton(double beta, out IEnumerable<Line> lines)
        {
            return ToBetaSkeleton(e => beta, out lines);
        }
        /// <summary>
        /// Constructs the beta skeleton graph.
        /// </summary>
        /// <param name="betaFunc">Beta factor function for each edge.<para>If beta &lt; 1, it uses the intersection of the two beta circles with radius edge length / beta.</para> <para>If beta = 1, the diameter of the beta circle is the edge length (Gabriel Graph).</para> <para>If beta &gt; 1, it uses the union of the two beta circles with radius edge length * beta.</para></param>
        /// <param name="lines">The resulting lines.</param>
        /// <returns>The collection of beta skeleton edges.</returns>
        /// <see cref="ToBetaSkeleton(double, out IEnumerable{Line})"/>
        public IEnumerable<Edge> ToBetaSkeleton(Func<Edge, double> betaFunc, out IEnumerable<Line> lines)
        {
            bool BetaSkeletonCondition(Edge edge, double beta)
            {
                var x1 = edge.A.Position.X;
                var y1 = edge.A.Position.Y;
                var x2 = edge.B.Position.X;
                var y2 = edge.B.Position.Y;
                var xa = 0.5 * (x2 - x1);
                var ya = 0.5 * (y2 - y1);
                var x0 = x1 + xa;
                var y0 = y1 + ya;
                var len = edge.GetLength();
                bool betaLarger = beta >= 1.0;
                var diam = betaLarger ? len * beta : len / beta;
                var radius = diam / 2.0;
                var a = Math.Sqrt(xa * xa + ya * ya);
                var b = Math.Sqrt(radius * radius - a * a);
                var c0 = new Point2d(x0 + (b * ya) / a, y0 - (b * xa) / a);
                var c1 = new Point2d(x0 - (b * ya) / a, y0 + (b * xa) / a);

                bool nope = false;
                foreach (var nei in edge.A.GetConnectedVertices().Concat(edge.B.GetConnectedVertices()).Distinct())
                {
                    if (nei == edge.A || nei == edge.B)
                        continue;
                    if (betaLarger)
                    {
                        if (nei.Position.DistanceTo(c0) < radius || nei.Position.DistanceTo(c1) < radius)
                        {
                            nope = true;
                            continue;
                        }
                    }
                    else
                    {
                        if (nei.Position.DistanceTo(c0) < radius && nei.Position.DistanceTo(c1) < radius)
                        {
                            nope = true;
                            continue;
                        }
                    }
                    if (nope)
                        break;
                }
                return !nope;
            }

            var edges = GetEdges().Where(e => BetaSkeletonCondition(e, betaFunc(e)));
            lines = edges.Select(e => e.ToLine());
            return edges;
        }
        #endregion
        #region RelativeNeighborhood        
        /// <summary>
        /// Constructs the relative neighborhood graph, where for each edge there is no other point closer (using power distance) to both than they are to each other.
        /// </summary>
        /// <returns>The relative neighborhood graph edges.</returns>
        public IEnumerable<Edge> ToRelativeNeighborhood()
        {
            bool IsRelativeNeighborEdge(Edge edge)
            {
                var edgeLength = edge.GetPowerLength(); 
                foreach (var nei in edge.GetConnectedVertices())
                { 
                    if (Math.Max(nei.PowerDistanceTo(edge.A), nei.PowerDistanceTo(edge.B)) <= edgeLength)
                        return false;
                }
                return true;
            } 
            return GetEdges().Where(e => IsRelativeNeighborEdge(e));
        }
        /// <summary>
        /// Constructs the relative neighborhood graph, where for each edge there is no other point closer (using power distance) to both than they are to each other.
        /// </summary>
        /// <param name="lines">The resulting lines.</param>
        /// <returns>The relative neighborhood graph edges.</returns>
        public IEnumerable<Edge> ToRelativeNeighborhood(out IEnumerable<Line> lines)
        {
            var edges = ToRelativeNeighborhood();
            lines = edges.Select(e => e.ToLine());
            return edges;
        }
        #endregion
        #region NearestNeighbor 
        /// <summary>
        /// Constructs the nearest neighbor graph, where for each edge there is no other point closer to some of its ends than they are to each other.
        /// </summary> 
        /// <returns>The nearest neighbor graph edges.</returns>
        public IEnumerable<Edge> ToNearestNeighbor()
        {
            bool IsNearestNeighborEdge(Edge edge)
            {
                var edgeLength = edge.GetPowerLength();
                foreach (var nei in edge.GetSharedConnectedVertices())
                {
                    if (Math.Min(nei.PowerDistanceTo(edge.A), nei.PowerDistanceTo(edge.B)) <= edgeLength)
                        return false;
                }
                return true;
            }
            return GetEdges().Where(e => IsNearestNeighborEdge(e));
        }
        /// <summary>
        /// Constructs the nearest neighbor graph, where for each edge there is no other point closer to some of its ends than they are to each other.
        /// </summary>
        /// <param name="lines">The resulting lines.</param>
        /// <returns>The nearest neighbor graph edges.</returns>
        public IEnumerable<Edge> ToNearestNeighbor(out IEnumerable<Line> lines)
        {
            var edges = ToNearestNeighbor();
            lines = edges.Select(e => e.ToLine());
            return edges;
        }
        #endregion
        #region Urquhart
        /// <summary>
        /// Constructs the Urquhart graph, obtained by removing the longest edge (using the power distance) from each triangle.
        /// </summary> 
        /// <returns>The Urquhart graph edges.</returns>
        public IEnumerable<Edge> ToUrquhart()
        {
            var edges = new HashSet<Edge>();

            foreach (var tr in _triangles)
            {
                edges.Add(tr.GetEdges().OrderBy(e => e.GetPowerLength()).Last());
            }

            return GetEdges().Except(edges);
        }
        /// <summary>
        /// Constructs the Urquhart graph, obtained by removing the longest edge from each triangle.
        /// </summary>
        /// <param name="lines">The resulting lines.</param>
        /// <returns>The Urquhart graph edges.</returns>
        public IEnumerable<Edge> ToUrquhart(out IEnumerable<Line> lines)
        {
            var edges = ToUrquhart();
            lines = edges.Select(e => e.ToLine());
            return edges;
        }
        #endregion
        #region MinimumSpanningTree
        /// <summary>
        /// Constructs the Minimum Spanning Tree graph, obtained by connecting all the vertices with the minimum total power length.
        /// </summary> 
        /// <returns>The Minimum Spanning Tree edges.</returns>
        public IEnumerable<Edge> ToMinimumSpanningTree()
        {
            return ToMinimumSpanningTree(e => e.GetPowerLength());
        }
        /// <summary>
        /// Constructs the Minimum Spanning Tree graph, obtained by connecting all the vertices with the minimum total edge metric.
        /// </summary>
        /// <param name="weightMetric">Replace the edge length by a custom function.</param>
        /// <returns>The Minimum Spanning Tree edges.</returns>
        public IEnumerable<Edge> ToMinimumSpanningTree(Func<Edge, double> weightMetric)
        {
            var adjacency = new Dictionary<Vertex, SortedList<Vertex, double>>();
            var empty = new HashSet<Vertex>();
            foreach (var v in _vertices)
            {
                if (!adjacency.ContainsKey(v))
                    adjacency.Add(v, new SortedList<Vertex, double>());
                var sl = adjacency[v];
                var cv = v.GetConnectedVertices();
                if (cv.Any())
                {
                    foreach (var c in cv)
                    {
                        if (!adjacency.ContainsKey(c))
                            adjacency.Add(c, new SortedList<Vertex, double>());
                        if (sl.ContainsKey(c) || adjacency[c].ContainsKey(v))
                            continue;
                        var w = weightMetric(new Edge(v, c));
                        sl.Add(c, w);
                        adjacency[c].Add(v, w);
                    }
                }
                else
                {
                    empty.Add(v);
                }
               
            }

            var tree = new HashSet<Edge>();
            var visited = new HashSet<Vertex>();
            foreach (var e in empty)
                visited.Add(e);
            visited.Add(adjacency.First(k => k.Value.Count > 0).Key);

            do
            {
                var min = double.MaxValue;
                Edge edge = null;
                foreach (var v in visited)
                {
                    foreach (var cw in adjacency[v])
                    {
                        if (!visited.Contains(cw.Key) && cw.Value < min)
                        {
                            min = cw.Value;
                            edge = new Edge(v, cw.Key);
                        }
                    }
                }
                if (edge == null)
                    break;

                tree.Add(edge);
                visited.Add(edge.A);
                visited.Add(edge.B);

            }
            while (visited.Count < _vertices.Count);

            return tree;
        }
        /// <summary>
        /// Constructs the Minimum Spanning Tree graph, obtained by connecting all the edges with the minimum total length.
        /// </summary>
        /// <param name="lines">The resulting lines.</param>
        /// <returns>The Minimum Spanning Tree edges.</returns>
        public IEnumerable<Edge> ToMinimumSpanningTree(out IEnumerable<Line> lines)
        {
            var edges = ToMinimumSpanningTree();
            lines = edges.Select(e => e.ToLine());
            return edges;
        }
        #endregion 
        /// <summary>
        /// Converts to string using format: Delaunay2D [V:{Vertices.Count} T:{Triangles.Count}].
        /// </summary> 
        public override string ToString()
        { 
            return $"Delaunay2D [V:{Vertices.Count} T:{Triangles.Count}]";
        }
        #endregion
        #endregion

    }

    /// <summary>
    /// Triplas of weighted points (<see cref="Vertex"/>) used by <see cref="Delaunay2d"/>.
    /// </summary>
    public class Triangle : IEquatable<Triangle>, IComparable<Triangle>
    {
        #region Fields 
        private int _hash = int.MaxValue;
        private Point3d _center = Point3d.Unset;
        private Vector3d _normal = Vector3d.Unset;
        private Edge[] _edges;
        #endregion

        #region Properties        
        /// <summary>
        /// First vertex.
        /// </summary> 
        public Vertex A { get; protected set; }
        /// <summary>
        /// Second vertex.
        /// </summary> 
        public Vertex B { get; protected set; }
        /// <summary>
        /// Third vertex.
        /// </summary> 
        public Vertex C { get; protected set; }
        /// <summary>
        /// Array of <see cref="A"/>, <see cref="B"/> and <see cref="C"/> vertices.
        /// </summary> 
        public Vertex[] Vertices
        {
            get
            {
                return new Vertex[] { A, B, C };
            }
        }
        /// <summary>
        /// Gets the orthocenter.
        /// </summary> 
        public Point3d Orthocenter { get; protected set; }
        /// <summary>
        /// Gets the orthoradius.
        /// </summary> 
        public double Radius { get; protected set; }
        /// <summary>
        /// Gets the square of <see cref="Radius"/>.
        /// </summary> 
        public double RadiusSquared { get; protected set; }
        /// <summary>
        /// Gets the orthosphere constructed with <see cref="Orthocenter"/> and <see cref="Radius"/>.
        /// </summary> 
        public Sphere Orthosphere { get { return new Sphere(Orthocenter, Radius); } }
        /// <summary>
        /// Gets if some of its vertices is an infinite verex.
        /// </summary> 
        ///  /// <seealso cref="Vertex.IsInfinite"/>
        public bool IsInfinite
        {
            get
            {
                return A.IsInfinite || B.IsInfinite || C.IsInfinite;
            }
        }
        /// <summary>
        /// Gets if at least two of its vertices are external.
        /// </summary> 
        /// <seealso cref="Vertex.IsExternal"/>
        public bool IsExternal
        {
            get
            {
                var e = A.IsExternal ? 1 : 0;
                e += B.IsExternal ? 1 : 0;
                if (e == 2)
                    return true;
                return e == 1 && C.IsExternal;
            }
        }
        #endregion

        #region Constructors        
        /// <summary>
        /// Create a new triangle from three vertices.
        /// </summary> 
        /// <remarks>Vertices b and c can be interchanged to maintain counter-clockwise orientation.</remarks>
        public Triangle(Vertex a, Vertex b, Vertex c)
        {
            if ((b.Position.X - a.Position.X) * (c.Position.Y - a.Position.Y) -
              (c.Position.X - a.Position.X) * (b.Position.Y - a.Position.Y) > 0.0)
            {
                A = a; B = b; C = c;
            }
            else
            {
                A = a; B = c; C = b;
            }
            UpdateOrthoball();
        }
        #endregion

        #region Methods        
        /// <summary>
        /// Calculate the orthocenter and orthoradius.
        /// </summary>
        public void UpdateOrthoball()
        { 
            Orthocenter = Orthocenter(A.Position3d, B.Position3d, C.Position3d, A.Radius, B.Radius, C.Radius);
            Radius = Math.Sqrt(Math.Max(0, Orthocenter.DistanceToSquared(A.Position3d) - A.Weight));
            RadiusSquared = Radius * Radius;
        }
        /// <summary>
        /// Calculate the distance of the triangle orhoballl and a vertex.
        /// </summary>
        /// <remarks>Only the power metric will work properly.</remarks>
        /// <returns>Distance in metric units.</returns> 
        public double DistanceToOrthoball(Vertex v, DelaunayMetric metric = DelaunayMetric.Power)
        {
            switch (metric)
            {
                case DelaunayMetric.Power: 
                    return v.Position3d.DistanceToSquared(Orthocenter) - v.Weight - RadiusSquared;
                case DelaunayMetric.PowerWeight: 
                    return v.Position3d.DistanceToSquared(Orthocenter) - v.WeightSquared - RadiusSquared;
                case DelaunayMetric.Euclidian: 
                    return v.Position3d.DistanceTo(Orthocenter);
                case DelaunayMetric.Multiplicative: 
                    return v.Position3d.DistanceTo(Orthocenter) * v.Weight;
                default:
                    throw new NotImplementedException();
            }
        }
        /// <summary>
        /// Calculate the distance of the triangle orhoballl and a vertex.
        /// </summary>
        /// <remarks>Only the power metric will work properly.</remarks>
        /// <returns>Distance in metric units.</returns> 
        public bool IsInsideOrthoball(Vertex vertex, DelaunayMetric metric = DelaunayMetric.Power)
        {
            switch (metric)
            {
                case DelaunayMetric.Power: 
                    return DistanceToOrthoball(vertex,metric) <= 0;
                case DelaunayMetric.PowerWeight: 
                    return DistanceToOrthoball(vertex, metric) <= 0;
                case DelaunayMetric.Euclidian: 
                    return DistanceToOrthoball(vertex, metric) < Radius;
                case DelaunayMetric.Multiplicative: 
                    return DistanceToOrthoball(vertex, metric) <= Radius;
                default:
                    throw new NotImplementedException();
            }
        }
        /// <summary>
        /// Determine if a point is inside the triangular polygon.
        /// </summary>
        /// <param name="point">Point to evaluate.</param>
        /// <returns>
        /// True if point is inside this triangular polygon, false if not.
        /// </returns>
        public bool IsInsideTriangle(Point3d point)
        {
            return Utils.IsPointInTriangle(A.Position3d, B.Position3d, C.Position3d, point);
        }
        /// <summary>
        /// This method is called when some of its vertices are modified.
        /// </summary>
        public void ClearCaches()
        {
            _hash = int.MaxValue;
            _edges = null;
            _center = Point3d.Unset;
            _normal = Vector3d.Unset;
            UpdateOrthoball();
        }
        #region Get        
        /// <summary>
        /// Gets the connected triangles that share an edge.
        /// </summary> 
        public IEnumerable<Triangle> GetSharedEdgeTriangles()
        {
            var set = new HashSet<Triangle>();
            foreach (var v in Vertices)
            {
                foreach (var t in v.GetAllTriangles())
                {
                    if (t == this)
                        continue;
                    var c = t.Contains(A) ? 1 : 0;
                    c += t.Contains(B) ? 1 : 0;
                    if (c == 2 || (c == 1 && t.Contains(C)))
                        set.Add(t);
                }
            }
            return set;
        }
        /// <summary>
        /// Gets the connected triangles that share one or two vertices.
        /// </summary> 
        public IEnumerable<Triangle> GetSharedVertexTriangles()
        {
            return Vertices.SelectMany(v => v.GetAllTriangles()).Distinct().Except(new Triangle[] { this });
        }
        /// <summary>
        /// Gets the edge shared with other triangle.
        /// </summary>
        /// <param name="other">The other triangle.</param> 
        public Edge SharedEdge(Triangle other)
        {
            return GetEdges().Intersect(other.GetEdges()).FirstOrDefault();
        }
        /// <summary>
        /// Determines whether a vertex is one of its triangle vertices.
        /// </summary>
        /// <param name="vertex">The v.</param>
        /// <returns>
        /// True if the vertex is <see cref="A"/> or <see cref="B"/> or <see cref="C"/>, false if not.
        /// </returns>
        public bool Contains(Vertex vertex)
        {
            return vertex == A || vertex == B || vertex == C;
        }
        /// <summary>
        /// Gets the edges AB, BC and CA.
        /// </summary> 
        public Edge[] GetEdges()
        {
            if(_edges == null)
                _edges = new Edge[] { new Edge(A, B), new Edge(B, C), new Edge(C, A) };
            return _edges;
        }
        /// <summary>
        /// Gets the opposite edge of one of its triangle vertices.
        /// </summary>
        /// <param name="vertex">One of the triangle vertices.</param>
        /// <returns>Triangle edge that do not contains the given triangle vertex.</returns>
        public Edge GetOppositeEdge(Vertex vertex)
        {
            if (vertex == A)
                return new Edge(B, C);
            if (vertex == B)
                return new Edge(C, A);
            if (vertex == C)
                return new Edge(A, B);
            return null;
        }
        /// <summary>
        /// Gets the opposite vertex of one of its triangle edges.
        /// </summary>
        /// <param name="edge">One of the triangle edges.</param>
        /// <returns>Triangle vertex that is not conneted with the given triangle edge.</returns>
        public Vertex GetOppositeVertex(Edge edge)
        {
            if (edge.Contains(A) && edge.Contains(B))
                return C;
            if (edge.Contains(B) && edge.Contains(C))
                return A;
            if (edge.Contains(C) && edge.Contains(A))
                return B;
            return null;
        }
        /// <summary>
        /// Calculate the average position of this triangle.
        /// </summary> 
        public Point3d GetCenter()
        { 
            if(_center == Point3d.Unset)
                _center =(A.Position3d + B.Position3d + C.Position3d) / 3.0;
            return _center;
        }
        /// <summary>
        /// Calculate the normal direction of this triangle.
        /// </summary> 
        public Vector3d GetNormal()
        {
            if(_normal == Vector3d.Unset)
            {
                _normal = A.GetNormal() + B.GetNormal() + C.GetNormal();
                _normal.Unitize();
            } 
            return _normal;
        }
        /// <summary>
        /// Calculate the plane of this triangle using the <see cref="Orthocenter"/> and the <see cref="GetNormal()"/>.
        /// </summary> 
        public Plane GetPlane()
        {
            return new Plane(Orthocenter, GetNormal());
        }
        /// <summary>
        /// Create a Rhino MeshFace using the vertex indices.
        /// </summary> 
        public MeshFace ToMeshFace()
        {
            return new MeshFace(A.Index, B.Index, C.Index);
        }
        /// <summary>
        /// Create a Rhino mesh of a single triangular face.
        /// </summary> 
        public Mesh ToMesh()
        {
            var m = new Mesh();
            m.Vertices.AddVertices(ToPoint3dArray());
            m.Faces.AddFace(0,1,2);
            m.Normals.ComputeNormals();
            m.FaceNormals.ComputeFaceNormals();
            return m;
        }
        /// <summary>
        /// Create a closed polyline with its vertices.
        /// </summary> 
        public Polyline ToPolyline()
        {
            var pln = new Polyline();
            pln.Add(A.Position3d);
            pln.Add(B.Position3d);
            pln.Add(C.Position3d);
            pln.Add(pln[0]);
            return pln;
        }
        /// <summary>
        /// Create a circle using <see cref="GetPlane()"/> and <see cref="Radius"/>.
        /// </summary> 
        public Circle ToCircle()
        {
            return new Circle(GetPlane(), Radius);
        }
        /// <summary>
        /// Create an array of the vertex positions.
        /// </summary> 
        public Point3d[] ToPoint3dArray()
        {
            return Vertices.Select(v => v.Position3d).ToArray();
        }
        /// <summary>
        /// Converts to string using format: Triangle [{A.Index}, {B.Index}, {C.Index}]".
        /// </summary> 
        public override string ToString()
        {
            return $"Triangle [{A.Index}, {B.Index}, {C.Index}]";
        }
        /// <summary>
        /// Return the full triangle info formated.
        /// </summary> 
        public string ToInfo()
        {
            return $"Triangle [A: {A.Index} | B: {B.Index} | C: {C.Index} | Radius: {Radius.ToString("N2")} | External: {IsExternal}]";
        }
        #endregion

        #region Equals     
        public override int GetHashCode()
        {
            if(_hash == int.MaxValue)
            {
                var pts = Vertices.Select(v => v.Position).ToArray();
                Array.Sort(pts.Select(p => p.X).ToArray(), pts);
                Array.Sort(pts.Select(p => p.Y).ToArray(), pts);
                _hash = 1;
                _hash = _hash * 31 + pts[0].GetHashCode();
                _hash = _hash * 31 + pts[1].GetHashCode();
                _hash = _hash * 31 + pts[2].GetHashCode();
            }
            return _hash; 
        }
        public int CompareTo(Triangle other)
        {
            if (Equals(other))
                return 0;
            return Vertices.Min(v => v.Index).CompareTo(other.Vertices.Min(v => v.Index));
        }
        public bool Equals(Triangle t)
        {
            if (Object.ReferenceEquals(t, null))
                return false;
            if (Object.ReferenceEquals(this, t))
                return true;
            if (this.GetType() != t.GetType())
                return false;
            return Contains(t.A) && Contains(t.B) && Contains(t.C);
        }
        public override bool Equals(object obj)
        {
            return Equals(this, (Triangle)obj);
        }
        public static bool operator ==(Triangle a, Triangle b)
        {
            if (Object.ReferenceEquals(a, null))
            {
                if (Object.ReferenceEquals(b, null))
                {
                    return true;
                }
                return false;
            }

            return a.Equals(b);
        } 
        public static bool operator !=(Triangle a, Triangle b)
        {
            return !(a == b);
        }
        #endregion
        #endregion
    }

    /// <summary>
    /// Pair of weighted points (<see cref="Vertex"/>) used by <see cref="Delaunay2d"/>.
    /// </summary> 
    public class Edge : IEquatable<Edge>, IComparable<Edge>
    {
        #region Fields
        private int _hash = int.MaxValue;
        #endregion

        #region Properties        
        /// <summary>
        /// Starting vertex.
        /// </summary> 
        public Vertex A { get; protected set; }
        /// <summary>
        /// End vertex.
        /// </summary> 
        public Vertex B { get; protected set; }
        /// <summary>
        /// Array with <see cref="A"/> and <see cref="B"/>.
        /// </summary> 
        public Vertex[] Vertices { get { return new Vertex[] { A, B }; } }
        /// <summary>
        /// True if both vertex are external and it is being used by a single triangle, false if not.
        /// </summary> 
        /// <seealso cref="Vertex.IsExternal"/>
        public bool IsExternal
        {
            get
            {
                if (!A.IsExternal || !B.IsExternal)
                    return false;
                return GetConnnectedTriangles().Count() <= 1;
            }
        }
        /// <summary>
        /// True if <see cref="A"/> or <see cref="B"/> is infinite.
        /// </summary> 
        /// <seealso cref="Vertex.IsInfinite"/>
        public bool IsInfinite
        {
            get
            {
                return A.IsInfinite || B.IsInfinite;
            }
        }
        #endregion

        #region Constructors        
        /// <summary>
        /// Create a new edge using two vertices.
        /// </summary>
        /// <param name="a">The staring vertex.</param>
        /// <param name="b">The end vertex.</param>
        public Edge(Vertex a, Vertex b)
        {
            A = a; B = b;
        }
        #endregion

        #region Methods        
        /// <summary>
        /// Determines whether this edge is connected to other edge, due they share a vertex.
        /// </summary>
        /// <param name="edge">The other edge.</param> 
        public bool IsConnectedTo(Edge edge)
        {
            return A == edge.A || A == edge.B || B == edge.A || B == edge.B;
        }
        /// <summary>
        /// True if <see cref="A"/> or <see cref="B"/> are the given vertex.
        /// </summary>
        /// <param name="vertex">The vertex to check.</param> 
        public bool Contains(Vertex vertex)
        {
            return A == vertex || B == vertex;
        }
        /// <summary>
        /// Returns a new Edge going from <see cref="B"/> to <see cref="A"/>.
        /// </summary> 
        public Edge Reverse()
        {
            return new Edge(B, A);
        }
        /// <summary>
        /// This method if called when vertex positions or weights are changed. 
        /// </summary>
        /// <remarks>You shouldn't have to call this method.</remarks>
        public void ClearCaches()
        { 
            _hash = int.MaxValue; 
        }
        #region Get        
        /// <summary>
        /// Constructs a line going from <see cref="A"/> to <see cref="B"/>.
        /// </summary> 
        public Line ToLine()
        {
            return new Line(A.Position3d, B.Position3d);
        }
        /// <summary>
        /// Gets the triangles that are using this vertex.
        /// </summary> 
        public IEnumerable<Triangle> GetConnnectedTriangles()
        {
            return A.GetAllTriangles().Intersect(B.GetAllTriangles());
            //_connectedTriangles = A.Triangles.Intersect(B.Triangles); 
        }
        /// <summary>
        /// Gets the vertices connected with <see cref="A"/> and <see cref="B"/>.
        /// </summary> 
        public IEnumerable<Vertex> GetConnectedVertices()
        {
            return A.GetConnectedVertices().Concat(B.GetConnectedVertices()).Except(Vertices).Distinct();
        }
        /// <summary>
        /// Gets the shared vertices connected to <see cref="A"/> and <see cref="B"/>.
        /// </summary> 
        public IEnumerable<Vertex> GetSharedConnectedVertices()
        {
            return A.GetConnectedVertices().Intersect(B.GetConnectedVertices());
        }
        /// <summary>
        /// Gets the normal as the average of its vertex normals.
        /// </summary> 
        public Vector3d GetNormal()
        {
            var n = A.GetNormal() + B.GetNormal();
            n.Unitize();
            return n;
        }
        /// <summary>
        /// Gets the 2d length of this edge, using <see cref="Vertex.Position"/>.
        /// </summary>
        /// <remarks>Use <see cref="GetLength3d()"/> if you want the euclidian distance.</remarks>
        public double GetLength()
        {
            return A.Position.DistanceTo(B.Position);
        }
        /// <summary>
        /// Gets the euclidian length of this edge.
        /// </summary> 
        public double GetLength3d()
        {
            return A.Position3d.DistanceTo(B.Position3d);
        }
        /// <summary>
        /// Gets the power distance of this edge.
        /// </summary> 
        public double GetPowerLength()
        {
            return A.PowerDistanceTo(B);
        }
        /// <summary>
        /// Converts to string using format: Edge [{A.Index}, {B.Index}].
        /// </summary> 
        public override string ToString()
        {
            return $"Edge [{A.Index}, {B.Index}]";
        }
        #endregion
        #region Equals 
        public override int GetHashCode()
        {
            if(_hash == int.MaxValue)
            {
                var pts = Vertices.Select(v => v.Position).ToArray();
                Array.Sort(pts.Select(p => p.X).ToArray(), pts);
                Array.Sort(pts.Select(p => p.Y).ToArray(), pts);
                _hash = 1;
                _hash = _hash * 31 + pts[0].GetHashCode();
                _hash = _hash * 31 + pts[1].GetHashCode(); 
            }
            return _hash; 
        }
        public int CompareTo(Edge other)
        {
            if (Equals(other))
                return 0;
            var va = Vertices.Select(v => v.Index).OrderBy(i=>i).ToArray();
            var vb = other.Vertices.Select(v => v.Index).OrderBy(i => i).ToArray();
            var c0 = va[0].CompareTo(vb[0]);
            if(c0 == 0)
            {
                var c1 = va[1].CompareTo(vb[1]);
                if (c1 == 0)
                {
                    return va[2].CompareTo(vb[2]);
                }
                return c1;
            }
            return c0;
        }
        public bool Equals(Edge e)
        {
            if (Object.ReferenceEquals(e, null))
                return false;
            if (Object.ReferenceEquals(this, e))
                return true;
            if (this.GetType() != e.GetType())
                return false;
            return Contains(e.A) && Contains(e.B);
        }
        public override bool Equals(object obj)
        {
            return Equals(this, (Edge)obj);
        }
        public static bool operator ==(Edge a, Edge b)
        {
            if (Object.ReferenceEquals(a, null))
            {
                if (Object.ReferenceEquals(b, null))
                {
                    return true;
                }
                return false;
            }

            return a.Equals(b);
        }

        public static bool operator !=(Edge a, Edge b)
        {
            return !(a == b);
        }
        #endregion
        #endregion
    }

    /// <summary>
    /// Weighted point used by <see cref="Delaunay2d"/>.
    /// </summary> 
    public class Vertex : IEquatable<Vertex>, IComparable<Vertex>
    {
        #region Fields
        private int _hash = int.MaxValue;
        private Point2d _position;
        private Point3d _position3d;
        private double _weight;
        private int _external = 1;
        protected List<Triangle> _triangles;
        #endregion

        #region Properties        
        /// <summary>
        /// Gets the vertex index.
        /// </summary>
        /// <remarks>Infinite vertices have negative indices.</remarks>
        public int Index { get; protected set; }
        /// <summary>
        /// Gets the position in the triangulation plane coordinates.
        /// </summary> 
        public Point2d Position
        {
            get
            {
                return _position;
            }
            internal set
            {
                if(_position != value)
                {
                    _position = value;
                    ClearCaches();
                }
            }
        }
        /// <summary>
        /// Gets the 3d position.
        /// </summary> 
        public Point3d Position3d
        {
            get
            {
                return _position3d;
            }
            internal set
            {
                if (_position3d != value)
                {
                    _position3d = value;
                    ClearCaches();
                }
            }
        }
        /// <summary>
        /// Gets the weight.
        /// </summary> 
        public double Weight
        {
            get
            {
                return _weight;
            }
            internal set
            {
                if (_weight != value)
                {
                    _weight = value;
                    ClearCaches();
                }
            }
        }
        /// <summary>
        /// Gets the weight squared.
        /// </summary>
        public double WeightSquared { get; protected set; }
        /// <summary>
        /// Gets the radius of this weighted vertex, equal to the square root of <see cref="Weight"/>.
        /// </summary> 
        public double Radius { get; protected set; }
        /// <summary>
        /// Gets the triangles that shared this vertex.
        /// </summary>
        /// <remarks>This doesn't returns the infinite triangles, use <see cref="GetAllTriangles()"/> instead.</remarks>
        public IReadOnlyList<Triangle> Triangles { get { return _triangles.Where(t => !t.IsInfinite).ToList(); } }
        /// <summary>
        /// Gets true if this vertex is not completely surrounded by triangles, if not false.
        /// </summary> 
        public bool IsExternal
        {
            get
            {
                if (_external == -1)
                {
                    _external = 0;
                    for (int i = 0; i < _triangles.Count; i++)
                    {
                        if (_triangles[i].Vertices.Where(v => !v.IsInfinite)
                            .Intersect(_triangles[(i + 1) % _triangles.Count].Vertices
                            .Where(v => !v.IsInfinite)).Count() != 2)
                        {
                            _external = 1;
                            break;
                        }
                    }
                }
                return _external == 1;
            }
        }
        /// <summary>
        /// Gets true if this vertex belongs to the infinite/bounding/auxiliar triangulation, so it is used to construct the triangulation. False if is a common vertex.
        /// </summary> 
        public bool IsInfinite
        {
            get
            {
                return Index < 0;
            }
        }
        /// <summary>
        /// Get or set the epsilon tolerance, used to consider two positions the same if its distance is less than this value.
        /// </summary> 
        public static double Epsilon { get; set; } = 1e-8;
        #endregion

        #region Constructors        
        /// <summary>
        /// Create a new vertex instance.
        /// </summary>
        /// <param name="index">The index in the triangulation.</param>
        /// <param name="point">The 2d position in the triangulation plane coordinates.</param>
        /// <param name="weight">The weight.</param>
        /// <param name="point3d">The 3d position.</param>
        public Vertex(int index, Point2d point, double weight, Point3d point3d)
        {
            Index = index;
            _position = SnapPoint(point, Epsilon);
            _weight = weight;
            WeightSquared = weight * weight;
            Radius = Math.Sqrt(Weight);
            _position3d = SnapPoint(point3d, Epsilon);
            _triangles = new List<Triangle>();
        }
        /// <summary>
        /// Create a copy of other vertex.
        /// </summary>
        /// <param name="other">The other vertex to clone.</param>
        public Vertex(Vertex other)
        {
            Index = other.Index;
            _position = other._position;
            _weight = other._weight;
            WeightSquared = other.WeightSquared;
            Radius = Math.Sqrt(Weight);
            _position3d = other._position3d;
            _triangles = new List<Triangle>();
        }
        #endregion

        #region Methods
        #region Topology        
        /// <summary>
        /// Determines whether this vertex is connected (by an edge) with other vertex.
        /// </summary>
        /// <param name="vertex">Other vertex.</param> 
        public bool IsConnectedTo(Vertex vertex)
        {
            return _triangles.Any(t => t.Contains(vertex));
        }
        /// <summary>
        /// Gets the star edges, those that belongs to the connected triangles but are not connected with this.
        /// </summary> 
        public IEnumerable<Edge> GetStarEdges()
        {
            return Triangles.Select(t => t.GetOppositeEdge(this));
        }
        /// <summary>
        /// Gets the connected vertices.
        /// </summary> 
        public IEnumerable<Vertex> GetConnectedVertices()
        {
            return Triangles.Select(t => t.GetOppositeEdge(this)).SelectMany(e => e.Vertices.Where(v => !v.IsInfinite)).Distinct();
        }
        /// <summary>
        /// Gets the connected edges by constructing them from this to the connected vertices.
        /// </summary> 
        public IEnumerable<Edge> GetConnectedEdges()
        {
            return GetConnectedVertices().Select(v => new Edge(this, v));
        }
        #endregion
        #region Triangles        
        /// <summary>
        /// Adds a triangle that is using this vertex.
        /// </summary> 
        /// <returns>You shouldn't have to call this method.</returns>
        public bool AddTriangle(Triangle triangle)
        {
            if (_triangles.Contains(triangle) || !triangle.Contains(this))
                return false;

            _external = -1;
            if (_triangles.Count == 0)
            {
                _triangles.Add(triangle);
                return true;
            }
            if (_triangles.Count == 1)
            {
                var edge = _triangles[0].GetOppositeEdge(this);
                var candidate = triangle.GetOppositeEdge(this);
                if (edge.Contains(candidate.B))
                {
                    _triangles.Insert(0, triangle);
                }
                else
                {
                    _triangles.Add(triangle);
                }
            }
            else
            {
                _triangles.Add(triangle);
                var plane = new Plane(Position3d, _triangles[0].GetCenter(), _triangles[1].GetCenter());
                _triangles = SortByRadial<Triangle>(_triangles.ToArray(), plane, tr => tr.GetCenter()).ToList();

                for (int i = 0; i < _triangles.Count - 1; i++)
                {
                    if (_triangles[i].B != _triangles[i + 1].A)
                    {
                        _triangles = ShiftList(_triangles, i);
                        break;
                    }
                }
                var fis = _triangles.Where(tr => tr.IsInfinite);
                if (fis.Any())
                {
                    var id = fis.Select(f => _triangles.IndexOf(f)).Min() + 1;
                    if (id != 0)
                        _triangles = ShiftList(_triangles, id);

                }
                var pts = _triangles.Select(tr => tr.GetCenter()).Distinct().Take(2).ToArray();
                if (pts.Length == 2 && !IsCounterClockwise(pts[0], pts[1], Position3d, GetNormal()))
                {
                    _triangles.Reverse();
                }
            }

            return true;
        }
        /// <summary>
        /// Remove a triangle that is using this vertex.
        /// </summary> 
        /// <returns>You shouldn't have to call this method.</returns>
        public bool RemoveTriangle(Triangle t)
        {
            _external = -1;
            return _triangles.Remove(t);
        }
        /// <summary>
        /// Gets all triangles that are using this vertex, including the infinite triangles.
        /// </summary>
        /// <remarks>If you are not interested in the infinite triangles, consider using <see cref="Triangles"/> instead.</remarks>
        public IEnumerable<Triangle> GetAllTriangles()
        {
            return _triangles;
        }
        #endregion
        #region Voronoi        
        /// <summary>
        /// Compute the voronoi cell of this vertex.
        /// </summary>
        /// <param name="cellOrientation">The cell orientation composed by a voronoi cell position and its normal.</param>
        /// <returns>The polyline cell.</returns>
        public Polyline ToVoronoiCell(out List<Tuple<Point3d, Vector3d, int>> cellOrientation)
        {
            cellOrientation = new List<Tuple<Point3d, Vector3d, int>>();
            if (_triangles.Count < 2)
                return null;

            var pts = new Rhino.Collections.Point3dList();
            var center = Point3d.Origin;
            foreach (var t in _triangles)
            {
                var p = t.Orthocenter;
                if (pts.Count == 0)
                {
                    cellOrientation.Add(new Tuple<Point3d, Vector3d, int>(p, t.GetNormal(), 0));
                    pts.Add(p);
                    center += p;
                }
                else
                {
                    if (pts[pts.ClosestIndex(p)].DistanceTo(p) > Vertex.Epsilon)
                    {
                        cellOrientation.Add(new Tuple<Point3d, Vector3d, int>(p, t.GetNormal(), 0));
                        pts.Add(p);
                        center += p;
                    }
                }
            }

            if (cellOrientation.Count < 2)
                return null;

            var cell = new Polyline(pts);
            if (cell.Count > 2)
            {
                center /= pts.Count;
                if (!IsCounterClockwise(pts[0], pts[1], center, GetNormal()))
                {
                    cell.Reverse();
                    cellOrientation.Reverse();
                }
                cell.Add(cell[0]);
            }


            return cell;
        }
        /// <summary>
        ///Compute the voronoi cell of this vertex.
        /// </summary>
        /// <param name="cellOrientation">The cell orientation composed by a voronoi cell position and its normal.</param>
        /// <param name="bounds">The bounds to intersect the cell.</param>
        /// <returns>The polyline cell.</returns>
        public Polyline ToVoronoiCell(out List<Tuple<Point3d, Vector3d, int>> cellOrientation, Polyline bounds = null)
        {
            var cell = ToVoronoiCell(out cellOrientation);
            if (cell == null)
                return null;


            if (bounds != null && bounds.IsValid)
            {
                var ints = Curve.CreateBooleanIntersection(cell.ToNurbsCurve(), bounds.ToNurbsCurve(), Utils.RhinoTolerance);
                if (ints != null && ints.Length > 0 && ints[0].TryGetPolyline(out Polyline pln) && pln != null)
                {
                    var cellOrientation2 = new List<Tuple<Point3d, Vector3d, int>>();
                    if (!Curve.DoDirectionsMatch(cell.ToNurbsCurve(), pln.ToNurbsCurve()))
                    {
                        pln.Reverse();
                        cellOrientation.Reverse();
                    }
                    cell.RemoveAt(cell.Count - 1);
                    var closed = pln.IsClosed ? 1 : 0;
                    var normal = Vector3d.Zero;
                    foreach (var co in cellOrientation)
                        normal += co.Item2;
                    if (normal.IsZero)
                        normal = Vector3d.ZAxis;
                    normal.Unitize();
                    for (int i = 0; i < pln.Count - closed; i++)
                    {
                        var p = pln[i];
                        var cid = cell.ClosestIndex(p);
                        var d = p.DistanceTo(cell[cid]);
                        if (d < 1e-8)
                        {
                            cellOrientation2.Add(cellOrientation[cid]);
                        }
                        else
                        {
                            var p0 = pln[(i - 1 + pln.Count) % pln.Count];
                            var p1 = pln[(i + 1) % pln.Count];
                            var v0 = p0 - p;
                            var v1 = p1 - p;
                            var nor = Vector3d.CrossProduct(v0, v1);
                            if (nor.IsZero)
                                nor = normal;
                            if (nor * normal < 0)
                                nor *= -1;
                            nor.Unitize();
                            var tp = p.DistanceTo(Position3d) < 1e-3 ? 1 : 2;
                            cellOrientation2.Add(new Tuple<Point3d, Vector3d, int>(p, nor, tp));
                        }
                    }
                    cellOrientation = cellOrientation2;
                    return pln;
                }
                else
                {
                    return null;
                }
            }

            return cell;
        }
        #endregion
        #region Get
        /// <summary>
        /// This method is called when the position or weight are changed.
        /// </summary>
        public void ClearCaches()
        {
            _external = 1;
            _hash = int.MaxValue;
            Position = SnapPoint(Position, Epsilon); 
            WeightSquared = Weight * Weight;
            Radius = Math.Sqrt(Weight);
            Position3d = SnapPoint(Position3d, Epsilon);
            foreach (var t in _triangles)
                t.ClearCaches();
        }
        /// <summary>
        /// Compute the 3d euclidean distance.
        /// </summary> 
        public double DistanceTo(Point3d pt)
        {
            return Position3d.DistanceTo(pt);
        }
        /// <summary>
        /// Compute the 2d euclidean distance.
        /// </summary> 
        public double DistanceTo(Point2d pt)
        {
            return Position.DistanceTo(pt);
        }
        /// <summary>
        /// Compute the power distance with other vertex.
        /// </summary>
        /// <param name="other">Other vertex.</param> 
        public double PowerDistanceTo(Vertex other)
        {
            return Position3d.DistanceToSquared(other.Position3d) - WeightSquared - other.WeightSquared;
        }
        /// <summary>
        /// Compute the power distance with other weighted point.
        /// </summary>
        /// <param name="other">Other vertex.</param> 
        public double PowerDistanceTo(Point3d point, double weight)
        {
            return Position3d.DistanceToSquared(point) - WeightSquared - weight * weight;
        }
        /// <summary>
        /// Compute the normal using the cross product of its connected triangles or global Z axis if there is not connected triangle.
        /// </summary> 
        public Vector3d GetNormal()
        {
            if (_triangles.Count == 0)
                return Vector3d.ZAxis;
            Vector3d normal = Vector3d.Zero;
            foreach (var t in _triangles)
            {
                var edge = t.GetOppositeEdge(this);
                var n = Vector3d.CrossProduct(edge.A.Position3d - Position3d, edge.B.Position3d - Position3d);
                n.Unitize();
                normal += n;
            }
            normal.Unitize();
            return normal;
        }
        /// <summary>
        /// Compute the plane using the <see cref="Vertex.Position3d"/> and the <see cref="Vertex.GetNormal()"/>.
        /// </summary> 
        public Plane GetPlane()
        {
            return new Plane(Position3d, GetNormal());
        }
        /// <summary>
        /// Create a circle using the <see cref="Vertex.GetPlane()"/> and the <see cref="Vertex.Radius"/>.
        /// </summary> 
        public Circle ToCircle()
        {
            return new Circle(GetPlane(), Radius);
        }
        /// <summary>
        /// Create a sphere using the <see cref="Vertex.GetPlane()"/> and the <see cref="Vertex.Radius"/>.
        /// </summary> 
        public Sphere ToSphere()
        {
            return new Sphere(GetPlane(), Radius);
        }
        /// <summary>
        /// Return "Vertex [{Index} | {Position}]".
        /// </summary> 
        public override string ToString()
        {
            return string.Format("Vertex [{0} | {1}]", Index, Position);
        }
        /// <summary>
        /// Return the full vertex info formated.
        /// </summary> 
        public string ToInfo()
        {
            return $"Vertex [Index: {Index} | Position: {Position.X.ToString("N2")}, {Position.Y.ToString("N2")} | Weight: {Weight.ToString("N2")} | Triangles: {Triangles.Count} | External: {IsExternal}]";
        }
        #endregion
        #region Equals 
        public override int GetHashCode()
        {
            if(_hash == int.MaxValue)
            {
                var p0 = Position.X > 0 ? 31 : 37;
                var p1 = Position.Y > 0 ? 31 : 37;
                _hash = (p0 + Position.X.GetHashCode()) * (p1 + Position.Y.GetHashCode());
            }
            return _hash;
         
        }
        /// <summary>
        /// Compare equality with other <see cref="Vertex"/> by checking the 2d position epsilon equality using <see cref="Vertex.Epsilon"/>.
        /// </summary>
        /// <param name="v">Other vertex to check equiality.</param>
        /// <returns>True if are the same instance or the same 2d position.</returns>
        public bool Equals(Vertex v)
        {
            if (Object.ReferenceEquals(v, null))
                return false;
            if (Object.ReferenceEquals(this, v))
                return true;
            if (this.GetType() != v.GetType())
                return false;
            return Position.EpsilonEquals(v.Position, Epsilon);
        }
        public override bool Equals(object obj)
        {
            return Equals(this, (Vertex)obj);
        } 
        public int CompareTo(Vertex other)
        {
            if (Equals(other))
                return 0;
            var c0 = Index.CompareTo(other.Index);
            if (c0 != 0)
                return c0;
            return Position3d.CompareTo(other.Position3d);
        } 
        public static bool operator ==(Vertex a, Vertex b)
        {
            if (Object.ReferenceEquals(a, null))
            {
                if (Object.ReferenceEquals(b, null))
                {
                    return true;
                }
                return false;
            }

            return a.Equals(b);
        } 
        public static bool operator !=(Vertex a, Vertex b)
        {
            return !(a == b);
        }
        #endregion
        #endregion
    }

    /// <summary>
    /// Experimental metrics to calculate distance from weighted points.
    /// Just use Power please, the others will probably not work.
    /// This is a residue of a more ambitious implementation attempt that was discarded but which can be taken up again in the future.
    /// </summary>
    public enum DelaunayMetric { Power, PowerWeight, Euclidian, Multiplicative }

}
