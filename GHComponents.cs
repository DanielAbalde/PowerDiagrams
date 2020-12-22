using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq; 
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using System.Windows.Forms;
using Grasshopper.Kernel.Parameters;
using System.Drawing;
using static SuperDelaunay.Utils;
using Grasshopper.GUI.Canvas;
using Grasshopper.GUI;

namespace SuperDelaunay.GH
{
    public class Comp_Delaunay2D : GH_Component, IGH_VariableParameterComponent
    {
        #region Fields
        internal Delaunay2d DelaunayInstance; 
        #endregion

        #region Properties
        public override Guid ComponentGuid => new Guid("84e042eb-39e3-42e8-8cbf-119b6f203e9d");
        protected override Bitmap Icon => Properties.Resources.delaunay2_24x24;
        public UI.DebuggerForm Controller { get; internal set; }
        public bool IsControllerAlive { get { return Controller != null; } }
        #endregion

        #region Constructors
        public Comp_Delaunay2D() : base("Delaunay", "Delaunay", "Perform the 2D weighted delaunay triangulation", "Mesh", "Triangulation")
        { 
            ValuesChanged();
        }
        #endregion

        #region Parameters
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            UpdateInputParameters(GetInputMode(), GetOutputMode());
        } 
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            UpdateOutputParameters(GetInputMode(), GetOutputMode());
        }

        private void UpdateInputParameters(DelaunayInputMode inputMode, DelaunayOutputMode outputMode)
        {
            IGH_Param CreateDelaunayInput()
            {
                return new Param_GenericObject()
                {
                    Name = "Preexisting Delaunay",
                    NickName = "D",
                    Description = "Preexisting delaunay where to include new points",
                    Access = GH_ParamAccess.item
                };
            }
            IGH_Param CreatePointsInput(bool optional = false)
            {
                return new Param_Point()
                {
                    Name = "Points",
                    NickName = "P",
                    Description = "Points to triangulate",
                    Access = GH_ParamAccess.list,
                    Hidden = true,
                    Optional = optional
                };
            }
            IGH_Param CreateWeightsInput()
            {
                return new Param_Number()
                {
                    Name = "Weights",
                    NickName = "W",
                    Description = "Optional positive weights per point",
                    Access = GH_ParamAccess.list,
                    Optional = true
                };
            }
            IGH_Param CreatePlaneInput()
            {
                var pln = new Param_Plane()
                {
                    Name = "Plane",
                    NickName = "Pl",
                    Description = "Optional plane, if none then the best fit plane will be used",
                    Access = GH_ParamAccess.item,
                    Optional = true
                };
                //pln.SetPersistentData(new GH_Plane(Plane.WorldXY));
                return pln;
            }
            IGH_Param CreateVoronoiInput()
            {
                var v = new Param_Curve()
                {
                    Name = "Bounds",
                    NickName = "B",
                    Description = "Planar polyline to intersect the cells",
                    Access = GH_ParamAccess.item,
                    Optional = true
                };
                return v;
            }
            IGH_Param CreateBetaInput()
            {
                var bk = new Param_Number()
                {
                    Name = "Beta",
                    NickName = "B",
                    Description = "Beta factor.\r\n" +
                       "If beta < 1, it uses the intersection of the two beta circles with radius edge length / beta.\r\n" +
                       "If beta == 1, the diameter of the beta circle is the edge length (Gabriel Graph).\r\n" +
                       "If beta > 1, it uses the union of the two beta circles with radius edge length * beta.",
                    Access = GH_ParamAccess.item
                };
                bk.SetPersistentData(new GH_Number(1.0));
                return bk;
            }
            IGH_Param CreateAlphaInput()
            {
                var ash = new Param_Number()
                {
                    Name = "Alpha",
                    NickName = "A",
                    Description = "Radius of the alpha shape ball",
                    Access = GH_ParamAccess.item
                };
                ash.SetPersistentData(new GH_Number(1.0));
                return ash;
            }
            IGH_Param CreateRadiusInput()
            {
                var ash = new Param_Number()
                {
                    Name = "Radius",
                    NickName = "R",
                    Description = "Radius of the skeleton",
                    Access = GH_ParamAccess.item
                };
                ash.SetPersistentData(new GH_Number(1.0));
                return ash;
            }
            IGH_Param CreateDivisionsInput()
            {
                var ash = new Param_Number()
                {
                    Name = "Division Factor",
                    NickName = "D",
                    Description = "The number of divisions along the edge is the length of the edge multiplied by this value. ",
                    Access = GH_ParamAccess.item
                };
                ash.SetPersistentData(new GH_Number(1.0));
                return ash;
            }

            var delaunayInput = Params.Input.Find(p => p.Name == "Preexisting Delaunay");
            var pointsInput = Params.Input.Find(p => p.Name == "Points") as Param_Point;
            var weightsInput = Params.Input.Find(p => p.Name == "Weights") as Param_Number;
            var planeInput = Params.Input.Find(p => p.Name == "Plane") as Param_Plane;
            var voronoiInput = Params.Input.Find(p => p.Name == "Bounds") as Param_Curve;
            var betaInput = Params.Input.Find(p => p.Name == "Beta") as Param_Number;
            var alphaInput = Params.Input.Find(p => p.Name == "Alpha") as Param_Number;
            var radiusInput = Params.Input.Find(p => p.Name == "Radius") as Param_Number;
            var divisionsInput = Params.Input.Find(p => p.Name == "Division Factor") as Param_Number;

            switch (inputMode)
            {
                case DelaunayInputMode.Create:
                    if (delaunayInput != null)
                        Params.UnregisterInputParameter(delaunayInput);
                    if (pointsInput != null)
                        pointsInput.Optional = false;
                    else
                        Params.RegisterInputParam(CreatePointsInput(), 0);
                    if (weightsInput == null)
                        Params.RegisterInputParam(CreateWeightsInput(), 1);
                    if (planeInput == null)
                        Params.RegisterInputParam(CreatePlaneInput(), 2);

                    break;
                case DelaunayInputMode.Insert:
                    if (delaunayInput == null)
                        Params.RegisterInputParam(CreateDelaunayInput(), 0);
                    if (pointsInput == null)
                        Params.RegisterInputParam(CreatePointsInput(true), 1);
                    if (weightsInput == null)
                        Params.RegisterInputParam(CreateWeightsInput(), 2);
                    if (planeInput != null)
                        Params.UnregisterInputParameter(planeInput);
                    break;
                case DelaunayInputMode.Extract:
                    if (delaunayInput == null)
                        Params.RegisterInputParam(CreateDelaunayInput(), 0);
                    if (pointsInput != null)
                        Params.UnregisterInputParameter(pointsInput);
                    if (weightsInput != null)
                        Params.UnregisterInputParameter(weightsInput);
                    if (planeInput != null)
                        Params.UnregisterInputParameter(planeInput);
                    break;
            }

            switch (outputMode)
            {
                case DelaunayOutputMode.Voronoi:
                    if (voronoiInput == null)
                        Params.RegisterInputParam(CreateVoronoiInput(), 3);
                    if (betaInput != null)
                        Params.UnregisterInputParameter(betaInput);
                    if (alphaInput != null)
                        Params.UnregisterInputParameter(alphaInput);
                    if (radiusInput != null)
                        Params.UnregisterInputParameter(radiusInput);
                    if (divisionsInput != null)
                        Params.UnregisterInputParameter(divisionsInput);
                    break;
                case DelaunayOutputMode.Beta_Skeleton:
                    if (betaInput == null)
                        Params.RegisterInputParam(CreateBetaInput(), 3);
                    if (voronoiInput != null)
                        Params.UnregisterInputParameter(voronoiInput);
                    if (alphaInput != null)
                        Params.UnregisterInputParameter(alphaInput);
                    if (radiusInput != null)
                        Params.UnregisterInputParameter(radiusInput);
                    if (divisionsInput != null)
                        Params.UnregisterInputParameter(divisionsInput);
                    break;
                case DelaunayOutputMode.Alpha_Shape:
                    if (alphaInput == null)
                        Params.RegisterInputParam(CreateAlphaInput(), 3);
                    if (betaInput != null)
                        Params.UnregisterInputParameter(betaInput);
                    if (voronoiInput != null)
                        Params.UnregisterInputParameter(voronoiInput);
                    if (radiusInput != null)
                        Params.UnregisterInputParameter(radiusInput);
                    if (divisionsInput != null)
                        Params.UnregisterInputParameter(divisionsInput);
                    break;
                case DelaunayOutputMode.Thickener_Primal:
                    if (voronoiInput != null)
                        Params.UnregisterInputParameter(voronoiInput);
                    if (betaInput != null)
                        Params.UnregisterInputParameter(betaInput);
                    if (alphaInput != null)
                        Params.UnregisterInputParameter(alphaInput);
                    if (radiusInput == null)
                        Params.RegisterInputParam(CreateRadiusInput());
                    if (divisionsInput == null)
                        Params.RegisterInputParam(CreateDivisionsInput());
                    break;
                case DelaunayOutputMode.Thickener_Dual:
                    if (voronoiInput == null)
                        Params.RegisterInputParam(CreateVoronoiInput());
                    if (betaInput != null)
                        Params.UnregisterInputParameter(betaInput);
                    if (alphaInput != null)
                        Params.UnregisterInputParameter(alphaInput);
                    if (radiusInput == null)
                        Params.RegisterInputParam(CreateRadiusInput());
                    if (divisionsInput == null)
                        Params.RegisterInputParam(CreateDivisionsInput());
                    break;
                default:
                    if (voronoiInput != null)
                        Params.UnregisterInputParameter(voronoiInput);
                    if (betaInput != null)
                        Params.UnregisterInputParameter(betaInput);
                    if (alphaInput != null)
                        Params.UnregisterInputParameter(alphaInput);
                    if (radiusInput != null)
                        Params.UnregisterInputParameter(radiusInput);
                    if (divisionsInput != null)
                        Params.UnregisterInputParameter(divisionsInput);
                    break;
            }

            Params.OnParametersChanged();
        }
        private void UpdateOutputParameters(DelaunayInputMode inputMode, DelaunayOutputMode outputMode)
        {
            IGH_Param CreateDelaunayOutput()
            {
                return new Param_GenericObject()
                {
                    Name = "Delaunay",
                    NickName = "D",
                    Description = "Resulting delaunay object",
                    Access = GH_ParamAccess.item,
                    Hidden = true
                };
            }
            IGH_Param CreateGraphOutput()
            {
                IGH_Param param = null;
                switch (outputMode)
                {
                    case DelaunayOutputMode.Vertices:
                        param = new Param_Point() { Access = GH_ParamAccess.list };
                        break;
                    case DelaunayOutputMode.Edges:
                    case DelaunayOutputMode.Beta_Skeleton:
                    case DelaunayOutputMode.Relative_Neighborhood_Graph:
                    case DelaunayOutputMode.Nearest_Neighbor_Graph:
                    case DelaunayOutputMode.Urquhart_Graph:
                    case DelaunayOutputMode.Minimum_Spanning_Tree:
                        param = new Param_Line() { Access = GH_ParamAccess.list };
                        break;
                    case DelaunayOutputMode.Voronoi:
                    case DelaunayOutputMode.Alpha_Shape:
                        param = new Param_Curve() { Access = GH_ParamAccess.list };
                        break;
                    case DelaunayOutputMode.Convex_Hull:
                        param = new Param_Curve() { Access = GH_ParamAccess.item };
                        break;
                    case DelaunayOutputMode.Mesh:
                    case DelaunayOutputMode.Thickener_Primal:
                    case DelaunayOutputMode.Thickener_Dual:
                        param = new Param_Mesh() { Access = GH_ParamAccess.item };
                        break;
                    default:
                        throw new Exception();
                }
                param.Name = GetModeName(outputMode);
                param.NickName = param.Name[0].ToString();
                param.Description = $"Resulting {param.Name.ToLower()}, {GetModeDescription(outputMode).ToLower()}.";
                return param;
            }
            IGH_Param CreateTopologyOutput()
            {
                IGH_Param param = new Param_Integer();
                param.Name = $"{GetModeName(outputMode)} topology";
                param.NickName = "T";
                param.Access = GH_ParamAccess.tree;
                switch (outputMode)
                {
                    case DelaunayOutputMode.Edges:
                    case DelaunayOutputMode.Beta_Skeleton:
                    case DelaunayOutputMode.Relative_Neighborhood_Graph:
                    case DelaunayOutputMode.Nearest_Neighbor_Graph:
                    case DelaunayOutputMode.Urquhart_Graph:
                    case DelaunayOutputMode.Minimum_Spanning_Tree:
                    case DelaunayOutputMode.Thickener_Primal:
                        param.Description = "For each edge, indices of its points.";
                        break;
                    case DelaunayOutputMode.Vertices:
                    case DelaunayOutputMode.Voronoi:
                        param.Description = "For each cell/point, indices of its connected cells/points.";
                        break;
                    case DelaunayOutputMode.Alpha_Shape:
                        param.Description = "For each polygon, indices of its points.";
                        break;
                    case DelaunayOutputMode.Convex_Hull:
                        param.Description = "Indices of its points.";
                        break;
                    case DelaunayOutputMode.Mesh:
                        param.Description = "For each triangle, indices of its points.";
                        break;
                    case DelaunayOutputMode.Thickener_Dual:
                        param.Description = "For each cell/point, indices of its connected cells/points.";
                        break;
                    default:
                        throw new Exception();
                }

                return param;
            }
            IGH_Param CreateAdditionalOutput()
            {
                switch (outputMode)
                {
                    case DelaunayOutputMode.Vertices:
                        return new Param_Boolean()
                        {
                            Name = "Connected",
                            NickName = "C",
                            Description = "For each point, true if is connected to other vertex.\r\nSome points may not be connected because of nearby points with larger weights.",
                            Access = GH_ParamAccess.list
                        };
                    default:
                        return null;
                }
            }

            var delaunayOutput = Params.Output.Find(p => p.Name == "Delaunay") as Param_GenericObject;
            var graphOutput = Params.Output.Count > 1 ? Params.Output[1] : null;
            var topologyOutput = Params.Output.Count > 2 ? Params.Output[2] : null;
            var additionalOutput = Params.Output.Count > 3 ? Params.Output[3] : null;

            if (delaunayOutput == null)
                Params.RegisterOutputParam(CreateDelaunayOutput(), 0);
            else
                delaunayOutput.Hidden = true;

            if (graphOutput == null)
            {
                Params.RegisterOutputParam(CreateGraphOutput(), 1);
            }
            else
            {
                if (graphOutput.Name != GetModeName(outputMode))
                {
                    var output = CreateGraphOutput();
                    foreach (var recipient in graphOutput.Recipients)
                        recipient.AddSource(output);
                    Params.UnregisterOutputParameter(graphOutput);
                    Params.RegisterOutputParam(output, 1);
                }
            }

            if (topologyOutput == null)
            {
                Params.RegisterOutputParam(CreateTopologyOutput(), 2);
            }
            else
            {
                if (topologyOutput.Name != $"{GetModeName(outputMode)} topology")
                {
                    var output = CreateTopologyOutput();
                    foreach (var recipient in topologyOutput.Recipients)
                        recipient.AddSource(output);
                    Params.UnregisterOutputParameter(topologyOutput);
                    Params.RegisterOutputParam(output, 2);
                }
            }

            if (additionalOutput == null)
            {
                var aoutput = CreateAdditionalOutput();
                if (aoutput != null)
                    Params.RegisterOutputParam(aoutput);
            }
            else
            {
                if (outputMode != DelaunayOutputMode.Vertices && additionalOutput.Name == "Is External")
                {
                    Params.UnregisterOutputParameter(additionalOutput);
                }
            }

            Params.OnParametersChanged();
        }
         
        #region InputMode
        public enum DelaunayInputMode { Create, Insert, Extract }
        private DelaunayInputMode GetInputMode()
        {
            return (DelaunayInputMode)GetValue(nameof(DelaunayInputMode), 0);
        }
        private void SetInputMode(DelaunayInputMode inputMode)
        {
            SetValue(nameof(DelaunayInputMode), (int)inputMode);
        }
        public static string GetModeName(DelaunayInputMode inputMode)
        {
            switch (inputMode)
            {
                case DelaunayInputMode.Create:
                    return "Create new delaunay";
                case DelaunayInputMode.Insert:
                    return "Insert weighted points";
                case DelaunayInputMode.Extract:
                    return "Extract only";
                default:
                    throw new Exception();
            }
        }
        private static string GetModeDescription(DelaunayInputMode inputMode)
        {
            switch (inputMode)
            {
                case DelaunayInputMode.Create:
                    return "Create a new delaunay from weighted points";
                case DelaunayInputMode.Insert:
                    return "Insert weighted points in a existing delaunay";
                case DelaunayInputMode.Extract:
                    return "Extract a subgraph or some property in a existing delaunay";
                default:
                    throw new Exception();
            }
        }
        #endregion

        #region OutputMode
        public enum DelaunayOutputMode { Vertices, Edges, Mesh, Voronoi, Thickener_Primal, Thickener_Dual, Convex_Hull, Alpha_Shape, Beta_Skeleton, Relative_Neighborhood_Graph, Nearest_Neighbor_Graph, Urquhart_Graph, Minimum_Spanning_Tree }
        private DelaunayOutputMode GetOutputMode()
        {
            return (DelaunayOutputMode)GetValue(nameof(DelaunayOutputMode), 1);
        }
        private void SetOutputMode(DelaunayOutputMode outputMode)
        {
            SetValue(nameof(DelaunayOutputMode), (int)outputMode);
        }
        private static string GetModeName(DelaunayOutputMode outputMode)
        {
            return outputMode.ToString().Replace("_", " ");
        }
        private static string GetModeDescription(DelaunayOutputMode outputMode)
        {
            switch (outputMode)
            {
                case DelaunayOutputMode.Vertices:
                    return "The triangulation vertices";
                case DelaunayOutputMode.Edges:
                    return "The triangulation lines";
                case DelaunayOutputMode.Mesh:
                    return "The triangulation mesh";
                case DelaunayOutputMode.Voronoi:
                    return "The delaunay dual";
                case DelaunayOutputMode.Thickener_Primal:
                    return "The delaunay edges with thickness";
                case DelaunayOutputMode.Thickener_Dual:
                    return "The delaunay dual with thickness";
                case DelaunayOutputMode.Convex_Hull:
                    return "The convex bounding polygon";
                case DelaunayOutputMode.Alpha_Shape:
                    return "The bounding polygons where for each edge there is no point within the circle formed by its ends and a given radius";
                case DelaunayOutputMode.Beta_Skeleton:
                    return "The graph where for each edge there is no point within the intersection of the circles formed by its ends and a given factor";
                case DelaunayOutputMode.Relative_Neighborhood_Graph:
                    return "The graph where for each edge there is no other point closer to both than they are to each other";
                case DelaunayOutputMode.Nearest_Neighbor_Graph:
                    return "The graph where for each edge there is no other point closer to some of its ends than they are to each other";
                case DelaunayOutputMode.Urquhart_Graph:
                    return "The graph obtained by removing the longest edge from each triangle";
                case DelaunayOutputMode.Minimum_Spanning_Tree:
                    return "The graph obtained by connecting all the edges with the minimum total length";
                default:
                    throw new Exception();
            }
        }

        protected override void ValuesChanged()
        {
            Message = GetModeName(GetOutputMode());
            if (Message.Length > 15)
                Message = Message.Replace(" ", Environment.NewLine);
            ClearRuntimeMessages();
            foreach (var input in Params.Input)
                input.ClearRuntimeMessages();
        }
        #endregion

        #region Metric
        private DelaunayMetric GetMetric()
        {
            return (DelaunayMetric)GetValue(nameof(DelaunayMetric), 0);
        }
        private void SetMetric(DelaunayMetric metric)
        {
            SetValue(nameof(DelaunayMetric), (int)metric);
        }
        #endregion

        #region IGH_VariableParameterComponent
        public bool CanInsertParameter(GH_ParameterSide side, int index)
        {
            return false;
        }
        public bool CanRemoveParameter(GH_ParameterSide side, int index)
        {
            return false;
        }
        public IGH_Param CreateParameter(GH_ParameterSide side, int index)
        {
            return null;
        }
        public bool DestroyParameter(GH_ParameterSide side, int index)
        {
            return true;
        }
        public void VariableParameterMaintenance()
        {
        }
        #endregion
        #endregion
         
        #region Preview 
        private enum DelaunayPreview { Points, Edges, Weights, Orthoballs, InfiniteTriangles }
        private Dictionary<DelaunayPreview, bool> GetPreviewMode()
        {
            var dic = new Dictionary<DelaunayPreview, bool>();
            foreach (var mode in GetEnumValues<DelaunayPreview>())
                dic.Add(mode, GetPreviewMode(mode));
            return dic;
        }
        private bool GetPreviewMode(DelaunayPreview mode)
        {
            return GetValue($"{nameof(DelaunayPreview)}.{mode}", false);
        }
        private void SetPreviewMode(DelaunayPreview mode, bool value)
        {
            SetValue($"{nameof(DelaunayPreview)}.{mode}", value); 
        }
        private Color GetPreviewColor(DelaunayPreview mode)
        {
            var name = $"{nameof(DelaunayPreview)}.{mode}.Color";
            switch (mode)
            {
                case DelaunayPreview.Points:
                    return GetValue(name, Color.DarkGray);
                case DelaunayPreview.Edges:
                    return GetValue(name, Color.Gray);
                case DelaunayPreview.Weights:
                    return GetValue(name, Color.Orange);
                case DelaunayPreview.Orthoballs:
                    return GetValue(name, Color.Red); 
                case DelaunayPreview.InfiniteTriangles:
                    return GetValue(name, Color.Magenta);
                default:
                    throw new NotImplementedException();
            } 
        }
        private void SetPreviewColor(DelaunayPreview mode, Color color)
        { 
            SetValue($"{nameof(DelaunayPreview)}.{mode}.Color", color);
        }
        private static string GetModeName(DelaunayPreview mode)
        {
            switch (mode)
            {
                case DelaunayPreview.Points:
                    return "Draw points";
                case DelaunayPreview.Edges:
                    return "Draw edges";
                case DelaunayPreview.Weights:
                    return "Draw weights";
                case DelaunayPreview.Orthoballs:
                    return "Draw orthoballs";
                case DelaunayPreview.InfiniteTriangles:
                    return "Draw infinite triangles";
                default:
                    throw new NotImplementedException();
            }
        }
        private static string GetModeDescription(DelaunayPreview mode)
        {
            switch (mode)
            {
                case DelaunayPreview.Points:
                    return "Display the delaunay vertices";
                case DelaunayPreview.Edges:
                    return "Display the delaunay edges";
                case DelaunayPreview.Weights:
                    return "Display the weights as circles";
                case DelaunayPreview.Orthoballs:
                    return "Display the weighted circumcenters as circles";
                case DelaunayPreview.InfiniteTriangles:
                    return "Display the edges of the auxiliar triangles used to build the delaunay";
                default:
                    throw new NotImplementedException();
            }
        }

        Line[] edgesCache;
        Circle[] circleWeightsCache;
        Circle[] orthoballsCache;
        Line[] infiniteTriangles;

        public void Draw(Rhino.Display.DisplayPipeline pipeline, Color mainColor)
        {
            
            if (DelaunayInstance == null)
                return;

            if (IsControllerAlive)
            {
                if (!Controller.HasInstance)
                    return;
                var style = Grasshopper.CentralSettings.PreviewPointStyle;
                if (Controller.ShowTriangulationEdges)
                {
                    var colorEdges = GetPreviewColor(DelaunayPreview.Edges);
                    edgesCache = DelaunayInstance.GetAllEdges().Select(e => e.ToLine()).ToArray();
                    pipeline.DrawLines(edgesCache, colorEdges);
                }
                if (Controller.ShowTriangulationVertices)
                {
                    var colorVertices = GetPreviewColor(DelaunayPreview.Points);
                    foreach (var v in DelaunayInstance.GetAllVertices())
                        pipeline.DrawPoint(v.Position3d, style, 3, colorVertices);
                }
             
                var visibleTriangles = Controller.GetSelectedTriangles();
                if (visibleTriangles.Any())
                {
                    if (Controller.ShowTriangulationWeights)
                    {
                        var visibleVertices = visibleTriangles.SelectMany(t => t.Vertices).Distinct();
                        circleWeightsCache = visibleVertices.Select(v => v.ToCircle()).ToArray();
                        var color = GetPreviewColor(DelaunayPreview.Weights);
                        foreach (var c in circleWeightsCache)
                            pipeline.DrawCircle(c, color);
                    }

                    if (Controller.ShowTriangulationOrthoballs)
                    {
                        orthoballsCache = visibleTriangles.Select(t => t.ToCircle()).ToArray();
                        var color = GetPreviewColor(DelaunayPreview.Orthoballs);
                        foreach (var c in orthoballsCache)
                            pipeline.DrawCircle(c, color);
                        foreach (var c in orthoballsCache)
                            pipeline.DrawPoint(c.Center, style, 3, color);
                    }
                }


                var currentVertex = Controller.CurrentVertex();
                if (currentVertex != null)
                {
                    pipeline.DrawPoint(currentVertex.Position3d, style, 7, mainColor);

                    if (Controller.ShowNextVertexWeight)
                    {
                        pipeline.DrawCircle(currentVertex.ToCircle(), Color.Orange);
                    }
                    IEnumerable<Triangle> badTriangles = null;
                   
                    if (Controller.ShowNextVertexBadTriangles)
                    {
                        badTriangles = DelaunayInstance.FindBadTriangles(currentVertex);
                        foreach (var t in badTriangles)
                        {
                            pipeline.DrawPolyline(t.ToPolyline(), Color.Magenta, 5);
                        }

                      
                    }
                    if (Controller.ShowNextVertexOrthoballs)
                    {
                        if(badTriangles == null)
                            badTriangles = DelaunayInstance.FindBadTriangles(currentVertex);
                        foreach (var t in badTriangles)
                        {
                            pipeline.DrawCircle(t.ToCircle(), Color.DarkRed, 2);
                        }
                    }

                    if (Controller.ShowNextVertexNewTriangles)
                    {
                        if (badTriangles == null)
                            badTriangles = DelaunayInstance.FindBadTriangles(currentVertex);
                        var newTriangles = DelaunayInstance.Polygonize(badTriangles, currentVertex);
                        foreach (var t in newTriangles)
                        {
                            pipeline.DrawPolyline(t.ToPolyline(), Color.Green, 5);
                        }
                    }
                   
                    
                   
                }


            }
            else
            {
                if (GetPreviewMode(DelaunayPreview.Points))
                {
                    var color = Attributes.Selected ? mainColor : GetPreviewColor(DelaunayPreview.Points);
                    var style = Grasshopper.CentralSettings.PreviewPointStyle;
                    foreach (var v in DelaunayInstance.Vertices)
                    {
                        pipeline.DrawPoint(v.Position3d, style, 5, color);
                    }
                }

                if (GetPreviewMode(DelaunayPreview.Edges))
                {
                    if (edgesCache == null)
                        edgesCache = DelaunayInstance.GetEdges().Select(e => e.ToLine()).ToArray();
                    var color = Attributes.Selected ? mainColor : GetPreviewColor(DelaunayPreview.Edges);
                    pipeline.DrawLines(edgesCache, color);
                }

                if (GetPreviewMode(DelaunayPreview.Weights))
                {
                    if (circleWeightsCache == null)
                    {
                        circleWeightsCache = DelaunayInstance.Vertices.Select(v =>v.ToCircle()).ToArray();
                    }
                    var color = Attributes.Selected ? mainColor : GetPreviewColor(DelaunayPreview.Weights);
                    foreach (var c in circleWeightsCache)
                    {
                        pipeline.DrawCircle(c, color);
                    }
                }

                if (GetPreviewMode(DelaunayPreview.Orthoballs))
                {
                    if (orthoballsCache == null)
                    {
                        orthoballsCache = DelaunayInstance.Triangles.Select(t => t.ToCircle()).ToArray();
                    }
                    var color = Attributes.Selected ? mainColor : GetPreviewColor(DelaunayPreview.Orthoballs);
                    foreach (var c in orthoballsCache)
                    {
                        pipeline.DrawCircle(c, color);
                    }
                }

                if (GetPreviewMode(DelaunayPreview.InfiniteTriangles))
                {
                    if (infiniteTriangles == null)
                    {
                        infiniteTriangles = DelaunayInstance.GetAllEdges().Where(e => e.A.IsInfinite || e.B.IsInfinite).Select(e => e.ToLine()).ToArray();
                    }
                    var color = Attributes.Selected ? mainColor : GetPreviewColor(DelaunayPreview.InfiniteTriangles);
                    pipeline.DrawLines(infiniteTriangles, color);
                }
            }
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);

            if(!IsControllerAlive)
            Draw(args.Display, Attributes.Selected ? args.WireColour_Selected : args.WireColour);
        }

        public override void ClearData()
        {
            base.ClearData();
            if(!IsControllerAlive)
              DelaunayInstance = null;
            edgesCache = null;
            circleWeightsCache = null;
            orthoballsCache = null;
            infiniteTriangles = null;
        }
        #endregion

        #region Menu 
        protected override void AppendAdditionalComponentMenuItems(ToolStripDropDown menu)
        {
            var inputMode = GetInputMode();
            foreach (var mode in GetEnumValues<DelaunayInputMode>())
            {
                var tsmi = Menu_AppendItem(menu, GetModeName(mode), Menu_ChangeInputMode, true, inputMode == mode);
                tsmi.ToolTipText = $"When checked, {GetModeDescription(mode).ToLower()}.";
                tsmi.Tag = mode;
            }

            Menu_AppendSeparator(menu);
            var outputMode = GetOutputMode();
            foreach (var mode in GetEnumValues<DelaunayOutputMode>())
            { 
                var tsmi = Menu_AppendItem(menu, GetModeName(mode), Menu_ChangeOutputMode, true, outputMode == mode);
                tsmi.ToolTipText = $"When checked, returns {GetModeDescription(mode).ToLower()}.";
                tsmi.Tag = mode;
            }

            Menu_AppendSeparator(menu);
            var previewMode = GetPreviewMode(); 
            foreach (var mode in GetEnumValues<DelaunayPreview>())
            {  
                var tsmi = Menu_AppendItem(menu, GetModeName(mode), Menu_ChangePreview, true, previewMode[mode]);
                tsmi.ToolTipText = $"When checked, {GetModeDescription(mode).ToLower()}.";
                tsmi.Tag = mode;
                Menu_AppendColourPicker(tsmi.DropDown, GetPreviewColor(mode), Menu_ChangePreviewColor);
            }
             
#if DEBUG
            Menu_AppendSeparator(menu);
            if(DelaunayInstance != null)
            {
                var currentMetric = GetMetric();
                var mem = Menu_AppendItem(menu, "Metric");
                foreach (var mode in GetEnumValues<DelaunayMetric>())
                {
                    Menu_AppendItem(mem.DropDown, mode.ToString(), Menu_ChangeMetric, true, currentMetric == mode).Tag = mode;
                }
            }
#endif
        }

        private void Menu_ChangeInputMode(object sender, EventArgs e)
        {
            var currentMode = GetInputMode();
            var newMode = (DelaunayInputMode)((ToolStripMenuItem)sender).Tag;
            if (currentMode != newMode)
            {
                RecordUndoEvent("Change delaunay mode");
                UpdateInputParameters(newMode, GetOutputMode());
                SetInputMode(newMode);
                ExpireSolution(true);
            }

        }
        private void Menu_ChangeOutputMode(object sender, EventArgs e)
        {
            var currentGraph = GetOutputMode();
            var newGraph = (DelaunayOutputMode)((ToolStripMenuItem)sender).Tag;
            if (currentGraph != newGraph)
            {
                RecordUndoEvent("Change delaunay graph");
                var mode = GetInputMode();
                UpdateInputParameters(mode, newGraph);
                UpdateOutputParameters(mode, newGraph);
                SetOutputMode(newGraph);
                ExpireSolution(true);
            }

        }
        private void Menu_ChangePreview(object sender, EventArgs e)
        {
            RecordUndoEvent("Change delaunay preview");
            var menu = (ToolStripMenuItem)sender;
            var mode = (DelaunayPreview)menu.Tag;
            SetPreviewMode(mode, !menu.Checked); 
            var parent = menu.GetCurrentParent();
            if (parent != null)
                parent.Hide();
            menu.HideDropDown();
            ExpirePreview(true);
        }
        private void Menu_ChangePreviewColor(Grasshopper.GUI.GH_ColourPicker picker, Grasshopper.GUI.Base.GH_ColourPickerEventArgs e)
        { 
            RecordUndoEvent("Change delaunay preview");
            var mode = (DelaunayPreview)((ToolStripDropDownMenu)picker.Parent).OwnerItem.Tag;
            SetPreviewColor(mode, e.Colour); 
            ExpirePreview(true);
        }
        private void Menu_ChangeMetric(object sender, EventArgs e)
        {
            if (DelaunayInstance != null)
            {
                RecordUndoEvent("Change delaunay metric");
                SetMetric((DelaunayMetric)((ToolStripMenuItem)sender).Tag);
                ExpireSolution(true);
            }
        }
        #endregion

        #region Solution 
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var points = new List<Point3d>();
            var weights = new List<double>();
            var plane = Plane.WorldXY;

            bool ValidateWeightedPoints(DelaunayInputMode mode)
            {
                List<GH_Point> pts = new List<GH_Point>();
                List<GH_Number> wgs = new List<GH_Number>();
                
                if (!DA.GetDataList("Points", pts))
                    return false;

                for (int i = 0; i < pts.Count; i++)
                {
                    var g = pts[i];
                    if(g == null)
                    { 
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Null point at {i}.");
                        return false; 
                    }
                    if (!g.IsValid)
                    {
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Invalid point at {i}: {g.IsValidWhyNot}");
                        return false;
                    }
                    points.Add(g.Value);
                }
                 
                if (!DA.GetDataList("Weights", wgs))
                {
                    weights = Repeat(0.0, points.Count).ToList();
                }
                else
                {
                    if (points.Count != wgs.Count)
                    {
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The number of points and weights must be the same");
                        return false;
                    }

                    for (int i = 0; i < wgs.Count; i++)
                    {
                        var g = wgs[i];
                        if (g == null)
                        {
                            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Null weight at {i}.");
                            return false;
                        }
                        if (!g.IsValid)
                        {
                            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Invalid weight at {i}: {g.IsValidWhyNot}");
                            return false;
                        }
                        weights.Add(g.Value);
                    }
                }
                   
                if(mode == DelaunayInputMode.Create)
                {
                    if (!DA.GetData("Plane", ref plane))
                        plane = FitPlane(points);
                } 

                return true;
            }
         
            switch (GetInputMode())
            {
                case DelaunayInputMode.Create:
                    if (!ValidateWeightedPoints(DelaunayInputMode.Create))
                        return;

                    if (IsControllerAlive)
                    {
                        if(DelaunayInstance == null)
                        DelaunayInstance = new Delaunay2d();
                        Controller.InsertPoints(points, weights); 
                    }
                    else
                    {
                        DelaunayInstance = new Delaunay2d(points.ToArray(), weights.ToArray(), plane, GetMetric());
                    }
                   
                    break;
                case DelaunayInputMode.Insert:
                    if (!DA.GetData(0, ref DelaunayInstance))
                        DelaunayInstance = null;
                    if (!ValidateWeightedPoints(DelaunayInputMode.Insert))
                        return;

                    if (IsControllerAlive)
                    {
                        if (!Controller.HasInstance || !Controller.IsSameInstance(points, weights))
                        {
                            DelaunayInstance = new Delaunay2d(DelaunayInstance);
                            Controller.InsertPoints(points, weights);
                        }
                    }
                    else
                    {
                        DelaunayInstance = new Delaunay2d(DelaunayInstance);
                        DelaunayInstance.Insert(points.ToArray(), weights.ToArray());
                    }
                 
                    break;
                case DelaunayInputMode.Extract:
                    if (DA.GetData(0, ref DelaunayInstance) && DelaunayInstance != null)
                    {
                        if (IsControllerAlive)
                        {
                            if (!Controller.HasInstance || !Controller.IsSameInstance(points, weights))
                            {
                                DelaunayInstance = new Delaunay2d(DelaunayInstance);
                                Controller.InsertPoints(points, weights);
                            }
                        }
                        else
                        {
                            DelaunayInstance = new Delaunay2d(DelaunayInstance); 
                        }
                    }
                    else
                    {
                        DelaunayInstance = null;
                    }
                    break;
            }

            if (DelaunayInstance == null)
                return;
            if (!DelaunayInstance.IsPlanar())
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Points should be on the same plane, some outputs could fail");
             
            DA.SetData(0, new Goo_Delaunay2D(DelaunayInstance));

            switch (GetOutputMode())
            {
                case DelaunayOutputMode.Vertices:
                    var vertices = new List<Point3d>();
                    var neighbov = new GH_Structure<GH_Integer>();
                    var connected = new List<bool>();
                    var vertexs = DelaunayInstance.Vertices;
                    for (int i = 0; i < vertexs.Count; i++)
                    {
                        var v = vertexs[i];
                        vertices.Add(v.Position3d);
                        var cv = v.GetConnectedVertices();
                        neighbov.AppendRange(cv.Select(ve => new GH_Integer(ve.Index)), new GH_Path(DA.Iteration, i));
                        connected.Add(cv.Any());
                    }
                    DA.SetDataList(1, vertices);
                    DA.SetDataTree(2, neighbov);
                    DA.SetDataList(3, connected);
                    break;
                case DelaunayOutputMode.Edges:
                    var edges = DelaunayInstance.GetEdges(out IEnumerable<Line> lines);
                    DA.SetDataList(1, lines);
                    DA.SetDataTree(2, ToTopologyTree(edges, DA.Iteration));
                    break;
                case DelaunayOutputMode.Mesh:
                    DA.SetData(1, DelaunayInstance.ToMesh());
                    DA.SetDataTree(2, ToTopologyTree(DelaunayInstance.Triangles, DA.Iteration));
                    break;
                case DelaunayOutputMode.Voronoi:
                    Curve voronoiBounds = null;
                    DA.GetData("Bounds", ref voronoiBounds);
                    DA.SetDataList(1, DelaunayInstance.ToVoronoi(voronoiBounds));
                    DA.SetDataTree(2, ToTopologyTree(DelaunayInstance.Vertices.Select(v => v.GetConnectedVertices()), DA.Iteration));

                    break;
                case DelaunayOutputMode.Thickener_Primal:
                    double radius1 = 1.0;
                    double divisionFactorP = 0.0;
                    if (DA.GetData("Radius", ref radius1) && DA.GetData("Division Factor", ref divisionFactorP))
                    {
                        DA.SetData(1, DelaunayInstance.ToThickenerPrimal(radius1, divisionFactorP));
                        DA.SetDataTree(2, ToTopologyTree(DelaunayInstance.GetEdges(), DA.Iteration));
                    }
                    break;
                case DelaunayOutputMode.Thickener_Dual:
                    double radius2 = 1.0;
                    double divisionFactorD = 0.0;
                    Curve dualBounds = null;
                    DA.GetData("Bounds", ref dualBounds);
                    if (DA.GetData("Radius", ref radius2) && DA.GetData("Division Factor", ref divisionFactorD))
                    {
                        DA.SetData(1, DelaunayInstance.ToThickenerDual(radius2, divisionFactorD, dualBounds));
                        DA.SetDataTree(2, ToTopologyTree(DelaunayInstance.Vertices.Select(v => v.GetConnectedVertices()), DA.Iteration));
                    }
                    break;
                case DelaunayOutputMode.Convex_Hull:
                    var ch = DelaunayInstance.ToConvexHull(out Polyline chp);
                    DA.SetData(1, chp.ToNurbsCurve());
                    DA.SetDataTree(2, ToTopologyTree(ch, DA.Iteration));
                    break;
                case DelaunayOutputMode.Alpha_Shape:
                    double alpha = 1.0;
                    if (DA.GetData("Alpha", ref alpha))
                    {
                        var ashape = DelaunayInstance.ToAlphaShape(alpha, out List<Polyline> shapes);
                        DA.SetDataList(1, shapes);
                        DA.SetDataTree(2, ToTopologyTree(ashape, DA.Iteration));
                    }
                    break;
                case DelaunayOutputMode.Beta_Skeleton:
                    double beta = 1.0;
                    if (DA.GetData("Beta", ref beta))
                    {
                        var bsk = DelaunayInstance.ToBetaSkeleton(beta, out IEnumerable<Line> bskLines);
                        DA.SetDataList(1, bskLines);
                        DA.SetDataTree(2, ToTopologyTree(bsk, DA.Iteration));
                    }
                    break;
                case DelaunayOutputMode.Relative_Neighborhood_Graph:
                    var rng = DelaunayInstance.ToRelativeNeighborhood(out IEnumerable<Line> rngLines);
                    DA.SetDataList(1, rngLines);
                    DA.SetDataTree(2, ToTopologyTree(rng, DA.Iteration));
                    break;
                case DelaunayOutputMode.Nearest_Neighbor_Graph:
                    var nng = DelaunayInstance.ToNearestNeighbor(out IEnumerable<Line> nngLines);
                    DA.SetDataList(1, nngLines);
                    DA.SetDataTree(2, ToTopologyTree(nng, DA.Iteration));
                    break;
                case DelaunayOutputMode.Urquhart_Graph:
                    var ug = DelaunayInstance.ToUrquhart(out IEnumerable<Line> ugLines);
                    DA.SetDataList(1, ugLines);
                    DA.SetDataTree(2, ToTopologyTree(ug, DA.Iteration));
                    break;
                case DelaunayOutputMode.Minimum_Spanning_Tree:
                    var mst = DelaunayInstance.ToMinimumSpanningTree(out IEnumerable<Line> mstLines);
                    DA.SetDataList(1, mstLines);
                    DA.SetDataTree(2, ToTopologyTree(mst, DA.Iteration));
                    break;
            }

        }
        #endregion

        #region Attributes
        public override void CreateAttributes()
        {
#if DEBUG
            m_attributes = new CompAtt_Delaunay2D(this);
#else
            base.CreateAttributes();
#endif
        }
        public class CompAtt_Delaunay2D : Grasshopper.Kernel.Attributes.GH_ComponentAttributes
        {
            
            public CompAtt_Delaunay2D(Comp_Delaunay2D owner) : base(owner) { }
            private Comp_Delaunay2D GetOwner()
            {
                return Owner as Comp_Delaunay2D;
            }
            public override GH_ObjectResponse RespondToMouseDoubleClick(GH_Canvas sender, GH_CanvasMouseEvent e)
            {
                if(e.Button == MouseButtons.Left)
                {
                    var owner = GetOwner();
                    var f = owner.Controller = new UI.DebuggerForm(owner);
                    f.Show(Grasshopper.Instances.DocumentEditor);
                    owner.ExpireSolution(true);
                }
                return base.RespondToMouseDoubleClick(sender, e);
            }
        }
#endregion
    }
     
    public class Goo_Delaunay2D : GH_GeometricGoo<Delaunay2d>, IGH_PreviewData
    {
        #region Properties
                public override bool IsValid => m_value != null;
                public override string TypeName => "Delaunay2D";
                public override string TypeDescription => "Weighted delaunay triangulation";
                public override BoundingBox Boundingbox => m_value != null ? m_value.BoundingBox.BoundingBox : BoundingBox.Unset;
                public BoundingBox ClippingBox => Boundingbox;
        #endregion

        #region Constructors
                public Goo_Delaunay2D() : base() { }
                public Goo_Delaunay2D(Delaunay2d val) : base(val) { }
                public Goo_Delaunay2D(Goo_Delaunay2D other)
                {
                    if (other.Value != null)
                    {
                        m_value = new Delaunay2d(other.Value);
                    }
                }
        #endregion

        #region Methods 
        public override string ToString()
        {
            if (m_value == null)
                return $"Null {TypeName}";
            return m_value.ToString();
        }

        public override IGH_GeometricGoo DuplicateGeometry()
        {
            return new Goo_Delaunay2D(this);
        }

        public override BoundingBox GetBoundingBox(Transform xform)
        {
            if (m_value == null)
                return BoundingBox.Unset;
            var bb = m_value.BoundingBox.BoundingBox;
            bb.Transform(xform);
            return bb;
        }

        public override IGH_GeometricGoo Transform(Transform xform)
        {
            if (m_value == null)
                return null;
            m_value.Transform(xform);
            return this;
        }

        public override IGH_GeometricGoo Morph(SpaceMorph xmorph)
        {
            throw new NotImplementedException($"{TypeName} doesn't support morphs");
        }

        public override bool CastFrom(object source)
        {
            if (source is Delaunay2d d)
            {
                m_value = d;
                return true;
            }
            else if (source is Goo_Delaunay2D g)
            {
                m_value = g.Value;
                return true;
            }
            return base.CastFrom(source);
        }
        public override bool CastTo<Q>(ref Q target)
        {
            if (typeof(Q) == typeof(Delaunay2d))
            {
                if (m_value != null)
                    target = (Q)(object)new Delaunay2d(m_value);
                return true;
            }
            else if (typeof(Q) == typeof(Goo_Delaunay2D))
            {
                if (m_value != null)
                    target = (Q)(object)new Goo_Delaunay2D(new Delaunay2d(m_value));
                return true;
            }
            return base.CastTo<Q>(ref target);
        }
        #endregion

        #region IGH_PreviewData 
                public void DrawViewportWires(GH_PreviewWireArgs args)
                {
                    if (m_value == null)
                        return;

                    foreach(var e in m_value.GetEdges())
                    {
                        args.Pipeline.DrawLine(e.ToLine(), args.Color);
                    }
                    var style = Grasshopper.CentralSettings.PreviewPointStyle;
                    foreach (var v in m_value.Vertices)
                    {
                        args.Pipeline.DrawPoint(v.Position3d, style, 5, args.Color);
                    }
                }

                public void DrawViewportMeshes(GH_PreviewMeshArgs args)
                { 
                }

        #endregion
    }

    public class Plugin_WeightedDelaunay : GH_AssemblyInfo
    {
        public override string Name => "Super Delaunay 2d";
        public override string Description => "Dynamic Weighted Delaunay 2d";
        public override Guid Id => new Guid("0baa4891-84aa-4d05-b1e5-71d7210c616a"); 
        public override string AuthorName => "Daniel Gonzalez Abalde";
        public override string AuthorContact => "dga_3@hotmail.com";
        public override Bitmap Icon => null;
    }
}
