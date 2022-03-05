using System;
using Balma.ADT;
using Unity.Collections;
using Unity.Mathematics;

namespace Balma.Navigation
{
    public partial struct NavMesh
    {
        private const int None = -1;

        public struct NavPoint
        {
            public int triangleIndex;
            public float3 worldPoint;
        }

        public struct Triangle
        {
            public int e0;
            public int e1;
            public int e2;

            public int v0;
            public int v1;
            public int v2;

            public int group;
        }

        public struct HalfEdge
        {
            public int t;

            public int eAdjacent;
            public int ePrevious;
            public int eNext;

            public int v0;
            public int v1;
            public int vOpposite;

            public bool IsBorder => eAdjacent < 0;

            public HalfEdge(int t, int v0, int v1, int vOpposite) : this()
            {
                this.t = t;
                this.v0 = v0;
                this.v1 = v1;
                this.vOpposite = vOpposite;

                eAdjacent = None;
                ePrevious = None;
                ePrevious = None;
            }
        }

        public int VertexCount => vertices.Length;
        public int TriangleCount => triangles.Length;

        private static readonly Triangle InvalidTriangle = new Triangle() {e0 = None, e1 = None, e2 = None, v0 = None, v1 = None, v2 = None};
        private static readonly HalfEdge InvalidEdge = new HalfEdge() {eAdjacent = None, eNext = None, ePrevious = None, t = None, v0 = None, v1 = None};
        private static readonly NavPoint InvalidNavPoint = new NavPoint() {triangleIndex = None};
        
        private NativeHashMap<float3, int> vertexToIndex;
        private NativeHashMap<int2, int> edgeVerticesToIndex;

        private NativeList<Triangle> triangles;
        private NativeList<HalfEdge> edges;
        private NativeList<float3> vertices;

        private NativeMultiHashMap<int, int> vertexToEdgesOut;
        private NativeMultiHashMap<int, int> vertexToEdgesIn;
        
        private int maxLinks;
        private NativeMultiHashMap<int, Link> vertexLinks;

        private GroupingHelper grouping;

        public int GetIsland(int group) => grouping.GetIsland(group);

        public NavMesh(Allocator allocator, int maxLinks = Int32.MaxValue)
        {
            vertexToIndex = new NativeHashMap<float3, int>(1024, allocator);
            edgeVerticesToIndex = new NativeHashMap<int2, int>(1024, allocator);

            triangles = new NativeList<Triangle>(1024, allocator);
            edges = new NativeList<HalfEdge>(1024, allocator);
            vertices = new NativeList<float3>(1024, allocator);
            
            vertexToEdgesOut = new NativeMultiHashMap<int, int>(1024, allocator);
            vertexToEdgesIn = new NativeMultiHashMap<int, int>(1024, allocator);
            
            this.maxLinks = maxLinks;
            vertexLinks = new NativeMultiHashMap<int, Link>(1024, allocator);

            grouping = new GroupingHelper(allocator);
        }

        public void Dispose()
        {
            vertexToIndex.Dispose();
            edgeVerticesToIndex.Dispose();
            triangles.Dispose();
            edges.Dispose();
            vertices.Dispose();
            vertexToEdgesOut.Dispose();
            vertexToEdgesIn.Dispose();
            vertexLinks.Dispose();
            grouping.Dispose();
        }
        
        //TODO Jobify
        public void GenerateLinks()
        {
            vertexLinks.Clear();
            for (int i = 0; i < vertices.Length; i++)
            {
                GenerateLinks(i, vertexLinks, maxLinks);
            }
        }
        
        public NativeMultiHashMap<int, Link>.Enumerator GetLinks(int vertexIndex)
        {
            return vertexLinks.GetValuesForKey(vertexIndex);
        }

        public void AddTriangle(float3 vertex0, float3 vertex1, float3 vertex2)
        {
            var triangle = new Triangle();

            var v0 = AddOrGetVertex(vertex0);
            var v1 = AddOrGetVertex(vertex1);
            var v2 = AddOrGetVertex(vertex2);

            triangle.v0 = v0;
            triangle.v1 = v1;
            triangle.v2 = v2;

            triangle.e0 = AddEdge(new HalfEdge(triangles.Length, v0, v1, v2));
            triangle.e1 = AddEdge(new HalfEdge(triangles.Length, v1, v2, v0));
            triangle.e2 = AddEdge(new HalfEdge(triangles.Length, v2, v0, v1));

            var edge0 = edges[triangle.e0];
            var edge1 = edges[triangle.e1];
            var edge2 = edges[triangle.e2];

            edge0.t = triangles.Length;
            edge1.t = triangles.Length;
            edge2.t = triangles.Length;

            edge0.ePrevious = triangle.e2;
            edge1.ePrevious = triangle.e0;
            edge2.ePrevious = triangle.e1;

            edge0.eNext = triangle.e1;
            edge1.eNext = triangle.e2;
            edge2.eNext = triangle.e0;

            edges[triangle.e0] = edge0;
            edges[triangle.e1] = edge1;
            edges[triangle.e2] = edge2;
            
            triangle.group = GroupingHelper.NO_GROUP;
            JoinEdgeTriangles(edge0, ref triangle);
            JoinEdgeTriangles(edge1, ref triangle);
            JoinEdgeTriangles(edge2, ref triangle);

            triangles.Add(triangle);
        }

        void JoinEdgeTriangles(HalfEdge edge, ref Triangle triangle)
        {
            if(edge.IsBorder) return;
            var otherTriangle = triangles[edges[edge.eAdjacent].t];
            grouping.JoinGroups(ref triangle.@group, ref otherTriangle.@group);
            triangles[edges[edge.eAdjacent].t] = otherTriangle;
        }

        private int AddOrGetVertex(float3 vertex)
        {
            if (vertexToIndex.TryGetValue(vertex, out var index))
            {
                return index;
            }

            vertices.Add(vertex);
            vertexToIndex.Add(vertex, vertices.Length - 1);
            return vertices.Length - 1;
        }

        private int AddEdge(HalfEdge halfEdge)
        {
            vertexToEdgesOut.Add(halfEdge.v0, edges.Length);
            vertexToEdgesIn.Add(halfEdge.v1, edges.Length);
            
            var edgeVertexIndices = new int2(halfEdge.v0, halfEdge.v1);

            if (edgeVerticesToIndex.TryGetValue(edgeVertexIndices.yx, out var adjacentIndex))
            {
                var adjacent = edges[adjacentIndex];
                adjacent.eAdjacent = edges.Length;
                edges[adjacentIndex] = adjacent;

                halfEdge.eAdjacent = adjacentIndex;
            }

            edges.Add(halfEdge);
            edgeVerticesToIndex.Add(edgeVertexIndices, edges.Length - 1);
            return edges.Length - 1;
        }

        public float3 GetPosition(int vertexIndex)
        {
            return vertices[vertexIndex];
        }
        
        public Triangle GetTriangle(int triangleIndex)
        {
            return triangles[triangleIndex];
        }
        
        public HalfEdge GetEdge(int edgeIndex)
        {
            return edges[edgeIndex];
        }
    }
}

