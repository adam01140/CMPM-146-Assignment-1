using UnityEngine;
using System.Collections.Generic;

public class MapCollection
{
    public virtual void Generate(int which)
    {


    }

    protected void FindNeighbors(List<GraphNode> nodes)
    {
        foreach (var a in nodes)
        {
            List<Wall> a_walls = a.GetPolygon();
            foreach (var b in nodes)
            {
                if (a.GetID() != b.GetID())
                {
                    List<Wall> b_walls = b.GetPolygon();
                    for (int i = 0; i < a_walls.Count; ++i)
                    {
                        for (int j = 0; j < b_walls.Count; ++j)
                        {
                            if (a_walls[i].Same(b_walls[j]))
                            {
                                AddNeighbor(a, b, i, j);
                            }
                        }
                    }
                }
            }
        }
    }

    protected List<Wall> FindOuterWalls(List<GraphNode> nodes)
    {
        List<Wall> outer = new List<Wall>();
        foreach (var a in nodes)
        {
            List<Wall> a_walls = a.GetPolygon();
            foreach (var a_wall in a_walls)
            {
                bool found = false;
                foreach (var b in nodes)
                {
                    if (a.GetID() != b.GetID())
                    {
                        List<Wall> b_walls = b.GetPolygon();

                        foreach (var b_wall in b_walls)
                        {
                            if (a_wall.Same(b_wall))
                            {
                                found = true;
                            }
                        }
                    }
                }
                if (!found) outer.Add(a_wall);
            }
        }
        return outer;
    }

    protected void AddNeighbor(List<GraphNode> nodes, int a, int b, int edge_a, int edge_b)
    {
        nodes[a].AddNeighbor(nodes[b], edge_a);
        nodes[b].AddNeighbor(nodes[a], edge_b);
    }

    protected void AddNeighbor(GraphNode a, GraphNode b, int edge_a, int edge_b)
    {
        a.AddNeighbor(b, edge_a);
    }

    protected void AddNode(List<GraphNode> nodes, float[] vertices)
    {
        int vertex_count = vertices.Length / 3;
        List<Wall> walls = new List<Wall>();
        Vector3 center = new Vector3(0, 0, 0);
        for (int i = 0; i < vertex_count; ++i)
        {
            Vector3 v = new Vector3(vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2]);
            int j = (i + 1) % vertex_count;
            Vector3 w = new Vector3(vertices[j * 3], vertices[j * 3 + 1], vertices[j * 3 + 2]);
            walls.Add(new Wall(v, w));
            center += v;
        }
        GraphNode n = new GraphNode(nodes.Count, center / vertex_count, walls);
        nodes.Add(n);
    }

    protected List<Wall> OutlineFromArray(float[] coordinates)
    {
        var outline = new List<Wall>();
        for (int i = 0; i < coordinates.Length / 3; ++i)
        {
            int j = (i + 1) % (coordinates.Length / 3);
            outline.Add(new Wall(new Vector3(coordinates[i * 3], coordinates[i * 3 + 1], coordinates[i * 3 + 2]),
                                 new Vector3(coordinates[j * 3], coordinates[j * 3 + 1], coordinates[j * 3 + 2])));

        }
        return outline;
    }
}
