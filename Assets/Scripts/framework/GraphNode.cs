using UnityEngine;
using System.Collections.Generic;

public class Graph
{
    public List<GraphNode> all_nodes;
    public List<Wall> outline;
}

public class GraphNeighbor
{
    GraphNode neighbor;
    Wall wall;

    public GraphNeighbor(GraphNode neighbor, Wall wall)
    {
        this.neighbor = neighbor;
        this.wall = wall;
    }

    public GraphNode GetNode()
    {
        return neighbor;
    }

    public Wall GetWall()
    {
        return wall;
    }
}

public class GraphNode 
{
    Vector3 center;
    List<GraphNeighbor> neighbors;
    List<Wall> polygon;
    int id;
    

    public GraphNode(int id, Vector3 center, List<Wall> polygon)
    {
        this.id = id;
        this.center = center;
        this.polygon = polygon;
        this.neighbors = new List<GraphNeighbor>();
    }

    public GraphNode(int id, List<Wall> polygon)
    {
        this.id = id;
        this.polygon = polygon;
        this.center = new Vector3(0, 0, 0);
        foreach (var w in polygon)
        {
            this.center += w.start;
        }
        this.center /= polygon.Count;
        this.neighbors = new List<GraphNeighbor>();
    }

    public void AddNeighbor(GraphNode neighbor, int edge)
    {
        neighbors.Add(new GraphNeighbor(neighbor, polygon[edge]));
    }

    public GraphNeighbor GetNeighbor(int edge)
    {
        return neighbors[edge];
    }

    public List<GraphNeighbor> GetNeighbors()
    {
        return neighbors;
    }

    public Wall GetEdge(int edge)
    {
        return polygon[edge];
    }

    public List<Wall> GetPolygon()
    {
        return polygon;
    }

    public Vector3 GetCenter()
    {
        return center;
    }

    public int GetID()
    {
        return id;
    }
}
