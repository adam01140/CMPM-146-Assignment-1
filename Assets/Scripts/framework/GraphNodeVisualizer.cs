using UnityEngine;
using System.Collections.Generic;

public class GraphNodeVisualizer : MonoBehaviour
{
    public LineRenderer polygon;
    public GameObject center;
    public LineRenderer edges;

    public void SetGraphNode(GraphNode node)
    {
        polygon.positionCount = node.GetPolygon().Count + 1;
        List<Vector3> vertices = new List<Vector3>();
        foreach (Wall w in node.GetPolygon())
        {
            vertices.Add(w.start);
        }
        vertices.Add(vertices[0]);
        
        polygon.SetPositions(vertices.ToArray());
        center.transform.position = node.GetCenter();

        List<GraphNeighbor> neighbors = node.GetNeighbors();
        edges.positionCount = neighbors.Count * 2;
        vertices.Clear();
        foreach(var gn in neighbors)
        {
            vertices.Add(gn.GetNode().GetCenter());
            vertices.Add(node.GetCenter());
        }
        edges.SetPositions(vertices.ToArray());
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
