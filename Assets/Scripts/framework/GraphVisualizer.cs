using UnityEngine;
using System.Collections.Generic;

public class GraphVisualizer : MonoBehaviour
{
    public GameObject NodeVisualizer;
    public List<GameObject> visualizers;
    public Graph graph;
    bool visible;

    void Clear()
    {
        foreach (var go in visualizers)
        {
            Destroy(go);
        }
        visualizers.Clear();
    }

    void Show()
    {
        foreach (var n in graph.all_nodes)
        {
            GameObject nv = Instantiate(NodeVisualizer, Vector3.zero, Quaternion.identity);
            GraphNodeVisualizer gnv = nv.GetComponent<GraphNodeVisualizer>();
            gnv.SetGraphNode(n);
            visualizers.Add(nv);
        }
    }

    public void ShowGraph(Graph g)
    {
        graph = g;
        if (visible)
        {
            Clear();
            Show();
        }
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        visible = true;
        visualizers = new List<GameObject>();
        EventBus.OnSetGraph += ShowGraph;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void Toggle()
    {
        visible = !visible;
        if (visible)
        {
            Show();
        }
        else
        {
            Clear();
        }
    }
}
