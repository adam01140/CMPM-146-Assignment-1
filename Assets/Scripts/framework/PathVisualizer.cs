using UnityEngine;
using System.Collections.Generic;

public class PathVisualizer : MonoBehaviour
{
    public GameObject origin;
    public LineRenderer line;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        line = GetComponent<LineRenderer>();
        line.positionCount = 0;
        EventBus.OnPath += SetPath;
        EventBus.OnSetGraph += ResetMap;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void SetPath(List<Vector3> path)
    {
        if (path == null)
        {
            line.positionCount = 0;
            return;
        }
        line.positionCount = path.Count + 1;
        List<Vector3> vertices = new List<Vector3>(path);
        vertices.Insert(0, origin.transform.position);
        line.SetPositions(vertices.ToArray());
    }

    public void ResetMap(Graph g)
    {
        line.positionCount = 0;
    }
}
