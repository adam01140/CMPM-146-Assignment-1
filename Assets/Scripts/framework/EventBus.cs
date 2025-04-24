using UnityEngine;
using System;
using System.Collections.Generic;

public class EventBus 
{
    public static event Action<Vector3> OnTarget;
    public static event Action<Vector3> OnShowTarget;
    public static event Action<List<Vector3>> OnPath;
    public static event Action OnMapReset;
    public static event Action<Graph> OnSetGraph;
    public static event Action<List<Wall>> OnSetMap;
    

    public static void SetTarget(Vector3 target)
    {
        OnTarget?.Invoke(target);
    }

    public static void ShowTarget(Vector3 target)
    {
        OnShowTarget?.Invoke(target);
    }

    public static void SetPath(List<Vector3> path)
    {
        OnPath?.Invoke(path);
    }

    public static void ResetMap()
    {
        OnMapReset?.Invoke();
    }

    public static void SetGraph(Graph graph)
    {
        OnSetGraph?.Invoke(graph);
    }

    public static void SetMap(List<Wall> outline)
    {
        OnSetMap?.Invoke(outline);
    }
}
