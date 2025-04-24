using UnityEngine;
using System.Collections.Generic;

public class StandaloneTests : MonoBehaviour
{
    Graph graph;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        var maps = new TestCase();

        graph = maps.MakeTestCase();
        RunTest(0, 8, 87.97f);
        RunTest(8, 0, 87.97f);
        RunTest(2, 9, 79.81f);
        RunTest(9, 2, 79.81f);
        RunTest(1, 10, 114.92f);
        RunTest(10, 1, 114.92f);

    }

    void RunTest(int start, int destination, float reference_length)
    {
        Debug.Log("AStar from node " + start + " to node " + destination);
        (var path, var expanded) = PathFinder.AStar(graph.all_nodes[start], graph.all_nodes[destination], graph.all_nodes[destination].GetCenter());
        float length = PathLength(graph.all_nodes[start].GetCenter(), path);
        Debug.Log("Found path of length " + length + " by expanding " + expanded + " nodes (out of " + graph.all_nodes.Count+")");
        if (length - reference_length > 0.5f)
        {
            Debug.LogWarning("Reference path has length " + reference_length +"; non-optimal path returned");
        }
    }

    float PathLength(Vector3 start, List<Vector3> path)
    {
        float result = 0;
        Vector3 current = start;
        foreach(var v in path)
        {
            result += (v - current).magnitude;
            current = v;
        }
        return result;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    class TestCase : MapCollection
    {
        public Graph MakeTestCase()
        {
            Graph g = new Graph();
            List<GraphNode> nodes = new List<GraphNode>();
            // based on a graph of Austria

            // Bregenz
            AddNode(nodes, new float[] { 0,0,0,
                                         20,0,0,
                                         20,0,25,
                                         15,0,30,
                                         0,0,25
                                       });

            // Innsbruck
            AddNode(nodes, new float[] { 20,0,0,
                                         35,0,0,
                                         45,0,25,
                                         40,0,35,
                                         20,0,25
                                       });

            // Munich
            AddNode(nodes, new float[] { 20,0,25,
                                         40,0,35,
                                         45,0,40,
                                         40,0,50,
                                         40,0,18,
                                         15,0,30
                                         });

            // Salzburg
            AddNode(nodes, new float[] { 40,0,35,
                                         45,0,25,
                                         58,0,28,
                                         60,0,45,
                                         55,0,55,
                                         45,0,40
                                         });

            // Lienz
            AddNode(nodes, new float[] { 35,0,0,
                                         45,0,0,
                                         45,0,25
                                         });

            // Klagenfurt
            AddNode(nodes, new float[] { 45,0,0,
                                         65,0,0,
                                         65,0,15,
                                         58,0,28,
                                         45,0,25
                                         });

            // Bruck
            AddNode(nodes, new float[] { 58,0,28,
                                         65,0,15,
                                         85,0,40,
                                         70,0,45,
                                         60,0,45
                                         });

            // Linz
            AddNode(nodes, new float[] { 60,0,45,
                                         70,0,45,
                                         70,0,70,
                                         60,0,75,
                                         55,0,55
                                         });

            // Wien
            AddNode(nodes, new float[] { 70,0,45,
                                         85,0,40,
                                         100,0,60,
                                         70,0,70
                                         });

            // Eisenstadt
            AddNode(nodes, new float[] { 100,0,0,
                                         100,0,60,
                                         85,0,40
                                         });

            // Graz
            AddNode(nodes, new float[] { 65, 0, 0,
                                         100, 0, 0,
                                         85, 0, 40,
                                         65, 0, 25
                                         });


            FindNeighbors(nodes);

            List<Wall> outer_walls = FindOuterWalls(nodes);
            g.outline = GraphGenerator.GetOutline(outer_walls);
            g.all_nodes = nodes;
            return g;
        }
    }
}
