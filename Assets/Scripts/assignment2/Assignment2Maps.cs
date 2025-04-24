using UnityEngine;
using System.Collections.Generic;

public class Assignment2Maps : MapCollection
{
    public void StudentMaps(int which)
    {
        /* you can create your own test cases:
              create an outline (list of vertices); coordinates should range from ~-100 to +100; it's helpful to draw this on paper
              define individual nodes, where each node is associated with a polygon; together all polygons should cover the outline
                the polygon vertices should be listed in counterclockwise order
              AddNode can add a polygon to the list of nodes
              FindNeighbors will then connect adjacent nodes as neighbors
           Call EventBus.SetMap **and** EventBus.SetGraph afterwards (in assignment 3, we will skip the SetGraph call)
        
           you can take a look at (which == 2) below to see an example. For example, the first two nodes are:

               AddNode(nodes, new float[] { -20, 0, 20,
                                           0, 0,-20,
                                          20, 0, 20});

              AddNode(nodes, new float[] { -20, 0, 20,
                                          20, 0, 20,
                                           10, 0,50});

            these are both triangles, where last edge of the first triangle goes from (20,0,20) to (-20,0,20), and the first edge of the second triangle
               goes from (-20,0,20) to (20,0,20); this is the same edge (in opposite directions, because each is traversed counterclockwise), and 
               therefore FindNeighbors will connect these two nodes as neighbors.
        */

        if (which == 3)
        {
            // make your own graph here

            // Don't forget to call:
            // EventBus.SetMap(g.outline);
            // EventBus.SetGraph(g);
        }

        // .. maps 3-7 are available for your own test cases

        if (which == 7)
        {
            // make your own graph here
        }

    }


    public override void Generate(int which)
    {
        if (which == 0)
        {
            List<Wall> outline = OutlineFromArray(new float[] {
                   50, 0, 50,
                   -50, 0, 50,
                   -50, 0, -50,
                   50, 0, -50
                });

            GraphNode n = new GraphNode(0, outline);
            Graph g = new Graph();
            g.outline = outline;
            g.all_nodes = new List<GraphNode>() { n };
            EventBus.SetMap(outline);
            EventBus.SetGraph(g);
            return;
        }
        if (which == 1)
        {
            List<Wall> outline = OutlineFromArray(new float[] {
                   50, 0, 50,
                   -50, 0, 50,
                   -50, 0, -30,
                   -25, 0, -30,
                   -25, 0, -50,
                   50, 0, -50
                });


            GraphNode n = new GraphNode(0, new List<Wall> { outline[0], outline[1], outline[2], new Wall(new Vector3(-25, 0, -30), new Vector3(50, 0, 50)) });
            GraphNode n1 = new GraphNode(1, new List<Wall> { new Wall(new Vector3(50, 0, 50), new Vector3(-25, 0, -30)), outline[3], outline[4], outline[5] });
            n.AddNeighbor(n1, 3);
            n1.AddNeighbor(n, 0);
            Graph g = new Graph();
            g.outline = outline;
            g.all_nodes = new List<GraphNode>() { n, n1 };
            EventBus.SetMap(outline);
            EventBus.SetGraph(g);
            return;
        }

        if (which == 2)
        {
            Graph g = new Graph();
            List<GraphNode> nodes = new List<GraphNode>();

            AddNode(nodes, new float[] { -20, 0, 20,
                                           0, 0,-20,
                                          20, 0, 20});

            AddNode(nodes, new float[] { -20, 0, 20,
                                          20, 0, 20,
                                           10, 0,50});

            AddNode(nodes, new float[] {-30, 0, 26,
                                        -40, 0, 4,
                                        -20, 0, -40,
                                          0, 0, -20,
                                        -20, 0, 20
                                         });

            AddNode(nodes, new float[] {-20, 0, -40,
                                        0, 0, -70,
                                        30, 0, -50,
                                        30, 0, -30,
                                         0, 0, -20
                                         });

            AddNode(nodes, new float[] { -40, 0, 4,
                                          -60, 0, -16,
                                           -20, 0, -40});


            FindNeighbors(nodes);

            List<Wall> outer_walls = FindOuterWalls(nodes);
            g.outline = GraphGenerator.GetOutline(outer_walls);
            g.all_nodes = nodes;
            EventBus.SetMap(g.outline);
            EventBus.SetGraph(g);
            return;
        }



        if (which == 8)
        {
            Graph g = GraphGenerator.Generate(false);
            EventBus.SetMap(g.outline);
            EventBus.SetGraph(g);
            return;
        }

        if (which == 9)
        {
            Graph g = GraphGenerator.AdditiveGenerate(false);
            EventBus.SetMap(g.outline);
            EventBus.SetGraph(g);
            return;
        }

        StudentMaps(which);
    }
}
