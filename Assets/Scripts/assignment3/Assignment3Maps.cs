using UnityEngine;
using System.Collections.Generic;

public class Assignment3Maps : MapCollection
{
    public void StudentMaps(int which)
    {
        /* you can create your own test cases:
              create an outline (list of vertices); coordinates should range from ~-500 to +500; it's helpful to draw this on paper
              put the outline into a float array and call OutlineFromArray
              this will give you a List of Walls, which should pass to EventBus.SetMap

           that's it; your NavMesh code will split the outline and generate the graph
              
        */

        if (which == 4)
        {
            // make your own graph here

            // Don't forget to call:
            // EventBus.SetMap(outline);
            
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

            EventBus.SetMap(outline);
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

            EventBus.SetMap(outline);
            return;
        }

        if (which == 2)
        {
            List<Wall> outline = OutlineFromArray(new float[] {
                   400, 0, 300,
                   340, 0, 380,
                   160, 0, 350,
                   -60, 0, 220,
                   -80, 0, 250,
                   -120, 0, 110,
                   -360, 0, 340,
                   -400, 0, 225,
                   -380, 0, 80,
                   -450, 0, 25,
                   -375, 0, 16,
                   -225, 0, 45,
                   -200, 0, 85,
                   -170, 0, 115,
                   -125, 0, -25,
                   -100, 0, -15, 
                   -80, 0, 15,
                   -60, 0, -30,
                    25, 0, -50,
                   -85, 0, -45,
                   -150, 0, -35,
                   -250, 0, -55,
                   -350, 0, 0,
                   -320, 0, -120,
                   -425, 0, -226,
                   -400, 0, -350,
                   -420, 0, -420,
                   -365, 0, -380,
                   -225, 0, -450,
                   -150, 0, -405,
                   -80, 0, -370,
                   0, 0, -250,
                   55, 0, -300,
                   180, 0, -380,
                   250, 0, -300,
                   330, 0, -333,
                   400, 0, -400,
                   420, 0, -360,
                   340, 0, -120,
                   350, 0, -80,
                   -15, 0, 100,
                   45, 0, 160,
                   350, 0, 140
                });

            EventBus.SetMap(outline);
            return;
            
        }



        if (which == 8)
        {
            // we can also use the graph generator from before and just use the outline
            // hopefully our generated NavMeshes are better than the graphs we had (fewer nodes)
            Graph g = GraphGenerator.Generate(false);
            EventBus.SetMap(g.outline);
            return;
        }

        if (which == 9)
        {
            Graph g = GraphGenerator.AdditiveGenerate(false);
            EventBus.SetMap(g.outline);
            return;
        }

        if (which < 0)
        {
            List<Wall> outline = OutlineFromArray(GenerateMap());
            EventBus.SetMap(outline);
        }

        StudentMaps(which);
    }

    public float[] GenerateMap()
    {
        List<float> coordinates = new List<float>();
        int n = Mathf.RoundToInt(Random.value * 25) + 6;
        float alpha = 0;
        for (int i = 0; i < n || alpha < 270; ++i)
        {
            float distance = Random.value * 150 + 50;
            AddCoordinates(coordinates, Mathf.Cos(alpha * Mathf.Deg2Rad) * distance, 0, Mathf.Sin(alpha * Mathf.Deg2Rad) * distance);
            float remaining = Mathf.Clamp(360 - alpha - 5, 0, 55);
            alpha += Random.value * remaining + 5;
            if (alpha >= 355) break;
        }
        return coordinates.ToArray();
    }

    public void AddCoordinates(List<float> coords, float x, float y, float z)
    {
        coords.Add(x);
        coords.Add(y);
        coords.Add(z);
    }

    public void AddCoordinates(List<float> coords, Vector2 where)
    {
        coords.Add(where.x);
        coords.Add(0);
        coords.Add(where.y);
    }
}
