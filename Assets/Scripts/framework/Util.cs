using UnityEngine;
using System.Collections.Generic;

public class Util 
{
    public static bool PointInPolygon(Vector3 point, List<Wall> polygon)
    {
        int n = 0;
        foreach (var w in polygon)
        {
            if (w.Crosses(point, point + new Vector3(100000, 0, 0)))
            {
                n++;
            }

        }
        return (n % 2 == 1);
    }
}
