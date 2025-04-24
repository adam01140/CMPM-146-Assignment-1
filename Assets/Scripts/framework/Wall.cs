using UnityEngine;

public class Wall
{
    public Vector3 start;
    public Vector3 end;
    public Wall(Vector3 s, Vector3 e)
    {
        start = s;
        start.y = 0;
        end = e;
        end.y = 0;
    }

    public Vector3 direction {  get { return (end - start).normalized; } }
    public Vector3 normal { get { return new Vector3(-direction.z, 0, direction.x); } }

    public Vector3 midpoint {  get { return (start + end) / 2; } }

    public float length { get { return (end - start).magnitude; } }

    public bool Crosses(Wall other)
    {
        return Crosses(other.start, other.end);
    }

    public bool Crosses(Vector3 from, Vector3 to)
    {
        

        Vector3 d1 = from - this.start;
        Vector3 d2 = to - this.start;

        // If both vectors are on the same side of the wall
        // their dot products with the normal will have the same sign
        // If they are both positive, or both negative, their product will
        // be positive.

        float dist1 = Vector3.Dot(normal, d1);
        float dist2 = Vector3.Dot(normal, d2);

        // check if the line is parallel to the wall
        if (Mathf.Abs(Vector3.Dot(to - from, normal)) < Mathf.Epsilon)
        {
            // parallel, but not overlapping (normal distance is > epsilon)
            if (Mathf.Abs(dist1) > Mathf.Epsilon) return false;
            
            // otherwise check if line segments overlap
            float point1dist = Vector3.Dot(direction, d1);
            float point2dist = Vector3.Dot(direction, d2);
            // one point is "behind" start, the other one is in front -> there is an overlap
            if (point1dist * point2dist < 0) return true;
            // if both points are ahead of start, one of them needs to be before "end"
            if (point1dist >= 0 && point1dist < length) return true;
            if (point2dist >= 0 && point2dist < length) return true;
            return false;
        }

       

        if (dist1 * dist2 > 0) return false;
        // if the start and end are on different sides, we need to determine 
        // how far the intersection point is along the wall
        // first we determine how far the projections of from and to are 
        // along the wall
        float ldist1 = Vector3.Dot(direction, d1);
        float ldist2 = Vector3.Dot(direction, d2);

        // the distance of the intersection point from the start
        // is proportional to the normal distance of `from`  
        // along the total movement (one of dist1 and dist2 is negative!)
        float t = dist1 / (dist1 - dist2);

        // calculate the intersection as this proportion
        float intersection = ldist1 + t * (ldist2 - ldist1);
        if (intersection < 0 || intersection > length) return false;
        return true;
    }

    public bool Same(Wall other)
    {
        if ((start - other.start).magnitude < Mathf.Epsilon &&
            (end - other.end).magnitude < Mathf.Epsilon) return true;
        if ((start - other.end).magnitude < Mathf.Epsilon &&
            (end - other.start).magnitude < Mathf.Epsilon) return true;
        return false;
    }
}
