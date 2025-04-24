using UnityEngine;
using System.Collections.Generic;

public class GraphGenerator 
{
    const int WIDTH = 25;
    const int HEIGHT = 25;
    const int MINPATHS = 10;
    const int MAXPATHS = 25;
    const int MINCELLS = WIDTH + HEIGHT;
    const int MAXCELLS = WIDTH*HEIGHT;
    const float GRID_SIZE = 20;
    const int OFFSET = 3;
    const int MIN_SIZE = WIDTH;

    public static Graph AdditiveGenerate(bool noisy)
    {
        bool[] present = new bool[WIDTH * HEIGHT];
        for (int i = 0; i < WIDTH * HEIGHT; ++i)
        {
            present[i] = false;
        }
        Vector2Int center = new Vector2Int(Random.Range(OFFSET, WIDTH - OFFSET), Random.Range(OFFSET, HEIGHT - OFFSET));

        present[center.x + center.y * WIDTH] = true;

        List<Vector2Int> candidates = new List<Vector2Int>();
        candidates.Add(center);
        int paths = Random.Range(MINCELLS, MAXCELLS);
        for (int i = 0; i < paths; ++i)
        {
            int expand_which = Random.Range(0, candidates.Count);
            Vector2Int to_expand = candidates[expand_which];
            List<Vector2Int> next_options = new List<Vector2Int>();
            foreach (var d in new List<Vector2Int> { new Vector2Int(-1,0), new Vector2Int(1, 0), new Vector2Int(0, -1), new Vector2Int(0, 1), })
            {
                if (InBounds(to_expand + d) && Viable(to_expand + d, present))
                    next_options.Add(to_expand + d);
            }
            if (next_options.Count > 0)
            {
                Vector2Int next = next_options[Random.Range(0, next_options.Count)];
                present[next.x + next.y * WIDTH] = true;
                candidates.Add(next);
            }
            else
            {
                candidates.RemoveAt(expand_which);
            }
        }

        Graph result = ToGraph(center, present, noisy);
        return result;
    }

    static bool InBounds(Vector2Int which)
    {
        return (which.x >= 0 && which.x < WIDTH && 
                which.y >= 0 && which.y < HEIGHT);
    }

    static bool Viable(Vector2Int which, bool[] present)
    {
        int neighbors = 0;
        Vector2Int neighbor = new Vector2Int(0, 0) ;
        if (which.x > 0)
        {
            if (present[(which.x - 1) + which.y * WIDTH])
            {
                neighbors++;
                neighbor = new Vector2Int(-1, 0);
            }
        }
        if (which.x < WIDTH - 1)
        {
            if (present[(which.x + 1) + which.y * WIDTH])
            {
                neighbors++;
                neighbor = new Vector2Int(1, 0);
            }
        }
        if (which.y > 0)
        {
            if (present[which.x + (which.y - 1) * WIDTH])
            {
                neighbors++;
                neighbor = new Vector2Int(0, -1);
            }
        }
        if (which.y < HEIGHT - 1)
        {
            if (present[which.x + (which.y + 1) * WIDTH])
            {
                neighbors++;
                neighbor = new Vector2Int(0, 1);
            }
        }
        if (neighbors == 1)
        {
            neighbor = -neighbor;
            if (neighbor.x != 0)
            {
                Vector2Int option = which + neighbor + new Vector2Int(0, -1);
                if (InBounds(option) && present[option.x + option.y * WIDTH]) return false;
                option = which + neighbor + new Vector2Int(0, 1);
                if (InBounds(option) && present[option.x + option.y * WIDTH]) return false;
            }
            else
            {
                Vector2Int option = which + neighbor + new Vector2Int(-1, 0);
                if (InBounds(option) && present[option.x + option.y * WIDTH]) return false;
                option = which + neighbor + new Vector2Int(1, 0);
                if (InBounds(option) && present[option.x + option.y * WIDTH]) return false;
            }
            return true;
        }
        return false;
    }


    public static Graph ToGraph(Vector2Int center, bool[] reachable, bool noisy)
    {
        List<GraphNode> all_nodes = new List<GraphNode>();
        GraphNode[] nodes = new GraphNode[WIDTH * HEIGHT];
       
        Vector3 offset = new Vector3(-center.x * GRID_SIZE, 0, -center.y * GRID_SIZE);
        for (int x = 0; x < WIDTH; ++x)
        {
            for (int y = 0; y < HEIGHT; ++y)
            {
                if (reachable[x + y * WIDTH])
                {
                    nodes[x + y * WIDTH] = MakeNode(x + y * WIDTH, new Vector2Int(x, y), offset, noisy);
                    all_nodes.Add(nodes[x + y * WIDTH]);

                }
            }
        }

        List<Wall> outer_walls = new List<Wall>();

        for (int x = 0; x < WIDTH; ++x)
        {
            for (int y = 0; y < HEIGHT; ++y)
            {

                int node = x + y * WIDTH;
                if (!reachable[node]) continue;

                int left = (x - 1) + y * WIDTH;
                int right = (x + 1) + y * WIDTH;
                int above = x + (y - 1) * WIDTH;
                int below = x + (y + 1) * WIDTH;
                if (y > 0 && reachable[above])
                {
                    nodes[node].AddNeighbor(nodes[above], 0);
                }
                else
                {
                    outer_walls.Add(nodes[node].GetEdge(0));
                }
                if (x < WIDTH - 1 && reachable[right])
                {
                    nodes[node].AddNeighbor(nodes[right], 1);
                }
                else
                {
                    outer_walls.Add(nodes[node].GetEdge(1));
                }
                if (y < HEIGHT - 1 && reachable[below])
                {
                    nodes[node].AddNeighbor(nodes[below], 2);
                }
                else
                {
                    outer_walls.Add(nodes[node].GetEdge(2));
                }
                if (x > 0 && reachable[left])
                {
                    nodes[node].AddNeighbor(nodes[left], 3);
                }
                else
                {
                    outer_walls.Add(nodes[node].GetEdge(3));
                }
            }
        }

        Graph result = new Graph();
        result.outline = GetOutline(outer_walls);
        result.all_nodes = all_nodes;
        return result;
    }

    public static List<Wall> GetOutline(List<Wall> outer_walls)
    {
        List<Wall> outline = new List<Wall>();
        Vector3 start = outer_walls[0].start;
        Vector3 previous = start;
        Vector3 current = outer_walls[0].end;
        outline.Add(outer_walls[0]);
        while ((start - current).magnitude > Mathf.Epsilon)
        {
            bool found = false;
            foreach (Wall w in outer_walls)
            {
                if ((w.start - current).magnitude <= Mathf.Epsilon)
                {
                    if ((w.end - previous).magnitude > Mathf.Epsilon)
                    {
                        outline.Add(w);
                        previous = current;
                        current = w.end;
                        found = true;
                        break;
                    }
                }
            }
            if (!found)
            {
                return null;
            }
            if (outline.Count > 10000)
            {
                return null;
            }
        }
        return outline;
    }

    public static Graph Generate(bool noisy)
    {
        bool[] present = new bool[WIDTH*HEIGHT];
        bool[] reachable = new bool[WIDTH * HEIGHT];
        for (int i = 0; i < WIDTH*HEIGHT; ++i)
        {
            present[i] = true;
            reachable[i] = false;
        }
        Vector2Int center = new Vector2Int(Random.Range(OFFSET, WIDTH - OFFSET), Random.Range(OFFSET, HEIGHT - OFFSET));

        int paths = Random.Range(MINPATHS, MAXPATHS);
        for (int i = 0; i < paths; ++i)
        {
            Vector2Int position;
            float r = Random.value;
            if (r < 0.6f)
                position = new Vector2Int(Random.Range(center.x - OFFSET, center.x + OFFSET + 1), Random.Range(center.y - OFFSET, center.y + OFFSET + 1));
            else if (r < 0.7f)
                position = new Vector2Int(Random.Range(0, OFFSET), Random.Range(0, HEIGHT));
            else if (r < 0.8f)
                position = new Vector2Int(Random.Range(0, WIDTH), Random.Range(0, OFFSET));
            else if (r < 0.9f)
                position = new Vector2Int(Random.Range(WIDTH - OFFSET - 1, WIDTH), Random.Range(0, HEIGHT));
            else
                position = new Vector2Int(Random.Range(0, WIDTH), Random.Range(HEIGHT - OFFSET - 1, HEIGHT));
            
            if (center != position)
            {
                while (position.x >= 0 && position.x < WIDTH && position.y >= 0 && position.y < HEIGHT)
                {
                    present[position.x + position.y * WIDTH] = false;
                    
                    List<Vector2Int> options = new List<Vector2Int>();
                    if (position.x <= center.x + OFFSET)
                    {
                        options.Add(new Vector2Int(-1, 0));
                       
                    }
                    if (position.x >= center.x - OFFSET)
                    {
                        options.Add(new Vector2Int(1, 0));
                    }
                    if (position.y <= center.y + OFFSET)
                    {
                        options.Add(new Vector2Int(0, -1));
                    }
                    if (position.y >= center.y - OFFSET)
                    {
                        options.Add(new Vector2Int(0, 1));
                    }
                    Vector2Int dir = options[Random.Range(0, options.Count)];
                    if (position + dir != center)
                        position += dir;
                }

            }
        }
        FloodFill(present, reachable, center);

        Graph result = ToGraph(center, reachable, noisy);

        if (result.all_nodes.Count < MIN_SIZE) return Generate(noisy);
        return result;
    }

    static Vector3 MakeVector(Vector2Int where, float xoffset, float yoffset, bool noisy)
    {
        if (noisy)
            return new Vector3((where.x + xoffset) * GRID_SIZE + Random.value * 0.1f * GRID_SIZE, 0, (where.y + yoffset) * GRID_SIZE + Random.value * 0.1f * GRID_SIZE);
        return new Vector3((where.x + xoffset) * GRID_SIZE, 0, (where.y + yoffset) * GRID_SIZE);
    }

    static GraphNode MakeNode(int id, Vector2Int where, Vector3 offset, bool noisy)
    {
        List<Wall> polygon = new List<Wall>();

        Vector3[] corners = new Vector3[4] { MakeVector(where, -0.5f, -0.5f, noisy) + offset,
                                             MakeVector(where, +0.5f, -0.5f, noisy) + offset,
                                             MakeVector(where, +0.5f, +0.5f, noisy) + offset,
                                             MakeVector(where, -0.5f, +0.5f, noisy) + offset
                                           };

        for (int i = 0; i < 4; ++i)
        {
            polygon.Add(new Wall(corners[i], corners[(i + 1) % 4]));
        }


        return new GraphNode(id, polygon);
    }

    static void FloodFill(bool[] present, bool[] visited, Vector2Int position)
    {
        visited[position.x + position.y * WIDTH] = true;
        foreach (var d in new Vector2Int[] {  new Vector2Int(-1,0), new Vector2Int(1,0), new Vector2Int(0, -1), new Vector2Int(0,1)})
        {
            Vector2Int np = position + d;
            if (np.x >= 0 && np.x < WIDTH && np.y >= 0 && np.y < HEIGHT)
            {
                if (!visited[np.x + np.y*WIDTH] && present[np.x + np.y*WIDTH])
                {
                    visited[np.x + np.y * WIDTH] = true;
                    FloodFill(present, visited, np);
                }
            }
        }
    }
}
