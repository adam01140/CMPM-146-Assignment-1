using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class MapVisualizer : MonoBehaviour
{
    public GameObject WallPrefab;
    public GameObject NormalPrefab;
    public List<GameObject> walls;
    public List<Wall> outline;
    public bool outline_mode;
    bool show_normals;
    List<GameObject> normals;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        EventBus.OnSetMap += SetMap;
        EventBus.OnMapReset += Show;
        outline_mode = true;
        show_normals = false;
        normals = new List<GameObject>();
        walls = new List<GameObject>();
    }

    public void Show()
    {
        bool normals = show_normals;
        if (normals)
        {
            ToggleNormals();
        }
        GetComponent<LineRenderer>().positionCount = outline.Count;
        List<Vector3> vertices = new List<Vector3>();
        foreach (Wall w in outline)
        {
            vertices.Add(w.start);
        }
        GetComponent<LineRenderer>().SetPositions(vertices.ToArray());
        MakeWalls();
        ViewMode(outline_mode);
        if (normals)
        {
            ToggleNormals();
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (Keyboard.current.qKey.wasPressedThisFrame)
        {
            ViewMode(!outline_mode);
        }
    }

    public void ToggleNormals()
    {
        show_normals = !show_normals;
        if (show_normals)
        {
            foreach (Wall w in outline)
            {
                GameObject new_normal = Instantiate(NormalPrefab, w.midpoint, Quaternion.identity);
                LineRenderer line = new_normal.GetComponent<LineRenderer>();
                line.positionCount = 2;
                line.SetPositions(new Vector3[] { new Vector3(0,0,0), w.normal*3 });
                normals.Add(new_normal);
            }
        }
        else
        {
            foreach (var go in normals)
            {
                Destroy(go);
            }
            normals = new List<GameObject>();
        }
    }

    void SetMap(List<Wall> outline)
    {
        this.outline = outline;
        Show();
    }

    public void ViewMode(bool outline_only)
    {
        outline_mode = outline_only;
        if (outline_only)
        {
            foreach (GameObject obj in walls)
            {
                obj.GetComponent<MeshRenderer>().enabled = false;
            }
        }
        else
        {
            if (walls.Count > 0)
            {
                foreach (GameObject obj in walls)
                {
                    obj.GetComponent<MeshRenderer>().enabled = true;
                }
                return;
            }
        }
    }

    public void MakeWalls()
    {
        if (walls.Count > 0)
        {
            foreach (GameObject obj in walls)
            {
                Destroy(obj);
            }
            walls = new List<GameObject>();
        }
        foreach (Wall w in outline)
        {
            Vector3 dir = w.end - w.start;
            GameObject neww = Instantiate(WallPrefab, (w.start + w.end) / 2, Quaternion.Euler(0, -Mathf.Rad2Deg * Mathf.Atan2(dir.z, dir.x), 0));
            neww.transform.position = new Vector3(neww.transform.position.x, 2, neww.transform.position.z);
            neww.transform.localScale = new Vector3(dir.magnitude, 4, 1);
            walls.Add(neww);
        }
    }
}
