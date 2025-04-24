using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;
using UnityEngine.EventSystems;

public class CameraControls : MonoBehaviour
{
    public Vector2 move;
    public float speed;
    public float zoom_speed;
    public Plane floor;
    public float zoom;
    public GameObject car;

    public bool accept_path;

    Quaternion orig;
    public bool ingame;
    float orig_y;
    List<Vector3> path;

    bool building_path;
    Vector3 last_known_mouse;
    public LineRenderer path_visualization;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        floor = new Plane(Vector3.up, 0);
        orig = transform.rotation;
        orig_y = transform.position.y;
        OnCameraMode(null);
        path = new List<Vector3>();
    }

    // Update is called once per frame
    void Update()
    {
        if (building_path)
        {
            Vector3? where = GetMousePositionOnMap();
            if (where != null)
            {
                last_known_mouse = where.Value;
                UpdatePathConstructionViz();
            }
        }
        transform.Translate(move.x * Time.deltaTime*speed, 0, move.y * Time.deltaTime*speed, Space.World);
        if (!ingame)
        {
            transform.Translate(0, -zoom * zoom_speed * Time.deltaTime, 0, Space.World);
        }
        PlayerInput inputSystem = GetComponent<PlayerInput>();
        if (EventSystem.current.IsPointerOverGameObject())
        {
            inputSystem.currentActionMap.Disable();
        }
        else
        {
            inputSystem.currentActionMap.Enable();
        }
    }

    private void OnMove(InputValue value)
    {
        move = value.Get<Vector2>();
    }

    private void OnZoom(InputValue value)
    {
        zoom = value.Get<float>();
    }

    private void OnAttack(InputValue value)
    {
        Vector3? world = GetMousePositionOnMap();
        if (world != null)
        {
            building_path = false;
            if (path.Count > 0)
            {
                path.Add(world.Value);
                EventBus.SetPath(path);
                EventBus.ShowTarget(world.Value);
                path = new List<Vector3>();
                UpdatePathConstructionViz();
            }
            else
            {
                EventBus.SetPath(null);
                EventBus.SetTarget(world.Value);
            }
        }
    }


    private Vector3? GetMousePositionOnMap()
    {
        Vector2 mouse = Mouse.current.position.value;
        Ray r = Camera.main.ScreenPointToRay(mouse);
        float d;
        if (floor.Raycast(r, out d))
        {
            return r.origin + d * r.direction;
        }
        return null;
    }

    private void OnRightClick(InputValue value)
    {
        if (!accept_path) return;
        if (value.Get<float>() > 0)
        {
            Vector3? world = GetMousePositionOnMap();
            if (world != null)
            {
                building_path = true;
                path.Add(world.Value);
                UpdatePathConstructionViz();
            }
        }
    }

    private void OnCameraMode(InputValue value)
    {
        if (ingame)
        {
            transform.rotation = Quaternion.Euler(90, 0, 0);
            transform.position += new Vector3(0, 200, 0);
            ingame = false;
        }
        else
        {
            transform.rotation = orig;
            transform.position = new Vector3(transform.position.x, orig_y, transform.position.z);
            ingame = true;
        }
    }

    private void UpdatePathConstructionViz()
    {
        if (!building_path)
        {
            path_visualization.positionCount = 0;
            return;
        }
        List<Vector3> current_path = new List<Vector3>(path);
        if (last_known_mouse != null)
            current_path.Add(last_known_mouse);
        current_path.Insert(0, car.transform.position);
        path_visualization.positionCount = current_path.Count;
        path_visualization.SetPositions(current_path.ToArray());
    }
}
