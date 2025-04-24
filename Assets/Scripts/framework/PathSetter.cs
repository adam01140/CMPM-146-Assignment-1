using UnityEngine;

public class PathSetter : MonoBehaviour
{
    public SteeringBehavior steering;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        steering = GetComponent<SteeringBehavior>();
        EventBus.OnPath += steering.SetPath;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
