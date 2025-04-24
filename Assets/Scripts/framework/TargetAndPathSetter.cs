using UnityEngine;

public class TargetAndPathSetter : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        var steering = GetComponent<SteeringBehavior>();
        EventBus.OnTarget += steering.SetTarget;
        EventBus.OnPath += steering.SetPath;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

}
