using UnityEngine;

public class TargetIndicator : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        EventBus.OnShowTarget += SetTarget;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void SetTarget(Vector3 target)
    {
        transform.position = target;
    }
}
