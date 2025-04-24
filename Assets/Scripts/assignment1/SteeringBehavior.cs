using UnityEngine;
using System.Collections.Generic;
using TMPro;

public class SteeringBehavior : MonoBehaviour
{
    public Vector3 target;
    public KinematicBehavior kinematic;
    public List<Vector3> path;
    // you can use this label to show debug information,
    // like the distance to the (next) target
    public TextMeshProUGUI label;
    
    // Parameters for arrival behavior
    [Header("Arrival Parameters")]
    public float arrivalRadius = 2.0f;        // Radius at which to start slowing down
    public float stopRadius = 0.5f;           // Radius at which to stop completely
    public float angleThreshold = 10.0f;      // Angle threshold for speed adjustment
    public float maxTurnRate = 2.5f;          // Increased from 1.0 to 2.5 for tighter turns

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        kinematic = GetComponent<KinematicBehavior>();
        target = transform.position;
        path = null;
        EventBus.OnSetMap += SetMap;
    }

    // Update is called once per frame
    void Update()
    {
        // Calculate direction to target
        Vector3 directionToTarget = target - transform.position;
        float distanceToTarget = directionToTarget.magnitude;

        // Get the angle between forward direction and target direction
        float angleToTarget = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);

        // Calculate desired rotational velocity based on angle - Modified for tighter turns
        float turnDirection = Mathf.Sign(angleToTarget);
        // Changed from 180.0f to 90.0f to make turns more responsive
        float turnMagnitude = Mathf.Min(Mathf.Abs(angleToTarget) / 90.0f, 1.0f);
        // Added power function to make small angles turn more aggressively
        turnMagnitude = Mathf.Pow(turnMagnitude, 0.7f);
        float desiredRotation = turnDirection * turnMagnitude * kinematic.GetMaxRotationalVelocity() * maxTurnRate;

        // Calculate desired speed based on distance and angle
        float desiredSpeed = kinematic.GetMaxSpeed();

        // Modified angle speed adjustment for tighter turns
        float angleSpeedFactor = Mathf.Cos(Mathf.Deg2Rad * Mathf.Min(Mathf.Abs(angleToTarget), 120.0f));
        // Reduced minimum speed factor to allow for tighter turns
        desiredSpeed *= Mathf.Max(angleSpeedFactor, 0.05f);

        // Implement arrival behavior
        if (distanceToTarget < arrivalRadius)
        {
            // Gradually reduce speed as we get closer to target
            float arrivalFactor = Mathf.Clamp01((distanceToTarget - stopRadius) / (arrivalRadius - stopRadius));
            desiredSpeed *= arrivalFactor;
            // Don't reduce rotation as much during arrival to maintain tight turns
            desiredRotation *= Mathf.Lerp(0.8f, 1.0f, arrivalFactor);

            // Come to a complete stop when very close
            if (distanceToTarget < stopRadius)
            {
                desiredSpeed = 0;
                desiredRotation = 0;
            }
        }

        // Set the desired velocities
        kinematic.SetDesiredRotationalVelocity(desiredRotation);
        kinematic.SetDesiredSpeed(desiredSpeed);

        // Update debug label if available
        if (label != null)
        {
            label.text = $"Distance: {distanceToTarget:F1}m\nAngle: {angleToTarget:F1}°";
        }
    }

    public void SetTarget(Vector3 target)
    {
        this.target = target;
        EventBus.ShowTarget(target);
    }

    public void SetPath(List<Vector3> path)
    {
        this.path = path;
    }

    public void SetMap(List<Wall> outline)
    {
        this.path = null;
        this.target = transform.position;
    }
}
