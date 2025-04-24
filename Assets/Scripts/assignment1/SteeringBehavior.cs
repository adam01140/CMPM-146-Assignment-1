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
    public float maxTurnRate = 2.5f;          // Maximum turn rate multiplier

    // Parameters for path following
    [Header("Path Following Parameters")]
    public float waypointRadius = 1.0f;       // How close we need to get to a waypoint
    public float lookAheadDistance = 2.0f;    // How far to look ahead for smooth turns
    public float sharpTurnAngle = 60.0f;      // Angle that defines a sharp turn
    private int currentWaypointIndex = -1;    // Current waypoint index (-1 means no path)
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        kinematic = GetComponent<KinematicBehavior>();
        target = transform.position;
        path = null;
        currentWaypointIndex = -1;
        EventBus.OnSetMap += SetMap;
    }

    // Update is called once per frame
    void Update()
    {
        // Handle path following if we have a path
        if (path != null && path.Count > 0)
        {
            UpdatePathFollowing();
            return;
        }

        // Single target seeking behavior
        UpdateTargetSeeking();
    }

    void UpdateTargetSeeking()
    {
        // Calculate direction to target
        Vector3 directionToTarget = target - transform.position;
        float distanceToTarget = directionToTarget.magnitude;

        // Get the angle between forward direction and target direction
        float angleToTarget = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);

        // Calculate desired rotational velocity based on angle
        float turnDirection = Mathf.Sign(angleToTarget);
        float turnMagnitude = Mathf.Min(Mathf.Abs(angleToTarget) / 90.0f, 1.0f);
        turnMagnitude = Mathf.Pow(turnMagnitude, 0.7f);
        float desiredRotation = turnDirection * turnMagnitude * kinematic.GetMaxRotationalVelocity() * maxTurnRate;

        // Calculate desired speed based on distance and angle
        float desiredSpeed = kinematic.GetMaxSpeed();

        // Modified angle speed adjustment for tighter turns
        float angleSpeedFactor = Mathf.Cos(Mathf.Deg2Rad * Mathf.Min(Mathf.Abs(angleToTarget), 120.0f));
        desiredSpeed *= Mathf.Max(angleSpeedFactor, 0.05f);

        // Implement arrival behavior
        if (distanceToTarget < arrivalRadius)
        {
            float arrivalFactor = Mathf.Clamp01((distanceToTarget - stopRadius) / (arrivalRadius - stopRadius));
            desiredSpeed *= arrivalFactor;
            desiredRotation *= Mathf.Lerp(0.8f, 1.0f, arrivalFactor);

            if (distanceToTarget < stopRadius)
            {
                desiredSpeed = 0;
                desiredRotation = 0;
            }
        }

        // Set the desired velocities
        kinematic.SetDesiredRotationalVelocity(desiredRotation);
        kinematic.SetDesiredSpeed(desiredSpeed);

        // Update debug label
        if (label != null)
        {
            label.text = $"Distance: {distanceToTarget:F1}m\nAngle: {angleToTarget:F1}°";
        }
    }

    void UpdatePathFollowing()
    {
        if (currentWaypointIndex == -1)
        {
            currentWaypointIndex = 0;
        }

        // Get current and next waypoint positions
        Vector3 currentWaypoint = path[currentWaypointIndex];
        Vector3 nextWaypoint = currentWaypointIndex < path.Count - 1 ? path[currentWaypointIndex + 1] : currentWaypoint;

        // Calculate distances and directions
        Vector3 directionToCurrent = currentWaypoint - transform.position;
        float distanceToCurrent = directionToCurrent.magnitude;
        
        // Look ahead to calculate the turn angle
        Vector3 currentToNext = nextWaypoint - currentWaypoint;
        float turnAngle = Vector3.Angle(directionToCurrent, currentToNext);

        // Calculate target point (look ahead for smooth turns)
        Vector3 targetPoint = currentWaypoint;
        if (distanceToCurrent < lookAheadDistance && currentWaypointIndex < path.Count - 1)
        {
            // Blend between current and next waypoint based on distance
            float blend = 1.0f - (distanceToCurrent / lookAheadDistance);
            targetPoint = Vector3.Lerp(currentWaypoint, nextWaypoint, blend);
        }

        // Calculate steering towards target point
        Vector3 directionToTarget = targetPoint - transform.position;
        float angleToTarget = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);

        // Calculate desired rotation
        float turnDirection = Mathf.Sign(angleToTarget);
        float turnMagnitude = Mathf.Min(Mathf.Abs(angleToTarget) / 90.0f, 1.0f);
        turnMagnitude = Mathf.Pow(turnMagnitude, 0.7f);
        float desiredRotation = turnDirection * turnMagnitude * kinematic.GetMaxRotationalVelocity() * maxTurnRate;

        // Calculate base speed
        float desiredSpeed = kinematic.GetMaxSpeed();

        // Adjust speed based on turn angle and distance
        float angleSpeedFactor = Mathf.Cos(Mathf.Deg2Rad * Mathf.Min(Mathf.Abs(angleToTarget), 120.0f));
        float turnSpeedFactor = turnAngle > sharpTurnAngle ? 
            Mathf.Lerp(0.3f, 1.0f, (180.0f - turnAngle) / (180.0f - sharpTurnAngle)) : 1.0f;
        
        desiredSpeed *= Mathf.Min(angleSpeedFactor, turnSpeedFactor);
        desiredSpeed = Mathf.Max(desiredSpeed, kinematic.GetMaxSpeed() * 0.1f); // Maintain minimum speed

        // Progress to next waypoint if close enough
        if (distanceToCurrent < waypointRadius && currentWaypointIndex < path.Count - 1)
        {
            currentWaypointIndex++;
        }
        // Handle path completion
        else if (distanceToCurrent < stopRadius && currentWaypointIndex == path.Count - 1)
        {
            desiredSpeed = 0;
            desiredRotation = 0;
        }

        // Set the desired velocities
        kinematic.SetDesiredRotationalVelocity(desiredRotation);
        kinematic.SetDesiredSpeed(desiredSpeed);

        // Update debug label
        if (label != null)
        {
            label.text = $"Waypoint: {currentWaypointIndex + 1}/{path.Count}\n" +
                        $"Distance: {distanceToCurrent:F1}m\n" +
                        $"Turn Angle: {turnAngle:F1}°";
        }
    }

    public void SetTarget(Vector3 target)
    {
        this.target = target;
        this.path = null;
        this.currentWaypointIndex = -1;
        EventBus.ShowTarget(target);
    }

    public void SetPath(List<Vector3> path)
    {
        this.path = path;
        this.currentWaypointIndex = -1;
        if (path != null && path.Count > 0)
        {
            this.target = path[0];
        }
    }

    public void SetMap(List<Wall> outline)
    {
        this.path = null;
        this.currentWaypointIndex = -1;
        this.target = transform.position;
    }
}
