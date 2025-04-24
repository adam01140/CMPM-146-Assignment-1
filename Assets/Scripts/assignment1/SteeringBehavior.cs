using UnityEngine;
using System.Collections.Generic;
using TMPro;

public class SteeringBehavior : MonoBehaviour
{
    public Vector3 target;
    public KinematicBehavior kinematic;
    public List<Vector3> path;
    public TextMeshProUGUI label;
    
    [Header("Arrival Parameters")]
    public float arrivalRadius = 2.0f;
    public float stopRadius = 0.5f;
    public float angleThreshold = 10.0f;
    public float maxTurnRate = 2.5f;

    [Header("Path Following Parameters")]
    public float waypointRadius = 1.0f;
    public float lookAheadDistance = 2.0f;
    public float sharpTurnAngle = 60.0f;
    private int currentWaypointIndex = -1;
    
    void Start()
    {
        kinematic = GetComponent<KinematicBehavior>();
        target = transform.position;
        path = null;
        currentWaypointIndex = -1;
        EventBus.OnSetMap += SetMap;
    }

    void Update()
    {
        if (path != null && path.Count > 0)
        {
            UpdatePathFollowing();
            return;
        }
        UpdateTargetSeeking();
    }

    void UpdateTargetSeeking()
    {
        Vector3 directionToTarget = target - transform.position;
        float distanceToTarget = directionToTarget.magnitude;

        float angleToTarget = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);

        float turnDirection = Mathf.Sign(angleToTarget);
        float turnMagnitude = Mathf.Min(Mathf.Abs(angleToTarget) / 90.0f, 1.0f);
        turnMagnitude = Mathf.Pow(turnMagnitude, 0.7f);
        float desiredRotation = turnDirection * turnMagnitude * kinematic.GetMaxRotationalVelocity() * maxTurnRate;

        float desiredSpeed = kinematic.GetMaxSpeed();

        float angleSpeedFactor = Mathf.Cos(Mathf.Deg2Rad * Mathf.Min(Mathf.Abs(angleToTarget), 120.0f));
        desiredSpeed *= Mathf.Max(angleSpeedFactor, 0.05f);

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

        kinematic.SetDesiredRotationalVelocity(desiredRotation);
        kinematic.SetDesiredSpeed(desiredSpeed);

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

        Vector3 currentWaypoint = path[currentWaypointIndex];
        Vector3 nextWaypoint = currentWaypointIndex < path.Count - 1 ? path[currentWaypointIndex + 1] : currentWaypoint;

        Vector3 directionToCurrent = currentWaypoint - transform.position;
        float distanceToCurrent = directionToCurrent.magnitude;
        
        Vector3 currentToNext = nextWaypoint - currentWaypoint;
        float turnAngle = Vector3.Angle(directionToCurrent, currentToNext);

        Vector3 targetPoint = currentWaypoint;
        if (distanceToCurrent < lookAheadDistance && currentWaypointIndex < path.Count - 1)
        {
            float blend = 1.0f - (distanceToCurrent / lookAheadDistance);
            targetPoint = Vector3.Lerp(currentWaypoint, nextWaypoint, blend);
        }

        Vector3 directionToTarget = targetPoint - transform.position;
        float angleToTarget = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);

        float turnDirection = Mathf.Sign(angleToTarget);
        float turnMagnitude = Mathf.Min(Mathf.Abs(angleToTarget) / 90.0f, 1.0f);
        turnMagnitude = Mathf.Pow(turnMagnitude, 0.7f);
        float desiredRotation = turnDirection * turnMagnitude * kinematic.GetMaxRotationalVelocity() * maxTurnRate;

        float desiredSpeed = kinematic.GetMaxSpeed();

        float angleSpeedFactor = Mathf.Cos(Mathf.Deg2Rad * Mathf.Min(Mathf.Abs(angleToTarget), 120.0f));
        float turnSpeedFactor = turnAngle > sharpTurnAngle ? 
            Mathf.Lerp(0.3f, 1.0f, (180.0f - turnAngle) / (180.0f - sharpTurnAngle)) : 1.0f;
        
        desiredSpeed *= Mathf.Min(angleSpeedFactor, turnSpeedFactor);
        desiredSpeed = Mathf.Max(desiredSpeed, kinematic.GetMaxSpeed() * 0.1f);

        if (distanceToCurrent < waypointRadius && currentWaypointIndex < path.Count - 1)
        {
            currentWaypointIndex++;
        }
        else if (distanceToCurrent < stopRadius && currentWaypointIndex == path.Count - 1)
        {
            desiredSpeed = 0;
            desiredRotation = 0;
        }

        kinematic.SetDesiredRotationalVelocity(desiredRotation);
        kinematic.SetDesiredSpeed(desiredSpeed);

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
