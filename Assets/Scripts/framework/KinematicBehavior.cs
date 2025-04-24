using UnityEngine;
using System.Collections.Generic;

public class KinematicBehavior : MonoBehaviour
{

    public float rotational_velocity;
    public float desired_rotational_velocity;
    public float max_rotational_velocity;
    public float speed;
    public float desired_speed;
    public float max_speed;
    public float linear_acceleration;
    public float rotational_acceleration;

    public bool holonomic;
    public float nonholonomic_factor;

    public Vector3 start_position;
    public Quaternion start_rotation;

    public MapController map;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        start_position = transform.position;
        start_rotation = transform.rotation;
        EventBus.OnSetMap += ResetCar;
    }

    // Update is called once per frame
    void Update()
    {
        if (Mathf.Abs(speed) > 0.01f)
        {
            Vector3 premove = transform.position;
            transform.Translate(0, 0, speed * Time.deltaTime, Space.Self);
            if (map != null && map.CheckCollision(gameObject))
                transform.position = premove;
        }
        if (Mathf.Abs(desired_speed - speed) > 0.01f)
        {
            float acc = desired_speed - speed;
            speed += Mathf.Sign(acc) * linear_acceleration * Time.deltaTime;
            speed = Mathf.Clamp(speed, -max_speed, max_speed);
        }

        if (Mathf.Abs(rotational_velocity) > 0.01f)
        {
            Quaternion prerot = transform.rotation;

            // holonomic: can rotate in place (like a person)
            if (holonomic)
            {
                transform.Rotate(0, rotational_velocity * Time.deltaTime, 0, Space.Self);
            }
            // non-holonomic: Can only rotate when also moving forward/backward (like a car)
            else
            {
                float rot = Mathf.Clamp(rotational_velocity, -nonholonomic_factor * speed, nonholonomic_factor * speed);
                if (Mathf.Abs(rot) > 0.01f)
                {
                    transform.Rotate(0, rot * Time.deltaTime, 0, Space.Self);
                }
            }
            if (map != null && map.CheckCollision(gameObject))
                transform.rotation = prerot;
        }
        if (Mathf.Abs(desired_rotational_velocity - rotational_velocity) > 0.01f)
        {
            float racc = desired_rotational_velocity - rotational_velocity;
            rotational_velocity += Mathf.Sign(racc) * rotational_acceleration * Time.deltaTime;
            rotational_velocity = Mathf.Clamp(rotational_velocity, -max_rotational_velocity, max_rotational_velocity);
        }
    }

    public void SetDesiredSpeed(float des)
    {
        desired_speed = des;
    }

    public void SetDesiredRotationalVelocity(float des)
    {
        desired_rotational_velocity = des;
    }

    public void ResetCar(List<Wall> outline)
    {
        transform.position = start_position;
        transform.rotation = start_rotation;
        desired_rotational_velocity = 0;
        desired_speed = 0;
        speed = 0;
        rotational_velocity = 0;
    }

    public float GetMaxSpeed()
    {
        return max_speed;
    }

    public float GetMaxRotationalVelocity()
    {
        return max_rotational_velocity;
    }
}
