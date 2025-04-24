using UnityEngine;
using System.Collections.Generic;
using System.Collections;
using UnityEngine.InputSystem;

public class MapController : MonoBehaviour
{
    public bool graph_mode;

    MapCollection maps;
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        if (graph_mode)
        {
            maps = new Assignment2Maps();
        }
        else
        {
            maps = new Assignment3Maps();
        }
        StartCoroutine(ShowMap());
    }

    IEnumerator ShowMap()
    {
        yield return new WaitForEndOfFrame();
        maps.Generate(0);
    }

    

    // Update is called once per frame
    void Update()
    {
        
        if (Keyboard.current.digit1Key.wasPressedThisFrame)
        {
            maps.Generate(1);
        }
        if (Keyboard.current.digit2Key.wasPressedThisFrame)
        {
            maps.Generate(2);
        }
        if (Keyboard.current.digit3Key.wasPressedThisFrame)
        {
            maps.Generate(3);
        }
        if (Keyboard.current.digit4Key.wasPressedThisFrame)
        {
            maps.Generate(4);
        }
        if (Keyboard.current.digit5Key.wasPressedThisFrame)
        {
            maps.Generate(5);
        }
        if (Keyboard.current.digit6Key.wasPressedThisFrame)
        {
            maps.Generate(6);
        }
        if (Keyboard.current.digit7Key.wasPressedThisFrame)
        {
            maps.Generate(7);
        }
        if (Keyboard.current.digit8Key.wasPressedThisFrame)
        {
            maps.Generate(8);
        }
        if (Keyboard.current.digit9Key.wasPressedThisFrame)
        {
            maps.Generate(9);
        }
        if (Keyboard.current.digit0Key.wasPressedThisFrame)
        {
            maps.Generate(0);
        }
        if (Keyboard.current.gKey.wasPressedThisFrame)
        {
            maps.Generate(-1);
        }
    }   

    public bool CheckCollision(GameObject car)
    {
        Collider[] coll = Physics.OverlapBox(car.transform.position, car.transform.localScale / 2, car.transform.rotation);
        if (coll.Length > 0)
        {
            for (int i = 0; i < coll.Length; ++i)
            {
                if (coll[i].gameObject.CompareTag("wall"))
                {
                    return true;
                }
            }
        }
        return false;
    }


}
