using UnityEngine;
using System.Collections;

public class BreadcrumbController : MonoBehaviour
{
    public float lifetime;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        StartCoroutine(Disappear());
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    IEnumerator Disappear()
    {
        for (int i = 0; i < lifetime; ++i)
        {
            transform.localScale = new Vector3((lifetime + 1 - i) / (lifetime + 1), (lifetime + 1 - i) / (lifetime + 1), (lifetime + 1 - i) / (lifetime + 1));
            yield return new WaitForSeconds(1);
        }
        Destroy(gameObject);
    }
}
