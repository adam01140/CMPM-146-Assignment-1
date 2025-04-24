using UnityEngine;
using System.Collections;

public class BreadcrumbDropper : MonoBehaviour
{
    Vector3 last_drop;
    public GameObject breadcrumb;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        StartCoroutine(DropCrumbs());
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    IEnumerator DropCrumbs()
    {
        while (true)
        {
            if ((last_drop - transform.position).sqrMagnitude > 1)
            {
                Instantiate(breadcrumb, transform.position, Quaternion.identity);
                last_drop = transform.position;
            }
            yield return new WaitForSeconds(1);
        }
    }
}
