using UnityEngine;

public class OptionsUI : MonoBehaviour
{
    public GameObject options;
    public GameObject options_button;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void ShowOptions()
    {
        options.SetActive(true);
        options_button.SetActive(false);
    }

    public void HideOptions()
    {
        options.SetActive(false);
        options_button.SetActive(true);
    }
}
