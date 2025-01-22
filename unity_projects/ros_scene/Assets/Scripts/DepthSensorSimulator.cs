using UnityEngine;

[ExecuteInEditMode]
[RequireComponent(typeof(Camera))]
public class DepthCameraEffect : MonoBehaviour
{
    public Material depthMaterial;

    void Start()
    {
        // Enable depth texture mode on the camera
        GetComponent<Camera>().depthTextureMode = DepthTextureMode.Depth;
    }

    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        // Apply the depth material when rendering the image
        if (depthMaterial != null)
        {
            Graphics.Blit(source, destination, depthMaterial);
        }
        else
        {
            Graphics.Blit(source, destination);
        }
    }
}

