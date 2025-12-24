using UnityEngine;
using UnityEngine.InputSystem;

public class CameraController : MonoBehaviour
{
    public float panSpeed = 20f;
    public float zoomSpeed = 200f;
    public float rotationSpeed = 5f;
    public float keyboardRotationSpeed = 100f;
    public float panBorderThickness = 10f;
    public Vector2 panLimit;

    private bool isPaused = false;

    void Update()
    {
        if (Keyboard.current != null && Keyboard.current.pKey.wasPressedThisFrame)
        {
            TogglePause();
        }

        Vector3 pos = transform.position;

        if (Keyboard.current != null)
        {
            Vector3 forward = transform.forward;
            forward.y = 0;
            forward.Normalize();

            Vector3 right = transform.right;
            right.y = 0;
            right.Normalize();

            if (Keyboard.current.wKey.isPressed || Keyboard.current.upArrowKey.isPressed)
            {
                pos += panSpeed * Time.unscaledDeltaTime * forward;
            }
            if (Keyboard.current.sKey.isPressed || Keyboard.current.downArrowKey.isPressed)
            {
                pos -= panSpeed * Time.unscaledDeltaTime * forward;
            }
            if (Keyboard.current.dKey.isPressed || Keyboard.current.rightArrowKey.isPressed)
            {
                pos += panSpeed * Time.unscaledDeltaTime * right;
            }
            if (Keyboard.current.aKey.isPressed || Keyboard.current.leftArrowKey.isPressed)
            {
                pos -= panSpeed * Time.unscaledDeltaTime * right;
            }

            if (Keyboard.current.qKey.isPressed)
            {
                transform.Rotate(Vector3.up, -keyboardRotationSpeed * Time.unscaledDeltaTime, Space.World);
            }
            if (Keyboard.current.eKey.isPressed)
            {
                transform.Rotate(Vector3.up, keyboardRotationSpeed * Time.unscaledDeltaTime, Space.World);
            }

            if(Keyboard.current.zKey.isPressed)
            {
                transform.Rotate(Vector3.right, rotationSpeed * Time.unscaledDeltaTime, Space.Self);
            }

            if(Keyboard.current.xKey.isPressed)
            {
                transform.Rotate(Vector3.right, -rotationSpeed * Time.unscaledDeltaTime, Space.Self);
            }

            if (Keyboard.current.rKey.isPressed)
            {
                pos.y += zoomSpeed * Time.unscaledDeltaTime * 0.1f;
            }
            if (Keyboard.current.fKey.isPressed)
            {
                pos.y -= zoomSpeed * Time.unscaledDeltaTime * 0.1f;
            }
        }

        if (Mouse.current != null)
        {
            float scroll = Mouse.current.scroll.ReadValue().y;
            pos.y -= scroll * zoomSpeed * Time.unscaledDeltaTime * 0.01f;

            if (Mouse.current.rightButton.isPressed)
            {
                float mouseX = Mouse.current.delta.x.ReadValue() * rotationSpeed * Time.unscaledDeltaTime;
                float mouseY = Mouse.current.delta.y.ReadValue() * rotationSpeed * Time.unscaledDeltaTime;

                transform.Rotate(Vector3.up, mouseX, Space.World);
                
                transform.Rotate(Vector3.right, -mouseY, Space.Self);
            }
        }
        
        transform.position = pos;
    }

    private void TogglePause()
    {
        isPaused = !isPaused;
        Time.timeScale = isPaused ? 0f : 1f;
        Debug.LogWarning(isPaused ? "Sim Paused" : "Sim Resumed");
    }
}
