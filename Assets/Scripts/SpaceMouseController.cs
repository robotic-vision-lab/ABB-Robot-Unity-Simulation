using UnityEngine;
using TDx.TDxInput;

/// <summary>
/// Attach to an object for the spacemouse to control
/// </summary>

public class SpaceMouseController : MonoBehaviour
{
    
    public float TSpeed = 10.0f;
    public float RSpeed = 10.0f;
    private Device mouse;
    private Sensor sensor;

    // Check connection
    void Awake(){
        try{
            mouse = new Device();
            sensor = mouse.Sensor;
            mouse.Connect();
        }
        catch (System.Exception e){
            Debug.LogError($"Connection Error: {e.Message}");
            mouse = null;
        }
    }

    // Update object from spacemouse input
    void Update(){
        if (mouse == null){
            return;
        }
        Vector3 TVector = new Vector3(
            (float)sensor.Translation.X,
            (float)sensor.Translation.Y,
            -(float)sensor.Translation.Z
        );
        Vector3 RVector = new Vector3(
            (float)sensor.Rotation.X,
            (float)sensor.Rotation.Y,
            -(float)sensor.Rotation.Z
        );

        if (TVector.sqrMagnitude > 0.0001f || RVector.sqrMagnitude > 0.0001f){
            Debug.Log($"Translation: {TVector}, Rotation: {RVector}");
        }

        transform.Translate(TVector * TSpeed * Time.deltaTime, Space.Self);
        transform.Rotate(Vector3.up, RVector.y * RSpeed * Time.deltaTime, Space.World);
        transform.Rotate(Vector3.right, RVector.x * RSpeed * Time.deltaTime, Space.Self);
        transform.Rotate(Vector3.forward, RVector.z * RSpeed * Time.deltaTime, Space.Self);
    }

    // Cleanup
    void OnDestroy(){
        if (mouse != null){
            mouse.Disconnect();
            mouse = null;
            Debug.Log("Space Mouse disconnected.");
        }
    }
}