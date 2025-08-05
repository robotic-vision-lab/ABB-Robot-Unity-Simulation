
using UnityEngine;
using TDx.TDxInput;


public class SpaceMouseController : MonoBehaviour
{
    
    public float TranslationSpeed = 10.0f;
    public float RotationSpeed = 10.0f;
    private Device _spaceMouseDevice;
    private Sensor _sensor;

    void Awake(){
        try{
            _spaceMouseDevice = new Device();
            _sensor = _spaceMouseDevice.Sensor;
            _spaceMouseDevice.Connect();
        }
        catch (System.Exception e){
            Debug.LogError($"Error connecting to Space Mouse: {e.Message}");
            _spaceMouseDevice = null;
        }
    }

    void Update(){
        if (_spaceMouseDevice == null){
            return;
        }
        Vector3 translationVector = new Vector3(
            (float)_sensor.Translation.X,
            (float)_sensor.Translation.Y,
            -(float)_sensor.Translation.Z
        );
        Vector3 rotationVector = new Vector3(
            (float)_sensor.Rotation.X,
            (float)_sensor.Rotation.Y,
            -(float)_sensor.Rotation.Z
        );

        if (translationVector.sqrMagnitude > 0.0001f || rotationVector.sqrMagnitude > 0.0001f){
            Debug.Log($"Space Mouse Input - Translation: {translationVector}, Rotation: {rotationVector}");
        }

        transform.Translate(translationVector * TranslationSpeed * Time.deltaTime, Space.Self);
        transform.Rotate(Vector3.up, rotationVector.y * RotationSpeed * Time.deltaTime, Space.World);
        transform.Rotate(Vector3.right, rotationVector.x * RotationSpeed * Time.deltaTime, Space.Self);
        transform.Rotate(Vector3.forward, rotationVector.z * RotationSpeed * Time.deltaTime, Space.Self);
    }

    void OnDestroy(){
        if (_spaceMouseDevice != null){
            _spaceMouseDevice.Disconnect();
            _spaceMouseDevice = null;
            Debug.Log("Space Mouse disconnected.");
        }
    }
}