using System;
using UnityEngine;
using UnityEditor;
using Unity.Robotics;
using Unity.Mathematics;

public class RobotController : MonoBehaviour {
    public float stiffness;
    public float damping;
    public float forceLimit;
    public float speed = 5f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 5f;// Units: m/s^2 / degree/s^2
    
    private ArticulationBody[] articulationChain;
    public GameObject FK_target;
    public GameObject IK_target;

    public ArticulationBody j1;
    public ArticulationBody j2;
    public ArticulationBody j3;
    public ArticulationBody j4;
    public ArticulationBody j5;
    public ArticulationBody j6;

    public bool perform_IK = false;
    public float IK_threshold = 0.001f;
    public float learning_rate = 0.1f;
    public int iterations = 100;
    public float delta = 0.0001f;

    [Range(-165.0f, 165.0f)]
    public float theta_1;
    [Range(-110.0f, 110.0f)]
    public float theta_2;
    [Range(-70.0f, 110.0f)]
    public float theta_3;
    [Range(-160.0f, 160.0f)]
    public float theta_4;
    [Range(-120.0f, 120.0f)]
    public float theta_5;
    [Range(0f, 360f)]
    public float theta_6;

    public UnityEngine.Vector3 end_effector_position;
    public UnityEngine.Quaternion end_effector_rotation;

    public UnityEngine.Vector3 IK_target_position;
    public float IK_target_distance;

    public void OnGUI(){
        perform_IK = EditorGUILayout.ToggleLeft("IK", perform_IK);
    }
    
    UnityEngine.Matrix4x4 GetRotation(UnityEngine.Vector3 angles){
        float rad_x = (float)(angles.x * (Math.PI / 180));
        float rad_y = (float)(angles.y * (Math.PI / 180));
        float rad_z = (float)(angles.z * (Math.PI / 180));

        UnityEngine.Matrix4x4 rot_x = new UnityEngine.Matrix4x4(
            new UnityEngine.Vector4(1, 0, 0, 0),
            new UnityEngine.Vector4(0, Mathf.Cos(rad_x), -Mathf.Sin(rad_x), 0),
            new UnityEngine.Vector4(0, Mathf.Sin(rad_x), Mathf.Cos(rad_x), 0),
            new UnityEngine.Vector4(0, 0, 0, 1)
        );
        UnityEngine.Matrix4x4 rot_y = new UnityEngine.Matrix4x4(
            new UnityEngine.Vector4(Mathf.Cos(rad_y), 0, Mathf.Sin(rad_y), 0),
            new UnityEngine.Vector4(0, 1, 0, 0),
            new UnityEngine.Vector4(-Mathf.Sin(rad_y), 0, Mathf.Cos(rad_y), 0),
            new UnityEngine.Vector4(0, 0, 0, 1)
        );
        UnityEngine.Matrix4x4 rot_z = new UnityEngine.Matrix4x4(
            new UnityEngine.Vector4(Mathf.Cos(rad_z), -Mathf.Sin(rad_z), 0, 0),
            new UnityEngine.Vector4(Mathf.Sin(rad_z), Mathf.Cos(rad_z), 0, 0),
            new UnityEngine.Vector4(0, 0, 1, 0),
            new UnityEngine.Vector4(0, 0, 0, 1)
        );
        return rot_z * rot_y * rot_x;
    }

    UnityEngine.Matrix4x4 GetFK(float t1, float t2, float t3, float t4, float t5, float t6){
        UnityEngine.Matrix4x4 T = UnityEngine.Matrix4x4.identity;
        UnityEngine.Matrix4x4 T_1 = UnityEngine.Matrix4x4.Translate(new UnityEngine.Vector3(0, 0.166f, 0));
        UnityEngine.Matrix4x4 R_1 = GetRotation(new UnityEngine.Vector3(0, -t1, 0));
        T = T * (T_1 * R_1);
        UnityEngine.Matrix4x4 T_2 = UnityEngine.Matrix4x4.Translate(new UnityEngine.Vector3(0f, 0.122f, 0.0595f));
        UnityEngine.Matrix4x4 R_2 = GetRotation(new UnityEngine.Vector3(0, 0, -t2));
        T = T * (T_2 * R_2);
        UnityEngine.Matrix4x4 T_3 = UnityEngine.Matrix4x4.Translate(new UnityEngine.Vector3(0f, 0.27f, -0.1095f));
        UnityEngine.Matrix4x4 R_3 = GetRotation(new UnityEngine.Vector3(0, 0, -t3));
        T = T * (T_3 * R_3);
        UnityEngine.Matrix4x4 T_4 = UnityEngine.Matrix4x4.Translate(new UnityEngine.Vector3(0.149637f, 0.069905f, 0.05002f));
        UnityEngine.Matrix4x4 R_4 = GetRotation(new UnityEngine.Vector3(t4, 0f, 0));
        T = T * (T_4 * R_4);
        UnityEngine.Matrix4x4 T_5 = UnityEngine.Matrix4x4.Translate(new UnityEngine.Vector3(0.152405f, 0f, -0.03098f));
        UnityEngine.Matrix4x4 R_5 = GetRotation(new UnityEngine.Vector3(0, 0, t5));
        T = T * (T_5 * R_5);
        UnityEngine.Matrix4x4 T_6 = UnityEngine.Matrix4x4.Translate(new UnityEngine.Vector3(0.078995f, 0f, 0.031008f));
        UnityEngine.Matrix4x4 R_6 = GetRotation(new UnityEngine.Vector3(t6, 0, 0));
        T = T * (T_6 * R_6);
        return T;
    }
    float GetGradient(float t1, float t2, float t3, float t4, float t5, float t6, int joint_num){
        float D = UnityEngine.Vector3.Distance(GetFK(t1, t2, t3, t4, t5, t6).GetColumn(3), IK_target.transform.position);
        if (joint_num == 1){
            t1 = t1 + delta;
        }
        else if (joint_num == 2){
            t2 = t2 + delta;
        }
        else if (joint_num == 3){
            t3 = t3 + delta;
        }
        else if (joint_num == 4){
            t4 = t4 + delta;
        }
        else if (joint_num == 5){
            t5 = t5 + delta;
        }
        else if (joint_num == 6){
            t6 = t6 + delta;
        }
        float D_prime = UnityEngine.Vector3.Distance(GetFK(t1, t2, t3, t4, t5, t6).GetColumn(3), IK_target.transform.position);
        return((D_prime - D) / delta);
    }

    void GetIK(float t1, float t2, float t3, float t4, float t5, float t6){
        for (int i = 0; i < iterations; i++){
            float error_distance = UnityEngine.Vector3.Distance(GetFK(t1, t2, t3, t4, t5, t6).GetColumn(3), IK_target.transform.position);
            
            if (error_distance < IK_threshold){
                break;
            }
            float grad_t1 = GetGradient(t1, t2, t3, t4, t5, t6, 1);
            float grad_t2 = GetGradient(t1, t2, t3, t4, t5, t6, 2);
            float grad_t3 = GetGradient(t1, t2, t3, t4, t5, t6, 3);
            float grad_t4 = GetGradient(t1, t2, t3, t4, t5, t6, 4);
            float grad_t5 = GetGradient(t1, t2, t3, t4, t5, t6, 5);

            t1 -= learning_rate * grad_t1;
            t2 -= learning_rate * grad_t2;
            t3 -= learning_rate * grad_t3;
            t4 -= learning_rate * grad_t4;
            t5 -= learning_rate * grad_t5;

            theta_1 = t1;
            theta_2 = t2;
            theta_3 = t3;
            theta_4 = t4;
            theta_5 = t5;
        }
    }
    void Start(){
    // PID
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
            {   
                ArticulationDrive currentDrive = joint.xDrive;
                joint.jointFriction = defDyanmicVal;
                joint.angularDamping = defDyanmicVal;
                currentDrive.forceLimit = forceLimit;
                joint.xDrive = currentDrive;
            }
    }
    void Update(){
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        foreach (ArticulationBody joint in articulationChain)
            {   
                ArticulationDrive currentDrive = joint.xDrive;
                currentDrive.stiffness = stiffness;
                currentDrive.damping = damping;
                joint.xDrive = currentDrive;
            }
    }
    void FixedUpdate(){


        ArticulationDrive j1d = j1.xDrive;
        ArticulationDrive j2d = j2.xDrive;
        ArticulationDrive j3d = j3.xDrive;
        ArticulationDrive j4d = j4.xDrive;
        ArticulationDrive j5d = j5.xDrive;
        ArticulationDrive j6d = j6.xDrive;
        
        if (perform_IK){
            //FK_target.transform.position = new UnityEngine.Vector3(-2, 0, 0); //move FK cube out of the way
            Debug.Log((theta_1, theta_2, theta_3, theta_4, theta_5));
            end_effector_position = GetFK(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6).GetColumn(3);
            IK_target_position = IK_target.transform.position;
            IK_target_distance = UnityEngine.Vector3.Distance(end_effector_position, IK_target_position);
            GetIK(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6);
        }
        else{
            //IK_target.transform.position = new UnityEngine.Vector3(2, 0, 0); // move IK sphere ut of the way
            UnityEngine.Matrix4x4 T = GetFK(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6);
            FK_target.transform.position = T.GetColumn(3);
            FK_target.transform.rotation = UnityEngine.Quaternion.LookRotation(T.GetColumn(2), T.GetColumn(1));
            end_effector_position = T.GetColumn(3);
            end_effector_rotation = UnityEngine.Quaternion.LookRotation(T.GetColumn(2), T.GetColumn(1));
        }
        
        j1d.target = theta_1;
        j2d.target = theta_2;
        j3d.target = theta_3;
        j4d.target = theta_4;
        j5d.target = theta_5;
        j6d.target = theta_6;

        j1.xDrive = j1d;
        j2.xDrive = j2d;
        j3.xDrive = j3d;
        j4.xDrive = j4d;
        j5.xDrive = j5d;
        j6.xDrive = j6d;
    }

}