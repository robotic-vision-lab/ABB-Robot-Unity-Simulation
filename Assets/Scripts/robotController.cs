using System;
using UnityEngine;
using UnityEditor;
using Unity.Mathematics;
using System.Linq;
using System.Collections.Generic;
using System.Collections;

public class RobotController : MonoBehaviour {
    public float stiffness;
    public float damping;
    public float forceLimit;
    public float speed = 5f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 5f;// Units: m/s^2 / degree/s^2

    
    private ArticulationBody[] articulationChain;
    private PythonSocketConnector image_info;
    public GameObject target;
    public Camera cam_1;
    float3x3 k;

    public ArticulationBody j1;
    public ArticulationBody j2;
    public ArticulationBody j3;
    public ArticulationBody j4;
    public ArticulationBody j5;
    public ArticulationBody j6;

    public bool pick = false;
    public bool place = false;
    public float IK_threshold = 0.001f;
    public float learning_rate = 0.1f;
    public int iterations = 10;
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

    public float step;
    public float maxDistance;
    public int maxIterations;
    public LayerMask obstacleLayer;
    public List<Vector3> path_wc = new List<Vector3>();

    public UnityEngine.Vector3 ef_wc;
    public UnityEngine.Vector3 target_wc;
    public UnityEngine.Vector3 target_loc;
    public UnityEngine.Vector3 interop_point;

    public float img_width;
    public float img_height;
    public UnityEngine.Vector3 cam_1_position;
    public UnityEngine.Vector3 cam_1_rotation;
    public UnityEngine.Vector3 pred_target_position;
    public float3x3 GetIntrinsic(Camera cam){
        img_width = cam.pixelWidth;
        img_height = cam.pixelHeight;
        float pixel_aspect_ratio = cam.pixelWidth / cam.pixelHeight;

        float alpha_u = cam.focalLength * (cam.pixelWidth / cam.sensorSize.x);
        float alpha_v = cam.focalLength * pixel_aspect_ratio * (cam.pixelHeight / cam.sensorSize.y);

        float u_0 = cam.pixelWidth / 2;
        float v_0 = cam.pixelHeight / 2;

        //IntrinsicMatrix in row major
        float3x3 camIntriMatrix = new float3x3(alpha_u,      0f, u_0,
                                                    0f, alpha_v, v_0,
                                                    0f,      0f,  1f);
        return camIntriMatrix;
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
        UnityEngine.Matrix4x4 T_6 = UnityEngine.Matrix4x4.Translate(new UnityEngine.Vector3(0.118995f, 0f, 0.031008f));
        UnityEngine.Matrix4x4 R_6 = GetRotation(new UnityEngine.Vector3(t6, 0, 0));
        T = T * (T_6 * R_6);
        return T;
    }
    float GetGradient(float t1, float t2, float t3, float t4, float t5, float t6, int joint_num, UnityEngine.Vector3 target_point){
        float D = UnityEngine.Vector3.Distance(GetFK(t1, t2, t3, t4, t5, t6).GetColumn(3), target_point);
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
        float D_prime = UnityEngine.Vector3.Distance(GetFK(t1, t2, t3, t4, t5, t6).GetColumn(3), target_point);
        return((D_prime - D) / delta);
    }

    void GetIK(float t1, float t2, float t3, float t4, float t5, float t6, UnityEngine.Vector3 target_point){
        for (int i = 0; i < iterations; i++){
            float error_distance = UnityEngine.Vector3.Distance(GetFK(t1, t2, t3, t4, t5, t6).GetColumn(3), target_point);
            
            if (error_distance < IK_threshold){
                break;
            }
            float grad_t1 = GetGradient(t1, t2, t3, t4, t5, t6, 1, target_point);
            float grad_t2 = GetGradient(t1, t2, t3, t4, t5, t6, 2, target_point);
            float grad_t3 = GetGradient(t1, t2, t3, t4, t5, t6, 3, target_point);
            float grad_t4 = GetGradient(t1, t2, t3, t4, t5, t6, 4, target_point);
            float grad_t5 = GetGradient(t1, t2, t3, t4, t5, t6, 5, target_point);

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
    //=========================================================== RRT ==================================================//
    List<Vector3> GetPath(){   
        UnityEngine.Vector3 current_loc = GetRandomLoc();
        List<Vector3> tree = new List<Vector3> { current_loc };
        //Debug.Log(current_loc);
        for (int i = 0; i < maxIterations; i++)
        {
            UnityEngine.Vector3 random_loc = GetRandomLoc();
            Vector3 nearest_node_loc = GetNearestNode(tree, random_loc);
            Vector3 next_node_loc = GetNextNode(nearest_node_loc, random_loc);
            if (IsCollisionFree(nearest_node_loc, next_node_loc)){
                tree.Add(next_node_loc);
            }
            if (Vector3.Distance(next_node_loc, target_loc) < maxDistance){
                //Debug.Log("End");
                return GetShortestPath(tree, next_node_loc);
            }
        }
        return tree;
    }
    UnityEngine.Vector3 GetRandomLoc(){ 
        UnityEngine.Vector3 xyz_loc = new Vector3(UnityEngine.Random.Range(0, 11), 0f, UnityEngine.Random.Range(0, 11));
        //Debug.Log(target_loc);
        return xyz_loc;
    }
    UnityEngine.Vector3 GetWCFromLoc(UnityEngine.Vector3 xyz_loc){
        return new UnityEngine.Vector3(0.07535f + xyz_loc.x * 0.0148f, 0.14755f, 0.25636f + xyz_loc.z * 0.0148f);
    }
    
    void SetTargetLoc(){
        target.transform.position = new UnityEngine.Vector3(0.07535f + target_loc.x * 0.0148f, 0.14755f, 0.25636f + target_loc.z * 0.0148f);
        target_wc = target.transform.position;
    }

    Vector3 GetNearestNode(List<UnityEngine.Vector3> tree, UnityEngine.Vector3 xyz_loc){
        Vector3 nearest_node = tree[0];
        float min_dist = UnityEngine.Vector3.Distance(xyz_loc, nearest_node);
        foreach (var node in tree){
            float current_dist = UnityEngine.Vector3.Distance(xyz_loc, node);
            if (current_dist < min_dist){
                nearest_node = node;
                min_dist = current_dist;
            }
        }
        return nearest_node;
    }

    Vector3 GetNextNode(UnityEngine.Vector3 start, UnityEngine.Vector3 end){
        Vector3 dir = (end - start).normalized;
        return start + dir * Mathf.Min(step, UnityEngine.Vector3.Distance(start, end));
    }

    bool IsCollisionFree(UnityEngine.Vector3 start, UnityEngine.Vector3 end){
        RaycastHit hit;
        if (Physics.Linecast(start, end, out hit, obstacleLayer)){
            return false;
        }
        return true;
    }

    List<Vector3> GetShortestPath(List<UnityEngine.Vector3> tree, UnityEngine.Vector3 end){
        List<Vector3> path_loc = new List<Vector3>();
        path_loc.Add(end);
        UnityEngine.Vector3 current_node = end;
        for (int i = tree.Count - 1; i >= 0; i--){
            tree.RemoveAt(tree.Count - 1);
            Vector3 next_node = GetNearestNode(tree, current_node);
            path_loc.Add(next_node);
            if (next_node == tree[0]){
                break;
            }
            current_node = next_node;
        }
        
        List<Vector3> path_loc_unique = path_loc.Distinct().ToList();
        foreach (var node in path_loc_unique){
            path_wc.Add(GetWCFromLoc(node));
        }
        path_wc.Add(ef_wc);
        path_wc.Reverse();
        return path_wc;
    }

    void OnDrawGizmos()
    {
        if (path_wc == null || path_wc.Count < 2)
            return;
        Gizmos.color = Color.green;
        for (int i = 0; i < path_wc.Count - 1; i++){
            Gizmos.DrawLine(path_wc[i], path_wc[i + 1]);
        }
    }

    void Start(){
    // PID
        image_info = GameObject.Find("cam_1").GetComponent<PythonSocketConnector>();
        ef_wc = GetFK(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6).GetColumn(3);
        target_loc = GetRandomLoc();
        SetTargetLoc();
        path_wc = GetPath();
        interop_point = path_wc[0];
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        k = GetIntrinsic(cam_1);
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
            {   
                ArticulationDrive currentDrive = joint.xDrive;
                joint.jointFriction = defDyanmicVal;
                joint.angularDamping = defDyanmicVal;
                currentDrive.forceLimit = forceLimit;
                currentDrive.stiffness = stiffness;
                currentDrive.damping = damping;
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

        if(pick == true && place == false){
            target.transform.position = GetFK(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6).GetColumn(3);
            ef_wc = target.transform.position;
            GetIK(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, interop_point);

        } 
        if (pick == false && place == true) {
            target.transform.position = GetFK(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6).GetColumn(3);
            ef_wc = target.transform.position;
            GetIK(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, new UnityEngine.Vector3(0.2995f, 0.00739f, -0.0321f));
        }
        if (pick == false && place == false){
            target.transform.position = target_wc;
            ef_wc = GetFK(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6).GetColumn(3);
            GetIK(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, interop_point);
            }
        cam_1_rotation = new UnityEngine.Vector3(20f, 0f, 0f);
        cam_1_position = cam_1.transform.position;
        if (image_info != null){
            if (image_info.GetLastIncomingMessage() != null){
                var uvz = image_info.GetLastIncomingMessage().Trim('(', ')').Split(',').Select(part => float.Parse(part.Trim())).ToArray();
                float u = uvz[0];
                float v = uvz[1];
                float Z = uvz[2] / 100;
                float3 uvz_ = new float3(u, v, Z);

                float3 xy_norm = math.mul(math.inverse(k), new float3(u,v,1));
                float3 xy_scaled = xy_norm * Z;
                float rad_x = (float)(cam_1_rotation.x * (Math.PI / 180));
                float3x3 R = new float3x3(1f,              0f,               0f,
                                        0f, math.cos(rad_x), -math.sin(rad_x),
                                        0f, math.sin(rad_x),  math.cos(rad_x));
                float3 camera_world = new float3(cam_1_position.z, cam_1_position.x, cam_1_position.y);
                float3 xy_local = math.mul(R, xy_scaled);
                float3 xy_world = camera_world - xy_local;
                pred_target_position = new UnityEngine.Vector3(xy_world[1], Z, xy_world[0]);
                Debug.Log(camera_world);
                Debug.Log(xy_local);
                Debug.Log(xy_world);
                }
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