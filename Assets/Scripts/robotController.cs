using System;
using UnityEngine;
using System.Linq;
using System.Collections.Generic;

/// <summary>
/// Robot movement and pathing functions
/// </summary>

public class RobotController : MonoBehaviour {
    public float stiffness;
    public float damping;
    public float forceLimit;
    public float speed = 5f;
    public float torque = 100f;
    public float acceleration = 5f;

    private ArticulationBody[] articulationChain;
    public GameObject[] all_targets;
    public Dictionary<GameObject, Vector3> target_loc_dict;
    private HashSet<Vector3> target_loc_unqiue = new HashSet<Vector3>();
    public GameObject current_target;

    public ArticulationBody j1;
    public ArticulationBody j2;
    public ArticulationBody j3;
    public ArticulationBody j4;
    public ArticulationBody j5;
    public ArticulationBody j6;

    public bool pick = false;
    public bool place = false;
    public bool next_target = false;
    public bool target_set = false;
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
    [Range(-20.0f, 100.0f)]
    public float theta_5;
    [Range(0f, 360f)]
    public float theta_6;

    public float step;
    public float maxDistance;
    public int maxIterations;
    public LayerMask obstacleLayer;
    public List<Vector3> path_wc;

    public UnityEngine.Vector3 ef_wc;
    public UnityEngine.Vector3 current_target_wc;
    public UnityEngine.Vector3 current_target_loc;
    public UnityEngine.Vector3 current_interop_point;
    public UnityEngine.Vector3 start_state;
    public int current_target_index;

    // Transformation matrix calculation used in forward kinematics
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

    // Forward kinematics calculation using robot arm dimensions
    public UnityEngine.Matrix4x4 GetFK(float t1, float t2, float t3, float t4, float t5, float t6){
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

    // Applying small delta changes to joint angles for determining gradients
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

    // Inverse kinematics using gradient descent
    public void GetIK(float t1, float t2, float t3, float t4, float t5, float t6, UnityEngine.Vector3 target_point){
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
    
    // Generate the RRT tree
    public List<Vector3> GetPath(){   
        UnityEngine.Vector3 current_loc = GetRandomLoc();
        List<Vector3> tree = new List<Vector3> { current_loc };
        for (int i = 0; i < maxIterations; i++)
        {
            UnityEngine.Vector3 random_loc = GetRandomLoc();
            Vector3 nearest_node_loc = GetNearestNode(tree, random_loc);
            Vector3 next_node_loc = GetNextNode(nearest_node_loc, random_loc);
            if (IsCollisionFree(nearest_node_loc, next_node_loc)){
                tree.Add(next_node_loc);
            }
            if (Vector3.Distance(next_node_loc, current_target_loc) < maxDistance){
                return GetShortestPath(tree, next_node_loc);
            }
        }
        return tree;
    }

    // Select random location within the search space (shown under Display 1 camera)
    Vector3 GetRandomLoc(){
        Vector3 xyz_loc = new Vector3(UnityEngine.Random.Range(0, 11), 0f, UnityEngine.Random.Range(0, 11));
        return xyz_loc;
    }

    // Randomize and store the location of the target objects within search space
    public void InitRandomLoc(){
        target_loc_dict = new Dictionary<GameObject, Vector3>();
        for (int i = 0; i < all_targets.Count(); i++){
            Vector3 xyz_loc = new Vector3(UnityEngine.Random.Range(0, 11), 0f, UnityEngine.Random.Range(0, 11));
            if (!target_loc_unqiue.Contains(xyz_loc)){
                target_loc_unqiue.Add(xyz_loc);
                target_loc_dict[all_targets[i]] = xyz_loc;
            }
            else{
                i--;
            }
        }
    }

    // Position the targets at their corresponding locations from initRandomLoc() 
    public void SetRandomLoc(){
        foreach(KeyValuePair<GameObject, Vector3> target in target_loc_dict){
            target.Key.transform.position = GetWCFromLoc(target.Value);
        }
    }

    // Transfrom search space coordinates into world coordinates
    UnityEngine.Vector3 GetWCFromLoc(UnityEngine.Vector3 xyz_loc){
        return new UnityEngine.Vector3(0.07535f + xyz_loc.x * 0.0148f, 0.14755f, 0.25636f + xyz_loc.z * 0.0148f);
    }

    // Iterate through tree to determine next node that is closest to target
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

    // Determine direction to move from start node to a random end location while limiting movements to a preset step distance
    Vector3 GetNextNode(UnityEngine.Vector3 start, UnityEngine.Vector3 end){
        Vector3 dir = (end - start).normalized;
        return start + dir * Mathf.Min(step, UnityEngine.Vector3.Distance(start, end));
    }

    // Use raycasting to determine collision in the RRT path
    bool IsCollisionFree(UnityEngine.Vector3 start, UnityEngine.Vector3 end){
        RaycastHit hit;
        if (Physics.Linecast(start, end, out hit, obstacleLayer)){
            return false;
        }
        return true;
    }

    // Get the final tree path
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
        path_wc[0] = start_state;
        return path_wc;
    }

    // Draw the RRT path (green)
    void OnDrawGizmos(){
        if (path_wc == null || path_wc.Count < 2)
            return;
        Gizmos.color = Color.green;
        for (int i = 0; i < path_wc.Count - 1; i++){
            Gizmos.DrawLine(path_wc[i], path_wc[i + 1]);
        }
    }

    // Controls the physics and movement of the robot arm
    public void PID(){
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;

        // PID Controller
        foreach (ArticulationBody joint in articulationChain){   
                ArticulationDrive currentDrive = joint.xDrive;
                joint.jointFriction = defDyanmicVal;
                joint.angularDamping = defDyanmicVal;
                currentDrive.forceLimit = forceLimit;
                currentDrive.stiffness = stiffness;
                currentDrive.damping = damping;
                joint.xDrive = currentDrive;
            }
        }
    }   