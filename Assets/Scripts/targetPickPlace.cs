using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Pick and place task using RRT (Rapidly-exploring Random Tree) algorithm
/// </summary>

public class TargetPickPlace : MonoBehaviour {
    public RobotController rc;

    // Initialize end effector, targets, and PID controller
    void Start(){
        rc.start_state = new Vector3(0.421037f, 0.627905f, 4.800409e-05f);
        rc.ef_wc = rc.GetFK(rc.theta_1, rc.theta_2, rc.theta_3, rc.theta_4, rc.theta_5, rc.theta_6).GetColumn(3);
        rc.all_targets = GameObject.FindGameObjectsWithTag("target");
        rc.InitRandomLoc();
        rc.SetRandomLoc();
        rc.PID();
        }

    // Joint angle update; logic to direct the end effector along the RRT path and switch targets
    void FixedUpdate(){
        ArticulationDrive j1d = rc.j1.xDrive;
        ArticulationDrive j2d = rc.j2.xDrive;
        ArticulationDrive j3d = rc.j3.xDrive;
        ArticulationDrive j4d = rc.j4.xDrive;
        ArticulationDrive j5d = rc.j5.xDrive;
        ArticulationDrive j6d = rc.j6.xDrive;

        if(rc.pick == true && rc.place == false){
            rc.current_target.transform.position = rc.GetFK(rc.theta_1, rc.theta_2, rc.theta_3, rc.theta_4, rc.theta_5, rc.theta_6).GetColumn(3);
            rc.ef_wc = rc.current_target.transform.position;
            rc.GetIK(rc.theta_1, rc.theta_2, rc.theta_3, rc.theta_4, rc.theta_5, rc.theta_6, rc.current_interop_point);
            }

        if (rc.pick == false && rc.place == true){
            rc.current_target.transform.position = rc.GetFK(rc.theta_1, rc.theta_2, rc.theta_3, rc.theta_4, rc.theta_5, rc.theta_6).GetColumn(3);
            rc.ef_wc = rc.current_target.transform.position;
            if (rc.current_target_index == 0){
                rc.GetIK(rc.theta_1, rc.theta_2, rc.theta_3, rc.theta_4, rc.theta_5, rc.theta_6, new UnityEngine.Vector3(0.50279f, 0.00739f, 0.0378f));
            }
            if (rc.current_target_index == 1){
                rc.GetIK(rc.theta_1, rc.theta_2, rc.theta_3, rc.theta_4, rc.theta_5, rc.theta_6, new UnityEngine.Vector3(0.5138f, 0.00739f, 0.2041f));
            }
            if (rc.current_target_index == 2){
                rc.GetIK(rc.theta_1, rc.theta_2, rc.theta_3, rc.theta_4, rc.theta_5, rc.theta_6, new UnityEngine.Vector3(0.5138f, 0.00739f, 0.3539f));
            }
            if (rc.current_target_index == 3){
                rc.GetIK(rc.theta_1, rc.theta_2, rc.theta_3, rc.theta_4, rc.theta_5, rc.theta_6, new UnityEngine.Vector3(0.2995f, 0.00739f, -0.0321f));
            }
            rc.target_set = false;
            }

        if (rc.pick == false && rc.place == false && rc.target_set == true){
            rc.ef_wc = rc.GetFK(rc.theta_1, rc.theta_2, rc.theta_3, rc.theta_4, rc.theta_5, rc.theta_6).GetColumn(3);
            rc.GetIK(rc.theta_1, rc.theta_2, rc.theta_3, rc.theta_4, rc.theta_5, rc.theta_6, rc.current_interop_point);
            }

        if (rc.next_target == true){
            rc.path_wc = new List<Vector3>();
            if (rc.current_target_index == 0){
                rc.current_target = rc.all_targets[0];
                rc.current_target_loc = rc.target_loc_dict[rc.current_target];
                rc.path_wc = rc.GetPath();
                rc.current_interop_point = rc.path_wc[0];
                rc.target_set = true;
                rc.next_target = false;
            }
            if (rc.current_target_index == 1){
                rc.current_target = rc.all_targets[1];
                rc.current_target_loc = rc.target_loc_dict[rc.current_target];
                rc.path_wc = rc.GetPath();
                rc.target_set = true;
                rc.next_target = false;
            }
            if (rc.current_target_index == 2){
                rc.current_target = rc.all_targets[2];
                rc.current_target_loc = rc.target_loc_dict[rc.current_target];
                rc.path_wc = rc.GetPath();
                rc.target_set = true;
                rc.next_target = false;
            }
            if (rc.current_target_index == 3){
                rc.current_target = rc.all_targets[3];
                rc.current_target_loc = rc.target_loc_dict[rc.current_target];
                rc.path_wc = rc.GetPath();
                rc.target_set = true;
                rc.next_target = false;
            }
        }
        
        j1d.target = rc.theta_1;
        j2d.target = rc.theta_2;
        j3d.target = rc.theta_3;
        j4d.target = rc.theta_4;
        j5d.target = rc.theta_5;
        j6d.target = rc.theta_6;

        rc.j1.xDrive = j1d;
        rc.j2.xDrive = j2d;
        rc.j3.xDrive = j3d;
        rc.j4.xDrive = j4d;
        rc.j5.xDrive = j5d;
        rc.j6.xDrive = j6d;
        }
    }