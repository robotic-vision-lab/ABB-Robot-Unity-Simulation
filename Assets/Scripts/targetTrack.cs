using UnityEngine;

/// <summary>
/// Following target object w/ end effector 
/// </summary>

public class TargetTrack : MonoBehaviour {

    public RobotController rc;

   // Initialize PID controller
    void Start(){
        rc.PID();
        }
    
    // Update joint angles as the target moves
    void FixedUpdate(){
        ArticulationDrive j1d = rc.j1.xDrive;
        ArticulationDrive j2d = rc.j2.xDrive;
        ArticulationDrive j3d = rc.j3.xDrive;
        ArticulationDrive j4d = rc.j4.xDrive;
        ArticulationDrive j5d = rc.j5.xDrive;
        ArticulationDrive j6d = rc.j6.xDrive;
        
        rc.current_target_wc = rc.current_target.transform.position;
        rc.GetIK(rc.theta_1, rc.theta_2, rc.theta_3, rc.theta_4, rc.theta_5, rc.theta_6, rc.current_target_wc);
        
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