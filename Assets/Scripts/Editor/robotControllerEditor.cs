using UnityEngine;
using UnityEditor;
using Unity.Mathematics;
using System.Collections.Generic;
using System.Collections;

[CustomEditor(typeof(RobotController))]
public class RobotControllerEditor : Editor {
    private int i = 0;
    private int j = 0;
    public override void OnInspectorGUI(){
        RobotController robotController = (RobotController)target;
        
        robotController.theta_1 = EditorGUILayout.FloatField("Theta_1", robotController.theta_1);
        robotController.theta_2 = EditorGUILayout.FloatField("Theta_2", robotController.theta_2);
        robotController.theta_3 = EditorGUILayout.FloatField("Theta_3", robotController.theta_3);
        robotController.theta_4 = EditorGUILayout.FloatField("Theta_4", robotController.theta_3);
        robotController.theta_5 = EditorGUILayout.FloatField("Theta_5", robotController.theta_5);
        robotController.theta_6 = EditorGUILayout.FloatField("Theta_6", robotController.theta_6);
        robotController.learning_rate = EditorGUILayout.FloatField("IK-Learning-Rate", robotController.learning_rate);
        robotController.ef_wc = EditorGUILayout.Vector3Field("End-Effector", robotController.ef_wc);
        robotController.current_target_wc = EditorGUILayout.Vector3Field("Target-WC", robotController.current_target_wc);
        robotController.current_target_loc = EditorGUILayout.Vector3Field("Target-Loc", robotController.current_target_loc);
        robotController.current_interop_point = EditorGUILayout.Vector3Field("Interop-Point", robotController.current_interop_point);
        robotController.maxIterations = EditorGUILayout.IntField("RRT-Iterations", robotController.maxIterations);
        
        
        
        if(GUILayout.Button("Next Point")){
            
            robotController.current_interop_point = robotController.path_wc[i];
            if(i == robotController.path_wc.Count - 1){
                robotController.pick = true;
                robotController.place = false;
                i = 0;
            }
            else{
                i += 1;
            }
            
        }
        if(GUILayout.Button("Place")){
                robotController.pick = false;
                robotController.place = true;
            }
        if(GUILayout.Button("Next Target")){
            robotController.pick = false;
            robotController.place = false;
            robotController.current_target_index= j;
            robotController.next_target = true;
            j += 1;
            }
        }
    }