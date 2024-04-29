using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;

public class MoveToGoalAgents : Agent
{
    //Sotring the target(goal position)
    [SerializeField] private Transform targetTransform;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floorMeshRenderer;

    public override void OnEpisodeBegin()
    {
        Debug.Log("New Episode!!!!!");

        //We need to add randomness or else the model will learn to just move right if the goal was only right of the agent
        //Learn to go to a target and not just a specific position relative to the local
        transform.localPosition = new Vector3(Random.Range(-4.3f, +4.3f), 0f, Random.Range(-3.6f, +4.0f));
        targetTransform.localPosition = new Vector3(Random.Range(-4.2f, +4.3f), 0f, Random.Range(-3.6f, +4.2f));
    }

    /// <summary>
    /// This function is called every fixed framerate frame, if the MonoBehaviour is enabled.
    /// </summary>
    void FixedUpdate()
    {
        Debug.Log(GetCumulativeReward());
    }

    //Used for collection observations about the scene around the agent
    public override void CollectObservations(VectorSensor sensor)
    {
        //Observes the agents current position
        sensor.AddObservation(transform.localPosition);
        //Observes the agents target position
        sensor.AddObservation(targetTransform.localPosition);
    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];

        float moveSpeed = 2.5f;
        transform.localPosition += new Vector3(moveX, 0, moveZ) * Time.deltaTime * moveSpeed;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continousActions = actionsOut.ContinuousActions;
        continousActions[0] = Input.GetAxisRaw("Horizontal");
        continousActions[1] = Input.GetAxisRaw("Vertical");
    }

    private void OnTriggerEnter(Collider other)
    {
        if(other.TryGetComponent<Goal>(out Goal goal))
        {
            Debug.Log("Hit goal!!");
            //Change reward here
            SetReward(+1f);
            floorMeshRenderer.material = winMaterial;

            //End the current episode(run of the training scene)
            EndEpisode();
        }
        if(other.TryGetComponent<Wall>(out Wall wall))
        {
            Debug.Log("Hit wall!!");
            //Change reward here
            SetReward(-1f);
            floorMeshRenderer.material = loseMaterial;

            //End the current episode(run of the training scene)
            EndEpisode();
        }
    }
        
}
