using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;

public class CrowdSimAgent : Agent
{
    //Used for grouping agents
    public int groupId;

    //For checking if agent readched the goal
    public bool reachedGoal = false;

    [SerializeField] private Transform goalTransform;
    private Vector3 startPos;
    private float currentDistance;
    private float lastDistance;

    //Different people might have different speeds at which they walk
    private float moveSpeed = 3f;

    private float currentSpeed = 0f;
    private Vector3 moveDirection;

    private Rigidbody rb;
    private List<Vector3> previousPositions;

    // Reward percentages for different behaviors (reward values are relative to each other)
    private float rewardForMovingCloser = 0.01f; //applied every action
    private float rewardForSmoothMovement = 0.001f; //applied every "smoothCheckAmount" frames
    private int smoothCheckAmount = 10; //The amount of frames for each smoothMovement check
    private float rewardForCollision = 0.4f; //Applied every time agent hits an obstacle
    private float rewardForGoal = 1.0f; //Applied once per episode when the agent arrives at the goal
    private float rewardForAgentAvoidance = 0.1f; //Applied every time an agent gets within a certain distance of another agent

    //Called once when the agent is first initialized
    public override void Initialize()
    {
        //Store rigid body component of agent
        rb = GetComponent<Rigidbody>();
        previousPositions = new List<Vector3>();
        startPos = transform.localPosition;
    }

    //Runs every frame
    void FixedUpdate()
    {
        //Record current position
        recordPosition();

        //Check for smooth movement periodically
        if (Time.frameCount % smoothCheckAmount == 0 && previousPositions.Count >= 10)
        {
            checkSmoothMovement();
        }
        //Adjusting spped dynamically
        currentSpeed = moveSpeed;
    }


    //Soley used for apply basic movement to agent
    void applyMovement(float moveX, float moveZ)
    {
        moveDirection = new Vector3(moveX, 0, moveZ).normalized;

        float targetSpeed = moveDirection.magnitude * moveSpeed;
        currentSpeed = targetSpeed;

        //Simple movement of the agent 
        //Should add acceleration and decceleration to model to mimic more closely how people move
        transform.localPosition += moveDirection * currentSpeed * Time.deltaTime;

        //Solely for rotating model to visual direction agent is going
        if (moveDirection != Vector3.zero)
        {
            //Uses quanternion angles to calculate the look rotation
            Quaternion targetRotation = Quaternion.LookRotation(moveDirection);
            //Apply a -90 degree y rotation after to fix bug
            targetRotation *= Quaternion.Euler(0, -90, 0);
            //Smoothly interpolates between the current rotation and the target rotation
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 10f * Time.deltaTime);
        }
    }

    void recordPosition()
    {
        previousPositions.Add(transform.position);
    }

    //Used for penalizing agent for making eratic movement
    void checkSmoothMovement()
    {
        //Gets the velocity difference from the current to the last
        Vector3 velocity = (previousPositions[previousPositions.Count - 1] - previousPositions[0]) / Time.fixedDeltaTime;

        //Check for smooth movement based on velocity magnitude
        float smoothnessThreshold = 250.0f; // Adjust threshold as needed
        if (velocity.magnitude < smoothnessThreshold)
        {
            //Penalize agent for lack of smooth movement
            AddReward(-rewardForSmoothMovement);
        }
        else
        {
            //Rewarded for smooth movement
            AddReward(rewardForSmoothMovement);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        //Actions the agent can take (move on the x and z)
        //the first float on the action buffer refers to movement on the x
        float moveX = actions.ContinuousActions[0];
        //the second float on the action buffer refers to movement on the z
        float moveZ = actions.ContinuousActions[1];

        //Apply movement based on the action buffer
        applyMovement(moveX, moveZ);

        //We need to compare distance from last action to current distance to see if we are getting closer
        //If we are not getting closer take some reward but if we are gain some. So Agent will learn to minimize distance thus getting to goal
        currentDistance = Vector3.Distance(transform.localPosition, goalTransform.localPosition);
        float reward = (currentDistance < lastDistance) ? +rewardForMovingCloser : -rewardForMovingCloser;
        AddReward(reward);
        lastDistance = currentDistance;
    }

    //Used to provide custom decision logic or apply manual control of an agent
    //Only runs when agent "Inference Type" is set to Heuristic only
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //We can move the agent with our keyboard for testing
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxisRaw("Horizontal");
        continuousActions[1] = Input.GetAxisRaw("Vertical");
    }

    //Called at the beginning of every epsiode
    //Here we want to reset positions of gameobjects
    public override void OnEpisodeBegin()
    {
        transform.localPosition = startPos;
        reachedGoal = false;

        //Randomize parameters so agent does not learn to go to a certain position
        //Randomize Agent position
        //transform.localPosition = new Vector3(Random.Range(-9.5f, 6.5f), 1.5f, Random.Range(-13.5f, 2.5f));
        //Randomize goal position
        //goalTransform.localPosition = new Vector3(Random.Range(-9.5f, 6.5f), 0.5f, Random.Range(-13.5f, 2.5f));

        //Randomize agent speed to more closely mimic realism
        moveSpeed = Random.Range(2f, 4f);

        //Reset positions
        previousPositions.Clear();

    }

    //What data does the agent need to effectively simulate a crowd?
    /*
    - Its current position
    - Its target position
    - ...
    */
    public override void CollectObservations(VectorSensor sensor)
    {
        //Observe current position
        sensor.AddObservation(transform.localPosition);
        //Observe position of goal
        sensor.AddObservation(goalTransform.localPosition);
    }

    private void OnTriggerEnter(Collider other)
    {
        //Used for reward agent for avoiding getting within close proximity of other agents
        if(other.tag == "PersonalSpace")
        {
            AddReward(-rewardForAgentAvoidance);
            currentSpeed *= 0.6f;
        }

        if(other.tag == "Wall")
        {
            Debug.Log("Hit wall");
            AddReward(-rewardForCollision * 2);
        }

        //Penalized for hitting an obstacle
        if(other.tag == "Obstacle")
        {
            Debug.Log("Hit obstacle");
            AddReward(-rewardForCollision);
        }

        //Rewarded for getting to the goal
        else if(other.tag == "Goal")
        {
        //     if(!reachedGoal)
        //     {
        //         AddReward(+rewardForGoal);
        //     }
        //     reachedGoal = true;
            Debug.Log("Reached goal");
            AddReward(+rewardForGoal);
            EndEpisode();
        } 
    }
}
