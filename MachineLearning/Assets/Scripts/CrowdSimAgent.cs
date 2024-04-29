using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;

public class CrowdSimAgent : Agent
{
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

    // Reward percentages for different behaviors
    private float rewardForMovingCloser = 0.01f;
    private float rewardForSmoothMovement = 0.001f;
    private float rewardForCollision = 4.0f;
    private float rewardForGoal = 6.0f;
    private float rewardForPersoanlSpace = 0.2f;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        previousPositions = new List<Vector3>();
        startPos = transform.localPosition;
    }

    void FixedUpdate()
    {
        recordPosition();

        //Check for smooth movement periodically
        if (Time.frameCount % 10 == 0 && previousPositions.Count >= 10)
        {
            checkSmoothMovement();
        }
    }

    void applyMovement(float moveX, float moveZ)
    {
        moveDirection = new Vector3(moveX, 0, moveZ).normalized;

        float targetSpeed = moveDirection.magnitude * moveSpeed;
        currentSpeed = targetSpeed;

        //Simple movement of the agent added acceleration and decceleration to model more closely how people move
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

    void checkSmoothMovement()
    {
        Vector3 velocity = (previousPositions[previousPositions.Count - 1] - previousPositions[0]) / Time.fixedDeltaTime;

        //Check for smooth movement based on velocity magnitude
        float smoothnessThreshold = 280.0f; // Adjust threshold as needed
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

        applyMovement(moveX, moveZ);

        //We need to compare distance from last action to current distance to see if we are getting closer
        //If we are not getting closer take some reward but if we are gain some. So Agent will learn to minimize distance thus getting to goal
        currentDistance = Vector3.Distance(transform.localPosition, goalTransform.localPosition);
        float reward = (currentDistance < lastDistance) ? +rewardForMovingCloser : -rewardForMovingCloser;
        AddReward(reward);
        lastDistance = currentDistance;
    }

    //Used to provide custom decision maing logic or apply manual control of an agent
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxisRaw("Horizontal");
        continuousActions[1] = Input.GetAxisRaw("Vertical");
    }

    public override void OnEpisodeBegin()
    {
        transform.localPosition = startPos;
        reachedGoal = false;

        //No Longer needed as we already trained model on randomness navigation
        //Randomize Agent position
        //transform.localPosition = new Vector3(Random.Range(-9.8f, 12.0f), 0, Random.Range(-2.5f, 8.1f));
        //Randomize goal position
        //goalTransform.localPosition = new Vector3(Random.Range(-9.17f, 10.6f), -2.787638f, Random.Range(-1.2f, 8.2f));

        //Randomize agent speed to more closely mimic realism
        moveSpeed = Random.Range(2f, 4f);

        //Reset positions
        previousPositions.Clear();

    }

    //What data does the agent need to effectively simulate a crowd?
    public override void CollectObservations(VectorSensor sensor)
    {
        //Observe current position
        sensor.AddObservation(transform.localPosition);
        //Observe position of goal
        sensor.AddObservation(goalTransform.localPosition);
    }

    private void OnTriggerEnter(Collider other)
    {
        if(other.tag == "PersonalSpace")
        {
            AddReward(-rewardForPersoanlSpace);
        }
        if(other.tag == "Obstacle")
        {
            AddReward(-rewardForCollision);
        }
        else if(other.tag == "Goal")
        {
            if(!reachedGoal)
            {
                AddReward(+rewardForGoal);
            }
            reachedGoal = true;
        } 
    }
}
