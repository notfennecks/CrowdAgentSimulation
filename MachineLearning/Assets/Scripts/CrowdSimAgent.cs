using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;

public class CrowdSimAgent : Agent
{

    private RayPerceptionSensorComponent3D perception;

    //For checking if agent readched the goal
    public bool reachedGoal = false;

    //Used for setting the goal position the agent has to travel to
    [SerializeField] private Transform goalTransform;
    private UnityEngine.Vector3 startPos;
    private float currentDistance;
    private float lastDistance;

    //Movement speed of agent
    private float moveSpeed = 3f;
    //Current speed of agent
    private float currentSpeed = 0f;
    //Direction of movement for agent
    private UnityEngine.Vector3 moveDirection;
    //Stores rigidbody componment for movement calculations
    private Rigidbody rb;
    //Storing the previous positions of the agent
    private List<UnityEngine.Vector3> previousPositions;

    // Reward percentages for different behaviors (reward values are relative to each other)
    private float rewardForGoal = 30.0f; //Applied once per episode when the agent arrives at the goal
    private float rewardForMovingCloser = 0.05f; //applied every action
    private float rewardForSmoothMovement = 0.005f; //applied every "smoothCheckAmount" frames
    private int smoothCheckAmount = 10; //The amount of frames needed for each smoothMovement check (higher values mean it is checked less often and reward are applied less often)
    private float rewardForCollision = 0.6f; //Applied every time agent hits an obstacle
    private float rewardForAgentAvoidance = 0.1f; //Applied every time an agent gets within a certain distance of another agent
    public float rewardForAlignment = 0.05f;  //For flocking
    public float rewardForCohesion = 0.05f;  //For flocking

    //Quantitative Evaluation Metrics for each episode
    private float totalTimeTaken; //Total time taken to reach the goal
    private int totalCollisions; //Total number of collisions with obstacles
    private int totalAgentAvoidance; //Total number of time agent avoided getting to other agents

    //Dynamic Speed adjustment for apporaching certain obstacles
    public float minSpeed = 1f;
    public float maxSpeed;
    public float safeDistance = 2f;
    public float acceleration = 1f;
    public float deceleration = 2f;

    //Storing a list of all other agents in the scene
    private List<CrowdSimAgent> allAgents = new List<CrowdSimAgent>();

    //Flocking parameters
    public float separationRadius = 2f;
    public float alignmentRadius = 5f;
    public float cohesionRadius = 5f;
    public float separationWeight = 1f;
    public float alignmentWeight = 1f;
    public float cohesionWeight = 1f;

    //Called once when the agent is first initialized
    public override void Initialize()
    {
        //Randomize agent speed to more closely mimic realism
        moveSpeed = UnityEngine.Random.Range(1.5f, 3f);

        //For storing perception sensors used for detection
        perception = GetComponent<RayPerceptionSensorComponent3D>();
        //Store rigid body component of agent
        rb = GetComponent<Rigidbody>();
        //Stoing previousPositions for smooth movement calculations
        previousPositions = new List<UnityEngine.Vector3>();
        //Store starting positions of agent to reset every episode
        startPos = transform.localPosition;

        //Set maxSpeed to agents randomized speed when initialized
        maxSpeed = moveSpeed;

        //For adding a reference to all other agents in the scene
        CrowdSimAgent[] agentsInScene = GameObject.FindObjectsByType<CrowdSimAgent>(FindObjectsSortMode.None);
        allAgents.AddRange(agentsInScene);
        allAgents.Remove(this);
    }

    private void UpdateEvaluationMetrics()
    {
        //Update the time taken to reach goal(each episode)
        totalTimeTaken += Time.deltaTime;
    }
    

    //Runs every frame
    void FixedUpdate()
    {
        UpdateEvaluationMetrics();

        //Record current position
        recordPosition();

        //Check for smooth movement periodically
        if (Time.frameCount % smoothCheckAmount == 0 && previousPositions.Count >= 10)
        {
            checkSmoothMovement();
        }

        //Adjust speed dynamically based on obstacle and agent distance
        adjustSpeed();

        //Apply flocking behavior based on other agents
        //FlockingBehavior();
    }

    private void FlockingBehavior()
    {
        //Calcualte vectors for separation, alignemnt, and cohesion
        UnityEngine.Vector3 separation = CalculateSeparation();
        UnityEngine.Vector3 alignment = CalculateAlignment();
        UnityEngine.Vector3 cohesion = CalculateCohesion();

        // Combine flocking behaviors (adjust weights as needed)
        UnityEngine.Vector3 flockingBehavior = separation * separationWeight +
                                   alignment * alignmentWeight +
                                   cohesion * cohesionWeight;

        // Apply flocking behavior to adjust agent's velocity
        rb.velocity += flockingBehavior * Time.deltaTime;

        //Update rewards for flocking
        //Debug.Log("Reward: Flocking reward");
        AddReward(rewardForAlignment * alignment.magnitude);
        AddReward(rewardForCohesion * cohesion.magnitude);
    }

    //Calculate separation behavior
    private UnityEngine.Vector3 CalculateSeparation()
    {
        //Set separation to ZERO vector
        UnityEngine.Vector3 separation = UnityEngine.Vector3.zero;
        int count = 0;

        //Iterate through all agents in scene
        foreach (var agent in allAgents)
        {
            if (agent != this && UnityEngine.Vector3.Distance(transform.position, agent.transform.position) < separationRadius)
            {
                separation += (transform.position - agent.transform.position).normalized;
                count++;
            }
        }

        if (count > 0)
        {
            separation /= count;
        }

        return separation.normalized;
    }

    //Calculate alignment behavior
    private UnityEngine.Vector3 CalculateAlignment()
    {
        UnityEngine.Vector3 alignment = UnityEngine.Vector3.zero;
        int count = 0;

        foreach (var agent in allAgents)
        {
            if (agent != this && UnityEngine.Vector3.Distance(transform.position, agent.transform.position) < alignmentRadius)
            {
                alignment += agent.rb.velocity;
                count++;
            }
        }

        if (count > 0)
        {
            alignment /= count;
        }

        return alignment.normalized;
    }

    //Calculate cohesion behavior
    private UnityEngine.Vector3 CalculateCohesion()
    {
        UnityEngine.Vector3 cohesion = UnityEngine.Vector3.zero;
        int count = 0;

        foreach (var agent in allAgents)
        {
            if (agent != this && UnityEngine.Vector3.Distance(transform.position, agent.transform.position) < cohesionRadius)
            {
                cohesion += agent.transform.position;
                count++;
            }
        }

        if (count > 0)
        {
            cohesion /= count;
            cohesion -= transform.position;
        }

        return cohesion.normalized;
    }

    //Dynamically adjust agent speed based on obstacles, agents, and safe distance variables
    private void adjustSpeed()
    {
        float closestObstacleDistance = getClosestObstacleDistance();
        float closestAgentDistance = getClosestAgentDistance();
        float desiredSpeed = Mathf.Clamp(maxSpeed - (closestObstacleDistance + closestAgentDistance - safeDistance), minSpeed, maxSpeed);

        if(desiredSpeed > currentSpeed)
        {
            currentSpeed = Mathf.Min(currentSpeed + acceleration * Time.deltaTime, desiredSpeed);
        }
        else if (desiredSpeed < currentSpeed)
        {
            currentSpeed = Mathf.Max(currentSpeed - deceleration * Time.deltaTime, desiredSpeed);
        }
        currentSpeed = moveSpeed;
    }

    //For getting closest obstacle in front of agent
    private float getClosestObstacleDistance()
    {
        int obstacleLayerMask = 1 << 6;
        obstacleLayerMask = ~obstacleLayerMask;
        //Raycast in front of the agent to detect obstacles
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, 4, obstacleLayerMask))
        {
            return hit.distance;
        }
        return Mathf.Infinity; // No obstacle detected
    }

    // Get the distance to the closest agent
    private float getClosestAgentDistance()
    {
        // Iterate through nearby agents and find the closest one
        float closestAgentDistance = Mathf.Infinity;
        foreach (var agent in allAgents)
        {
            float distance = UnityEngine.Vector3.Distance(transform.position, agent.transform.position);
            if (distance < closestAgentDistance)
            {
                closestAgentDistance = distance;
            }
        }
        return closestAgentDistance;
    }


    //Soley used for apply basic movement to agent
    void applyMovement(float moveX, float moveZ)
    {
        moveDirection = new UnityEngine.Vector3(moveX, 0, moveZ).normalized;

        float targetSpeed = moveDirection.magnitude * moveSpeed;

        //Simple movement of the agent
        rb.velocity = moveDirection * targetSpeed;

        //Solely for rotating model to visual direction agent is going
        if (moveDirection != UnityEngine.Vector3.zero)
        {
            //Uses quanternion angles to calculate the look rotation
            UnityEngine.Quaternion targetRotation = UnityEngine.Quaternion.LookRotation(moveDirection);
            //Smoothly interpolates between the current rotation and the target rotation
            //transform.rotation = UnityEngine.Quaternion.Slerp(transform.rotation, targetRotation, 10f * Time.deltaTime);
            rb.rotation = UnityEngine.Quaternion.Slerp(rb.rotation, targetRotation, 10f * Time.deltaTime);
        }
    }

    //Records agent position (used for smooth movement checks)
    void recordPosition()
    {
        previousPositions.Add(transform.localPosition);
    }

    //Used for penalizing agent for making eratic movement
    void checkSmoothMovement()
    {
        //Gets the velocity difference from the current to the last
        UnityEngine.Vector3 velocity = (previousPositions[previousPositions.Count - 1] - previousPositions[0]) / Time.fixedDeltaTime;

        //Check for smooth movement based on velocity magnitude
        float smoothnessThreshold = 250.0f; // Adjust threshold as needed
        if (velocity.magnitude < smoothnessThreshold)
        {
            //Debug.Log("Reward: Not smooth --");
            //Penalize agent for lack of smooth movement
            AddReward(-rewardForSmoothMovement);
        }
        else
        {
            //Debug.Log("Reward: Smooth ++");
            //Rewarded for smooth movement
            AddReward(rewardForSmoothMovement);
        }
    }

    private bool IsObstacleInFront()
    {
        // Set the raycast distance
        float raycastDistance = 3f;
        // Set the raycast angle

        // Define the ray origin
        UnityEngine.Vector3 rayOrigin = transform.position;

        // Define the ray direction (straight)
        UnityEngine.Vector3 rayDirection = transform.forward;

        // Perform the raycast in front
        RaycastHit hitFront;
        if (Physics.Raycast(rayOrigin, rayDirection, out hitFront, raycastDistance))
        {
            // Check if the hit object is an obstacle
            if (hitFront.collider.CompareTag("Obstacle"))
            {
                // An obstacle is in front of the agent
                return true;

            }
        }

        // No obstacle in front, left, or right of the agent
        return false;
    }

    //When an agent makes an action this function is called and the actions taken are calculated based on the ActionBuffer's values
    public override void OnActionReceived(ActionBuffers actions)
    {
       
        //Actions the agent can take (move on the x and z)
        //the first float on the action buffer refers to movement on the x
        float moveX = actions.ContinuousActions[0];
        //the second float on the action buffer refers to movement on the z
        float moveZ = actions.ContinuousActions[1];

        if(IsObstacleInFront())
        {
            Debug.Log("Obstacle in front");
            moveZ += -2f;
        }

        //Apply movement based on the action buffer
        applyMovement(moveX, moveZ);

        //We need to compare distance from last action to current distance to see if we are getting closer
        //If we are not getting closer take some reward but if we are gain some. So Agent will learn to minimize distance thus getting to goal
        currentDistance = UnityEngine.Vector3.Distance(transform.localPosition, goalTransform.localPosition);
        float reward = (currentDistance < lastDistance) ? +rewardForMovingCloser : -rewardForMovingCloser;
        //Debug.Log("Reward: distance reward: " + reward);
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
    public override void OnEpisodeBegin()
    {
        //Debug.Log("New Episode....");
        //Set episode evaluation metrics back to 0
        totalTimeTaken = 0f;
        totalCollisions = 0;
        totalAgentAvoidance = 0;

        //Reset agent position to its starting position
        //transform.localPosition = startPos;

        //Reset reachGoal boolean variable
        reachedGoal = false;

        //Randomize parameters so agent does not learn to go to a certain position
        //Randomize Agent position
        //transform.localPosition = new Vector3(Random.Range(-9.5f, 6.5f), 1.5f, Random.Range(-13.5f, 2.5f));
        //Randomize goal position
        //goalTransform.localPosition = new Vector3(Random.Range(-9.5f, 6.5f), 0.5f, Random.Range(-13.5f, 2.5f));

        //Resetting random goal position for (Obstacles Env)
        //goalTransform.localPosition = new Vector3(13.5f, 0.5f, Random.Range(2.5f, -13.5f));

        //Resetting random agent position for (Obstacles Env) required for training basic navigation
        //transform.localPosition = new Vector3(Random.Range(-5.5f, -9.5f), 1.5f, Random.Range(-13.5f, 2.5f));

        transform.localPosition = startPos;

        //Reset positions
        previousPositions.Clear();

    }

    //What data does the agent need to effectively simulate a crowd?
    /*
    - Its current position
    - Its target position
    - For simplciity sake we dont add too many observations here...
    - ...
    */
    public override void CollectObservations(VectorSensor sensor)
    {
        //Observe current position
        sensor.AddObservation(transform.localPosition);
        //Observe position of goal
        sensor.AddObservation(goalTransform.localPosition);
    }

    //When agent triggers another collider
    private void OnTriggerEnter(Collider other)
    {
        //Used for reward agent for avoiding getting within close proximity of other agents
        if(other.tag == "PersonalSpace")
        {
            //Debug.Log("Reward: Collided with personalspace --");
            AddReward(-rewardForAgentAvoidance);
            currentSpeed *= 0.6f;
        }

        //Rewarded for getting to the goal
        else if(other.tag == "Goal")
        {
            //Debug.Log("Reward: reached goal ++");
            reachedGoal = true;
            //Debug.Log("Reached goal");
            AddReward(+rewardForGoal);
            EndEpisode();
        } 
    }

    /// <summary>
    /// OnCollisionEnter is called when this collider/rigidbody has begun
    /// touching another rigidbody/collider.
    /// </summary>
    /// <param name="other">The Collision data associated with this collision.</param>
    private void OnCollisionEnter(Collision other)
    {
        //Penalized for hitting an obstacle
        if(other.gameObject.tag == "Obstacle")
        {
            Debug.Log("Reward: Hit obstacle --");
            totalCollisions++;
            //Debug.Log("Hit obstacle");
            AddReward(-rewardForCollision * 5);
            EndEpisode();
        }

        //Penalized for hitting a wall (2 times for than hitting an obstacle)
        if(other.gameObject.tag == "Wall")
        {
            //Debug.Log("Reward: Hit wall --");
            totalCollisions++;
            //Debug.Log("Hit wall");
            AddReward(-rewardForCollision * 10);
            EndEpisode();
        }
    }
}
