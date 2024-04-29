using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

public class CrowdSimManager : MonoBehaviour
{
    private int numAgents;
    private GameObject[] agents;
    private int counter = 0;

    private void Start()
    {
        agents = GameObject.FindGameObjectsWithTag("Agent");

        numAgents = agents.Length;
        
    }

    void FixedUpdate()
    {
        foreach(GameObject agent in agents)
        {
             if(agent.GetComponent<CrowdSimAgent>().reachedGoal)
             {
                //agent.SetActive(false);
                counter++;
             }
        }
        if(counter == numAgents)
        {
            Debug.Log("Resetting Entire Scene");
            foreach(GameObject agent1 in agents)
            {
                agent1.GetComponent<CrowdSimAgent>().EndEpisode();
                //agent1.SetActive(true);
            }
        }
        counter = 0;
    }

}
