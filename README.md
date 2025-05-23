# Multi-Agent Vision-Language-Action Drone Research Project
University of North Florida - School of Computing - CIS6917/6918

Created by: David Lelis (N00957151)

Supervised by: Ayan Dutta, PhD

Update as of 2025.05.18: The system currently is able to detect people in an environment to 70% confidence and return back coordinate information so the server computes a path from origin (center of the map / 50,50) to the goal coordinate that is supposed to be the person found. However, a couple of issues have arose:
1. The system doesn't have a way to determine if a scenario involving a person has already been discovered and skip ahead. This means that one scene with a person is being repeatedly detected until the drone passes them, making for multiple paths to the same person.
2. Issue 1 has also shown that the RRT* algorith may also be creating new trees each time instead of using the exsisting tree. Need to update to use the existing tree and update paths to goal coordinates using that.

## Introduction
Purpose: Create a multi-agent vision-language-action (VLA) drone system that is used to survey an environment for dangerous scenarios or injured individuals.

## Methodology
The framework is made up of four agents: an object detection model to detect people, a Vision-Language Model (VLM), a Large Language Model (LLM), Environment Exploration, and a Pathfinding algorithm. There are assumptions that the system takes:

1. The environment at some point has been mapped in the past via GPS or other mapping system.
2. There may be mulitple victims requiring assistance and multiple SAR first responder teams allowing for all paths start from the homebase.
3. The environment the system is in is confirmed to be considered a disaster, therefore everyone in the environment needs some level of rescue but some may be injured and require specific assistance.

### Object Detection Model: YOLOv8
YOLOv8 is an object detection model that is used to detect people. As it is assumed that people may be in the environment, the model will flag when a person is detected to a 90% confidence that is provided by the results of running the image through the model. This is also used as the VLM is computationally intensive and would slow down the system if it was solely running as the person detection model. If a person is detected in the image, the framework will move the image through the VLM.

### Vision-Language Model (VLM): Florence2-base
The VLM being used in the framework is Florence2-base, a lightweight VLM that can provide basic text from an image. The purpose of the VLM is to provide a description of the image that the object detection model had flagged. The description is then moved through the LLM. 

### Large-Language Model (LLM): Qwen2.5-72B
The LLM used in the framework is Qwen2.5-72B and is ran online as it is computationally intensive, more so than the VLM. The image description is ran through the model with the given prompt:

            You are a search-and-rescue assistant in an area victim to a disaster and are tasked to confirm if people are 
            safe or injured. Currently, it is being done in a 3D simualtion so assume renderings are real. 
            Given the description below, provide a response only if a person is found.

            Description: "{description}"

            Respond with "Person Found" if the description contains a person and if so, determine 
            if the person may be injured or in danger and what assistance they would need from first responders.

As seen, the model is instructed to return a JSON string in order to easily detect if a person is found, if they may require special assistance, and the instructions to assist the victim. This will be sent to the server along with the coordinates of the scene to pass onto first responders to evacuate the victim as necessary.

### Environment Exploration
It is assumed that the environment that the system is placed in as been mapped before, however due to most disaster events causing significant damage and change to an environment, it can't be assumed that the environment is the same as originally mapped. The assumption is placed so drones can easily navigate the environment given previous knowledge, but should also be able to adapt to new environmental changes. This part of the framework will tackle that aspect by including obstacle avoidance techniques. (TBD as of 2025.05.06)

### Pathfinding Algorithm: A* or RRT
Once a victim is found, the finding the fastest path is crucial to keep them safe. Combining the assumption of the environment being previously mapped and the new knowledge of any changes in the environment, a path from the homebase of the drone system to the victim will be made. (TBD as of 2025.05.06)