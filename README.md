# Multi-Agent Vision-Language-Action Drone Research Project
University of North Florida - School of Computing - CIS6917/6918

Created by: David Lelis (N00957151)

Supervised by: Ayan Dutta, PhD

Update as of 2025.07.03: The system is currently able to detect people in an environment, determine what the person is doing the scene, return back a response to assist the person, and provide fairly accurate coordinates of the person's location. Testing is currently being done on the environment to evaluate accuracy and quality of the response. 

## Introduction
Purpose: Create a multi-agent vision-language-action (VLA) drone system that is used to survey an environment for dangerous scenarios or injured individuals.

## Methodology
The framework is made up of four agents: an object detection model to detect people, a Vision-Language Model (VLM), a Large Language Model (LLM), and a Pathfinding algorithm. There are assumptions that the system takes:

1. The environment at some point has been mapped in the past via GPS or other mapping system to observe obstacles.
2. There may be mulitple victims requiring assistance and multiple SAR first responder teams allowing for all paths start from the homebase.
3. The environment the system is in is confirmed to be considered a disaster and people in the environment may require direct assistance.

The framework flows from object detection to a VLM that computes a description of a scene to a LLM that computes a response to the description and to a Pathfinding algorithm that computes a path from the origin point to the victim(s) location.

### Object Detection Model: YOLOv8
YOLOv8 is an object detection model that is used to detect people. As it is assumed that people may be in the environment, the model will flag when a person is detected to a 40% confidence that is provided by the results of running the image through the model. This percentage was found to be the most effective due to the unique angle that drones observe when flying above people. This is also used as the VLM is computationally intensive and would slow down the system if it was solely running as the person detection model. If a person is detected in the image, the framework will move the image through the VLM.

### Vision-Language Model (VLM): Florence2-base
The VLM being used in the framework is Florence2-base, a lightweight VLM that can provide basic text from an image. The purpose of the VLM is to provide a description of the image that the object detection model had flagged. The description is then moved through the LLM to provide a response to the details in the scene. 

### Large-Language Model (LLM): Qwen2.5-72B
The LLM used in the framework is Qwen2.5-72B and is ran online as it is computationally intensive, more so than the VLM. The image description is ran through the model with the given prompt:

            You are a search-and-rescue assistant in an area victim to a disaster and are tasked to confirm if people are 
            safe or injured. Currently, it is being done in a 3D simulation so assume all 3D renderings are real. 
            Given the description below, provide a response only if a person is found.

            Description: "{description}"

            Explain the scene. Respond with "1. Person Found" if the description includes a person. If so, include in the response "2. Person Requires Assistance" if and only if the person may be injured or in a dangerous situation. If they are not injured or in a dangerous situation, state "2. Person does not require assistance". If the person does require assistance, include "3. Assistance needed" and the kind of assistance they would need from first responders.

The response to the description allows for a second level of filteration as YOLO may pick up similar humanoid shapes that may not be people in a scene. LLM is also tasked to analyze the description to determine if the person is in a dangerous situation and if so, what assistance they need from first responders.

### Pathfinding Algorithm: Rapidly-Exploring Random Trees Star (RRT*)
The pathfinding algorithm used in the current framework is RRT* due to it's efficiency. A new tree is created and searched for each victim found.

## Future Work
In terms of implementation, the future research should explore more efficient methods to execute the framework and more detailed environments. Currently, the framework is having trouble detecting persons in the current simulation due to the lack of detail some of these models have. Secondly, the VLM-aspect of the framework is having trouble with properly describing the scene percisely. For example in one scene, a person is lying infront of a forest. However, the VLM describes the person as "A man in a suit is jumping in the air." This may be due to the drone's position, the simulation's lack of detail of the ground, or the person's position while lying flat.