# Multi-Agent Vision-Language-Action Drone Research Project
University of North Florida - School of Computing - CIS6917/6918

Created by: David Lelis (N00957151)

Supervised by: Ayan Dutta, PhD

## Introduction
Purpose: Create a multi-agent vision-language-action (VLA) drone system that is used to survey an environment for dangerous scenarios or injured individuals.

## Methodology
The framework is made up of four agents: a Vision-Language Model (VLM), a Large Language Model (LLM), Environment Exploration, and a Pathfinding algorithm.

### Vision-Language Model (Florence2-base)
The VLM utilized for this study is Florence2-base, that uses the onboard computer. The purpose of the VLM is to take an image and return text regarding the image. For this study, the VLM is prompted to describe the scene in the image. The drone takes the live images every 5 seconds of the environment and provides a description of the scene. The description is then passed to the LLM for further evaluation.

Latest update (2025.04.26): The VLM is fully implemented but an issue in regards to performance has been observed. The full image-to-text description process takes a few seconds creating lags in the system.

### Large Language Model (Qwen2.5-72B-Instruct)
The LLM utilized for this study is Qwen2.5-72B-Instruct, where the text description from the VLM is evaulated via a Cloud API from Hugging Face. The LLM is prompted to analyze the text description and determine if the scene contains an urgent situations in regards to an injured person or anything dangerous. If an urgent situation is found, LLM returns a "Yes" and a set of instructions to assist any individuals in the scene. The next steps would then to return location points to a server that acts as the middle-man between the drones and first responders.

Latest update (2025.04.26): The LLM is fully implmented and provides the confirmation and instructions when urgent scenes are described. Due to some limitations or unexpected observations, the VLM-LLM process is discovering unintended dangerous scenes. For example due to the similarly sandy color of the wooden floor, when the drone takes an image of a single person on the ground, the VLM describes the scene as "A person walking through the desert", which is then sent to the LLM which confirms that the text description is dangerous and provides feedback. While the scene is actually dangerous, the intention of the person wasn't to place them in an urgent scene.

### Environment Exploration (SLAM)
The drone is defaulted to explore the environment for mapping and to record any obstacles. While the drone is mapping, it is also performing the VLM-to-LLM process to locate urgent situations.

Latest update (2025.04.26): TBD

### Pathfinding Algorithm (RRT)
Once an urgent scene is found, the drone returns the location of the scene to a server that acts as the middle-man between the drones and first responders. While notifying the first responders, a pathfinding algorithm is initiated to direct first responders to the fastest path to the scene.

Latest update (2025.04.26): TBD