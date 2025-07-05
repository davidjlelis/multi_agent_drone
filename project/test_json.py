import json
from pprint import pprint

test_world = "" 

world_input = input("""
                    Please select number that corresponds to the active test world: \n
                    1. Test World 1 (Neutral)\n
                    2. Test World 2 (Flood)
                    3. Test World 3 (Forest Fire)\n
                    4. Test World 4 (Earthquake)\n
                    Selection: """)

if world_input == "1":
    test_world = "test_world_1"
elif world_input == "2":
    test_world = "test_world_2"
elif world_input == "3":
    test_world = "test_world_3"

world_description_fp = './world_descriptions/worlds.json'
environment = []

with open(world_description_fp) as f:
    world_description = json.load(f)

for desc in world_description:
    if desc['world_name'] == test_world:
        environment = desc['environment']