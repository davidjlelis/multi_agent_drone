import os
import json

json_folder_path = '../videos/results/JSON/'
results_data = []

for filename in os.listdir(json_folder_path):
    if filename.endswith('.json'):
        file_path = os.path.join(json_folder_path, filename)

        with open(file_path, 'r') as f:
            try:
                data = json.load(f)
                # print(f'Results {filename}: {data}\n')
                results_data.append(data)
            except json.JSONDecodeError:
                print(f'Warning: {filename} is not valid JSON')

for result in results_data:
    for i in result:
        json_i = json.loads(i)
        print(json_i['detection_id'], json_i['assistance_instructions'], '\n\n')