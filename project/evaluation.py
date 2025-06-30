import os
import json
from wordcloud import WordCloud
import matplotlib.pyplot as plt

json_folder_path = '../videos/results/JSON/'
results_data = []
video_titles = []
llm_results_per_video = [] # {'video_title': 'title', 'llm_responses': [response_1, response2, ..., responsex]}

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



for i in range(len(results_data)):
    for j in range(len(results_data[i])):
        json_result = json.loads(results_data[i][j])
        if json_result[i][j]['video'] not in video_titles:
            video_titles.append(json_result[i][j]['video'])
            llm_results_per_video.append({'video_title': json_result[i][j]['video'], 'llm_responses': json_result[i][j]['assistance_instructions']})
        else:
            print(video_titles)