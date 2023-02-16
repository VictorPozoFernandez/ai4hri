import openai
import os
import numpy as np
from sklearn.cluster import KMeans

openai.organization = os.environ.get("OPENAI_ORG_ID")
openai.api_key = os.environ.get("OPENAI_API_KEY")

model = "text-embedding-ada-002"

res = openai.Embedding.create( input = ["this camera is available in black, white and silver"], engine=model)

list=[]
for vec in res["data"]:
    list.append(vec["embedding"])

arr = np.asarray(list)
keyword_dict = np.load('/home/victor/catkin_ws_AI4HRI/src/ai4hri/scripts/keyword_dict.npy',allow_pickle='TRUE').item()
kmeans = np.load('/home/victor/catkin_ws_AI4HRI/src/ai4hri/scripts/kmeans_model.npy',allow_pickle='TRUE').item()

y = kmeans.predict(arr)
print("Cluster:" + str(y[0]))
print("Keywords:" + str(keyword_dict[y[0]]))