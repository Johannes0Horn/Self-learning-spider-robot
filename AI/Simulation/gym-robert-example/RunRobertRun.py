import gym
import gym_robert
import time
import matplotlib.pyplot as plt
import cv2
import numpy as np
from tqdm import tqdm
import pandas as pd


env = gym.make("robert-v0")
env.reset()
observation_space = env.observation_space
action_space = env.action_space

state_size = env.observation_space.shape[0]
action_size = action_space.shape[0]
totalSteps = 0
numEpochs = 1000

data = pd.read_csv("D:/Studium/11 WS_19/RoboRob/Repo/ai/Simulation/gym-robert-example/actionlist.csv")
networkOutputList = data.values.tolist()

for epoch in tqdm(range(0,numEpochs),position=1,leave=True):
    state = env.reset()

    #Set this to True if the current epoch should be captured as a video. Use it rarely because its EXTREME SLOW!
    captureEpoch = False

    if ((epoch+1) % 25) == 0:
        captureEpoch = True

    #Create the video writer, resolution is fixed!
    if captureEpoch:
        videoResolution = (384,384)
        videoFPS = 30
        videoName = "Epoch"+str(epoch)+".avi"
        videoFourcc = cv2.VideoWriter_fourcc(*"MJPG")
        videoEpoch = cv2.VideoWriter(videoName,videoFourcc,videoFPS,videoResolution)

    counter=0
    vorzeichen = 1
    while True:
        #action = env.action_space.sample()
        #action = np.random.uniform(0,2*3.14,4)
        #action = np.zeros(12)
        #action[5] = 1
        #action[6:12] = 0.5
        #if counter > 1 or counter < 0:
        #        vorzeichen*=-1
        #    counter += 0.1 * vorzeichen

        for actions in networkOutputList:
            #print(actions)
           

            next_state, reward, done, info = env.step(actions)

            #print(action[0])
            if captureEpoch:
                images = env.render()
                if len(images) > 0:
                    for img in images:
                        #Write to video
                        if img.shape[0] != videoResolution[0] or img.shape[1] != videoResolution[0]:
                            print("Video resolution should match image resoltuion!")
                        else:
                            rgbImage0to1 = img[:,:,:3]
                            videoEpoch.write( (rgbImage0to1 * 255).astype('uint8'))

            totalSteps += 1
            #stop current simulation
            #if done:
                #break



        #time.sleep(1/30)
    #Save video of the currently tracked epoch
    if captureEpoch:   
        videoEpoch.release()

        
        
