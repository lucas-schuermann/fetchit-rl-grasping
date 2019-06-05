import gym
import time
import random
import numpy as np
from random import randint
from gym.utils import seeding
from gym.envs.robotics import rotations

ep_returns = []
actions = []
observations = []
rewards = []
infos = []


def main():
    env = gym.make('LVSRotatedFetchPickAndPlace-v1')
    env._max_episode_steps *= 1.25
    numItr = 100
    initStateSpace = "random"

    env.reset()
    print("Reset!")
    time.sleep(1)

    while len(actions) < numItr:
        obs = env.reset()
        env.render()
        print("Reset!")
        print("ITERATION NUMBER ", len(actions))
        goToGoal(env, obs)

    fileName = "data_rotated"
    fileName += "_" + initStateSpace
    fileName += "_" + str(numItr)
    fileName += ".npz"

    np.savez_compressed(fileName, acs=actions, obs=observations, info=infos)


def goToGoal(env, lastObs):

    #goal = self.sampleGoal()
    goal = lastObs['desired_goal']

    #objectPosition
    objectPos = lastObs['observation'][3:6]
    gripperPos = lastObs['observation'][:3]
    gripperState = lastObs['observation'][9:11]
    objectRot = lastObs['observation'][12:15]  # Euler angles
    object_rel_pos = lastObs['observation'][6:9]

    #print("relative position ", object_rel_pos)
    #print("Goal position ", goal)
    #print("gripper Position ", gripperPos)
    #print("Object Position ", objectPos)
    #print("Object rotation ", objectRot)
    #print("Gripper state  ", gripperState)

    episodeAcs = []
    episodeObs = []
    episodeInfo = []

    object_oriented_goal = object_rel_pos.copy()
    object_oriented_goal[2] += 0.03

    print("Max episode steps ", env._max_episode_steps)

    timeStep = 0

    episodeObs.append(lastObs)

    while np.linalg.norm(object_oriented_goal
                        ) >= 0.0025 and timeStep <= env._max_episode_steps:
        env.render()
        action = [0, 0, 0, 0] + [1., 0., 1., 0.]

        object_oriented_goal = object_rel_pos.copy()
        object_oriented_goal[2] += 0.03
        object_rot_goal = rotations.euler2quat([0, 0, objectRot[1]])

        for i in range(len(object_oriented_goal)):
            action[i] = object_oriented_goal[i] * 6

        action[4:] = object_rot_goal
        action[3] = 0.05

        obsDataNew, reward, done, info = env.step(action)
        timeStep += 1

        episodeAcs.append(action)
        episodeInfo.append(info)
        episodeObs.append(obsDataNew)

        # note: don't need to reobserve object orientation, no online correction
        objectPos = obsDataNew['observation'][3:6]
        gripperPos = obsDataNew['observation'][:3]
        gripperState = obsDataNew['observation'][9:11]
        object_rel_pos = obsDataNew['observation'][6:9]

    while np.linalg.norm(
            object_rel_pos) >= 0.0025 and timeStep <= env._max_episode_steps:
        env.render()
        action = [0, 0, 0, 0] + [1., 0., 1., 0.]

        object_rot_goal = rotations.euler2quat([0, 0, objectRot[1]])
        for i in range(len(object_rel_pos)):
            action[i] = object_rel_pos[i] * 6

        action[4:] = object_rot_goal
        action[3] = -0.005

        obsDataNew, reward, done, info = env.step(action)
        timeStep += 1

        episodeAcs.append(action)
        episodeInfo.append(info)
        episodeObs.append(obsDataNew)

        objectPos = obsDataNew['observation'][3:6]
        gripperPos = obsDataNew['observation'][:3]
        gripperState = obsDataNew['observation'][9:11]
        object_rel_pos = obsDataNew['observation'][6:9]

    while np.linalg.norm(
            goal - objectPos) >= 0.001 and timeStep <= env._max_episode_steps:
        env.render()
        action = [0, 0, 0, 0] + [1., 0., 1., 0.]

        object_rot_goal = rotations.euler2quat([0, 0, objectRot[1]])
        for i in range(len(goal - objectPos)):
            action[i] = (goal - objectPos)[i] * 6

        action[4:] = object_rot_goal
        action[3] = -0.005

        obsDataNew, reward, done, info = env.step(action)
        timeStep += 1

        episodeAcs.append(action)
        episodeInfo.append(info)
        episodeObs.append(obsDataNew)

        objectPos = obsDataNew['observation'][3:6]
        gripperPos = obsDataNew['observation'][:3]
        gripperState = obsDataNew['observation'][9:11]
        object_rel_pos = obsDataNew['observation'][6:9]

    while True:
        env.render()
        action = [0, 0, 0, 0] + [1., 0., 1., 0.]

        action[3] = -0.005

        obsDataNew, reward, done, info = env.step(action)
        timeStep += 1

        episodeAcs.append(action)
        episodeInfo.append(info)
        episodeObs.append(obsDataNew)

        objectPos = obsDataNew['observation'][3:6]
        gripperPos = obsDataNew['observation'][:3]
        gripperState = obsDataNew['observation'][9:11]
        object_rel_pos = obsDataNew['observation'][6:9]

        if timeStep >= env._max_episode_steps: break

    #print("Toatal timesteps taken ", timeStep)

    actions.append(episodeAcs)
    observations.append(episodeObs)
    infos.append(episodeInfo)


if __name__ == "__main__":
    main()
