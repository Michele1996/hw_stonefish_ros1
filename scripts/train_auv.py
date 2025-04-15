#!/usr/bin/env python3

import rospy
import gymnasium as gym
from custom_env import CustomEnv
import time 

time_move_step_for_auv = 1

def main():
    rospy.init_node('custom_env_node', anonymous=True)
    gym.envs.registration.register(id='CustomEnv-v0', entry_point='custom_env:CustomEnv')
    env = gym.make("CustomEnv-v0")
    episodes = 10
    max_steps_per_episode = 10
    state = env.reset(sim_reset=False)
    for episode in range(episodes):
        total_reward = 0

        for step in range(max_steps_per_episode):
            action = env.action_space.sample()
            next_state, reward, done, _ = env.step(action)
            time.sleep(time_move_step_for_auv)
            total_reward += reward
            print(f"Episode: {episode}, Step: {step}, Action: {action}, Step Reward: {reward}")
            #if done:
            #    break
            
        state=env.reset()
        print(f"End of Episode {episode}, Total Reward: {total_reward}")

    env.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
