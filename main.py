
from map_env import Map
from RL import QLearningTable
import matplotlib.pyplot as plt

def update():

    for episode in range(1000000):
        # initial observation
        observation = env.reset()

        step = 1
        flag = 1
        while flag:
            # fresh env
            env.render()

            # RL choose action based on observation
            while True:
                action = RL.choose_action(str(observation))

                # RL take action and get next observation and reward
                observation_, reward, done, repeat = env.step(action)
                if repeat == 0:
                    break

            # RL learn from this transition
            RL.learn(str(observation), action, reward, str(observation_))

            # swap observation
            observation = observation_[:]

            # break while loop when end of this episode
            if done:
                flag = 0

            step += 1
        print('episode:{0},step:{1}'.format(episode, step))

        # data record
        data_episode.append(episode)
        data_step.append(step)
        # print(RL.q_table)

    # end of game
    env.destroy()

if __name__ == "__main__":
    env = Map()
    RL = QLearningTable(actions=list(range(env.n_actions)))

    data_episode = []
    data_step = []
    env.after(100, update)
    env.mainloop()

    print('data show')

    plt.figure()
    plt.plot(data_episode, data_step)
    plt.xlabel('episode')
    plt.ylabel('data_step')
    plt.show()
