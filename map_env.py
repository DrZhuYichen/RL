import numpy as np
import time
import tkinter as tk
import random


UNIT = 40   # pixels
MAZE_H = 8  # grid height
MAZE_W = 8  # grid width


class Map(tk.Tk, object):
    def __init__(self):
        super(Map, self).__init__()
        self.action_space = ['u', 'd', 'l', 'r']
        self.n_actions = len(self.action_space)
        self.title('maze')
        self.geometry('{0}x{1}'.format(MAZE_H * UNIT, MAZE_H * UNIT))
        self.obstacle = [[0, 1], [0, 2], [1, 1], [1, 2], [3, 3], [3, 4], [4, 3], [4, 4],
                         [0, 5], [0, 6], [1, 6],
                         [0, 7], [6, 4], [6, 5], [7, 4], [7, 5], [7, 0], [7, 1], [7, 2]]
        self.origin = self.random_origin()
        self.rectangle_obstacle = []
        self._build_maze()

    def _build_maze(self):
        self.canvas = tk.Canvas(self, bg='white',
                           height=MAZE_H * UNIT,
                           width=MAZE_W * UNIT)

        # create grids
        for c in range(0, MAZE_W * UNIT, UNIT):
            x0, y0, x1, y1 = c, 0, c, MAZE_H * UNIT
            self.canvas.create_line(x0, y0, x1, y1)
        for r in range(0, MAZE_H * UNIT, UNIT):
            x0, y0, x1, y1 = 0, r, MAZE_H * UNIT, r
            self.canvas.create_line(x0, y0, x1, y1)

        # obstacle
        self.draw_obstacle()

        # create red rect
        self.rect = self.canvas.create_rectangle(
            self.origin[0] - 15, self.origin[1] - 15,
            self.origin[0] + 15, self.origin[1] + 15,
            fill='red')

        self.path = [self.canvas.coords(self.rect)]
        # pack all
        self.canvas.pack()

    def reset(self):
        # clear all
        self.canvas.destroy()
        self._build_maze()

        self.update()
        # time.sleep(0.001)
        self.canvas.delete(self.rect)
        self.rect = self.canvas.create_rectangle(
            self.origin[0] - 15, self.origin[1] - 15,
            self.origin[0] + 15, self.origin[1] + 15,
            fill='red')
        # return observation
        pos = self.canvas.coords(self.rect)
        # state_refer
        temp = np.zeros(MAZE_H*MAZE_W+1)
        for obs in self.obstacle:
            index = MAZE_H*obs[1] + obs[0]
            temp[index] = 1
        self.state_refer = list(temp[:])
        state, good_move = self.update_state(pos[0], pos[1])
        observation = state[:]
        return observation

    def step(self, action):
        #updata state
        pos = self.canvas.coords(self.rect)

        self.canvas.create_oval(pos, fill='green')

        # action is ok?
        base_action = np.array([0, 0])
        if action == 0:   # up
            if pos[1] > UNIT:
                base_action[1] -= UNIT
        elif action == 1:   # down
            if pos[1] < (MAZE_H - 1) * UNIT:
                base_action[1] += UNIT
        elif action == 2:   # left
            if pos[0] > UNIT:
                base_action[0] -= UNIT
        elif action == 3:   # right
            if pos[0] < (MAZE_W - 1) * UNIT:
                base_action[0] += UNIT

        # if move collide or leave edge
        repeat = 1
        state_new, reward, done = None, None, None
        if list(base_action) != [0, 0]:
            predict_pos = [(self.canvas.coords(self.rect)[0] + self.canvas.coords(self.rect)[2])/2+base_action[0],
                           (self.canvas.coords(self.rect)[1] + self.canvas.coords(self.rect)[3])/2+base_action[1]]
            predict_pos = [predict_pos[0]//UNIT, predict_pos[1]//UNIT]
            if predict_pos not in self.obstacle:
                repeat = 0

                self.canvas.move(self.rect, base_action[0], base_action[1])  # move agent
                pos_ = self.canvas.coords(self.rect)
                state_new, good_move = self.update_state(pos_[0], pos_[1])  # new state

                # reward function
                r_list = np.array(state_new[0:MAZE_W*MAZE_H])
                # print(r_list)
                # print(np.sum(r_list == 1))
                if np.sum(r_list == 1) == MAZE_W*MAZE_H:
                    reward = 10
                    done = True
                    # print('success')

                elif good_move == 1:
                    reward = 1
                    done = False

                elif pos_ in self.rectangle_obstacle:
                    reward = -5
                    done = False
                    # print('collide')

                else:
                    reward = -1
                    done = False
        return state_new, reward, done, repeat

    def draw_obstacle(self):
        for obs in self.obstacle:
            center = [0.5*UNIT+obs[0]*UNIT, 0.5*UNIT+obs[1]*UNIT]
            pos = [center[0]-15, center[1]-15, center[0]+15, center[1]+15]
            self.canvas.create_rectangle(pos, fill='black')
            self.rectangle_obstacle.append(pos)

    def update_state(self, s_x, s_y):
        'left and up dot of state'
        x = int((s_x-5)/UNIT)
        y = int((s_y-5)/UNIT)
        index = y*MAZE_H + x
        state = self.state_refer
        if state[index] == 0:
            state[index] = 1
            good_move = 1
        else:
            good_move = 0
        state[-1] = index
        return state, good_move

    def random_origin(self):
        number = MAZE_H*MAZE_W
        list = []
        for i in range(number):
            x = i // MAZE_W
            y = i % MAZE_W
            list.append([0.5*UNIT+UNIT*x, 0.5*UNIT+UNIT*y])
        for data in self.obstacle:
            obs = [0.5*UNIT+UNIT*data[0], 0.5*UNIT+UNIT*data[1]]
            list.remove(obs)
        pos = random.choice(list)
        return pos

    def render(self):
        # time.sleep(0.002)
        self.update()


def update():
    for t in range(3):
        s = env.reset()
        while True:
            env.render()
            a = 3
            s, r, done = env.step(a)
            print(done)
            if done:
                break

if __name__ == '__main__':
    env = Map()
    env.after(100, update)
    env.mainloop()