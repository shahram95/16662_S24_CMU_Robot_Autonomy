# Author: Vibhakar Mohta (vmohta@cs.cmu.edu)

import numpy as np
import matplotlib.pyplot as plt
import random 

class RandomAgent():        
    def __init__(self):
        self.available_actions = ['UP', 'DOWN', 'LEFT', 'RIGHT']
        
    # Choose a random action
    def get_action(self, state, explore=True):
        # TODO: randomly choose an action
        return random.choice(self.available_actions)

class QAgent():
    def __init__(self, environment, alpha=0.1, gamma=0.99, epsilon=0.1):
        self.environment = environment
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        
        # TODO: Initialize the Q-table with random values
        self.q_table = dict() 
        
        for row in range(self.environment.height):
            for col in range(self.environment.width):
                self.q_table[(row, col)] = {action: np.random.uniform(-1, 0) for action in self.environment.actions}
        
        for terminal_state in self.environment.terminal_states:
            self.q_table[terminal_state] = {action: 0.0 for action in self.environment.actions}
        
    def get_action(self, state, explore=True):
        """
        Returns the optimal action from Q-Value table. 
        Performs epsilon greedy exploration if explore is True (during training)
        If multiple optimal actions, chooses random choice.
        """
        # TODO: Implement this function
        if np.random.uniform(0,1) < self.epsilon and explore:
            action = random.choice(self.environment.actions)
        else:
            max_q_value = max(self.q_table[state].values())
            max_actions = [action for action, q_value in self.q_table[state].items() if q_value == max_q_value]
            action = random.choice(max_actions)
        return action
    
    def update(self, state, reward, next_state, action):
        """
        Updates the Q-value tables using Q-learning
        """
        # TODO: Implement this function
        
        best_next_action = max(self.q_table[next_state], key=self.q_table[next_state].get)
        self.q_table[state][action] += self.alpha * (reward + self.gamma * self.q_table[next_state][best_next_action] - self.q_table[state][action])
    
    def visualize_q_values(self, SCALE=100):
        """
        In the grid, plot the arrow showing the best action to take in each state.
        """
        plt.figure()
        # Display grid (red as bomb, yellow as gold)
        img = np.zeros((self.environment.height*SCALE, self.environment.width*SCALE, 3)) + 0.01
        for bomb_location in self.environment.bomb_locations:
            img[bomb_location[0]*SCALE:bomb_location[0]*SCALE+SCALE, bomb_location[1]*SCALE:bomb_location[1]*SCALE+SCALE] = [1, 0, 0]
        img[self.environment.gold_location[0]*SCALE:self.environment.gold_location[0]*SCALE+SCALE, self.environment.gold_location[1]*SCALE:self.environment.gold_location[1]*SCALE+SCALE] = [1, 1, 0]
        
        # Display best actions (print as text), show as blue arrow
        for i in range(self.environment.height):
            for j in range(self.environment.width):
                # skip terminal states
                if (i, j) in self.environment.terminal_states:
                    continue
                state = (i, j)
                best_action = self.get_action(state)
                if best_action == 'UP':
                    plt.arrow(j*SCALE+SCALE//2, i*SCALE+SCALE//2, 0, -40, color='blue', head_width=10)
                elif best_action == 'DOWN':
                    plt.arrow(j*SCALE+SCALE//2, i*SCALE+SCALE//2, 0, 40, color='blue', head_width=10)
                elif best_action == 'LEFT':
                    plt.arrow(j*SCALE+SCALE//2, i*SCALE+SCALE//2, -40, 0, color='blue', head_width=10)
                elif best_action == 'RIGHT':
                    plt.arrow(j*SCALE+SCALE//2, i*SCALE+SCALE//2, 40, 0, color='blue', head_width=10)
        
        plt.imshow(img)
        plt.xticks(np.arange(0, self.environment.width*SCALE, SCALE), np.arange(0, self.environment.width, 1))
        plt.yticks(np.arange(0, self.environment.height*SCALE, SCALE), np.arange(0, self.environment.height, 1))
        plt.title('Best Q-Value Actions')
        plt.savefig('q_values.png')
