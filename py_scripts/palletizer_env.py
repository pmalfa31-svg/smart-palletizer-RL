import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mio_simulatore

class PalletizerEnv(gym.Env):
    def __init__(self):
        super(PalletizerEnv, self).__init__()
        self.sim = mio_simulatore.AmbienteRobot()
        
        # ACTIONS (3): [Shoulder_Vel, Elbow_Vel, Vacuum_Switch]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)
        
        self.current_step = 0
        self.max_steps = 400 

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.current_step = 0
        stato = self.sim.reset()
        return np.array(stato, dtype=np.float32), {}

    def step(self, action):
        self.current_step += 1
        stato, reward, done = self.sim.step(action.tolist())
        
        truncated = False
        if self.current_step >= self.max_steps:
            truncated = True
            
        return np.array(stato, dtype=np.float32), float(reward), done, truncated, {}