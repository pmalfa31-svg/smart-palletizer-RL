import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mio_simulatore

class PalletizerEnv(gym.Env):
    def __init__(self):
        super(PalletizerEnv, self).__init__()
        self.sim = mio_simulatore.AmbienteRobot()
        
        # ACTION SPACE: 2 motori (Spalla e Gomito)
        # La rete neurale sputerà fuori 2 numeri compresi tra -1.0 e 1.0.
        # (Il C++ poi li moltiplica per 2.0 per ottenere i radianti al secondo reali)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        
        # OBSERVATION SPACE: 4 sensori
        # [angolo_spalla, angolo_gomito, velocità_spalla, velocità_gomito]
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        stato = self.sim.reset()
        return np.array(stato, dtype=np.float32), {}

    def step(self, action):
        stato, reward, done = self.sim.step(action.tolist())
        truncated = False 
        return np.array(stato, dtype=np.float32), float(reward), done, truncated, {}