import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mio_simulatore

class PalletizerEnv(gym.Env):
    def __init__(self):
        super().__init__()
        # Inizializziamo il nostro "motore" C++
        self.sim = mio_simulatore.AmbienteRobot()
        
        # 1. ACTION SPACE: Definiamo le forze che l'IA può applicare [Forza_X, Forza_Z]
        # Diciamo all'IA che può applicare un valore continuo tra -10.0 e +10.0
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        
        # 2. OBSERVATION SPACE: Quello che l'IA "vede" [X, Y, Z]
        # Valori che vanno da -infinito a +infinito
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)

    def reset(self, seed=None, options=None):
        # Gymnasium richiede questa struttura standard per il reset
        super().reset(seed=seed)
        
        # Chiamiamo la funzione reset() che abbiamo scritto in C++
        stato_iniziale = self.sim.reset()
        
        # Convertiamo la lista del C++ in un array NumPy (lo standard per l'IA)
        osservazione = np.array(stato_iniziale, dtype=np.float32)
        info = {} # Dizionario vuoto per informazioni extra
        
        return osservazione, info

    def step(self, action):
        # L'IA ci passa un array NumPy, noi lo convertiamo in lista per il C++
        azione_lista = action.tolist()
        
        # Facciamo calcolare la fisica al C++
        stato, reward, done = self.sim.step(azione_lista)
        
        # Prepariamo i valori per Gymnasium
        osservazione = np.array(stato, dtype=np.float32)
        terminated = done       # L'episodio finisce (il pacco tocca terra)
        truncated = False       # Serve per i limiti di tempo (per ora non lo usiamo)
        info = {}
        
        return osservazione, reward, terminated, truncated, info