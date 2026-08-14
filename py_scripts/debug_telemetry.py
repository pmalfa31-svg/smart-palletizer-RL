import math
from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

print("\n--- AVVIO TELEMETRIA DIAGNOSTICA ---")
env = PalletizerEnv()

# FIX 1: Aggiornato alla versione v9 e aggiunto env=env per il tensore
try:
    model = PPO.load("py_scripts/cervello_braccio_v9_dynamic", env=env)
except Exception as e:
    print(f"Errore caricamento modello: {e}")
    exit()

obs, info = env.reset()

print(f"{'FRAME':<7} | {'BOX_X':<7} | {'BOX_Y':<7} | {'DIST_EE_PACCO':<15} | {'COMANDO_PRESA':<15} | {'IN_MANO'}")
print("-" * 75)

for i in range(400):  # Allineato ai nuovi 400 step
    azione, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(azione)

    # FIX 2: Allineamento ai nuovi indici dell'array a 12 dimensioni
    ee_x = obs[6]
    ee_y = obs[7]
    box_x = obs[8]
    box_y = obs[9]
    is_holding = bool(obs[11])
    
    # L'azione è un array di 3 valori: [Vel_Spalla, Vel_Gomito, Interruttore_Ventosa]
    comando_presa = azione[2] 

    # Calcolo della distanza reale (se lo ha in mano, la forziamo a 0)
    dist = math.sqrt((ee_x - box_x)**2 + (ee_y - box_y)**2) if not is_holding else 0.0

    print(f"{i:<7} | {box_x:<7.2f} | {box_y:<7.2f} | {dist:<15.2f} | {comando_presa:<15.2f} | {is_holding}")

    if terminated or truncated:
        print("\n[!] Episodio Terminato.")
        break