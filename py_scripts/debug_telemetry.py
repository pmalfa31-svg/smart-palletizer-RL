from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

print("\n--- AVVIO TELEMETRIA DIAGNOSTICA ---")
env = PalletizerEnv()
model = PPO.load("py_scripts/cervello_braccio_v8_dynamic")

obs, info = env.reset()

print(f"{'FRAME':<6} | {'BOX_X':<8} | {'BOX_Y':<8} | {'DIST_MANO_PACCO':<16} | {'COMANDO_PRESA':<14} | {'IN_MANO'}")
print("-" * 75)

for i in range(600):
    azione, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(azione)
    
    box_x = obs[4]
    box_y = obs[5]
    comando_presa = azione[2]
    is_holding = bool(obs[7])
    
    # Calcoliamo la distanza mano-pacco estrapolandola dal reward
    if is_holding:
        dist = 0.0
    elif reward < -1000:
        # Se c'è stata la penalità di -2000 per scontro/caduta, la ripuliamo
        dist = abs(reward + 2000)
    else:
        dist = abs(reward)

    grip_str = "ATTIVO" if comando_presa > 0 else "SPENTO"
    hold_str = "SI" if is_holding else "NO"
    
    # Stampiamo i dati solo quando il pacco entra nella zona calda (X < 0.5 metri)
    # per evitare di inondare il terminale con frame inutili
    
    print(f"{i:<6} | {box_x:>5.2f}m   | {box_y:>5.2f}m   | {dist:>12.3f}m   | {grip_str:<14} | {hold_str}")
        
    if terminated or truncated:
        print(f"\n--- SIMULAZIONE TERMINATA AL FRAME {i} ---")
        break