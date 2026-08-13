from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

print("\n--- 1. Inizializzazione Nuovo Magazzino ---")
env = PalletizerEnv()

print("\n--- 2. Addestramento IA (Obiettivo: Centrare il Pallet!) ---")
# Creiamo un nuovo cervello vergine
model = PPO("MlpPolicy", env, verbose=1)

# Addestriamo per 20.000 fotogrammi (l'IA imparerà a evitare il pavimento)
model.learn(total_timesteps=20000)

# Salviamo il nuovo cervello specializzato
model.save("py_scripts/cervello_pallet_v1")
print("\n[OK] Nuovo cervello 'cervello_pallet_v1.zip' salvato!")

print("\n--- 3. Test Finale: Vediamo la mira dell'IA ---")
obs, info = env.reset()
done = False
step_count = 0

while not done:
    # L'IA sceglie l'azione migliore in base a quello che ha appena imparato
    azione, _states = model.predict(obs, deterministic=True)
    
    obs, reward, terminated, truncated, info = env.step(azione)
    done = terminated or truncated
    step_count += 1

print(f"\nBOOM! Impatto allo step {step_count}.")
print(f"Posizione finale pacco -> X: {obs[0]:.2f}, Y: {obs[1]:.2f}, Z: {obs[2]:.2f}")
print(f"Punteggio ottenuto (Reward): {reward}")