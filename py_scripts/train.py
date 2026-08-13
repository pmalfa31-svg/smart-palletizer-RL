from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

print("\n--- 1. Inizializzazione Fase 2: Stacking ---")
env = PalletizerEnv()

print("\n--- 2. Addestramento IA (Obiettivo: Torre di scatole!) ---")
# Usiamo 30.000 fotogrammi per assicurarci che capisca i nuovi limiti ristretti
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=150000)

model.save("py_scripts/cervello_stacking_v1")
print("\n[OK] Nuovo cervello 'cervello_stacking_v1.zip' salvato!")

print("\n--- 3. Test Finale: L'incastro perfetto ---")
obs, info = env.reset()
done = False
step_count = 0

while not done:
    azione, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(azione)
    done = terminated or truncated
    step_count += 1

print(f"\nBOOM! Impatto allo step {step_count}.")
print(f"Posizione finale pacco -> X: {obs[0]:.2f}, Y: {obs[1]:.2f}, Z: {obs[2]:.2f}")
print(f"Punteggio ottenuto (Reward): {reward}")