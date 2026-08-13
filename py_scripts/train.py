from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

print("\n--- 1. Inizializzazione Fase 4: Target Kinematics ---")
env = PalletizerEnv()

print("\n--- 2. Addestramento IA (Obiettivo: Raggiungere il target) ---")
# Aumentiamo i timesteps a 300k per dare tempo all'IA di mappare lo spazio
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=300000)

# Salviamo come v2 perché la logica di controllo è cambiata
model.save("py_scripts/cervello_braccio_v2")
print("\n[OK] Nuovo cervello 'cervello_braccio_v2.zip' salvato!")

print("\n--- 3. Test Finale: Precisione di Raggiungimento ---")
obs, info = env.reset()
done = False
# Testiamo su 50 step per vedere se il braccio si muove verso il target
for i in range(50):
    azione, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(azione)
    if terminated or truncated:
        break

print(f"\nTest completato.")
print(f"Sensori finali -> Angolo Spalla: {obs[0]:.2f} rad, Angolo Gomito: {obs[1]:.2f} rad")
print(f"Ultima Reward (Distanza dal target): {reward:.2f}")