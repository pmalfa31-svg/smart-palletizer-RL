from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

print("\n--- 1. Inizializzazione Ambiente Nativo OOP ---")
env = PalletizerEnv()

print("\n--- 2. Addestramento IA (Tabula Rasa) ---")
# Partiamo da zero. 500k step per dominare la nuova fisica modulare.
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=500000)

model.save("py_scripts/cervello_braccio_v8_dynamic")
print("\n[OK] Cervello 'cervello_braccio_v8_dynamic.zip' salvato!")

print("\n--- 3. Test Finale: Grasping OOP ---")
obs, info = env.reset()
done = False

for i in range(150):
    azione, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(azione)
    if terminated or truncated:
        break

is_holding = bool(obs[6])
box_height = obs[5]

print(f"\nTest completato.")
if is_holding and box_height > 1.0:
    print(f"TRIONFO ARCHITETTURALE! L'IA nativa ha sollevato il pacco a {box_height:.2f}m!")
else:
    print(f"Mancato. Altezza: {box_height:.2f}m.")