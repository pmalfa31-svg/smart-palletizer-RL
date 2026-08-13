from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

print("\n--- 1. Inizializzazione Curriculum Learning: Auto-Grip ---")
env = PalletizerEnv()

print("\n--- 2. Addestramento IA (Solo Cinematica di sollevamento) ---")
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=500000)

model.save("py_scripts/cervello_braccio_v4_autogrip")
print("\n[OK] Cervello 'cervello_braccio_v4_autogrip.zip' salvato!")

print("\n--- 3. Test Finale: Auto-Grip & Lift ---")
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
    print(f"BINGO! Il robot ha usato l'auto-grip e sollevato il pacco a {box_height:.2f}m!")
elif is_holding:
    print(f"PRESA RIUSCITA! Ma forza insufficiente per alzarlo (Altezza: {box_height:.2f}m).")
else:
    print("FALLIMENTO. L'IA sta ancora ballando intorno al pacco.")