from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

print("\n--- 1. Inizializzazione Fase Finale: Manual Grip (Hitbox 20cm) ---")
env = PalletizerEnv()

print("\n--- 2. Addestramento IA (Cinematica di precisione e uso interruttore) ---")
# 500k step con l'ambiente bilanciato dovrebbero essere sufficienti
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=500000)

model.save("py_scripts/cervello_braccio_v5_manual")
print("\n[OK] Cervello 'cervello_braccio_v5_manual.zip' salvato!")

print("\n--- 3. Test Finale: Grasping Autonomo ---")
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
    print(f"LAUREA OTTENUTA! Il robot ha attivato la ventosa da solo e sollevato il pacco a {box_height:.2f}m!")
elif is_holding:
    print(f"Mezzo successo: ha azionato la presa manuale, ma altezza insufficiente ({box_height:.2f}m).")
else:
    print("FALLIMENTO. L'IA non è riuscita a sincronizzare mira e interruttore della ventosa.")