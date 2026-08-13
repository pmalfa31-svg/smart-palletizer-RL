from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

print("\n--- 1. Inizializzazione Fase 3: Braccio Robotico ---")
env = PalletizerEnv()

print("\n--- 2. Addestramento IA (Obiettivo: Combattere la gravità) ---")
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=150000)

# IL NOME È STATO AGGIORNATO PER NON SOVRASCRIVERE IL VECCHIO MODELLO
model.save("py_scripts/cervello_braccio_v1")
print("\n[OK] Nuovo cervello 'cervello_braccio_v1.zip' salvato!")

print("\n--- 3. Test Finale: Sensori e Sopravvivenza ---")
obs, info = env.reset()
done = False
step_count = 0

# Limitiamo il test a 200 frame. Se il braccio non cade, 
# non vogliamo che il loop giri all'infinito!
while not done and step_count < 200:
    azione, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(azione)
    done = terminated or truncated
    step_count += 1

print(f"\nTest concluso allo step {step_count}.")
if done and step_count < 200:
    print("CRASH! Il braccio è crollato a terra sotto il suo stesso peso.")
else:
    print("SUCCESSO! L'IA ha imparato a sostenere il braccio in aria contro la gravità.")
    
print(f"Sensori (Encoder) -> Angolo Spalla: {obs[0]:.2f} rad, Angolo Gomito: {obs[1]:.2f} rad")
print(f"Ultima Reward ottenuta: {reward:.2f}")