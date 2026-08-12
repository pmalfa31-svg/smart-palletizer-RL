from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO
import time

print("\n--- 1. Inizializzazione Ambiente ---")
env = PalletizerEnv()

print("\n--- 2. Caricamento del Cervello IA ---")
# Carichiamo il file .zip che contiene la rete neurale
model = PPO.load("py_scripts/cervello_palletizer")
print("[OK] Rete Neurale caricata con successo!")

print("\n--- 3. Test dell'IA al comando ---")
obs, info = env.reset()
done = False
step_count = 0

while not done:
    # LA MAGIA È QUI: L'IA guarda le coordinate (obs) e decide l'azione!
    # deterministic=True significa che prenderà l'azione che reputa migliore in assoluto
    azione, _states = model.predict(obs, deterministic=True)
    
    # Eseguiamo l'azione decisa dall'IA nel mondo C++
    obs, reward, terminated, truncated, info = env.step(azione)
    done = terminated or truncated
    step_count += 1
    
    if step_count % 10 == 0:
        print(f"Step {step_count} | Posizione X: {obs[0]:.2f}, Y: {obs[1]:.2f}")
        time.sleep(0.1) # Rallentiamo per goderci lo spettacolo

print(f"\nImpatto allo step {step_count}!")
print(f"Posizione finale -> X: {obs[0]:.2f}, Y: {obs[1]:.2f}, Z: {obs[2]:.2f}")