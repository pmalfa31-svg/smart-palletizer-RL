from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

print("\n--- 1. Native OOP Environment Initialization ---")
env = PalletizerEnv()

print("\n--- 2. AI Training (Tabula Rasa) ---")
model = PPO("MlpPolicy", env, verbose=1, learning_rate=0.0001)
model.learn(total_timesteps=900000)

model.save("py_scripts/cervello_braccio_v9_dynamic")
print("\n[OK] Model 'cervello_braccio_v9_dynamic.zip' saved!")

print("\n--- 3. Final Test: Grasping OOP ---")
obs, info = env.reset()
done = False

for i in range(600):
    azione, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(azione)
    if terminated or truncated:
        break

# Extracted from the new 10-dimensional state vector
box_height = obs[9]
is_holding = bool(obs[11])

print(f"\nTest completed.")
if is_holding and box_height > 1.0:
    print(f"ARCHITECTURAL TRIUMPH! Native AI successfully lifted the package to {box_height:.2f}m!")
else:
    print(f"Missed. Final height: {box_height:.2f}m.")