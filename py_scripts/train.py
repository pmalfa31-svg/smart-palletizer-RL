import mio_simulatore
import time

print("Avvio simulazione...")
env = mio_simulatore.AmbienteRobot()

stato = env.reset()
print(f"Stato iniziale (X, Y, Z): {stato}")

done = False
step_count = 0

print("\n--- Inizio caduta libera con spinta laterale ---")
while not done:
    # Questa volta spingiamo costantemente il pacco verso destra (X positivo)
    azione = [5.0, 0.0] 
    
    stato, reward, done = env.step(azione)
    step_count += 1
    
    if step_count % 10 == 0:
        # Ora stampiamo sia l'altezza Y che lo spostamento laterale X
        print(f"Step {step_count} | Altezza Y: {stato[1]:.2f} m | Spostamento X: {stato[0]:.2f} m")
        time.sleep(0.1)

print(f"\nBOOM! Il pacco ha toccato terra allo step {step_count}.")
print(f"Posizione finale -> X: {stato[0]:.2f} m, Y: {stato[1]:.2f} m (Reward: {reward})")