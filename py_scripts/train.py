import mio_simulatore
import time

print("Avvio simulazione...")
env = mio_simulatore.AmbienteRobot()

stato = env.reset()
print(f"Stato iniziale (X, Y, Z): {stato}")

done = False
step_count = 0

print("\n--- Inizio caduta libera ---")
while not done:
    # Per ora ignoriamo l'azione, vogliamo solo vedere l'effetto della gravità
    azione = [0.0, 0.0] 
    
    # Facciamo avanzare il mondo di 1/60 di secondo
    stato, reward, done = env.step(azione)
    step_count += 1
    
    # Stampiamo l'altezza del pacco (il valore Y, ovvero stato[1]) ogni 10 step
    if step_count % 10 == 0:
        print(f"Step {step_count} | Altezza pacco: {stato[1]:.2f} m")
        time.sleep(0.1)  # Piccola pausa per vederlo scorrere come un'animazione

print(f"\nBOOM! Il pacco ha toccato terra allo step {step_count}.")
print(f"Altezza finale: {stato[1]:.2f} m (Reward ottenuta: {reward})")