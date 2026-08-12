import mio_simulatore

print("Avvio simulazione...")
env = mio_simulatore.AmbienteRobot()

stato = env.reset()
print(f"Stato iniziale dal C++: {stato}")

nuovo_stato, reward, done = env.step([1.5, 2.0])
print(f"Risultato step: Stato={nuovo_stato}, Reward={reward}")