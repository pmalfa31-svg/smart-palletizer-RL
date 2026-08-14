import pygame
import sys
from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

# --- CONFIGURAZIONE PYGAME ---
WIDTH, HEIGHT = 800, 600
SCALE = 200  # 1 metro = 200 pixel
OFFSET_X, OFFSET_Y = 400, 500  # L'origine (0,0) del mondo C++ sullo schermo

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Smart Palletizer - 2D Radar")
clock = pygame.time.Clock()

def world_to_screen(x, y):
    """Converte le coordinate in metri del C++ in pixel per lo schermo"""
    screen_x = int(OFFSET_X - (x * SCALE))  # Invertiamo X per far scorrere da destra a sinistra
    screen_y = int(OFFSET_Y - (y * SCALE))  # Invertiamo Y perché in Pygame lo 0 è in alto
    return screen_x, screen_y

# --- CARICAMENTO AMBIENTE ---
env = PalletizerEnv()
try:
    model = PPO.load("py_scripts/cervello_braccio_v8_dynamic")
except:
    print("Modello non trovato, avvio simulazione con azioni casuali per testare la grafica.")
    model = None

obs, info = env.reset()

running = True
step_counter = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 1. LOGICA IA
    if model:
        action, _ = model.predict(obs, deterministic=True)
    else:
        action = env.action_space.sample()

    obs, reward, terminated, truncated, info = env.step(action)
    
    # Estraiamo i dati di telemetria
    ee_x, ee_y = obs[0], obs[1]
    box_x, box_y = obs[4], obs[5]
    is_holding = obs[7] > 0.5

    # 2. RENDER GRAFICO
    screen.fill((30, 30, 30)) # Sfondo grigio scuro

    # Disegna il Nastro Trasportatore (Linea blu)
    belt_start = world_to_screen(2.5, 0.1)
    belt_end = world_to_screen(-1.0, 0.1)
    pygame.draw.line(screen, (50, 150, 250), belt_start, belt_end, 5)

    # Disegna il Pacco (Rettangolo rosso/verde se afferrato)
    box_screen_x, box_screen_y = world_to_screen(box_x, box_y)
    box_color = (50, 250, 50) if is_holding else (250, 50, 50)
    box_rect = pygame.Rect(box_screen_x - 15, box_screen_y - 15, 30, 30) # 30x30 pixel
    pygame.draw.rect(screen, box_color, box_rect)

    # Disegna il Braccio Robotico (Linea bianca con pallino finale)
    base_pos = world_to_screen(0.0, 0.0)
    ee_pos = world_to_screen(ee_x, ee_y)
    pygame.draw.line(screen, (200, 200, 200), base_pos, ee_pos, 4)
    pygame.draw.circle(screen, (255, 255, 0), ee_pos, 8) # Ventosa gialla

    # Testo a schermo
    font = pygame.font.SysFont(None, 24)
    text = font.render(f"Step: {step_counter} | Box X: {box_x:.2f}m", True, (255,255,255))
    screen.blit(text, (20, 20))

    pygame.display.flip()
    clock.tick(30) # 30 FPS per non far schizzare via la simulazione

    step_counter += 1
    if terminated or truncated:
        obs, info = env.reset()
        step_counter = 0
        pygame.time.wait(1000) # Pausa di 1 secondo prima del prossimo episodio

pygame.quit()
sys.exit()