import pygame
import sys
from palletizer_env import PalletizerEnv
from stable_baselines3 import PPO

# --- PYGAME CONFIGURATION ---
WIDTH, HEIGHT = 800, 600
SCALE = 200  # 1 meter = 200 pixels
OFFSET_X, OFFSET_Y = 400, 500  # C++ world origin (0,0) on screen
SHOULDER_PIVOT = (0.0, 1.0) # Shoulder height

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Smart Palletizer - 2D Radar")
clock = pygame.time.Clock()

def world_to_screen(x, y):
    """Converts C++ meters coordinates to screen pixels"""
    screen_x = int(OFFSET_X - (x * SCALE))  # Invert X to slide from right to left
    screen_y = int(OFFSET_Y - (y * SCALE))  # Invert Y because Pygame 0 is at the top
    return screen_x, screen_y

# --- ENVIRONMENT LOADING ---
env = PalletizerEnv()
try:
    # Added env=env to fix SB3 batching issue, and updated to v9
    model = PPO.load("py_scripts/cervello_braccio_v9_dynamic", env=env)
except Exception as e:
    print(f"Model not found or loading error: {e}")
    print("Starting simulation with random actions to test graphics.")
    model = None

obs, info = env.reset()

running = True
step_counter = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 1. AI LOGIC
    if model:
        action, _ = model.predict(obs, deterministic=True)
    else:
        action = env.action_space.sample()

    obs, reward, terminated, truncated, info = env.step(action)
    
    # Extraction with the new 12 indices (includes elbow coordinates)
    elbow_x = obs[4]
    elbow_y = obs[5]
    ee_x = obs[6]
    ee_y = obs[7]
    box_x = obs[8]
    box_y = obs[9]
    is_holding = bool(obs[11])

    # 2. GRAPHICAL RENDER
    screen.fill((30, 30, 30)) # Dark gray background

    # Draw Conveyor Belt (Blue line) at 0.6m height
    belt_start = world_to_screen(2.5, 0.6)
    belt_end = world_to_screen(-1.0, 0.6)
    pygame.draw.line(screen, (50, 150, 250), belt_start, belt_end, 5)

    # Draw Package (Red/Green rectangle if grasped)
    box_screen_x, box_screen_y = world_to_screen(box_x, box_y)
    box_color = (50, 250, 50) if is_holding else (250, 50, 50)
    box_rect = pygame.Rect(box_screen_x - 15, box_screen_y - 15, 30, 30) # 30x30 pixels
    pygame.draw.rect(screen, box_color, box_rect)

    # Draw Articulated Robotic Arm
    shoulder_pos = world_to_screen(*SHOULDER_PIVOT)
    elbow_pos = world_to_screen(elbow_x, elbow_y)
    ee_pos = world_to_screen(ee_x, ee_y)
    
    # Bicep (Shoulder -> Elbow)
    pygame.draw.line(screen, (200, 200, 200), shoulder_pos, elbow_pos, 6)
    # Forearm (Elbow -> Vacuum)
    pygame.draw.line(screen, (150, 150, 150), elbow_pos, ee_pos, 4)
    # Vacuum
    pygame.draw.circle(screen, (255, 255, 0), ee_pos, 8)

    # On-Screen Text
    font = pygame.font.SysFont(None, 24)
    text = font.render(f"Step: {step_counter} | Box X: {box_x:.2f}m", True, (255,255,255))
    screen.blit(text, (20, 20))

    pygame.display.flip()
    clock.tick(30) # 30 FPS

    step_counter += 1
    if terminated or truncated:
        obs, info = env.reset()
        step_counter = 0
        pygame.time.wait(1000)

pygame.quit()
sys.exit()