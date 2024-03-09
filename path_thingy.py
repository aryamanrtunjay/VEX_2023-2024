import pygame

pygame.init()

WIDTH, HEIGHT = 800, 600

screen = pygame.display.set_mode((WIDTH, HEIGHT))
done = False

img = pygame.image.load("../VEX_2023-2024/field.jpeg")
img = pygame.transform.scale(img, (600, 600))

screen.blit(img, (0,0))


class Bot:
    def __init__(self, x, y, width, height, heading):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.heading = heading
    

    def draw(self, screen):
        pygame.draw.rect(screen, (s))



while not done:
    for event in pygame.event.get():
        if(event.type == pygame.QUIT):
            done = True

    pygame.display.update()

pygame.quit()
quit()