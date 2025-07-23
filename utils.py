import pygame
import random
import time
import string



def scale_image(img, factor):
    size = round(img.get_width() * factor), round(img.get_height() * factor)
    return pygame.transform.scale(img, size)


def  blit_rotate_center(win, image, top_left, angle):
    rotated_image = pygame.transform.rotate(image, angle)
    new_rect = rotated_image.get_rect(
        center=image.get_rect(topleft=top_left).center)
    win.blit(rotated_image, new_rect.topleft)
    return new_rect.center

def manhattan_distance(x1,y1,x2,y2):
    return abs(x1-x2)+abs(y1-y2)

def id_generator(string_length=10):
    timestamp = time.time()
    random_text = ''.join(random.choices(string.ascii_letters + string.digits, k=string_length))

    return f"{timestamp}_{random_text}"



