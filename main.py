import numpy as np
import pygame
import cv2
from PIL import Image

width = 300
height = 300


black = (0,0,0)
white = (255,255,255)
red = (255,0,0)
shade = (40,40,40)
green = pygame.color.Color(0, 255, 100)
path = (230,230,250)
blue = (0, 0, 255)

pygame.init()


screen= pygame.display.set_mode((width,height))


def to_pygame(coords, height):
    """Convert coordinates into pygame coordinates (lower-left => top left)."""
    return (coords[0], height - coords[1])

def get_collision_point(shape,rc):
    
    for point in shape:
        point_global_coor = (point[0]*ex + point[1]*ey) + rc # in global coordinates
    
        if point_global_coor[1]<0:
            return point_global_coor
    return []

def to_global_coordinate(point,rc):
    return point[0]*ex + point[1]*ey + rc # in global coordinates

def get_moment_of_inertia(name):
    if name=='rect':
        return 80*20/12*(80**2+20**2)



g = np.array([0,-10,0])
dt = 1e-2

m = 1600
I = get_moment_of_inertia('rect') # Moment Of Inertia


def update_rc(rc_init,vc_init,t):
    return rc_init + vc_init*t + g*t**2/2

def get_vc(vc_init,t):
    return vc_init + t*g

def get_vc_after_collision(point,rc,vc,m,I,omega,eps):
    n = np.array([0,1,0])
    R = point - rc
    a = np.sum(R**2)-np.sum(n*R)**2
    v = vc + np.cross(omega,R)
    omega_x_r = np.cross(omega,R)
    k = m/I
    A = -eps*np.dot(n,v) + k*np.dot(n,vc)*a-np.dot(n,omega_x_r)
    B = 1 + k*a
    vc_perp = A/B
    return vc_perp

def get_omega_after_collision(point,rc,vc_old,vc_new,m,I,omega_old):
    R = point - rc
    return m/I*np.cross(R,vc_new - vc_old) + omega_old
running = True

# shape is a list of points that define the object's boundary, in local CS
shape = [ np.array([-40,10]), np.array([40,10]), np.array([40,-10]), np.array([-40,-10])]

def get_ex_ey(alpha):
    return [ np.array([np.cos(alpha), np.sin(alpha),0]), np.array([-np.sin(alpha), np.cos(alpha),0]) ]

# initial parameters
rc_init = np.array([150,250,0])
t = 0
alpha = 0.09
omega = np.array([0,0,-0.0])
vc_init = np.array([0,0,0])
frame=0
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    screen.fill(shade)
    rc = update_rc(rc_init, vc_init,t)
    alpha += omega[2]*dt
    ex,ey = get_ex_ey(alpha)

    t += dt
    collision_point = get_collision_point(shape,rc)
    vc = get_vc(vc_init,t)
    if len(collision_point) and np.dot(np.array([0,1,0]),vc + np.cross(omega,collision_point-rc))<=0:
        n = np.array([0,1,0])
        vertical_shift = n*np.dot(n,collision_point)
        rc = rc - vertical_shift 
        collision_point = collision_point - vertical_shift 
        vc = get_vc(vc_init,t)
        vc_y = get_vc_after_collision(collision_point,rc,vc,m,I,omega,0.9)
        vc_new = np.array([0,vc_y,0])
        omega = get_omega_after_collision(collision_point,rc,vc,vc_new,m,I,omega)
        vc_init = vc_new
        vc = vc_new
        rc_init = rc
        t = 0
        

    # pygame.draw.aaline(screen, red, to_pygame(r_a,height), to_pygame(r_b,height), 500)
    points_gc = [to_global_coordinate(point,rc) for point in shape]
    pygame.draw.polygon(screen, green, [to_pygame(point_gc,height) for point_gc in points_gc], 0)
    
    pygame.display.update()
    
    pygame.image.save(screen, './movie/'+'frame_'+str(frame)+'.png')
    frame+=1


