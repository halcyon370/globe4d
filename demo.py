import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
import pygame.locals
from globe import Globe4D, Player4D, Scene4D, Mesh4D
# np.random.seed(1001)


def convert_to_gl_coord(verts):
    x_ = verts[:, 1]
    y_ = verts[:, 0]
    z_ = -verts[:, 2]
    return list(np.stack([x_, y_, z_], -1).tolist())


class MyCube(Mesh4D):
    def __init__(self, color, size=1):
        super(MyCube, self).__init__()
        cube_vertices = np.array([
            (0.5, -0.5, -0.5),
            (0.5, 0.5, -0.5),
            (-0.5, 0.5, -0.5),
            (-0.5, -0.5, -0.5),
            (0.5, -0.5, 0.5),
            (0.5, 0.5, 0.5),
            (-0.5, 0.5, 0.5),
            (-0.5, -0.5, 0.5),
        ]) * size
        cube_vertices[:, 2] += 6.0
        cube_vertices[:, 0] -= 2.0
        self.vertices3d = cube_vertices
        self.surfaces = (
            (3,2,1,0),
            (7,6,2,3),
            (4,5,6,7),
            (0,1,5,4),
            (2,6,5,1),
            (7,3,0,4)
        )
        self.colors = np.array([
            (1., 1., 1.), (1., 1., 1.), (1., 1., 1.), (0.6, 0.6, 0.6),
            (1., 1., 1.), (0.6, 0.6, 0.6), (1., 1., 1.), (1., 1., 1.)
        ]) * color
        self.colors = self.colors.tolist()


main_globe = Globe4D("cubes in 4D space", radius=50)
main_scene = Scene4D(main_globe)

N = 70
# 随机散乱放置
random_locs = np.random.random(size=[N, 4]) * 2 - 1
random_locs /= np.linalg.norm(random_locs, axis=1).reshape(-1,1)
random_colors = np.random.random(size=[N, 3])
random_sizes = np.random.choice([1.,1.5,2.], size=N)
for loc_, color_, size_ in zip(random_locs, random_colors, random_sizes):
    cube = MyCube(color_, size_)
    main_scene.add_vertices(cube, loc=loc_)

# 均匀放置一圈
N = 31
random_locs = []
for angle in np.arange(N) / N * np.pi * 2:
    random_locs.append([0., 0., np.sin(angle), np.cos(angle)])
random_colors = np.random.random(size=[N, 3])
random_sizes = np.random.choice([1.,1.5,2.], size=N)
for loc_, color_, size_ in zip(random_locs, random_colors, random_sizes):
    cube = MyCube(color_, size_)
    main_scene.add_vertices(cube, loc=loc_)


pygame.init()
font = pygame.font.SysFont('arial', 16)


def view_update(p: Player4D):
    log_text = "Loc: {:+5.4f} {:+5.4f} {:+5.4f} {:+5.4f}".format(*p.loc)
    textSurface = font.render(log_text, True, (255, 0, 0, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)

    # 颜色缓冲和深度缓冲
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glBegin(GL_QUADS)

    for mesh, verts in main_scene.view_vertices(p):
        vertices = convert_to_gl_coord(verts)
        for surface in mesh.surfaces:
            for vertex in surface:
                glColor3fv(mesh.colors[vertex])
                glVertex3fv(vertices[vertex])

    glEnd()

    glWindowPos2d(50, 750)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)
    pygame.display.flip()
    pygame.time.wait(5)


def main():
    display = (800, 800)
    # 双缓冲
    screen = pygame.display.set_mode(display, pygame.locals.DOUBLEBUF | pygame.locals.OPENGL)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_CULL_FACE)
    glCullFace(GL_BACK)
    # 透视参数
    glLoadIdentity()
    gluPerspective(45, 1, 0.01, 1000)

    # 初始化
    player = Player4D(main_globe, view_update, pose_theta=np.pi / 90, pace_speed=2)
    a_pressed = False
    d_pressed = False
    w_pressed = False
    s_pressed = False
    up_pressed = False
    down_pressed = False

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_a:
                    a_pressed = True
                if event.key == pygame.K_d:
                    d_pressed = True
                if event.key == pygame.K_w:
                    w_pressed = True
                if event.key == pygame.K_s:
                    s_pressed = True
                if event.key == pygame.K_UP:
                    up_pressed = True
                if event.key == pygame.K_DOWN:
                    down_pressed = True
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_a:
                    a_pressed = False
                if event.key == pygame.K_d:
                    d_pressed = False
                if event.key == pygame.K_w:
                    w_pressed = False
                if event.key == pygame.K_s:
                    s_pressed = False
                if event.key == pygame.K_UP:
                    up_pressed = False
                if event.key == pygame.K_DOWN:
                    down_pressed = False

        # 位置和视角
        if up_pressed and not down_pressed:
            player.move_front()
        elif down_pressed and not up_pressed:
            player.move_back()
        if a_pressed and not d_pressed:
            player.turn_left()
        elif d_pressed and not a_pressed:
            player.turn_right()
        if w_pressed and not s_pressed:
            player.turn_up()
        elif s_pressed and not w_pressed:
            player.turn_down()


if __name__ == '__main__':
    main()
