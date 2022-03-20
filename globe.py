import numpy as np

np.set_printoptions(suppress=True)


class Globe4D(object):
    def __init__(self, name=None, radius=1000):
        self.name = name
        self.radius = radius


class Player4D(object):
    def __init__(self, parent: Globe4D = None, view_update=lambda x: None,
                 pose_theta=np.pi / 90, pace_speed=2):
        self.parent = parent
        self.pose_theta = pose_theta
        self.pace_theta = pace_speed / self.parent.radius
        self.loc = np.array([0., 0., 0., 1.])
        self.front = np.array([0., 0., 1., 0.])
        self.direction_r = np.array([0., 1., 0., 0.])
        self.direction_u = np.array([1., 0., 0., 0.])
        self.view_update = view_update
        self.view_update(self)

    def get_matrix(self):
        return np.stack([self.direction_u, self.direction_r, self.front, self.loc], 1)

    def _step_front(self, theta):
        M_ = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        result_ = M_.dot(np.stack([self.front, self.loc]))
        self.front = result_[0]
        self.loc = result_[1]
        self.view_update(self)

    def _step_direction(self, theta, axis):
        M_ = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]
        ])
        if axis == "right":
            result_ = M_.dot(np.stack([self.front, self.direction_r]))
            self.front = result_[0]
            self.direction_r = result_[1]
        elif axis == "up":
            result_ = M_.dot(np.stack([self.front, self.direction_u]))
            self.front = result_[0]
            self.direction_u = result_[1]
        else:
            raise Exception("axis must be right or up")
        self.view_update(self)

    def move_front(self):
        self._step_front(self.pace_theta)

    def move_back(self):
        self._step_front(-self.pace_theta)

    def turn_left(self):
        self._step_direction(-self.pose_theta, "right")

    def turn_right(self):
        self._step_direction(self.pose_theta, "right")

    def turn_up(self):
        self._step_direction(self.pose_theta, "up")

    def turn_down(self):
        self._step_direction(-self.pose_theta, "up")


def convert3dto4d(xyz: np.ndarray, M: np.ndarray):
    xyz = xyz.reshape(-1, 3)
    xyzp = np.concatenate([xyz, np.ones([xyz.shape[0], 1])], 1)
    xyzp = xyzp / np.linalg.norm(xyzp, axis=1).reshape(-1, 1)
    xyzp[:, 3] -= 1
    xyzw = xyzp.dot(M.T) + M[:, 3]
    return xyzw


def convert4dto3d(xyzw: np.ndarray, M: np.ndarray, eps=1e-7):
    xyzw = xyzw.reshape(-1, 4)
    M_inv = M.T
    xyz = (xyzw - M[:, 3]).dot(M_inv.T)
    ratio = np.maximum(eps, xyz[:, 3]+1).reshape(-1, 1)
    return xyz[:, :3] / ratio


class Mesh4D(object):
    def __init__(self):
        self.vertices3d = ()
        self.vertices4d = None
        self.surfaces = ()
        self.colors = ()


class Scene4D(object):
    def __init__(self, parent: Globe4D):
        self.parent = parent
        self.groups = []

    def add_vertices(self, mesh: Mesh4D, loc=(0,0,0,1)):
        loc_ = loc / np.linalg.norm(loc)
        if loc_[3] == -1:
            print("Loc (0,0,0,-1) is not permitted, skip")
            return
        M_ = get_rotation_4d(np.array([0,0,0,1]), loc_)
        mesh.vertices4d = convert3dto4d(np.asarray(mesh.vertices3d) / self.parent.radius, M_)
        self.groups.append(mesh)

    def view_vertices(self, player: Player4D):
        results = []
        for mesh in self.groups:
            result = convert4dto3d(mesh.vertices4d, player.get_matrix()) * self.parent.radius
            results.append((mesh, result))
        return results


def get_rotation_4d(src: np.ndarray, dst: np.ndarray, eps=1e-7):
    mxy = dst[0] * src[1] - src[0] * dst[1]
    mxz = dst[0] * src[2] - src[0] * dst[2]
    mxt = dst[0] * src[3] - src[0] * dst[3]
    myz = dst[1] * src[2] - src[1] * dst[2]
    myt = dst[1] * src[3] - src[1] * dst[3]
    mzt = dst[2] * src[3] - src[2] * dst[3]
    norm = np.linalg.norm([mxy, mxz, mxt, myz, myt, mzt])
    omega = np.array([
        [0, mxy, mxz, mxt],
        [-mxy, 0, myz, myt],
        [-mxz, -myz, 0, mzt],
        [-mxt, -myt, -mzt, 0],
    ]) / (norm + eps)
    theta = np.arccos(np.dot(src, dst))
    R4 = np.eye(4) + (1 - np.cos(theta)) * np.dot(omega, omega) + np.sin(theta) * omega
    return R4




if __name__ == '__main__':
    import cv2

    main_canvas = np.zeros([512, 512, 3], np.uint8)

    def update(p: Player4D):
        canvas = np.zeros([512, 512, 3], np.uint8)
        loc = p.loc * p.parent.radius
        cv2.putText(canvas, "Loc: {:+5.1f} {:+5.1f} {:+5.1f} {:+5.1f},".format(*loc), (4, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(canvas, "Face: {:+5.2f} {:+5.2f} {:+5.2f} {:+5.2f},".format(*p.front), (4, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        main_canvas[:] = canvas



