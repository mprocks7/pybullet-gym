from pybulletgym.envs.roboschool.envs.locomotion.walker_soccer_env import WalkerBaseBulletEnv
from pybulletgym.envs.roboschool.robots.locomotors.walker_soccer import WalkerSoccer
# from pybulletgym.envs.roboschool.robots.locomotors import Walker2D


class Walker2DSoccerBulletEnv(WalkerBaseBulletEnv):
    def __init__(self):
        self.robot = WalkerSoccer()
        # self.robot = Walker2D()
        WalkerBaseBulletEnv.__init__(self, self.robot)
