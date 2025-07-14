from factor_graph import FactorGraph
import lietorch
from lietorch import SE3
import numpy as np
import torch

from cuda_timer import CudaTimer


ENABLE_TIMING = False

class DroidFrontend:

    def __init__(self, net, video, args):
        self.video = video
        self.update_op = net.update
        self.graph = FactorGraph(
            video, net.update, max_factors=48, upsample=args.upsample
        )

        # local optimization window
        self.t0 = 0
        self.t1 = 0

        # frontent variables
        self.is_initialized = False
        self.count = 0

<<<<<<< HEAD
        self.max_age = 20
        self.iters1 = 3
        self.iters2 = 2
=======
        self.max_age = 25
        self.iters1 = 8
        self.iters2 = 4
>>>>>>> fbcb27e (Patch files w/ MegaSam changes)

        self.keyframe_removal_index = 3

        self.warmup = args.warmup
        self.beta = args.beta
        self.frontend_nms = args.frontend_nms
        self.keyframe_thresh = args.keyframe_thresh
        self.frontend_window = args.frontend_window
        self.frontend_thresh = args.frontend_thresh
        self.frontend_radius = args.frontend_radius

<<<<<<< HEAD
        self.depth_window = 3

        self.motion_damping = 0.0
        if hasattr(args, "motion_damping"):
            self.motion_damping = args.motion_damping

    def _init_next_state(self):
        # set pose / depth for next iteration
        self.video.poses[self.t1] = self.video.poses[self.t1 - 1]

        self.video.disps[self.t1] = torch.quantile(
            self.video.disps[self.t1 - 3 : self.t1 - 1], 0.5
        )

        # damped linear velocity model
        if self.motion_damping >= 0:
            poses = SE3(self.video.poses)
            vel = (poses[self.t1 - 1] * poses[self.t1 - 2].inv()).log()
            damped_vel = self.motion_damping * vel
            next_pose = SE3.exp(damped_vel) * poses[self.t1 - 1]
            self.video.poses[self.t1] = next_pose.data

    def _update(self):
        """add edges, perform update"""

=======
    def __update(self):
        """add edges, perform update"""
>>>>>>> fbcb27e (Patch files w/ MegaSam changes)
        self.count += 1
        self.t1 += 1

        if self.graph.corr is not None:
            self.graph.rm_factors(self.graph.age > self.max_age, store=True)

        self.graph.add_proximity_factors(
            self.t1 - 5,
            max(self.t1 - self.frontend_window, 0),
            rad=self.frontend_radius,
            nms=self.frontend_nms,
            thresh=self.frontend_thresh,
            beta=self.beta,
            remove=True,
        )

        self.video.disps[self.t1 - 1] = torch.where(
            self.video.disps_sens[self.t1 - 1] > 0,
            self.video.disps_sens[self.t1 - 1],
            self.video.disps[self.t1 - 1],
        )

        for itr in range(self.iters1):
            self.graph.update(
                None, None, use_inactive=True, viz_itr=None, use_mono=True
            )

        # set initial pose for next frame
<<<<<<< HEAD
        d = self.video.distance(
            [self.t1 - 4], [self.t1 - 2], beta=self.beta, bidirectional=True
        )

        if d.item() < 2 * self.keyframe_thresh:
            self.graph.rm_keyframe(self.t1 - 3)

=======
        poses = SE3(self.video.poses)
        d = self.video.distance(
            [self.t1 - 3], [self.t1 - 2], beta=self.beta, bidirectional=True
        )

        if d.item() < self.keyframe_thresh:
            self.graph.rm_keyframe(self.t1 - 2)

>>>>>>> fbcb27e (Patch files w/ MegaSam changes)
            with self.video.get_lock():
                self.video.counter.value -= 1
                self.t1 -= 1

        else:
            for itr in range(self.iters2):
                self.graph.update(None, None, use_inactive=True)


        # set pose for next itration
        self.video.poses[self.t1] = self.video.poses[self.t1 - 1]
<<<<<<< HEAD
        self.video.disps[self.t1] = torch.quantile(
            self.video.disps[self.t1 - self.depth_window - 1 : self.t1 - 1], 0.7
        )
=======
        self.video.disps[self.t1] = self.video.disps[self.t1 - 1].mean()
>>>>>>> fbcb27e (Patch files w/ MegaSam changes)

        # update visualization
        self.video.dirty[self.graph.ii.min() : self.t1] = True

<<<<<<< HEAD
    def _initialize(self):
=======
    def __initialize(self):
>>>>>>> fbcb27e (Patch files w/ MegaSam changes)
        """initialize the SLAM system"""

        self.t0 = 0
        self.t1 = self.video.counter.value

        self.graph.add_neighborhood_factors(self.t0, self.t1, r=3)

        for itr in range(10):
            self.graph.update(1, use_inactive=True, use_mono=True, motion_only=True)

        self.graph.add_proximity_factors(
            0, 0, rad=2, nms=2, thresh=self.frontend_thresh, remove=False
        )

        for itr in range(10):
            self.graph.update(
                1, use_inactive=True, use_mono=True, motion_only=True, viz_itr=None
            )

<<<<<<< HEAD
=======
        for itr in range(10):
            self.graph.update(
                1, use_inactive=True, use_mono=True, motion_only=False, viz_itr=None
            )

        # print("error ", error)
        # breakpoint()

>>>>>>> fbcb27e (Patch files w/ MegaSam changes)
        # self.video.normalize()
        self.video.poses[self.t1] = self.video.poses[self.t1 - 1].clone()
        self.video.disps[self.t1] = self.video.disps[self.t1 - 4 : self.t1].mean()

        # initialization complete
        self.is_initialized = True
        self.last_pose = self.video.poses[self.t1 - 1].clone()
        self.last_disp = self.video.disps[self.t1 - 1].clone()
        self.last_time = self.video.tstamp[self.t1 - 1].clone()

        with self.video.get_lock():
            self.video.ready.value = 1
            self.video.dirty[: self.t1] = True

        self.graph.rm_factors(self.graph.ii < self.warmup - 4, store=True)

<<<<<<< HEAD
    def __call__(self):
        """main update"""

        # do initialization
        if not self.is_initialized and self.video.counter.value == self.warmup:
            self._initialize()
            self._init_next_state()

        # do update
        elif self.is_initialized and self.t1 < self.video.counter.value:
            self._update()
            self._init_next_state()
=======
    def __call__(self, final_=False):
        """main update"""
        if not self.is_initialized and final_ == True:
            self.__initialize()

        # do initialization
        if not self.is_initialized and self.video.counter.value == self.warmup:
            self.__initialize()

        # do update
        elif self.is_initialized and self.t1 < self.video.counter.value:
            self.__update()
>>>>>>> fbcb27e (Patch files w/ MegaSam changes)
