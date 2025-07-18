from collections import OrderedDict

import cv2
from droid_net import DroidNet
import geom.projective_ops as pops
import lietorch
from modules.corr import CorrBlock
import torch

from functools import partial

if torch.__version__.startswith("2"):
    autocast = partial(torch.autocast, device_type="cuda")
else:
    autocast = torch.cuda.amp.autocast


class MotionFilter:
    """This class is used to filter incoming frames and extract features"""

    def __init__(self, net, video, thresh=2.5, device="cuda:0"):

        # split net modules
        self.cnet = net.cnet
        self.fnet = net.fnet
        self.update = net.update

        self.video = video
        self.thresh = thresh
        self.device = device

        self.count = 0

        # mean, std for image normalization
        self.MEAN = torch.as_tensor([0.485, 0.456, 0.406], device=self.device)[
            :, None, None
        ]
        self.STDV = torch.as_tensor([0.229, 0.224, 0.225], device=self.device)[
            :, None, None
        ]

    @torch.cuda.amp.autocast(enabled=True)
    def __context_encoder(self, image):
        """context features"""
        net, inp = self.cnet(image).split([128, 128], dim=2)
        return net.tanh().squeeze(0), inp.relu().squeeze(0)

    @autocast(enabled=True)
    def __feature_encoder(self, image):
        """features for correlation volume"""
        return self.fnet(image).squeeze(0)

    @autocast(enabled=True)
    @torch.no_grad()
    def track(
        self,
        tstamp,
        image,
        depth=None,
        intrinsics=None,
        mask=None,
        last_frame=False,
    ):
        """main update operation - run on every frame in video"""
        Id = lietorch.SE3.Identity(
            1,
        ).data.squeeze()
        ht = image.shape[-2] // 8
        wd = image.shape[-1] // 8

        image = image.cuda()

        # normalize images
        inputs = image[None, :, [2, 1, 0]].to(self.device) / 255.0
        inputs = inputs.sub_(self.MEAN).div_(self.STDV)

        # extract features
        gmap = self.__feature_encoder(inputs)

        ### always add first frame to the depth video ###
        if self.video.counter.value == 0:
            net, inp = self.__context_encoder(inputs[:, [0]])
            self.net, self.inp, self.fmap = net, inp, gmap
            self.video.append(
                tstamp,
                image[0],
                Id,
                1.0,
                depth,
                intrinsics / 8.0,
                mask,
                gmap,
                net[0, 0],
                inp[0, 0],
            )

        ### only add new frame if there is enough motion ###
        else:
            # index correlation volume
            coords0 = pops.coords_grid(ht, wd, device=self.device)[None, None]
            corr = CorrBlock(self.fmap[None, [0]], gmap[None, [0]])(coords0)

            # approximate flow magnitude using 1 update iteration
            _, delta, weight = self.update(self.net[None], self.inp[None], corr)

            # check motion magnitue / add new frame to video
            if delta.norm(dim=-1).mean().item() > self.thresh or last_frame:
                print("ADDING KEYFRAMES %d" % tstamp)
                self.count = 0
                net, inp = self.__context_encoder(inputs[:, [0]])
                self.net, self.inp, self.fmap = net, inp, gmap
                self.video.append(
                    tstamp,
                    image[0],
                    None,
                    None,
                    depth,
                    intrinsics / 8.0,
                    mask,
                    gmap,
                    net[0],
                    inp[0],
                )

            else:
                self.count += 1
