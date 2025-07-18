import geom.projective_ops as pops
import lietorch
from lietorch import SE3
import matplotlib.pyplot as plt
from modules.corr import AltCorrBlock, CorrBlock
import numpy as np
import torch
from torch_scatter import scatter_mean


MASK_TYPE = 2  # 1: GT, 2: Our pred
DAMPING_FACTOR = 1.0

from cuda_timer import CudaTimer
from functools import partial

if torch.__version__.startswith("2"):
    autocast = partial(torch.autocast, device_type="cuda")
else:
    autocast = torch.cuda.amp.autocast


class FactorGraph:

    def __init__(
        self,
        video,
        update_op,
        device="cuda:0",
        corr_impl="volume",
        max_factors=-1,
        upsample=False,
    ):
        self.video = video
        self.update_op = update_op
        self.device = device
        self.max_factors = max_factors
        self.corr_impl = corr_impl
        self.upsample = upsample
        # operator at 1/8 resolution
        self.ht = ht = video.ht // 8
        self.wd = wd = video.wd // 8

        self.coords0 = pops.coords_grid(ht, wd, device=device)
        self.ii = torch.as_tensor([], dtype=torch.long, device=device)
        self.jj = torch.as_tensor([], dtype=torch.long, device=device)
        self.age = torch.as_tensor([], dtype=torch.long, device=device)

        self.corr, self.net, self.inp = None, None, None
        self.damping = 1e-6 * torch.ones_like(self.video.disps)

        self.target = torch.zeros([1, 0, ht, wd, 2], device=device, dtype=torch.float)
        self.weight = torch.zeros([1, 0, ht, wd, 2], device=device, dtype=torch.float)
        # self.motion_prob = torch.zeros([1, 0, ht, wd, 1], device=device, dtype=torch.float)
        self.motion_prob = 0.0 * torch.ones_like(self.video.disps)

        # inactive factors
        self.ii_inac = torch.as_tensor([], dtype=torch.long, device=device)
        self.jj_inac = torch.as_tensor([], dtype=torch.long, device=device)
        self.ii_bad = torch.as_tensor([], dtype=torch.long, device=device)
        self.jj_bad = torch.as_tensor([], dtype=torch.long, device=device)

        self.target_inac = torch.zeros(
            [1, 0, ht, wd, 2], device=device, dtype=torch.float
        )
        self.weight_inac = torch.zeros(
            [1, 0, ht, wd, 2], device=device, dtype=torch.float
        )

    def __filter_repeated_edges(self, ii, jj):
        """remove duplicate edges"""
        # # filter if (ii, jj) insersect with any active edge
        # if len(self.ii) > 0:
        #     mask = ((ii[:, None] == self.ii) & (jj[:, None] == self.jj)).any(dim=-1)
        #     ii = ii[~mask]
        #     jj = jj[~mask]

        # # filter if (ii, jj) intersect with any inactive edge
        # if len(self.ii_inac) > 0:
        #     mask = ((ii[:, None] == self.ii_inac) & (jj[:, None] == self.jj_inac)).any(
        #         dim=-1
        #     )
        #     ii = ii[~mask]
        #     jj = jj[~mask]

        keep = torch.zeros(ii.shape[0], dtype=torch.bool, device=ii.device)
        eset = set(
            [(i.item(), j.item()) for i, j in zip(self.ii, self.jj)]
            + [(i.item(), j.item()) for i, j in zip(self.ii_inac, self.jj_inac)]
        )
        for k, (i, j) in enumerate(zip(ii, jj)):
            keep[k] = (i.item(), j.item()) not in eset

        return ii[keep], jj[keep]

    def print_edges(self):
        ii = self.ii.cpu().numpy()
        jj = self.jj.cpu().numpy()

        ix = np.argsort(ii)
        ii = ii[ix]
        jj = jj[ix]

        w = torch.mean(self.weight, dim=[0, 2, 3, 4]).cpu().numpy()
        w = w[ix]
        for e in zip(ii, jj, w):
            print(e)
        print()

    def filter_edges(self):
        """remove bad edges"""
        conf = torch.mean(self.weight, dim=[0, 2, 3, 4])
        mask = (torch.abs(self.ii - self.jj) > 2) & (conf < 0.001)

        self.ii_bad = torch.cat([self.ii_bad, self.ii[mask]])
        self.jj_bad = torch.cat([self.jj_bad, self.jj[mask]])
        self.rm_factors(mask, store=False)

    def clear_edges(self):
        self.rm_factors(self.ii >= 0)
        self.net = None
        self.inp = None

    @autocast(enabled=True)
    def add_factors(self, ii, jj, remove=False):
        """add edges to factor graph"""

        if not isinstance(ii, torch.Tensor):
            ii = torch.as_tensor(ii, dtype=torch.long, device=self.device)

        if not isinstance(jj, torch.Tensor):
            jj = torch.as_tensor(jj, dtype=torch.long, device=self.device)

        # remove duplicate edges
        ii, jj = self.__filter_repeated_edges(ii, jj)

        if ii.shape[0] == 0:
            return

        # place limit on number of factors
        if (
            self.max_factors > 0
            and self.ii.shape[0] + ii.shape[0] > self.max_factors
            and self.corr is not None
            and remove
        ):

            ix = torch.arange(len(self.age))[torch.argsort(self.age).cpu()]
            self.rm_factors(ix >= self.max_factors - ii.shape[0], store=True)

        net = self.video.nets[ii].to(self.device).unsqueeze(0)

        # correlation volume for new edges
        if self.corr_impl == "volume":
            c = (ii == jj).long()
            fmap1 = self.video.fmaps[ii, 0].to(self.device).unsqueeze(0)
            fmap2 = self.video.fmaps[jj, c].to(self.device).unsqueeze(0)
            corr = CorrBlock(fmap1, fmap2)
            self.corr = corr if self.corr is None else self.corr.cat(corr)

            inp = self.video.inps[ii].to(self.device).unsqueeze(0)
            self.inp = inp if self.inp is None else torch.cat([self.inp, inp], 1)

        with autocast(enabled=False):
            target, _ = self.video.reproject(ii, jj)
            weight = torch.zeros_like(target)

        self.ii = torch.cat([self.ii, ii], 0)
        self.jj = torch.cat([self.jj, jj], 0)
        self.age = torch.cat([self.age, torch.zeros_like(ii)], 0)

        # reprojection factors
        self.net = net if self.net is None else torch.cat([self.net, net], 1)

        self.target = torch.cat([self.target, target], 1)
        self.weight = torch.cat([self.weight, weight], 1)


    @autocast(enabled=True)
    def rm_factors(self, mask, store=False):
        """drop edges from factor graph"""

        # store estimated factors
        if store:
            self.ii_inac = torch.cat([self.ii_inac, self.ii[mask]], 0)
            self.jj_inac = torch.cat([self.jj_inac, self.jj[mask]], 0)
            self.target_inac = torch.cat([self.target_inac, self.target[:, mask]], 1)
            self.weight_inac = torch.cat([self.weight_inac, self.weight[:, mask]], 1)

        self.ii = self.ii[~mask]
        self.jj = self.jj[~mask]
        self.age = self.age[~mask]

        if self.corr_impl == "volume":
            self.corr = self.corr[~mask]

        if self.net is not None:
            self.net = self.net[:, ~mask]

        if self.inp is not None:
            self.inp = self.inp[:, ~mask]

        self.target = self.target[:, ~mask]
        self.weight = self.weight[:, ~mask]

    @autocast(enabled=True)
    def rm_keyframe(self, ix):
        # t = self.video.counter.value
        # with self.video.get_lock():
        # self.video.images[ix : t - 1] = self.video.images[ix + 1 : t].clone()
        # self.video.poses[ix : t - 1] = self.video.poses[ix + 1 : t].clone()
        # self.video.disps[ix : t - 1] = self.video.disps[ix + 1 : t].clone()
        # self.video.disps_sens[ix : t - 1] = self.video.disps_sens[ix + 1 : t].clone()
        # self.video.intrinsics[ix : t - 1] = self.video.intrinsics[ix + 1 : t].clone()

        # self.video.nets[ix : t - 1] = self.video.nets[ix + 1 : t].clone()
        # self.video.inps[ix : t - 1] = self.video.inps[ix + 1 : t].clone()
        # self.video.fmaps[ix : t - 1] = self.video.fmaps[ix + 1 : t].clone()
        # self.video.tstamp[ix: t - 1] = self.video.tstamp[ix + 1 : t].clone()
        """drop edges from factor graph"""

        with self.video.get_lock():
            self.video.images[ix] = self.video.images[ix + 1]
            self.video.poses[ix] = self.video.poses[ix + 1]
            self.video.disps[ix] = self.video.disps[ix + 1]
            self.video.disps_sens[ix] = self.video.disps_sens[ix + 1]
            self.video.intrinsics[ix] = self.video.intrinsics[ix + 1]
            self.video.motion_masks[ix] = self.video.motion_masks[ix + 1]
            self.video.motion_w[ix] = self.video.motion_w[ix + 1]

            self.video.nets[ix] = self.video.nets[ix + 1]
            self.video.inps[ix] = self.video.inps[ix + 1]
            self.video.fmaps[ix] = self.video.fmaps[ix + 1]
            self.video.tstamp[ix] = self.video.tstamp[ix + 1]

        m = (self.ii_inac == ix) | (self.jj_inac == ix)
        self.ii_inac[self.ii_inac >= ix] -= 1
        self.jj_inac[self.jj_inac >= ix] -= 1

        if torch.any(m):
            self.ii_inac = self.ii_inac[~m]
            self.jj_inac = self.jj_inac[~m]
            self.target_inac = self.target_inac[:, ~m]
            self.weight_inac = self.weight_inac[:, ~m]

        m = (self.ii == ix) | (self.jj == ix)

        self.ii[self.ii >= ix] -= 1
        self.jj[self.jj >= ix] -= 1
        self.rm_factors(m, store=False)

    @autocast(enabled=True)
    def update(
        self,
        t0=None,
        t1=None,
        itrs=2,
        use_inactive=False,
        EP=1e-7,
        motion_only=False,
        use_mono=True,
        viz_itr=None,
    ):
        """run update operator on factor graph"""

        # motion features
        with autocast(enabled=False):
            coords1, mask = self.video.reproject(self.ii, self.jj)
            motn = torch.cat([coords1 - self.coords0, self.target - coords1], dim=-1)
            motn = motn.permute(0, 1, 4, 2, 3).clamp(-64.0, 64.0)

        # correlation features
        corr = self.corr(coords1)

        self.net, delta, weight_, damping, upmask, mot_prob, refined_w = self.update_op(
            self.net,
            self.inp,
            corr,
            motn,
            self.ii,
            self.jj,
            self.video.disps[None],
            self.video.disps_sens[None],
        )
        # self.net, delta, weight_, damping, upmask, reg_prob, upmask_r, mot_prob, upmask_m = \

        if MASK_TYPE == 1:
            weight = (
                mot_prob
                * torch.index_select(self.video.motion_masks, dim=0, index=self.ii)[
                    None, ..., None
                ]
            )
            # weight = weight_ * torch.index_select(self.video.motion_masks, dim=0, index=self.ii)[None, ..., None]
        elif MASK_TYPE == 2:
            weight = mot_prob
            # self.video.motion_w[torch.unique(self.ii)] = mot_prob.float()
            # weight = weight * torch.index_select(self.video.motion_w, dim=0, index=self.ii)[None, ..., None]

        if viz_itr:
            # tstamp= self.video.tstamp
            print(self.ii[:5], self.jj[:5])
            plt.figure(figsize=(12, 7))
            plt.subplot(2, 3, 1)
            plt.imshow(
                np.linalg.norm(weight[0, 0, ...].cpu().numpy(), axis=-1), cmap="gray"
            )
            plt.title(
                "%02d-to-%02d"
                % (self.video.tstamp[self.ii[0]], self.video.tstamp[self.jj[0]])
            )
            plt.subplot(2, 3, 2)
            plt.imshow(
                np.linalg.norm(weight[0, 1, ...].cpu().numpy(), axis=-1), cmap="gray"
            )
            plt.title(
                "%02d-to-%02d"
                % (self.video.tstamp[self.ii[1]], self.video.tstamp[self.jj[1]])
            )
            plt.subplot(2, 3, 3)
            plt.imshow(
                np.linalg.norm(weight[0, 2, ...].cpu().numpy(), axis=-1), cmap="gray"
            )
            plt.title(
                "%02d-to-%02d"
                % (self.video.tstamp[self.ii[2]], self.video.tstamp[self.jj[2]])
            )
            plt.subplot(2, 3, 4)
            plt.imshow(
                self.video.images[self.ii[0]]
                .cpu()
                .numpy()
                .transpose(1, 2, 0)[..., ::-1]
            )
            plt.title(
                "frame %02d iteration %02d" % (self.video.tstamp[self.ii[0]], viz_itr)
            )
            plt.subplot(2, 3, 5)
            plt.imshow(
                self.video.images[self.jj[0]]
                .cpu()
                .numpy()
                .transpose(1, 2, 0)[..., ::-1]
            )
            plt.title(
                "frame %02d iteration %02d" % (self.video.tstamp[self.jj[0]], viz_itr)
            )
            plt.subplot(2, 3, 6)
            plt.imshow(
                self.video.images[self.jj[1]]
                .cpu()
                .numpy()
                .transpose(1, 2, 0)[..., ::-1]
            )
            plt.title(
                "frame %02d iteration %02d" % (self.video.tstamp[self.jj[1]], viz_itr)
            )
            plt.tight_layout()
            plt.savefig("debug/weights_%02d.png" % viz_itr)
            plt.close()

        if t0 is None:
            t0 = max(1, self.ii.min().item() + 1)

        with autocast(enabled=False):
            self.target = coords1 + delta.to(dtype=torch.float)
            self.weight = weight.to(dtype=torch.float)

            ht, wd = self.coords0.shape[0:2]
            self.damping[torch.unique(self.ii)] = damping

            if use_inactive:
                m = (self.ii_inac >= t0 - 3) & (self.jj_inac >= t0 - 3)
                ii = torch.cat([self.ii_inac[m], self.ii], 0)
                jj = torch.cat([self.jj_inac[m], self.jj], 0)
                target = torch.cat([self.target_inac[:, m], self.target], 1)
                weight = torch.cat([self.weight_inac[:, m], self.weight], 1)
            else:
                ii, jj, target, weight = self.ii, self.jj, self.target, self.weight

            damping = DAMPING_FACTOR * self.damping[torch.unique(ii)].contiguous() + EP

            target = target.view(-1, ht, wd, 2).permute(0, 3, 1, 2).contiguous()
            weight = weight.view(-1, ht, wd, 2).permute(0, 3, 1, 2).contiguous()

            # dense bundle adjustment
            error = self.video.ba(
                target,
                weight,
                damping,
                ii,
                jj,
                t0,
                t1,
                itrs=itrs,
                lm=1e-4,
                ep=0.1,
                use_mono=use_mono,
                motion_only=motion_only,
            )

            if self.upsample:
                self.video.upsample(torch.unique(self.ii), upmask)

        self.age += 1
        return error

    @autocast(enabled=False)
    def estimate_preconditor(self):
        """run update operator on factor graph - reduced memory implementation"""
        # alternate corr implementation
        t = self.video.counter.value
        median_hessian = self.video.estimate_preconditor(
            self.ii, self.jj, t0=1, t1=t, itrs=1, lm=1e-5, ep=1e-2
        )

        return median_hessian
    
    @autocast(enabled=False)
    def update_lowmem(
        self,
        t0=None,
        t1=None,
        itrs=2,
        use_mono=False,
        pytorch_ba=False,
        use_inactive=False,
        EP=1e-7,
        steps=8,
        opt_intr=True,
        alpha=0.005,
        ret_mask=False,
    ):
        """run update operator on factor graph - reduced memory implementation"""

        # alternate corr implementation
        t = self.video.counter.value

        num, rig, ch, ht, wd = self.video.fmaps.shape
        corr_op = AltCorrBlock(self.video.fmaps.view(1, num * rig, ch, ht, wd))

        errors = []

        for step in range(steps):
            # print("Global BA Iteration #{}".format(step+1))
            with CudaTimer("backend", enabled=False):
                with autocast(enabled=False):
                    coords1, mask = self.video.reproject(self.ii, self.jj)
                    motn = torch.cat([coords1 - self.coords0, self.target - coords1], dim=-1)
                    motn = motn.permute(0,1,4,2,3).clamp(-64.0, 64.0)

                s = 8
                for i in range(self.ii.min(), self.jj.max()+1, s):
                    v = (self.ii >= i) & (self.ii < i + s)
                    iis = self.ii[v]
                    jjs = self.jj[v]

                    if v.count_nonzero().item() == 0:
                        continue

                    ht, wd = self.coords0.shape[0:2]

                    with autocast(enabled=True):
                        corr1 = corr_op(coords1[:,v], rig * iis, rig * jjs + (iis == jjs).long())

                        net, delta, _weight, damping, upmask,  mot_prob, refined_w = (
                            self.update_op(
                                self.net[:,v], 
                                self.video.inps[None,iis], 
                                corr1, 
                                motn[:,v], 
                                iis, 
                                jjs, 
                                self.video.disps[None], 
                                self.video.disps_sens[None])
                        ) 

                        if self.upsample:
                            self.video.upsample(torch.unique(iis), upmask)
                        
                        if MASK_TYPE == 1:
                            weight = (
                                mot_prob
                                * torch.index_select(self.video.motion_masks, dim=0, index=iis)[
                                    None, ..., None
                                ]
                            )
                        # weight = weight_ * torch.index_select(self.video.motion_masks, dim=0, index=iis)[None, ..., None]
                        elif MASK_TYPE == 2:
                            weight = mot_prob
                    refined_w_scatter = scatter_mean(
                        refined_w, iis - torch.min(iis), dim=0
                    ).squeeze(1)

                    self.net[:,v] = net
                    self.target[:,v] = coords1[:,v] + delta.float()
                    self.weight[:,v] = weight.float()
                    self.damping[torch.unique(iis)] = damping
                    self.motion_prob[torch.unique(iis)] = refined_w_scatter.float()

                damping = .2 * self.damping[torch.unique(self.ii)].contiguous() + EP
                motion_prob = (
                    self.motion_prob[torch.unique(self.ii)].contiguous()
                    * self.video.motion_masks[torch.unique(self.ii)].contiguous()
                )

                target = target.view(-1, ht, wd, 2).permute(0,3,1,2).contiguous()
                weight = weight.view(-1, ht, wd, 2).permute(0,3,1,2).contiguous()
                
                self.age += 1

                # dense bundle adjustment
                error = self.video.ba(
                    target, 
                    weight, 
                    damping, 
                    self.ii, 
                    self.jj, 
                    1, 
                    t, 
                    itrs=itrs, 
                    lm=1e-5, 
                    ep=1e-2, 
                    use_mono=use_mono, 
                    pytorch_ba=pytorch_ba, 
                    motion_only=False, 
                    opt_intr=opt_intr, 
                    alpha=alpha
                )

                self.video.dirty[:t] = True

            if step == steps - 1:
                errors.append(error)

        return motion_prob

    def add_neighborhood_factors(self, t0, t1, r=3):
        """add edges between neighboring frames within radius r"""

        # ii, jj = torch.meshgrid(torch.arange(t0, t1), torch.arange(t0, t1))
        # ii = ii.reshape(-1).to(dtype=torch.long, device=self.device)
        # jj = jj.reshape(-1).to(dtype=torch.long, device=self.device)

        ii, jj = torch.meshgrid(
            torch.arange(t0, t1, device=self.device),
            torch.arange(t0, t1, device=self.device),
            indexing="ij",
        )

        c = 1 if self.video.stereo else 0

        keep = ((ii - jj).abs() > c) & ((ii - jj).abs() <= r)
        self.add_factors(ii[keep], jj[keep])

    def add_proximity_factors(
        self, t0=0, t1=0, rad=2, nms=2, beta=0.25, thresh=16.0, remove=False
    ):
        """add edges to the factor graph based on distance"""

        t = self.video.counter.value
        ix = torch.arange(t0, t)
        jx = torch.arange(t1, t)

        ii, jj = torch.meshgrid(ix, jx, indexing="ij")
        ii = ii.reshape(-1)
        jj = jj.reshape(-1)

        d = self.video.distance(ii, jj, beta=beta).cpu()
        d[ii - rad < jj] = np.inf
        d[d > 100] = np.inf

        ii1 = torch.cat([self.ii, self.ii_bad, self.ii_inac], 0)
        jj1 = torch.cat([self.jj, self.jj_bad, self.jj_inac], 0)
        for i, j in zip(ii1.cpu().numpy(), jj1.cpu().numpy()):
            for di in range(-nms, nms + 1):
                for dj in range(-nms, nms + 1):
                    if abs(di) + abs(dj) <= max(min(abs(i - j) - 2, nms), 0):
                        i1 = i + di
                        j1 = j + dj
                        if (t0 <= i1 < t) and (t1 <= j1 < t):
                            d[(i1 - t0) * (t - t1) + (j1 - t1)] = np.inf

        es = []
        for i in range(t0, t):
            if self.video.stereo:
                es.append((i, i))
                d[(i - t0) * (t - t1) + (i - t1)] = np.inf

            for j in range(max(i - rad - 1, 0), i):
                es.append((i, j))
                es.append((j, i))
                d[(i - t0) * (t - t1) + (j - t1)] = np.inf

        ix = torch.argsort(d)
        for k in ix:
            if d[k] > thresh:
                continue

            if self.max_factors > 0:
                if len(es) > self.max_factors:
                    break

            i = ii[k]
            j = jj[k]

            # bidirectional
            es.append((i, j))
            es.append((j, i))

            for di in range(-nms, nms + 1):
                for dj in range(-nms, nms + 1):
                    if abs(di) + abs(dj) <= max(min(abs(i - j) - 2, nms), 0):
                        i1 = i + di
                        j1 = j + dj

                        if (t0 <= i1 < t) and (t1 <= j1 < t):
                            d[(i1 - t0) * (t - t1) + (j1 - t1)] = np.inf

        ii, jj = torch.as_tensor(es, device=self.device).unbind(dim=-1)
        self.add_factors(ii, jj, remove)
