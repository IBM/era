#
#   Darknet RegionLoss
#   Copyright EAVISE
#

import logging
import math
import numpy as np
import torch
import torch.nn as nn
from distutils.version import LooseVersion

try:
    import pandas as pd
except ModuleNotFoundError:
    pd = None


__all__ = ['RegionLoss']
log = logging.getLogger(__name__)
torchversion = LooseVersion(torch.__version__)
version120 = LooseVersion("1.2.0")


class RegionLoss(nn.modules.loss._Loss):
    """ Computes region loss from darknet network output and target annotation (yoloV2).

    Args:
        num_classes (int): number of classes to detect
        anchors (list): 2D list representing anchor boxes (see :class:`lightnet.network.Darknet`)
        stride (optional, int): The downsampling factor of the network (input_dimension / output_dimension); Default **32**
        seen (optional, torch.Tensor): How many images the network has already been trained on; Default **0**
        coord_scale (optional, float): weight of bounding box coordinates; Default **1.0**
        noobject_scale (optional, float): weight of regions without target boxes; Default **1.0**
        object_scale (optional, float): weight of regions with target boxes; Default **5.0**
        class_scale (optional, float): weight of categorical predictions; Default **1.0**
        thresh (optional, float): minimum iou between a predicted box and ground truth for them to be considered matching; Default **0.6**
        coord_prefill (optional, int): This parameter controls for how many training samples the network will prefill the target coordinates, biassing the network to predict the center at **.5,.5**; Default **12800**
    """
    def __init__(self, num_classes, anchors, stride=32, seen=0, coord_scale=1.0, noobject_scale=1.0, object_scale=5.0, class_scale=1.0, thresh=0.6, coord_prefill=12800):
        super().__init__()
        self.num_classes = num_classes
        self.stride = stride
        self.num_anchors = len(anchors)
        self.anchor_step = len(anchors[0])
        self.anchors = torch.tensor(anchors, dtype=torch.float, requires_grad=False)
        self.register_buffer('seen', torch.tensor(seen))

        self.coord_scale = coord_scale
        self.noobject_scale = noobject_scale
        self.object_scale = object_scale
        self.class_scale = class_scale
        self.thresh = thresh
        self.coord_prefill = coord_prefill

        self.mse = nn.MSELoss(reduction='sum')
        self.cel = nn.CrossEntropyLoss(reduction='sum')

        self.loss_total = torch.tensor(0.0)
        self.loss_conf = torch.tensor(0.0)
        self.loss_coord = torch.tensor(0.0)
        self.loss_class = torch.tensor(0.0)

    @property
    def values(self):
        """ Return detached sub-losses in a dictionary.

        Note:
            You can access the individual loss values directly as ``object.loss_<name>`` as well. |br|
            This will return the actual loss tensor with its attached computational graph and gives you full freedom for modifying this loss prior to the backward pass.
        """
        return {
            'total': self.loss_total.detach(),
            'conf':  self.loss_conf.detach(),
            'coord': self.loss_coord.detach(),
            'class': self.loss_class.detach(),
        }

    @property
    def loss(self):
        log.deprecated('The "loss" attribute is deprecated in favor for "loss_total"')
        return self.loss_total

    def extra_repr(self):
        repr_str = f'classes={self.num_classes}, stride={self.stride}, threshold={self.thresh}, seen={self.seen.item()}\n'
        repr_str += f'coord_scale={self.coord_scale}, object_scale={self.object_scale}, noobject_scale={self.noobject_scale}, class_scale={self.class_scale}\n'
        repr_str += f'anchors='
        for a in self.anchors:
            repr_str += f'[{a[0]:.5g}, {a[1]:.5g}] '
        return repr_str

    def forward(self, output, target, seen=None):
        """ Compute Region loss.

        Args:
            output (torch.autograd.Variable): Output from the network
            target (brambox annotation dataframe or torch.Tensor): Brambox annotations or tensor containing the annotation targets (see :class:`lightnet.data.BramboxToTensor`)
            seen (int, optional): How many images the network has already been trained on; Default **Add batch_size to previous seen value**

        Note:
            If using a target tensor, it should have the dimensions `[num_batch, num_anno, 5]` and following format per image:

            .. math::

                \\begin{bmatrix}
                    class\\_idx & x\\_center & y\\_center & width & height \\\\
                    class\\_idx & x\\_center & y\\_center & width & height \\\\
                    ... \\\\
                    -1 & 0 & 0 & 0 & 0 \\\\
                    -1 & 0 & 0 & 0 & 0 \\\\
                    ...
                \\end{bmatrix}

            With all coordinates being relative to the image size. |br|
            Since the annotations from all images of a batch should be made of the same length, you can pad them with: `[-1, 0, 0, 0, 0]`.

        Note:
            Besides being easier to work with, brambox dataframes have the added benefit that
            this loss function will also consider the ``ignore`` flag of annotations and ignore detections that match with it.
            This allows you to have annotations that will not influence the loss in any way,
            as opposed to having them removed and counting them as false detections.
        """
        # Parameters
        nB = output.data.size(0)
        nA = self.num_anchors
        nC = self.num_classes
        nH = output.data.size(2)
        nW = output.data.size(3)
        nPixels = nH * nW
        device = output.device
        if seen is not None:
            self.seen = torch.tensor(seen)
        elif self.training:
            self.seen += nB

        # Get x,y,w,h,conf,cls
        output = output.view(nB, nA, -1, nPixels)
        coord = torch.zeros_like(output[:, :, :4])
        coord[:, :, :2] = output[:, :, :2].sigmoid()    # tx,ty
        coord[:, :, 2:4] = output[:, :, 2:4]            # tw,th
        conf = output[:, :, 4].sigmoid()
        if nC > 1:
            cls = output[:, :, 5:].contiguous().view(nB*nA, nC, nPixels).transpose(1, 2).contiguous().view(-1, nC)

        # Create prediction boxes
        pred_boxes = torch.FloatTensor(nB*nA*nPixels, 4)
        lin_x = torch.linspace(0, nW-1, nW).repeat(nH, 1).view(nPixels).to(device)
        lin_y = torch.linspace(0, nH-1, nH).view(nH, 1).repeat(1, nW).view(nPixels).to(device)
        anchor_w = self.anchors[:, 0].contiguous().view(nA, 1).to(device)
        anchor_h = self.anchors[:, 1].contiguous().view(nA, 1).to(device)

        pred_boxes[:, 0] = (coord[:, :, 0].detach() + lin_x).view(-1)
        pred_boxes[:, 1] = (coord[:, :, 1].detach() + lin_y).view(-1)
        pred_boxes[:, 2] = (coord[:, :, 2].detach().exp() * anchor_w).view(-1)
        pred_boxes[:, 3] = (coord[:, :, 3].detach().exp() * anchor_h).view(-1)
        pred_boxes = pred_boxes.cpu()

        # pred_boxes [x,y,w,h,conf,cls]
        # target []

        # Get target values
        coord_mask, conf_mask, cls_mask, tcoord, tconf, tcls = self.build_targets(pred_boxes, target, nB, nH, nW)
        coord_mask = coord_mask.expand_as(tcoord).to(device).sqrt()
        conf_mask = conf_mask.to(device).sqrt()
        tcoord = tcoord.to(device)
        tconf = tconf.to(device)
        if nC > 1:
            tcls = tcls[cls_mask].view(-1).long().to(device)
            cls_mask = cls_mask.view(-1, 1).repeat(1, nC).to(device)
            cls = cls[cls_mask].view(-1, nC)

        # Compute losses
        self.loss_coord = self.coord_scale * self.mse(coord*coord_mask, tcoord*coord_mask) / (2 * nB)
        self.loss_conf = self.mse(conf*conf_mask, tconf*conf_mask) / (2 * nB)
        if nC > 1:
            if tcls.numel() > 0:
                self.loss_class = self.class_scale * self.cel(cls, tcls) / nB
            else:
                self.loss_class = torch.tensor(0.0, device=device)
        else:
            self.loss_class = torch.tensor(0.0, device=device)

        self.loss_total = self.loss_coord + self.loss_conf + self.loss_class
        return self.loss_total


    def build_targets(self, pred_boxes, ground_truth, nB, nH, nW):
        """ Compare prediction boxes and targets, convert targets to network output tensors """
        if torch.is_tensor(ground_truth):
            return self.__build_targets_tensor(pred_boxes, ground_truth, nB, nH, nW)
        elif pd is not None and isinstance(ground_truth, pd.DataFrame):
            return self.__build_targets_brambox(pred_boxes, ground_truth, nB, nH, nW)
        else:
            raise TypeError(f'Unkown ground truth format [{type(ground_truth)}]')

    def __build_targets_tensor(self, pred_boxes, ground_truth, nB, nH, nW):
        """ Compare prediction boxes and ground truths, convert ground truths to network output tensors """
        # Parameters
        nT = ground_truth.size(1)
        nA = self.num_anchors
        nAnchors = nA*nH*nW
        nPixels = nH*nW

        # Tensors
        coord_mask = torch.zeros(nB, nA, nH, nW, requires_grad=False)
        conf_mask = torch.ones(nB, nA, nH, nW, requires_grad=False) * self.noobject_scale
        if torchversion >= version120:
            cls_mask = torch.zeros(nB, nA, nH, nW, dtype=torch.bool, requires_grad=False)
        else:
            cls_mask = torch.zeros(nB, nA, nH, nW, requires_grad=False).byte()
        tcoord = torch.zeros(nB, nA, 4, nH, nW, requires_grad=False)
        tconf = torch.zeros(nB, nA, nH, nW, requires_grad=False)
        tcls = torch.zeros(nB, nA, nH, nW, requires_grad=False)

        if self.training and self.seen < self.coord_prefill:
            coord_mask.fill_(math.sqrt(.01 / self.coord_scale))
            if self.anchor_step == 4:
                tcoord[:, :, 0] = self.anchors[:, 2].contiguous().view(1, nA, 1, 1).repeat(nB, 1, 1, nPixels)
                tcoord[:, :, 1] = self.anchors[:, 3].contiguous().view(1, nA, 1, 1).repeat(nB, 1, 1, nPixels)
            else:
                tcoord[:, :, 0].fill_(0.5)
                tcoord[:, :, 1].fill_(0.5)

        # Anchors
        if self.anchor_step == 4:
            anchors = self.anchors.clone()
            anchors[:, :2] = 0
        else:
            anchors = torch.cat([torch.zeros_like(self.anchors), self.anchors], 1)

        # Loop over GT
        for b in range(nB):
            gt = ground_truth[b][(ground_truth[b, :, 0] >= 0)[:, None].expand_as(ground_truth[b])].view(-1, 5)
            if gt.numel() == 0:     # No gt for this image
                continue

            # Build up tensors
            cur_pred_boxes = pred_boxes[b*nAnchors:(b+1)*nAnchors]
            gt = gt[:, 1:]
            gt[:, ::2] *= nW
            gt[:, 1::2] *= nH

            # Set confidence mask of matching detections to 0
            iou_gt_pred = bbox_ious(gt, cur_pred_boxes)
            mask = (iou_gt_pred > self.thresh).sum(0) >= 1
            conf_mask[b][mask.view_as(conf_mask[b])] = 0

            # Find best anchor for each gt
            iou_gt_anchors = bbox_wh_ious(gt, anchors)
            _, best_anchors = iou_gt_anchors.max(1)

            # Set masks and target values for each gt
            nGT = gt.shape[0]
            gi = gt[:, 0].clamp(0, nW-1).long()
            gj = gt[:, 1].clamp(0, nH-1).long()

            conf_mask[b, best_anchors, gj, gi] = self.object_scale
            tconf[b, best_anchors, gj, gi] = iou_gt_pred.view(nGT, nA, nH, nW)[torch.arange(nGT), best_anchors, gj, gi]
            coord_mask[b, best_anchors, gj, gi] = 2 - (gt[:, 2] * gt[:, 3]) / nPixels
            tcoord[b, best_anchors, 0, gj, gi] = gt[:, 0] - gi.float()
            tcoord[b, best_anchors, 1, gj, gi] = gt[:, 1] - gj.float()
            tcoord[b, best_anchors, 2, gj, gi] = (gt[:, 2] / self.anchors[best_anchors, 0]).log()
            tcoord[b, best_anchors, 3, gj, gi] = (gt[:, 3] / self.anchors[best_anchors, 1]).log()
            cls_mask[b, best_anchors, gj, gi] = 1
            tcls[b, best_anchors, gj, gi] = ground_truth[b, torch.arange(nGT), 0]

        return (
            coord_mask.view(nB, nA, 1, nPixels),
            conf_mask.view(nB, nA, nPixels),
            cls_mask.view(nB, nA, nPixels),
            tcoord.view(nB, nA, 4, nPixels),
            tconf.view(nB, nA, nPixels),
            tcls.view(nB, nA, nPixels)
        )

    def __build_targets_brambox(self, pred_boxes, ground_truth, nB, nH, nW):
        """ Compare prediction boxes and ground truths, convert ground truths to network output tensors """
        # Parameters
        nA = self.num_anchors
        nAnchors = nA*nH*nW
        nPixels = nH*nW

        # Tensors
        coord_mask = torch.zeros(nB, nA, nH, nW, requires_grad=False)
        conf_mask = torch.ones(nB, nA, nH, nW, requires_grad=False) * self.noobject_scale
        if torchversion >= version120:
            cls_mask = torch.zeros(nB, nA, nH, nW, dtype=torch.bool, requires_grad=False)
        else:
            cls_mask = torch.zeros(nB, nA, nH, nW, requires_grad=False).byte()
        tcoord = torch.zeros(nB, nA, 4, nH, nW, requires_grad=False)
        tconf = torch.zeros(nB, nA, nH, nW, requires_grad=False)
        tcls = torch.zeros(nB, nA, nH, nW, requires_grad=False)

        if self.training and self.seen < self.coord_prefill:
            coord_mask.fill_(math.sqrt(.01 / self.coord_scale))
            if self.anchor_step == 4:
                tcoord[:, :, 0] = self.anchors[:, 2].contiguous().view(1, nA, 1, 1).repeat(nB, 1, 1, nPixels)
                tcoord[:, :, 1] = self.anchors[:, 3].contiguous().view(1, nA, 1, 1).repeat(nB, 1, 1, nPixels)
            else:
                tcoord[:, :, 0].fill_(0.5)
                tcoord[:, :, 1].fill_(0.5)

        # Anchors
        if self.anchor_step == 4:
            anchors = self.anchors.clone()
            anchors[:, :2] = 0
        else:
            anchors = torch.cat([torch.zeros_like(self.anchors), self.anchors], 1)

        # Loop over GT
        for b, gt_filtered in ground_truth.groupby('batch_number', sort=False):
            cur_pred_boxes = pred_boxes[b*nAnchors:(b+1)*nAnchors]

            # Create ground_truth tensor
            gt = torch.empty((gt_filtered.shape[0], 4), requires_grad=False)
            gt[:, 2] = torch.from_numpy(gt_filtered.width.values).float() / self.stride
            gt[:, 3] = torch.from_numpy(gt_filtered.height.values).float() / self.stride
            gt[:, 0] = torch.from_numpy(gt_filtered.x_top_left.values).float() / self.stride + (gt[:, 2] / 2)
            gt[:, 1] = torch.from_numpy(gt_filtered.y_top_left.values).float() / self.stride + (gt[:, 3] / 2)

            # Set confidence mask of matching detections to 0
            iou_gt_pred = bbox_ious(gt, cur_pred_boxes)
            mask = (iou_gt_pred > self.thresh).sum(0) >= 1
            conf_mask[b][mask.view_as(conf_mask[b])] = 0

            # Find best anchor for each gt
            iou_gt_anchors = bbox_wh_ious(gt, anchors)
            _, best_anchors = iou_gt_anchors.max(1)

            # Set masks and target values for each gt
            nGT = gt.shape[0]
            gi = gt[:, 0].clamp(0, nW-1).long()
            gj = gt[:, 1].clamp(0, nH-1).long()

            conf_mask[b, best_anchors, gj, gi] = self.object_scale
            tconf[b, best_anchors, gj, gi] = iou_gt_pred.view(nGT, nA, nH, nW)[torch.arange(nGT), best_anchors, gj, gi]
            coord_mask[b, best_anchors, gj, gi] = 2 - (gt[:, 2] * gt[:, 3]) / nPixels
            tcoord[b, best_anchors, 0, gj, gi] = gt[:, 0] - gi.float()
            tcoord[b, best_anchors, 1, gj, gi] = gt[:, 1] - gj.float()
            tcoord[b, best_anchors, 2, gj, gi] = (gt[:, 2] / self.anchors[best_anchors, 0]).log()
            tcoord[b, best_anchors, 3, gj, gi] = (gt[:, 3] / self.anchors[best_anchors, 1]).log()
            cls_mask[b, best_anchors, gj, gi] = 1
            tcls[b, best_anchors, gj, gi] = torch.from_numpy(gt_filtered.class_id.values).float()

            # Set masks of ignored to zero
            if gt_filtered.ignore.any():
                if torchversion >= version120:
                    ignore_mask = torch.from_numpy(gt_filtered.ignore.values)
                else:
                    ignore_mask = torch.from_numpy(gt_filtered.ignore.values.astype(np.uint8))
                gi = gi[ignore_mask]
                gj = gj[ignore_mask]
                best_anchors = best_anchors[ignore_mask]

                conf_mask[b, best_anchors, gj, gi] = 0
                coord_mask[b, best_anchors, gj, gi] = 0
                cls_mask[b, best_anchors, gj, gi] = 0

        return (
            coord_mask.view(nB, nA, 1, nPixels),
            conf_mask.view(nB, nA, nPixels),
            cls_mask.view(nB, nA, nPixels),
            tcoord.view(nB, nA, 4, nPixels),
            tconf.view(nB, nA, nPixels),
            tcls.view(nB, nA, nPixels)
        )



def bbox_ious(boxes1, boxes2):
    """ Compute IOU between all boxes from ``boxes1`` with all boxes from ``boxes2``.

    Args:
        boxes1 (torch.Tensor): List of bounding boxes
        boxes2 (torch.Tensor): List of bounding boxes

    Returns:
        torch.Tensor[len(boxes1) X len(boxes2)]: IOU values

    Note:
        Tensor format: [[xc, yc, w, h],...]
    """
    b1x1, b1y1 = (boxes1[:, :2] - (boxes1[:, 2:4] / 2)).split(1, 1)
    b1x2, b1y2 = (boxes1[:, :2] + (boxes1[:, 2:4] / 2)).split(1, 1)
    b2x1, b2y1 = (boxes2[:, :2] - (boxes2[:, 2:4] / 2)).split(1, 1)
    b2x2, b2y2 = (boxes2[:, :2] + (boxes2[:, 2:4] / 2)).split(1, 1)

    dx = (b1x2.min(b2x2.t()) - b1x1.max(b2x1.t())).clamp(min=0)
    dy = (b1y2.min(b2y2.t()) - b1y1.max(b2y1.t())).clamp(min=0)
    intersections = dx * dy

    areas1 = (b1x2 - b1x1) * (b1y2 - b1y1)
    areas2 = (b2x2 - b2x1) * (b2y2 - b2y1)
    unions = (areas1 + areas2.t()) - intersections

    return intersections / unions


def bbox_wh_ious(boxes1, boxes2):
    """ Shorter version of :func:`lightnet.network.loss._regionloss.bbox_ious`
    for when we are only interested in W/H of the bounding boxes and not X/Y.

    Args:
        boxes1 (torch.Tensor): List of bounding boxes
        boxes2 (torch.Tensor): List of bounding boxes

    Returns:
        torch.Tensor[len(boxes1) X len(boxes2)]: IOU values when discarding X/Y offsets (aka. as if they were zero)

    Note:
        Tensor format: [[xc, yc, w, h],...]
    """
    b1w = boxes1[:, 2].unsqueeze(1)
    b1h = boxes1[:, 3].unsqueeze(1)
    b2w = boxes2[:, 2]
    b2h = boxes2[:, 3]

    intersections = b1w.min(b2w) * b1h.min(b2h)
    unions = (b1w * b1h) + (b2w * b2h) - intersections

    return intersections / unions