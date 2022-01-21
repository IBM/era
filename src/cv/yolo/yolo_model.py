import pytorch_lightning as pl
from loss_func import RegionLoss
from collections import OrderedDict, Iterable
import torch.nn as nn
import lightnet.network as lnn

class TinyYoloV2NonLeaky(lnn.module.Darknet):
    stride = 32
    inner_stride = 32
    remap_darknet = [
        (r'^layers.0.(\d+_)',   r'layers.\1'),  # All base layers (1-13)
    ]

    def __init__(self, classes, input_channels=3, anchors=[(1.08, 1.19), (3.42, 4.41), (6.63, 11.38), (9.42, 5.11), (16.62, 10.52)]):
        super().__init__()
        if not isinstance(anchors, Iterable) and not isinstance(anchors[0], Iterable):
            raise TypeError('Anchors need to be a 2D list of numbers')

        # Parameters
        self.classes = classes
        self.input_channels = input_channels
        self.anchors = anchors

        # Network
        momentum = 0.01
        self.layers = nn.Sequential(
            OrderedDict([
                ('1_convbatch',     lnn.layer.Conv2dBatchReLU(input_channels, 16, 3, 1, 1, momentum=momentum)),
                ('2_max',           nn.MaxPool2d(2, 2)),
                ('3_convbatch',     lnn.layer.Conv2dBatchReLU(16, 32, 3, 1, 1, momentum=momentum)),
                ('4_max',           nn.MaxPool2d(2, 2)),
                ('5_convbatch',     lnn.layer.Conv2dBatchReLU(32, 64, 3, 1, 1, momentum=momentum)),
                ('6_max',           nn.MaxPool2d(2, 2)),
                ('7_convbatch',     lnn.layer.Conv2dBatchReLU(64, 128, 3, 1, 1, momentum=momentum)),
                ('8_max',           nn.MaxPool2d(2, 2)),
                ('9_convbatch',     lnn.layer.Conv2dBatchReLU(128, 256, 3, 1, 1, momentum=momentum)),
                ('10_max',          nn.MaxPool2d(2, 2)),
                ('11_convbatch',    lnn.layer.Conv2dBatchReLU(256, 512, 3, 1, 1, momentum=momentum)),
                ('12_max',          lnn.layer.PaddedMaxPool2d(2, 1, (0, 1, 0, 1))),
                ('13_convbatch',    lnn.layer.Conv2dBatchReLU(512, 1024, 3, 1, 1, momentum=momentum)),
                ('14_convbatch',    lnn.layer.Conv2dBatchReLU(1024, 1024, 3, 1, 1, momentum=momentum)),
                ('15_conv',         nn.Conv2d(1024, len(self.anchors)*(5+len(self.classes)), 1, 1, 0)),
            ])
        )

class LITinyYolo(pl.LightningModule):
    def __init__(self, classes, map_style="coco",
                stride=32, 
                anchors=[(1.08, 1.19), (3.42, 4.41), (6.63, 11.38), (9.42, 5.11), (16.62, 10.52)]):
        super().__init__()
        self.map_style = map_style
        self.classes = classes
        self.anchors = anchors
        self.stride = stride
        self.network = TinyYoloV2NonLeaky(classes)
        self.loss = RegionLoss(
            num_classes=len(classes),
            anchors=self.network.anchors,
            stride=self.network.stride
        )
    
    def forward(self, image):
        prediction = self.network(image)
        return prediction