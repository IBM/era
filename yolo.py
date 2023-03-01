import site
import sys
from os import makedirs
from pathlib import Path

import numpy as np
from torch2hpvm import ModelExporter

self_folder = Path(__file__).parent.absolute()
site.addsitedir(self_folder.as_posix())

from torch_dnn import BUFFER_NAME, quantize
from torch_dnn.yolo import TinyYoloPL

# Configs
ASSET_DIR = self_folder / "assets/yolo"
QUANT_STRAT = "NONE"  # Quantization method
WORKING_DIR = Path("./gen_yolo")
N_IMAGES = 100
if len(sys.argv) > 1:
    WORKING_DIR = Path(sys.argv[1])

# Reproducibility
np.random.seed(42)
# Create working directory
makedirs(WORKING_DIR, exist_ok=True)

# Calculate quantization scales
ckpt = ASSET_DIR / "yolo.ckpt"
model: TinyYoloPL = TinyYoloPL.load_from_checkpoint(
    ckpt.as_posix(), num_classes=12, dataset_path=ASSET_DIR / "atr_dataset.tar.gz"
)
#model.merge_conv_bn_()
scale_output, _ = quantize(model, QUANT_STRAT, WORKING_DIR)

# Code generation (into ./yolo_output/hpvm-mod.nvdla)
nvdla_buffer = WORKING_DIR / BUFFER_NAME
print(f"Generating NVDLA buffer into {nvdla_buffer}")
dataset = model.test_dataloader().dataset
exporter = ModelExporter(model, dataset, WORKING_DIR, scale_output)
exporter.generate(n_images=N_IMAGES).compile(WORKING_DIR / "miniera", WORKING_DIR)

# Now we don't need the calibration file (calib.txt) except the very last value,
# so we take that out and write just that right back.
with open(WORKING_DIR / "calib.txt") as f:
    scale = float(f.readlines()[-1].split(":")[1].strip())
with open(WORKING_DIR / "calib.txt", "w") as f:
    print(scale, file=f)
