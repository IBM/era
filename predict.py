import site
import sys
from os import makedirs
from pathlib import Path
from torch import ones, unsqueeze, stack, tensor, reshape
from torch import max as torch_max
from torch import int8 as torch_int8
from torch import int32 as torch_int32
from PIL import Image, ImageDraw
from torchvision import transforms
from torchvision.utils import save_image
import numpy as np

import cv2
import lightnet as ln
import numpy as np

BOX_COLOR = (255, 0, 0)  # Red
TEXT_COLOR = (255, 255, 255)  # White

CLASSES = ['BMP2', 'D20', 'MTLB', 'T72', 'ZSU23', '2S3',
           'PICKUP', 'MAN', 'VEHICLE', 'SUV', 'BRDM2', 'BTR70']

self_folder = Path(__file__).parent.absolute()
site.addsitedir(self_folder.as_posix())

from torch_dnn import BUFFER_NAME, quantize
from torch_dnn.yolo import TinyYoloPL

# Configs
ASSET_DIR = self_folder / "assets/yolo"
QUANT_STRAT = "NONE"  # Quantization method
WORKING_DIR = Path("./gen_predict")
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


img_pre = Image.open("gen_yolo/images/44.jpg")
convert_img = transforms.ToTensor()
flat_img = convert_img(img_pre)
flat_img = unsqueeze(flat_img, dim=0)

def predict(model, img, img_path):
    # img_path is path to output image
    img_rgb = np.asarray(img)
    transform = transforms.Compose([transforms.ToTensor()])
    img = transform(img_rgb.astype(np.uint8)).to('cpu').unsqueeze(0)

    model.eval()
    detections = model(img)
    process_and_save(model, detections, img_rgb, img_path)
    return detections

def process_and_save(model, detections, img_rgb, img_path):
    img_detections = postprocessing(model, detections)
    print("DETECTIONS:", img_detections.shape)


    im_bboxes = draw_bboxes(model, img_rgb, img_detections)
    if (len(img_detections) > 0):
        cv2.imwrite(img_path, im_bboxes)

    return img_detections.to_dict(orient='records')

def postprocessing(model, predictions):
    return ln.data.transform.Compose([
        ln.data.transform.GetDarknetBoxes(
            conf_thresh=0.4,
            network_stride=model.stride,
            anchors=model.anchors
        ),
        ln.data.transform.NMS(
            nms_thresh=0.4
        ),
        ln.data.transform.TensorToBrambox(
            class_label_map=CLASSES,
        )
    ])(predictions)

def draw_single_bbox(model, img, bbox, class_name, color=BOX_COLOR, thickness=2):
    """Adds a single bounding box on the image"""
    print(bbox)
    x_min, y_min, w, h = bbox
    x_min, x_max, y_min, y_max = int(x_min), int(
        x_min + w), int(y_min), int(y_min + h)

    cv2.rectangle(img, (x_min, y_min), (x_max, y_max),
                color=color, thickness=thickness)

    ((text_width, text_height), _) = cv2.getTextSize(
        class_name, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)
    cv2.rectangle(img, (x_min, y_min - int(1.3 * text_height)),
                (x_min + text_width, y_min), BOX_COLOR, -1)
    cv2.putText(
        img,
        text=class_name,
        org=(x_min, y_min - int(0.3 * text_height)),
        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=0.35,
        color=TEXT_COLOR,
        lineType=cv2.LINE_AA,
    )
    return img

def draw_bboxes(model, image, detections):
    img = image.copy()
    detections = detections.reset_index()

    print("DETECTIONS RESET:", detections.shape, detections)
    for index, row in detections.iterrows():
        bbox = [row['x_top_left'], row['y_top_left'],
                row['width'], row['height']]
        class_name = row['class_label']

        print(class_name, bbox)
        img = draw_single_bbox(model, img, bbox, class_name)
    return img


# flat_img = ones(1, 3, 480, 640)
# save_image(flat_img[0], 'ones.jpg')

# ones_img_pre = Image.open("/dccstor/epochs/aporvaa/hpvm/hpvm/test/epoch_dnn/gen_yolo/images/44.jpg")
# convert_img = transforms.ToTensor()
# ones_img = convert_img(ones_img_pre)
# ones_img = unsqueeze(ones_img, dim=0)
# 
model.merge_conv_bn_()

print("Flat image", flat_img)
output_prequant = predict(model, img_pre, './gold_predict.jpg')
# print("Pre quantized output:", output_prequant.size(), output_prequant)

# show_images_with_boxes(flat_img, output_prequant, 'gold.png')

lines = []
with open("./output.dimg","r+") as f:
    lines = f.read()
# 
loaded_list = [float(a) * (0.4426948090671342) for a in lines.split(' ')[:-1]]
loaded_tensor = tensor(loaded_list)
loaded_tensor = reshape(loaded_tensor, output_prequant.size())
# 
print("LOADED TENSOR", loaded_tensor.size(), loaded_tensor)
# 
img_rgb = np.asarray(img_pre)
process_and_save(model, loaded_tensor, img_rgb, './fpga.jpg')

# show_images_with_boxes(flat_img, loaded_tensor, 'fpga.png')

# exit(1)




