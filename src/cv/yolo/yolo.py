# this program just mimics the behavior of mio.py but with the new yolo model
# im testing this w/ hpvm docker image + eventually will integrate with nvdla
# and approxhpvm as well

from random import randint
from PIL import Image
from torchvision import transforms
from yolo_model import LITinyYolo
import cv2
from matplotlib import pyplot as plt
import lightnet as ln
import argparse
import sys
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

# try:
#     from frontend.approxhpvm_translator import translate_to_approxhpvm
# except:
#     raise ImportError('uh oh! import error')
# translate_to_approxhpvm(model, "yolo_data/hpvm/", X_test, Y_test, num_classes)

batch_size = 1
num_classes = 12
img_cols = 640
img_rows = 480
num_channels = 3
run_dir = '/home/espuser/mini-era/cv/yolo/'

model = None
classes = ['BMP2', 'D20', 'MTLB', 'T72', 'ZSU23', '2S3',
           'PICKUP', 'MAN', 'VEHICLE', 'SUV', 'BRDM2', 'BTR70']

BOX_COLOR = (255, 0, 0) # Red
TEXT_COLOR = (255, 255, 255) # White

# This is to visualize the bounding box and label on the image
def visualize_bbox(img, bbox, class_name, color=BOX_COLOR, thickness=2):
    """Visualizes a single bounding box on the image"""
    x_min, y_min, w, h = bbox
    x_min, x_max, y_min, y_max = int(x_min), int(x_min + w), int(y_min), int(y_min + h)
   
    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color=color, thickness=thickness)
    
    ((text_width, text_height), _) = cv2.getTextSize(class_name, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)    
    cv2.rectangle(img, (x_min, y_min - int(1.3 * text_height)), (x_min + text_width, y_min), BOX_COLOR, -1)
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

# This is to visualize the bounding box and label on the image
def visualize(image, bbox, class_name):
    img = image.copy()
    img = visualize_bbox(img, bbox, class_name)
    print("Plotting image with bounded box")
    plt.figure(figsize=(12, 12))
    plt.axis('off')
    plt.imshow(img)
    plt.show()

# this is supposed to be replaced by a C version at some point ??
# this could also be replaced with non_eval_post_processing which
# i believe just leaves out the conversion to a pandas dataframe w/
# typical brambox info
def postprocessing(predictions, model, classes):
    return ln.data.transform.Compose([
        # GetBoxes transformation generates bounding boxes from network output
        ln.data.transform.GetDarknetBoxes(
            conf_thresh=0.5,
            network_stride=model.stride,
            anchors=model.anchors
        ),

        # Filter transformation to filter the output boxes
        ln.data.transform.NMS(
            nms_thresh=0.5
        ),

        # Miscellaneous transformation that transforms the output boxes to a brambox dataframe
        ln.data.transform.TensorToBrambox(
            class_label_map=classes,
        )
    ])(predictions)


def yolo_model():
    # eventually replace with function to pull model from nvdla
    model = LITinyYolo.load_from_checkpoint(
        run_dir + 'weights.ckpt', classes=classes)
    return model


def loadmodel(): # --> called by kernels_api.c
    global model
    model = yolo_model()


def predict(imagetype): # --> called by kernels_api.c
    # eventually replace the image selection code below with code to pull image from 
    # nvdla instead

    # not utilizing object from trace here since its hpvm is not passing args correctly
    #imagetype = randint(0, 11)

    # # fit MIO labels to YOLO model labels
    # if imagetype in [-1, 0]:         # nothing = nothing
    #     imagetype = -1
    # elif imagetype == 1:             # bus = large military vehicle
    #     imagetype = choice([0, 2, 4, 5])
    # elif imagetype == 2:             # car = normal vehicle
    #     imagetype = choice([6, 8, 9])
    # elif imagetype == 3:             # pedestrian = man
    #     imagetype = 7
    # elif imagetype == 4:             # truck = small military vehicle
    #     imagetype = 4
    # else:
    #     raise ValueError('uh oh!')

    # if imagetype != -1: # aka not myself or nothing
    # select text file of image locations for class specified by imagetype
    class_file = run_dir + 'class_files/' + str(imagetype) + '.txt'

    # randomly select an image from desired class (aka first label in label file)
    # note to self -> the logic by which images were split up into classes is
    # found in split_by_class.py, not here
    with open(class_file, 'r') as file:
        lines = file.readlines()
        mini = 0
        maxi = len(lines) - 1
        rand = randint(mini, maxi)
        path = lines[rand][:-1]
    test_image = Image.open(path).convert('RGB')
    #image_num  = cv2.imread(path)
    #image_num  = cv2.cvtColor(image_num, cv2.COLOR_BGR2RGB)

    # transform image to proper pytorch tensor format
    transform = transforms.Compose([
        transforms.ToTensor()])
    img_tensor = transform(test_image).to('cpu').unsqueeze(0)

    # predict and extract label from network output
    outputs = model(img_tensor)
    #print(outputs)
    print(path, imagetype)
    dets = postprocessing(outputs, model, classes) # eventually replace with other postprocessing
    #columns of dets: 'image', 'class_label', 'id', 'x_top_left', 'y_top_left', 'width','value_type'
    x_min = dets['x_top_left']
    y_min = dets['y_top_left']
    width = dets['width']
    height = dets['height']

    try:
        # attempt to match string label with integer label
        label_str = dets['class_label'][0]
        label = classes.index(label_str)
        #print("label", label, label_str)
        #bbox = [x_min, y_min, width, height]
        #visualize(image_num, bbox, label_str)
    except:
        # uh oh! no detections found
        label = -1
    # else: 
    #     label = -1
    
    # fit YOLO labels to MIO model labels, assigned in order to make likelihood of different
    # objects equal for testing purposes
    if label == -1: # myself or nothing
        corrected_label = 0
    elif label in (0, 1, 2, 3, 4):
        corrected_label = label
    else: #> 4 Say nothing
        correcteed_label = 0
    return corrected_label


# only needed for individual testing purposes
# def main(argv):
#     print('Running command:', str(sys.argv))
#     parser = argparse.ArgumentParser()

#     parser.add_argument("-t", "--objecttype", type=int,
#                         help="Class from ATR dataset")

#     args = parser.parse_args()
#     global model
#     loadmodel()
#     val = predict(args.objecttype)

#     print("CNN Predicted Label: %u\n" % val)


# if __name__ == "__main__":
#     main(sys.argv[1:])
