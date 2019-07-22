import numpy as np
from skimage import color, exposure, transform
from skimage import io
import os
import glob
import pandas as pd
import keras
from keras.models import Sequential, load_model
from keras.layers import Dense, Dropout, Flatten, Activation
from keras.layers import Conv2D, MaxPooling2D, ZeroPadding2D
from keras.callbacks import ModelCheckpoint
from keras.preprocessing.image import ImageDataGenerator
from sklearn.model_selection import train_test_split

from keras import backend as K
#from frontend.approxhpvm_translator import translate_to_approxhpvm


batch_size = 64
epochs = 10

num_classes = 43
img_size = 48

def preprocess_img(img):
    # Histogram normalization in v channel
    hsv = color.rgb2hsv(img)
    hsv[:, :, 2] = exposure.equalize_hist(hsv[:, :, 2])
    img = color.hsv2rgb(hsv)

    # central square crop
    min_side = min(img.shape[:-1])
    centre = img.shape[0] // 2, img.shape[1] // 2
    img = img[centre[0] - min_side // 2:centre[0] + min_side // 2,
              centre[1] - min_side // 2:centre[1] + min_side // 2,
              :]

    # rescale to standard size
    img = transform.resize(img, (img_size, img_size))

    # roll color axis to axis 0
    img = np.rollaxis(img, -1)

    return img


def get_class(img_path):
    return int(img_path.split('/')[-2])

def cnn_model():
    activation_type = 'relu'    
    model = Sequential()

    model.add(Conv2D(32, kernel_size=(3, 3),
                        activation=activation_type,
                        padding='same',
                        input_shape=(3, img_size, img_size)))
           
    model.add(Conv2D(32, (3, 3), activation=activation_type))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    # model.add(Dropout(0.2))

    model.add(Conv2D(64, (3, 3), padding='same',
                     activation=activation_type))
    model.add(Conv2D(64, (3, 3), activation=activation_type))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    # model.add(Dropout(0.2))

    model.add(Conv2D(128, (3, 3), padding='same',
                     activation=activation_type))
    model.add(Conv2D(128, (3, 3), activation=activation_type))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    # model.add(Dropout(0.2))

    model.add(Flatten())
    model.add(Dense(512, activation=activation_type))
    # model.add(Dropout(0.5))
    model.add(Dense(num_classes, activation='softmax'))


    return model


if __name__ == "__main__":   

    K.set_image_data_format('channels_first')

   root_dir = 'GTSRB/Final_Training/Images/'
   imgs = []
   labels = []


   all_img_paths = glob.glob(os.path.join(root_dir, '*/*.ppm'))
   np.random.shuffle(all_img_paths)
   i = 1
   for img_path in all_img_paths:
       img = preprocess_img(io.imread(img_path))
       label = get_class(img_path)
       imgs.append(img)
       labels.append(label)
       if (i % 10000 == 0):
         print("Image: " + str(i))
       i += 1

   X = np.array(imgs, dtype='float32')
   Y = np.eye(num_classes, dtype='uint8')[labels]

   X_train, X_val, Y_train, Y_val = train_test_split(X, Y,
                                                 test_size=0.2, random_state=42)

   datagen = ImageDataGenerator(featurewise_center=False,
                                featurewise_std_normalization=False,
                                width_shift_range=0.1,
                                height_shift_range=0.1,
                                zoom_range=0.2,
                                shear_range=0.1,
                                rotation_range=10.)

   datagen.fit(X_train)

   # model = cnn_model()
   model = load_model('model.h5')

   model.compile(loss=keras.losses.categorical_crossentropy,
                 optimizer=keras.optimizers.Adadelta(),
                 metrics=['accuracy'])
   
   model.fit_generator(datagen.flow(X_train, Y_train, batch_size=batch_size),
                   steps_per_epoch=X_train.shape[0],
                   epochs=epochs,
                   validation_data=(X_val, Y_val),
                   callbacks=[ModelCheckpoint('model_data.h5', save_best_only=True)]
                   )
#
  # Load saved model
    model = load_model('model_data.h5')

    test = pd.read_csv('GTSRB/GT-final_test.csv', sep=';')

    # Load test dataset
    X_test = []
    Y_test = []
    i = 0
    for file_name, class_id in zip(list(test['Filename']), list(test['ClassId'])):
        img_path = os.path.join('GTSRB/Final_Test/Images/', file_name)
        X_test.append(preprocess_img(io.imread(img_path)))
        Y_test.append(class_id)

    X_test = np.array(X_test)
    Y_test = np.array(Y_test)

    # predict and evaluate
    Y_pred = model.predict_classes(X_test)
    acc = np.sum(Y_pred == Y_test) / np.size(Y_pred)
    print("Test accuracy = {}".format(acc))

  
    model.summary()
    
    # translate_to_approxhpvm(model, "data/lenet_hpvm/", X_test, Y_test, num_classes)

