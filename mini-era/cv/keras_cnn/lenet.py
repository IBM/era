
import sys
import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Activation
from keras.layers import Conv2D, MaxPooling2D, ZeroPadding2D
from keras import backend as K
#from frontend.approxhpvm_translator import translate_to_approxhpvm


batch_size = 128
num_classes = 10
epochs = 5

# input image dimensions
img_rows, img_cols = 28, 28
  


if __name__ == "__main__":    
    
    K.set_image_data_format('channels_first')

    # the data, split between train and test sets
    (x_train, y_train), (x_test, y_test) = mnist.load_data()
    test_labels = y_test

    
    if K.image_data_format() == 'channels_first':
        x_train = x_train.reshape(x_train.shape[0], 1, img_rows, img_cols)
        x_test = x_test.reshape(x_test.shape[0], 1, img_rows, img_cols)
        input_shape = (1, img_rows, img_cols)
    else:
        x_train = x_train.reshape(x_train.shape[0], img_rows, img_cols, 1)
        x_test = x_test.reshape(x_test.shape[0], img_rows, img_cols, 1)
        input_shape = (img_rows, img_cols, 1)


    print(K.image_data_format())

    x_train = x_train.astype('float32')
    x_test = x_test.astype('float32')
    x_train /= 255
    x_test /= 255
    print('x_train shape:', x_train.shape)
    print(x_train.shape[0], 'train samples')
    print(x_test.shape[0], 'test samples')
    
    # convert class vectors to binary class matrices
    y_train = keras.utils.to_categorical(y_train, num_classes)
    y_test = keras.utils.to_categorical(y_test, num_classes)


    activation_type = 'relu'    
    model = Sequential()
    
    model.add(Conv2D(32, kernel_size=(5, 5),
                     activation=activation_type,
                     padding = 'same',
                     input_shape=input_shape))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Conv2D(64, (5, 5), activation=activation_type, padding = 'same'))
    model.add(ZeroPadding2D(padding = (1,1)))
    model.add(Conv2D(64, (3, 3), strides = (2,2), activation=activation_type) )
    model.add(Flatten())
    model.add(Dense(1024, activation=activation_type))
    model.add(Dense(num_classes, activation=activation_type))
    model.add(Activation('softmax'))

    
    model.compile(loss=keras.losses.categorical_crossentropy,
                  optimizer=keras.optimizers.Adadelta(),
                  metrics=['accuracy'])

    model.fit(x_train, y_train,
              batch_size=batch_size,
              epochs=6,
              verbose=1,
              validation_data=(x_test, y_test))


    score = model.evaluate(x_test, y_test, verbose=0)
    print('Test loss:', score[0])
    print('Test accuracy:', score[1])
    
    model.summary()
    
    # translate_to_approxhpvm(model, "data/lenet_hpvm/", x_test, test_labels, 10)

