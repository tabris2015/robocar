from keras.models import model_from_json
import numpy as np
import tensorflow as tf
from keras.models import Sequential
from keras.models import Model
from keras.layers import Input, Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D, BatchNormalization
from keras.layers import Activation
from keras import backend as K

def base(input_shape):
    model = Sequential()
    model.add(Conv2D(64, kernel_size=(3, 3), activation='relu', input_shape=input_shape))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))

    model.add(Conv2D(32, (3, 3), activation='relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))

    model.add(Conv2D(16, (3, 3), activation='relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))

    model.add(Flatten())
    model.add(Dense(32, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(12, activation='relu'))
    model.add(Dense(1, activation='tanh'))

    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])
    return model


def conv1(input_shape):
    model = Sequential()
    model.add(Conv2D(3, kernel_size=(1, 1), activation='relu', input_shape=input_shape))
    model.add(Conv2D(8, (3, 3), activation='relu'))
    model.add(Conv2D(16, (3, 3), activation='relu'))
    model.add(Conv2D(32, (3, 3), activation='relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(0.25))
    model.add(Flatten())
    model.add(Dense(32, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(12, activation='relu'))
    model.add(Dense(1, activation='tanh'))

    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])
    return model

def conv2(input_shape):
    model = Sequential()
    model.add(Conv2D(8, kernel_size=(5, 5), input_shape=input_shape))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    
    model.add(Conv2D(16, (3, 3)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(32, (3, 3)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(64, (2, 2)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(128, (1, 1)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    model.add(Dropout(0.25))
    model.add(Flatten())
    model.add(Dense(32, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(16, activation='relu'))
    model.add(Dense(1, activation='tanh'))


    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['accuracy'])
    return model


def custom_loss(y_true, y_pred):
    
    loss = tf.square(y_true - y_pred)
    loss = .5 * tf.reduce_mean(loss)
    return loss

def simple1(input_shape):   
    # this network is used with a 80 x 160 image size 
    # Construct the network 
    image_inp = Input(shape=input_shape)

    x = Conv2D(filters=16, kernel_size=(3, 5), activation='relu', padding='valid')(image_inp)
    x = Conv2D(filters=16, kernel_size=(3, 5), activation='relu', padding='valid')(x)
    x = MaxPooling2D((4, 2))(x)

    x = Conv2D(filters=32, kernel_size=(3, 5), activation='relu', padding='valid')(x)
    x = Conv2D(filters=32, kernel_size=(3, 5), activation='relu', padding='valid')(x)
    x = MaxPooling2D((4, 2))(x)

    x = Conv2D(filters=4,  kernel_size=(1, 1), activation='linear', padding='same')(x)

    x = Flatten()(x)

    x = Dense(1, activation='tanh', kernel_regularizer='l1')(x)

    angle_out = x

    model = Model(inputs=[image_inp], outputs=[angle_out])
    model.compile(loss=custom_loss, optimizer='adam', metrics=['accuracy'])

    return model

def simple2(input_shape):   
    # this network is used with a 80 x 160 image size 
    # Construct the network 
    image_inp = Input(shape=input_shape)

    x = Conv2D(filters=16, kernel_size=(3, 5), activation='relu', padding='valid')(image_inp)
    x = Conv2D(filters=16, kernel_size=(3, 5), activation='relu', padding='valid')(x)
    x = MaxPooling2D((4, 2))(x)

    x = Conv2D(filters=32, kernel_size=(3, 5), activation='relu', padding='valid')(x)
    x = Conv2D(filters=32, kernel_size=(3, 5), activation='relu', padding='valid')(x)
    x = MaxPooling2D((4, 2))(x)

    x = Conv2D(filters=64, kernel_size=(3, 5), activation='relu', padding='valid')(x)
    x = Conv2D(filters=64, kernel_size=(3, 5), activation='relu', padding='valid')(x)
    x = MaxPooling2D((4, 2))(x)

    x = Conv2D(filters=4,  kernel_size=(1, 1), activation='linear', padding='same')(x)
    
    x = Flatten()(x)

    x = Dense(1, activation='tanh', kernel_regularizer='l1')(x)

    angle_out = x

    model = Model(inputs=[image_inp], outputs=[angle_out])
    model.compile(loss=custom_loss, optimizer='adam', metrics=['accuracy'])

    return model