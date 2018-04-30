from keras.preprocessing.image import load_img
from keras.preprocessing.image import img_to_array

from keras.models import model_from_json

import numpy as np
import pandas as pd
import bcolz
import threading

import os
import sys
import glob
import shutil

from sklearn.model_selection import train_test_split

import models

### utils

def file_path_from_db_id(db_id, pattern="%d.bmp", top="dataset/"):
    s = '%09d' % db_id
    return os.path.join(top, pattern % db_id)


def generator_from_df(df, batch_size, target_size, target_column='target', features=None):

    nbatches, n_skipped_per_epoch = divmod(df.shape[0], batch_size)
    count = 1
    epoch = 0
    # New epoch.
    while 1:
        df = df.sample(frac=1) # shuffle in every epoch
        epoch += 1
        i, j = 0, batch_size
        # Mini-batches within epoch.
        mini_batches_completed = 0
        for _ in range(nbatches):
            sub = df.iloc[i:j]
            try:
                X = np.array([(2 * (img_to_array(load_img(f, target_size=target_size)) / 255.0 - 0.5)) for f in sub.imgpath])
                Y = sub[target_column].values
                # Simple model, one input, one output.
                mini_batches_completed += 1
                yield X, Y

            except IOError as err:
                count -= 1

            i = j
            j += batch_size
            count += 1

def extract(target):
    return np.array([float(x) for x in (target.replace("[","").replace("]","").split(","))])

