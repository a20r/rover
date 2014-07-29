import cv2
import os
import datetime


capture_dir = "captured/"


def create_dir(c_dir):
    dir_name = c_dir + str(datetime.datetime.now()).replace(" ", "_")

    try:
        os.mkdir(c_dir)
    except OSError as e:
        print e

    try:
        os.mkdir(dir_name)
    except OSError as e:
        print e
        exit()

    return dir_name


def main_loop():
    c = cv2.VideoCapture(0)
    # save_dir = create_dir(capture_dir)
    i = 0
    while(1):
        _, f = c.read()
        # filename = "{}/{}.jpg".format(save_dir, i)

        # cv2.imwrite(filename, f)
        cv2.imshow('window', f)
        i += 1

        if cv2.waitKey(1) == 27:
            break


if __name__ == "__main__":
    main_loop()
