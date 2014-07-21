import cv2
import time
import datetime


capture_dir = "captured/"


def create_dir(c_dir):
    dir_name = c_dir + str(datetime.datetime.now())

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
    save_dir = create_dir(capture_dir)
    while(1):
        _, f = c.read()
        filename = "{}/{}.jpg".format(save_dir, time.time())

        cv2.imwrite(f, filename)

        if cv2.waitKey(1) == 27:
            break


if __name__ == "__main__":
    main_loop()
