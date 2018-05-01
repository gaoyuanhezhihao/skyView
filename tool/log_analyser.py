import cv2
import argparse
from glob import glob
from os.path import join, split
import re
import numpy as np


def show_wait_order(img):
    cv2.imshow("test", img)
    key = cv2.waitKey(0)
    print(key)
    if 65363 == key:
        print('-->')
    elif 65361 == key:
        print('<--')
    elif ord('q') == key:
        print('quit')



def collect_ids(root):
    paths = glob(join(root, "keyPt/*.jpg"))
    ids = [int(split(p)[1][:-4]) for p in paths]
    return min(ids), max(ids)


def read_match_img(root, m):
    return cv2.imread(join(root, 'match', m+".jpg"))


def read_log(root):
    s = re.compile("\[(\d+--\d+)\]")
    with open(join(root, 'log.txt'), 'r') as f:
        lines = f.readlines()
        l = len(lines)
        p = 0
        n = 0
        matches = []
        logs = []
        log = []
        for line in lines:
            rst = s.search(line)
            if None != rst:
                if 0 != len(log):
                    logs.append('\n'.join(log))
                matches.append(rst.group(1))
                log = []
            else:
                log.append(line)
    return matches, logs




if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('id', action="store", type=int, default=1)
    parser.add_argument('--dir', default='../data/push_pull/far_odom/floor_vo/')
    args = parser.parse_args()

    matches, logs = read_log(args.dir)
    i = args.id
    size = 200, 200, 3
    zero_img = np.zeros(size, dtype=np.uint8)
    while True:
        print(matches[i])
        print(logs[i])
        match_im = read_match_img(args.dir, matches[i])
        match_im = match_im if None != match_im else zero_img
        cv2.imshow("match", match_im)
        key = cv2.waitKey(0)
        if 65363 == key:
            print('-->')
            i=(i+1)%len(matches)
        elif 65361 == key:
            print('<--')
            i-=1
        elif ord('q') == key:
            print('quit')
            break
