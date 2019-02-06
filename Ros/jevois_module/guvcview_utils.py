import os
from os.path import expanduser, join

HOME = expanduser("~")

def set_guvcview_fps(fps, video_num):
    # filename = "/home/niryo/.config/guvcview2/video0"
    filename = os.path.join(HOME, '.config/guvcview2/video%d' % video_num)
    if not os.path.exists(filename):
        open(filename, 'w').close()
    with open(filename, "r+") as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if line.startswith("fps_denom"):
                lines[i] = "fps_denom=" + str(fps) + "\n"
        
        text_to_write = ''.join(lines)
        f.write(text_to_write)

if __name__ == '__main__':
    set_guvcview_fps(7.5)

