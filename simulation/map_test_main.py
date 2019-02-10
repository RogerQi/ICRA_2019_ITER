# import the pygame module, so you can use it
from map import *
from arena_config import *
import arena_config as arena
import math
import cv2 as cv

# define a main function
def main():

    map = Map(MapResolution.CM)
    robo = MapRobot()
    cv.namedWindow('image', cv.WINDOW_NORMAL)
    cv.resizeWindow('image', map.get_y_cnt(),map.get_x_cnt())

    img = np.full((map.get_x_cnt(),map.get_y_cnt(),3), 125, np.uint8)

    running = True
    map.draw_map(img)
    map.draw_robot(img, robo)
    cv.imshow('image',img)

    while (running):
        key = cv.waitKey(0)
        map.move(robo, key)
        map.draw_map(img)
        map.draw_robot(img, robo)
        cv.imshow('image',img)
        if (key == 27): running = False

    cv.destroyAllWindows()

# run the main function only if this module is executed as the main script
# (if you import this as a module then nothing is executed)
if __name__=="__main__":
    # call the main function
    main()
