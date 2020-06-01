#!/usr/bin/env python
import random
import sys
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16


# rosservice call /gazebo/reset_simulation
# roslaunch usi_project project.launch
# roslaunch usi_project thymio_gazebo_bringup.launch name:=thymio10 world:=final

class ImageConverter:

    def __init__(self):
        rospy.init_node('thymio_controller')  # name of the node
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("thymio10/camera/image_raw", Image, self.sub_callback)
        self.dir_pub = rospy.Publisher("thymio10/direction", Int16, queue_size=10)
        self.image = dict()
        self.centroid = 0

    def image_resize(self):
        image = self.image['origin']
        width, height = image.shape[:2][::-1]
        img_resize = cv2.resize(image,
                                (int(width * 0.5), int(height * 0.5)), interpolation=cv2.INTER_CUBIC)
        self.image['resize'] = img_resize
        # cv2.imshow("Thymio View img_resize", img_resize)

    def image_gauss(self):
        image = self.image['resize']
        image_gauss = cv2.GaussianBlur(image, (3, 3), 0.1, 0.1)
        self.image['gauss'] = image_gauss
        cv2.imshow("Thymio View image_gauss", image_gauss)

    def image_threshold(self):
        image = self.image['gauss']
        image_threshold = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image_threshold = cv2.inRange(image_threshold, (0, 100, 100), (20, 255, 255))

        self.image['threshold'] = image_threshold
        cv2.imshow("Thymio View image_threshold", image_threshold)

    def image_mask(self):
        image = self.image['threshold']

        width, height = image.shape[:2]
        mask = np.zeros([width, height], dtype=np.uint8)
        mask[width // 2:width, 0:height] = 255

        image_mask = cv2.add(image, np.zeros(np.shape(image), dtype=np.uint8), mask=mask)

        self.image['mask'] = image_mask
        cv2.imshow("Thymio View image_mask", image_mask)

    def image_detect(self):
        image = self.image['mask']
        _, contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        drawing = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
        color = (random.randint(0, 256), random.randint(0, 256), random.randint(0, 256))
        cv2.drawContours(drawing, contours, 0, color, 2)

        # Get the moments

        if len(contours) >= 2:
            area = sorted([(i, cv2.contourArea(contours[i])) for i in range(len(contours))], key=lambda x: x[1],
                          reverse=True)
            contour = contours[area[0][0]]
        elif len(contours) == 1:
            contour = contours[0]
        else:
            contour = None
            self.centroid = 2

        if contour is not None:
            mu = cv2.moments(contour)
            # Get the mass centers
            # add 1e-5 to avoid division by zero
            mc = (mu['m10'] / (mu['m00'] + 1e-5), mu['m01'] / (mu['m00'] + 1e-5))
            # Draw contours

            cv2.circle(drawing, (int(mc[0]), int(mc[1])), 4, color, -1)

            self.centroid = int(mc[0])

        # print('point' + str((int(mc[0]), int(mc[1]))))

        cv2.imshow('Thymio View image_detect', drawing)
        self.image['detect'] = drawing

    def image_pub(self):
        cent = self.centroid
        tol = 15
        width = self.image['resize'].shape[:2][1]
        # print(cent)
        if cent >= 10:
            if cent < width // 2 - tol:
                direction = -1
            elif cent > width // 2 + tol:
                direction = 1
            else:
                direction = 0
        else:
            direction = 2
        # print((cent,direction,width))
        self.dir_pub.publish(direction)

    def sub_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image['origin'] = cv_image
            self.image_resize()
            self.image_gauss()
            self.image_threshold()
            self.image_mask()
            self.image_detect()
            self.image_pub()

            cv2.waitKey(5)

        except CvBridgeError as e:
            print(e)


def main(args):
    ic = ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
