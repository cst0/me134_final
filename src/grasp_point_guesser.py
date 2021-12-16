#!/usr/bin/env python3

import rospy
import ros_numpy
import numpy as np
from math import radians
from rospy.rostime import Duration
from sensor_msgs.msg import Image, CameraInfo
from me134_final.msg import Bar

""" Constant values that we'll likely need to adjust: """

# approx. 1 ft in mm
# we don't care about things further than this since we can't reach them
FAR_THRESHOLD_DISTANCE = 305

# chosen arbitrarily
# we don't care about things closer than this since they won't be a far enough
# reach to bother with
CLOSE_THRESHOLD_DISTANCE = 100

# FoV horizontally and vertically. From D435i datasheet.
F_v = radians(74)
F_h = radians(62)

class GraspPointGuesser:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.image_handler, queue_size=1)
        self.camera_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_handler, queue_size=1)
        self.point_pub = rospy.Publisher('grasp_bar', Bar, queue_size=1)

        self.debug_image_pub = rospy.Publisher("processed_images", Image, queue_size=1)
        self.debug_camera_info_pub = rospy.Publisher("camera_info", CameraInfo, queue_size=1)

        self.image = Image()
        self.have_image = False
        self.image_processor = rospy.Timer(Duration(1/5), self.handle_image_processing)

    def image_handler(self, msg:Image):

        if self.image is not None:
            self.image = msg
            self.have_image = True

    def camera_info_handler(self, msg:CameraInfo):
        self.debug_camera_info_pub.publish(msg)

    def handle_image_processing(self, event):
        del event
        if not self.have_image:
            # processing has outpaced getting images
            return

        self.have_image = False
        depth_array = ros_numpy.numpify(self.image)
        if depth_array is None:
            return

        maxval = np.max(depth_array)
        minval = np.min(depth_array)

        thresholded_array = depth_array < FAR_THRESHOLD_DISTANCE
        thresholded_array[0,0] = minval
        thresholded_array[0,1] = maxval
        bar_array = thresholded_array * depth_array

        img = Image()
        img = ros_numpy.msgify(
                Image,
                np.array(bar_array, dtype=np.uint16),
                self.image.encoding
                )
        img.encoding = self.image.encoding
        img.step = self.image.step
        img.header = self.image.header

        hist, edges = np.histogram(bar_array, bins='auto')
        # so we've now grouped our data. We care about only one of those bins.
        # Off the bat, we know it won't be bin 0: that's full of 0, because
        # that's the stuff we thresholded out.
        max_bin = sorted(hist[1:], reverse=True)[0]
        n = 0
        for n in range(0, len(hist[1:])):
            if max_bin == hist[n]:
                break

        # ok, so now we've got n: the index of our heftiest bin.
        # What values are in that grouping?
        # edges[n] is the start of the bin, edges[n+1] is the end of that bin.
        relevant_min_dist = edges[n]
        relevant_max_dist = edges[n+1]

        # the only points we really care about in this grouping are the ones
        # on the ends: we'll use that to define the ends of our line.

        # TODO: is this the best way? We can also assume the line extends
        # infinitely, which would allow us to select some representative
        # sampling and maybe get a more robust output that way. But,
        # how would we get a representative sample?

        # this flips height and width, but that's OK because we're rotating the camera.
        width, height = bar_array.shape()
        def get_left():
            for w in range(0, width):
                for h in range(0, height):
                    if relevant_min_dist < bar_array[w, h] < relevant_max_dist:
                        return w, h
            return -1, -1
        lw, lh = get_left()

        def get_right():
            for w in range(width, 0, -1):
                for h in range(height, 0, -1):
                    if relevant_min_dist < bar_array[w, h] < relevant_max_dist:
                        return w, h
            return -1, -1
        rw, rh = get_right()

        assert lw != -1
        assert rw != -1
        assert lh != -1
        assert rh != -1

        # we now have a pixel position and its depth.
        # using the camera FoV, we can turn that into radians
        l_horizon_rads = F_h - (lh * F_h)
        r_horizon_rads = F_h - (rh * F_h)
        l_vertical_rads = F_v - (lw * F_v)
        r_vertical_rads = F_v - (rw * F_v)

        bar = Bar()

        bar.l_horizon_rads = l_horizon_rads
        bar.r_horizon_rads = r_horizon_rads
        bar.l_dist_mm = bar_array[lw, lh]

        bar.l_vertical_rads = l_vertical_rads
        bar.r_vertical_rads = r_vertical_rads
        bar.r_dist_mm = bar_array[rw, rh]

        self.point_pub.publish(bar)
        self.debug_image_pub.publish(img)


def main():
    rospy.init_node('GraspPointGuesser')
    rospy.loginfo("Spinning grasp point guesser")
    gpg = GraspPointGuesser()
    rospy.spin()

if __name__ == '__main__':
    main()
