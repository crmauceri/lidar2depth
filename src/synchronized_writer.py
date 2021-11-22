#!/usr/bin/env python
import rospy, sys

import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os.path
import csv

from sensor_msgs.msg import CompressedImage

import numpy as np


class SynchronizedWriter:

    def __init__(self):
        self.save_root = ""
        self.iter = 0
       
        # Load params
        if rospy.has_param("save_root"):
            self.save_root = rospy.get_param("save_root")
            rospy.loginfo("Save to directory: {}".format(self.save_root))
        if rospy.has_param("subscribe_to"):
            self.sub_params = rospy.get_param("subscribe_to")
            rospy.loginfo("Subscribing to: {}".format(self.sub_params))
        else:
            rospy.logerr("Missing sub_params. Shutting down.")
            exit(1)

        self.log_file = open(os.path.join(self.save_root, 'log.csv'), 'w')
        self.log = csv.writer(self.log_file, delimiter='\t')

        subs = []
        headers = ['Iteration']
        for sub_params in self.sub_params:
            headers.append(sub_params['pub'])
            if sub_params['type'] == 'CompressedImage':
                subs.append(message_filters.Subscriber(sub_params['pub'], CompressedImage))
            elif sub_params['type'] == 'Image':
                # Input
                self.bridge = CvBridge()
                subs.append(message_filters.Subscriber(sub_params['pub'], Image))
            else:
                raise ValueError('Type not supported: {}'.format(sub_params['type']))
        self.log.writerow(headers)

        rospy.loginfo(subs)
        ts = message_filters.ApproximateTimeSynchronizer(subs, 10, 0.5)
        ts.registerCallback(self.callback)
        rospy.loginfo("Writer initialized")

    def callback(self, *msgs):
        rospy.loginfo("Callback")
        row = [self.iter]
        for i, msg in enumerate(msgs):
            stamp = msg.header.stamp
            time_str = '%d.%06d' % (stamp.secs, stamp.nsecs)
            row.append(time_str)

            sub_params = self.sub_params[i]
            if sub_params['format'] == "RGB" or sub_params['format']=="BGR":
                image = self.rgb_convert(msg, sub_params)
            elif sub_params['format'] == "mono16":
                image = self.mono_16_convert(msg, sub_params)
            else:
                raise ValueError('Image format not supported: {}'.format(sub_params['format']))
            
            # Save as png image
            stamp = msg.header.stamp
            time_str = '%d.%06d' % (stamp.secs, stamp.nsecs)
            
            filepath = os.path.join(self.save_root, (sub_params['format_string'] % self.iter))
            cv2.imwrite(filepath, image)

        rospy.loginfo(row)
        self.log.writerow(row)
        self.iter += 1

    def rgb_convert(self, img_msg, sub_params):
        # Convert ros image to cv matrix using same encoding
        if sub_params['type'] == 'CompressedImage':
            np_arr = np.fromstring(img_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
            except CvBridgeError as e:
                rospy.logerror(e)
                return

        # if sub_params['format'] == 'BGR':
        #     cv2.cvtColor(cv_image, cv_image, cv2.COLOR_BGR2RGB)
        return cv_image

    def mono_16_convert(self, img_msg, sub_params):
        stamp = img_msg.header.stamp

        # Convert ros image to cv matrix using same encoding
        if sub_params['type'] == 'CompressedImage':
            cv_image = self.compressedDepthDecode(img_msg)
        else:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding=sub_params['format'])
            except CvBridgeError as e:
                print(e)
                return

        return cv_image

    def compressedDepthDecode(self, img_msg):
        # Code from https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
        # 'msg' as type CompressedImage
        depth_fmt, compr_type = img_msg.format.split(';')
        # remove white space
        depth_fmt = depth_fmt.strip()
        compr_type = compr_type.strip()
        if compr_type != "compressedDepth":
            raise Exception("Compression type is not 'compressedDepth'."
                            "You probably subscribed to the wrong topic.")

        # remove header from raw data
        depth_header_size = 12
        raw_data = img_msg.data[depth_header_size:]

        depth_img_raw = cv2.imdecode(np.fromstring(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)
        if depth_img_raw is None:
            # probably wrong header size
            raise Exception("Could not decode compressed depth image."
                            "You may need to change 'depth_header_size'!")

        if depth_fmt == "16UC1":
            # write raw image data
            return depth_img_raw
        elif depth_fmt == "32FC1":
            raw_header = img_msg.data[:depth_header_size]
            # header: int, float, float
            [compfmt, depthQuantA, depthQuantB] = struct.unpack('iff', raw_header)
            depth_img_scaled = depthQuantA / (depth_img_raw.astype(np.float32) - depthQuantB)
            # filter max values
            depth_img_scaled[depth_img_raw == 0] = 0

            # depth_img_scaled provides distance in meters as f32
            # for storing it as png, we need to convert it to 16UC1 again (depth in mm)
            depth_img_mm = (depth_img_scaled * 1000).astype(np.uint16)
            return depth_img_mm
        else:
            raise Exception("Decoding of '" + depth_fmt + "' is not implemented!")

def main(args):
    rospy.init_node('writer', anonymous=True)
    node = SynchronizedWriter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        node.log_file.close()


if __name__ == '__main__':
    main(sys.argv)