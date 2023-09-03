

"""Publish a video as ROS messages.
"""

import argparse

import numpy as np

import cv2

import rospy


from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )



def main():
    """Publish a video as ROS messages.
    """
    # Patse arguments.
    parser = argparse.ArgumentParser(description="Convert video into a rosbag.")
    parser.add_argument("-c", "--camera", default="camera", help="Camera name.")
    parser.add_argument("-f", "--frame_id", default="camera",
                        help="tf frame_id.")
    parser.add_argument("--width", type=np.int32, default="640",
                        help="Image width.")
    parser.add_argument("--height", type=np.int32, default="480",
                        help="Image height.")
    parser.add_argument("--info_url", default="file:///camera.yml",
                        help="Camera calibration url.")

    args = parser.parse_args()

    print "Publishing %s." % (args.camera)

    # Set up node.
    rospy.init_node("video_publisher", anonymous=True)
    img_pub = rospy.Publisher("/" + args.camera + "/image_raw", Image,
                              queue_size=10)
    info_pub = rospy.Publisher("/" + args.camera + "/camera_info", CameraInfo,
                               queue_size=10)


    # Open video.
    video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0))

    # Get frame rate.
    #fps = video.get(cv2.cv.CV_CAP_PROP_FPS)
    rate = rospy.Rate(30)

    # Loop through video frames.
    while not rospy.is_shutdown() and video.grab():
        tmp, img = video.retrieve()

        if not tmp:
            print "Could not grab frame."
            break

        img_out = np.empty((args.height, args.width, img.shape[2]))

        # Compute input/output aspect ratios.
        aspect_ratio_in = np.float(img.shape[1]) / np.float(img.shape[0])
        aspect_ratio_out = np.float(args.width) / np.float(args.height)

        if aspect_ratio_in > aspect_ratio_out:
            # Output is narrower than input -> crop left/right.
            rsz_factor = np.float(args.height) / np.float(img.shape[0])
            img_rsz = cv2.resize(img, (0, 0), fx=rsz_factor, fy=rsz_factor,
                                 interpolation=cv2.INTER_AREA)

            diff = (img_rsz.shape[1] - args.width) / 2
            img_out = img_rsz[:, diff:-diff-1, :]
        elif aspect_ratio_in < aspect_ratio_out:
            # Output is wider than input -> crop top/bottom.
            rsz_factor = np.float(args.width) / np.float(img.shape[1])
            img_rsz = cv2.resize(img, (0, 0), fx=rsz_factor, fy=rsz_factor,
                                 interpolation=cv2.INTER_AREA)

            diff = (img_rsz.shape[0] - args.height) / 2

            img_out = img_rsz[diff:-diff-1, :, :]
        else:
            # Resize image.
            img_out = cv2.resize(img, (args.height, args.width))

        assert img_out.shape[0:2] == (args.height, args.width)

        try:
            # Publish image.
            img_msg = bridge.cv2_to_imgmsg(img_out, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = args.frame_id
            img_pub.publish(img_msg)

            # Publish camera info.
            info_msg = CameraInfo
            info_msg.header = img_msg.header
            info_pub.publish(info_msg)
        except CvBridgeError as err:
            print err

        rate.sleep()

    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
