# USAGE
# python rtod.py

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import argparse
import imutils
import time
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
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=2 ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            display_width,
            display_height,
        )
    )


def main():
    """Publish a video as ROS messages.
    """
    # construct the argument parse and parse the arguments
    '''ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--prototxt", required=True,
    	help="path to Caffe 'deploy' prototxt file")
    ap.add_argument("-m", "--model", required=True,
    	help="path to Caffe pre-trained model")
    ap.add_argument("-c", "--confidence", type=float, default=0.2,
    	help="minimum probability to filter weak detections")
    args = vars(ap.parse_args())'''
    
    # initialize the list of class labels MobileNet SSD was trained to
    # detect, then generate a set of bounding box colors for each class
    CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
    	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    	"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
    	"sofa", "train", "tvmonitor"]
    COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
    
    # load our serialized model from disk
    print("[INFO] loading model...")
    net = cv2.dnn.readNetFromCaffe('include/real_time_object_detection_deploy.prototxt.txt', 'include/real_time_object_detection_model.caffemodel')
    
    # initialize the video stream, allow the cammera sensor to warmup,
    # and initialize the FPS counter
    print("[INFO] starting video stream...")
    vs = VideoStream(gstreamer_pipeline(flip_method=0)).start()
    time.sleep(2.0)
    fps = FPS().start()
    # Parse arguments.
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
    #video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0))

    # Get frame rate.
    #fps = video.get(cv2.cv.CV_CAP_PROP_FPS)
    rate = rospy.Rate(30)

    # Loop through video frames.
    while not rospy.is_shutdown():
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	frame = vs.read()
	frame = imutils.resize(frame, width=400)

	# grab the frame dimensions and convert it to a blob
	(h, w) = frame.shape[:2]
	blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
		0.007843, (300, 300), 127.5)

	# pass the blob through the network and obtain the detections and
	# predictions
	net.setInput(blob)
	detections = net.forward()

	# loop over the detections
	for i in np.arange(0, detections.shape[2]):
		# extract the confidence (i.e., probability) associated with
		# the prediction
		confidence = detections[0, 0, i, 2]

		# filter out weak detections by ensuring the `confidence` is
		# greater than the minimum confidence
		if confidence > 0.2:
			# extract the index of the class label from the
			# `detections`, then compute the (x, y)-coordinates of
			# the bounding box for the object
			idx = int(detections[0, 0, i, 1])
			box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
			(startX, startY, endX, endY) = box.astype("int")

			# draw the prediction on the frame
			label = "{}: {:.2f}%".format(CLASSES[idx],
				confidence * 100)
			cv2.rectangle(frame, (startX, startY), (endX, endY),
				COLORS[idx], 2)
			y = startY - 15 if startY - 15 > 15 else startY + 15
			cv2.putText(frame, label, (startX, y),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

	# show the output frame
	#cv2.imshow("Frame", frame)
	#key = cv2.waitKey(1) & 0xFF

	# update the FPS counter
	fps.update()
        try:
            # Publish image.
            img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
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


    # stop the timer and display FPS information
    fps.stop()
    print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

    # do a bit of cleanup
    cv2.destroyAllWindows()
    vs.stop()

    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

