import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Image_converter:
    def __init__(self, callback_fcn):
        """
        Hace conversiones entre imágenes cv2 y las imágenes que 
        utiliza ROS.

        Parameters
        ----------
        callback_fcn : function
            Función que se ejecutará cada que se publique una nueva imagen
        """
        self.image_pub = rospy.Publisher("/camera_image", Image, queue_size=10)

        self.bridge = CvBridge()

        if callback_fcn is not None:
            self.image_sub = rospy.Subscriber("image_topic",Image, callback_fcn)

    def publish_image(self, cv2_image):
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv2_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def rosimg_to_cv2(self, rosimg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rosimg, "bgr8")
        except CvBridgeError as e:
            print(e)
        return cv_image
    """
    Función de ejemplo para un callback externo
    """
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)