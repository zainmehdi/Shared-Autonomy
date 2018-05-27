import sys
import os
import rospy
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


# Disable tensorflow compilation warnings
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import tensorflow as tf


def callback( image_msg):
    cv_image = cv_bridge.imgmsg_to_cv2(image_msg, "mono8")
   # cv2.imwrite('camera_image.jpeg',cv_image)

    image_data = cv2.imencode('.jpg', cv_image)[1].tostring()
   # cv2.imshow("Image window", cv_image)

    #image_data = tf.gfile.FastGFile(image_path, 'rb').read()

    # cv2.imwrite(filename="screens/" + str(1) + "alpha.png", img=image_data);  # write frame image to file

   # image_data = tf.gfile.FastGFile("screens/" + str(1) + "alpha.png", 'rb').read()  # get this image file

    softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
    predictions = sess.run(softmax_tensor, \
                           {'DecodeJpeg/contents:0': image_data})
    top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]

    # output
    for node_id in top_k:
        human_string = label_lines[node_id]
        score = predictions[0][node_id]
        print('%s (score = %.5f)' % (human_string, score))
        pub.publish(human_string)


   # cv2.waitKey(3)



if __name__ == '__main__':

    label_lines = [line.rstrip() for line
                   in tf.gfile.GFile("tf_files/retrained_labels.txt")]

    with tf.gfile.FastGFile("tf_files/retrained_graph.pb", 'rb') as f:
        graph_def = tf.GraphDef()  ## The graph-graph_def is a saved copy of a TensorFlow graph; objektinitialisierung
        graph_def.ParseFromString(f.read())  # Parse serialized protocol buffer data into variable
        _ = tf.import_graph_def(graph_def,
                                name='')

    cv_bridge = CvBridge()
    rospy.init_node('listener', anonymous=True)

    sub = rospy.Subscriber('mask', Image, callback, queue_size=1)
    pub = rospy.Publisher('result', String, queue_size=1)

    with tf.Session() as sess:
        rospy.spin()

