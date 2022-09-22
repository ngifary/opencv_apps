#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Yuki Furuta.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Kei Okada nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from __future__ import print_function

try:
    input = raw_input
except:
    pass

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from opencv_apps.msg import FaceArrayStamped
from opencv_apps.srv import FaceRecognitionTrain


class FaceRecognitionTrainer(Node):

    def __init__(self):
        super().__init__('face_recognition_trainer')
        self.queue_size = rclpy.get_param("~queue_size", 100)

        self.img_sub = message_filters.Subscriber("image", Image)
        self.face_sub = message_filters.Subscriber("faces", FaceArrayStamped)

        self.srv = self.create_service(
            FaceRecognitionTrain, 'face_recognition_train', self.face_recognition_train_callback)        # CHANGE

        # self.req = FaceRecognitionTrainRequest()
        self.label = ""
        self.ok = False

        self.sync = message_filters.TimeSynchronizer([self.img_sub, self.face_sub],
                                                     self.queue_size)
        self.sync.registerCallback(self.callback)

    def face_recognition_train_callback(self, request, response):
        response.sum = request.a + request.b + \
            request.c                                                  # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' %
                               (request.a, request.b, request.c))  # CHANGE

        return response

    def callback(self, img, faces):
        if len(faces.faces) <= 0:
            return
        if self.ok:
            faces.faces.sort(key=lambda f: f.face.width * f.face.height)
            self.req.images.append(img)
            self.req.rects.append(faces.faces[0].face)
            self.req.labels.append(self.label)
            self.ok = False

    def run(self):
        rclpy.wait_for_service("train")
        train = rclpy.ServiceProxy("train", FaceRecognitionTrain)
        self.label = input("Please input your name and press Enter: ")
        while len(self.label) <= 0 or input("Your name is %s. Correct? [y/n]: " % self.label) not in ["", "y", "Y"]:
            self.label = input("Please input your name and press Enter: ")

        input("Please stand at the center of the camera and press Enter: ")
        while True:
            self.ok = True
            while self.ok:
                print("taking picture...")
                rclpy.sleep(1)
            if input("One more picture? [y/n]: ") not in ["", "y", "Y"]:
                break
        print("sending to trainer...")

        res = train(self.req)
        if res.ok:
            print("OK. Trained successfully!")
        else:
            print("NG. Error: %s" % res.error)


def main(args=None):
    rclpy.init(args=args)

    face_recognition_trainer = FaceRecognitionTrainer()

    rclpy.spin(face_recognition_trainer)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

# if __name__ == '__main__':
#     rclpy.init_node("face_recognition_trainer")
#     t = FaceRecognitionTrainer()
#     t.run()
