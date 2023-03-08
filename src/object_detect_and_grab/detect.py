# Copyright 2021 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Main script to run the object detection routine."""
import argparse
import sys
import time

import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
def set_servo_angle(step1, step2, step3, step4, step5):
    Pstep1 = 120 + (180 - step1)* 430 / 180
    Pstep2 = 100 + step2 * 450 / 180
    Pstep3 = 120 + (180 - step3) * 480 / 180
    Pstep4 = 130 + step4 * 470 / 180
    Pstep5 = 400 - step5 * 100
    pwm.set_pwm(0, 0, int(Pstep1))
    pwm.set_pwm(1, 0, int(Pstep2))
    pwm.set_pwm(2, 0, int(Pstep3))
    pwm.set_pwm(3, 0, int(Pstep4))
    pwm.set_pwm(4, 0, int(Pstep5))

def run(model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool) -> None:
  """Continuously run inference on images acquired from the camera.

  Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
    num_threads: The number of CPU threads to run the model.
    enable_edgetpu: True/False whether the model is a EdgeTPU model.
  """

  # Variables to calculate FPS
  counter, fps = 0, 0
  start_time = time.time()

  # Start capturing video input from the camera
  cap = cv2.VideoCapture(camera_id)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
  out = cv2.VideoWriter('robot.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 4, (width,height))

  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 255)  # red
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10

  # Initialize the object detection model
  base_options = core.BaseOptions(
      file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
  detection_options = processor.DetectionOptions(
      max_results=3, score_threshold=0.3)
  options = vision.ObjectDetectorOptions(
      base_options=base_options, detection_options=detection_options)
  detector = vision.ObjectDetector.create_from_options(options)
  is_grab = False
  # Continuously capture images from the camera and run inference
  A1 =70
  A2 = 160
  A3 = 90
  A1_pre = 70
  A2_pre = 160
  A3_pre = 90
  set_servo_angle(70,160,90,70,0)
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      sys.exit(
          'ERROR: Unable to read from webcam. Please verify your webcam settings.'
      )

    counter += 1
    # image = cv2.flip(image, 1)

    # Convert the image from BGR to RGB as required by the TFLite model.
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Create a TensorImage object from the RGB image.
    input_tensor = vision.TensorImage.create_from_array(rgb_image)

    # Run object detection estimation using the model.
    detection_result = detector.detect(input_tensor)

    # Draw keypoints and edges on input image
    image, success, A1, A2, A3 = utils.visualize(image, detection_result)

    # Calculate the FPS
    if counter % fps_avg_frame_count == 0:
      end_time = time.time()
      fps = fps_avg_frame_count / (end_time - start_time)
      start_time = time.time()

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    # Stop the program if the ESC key is pressed.
    key = cv2.waitKey(1)
    if key == 27:
      break
    elif key == ord('g'):
      is_grab = True
    if success:
      for i in range(1,10):
        set_servo_angle(int(A1_pre + (A1-A1_pre)/10*i), int(A2_pre + (A2-A2_pre)/10*i), 90, int(A3_pre + (A3-A3_pre)/10*i), 0)
        time.sleep(0.03)
      if is_grab:
        set_servo_angle(int(A1),int(A2),90,int(A3),1)
        break
      else:
        set_servo_angle(int(A1),int(A2),90,int(A3),0)
        A1_pre = A1
        A2_pre = A2
        A3_pre = A3
    out.write(image)
    image = cv2.resize(image, (320,240), interpolation = cv2.INTER_AREA)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imshow('object_detector', image)
    # time.sleep(0.3)
  out.release()
  cap.release()
  cv2.destroyAllWindows()
  time.sleep(1)
  set_servo_angle(70,160,90,70,1)


def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      default='bottle_new.tflite')
  parser.add_argument(
      '--cameraId', help='Id of camera.', required=False, type=int, default=1)
  parser.add_argument(
      '--frameWidth',
      help='Width of frame to capture from camera.',
      required=False,
      type=int,
      default=640)
  parser.add_argument(
      '--frameHeight',
      help='Height of frame to capture from camera.',
      required=False,
      type=int,
      default=480)
  parser.add_argument(
      '--numThreads',
      help='Number of CPU threads to run the model.',
      required=False,
      type=int,
      default=3)
  parser.add_argument(
      '--enableEdgeTPU',
      help='Whether to run the model on EdgeTPU.',
      action='store_true',
      required=False,
      default=False)
  args = parser.parse_args()

  run(args.model, int(args.cameraId), args.frameWidth, args.frameHeight,
      int(args.numThreads), bool(args.enableEdgeTPU))


if __name__ == '__main__':
  main()
