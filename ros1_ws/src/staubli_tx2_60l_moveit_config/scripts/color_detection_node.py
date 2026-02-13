#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import cv2
from staubli_tx2_60l_moveit_config.msg import BBoxes

# trackbar callback fucntion does nothing but required for trackbar
def callback(x):
    pass

def talker():
    pub = rospy.Publisher('/bboxes', BBoxes, queue_size=10)
    rospy.init_node('color_detection_node', anonymous=True)

    cap = cv2.VideoCapture(0)
    cv2.namedWindow('Calibration')

    # management values
    calibration = 1
    detection = 0

    # default values
    x1d = 191
    y1d = 157
    x2d = 567
    y2d = 191
    x3d = 550
    y3d = 419
    x4d = 163
    y4d = 367	

    x1 = x1d
    y1 = y1d
    x2 = x2d
    y2 = y2d
    x3 = x3d
    y3 = y3d
    x4 = x4d
    y4 = y4d

    ilowH = 90
    ihighH = 179
    ilowS = 125
    ihighS = 255
    ilowV = 60
    ihighV = 146

    # create crop trackbars
    cv2.createTrackbar('x1', 'Calibration', x1, 639, callback)
    cv2.createTrackbar('y1', 'Calibration', y1, 479, callback)

    cv2.createTrackbar('x2', 'Calibration', x2, 639, callback)
    cv2.createTrackbar('y2', 'Calibration', y2, 479, callback)

    cv2.createTrackbar('x3', 'Calibration', x3, 639, callback)
    cv2.createTrackbar('y3', 'Calibration', y3, 479, callback)

    cv2.createTrackbar('x4', 'Calibration', x4, 639, callback)
    cv2.createTrackbar('y4', 'Calibration', y4, 479, callback)

    # create hsv trackbars for color change
    cv2.createTrackbar('lowH', 'Calibration', ilowH, 179, callback)
    cv2.createTrackbar('highH', 'Calibration', ihighH, 179, callback)

    cv2.createTrackbar('lowS', 'Calibration', ilowS, 255, callback)
    cv2.createTrackbar('highS', 'Calibration', ihighS, 255, callback)

    cv2.createTrackbar('lowV', 'Calibration', ilowV, 255, callback)
    cv2.createTrackbar('highV', 'Calibration', ihighV, 255, callback)

    while calibration:

        # grab the frame
        ret, frame = cap.read()

        # get trackbar positions
        x1 = cv2.getTrackbarPos('x1', 'Calibration')
        y1 = cv2.getTrackbarPos('y1', 'Calibration')
        x2 = cv2.getTrackbarPos('x2', 'Calibration')
        y2 = cv2.getTrackbarPos('y2', 'Calibration')
        x3 = cv2.getTrackbarPos('x3', 'Calibration')
        y3 = cv2.getTrackbarPos('y3', 'Calibration')
        x4 = cv2.getTrackbarPos('x4', 'Calibration')
        y4 = cv2.getTrackbarPos('y4', 'Calibration')
        ilowH = cv2.getTrackbarPos('lowH', 'Calibration')
        ihighH = cv2.getTrackbarPos('highH', 'Calibration')
        ilowS = cv2.getTrackbarPos('lowS', 'Calibration')
        ihighS = cv2.getTrackbarPos('highS', 'Calibration')
        ilowV = cv2.getTrackbarPos('lowV', 'Calibration')
        ihighV = cv2.getTrackbarPos('highV', 'Calibration')

        # compute in hsv color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([ilowH, ilowS, ilowV])
        higher_hsv = np.array([ihighH, ihighS, ihighV])
        mask = cv2.inRange(hsv_frame, lower_hsv, higher_hsv)
        frame = cv2.bitwise_and(frame, frame, mask=mask)

        # draw polygon
        crop_border = np.array([[[x1, y1], [x2, y2], [x3, y3], [x4, y4]]], np.int32)
        frame_poly = frame.copy()
        cv2.polylines(frame_poly, [crop_border], True, (0,0,255), thickness=1)

        # warp frame
        pts1 = np.float32([[x1,y1],[x2,y2],[x4,y4],[x3,y3]])
        pts2 = np.float32([[0,0],[853,0],[0,480],[853,480]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        mask_warp = cv2.warpPerspective(mask, matrix, (853, 480))

        # show thresholded image & mask
        mask_warp_3ch = cv2.merge((mask_warp,mask_warp,mask_warp))
        result = np.concatenate((frame_poly, mask_warp_3ch), axis=1)
        cv2.imshow('Calibration', result)

        # key input manager
        k = cv2.waitKey(1)
        if k == ord('q'):    # q key to stop
            print('quit')
            break
        elif k == ord('c'):  # c key to confirm calibration
            print('confirmed calibration')
            cv2.destroyWindow('Calibration')
            calibration = 0
            detection = 1
        elif k == ord('r'):  # r key to reset parameters
            print('resetted parameters')
            x1 = cv2.setTrackbarPos('x1', 'Calibration', x1d)
            y1 = cv2.setTrackbarPos('y1', 'Calibration', y1d)
            x2 = cv2.setTrackbarPos('x2', 'Calibration', x2d)
            y2 = cv2.setTrackbarPos('y2', 'Calibration', y2d)
            x3 = cv2.setTrackbarPos('x3', 'Calibration', x3d)
            y3 = cv2.setTrackbarPos('y3', 'Calibration', y3d)
            x4 = cv2.setTrackbarPos('x4', 'Calibration', x4d)
            y4 = cv2.setTrackbarPos('y4', 'Calibration', y4d)    
            ilowH = cv2.setTrackbarPos('lowH', 'Calibration', 0)
            ihighH = cv2.setTrackbarPos('highH', 'Calibration', 255)
            ilowS = cv2.setTrackbarPos('lowS', 'Calibration', 0)
            ihighS = cv2.setTrackbarPos('highS', 'Calibration', 255)
            ilowV = cv2.setTrackbarPos('lowV', 'Calibration', 0)
            ihighV = cv2.setTrackbarPos('highV', 'Calibration', 255)
        elif k != 255:
            print('c: confirm calibration\nr: reset parameters\nq: quit') # else print keys info

    while detection:

        # grab the frame
        ret, frame = cap.read()

        # warp frame
        pts1 = np.float32([[x1,y1],[x2,y2],[x4,y4],[x3,y3]])
        pts2 = np.float32([[0,0],[1000,0],[0,560],[1000,560]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        frame_warp = cv2.warpPerspective(frame, matrix, (1000, 560))

        # compute in hsv color space
        hsv_frame_warp = cv2.cvtColor(frame_warp, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([ilowH, ilowS, ilowV])
        higher_hsv = np.array([ihighH, ihighS, ihighV])
        mask_warp = cv2.inRange(hsv_frame_warp, lower_hsv, higher_hsv)

        # compute and draw the bounding boxes
        contours, hierarchy = cv2.findContours(mask_warp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        list_bboxes = []
        for index, c in enumerate(contours):
            area = cv2.contourArea(c)
            if (hierarchy[0,index,3] == -1 and area > 1000 and len(list_bboxes) < 10):
                cv2.drawContours(frame_warp, contours, index, (0,255,0), 1)
                x,y,w,h = cv2.boundingRect(c)
                lt_corner = (x, y)
                rb_corner = (x + w, y + h)
                list_bboxes.append([lt_corner, rb_corner])
                cv2.rectangle(frame_warp, lt_corner, rb_corner, (255,0,0), 2)

        cv2.imshow('Detection', frame_warp)

        # key input manager
        k = cv2.waitKey(1)
        if k == ord('q'):    # q key to stop
            print('quit')
            break
        elif k == ord('p'):  # p key to publish bboxes
            print('bboxes published: ' + str(len(list_bboxes)))
            msg_to_send = BBoxes()
            for b in list_bboxes:
                msg_to_send.bboxes.append(b[0][0])
                msg_to_send.bboxes.append(b[0][1])
                msg_to_send.bboxes.append(b[1][0])
                msg_to_send.bboxes.append(b[1][1])
            pub.publish(msg_to_send)
        elif k != 255:
            print('p: publish bboxes\nq: quit') # else print keys info

    cv2.destroyAllWindows()
    cap.release()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
