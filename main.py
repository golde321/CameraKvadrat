import cv2 as cv
import numpy as np
#import video
import math

if __name__ == '__main__':
    cv.namedWindow("result")
    cap = cv.VideoCapture(0)

    hsv_min = np.array((6, 100, 100), np.uint8)
    hsv_max = np.array((30, 255, 255), np.uint8)
    hsv_min1 = np.array((80, 99, 99), np.uint8)
    hsv_max1 = np.array((110, 255, 255), np.uint8)

    color_blue = (255, 0, 0)
    color_red = (0, 0, 128)
    color_red1 = (0, 100, 100)
    color_blue1 = (155, 100, 100)

    while True:
        flag, img = cap.read()
        img = cv.flip(img,1)
        try:
            hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
            thresh = cv.inRange(hsv, hsv_min, hsv_max)
            contours0, hierarchy = cv.findContours(thresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
            thresh1 = cv.inRange(hsv, hsv_min1, hsv_max1)
            contours10, hierarchy1 = cv.findContours(thresh1.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

            for cnt in contours0:
                rect = cv.minAreaRect(cnt)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                center = (int(rect[0][0]), int(rect[0][1]))
                area = int(rect[1][0]*rect[1][1])

                edge1 = np.int0((box[1][0] - box[0][0],box[1][1] - box[0][1]))
                edge2 = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))

                usedEdge = edge1
                if cv.norm(edge2) > cv.norm(edge1):
                    usedEdge = edge2

                reference = (1,0) # horizontal edge
                angle = 180.0/math.pi * math.acos((reference[0]*usedEdge[0] + reference[1]*usedEdge[1]) / (cv.norm(reference) *cv.norm(usedEdge)))

                if area > 500:
                    cv.drawContours(img,[box],0,color_blue,2)
                    cv.circle(img, center, 5, color_red, 2)
                    cv.putText(img, "%d" % int(angle), (center[0]+20, center[1]-20), cv.FONT_HERSHEY_SIMPLEX, 1, color_red, 2)
            cv.imshow('result', img)
            for cnt1 in contours10:
                rect = cv.minAreaRect(cnt1)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                center = (int(rect[0][0]), int(rect[0][1]))
                area = int(rect[1][0] * rect[1][1])

                edge1 = np.int0((box[1][0] - box[0][0], box[1][1] - box[0][1]))
                edge2 = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))

                usedEdge = edge1
                if cv.norm(edge2) > cv.norm(edge1):
                    usedEdge = edge2

                reference = (1, 0)  # horizontal edge
                angle = 180.0 / math.pi * math.acos(
                    (reference[0] * usedEdge[0] + reference[1] * usedEdge[1]) / (cv.norm(reference) * cv.norm(usedEdge)))

                if area > 500:
                    cv.drawContours(img, [box], 0, color_blue1, 2)
                    cv.circle(img, center, 5, color_red1, 2)
                    cv.putText(img, "%d" % int(angle), (center[0] + 20, center[1] - 20), cv.FONT_HERSHEY_SIMPLEX, 1,
                               color_red1, 2)
            cv.imshow('result', img)
        except:
            cap.release()
            raise
        ch = cv.waitKey(5)
        if ch == 27:
            break

    cap.release()
    cv.destroyAllWindows()