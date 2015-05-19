import numpy as np
import cv2
from detectBlob import DetectBlob

cap = cv2.VideoCapture(0)
d = DetectBlob(np.array([160, 50, 50], np.uint8), np.array([255, 255, 255], np.uint8))
    

while(True):
    # Capture frame-by-frame
    ret, img = cap.read()
    keypoints = d.getBlobs(img)
    for item in keypoints :
        print("\tx %d , y %d, d: %d" % (item.pt[0],item.pt[1], item.size))
        cv2.circle(img, (int(item.pt[0]),int(item.pt[1])), int(item.size), (0,255,100),5)
    print("")

    #frame = cv2.circle(frame, (100,100), 5, (0,255,100),5)
    

    # Our operations on the frame come here

    # Display the resulting frame
    cv2.imshow('frame',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
