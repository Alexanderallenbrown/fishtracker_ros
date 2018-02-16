#opens up a webcam feed so you can then test your classifer in real time
#using detectMultiScale
import numpy
import cv2

fourcc = cv2.cv.CV_FOURCC('m','p','4','v')#MAY NEED TO BE CHANGED FOR WINDOWS

record_vid = True
fps = 20



fname  = 'topview_test_short.mp4'

cap = cv2.VideoCapture(fname)
ret,frame=cap.read()
print frame.shape
width,height,depth = frame.shape

if record_vid==True:
    writer = cv2.VideoWriter('trackfish_output.mp4', fourcc, fps,(height,width),True)
    #cv2.VideoWriter(outputPath, fourcc, fps,
# (self.frames[0].shape[1], self.frames[0].shape[0]), True)

def detect(img):
    cascade = cv2.CascadeClassifier('fish_topview_2.xml')
    #print cascade
    #rejectLevels??
    #maxSize=(200,100),
    # rects = cascade.detectMultiScale(img, scaleFactor=1.6, minNeighbors=24,  minSize=(20,20),maxSize=(200,100),flags=cv2.CASCADE_SCALE_IMAGE)
    rects = cascade.detectMultiScale(img, scaleFactor=1.65, minNeighbors=20,  minSize=(20,20),maxSize=(200,100),flags=cv2.CASCADE_SCALE_IMAGE)

    #rects,weights = cv2.groupRectangles(list(rects),1)
    #print list(rects)

    if len(rects) == 0:
        return [], img
    rects[:, 2:] += rects[:, :2]

    return rects, img

def box(rects, img):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
    #cv2.imwrite('one.jpg', img);



while(True):
    ret, img = cap.read()
    imgbw = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    rects, img = detect(img)
    box(rects, img)
    cv2.imshow("frame", img)
    if record_vid is True:
        writer.write(img)
    if(cv2.waitKey(1) & 0xFF == ord('q')):
        break
writer.release()