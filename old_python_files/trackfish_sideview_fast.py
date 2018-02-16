#opens up a webcam feed so you can then test your classifer in real time
from numpy import *
import cv2

#fourcc = cv2.cv.CV_FOURCC('m','p','4','v')#MAY NEED TO BE CHANGED FOR WINDOWS

record_vid = False
fps = 20




def detect(img):
    #print cascade
    #rejectLevels??
    #maxSize=(200,100),
    # rects = cascade.detectMultiScale(img, scaleFactor=1.6, minNeighbors=24,  minSize=(20,20),maxSize=(200,100),flags=cv2.CASCADE_SCALE_IMAGE)
    rects = cascade.detectMultiScale(img, scaleFactor=1.5, minNeighbors=15,  minSize=(5,5),maxSize=(150,100),flags=cv2.CASCADE_SCALE_IMAGE)

    #rects,weights = cv2.groupRectangles(list(rects),1)
    #print list(rects)

    if len(rects) == 0:
        return [], img
    rects2=cleanRects(rects)
    rects2[:, 2:] += rects2[:, :2]

    return rects2, img

def cleanRects(rects):
    #gets rid of any rects that are fully contained within another
    rects = array(rects)
    rectsout=rects.copy()
    badrects = array([],dtype=int32)
    for rectnum in range(0,len(rects[:,0])):
        #what rect are we looking at?
        rect_tlx,rect_tly,rect_brx,rect_bry = rects[rectnum,0],rects[rectnum,1],rects[rectnum,0]+rects[rectnum,2],rects[rectnum,1]+rects[rectnum,3]
        #now see if any others are contained within it
        for testnum in range(0,len(rects[:,0])):
            testrect_tlx,testrect_tly,testrect_brx,testrect_bry = rects[testnum,0],rects[testnum,1],rects[testnum,0]+rects[testnum,2],rects[testnum,1]+rects[testnum,3]
            if ((rect_tlx-testrect_tlx)<0 and (rect_tly-testrect_tly)<0):
                #this means that the TL corner is inside the rect
                if ((rect_brx-testrect_brx)>0 and (rect_bry-testrect_bry)>0):
                    #this means that testrect is fully enclosed in rect, so delete it
                    badrects = append(badrects,testnum)
                    print "found bad rect at index "+str(testnum)+" of "+str(len(rects[:,0]))
    rectsout=delete(rectsout,badrects,0)

                    #if testrect

    
    return rectsout

def box(rects, img):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
    #cv2.imwrite('one.jpg', img);

def boxBW(rects, img):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), ( 255,255,255), 2)
    #cv2.imwrite('one.jpg', img);

fname  = 'sideview_short.mp4'

# cap = cv2.VideoCapture(fname)
cap = cv2.VideoCapture(0)
cascade = cv2.CascadeClassifier('fish_sideview_1.xml')
ret, img = cap.read()
img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
imscale = .5
img = cv2.resize(img,None,fx=imscale, fy=imscale, interpolation = cv2.INTER_CUBIC)
print img.shape


#print frame
width,height= img.shape

if record_vid==True:
    writer = cv2.VideoWriter('trackfish_output_sideview_fast.mp4', fourcc, fps,(height,width),True)
    #cv2.VideoWriter(outputPath, fourcc, fps,
# (self.frames[0].shape[1], self.frames[0].shape[0]), True)

while ret is not 0:
    ret, img = cap.read()
    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img = cv2.resize(img,None,fx=imscale, fy=imscale, interpolation = cv2.INTER_CUBIC)
    #w,h,d = img.shape
    if img is not None:
        #imgbw = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        rects, img = detect(img)
        boxBW(rects, img)
        cv2.imshow("frame", img)
        if record_vid is True:
            writer.write(cv2.cvtColor(img,cv2.COLOR_GRAY2RGB))
        if(cv2.waitKey(1) & 0xFF == ord('q')):
            break
    else:
        break
writer.release()