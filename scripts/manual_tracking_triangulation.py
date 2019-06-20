#text file format: self.f.write(str(now)+'\t'+str(data.header.seq)+'\t'+str(self.manx)+'\t'+str(self.many)+'\r\n')
#so the text file has u and v coordinates in that order (not row, col)


from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D
from numpy import *
import cv2

fname1 = "data/twofish_2019-01-31-12-26-06_7_output_fish1overhead.txt"
fname2 = "data/twofish_2019-01-31-12-26-06_7_output_fish1side.txt"
fname3 = "data/twofish_2019-01-31-12-26-06_7_output_fish2overhead.txt"
fname4 = "data/twofish_2019-01-31-12-26-06_7_output_fish2side.txt"


# top view projection:
P1 = array([[  1.40026115e-13,  -1.10332000e+03,  -5.91740000e+02,   5.59180600e+02],
     [ -1.10868000e+03,  -6.78870707e-14,  -3.13000000e+02,   4.36948400e+02],
    [  1.22464680e-16,   7.49879891e-33,  -1.00000000e+00,   5.00000000e-02]])
# print P1

# side view projection:
P2 =  array([[  6.34540000e+02,  -1.01849000e+03,   3.88543690e-14,  -5.18871100e+02],
    [  3.82020000e+02,   2.33919785e-14,  -1.01805000e+03,   2.15948700e+02],
    [  1.00000000e+00,   6.12323400e-17,   6.12323400e-17,  -1.54000000e+00]])
# print P2


#load the four files
f1top = loadtxt(fname1)
f1side = loadtxt(fname2)

f2top = loadtxt(fname3)
f2side = loadtxt(fname4)

#chop the files so that all start at the same seq value (to align the video)
minseqs = array([f1top[0,1], f1side[0,1], f2top[0,1], f2side[0,1]])
seqstart = max(minseqs)

f1top = f1top[f1top[:,1]>=seqstart,:]
f2top = f2top[f2top[:,1]>=seqstart,:]
f1side = f1side[f1side[:,1]>=seqstart,:]
f2side = f2side[f2side[:,1]>=seqstart,:]

#now chop the files so they're all the same length
lengths = array([len(f1top),len(f1side),len(f2top),len(f2side)])
minlength = min(lengths)

f1top = f1top[0:minlength,:]
f2top = f2top[0:minlength,:]
f1side = f1side[0:minlength,:]
f2side = f2side[0:minlength,:]

#now the move is to triangulate each fish

def triangulate(P1,P2,uv1,uv2):
    x1 = vstack(uv1)
    x2 = vstack(uv2)
    fishpos = cv2.triangulatePoints(P1, P2, x1, x2)
    fishpos/=fishpos[3,:]
    return fishpos

def createOutputFile(top,side):
    out = hstack((top,side))
    out = hstack((out,zeros((len(out),3))))
    for k in range(0,len(out)):
        fishxyz = triangulate(P1,P2,top[k,2:],side[k,2:])
        # fishxyz = zeros((4,1))-fishxyz
        out[k,8:]= -fishxyz[0:3,0]
    return out

f1out = createOutputFile(f1top,f1side)
f2out = createOutputFile(f2top,f2side)

savetxt('fish13d.txt',f1out)
savetxt('fish23d.txt',f2out)

f=figure()
ax = f.add_subplot(111,projection='3d')
ax.scatter(f1out[:,8],f1out[:,9],f1out[:,10],'k')
ax.scatter(f2out[:,8],f2out[:,9],f2out[:,10],'r')
legend(['fish 1','fish 2'])
show()