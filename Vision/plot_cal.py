import matplotlib.pyplot as plt
import numpy as np




# def circle_find(dat, color="red", x0=0, y0=0):
#     R = []
#     ys = []
#     for i in range(dat.shape[0]):
#         x, y, r = dat[i]
#         x -=x0
#         y -= y0
#         R.append([x*x, 2*x*y, y*y, -2*x, -2*y])
#         ys.append(1)
#     R = np.array(R)
#     y = np.array(ys)
#     x = np.linalg.solve(R.T@R, R.T@y)
#     Q = np.array([[x[0], x[1]],[x[1], x[2]]])
#     print(Q)
#     X0 = np.linalg.solve(Q, np.array([x[3], x[4]]))
#     print(Q, X0)

#     w, v = np.linalg.eigh(Q)
#     assert(np.linalg.norm(Q-v@np.diagflat(w)@v.T)<1e-7*np.linalg.norm(Q))
#     assert(np.linalg.norm(Q-v.T@np.diagflat(w)@v)<1e-7*np.linalg.norm(Q))

#     invsqrtQ = v.T@np.diagflat([1.0/np.sqrt(wi) for wi in w])@v
#     try:
#         assert(np.linalg.norm(np.eye(2)-invsqrtQ.T@Q@invsqrtQ)<1e-7)
#     except AssertionError:
#         return(None, None)
#     thetas = np.linspace(-np.pi, np.pi, 1000)
#     ellips_xs = np.array([[x0],[y0]])+invsqrtQ @ np.array([np.cos(thetas), np.sin(thetas)])
#     plt.plot(dat[:,0], dat[:, 1], '.', color=color)
#     print(np.mean(dat[:,2]))
#     plt.plot(ellips_xs[0,:],ellips_xs[1,:], color=color)

#     thetas = np.linspace(-np.pi, np.pi, 9)
#     ellips_xs = np.array([[x0],[y0]])+invsqrtQ @ np.array([np.cos(thetas), np.sin(thetas)])
#     for x, y in ellips_xs.T:
#         plt.plot([x0, x], [y0, y], color=color)

#     plt.axis('equal')
#     return (x0+X0[0], y0+X0[1])


# def circle_find_fixed_center(dat, color="red", x0=0, y0=0):
#     R = []
#     ys = []
#     for i in range(dat.shape[0]):
#         x, y, r = dat[i]
#         x -=x0
#         y -= y0
#         R.append([x*x, 2*x*y, y*y])
#         ys.append(1)
#     R = np.array(R)
#     y = np.array(ys)
#     x = np.linalg.solve(R.T@R, R.T@y)
#     Q = np.array([[x[0], x[1]],[x[1], x[2]]])
#     print(Q)

#     w, v = np.linalg.eigh(Q)
#     assert(np.linalg.norm(Q-v@np.diagflat(w)@v.T)<1e-7*np.linalg.norm(Q))
#     assert(np.linalg.norm(Q-v.T@np.diagflat(w)@v)<1e-7*np.linalg.norm(Q))

#     invsqrtQ = v.T@np.diagflat([1.0/np.sqrt(wi) for wi in w])@v
#     try:
#         assert(np.linalg.norm(np.eye(2)-invsqrtQ.T@Q@invsqrtQ)<1e-7)
#     except AssertionError:
#         return(None, None)
#     thetas = np.linspace(-np.pi, np.pi, 1000)
#     ellips_xs = np.array([[x0],[y0]])+invsqrtQ @ np.array([np.cos(thetas), np.sin(thetas)])
#     plt.plot(dat[:,0], dat[:, 1], '.', color=color)
#     print(np.mean(dat[:,2]))
#     plt.plot(ellips_xs[0,:],ellips_xs[1,:], color=color)


#     thetas = np.linspace(-np.pi, np.pi, 9)
#     ellips_xs = np.array([[x0],[y0]])+invsqrtQ @ np.array([np.cos(thetas), np.sin(thetas)])
#     for x, y in ellips_xs.T:
#         plt.plot([x0, x], [y0, y], color=color)


def circle_find_3_points(dat, color="red", x0=0, y0=0, point_spacing_deg=45, thresh=20):
    X0s = []
    Ys = []
    constellation = np.array([ [1,0], [np.sqrt(2)/2, np.sqrt(2)/2], [0, 1] ]).T
    # dat = dat[dat[:, 0].argsort()]
    dat_space = dat.shape[0]//3
    good_frames = 0
    R = np.block([
    # origin              # A 11              # A12               #A21 # A22
            [np.eye(2), np.array([
                [constellation[0,i], constellation[1,i],0,0], 
                [ 0 ,0 , constellation[0,i], constellation[1,i]]])] for i in range(3)])

    last_frame_ndx = 0
    start_from_ndx = 0
    frame_number = 0
    frame_number_old = -999
    while True:
        use_frame=False


        while not use_frame:
            if start_from_ndx>len(dat)-3:
                break
            frame_number = dat[start_from_ndx,3]
            if frame_number_old != frame_number:
                last_frame_ndx = start_from_ndx + 2
                frame_check = dat[last_frame_ndx,3]
                if frame_check == frame_number:
                    # print('FOUND A GOOD ONE! Frame: ',start_from_ndx,last_frame_ndx)
                    use_frame=True
            frame_number_old = frame_number
            start_from_ndx+=1
        if start_from_ndx>len(dat)-3:
            break

        if use_frame:
            XY = dat[start_from_ndx-1:last_frame_ndx+1,:2]
            XY = np.array(sorted(XY, key=lambda xy: xy[0], reverse=True))

        x1, y1 = XY[0,:]
        x2, y2 = XY[1,:]
        x3, y3 = XY[2,:]

        frame_is_good = abs(x1-x2)>thresh and abs(x2-x3)>thresh and abs(x1-x3)>thresh
        if not frame_is_good:
            # print('frame %d is questionable, so we skip it.'%i)
            continue
        else:
            good_frames += 1
            points = [[x1, y1], [x2, y2], [x3, y3]]
            points = sorted(points)
            xy = np.array(points)
            # points.append(points[0])
            plt.plot([x for x,y in points], [y for x,y in points])

            # xy.T~6 = [haverQ @ rot(theta)]~4 @ base_constellation.T + origin~2

            Y = np.array([[x1, y1, x2, y2, x3, y3]]).T
            Ys.append(Y)
            

            # print('shapes:',R.shape, Y.shape)
            # print(R, Y)
            # x0, y0, a11, a12, a21, a22 = np.linalg.solve(R.T@R, R.T@Y)[:,0]
            x0, y0, a11, a12, a21, a22 = np.linalg.solve(R, Y)[:,0]
            plt.plot(x0,y0,'.')
            # print(a11, a12, a21, a22)
            X0s.append([x0, y0])
    x0, y0 = [sum([x for x, y in X0s])/len(X0s), sum([y for x, y in X0s])/len(X0s)]
    # print('x0,y0: ', x0, y0)
    print('frames used: ',good_frames)
    USSU = []
    for Y in Ys:
        dY = Y - np.array([[x0, y0, x0, y0, x0, y0]]).T
        x1, y1, x2, y2, x3, y3 = dY[:,0]
        X1 = np.array([[x1, y1]]).T
        X3 = np.array([[x3, y3]]).T
        USSU.append(X1@ X1.T + X3@ X3.T)
    USSU = sum(USSU)/len(USSU)
    ss, U = np.linalg.eigh(USSU)
    assert(np.linalg.norm(U@np.diagflat(ss)@U.T - USSU)<1e-7*np.linalg.norm(USSU))
    S = np.diagflat([np.sqrt(ss) for ss in ss])
    SinvUT = np.linalg.solve(S, U.T)
    # print(USSU, S, U)
    # print(np.arctan2.__doc__)
    squared_errors=0
    squared_errors_N = 0
    for Y in Ys:
        angles = [0,0,0]
        for i in range(3):
            X = Y[2*i:2*(i+1),[0]] - np.array([[x0, y0]]).T
            (x,),(y,) = SinvUT @ X
            angles[i] = np.arctan2(y, x)+np.pi/4*i
            if angles[i]<-np.pi:
                angles[i]+=np.pi*2
            if angles[i]>np.pi:
                angles[i]-=np.pi*2
        
        angles -= angles[0]
        for i in range(3):
            if angles[i]<-np.pi:
                angles[i]+=np.pi*2
            if angles[i]>np.pi:
                angles[i]-=np.pi*2
        squared_errors += np.linalg.norm(180/np.pi*np.array(angles-sum(angles)/3))**2
        squared_errors_N += 2
        if np.linalg.norm(180/np.pi*np.array(angles-sum(angles)/3))**2 > 100:
            print(Y.T, np.array(angles)*180/np.pi, "avg error (deg):", 180/np.pi*np.array(angles-sum(angles)/3))
    RMSE = np.sqrt(squared_errors/squared_errors_N)
    print(" RMSE deg:", RMSE)
        # exit()


    plt.axis('equal')
    return (x0, y0, SinvUT)

if __name__ == '__main__':

    datb = np.loadtxt('blue_cal.csv', delimiter=',') # 1794-10342
    # x0 = 291.5# 291.4486709583431# 291.0794918315378# 282
    # y0 = -110#-187.60640720520388#-186.3370971343926# -176.0636270208106# -50
    # x0, y0 = 283.20392874995474, -114.07516573272562
    x0, y0, SinvUT =circle_find_3_points(datb, color='blue')
    thetas = np.linspace(0,np.pi*2,1000)
    circ = np.linalg.solve(SinvUT, np.block([[np.cos(thetas)],[np.sin(thetas)]]))+np.array([[x0],[y0]])
    print("blue cal (x0,y0,SinvUT):", x0, y0, SinvUT)
    plt.plot(circ[0,:], circ[1,:],'b')


    datr = np.loadtxt('red_cal.csv', delimiter=',')
    # x0 =  291.5
    # y0 = -115
    # print(circle_find_fixed_center(datr, color='red', x0=x0, y0=y0))
    x0, y0, SinvUT =circle_find_3_points(datr, color='red')
    print("red cal (x0,y0,SinvUT):", x0, y0, SinvUT)
    thetas = np.linspace(0,np.pi*2,1000)
    circ = np.linalg.solve(SinvUT, np.block([[np.cos(thetas)],[np.sin(thetas)]]))+np.array([[x0],[y0]])
    plt.plot(circ[0,:], circ[1,:],'r')

    #New: (280 -109) (283, -114)
    #Old: (280, -109) (283, -114)
    # equation for an ellipse: [x-x0; y-y0]^T Q [x-x0; y-y0] = 1
    # (X - X0)^T Q (X-X0) = 1
    # X^T Q (X-X0) - X0^T Q (X-X0)  = 1
    # X^T Q X - X^T Q X0 - X0^T Q X = 1 -X0^T Q X0 



    plt.show()