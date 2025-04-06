
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt


class CircleTracker():
    num_trackers=0
    def __init__(self, a, b, r, manhattan_threshold=10, radius_threshold=5,
            disp_color=(0, 255, 0), frame_drop_threshold=30, tracker_update_padding=15,
            cal=None):
        self.a = a
        self.b = b
        self.r = r
        self.legitimacy = 1.0
        self.manhattan_threshold = manhattan_threshold
        self.radius_threshold = radius_threshold
        self.disp_color = disp_color
        self.last_ndx = None
        self.frame_drop_threshold = frame_drop_threshold
        self.tracker_index = CircleTracker.num_trackers
        self.tracker_update_padding = tracker_update_padding
        CircleTracker.num_trackers+=1
        self.cal = cal

    def explains(self, a,b,r):
        # uses the manhattan distance and the radius to determine if the detected circle is explained by the existing model.
        return abs(self.a-a)+abs(self.b-b)<=self.manhattan_threshold and abs(self.r-r)<=self.radius_threshold

    def check_legit(self, ndx, frame, gray):
        """ determines if the image and history justifies maintaining the tracker. 
        Updates the estimate if the circle was missed by the Hough transform  
        ndx - global time indexing, used for detecting missed frames
        """
        if self.last_ndx is None:
            self.last_ndx = ndx
        else:
            if ndx-self.last_ndx > self.frame_drop_threshold:
                self.legitimacy = 0.0

        # check bounding box against frameshape
        xmax, ymax = gray.shape
        FOS=2.0

        # if self.b>xmax-self.r*FOS or self.b<self.r*FOS or self.a>ymax-self.r*FOS or self.a<self.r*FOS:
        #     print(self.a, self.b, xmax, ymax, self.r)
        #     print('caught an out of bounds error %d:%d, %d:%d'%(int(b-r),int(b+r),int(a-r),int(a+r)))
        #     cv2.imshow("zero division:Tracker_zone%d"%self.tracker_index,self.myimg)
        #     cv2.imshow('zero division, gray', gray)
        #     while not cv2.waitKey(25) & 0xFF == ord('q'):
        #         print('caught a zero division %d:%d, %d:%d'%(int(b-r),int(b+r),int(a-r),int(a+r)))
        #         pass
        #     raise(e)
        #     self.legitimacy = 0.0
        #     return False

        if ndx > self.last_ndx:
            # if we've missed updates, try to update based on priors.
            self.update(self.last_ndx, self.a, self.b, self.r, frame, gray)
        return self.legitimacy>0.0


    def update(self, ndx, a, b, r, frame, gray):
        """ updates the tracker using the frame and the grayscale frame
        ndx - global time indexing, used for detecting missed frames
        a, b, r - circle definition from Hough transform
    
        """
        self.last_ndx = ndx
        r += self.tracker_update_padding

        # print(gray.shape,"%d:%d, %d:%d"% ( int(b-r),int(b+r), int(a-r),int(a+r)))
        self.myimg = gray[ int(b-r):int(b+r),int(a-r):int(a+r)]
        # calculate moments of binary image

        M = cv2.moments(self.myimg)

        if M['m00']<320000:
            # acknowledge loss of tracking
            print("loss of tracking for tracker %d"%self.tracker_index)
            self.legitimacy=0.0
            return

        try:
            # calculate x,y coordinate of center
            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]
        except ZeroDivisionError as e:
            print('caught a zero division %d:%d, %d:%d'%(int(b-r),int(b+r),int(a-r),int(a+r)))
            cv2.imshow("zero division:Tracker_zone%d"%self.tracker_index,self.myimg)
            cv2.imshow('zero division, gray', gray)
            while not cv2.waitKey(25) & 0xFF == ord('q'):
                print('caught a zero division %d:%d, %d:%d'%(int(b-r),int(b+r),int(a-r),int(a+r)))
                pass
            raise(e)
            exit()

        cv2.circle(self.myimg,( int(cX), int(cY)), 5, (0,0,0), -1)
        self.a = int(a-r)+cX
        self.b = int(b-r)+cY
        if not self.cal is None:
            x0, SinvUT = self.cal
            xi = SinvUT@(np.array([[self.a,self.b]]).T-x0)
            self.theta = np.arctan2(-xi[0,0],xi[1,0])
            # vec = np.array([self.a, self.b]) - x0.T
            # self.theta = np.arctan2(vec[0,1],vec[0,0])
            # print('theta, theta2: ',self.theta*180/np.pi,theta2*180/np.pi)

        cv2.imshow("Tracker_zone",self.myimg)


    def display(self, img):
        pt = np.uint16(np.around(np.array([self.a, self.b, self.r])))
        a, b, r = pt[0], pt[1], pt[2]
  
        # Draw the circumference of the circle.
        cv2.circle(img, (a, b), r, self.disp_color, 2)
  
        # Draw a small circle (of radius 1) to show the center.
        cv2.circle(img, (a, b), 1, self.disp_color, 3)
        cv2.putText(img, "%d:%.2f deg"%(self.tracker_index, 180/np.pi*self.theta), (a+r, b-r), cv2.FONT_HERSHEY_SIMPLEX, .5, self.disp_color, 1, cv2.LINE_AA)
        # cv2.imshow("CircleTrackers", img)


class CircleAnnotator():
    def __init__(self, x0=283, y0=-110, 
            SinvUT=np.array([[-0.00391323,-0.00050613],[-0.00049765,0.0038477]]),
            calibration_file_name=None):
        self.circle_trackers = []
        self.ndx = 0
        self.calibration_file_name = calibration_file_name
        self.cal_data = []
        print(SinvUT)
        ϕ = np.arctan2(SinvUT[0,1], SinvUT[1,1])
        Q = np.array([[np.cos(ϕ), -np.sin(ϕ)], [np.sin(ϕ), np.cos(ϕ)]])
        SinvUT = Q@SinvUT
        self.prev_ang = np.pi/8
        # print(SinvUT)
        # exit()
        self.ellipse_def = (np.array([[x0, y0]]).T, SinvUT)
        self.angs_flag = False

    def save_cal(self):
        if not (self.calibration_file_name is None):
            with open(self.calibration_file_name, 'w') as f:
                np.savetxt(f, self.cal_data, fmt='%.3f', delimiter=", ")

    def find_nearest(self,array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return array[idx]

    def find_points(self,circles):
        threshold = 10*np.pi/180

        c1 = circles[-1,-1]
        pc1 = self.find_nearest(array=self.prev_circles[:,-1],value=c1)
        c2 = circles[-2,-1]
        pc2 = self.find_nearest(array=self.prev_circles[:,-1],value=c2)

        if np.abs(c1 - pc1) > threshold:
            c1 = c2
            pc1 = pc2
            c2 = circles[-3,-1]
            pc2 = self.find_nearest(array=self.prev_circles[:,-1],value=c2)
        elif np.abs(c2 - pc2) > threshold:
            c2 = circles[-3,-1]
            pc2 = self.find_nearest(array=self.prev_circles[:,-1],value=c2)

        return c1, c2, pc1, pc2

    def calc_camera_angs(self):
        circles = np.empty((0,3), float)
        for ct in self.circle_trackers:
            circles = np.vstack((circles,np.array([ct.a, ct.b, ct.theta])))
        if self.ndx == 0:
            self.angs_abs = np.array([0.0])
        else:
            circles = circles[circles[:, 1].argsort()] 
            c1, c2, pc1, pc2 = self.find_points(circles)
            del1 = c1 - pc1
            del2 = c2 - pc2
            
            w1 = np.abs(np.pi - c1)
            w2 = np.abs(np.pi - c2)
            shift = w2*del1/(w1 + w2) + w1*del2/(w1 + w2)
            new_ang_abs = shift + self.angs_abs[-1]

            self.angs_abs = np.append(self.angs_abs,new_ang_abs)
            
        self.prev_circles = circles

    def calc_camera_angs(self):
        if len(self.circle_trackers) > 0:
            raw_angs = [ct.theta for ct in self.circle_trackers]
            weights = [max((1 - abs(ct.theta)*(4/np.pi)),0) for ct in self.circle_trackers]
            # print(self.circle_trackers)
            normalization = 1./sum(weights)
            # print(np.array(raw_angs)*180./np.pi)
            # print(weights)
            mod_angles = [np.mod(ct.theta, np.pi/4) for ct in self.circle_trackers]
            # print(np.array(mod_angles)*180./np.pi)
            avg_mod_angle = sum([w*θ for w, θ in zip(weights, mod_angles)])*normalization

            # print(avg_mod_angle*180/np.pi)

            ang = avg_mod_angle
        else:
            ang = self.prev_ang

        if self.ndx == 0:
            self.angs_abs = np.array([ang])
            self.prev_ang = ang
            self.n_rotations = 0
        else:
            if abs(ang-self.prev_ang)>=np.pi/8: # if we are jumping
                if self.prev_ang>ang: # if we jumped from 45 deg to 0 deg
                    self.n_rotations+=1
                else: # the opposite jump type
                    self.n_rotations-=1

            self.prev_ang = ang
            ang = ang + self.n_rotations*np.pi/4
            self.angs_abs = np.append(self.angs_abs,ang)


    def annotate_circles(self, frame, gray, color):
        detected_circles = cv2.HoughCircles(gray, 
                       cv2.HOUGH_GRADIENT, .25, 20, param1 = 60,
                   param2 = 20, minRadius = 18, maxRadius = 21)
        cv2.imshow('gray%d%d%d'%color , gray)
        if detected_circles is not None:
            # print("detected %d circles. Trackers: "%detected_circles.shape[1]+", ".join(["%d"%ct.tracker_index for ct in self.circle_trackers]))
            for pt in detected_circles[0, :]:
                (a, b, r) = pt
                claimed_by_tracker=False
                for ct in self.circle_trackers:

                    if ct.explains(a,b,r):
                        if claimed_by_tracker:
                            ct.legitimacy=0.0 # kill redundant circle trackers
                        else:
                            claimed_by_tracker=True
                            ct.update(self.ndx, a,b,r, frame, gray)
                            # print('updating')

                if not claimed_by_tracker:
                    ct = CircleTracker(a, b, r, disp_color=color, cal=self.ellipse_def)
                    ct.update(self.ndx, a,b,r, frame, gray) # always update once at least
                    print("initializing new tracker", self.ndx, a,b,r)  
                    self.circle_trackers.append(ct)

            # drop any self-declared illigitimate circle trackers
            self.circle_trackers = [ct for ct in self.circle_trackers if ct.check_legit(self.ndx, frame, gray)]

            for ct in self.circle_trackers:
                ct.display(frame)
                if not (self.calibration_file_name is None):
                    self.cal_data.append([ct.a, ct.b, ct.r,self.ndx])

            if self.calibration_file_name is None:
                self.calc_camera_angs()
                self.angs_flag = True

            if False: # old plotting
                # Convert the circle parameters a, b and r to integers.
                detected_circles = np.uint16(np.around(detected_circles))
                # img=gray_blurred
                for pt in detected_circles[0, :]:
                    a, b, r = pt[0], pt[1], pt[2]
                    print(r)
              
                    # Draw the circumference of the circle.
                    cv2.circle(frame, (a, b), r, color, 2)
                    # Draw a small circle (of radius 1) to show the center.
                    cv2.circle(frame, (a, b), 1, color, 3)
        self.ndx+=1

def test(file='first_vido.h264', 
        inner_mask_loc = 'inner_mask5.png',
        outer_mask_loc = 'outer_mask3.png',
        pre_mask_save_loc = "pre_mask.png",
        red_cal_save_loc = None,
        blue_cal_save_loc = None):
    print("filename %s"%file)
    cap = cv2.VideoCapture(file)
    # To generate new callibration data

    if red_cal_save_loc is None or blue_cal_save_loc is None:
        # Read calibration values from file
        circle_cal = np.loadtxt('circle_cal.csv', delimiter=',')

        red_circle_annotator = CircleAnnotator(
            x0= circle_cal[0,0], y0 = circle_cal[0,1],
            SinvUT = np.array([[circle_cal[0,2], circle_cal[0,3]],
                                [circle_cal[0,4],  circle_cal[0,5] ]]))
        blue_circle_annotator = CircleAnnotator(
            x0= circle_cal[1,0], y0 = circle_cal[1,1],
            SinvUT = np.array([[circle_cal[1,2], circle_cal[1,3]],
                                [circle_cal[1,4],  circle_cal[1,5] ]]))
    else:
        red_circle_annotator = CircleAnnotator(calibration_file_name=red_cal_save_loc)
        blue_circle_annotator = CircleAnnotator(calibration_file_name=blue_cal_save_loc)

    

    mask = inner_mask = outer_mask = None
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret ==True:
            frame=frame[:290, 50:590]
            
            if mask is None:
                cv2.imwrite(pre_mask_save_loc, frame)
                inner_mask = cv2.resize(cv2.imread(inner_mask_loc), frame.shape[1::-1])
                outer_mask = cv2.resize(cv2.imread(outer_mask_loc), frame.shape[1::-1])
                # mask = cv2.resize(cv2.imread('testbed_mask.png'), frame.shape[1::-1])
                mask = cv2.bitwise_or(inner_mask, outer_mask)
            frame = cv2.bitwise_and(frame, mask)
            grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray0 = cv2.blur(cv2.threshold(cv2.bitwise_and(grayframe, inner_mask[:,:,0]), 60, 255, cv2.THRESH_BINARY)[1], (5, 5))
            gray2 = cv2.blur(cv2.threshold(cv2.bitwise_and(grayframe, outer_mask[:,:,0]), 60, 255, cv2.THRESH_BINARY)[1], (5, 5))

            # hist = cv2.calcHist([frame[:,:, 2]], [0], None, [256], [0, 256])
            # plt.plot( hist)
            # cv2.imshow('gray', gray2)
            # plt.show()
            # exit()
            blue_circle_annotator.annotate_circles(frame, gray0, (255,0,0))
            red_circle_annotator.annotate_circles(frame, gray2, (0,0,255))


            cv2.imshow('Frame', frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
            # time.sleep(0.1)
        else:
            break


    red_circle_annotator.save_cal()
    blue_circle_annotator.save_cal()

    cap.release()
    cv2.destroyAllWindows()

    if red_circle_annotator.angs_flag or blue_circle_annotator.angs_flag:
        frame_rate = 30
        cam_time = np.linspace(1,red_circle_annotator.ndx,num = red_circle_annotator.ndx)/frame_rate

        return blue_circle_annotator.angs_abs, red_circle_annotator.angs_abs, cam_time
    else:
        return 0

if __name__ == '__main__':
    # test(file='first_vido.h264')
    root_folder = "/home/gray/wk/nonlinear_spring_data/"
    folder = root_folder+"with0springsTake3/"
    test(file=folder+'camera_calibration_spring_test.h264',
        inner_mask_loc = folder+"blue_mask.png", #None, #folder+'inner_mask0826.png',
        outer_mask_loc = folder+"red_mask.png", #None, #folder+'outer_mask0826.png',
        pre_mask_save_loc = folder+"camera_calibration_pre_mask.png",
        red_cal_save_loc = None,# folder+"red_cal.csv",
        blue_cal_save_loc = None)#folder+"blue_cal.csv")

