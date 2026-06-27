
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt


FIDUCIAL_THRESHOLD = 135
FIDUCIAL_OPEN_KERNEL = (5, 5)
FIDUCIAL_CLOSE_KERNEL = (13, 13)
FIDUCIAL_MIN_AREA = 700
FIDUCIAL_MAX_AREA = 3600
FIDUCIAL_MIN_RADIUS = 18
FIDUCIAL_MAX_RADIUS = 32
FIDUCIAL_MIN_FILL_RATIO = 0.45
FIDUCIAL_MAX_ASPECT_RATIO = 1.75
FIDUCIAL_NEW_MIN_VISIBLE_FRACTION = 0.95
FIDUCIAL_NEW_EDGE_MARGIN = 2
FIDUCIAL_TRACK_MIN_VISIBLE_FRACTION = 0.90
FIDUCIAL_TRACK_EDGE_MARGIN = -2
FIDUCIAL_STALE_FRAME_LIMIT = 8
FIDUCIAL_DISPLAY_GRACE_FRAMES = 2


def isolate_white_fiducials(grayframe, mask_channel,
        threshold=FIDUCIAL_THRESHOLD,
        open_kernel=FIDUCIAL_OPEN_KERNEL,
        close_kernel=FIDUCIAL_CLOSE_KERNEL,
        min_area=FIDUCIAL_MIN_AREA,
        max_area=FIDUCIAL_MAX_AREA,
        min_radius=FIDUCIAL_MIN_RADIUS,
        max_radius=FIDUCIAL_MAX_RADIUS,
        min_fill_ratio=FIDUCIAL_MIN_FILL_RATIO,
        max_aspect_ratio=FIDUCIAL_MAX_ASPECT_RATIO):
    """Return a clean binary image containing one filled region per fiducial."""
    masked = cv2.bitwise_and(grayframe, mask_channel)
    masked = cv2.medianBlur(masked, 5)
    _, binary = cv2.threshold(masked, threshold, 255, cv2.THRESH_BINARY)

    open_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, open_kernel)
    close_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, close_kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, open_element)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, close_element)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    clean = np.zeros_like(binary)
    for contour in contours:
        circle = _circle_from_blob(
            contour,
            min_area=min_area,
            max_area=max_area,
            min_radius=min_radius,
            max_radius=max_radius,
            min_fill_ratio=min_fill_ratio,
            max_aspect_ratio=max_aspect_ratio,
        )
        if circle is None:
            continue

        cv2.drawContours(clean, [contour], -1, 255, thickness=cv2.FILLED)

    return clean


def _circle_from_blob(contour,
        min_area=FIDUCIAL_MIN_AREA,
        max_area=FIDUCIAL_MAX_AREA,
        min_radius=FIDUCIAL_MIN_RADIUS,
        max_radius=FIDUCIAL_MAX_RADIUS,
        min_fill_ratio=FIDUCIAL_MIN_FILL_RATIO,
        max_aspect_ratio=FIDUCIAL_MAX_ASPECT_RATIO):
    area = cv2.contourArea(contour)
    if area < min_area or area > max_area:
        return None

    (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
    if radius < min_radius or radius > max_radius:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = max(w, h) / max(1, min(w, h))
    fill_ratio = area / (np.pi * radius * radius)
    if aspect_ratio > max_aspect_ratio or fill_ratio < min_fill_ratio:
        return None

    moments = cv2.moments(contour)
    if moments["m00"] > 0:
        x_center = moments["m10"] / moments["m00"]
        y_center = moments["m01"] / moments["m00"]

    return np.array([x_center, y_center, radius], dtype=np.float32)


def _circle_visible_fraction(circle, valid_mask):
    if valid_mask is None:
        return 1.0

    a, b, r = circle
    x0 = max(0, int(np.floor(a - r)))
    x1 = min(valid_mask.shape[1], int(np.ceil(a + r)) + 1)
    y0 = max(0, int(np.floor(b - r)))
    y1 = min(valid_mask.shape[0], int(np.ceil(b + r)) + 1)
    if x0 >= x1 or y0 >= y1:
        return 0.0

    circle_roi = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
    cv2.circle(
        circle_roi,
        (int(round(a - x0)), int(round(b - y0))),
        int(round(r)),
        255,
        thickness=cv2.FILLED,
    )

    total_circle_area = np.pi * r * r
    visible_area = np.count_nonzero(
        cv2.bitwise_and(circle_roi, valid_mask[y0:y1, x0:x1])
    )
    return min(1.0, visible_area / max(total_circle_area, 1.0))


def _circle_edge_clearance(circle, valid_mask):
    if valid_mask is None:
        return np.inf

    a, b, _ = circle
    x = int(round(a))
    y = int(round(b))
    if x < 0 or x >= valid_mask.shape[1] or y < 0 or y >= valid_mask.shape[0]:
        return 0.0

    binary_mask = (valid_mask > 0).astype(np.uint8)
    distance_to_edge = cv2.distanceTransform(binary_mask, cv2.DIST_L2, 5)
    return float(distance_to_edge[y, x])


def _circle_quality(circle, valid_mask):
    visible_fraction = _circle_visible_fraction(circle, valid_mask)
    edge_clearance = _circle_edge_clearance(circle, valid_mask)
    return visible_fraction, edge_clearance - circle[2]


def _circle_passes_quality(circle, valid_mask, min_visible_fraction, edge_margin):
    visible_fraction, edge_margin_px = _circle_quality(circle, valid_mask)
    return visible_fraction >= min_visible_fraction and edge_margin_px >= edge_margin


def detect_fiducial_circles(gray, valid_mask=None,
        min_visible_fraction=FIDUCIAL_TRACK_MIN_VISIBLE_FRACTION,
        edge_margin=FIDUCIAL_TRACK_EDGE_MARGIN):
    contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    circles = []
    for contour in contours:
        circle = _circle_from_blob(contour)
        if circle is None:
            continue

        if not _circle_passes_quality(circle, valid_mask, min_visible_fraction, edge_margin):
            continue

        circles.append(circle)

    if not circles:
        return None

    circles.sort(key=lambda circle: circle[0])
    return np.array([circles], dtype=np.float32)


class CircleTracker():
    num_trackers=0
    def __init__(self, a, b, r, manhattan_threshold=10, radius_threshold=5,
            disp_color=(0, 255, 0), frame_drop_threshold=FIDUCIAL_STALE_FRAME_LIMIT,
            tracker_update_padding=15,
            cal=None):
        self.a = a
        self.b = b
        self.r = r
        self.measured_a = a
        self.measured_b = b
        self.measured_r = r
        self.va = 0.0
        self.vb = 0.0
        self.vr = 0.0
        self.legitimacy = 1.0
        self.manhattan_threshold = manhattan_threshold
        self.radius_threshold = radius_threshold
        self.disp_color = disp_color
        self.last_ndx = None
        self.prediction_ndx = None
        self.frame_drop_threshold = frame_drop_threshold
        self.tracker_index = CircleTracker.num_trackers
        self.tracker_update_padding = tracker_update_padding
        CircleTracker.num_trackers+=1
        self.cal = cal

    def explains(self, a,b,r):
        # uses the manhattan distance and the radius to determine if the detected circle is explained by the existing model.
        return abs(self.a-a)+abs(self.b-b)<=self.manhattan_threshold and abs(self.r-r)<=self.radius_threshold

    def has_current_measurement(self, ndx):
        return self.last_ndx == ndx

    def predict_to(self, ndx):
        if self.prediction_ndx is None:
            self.prediction_ndx = ndx
            return

        dt = ndx - self.prediction_ndx
        if dt <= 0:
            return

        self.a += self.va * dt
        self.b += self.vb * dt
        self.r = max(FIDUCIAL_MIN_RADIUS, min(FIDUCIAL_MAX_RADIUS, self.r + self.vr * dt))
        self.prediction_ndx = ndx

    def check_legit(self, ndx, frame, gray):
        """ determines if the image and history justifies maintaining the tracker. 
        Updates the estimate if the circle was missed by the Hough transform  
        ndx - global time indexing, used for detecting missed frames
        """
        if self.last_ndx is None:
            self.last_ndx = ndx
        else:
            if ndx-self.last_ndx >= self.frame_drop_threshold:
                self.legitimacy = 0.0
            else:
                self.predict_to(ndx)

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

        return self.legitimacy>0.0


    def update(self, ndx, a, b, r, frame, gray, display=True):
        """ updates the tracker using the frame and the grayscale frame
        ndx - global time indexing, used for detecting missed frames
        a, b, r - circle definition from Hough transform
    
        """
        if self.last_ndx is not None and ndx > self.last_ndx:
            dt = ndx - self.last_ndx
            self.va = (a - self.measured_a) / dt
            self.vb = (b - self.measured_b) / dt
            self.vr = (r - self.measured_r) / dt

        self.last_ndx = ndx
        self.prediction_ndx = ndx
        detected_radius = r
        crop_radius = r + self.tracker_update_padding

        x0 = max(0, int(a - crop_radius))
        x1 = min(gray.shape[1], int(a + crop_radius))
        y0 = max(0, int(b - crop_radius))
        y1 = min(gray.shape[0], int(b + crop_radius))
        self.myimg = gray[y0:y1, x0:x1]
        # calculate moments of binary image

        M = cv2.moments(self.myimg)

        if M['m00']<75000:
            # acknowledge loss of tracking
            print("loss of tracking for tracker %d"%self.tracker_index)
            self.legitimacy=0.0
            return

        cv2.circle(self.myimg,( int(a - x0), int(b - y0)), 5, (0,0,0), -1)
        self.a = a
        self.b = b
        self.r = detected_radius
        self.measured_a = a
        self.measured_b = b
        self.measured_r = detected_radius
        if not self.cal is None:
            x0, SinvUT = self.cal
            xi = SinvUT@(np.array([[self.a,self.b]]).T-x0)
            self.theta = np.arctan2(-xi[0,0],xi[1,0])
            # vec = np.array([self.a, self.b]) - x0.T
            # self.theta = np.arctan2(vec[0,1],vec[0,0])
            # print('theta, theta2: ',self.theta*180/np.pi,theta2*180/np.pi)

        if display:
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
            calibration_file_name=None,
            diagnostic_file_name=None,
            diagnostic_name=""):
        self.circle_trackers = []
        self.ndx = 0
        self.calibration_file_name = calibration_file_name
        self.diagnostic_file_name = diagnostic_file_name
        self.diagnostic_name = diagnostic_name
        self.cal_data = []
        self.diagnostic_rows = []
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

    def save_diagnostics(self):
        if self.diagnostic_file_name is None:
            return
        header = (
            "frame,channel,aggregate_angle,prev_angle_before,raw_avg_mod_angle,"
            "n_rotations,jump_flag,n_updated,tracker_id,tracker_theta,"
            "tracker_mod_angle,tracker_weight,tracker_a,tracker_b,tracker_r"
        )
        with open(self.diagnostic_file_name, 'w') as f:
            f.write(header + "\n")
            for row in self.diagnostic_rows:
                f.write(",".join(str(item) for item in row) + "\n")

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

    @staticmethod
    def _weighted_periodic_mean(angles, weights, period):
        angles = np.asarray(angles, dtype=float)
        weights = np.asarray(weights, dtype=float)
        if angles.size == 0:
            return np.nan
        if not np.any(weights > 0):
            weights = np.ones_like(angles)

        phase = np.mod(angles, period) * (2 * np.pi / period)
        mean_phase = np.arctan2(
            np.sum(weights * np.sin(phase)),
            np.sum(weights * np.cos(phase)),
        )
        return np.mod(mean_phase * period / (2 * np.pi), period)

    @staticmethod
    def _nearest_periodic_continuation(mod_angle, previous_angle, period):
        if not np.isfinite(previous_angle):
            return mod_angle
        base_rotation = int(np.round((previous_angle - mod_angle) / period))
        candidates = mod_angle + period * np.arange(base_rotation - 2, base_rotation + 3)
        return candidates[np.argmin(np.abs(candidates - previous_angle))]

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
        updated_trackers = [ct for ct in self.circle_trackers if ct.last_ndx == self.ndx]
        prev_ang_before = self.prev_ang
        raw_avg_mod_angle = np.nan
        jump_flag = 0
        period = np.pi / 4
        if len(updated_trackers) > 0:
            raw_angs = [ct.theta for ct in updated_trackers]
            weights = [max((1 - abs(ct.theta)*(4/np.pi)),0) for ct in updated_trackers]
            mod_angles = [np.mod(ct.theta, period) for ct in updated_trackers]
            avg_mod_angle = self._weighted_periodic_mean(mod_angles, weights, period)
            raw_avg_mod_angle = avg_mod_angle

            if self.ndx == 0:
                ang = avg_mod_angle
            else:
                ang = self._nearest_periodic_continuation(
                    avg_mod_angle,
                    self.angs_abs[-1],
                    period,
                )
        else:
            ang = self.prev_ang

        if self.ndx == 0:
            self.angs_abs = np.array([ang])
            self.prev_ang = ang
            self.n_rotations = 0
        else:
            if np.isfinite(raw_avg_mod_angle):
                new_rotations = int(np.round((ang - raw_avg_mod_angle) / period))
                jump_flag = int(new_rotations != self.n_rotations)
                self.n_rotations = new_rotations
            self.prev_ang = ang
            self.angs_abs = np.append(self.angs_abs,ang)

        if self.diagnostic_file_name is not None:
            if len(updated_trackers) == 0:
                self.diagnostic_rows.append([
                    self.ndx,
                    self.diagnostic_name,
                    ang,
                    prev_ang_before,
                    raw_avg_mod_angle,
                    self.n_rotations,
                    jump_flag,
                    0,
                    -1,
                    np.nan,
                    np.nan,
                    np.nan,
                    np.nan,
                    np.nan,
                    np.nan,
                ])
            else:
                for ct, weight in zip(updated_trackers, weights):
                    self.diagnostic_rows.append([
                        self.ndx,
                        self.diagnostic_name,
                        ang,
                        prev_ang_before,
                        raw_avg_mod_angle,
                        self.n_rotations,
                        jump_flag,
                        len(updated_trackers),
                        ct.tracker_index,
                        ct.theta,
                        np.mod(ct.theta, np.pi/4),
                        weight,
                        ct.a,
                        ct.b,
                        ct.r,
                    ])


    def annotate_circles(self, frame, gray, color, valid_mask=None, display=True):
        detected_circles = detect_fiducial_circles(gray, valid_mask=valid_mask)
        if display:
            cv2.imshow('gray%d%d%d'%color , gray)
        for ct in self.circle_trackers:
            if ct.last_ndx != self.ndx:
                ct.predict_to(self.ndx)

        updated_tracker_ids = set()
        if detected_circles is not None:
            # print("detected %d circles. Trackers: "%detected_circles.shape[1]+", ".join(["%d"%ct.tracker_index for ct in self.circle_trackers]))
            for pt in detected_circles[0, :]:
                (a, b, r) = pt
                matching_trackers = [
                    ct for ct in self.circle_trackers
                    if id(ct) not in updated_tracker_ids and ct.explains(a, b, r)
                ]
                if matching_trackers:
                    ct = min(
                        matching_trackers,
                        key=lambda tracker: abs(tracker.a - a) + abs(tracker.b - b) + abs(tracker.r - r),
                    )
                    ct.update(self.ndx, a,b,r, frame, gray, display=display)
                    updated_tracker_ids.add(id(ct))
                    continue

                if _circle_passes_quality(
                        pt,
                        valid_mask,
                        FIDUCIAL_NEW_MIN_VISIBLE_FRACTION,
                        FIDUCIAL_NEW_EDGE_MARGIN):
                    ct = CircleTracker(a, b, r, disp_color=color, cal=self.ellipse_def)
                    ct.update(self.ndx, a,b,r, frame, gray, display=display) # always update once at least
                    print("initializing new tracker", self.ndx, a,b,r)  
                    self.circle_trackers.append(ct)
                    updated_tracker_ids.add(id(ct))

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

        # drop any self-declared illigitimate circle trackers
        self.circle_trackers = [ct for ct in self.circle_trackers if ct.check_legit(self.ndx, frame, gray)]

        for ct in self.circle_trackers:
            if self.ndx - ct.last_ndx > FIDUCIAL_DISPLAY_GRACE_FRAMES:
                continue
            ct.display(frame)
            if not (self.calibration_file_name is None) and ct.last_ndx == self.ndx:
                self.cal_data.append([ct.a, ct.b, ct.r,self.ndx])

        if self.calibration_file_name is None:
            self.calc_camera_angs()
            self.angs_flag = True
        self.ndx+=1

def test(file='first_vido.h264', 
        inner_mask_loc = 'inner_mask5.png',
        outer_mask_loc = 'outer_mask3.png',
        pre_mask_save_loc = "pre_mask.png",
        red_cal_save_loc = None,
        blue_cal_save_loc = None,
        red_diagnostic_save_loc = None,
        blue_diagnostic_save_loc = None,
        display=True):
    print("filename %s"%file)
    cap = cv2.VideoCapture(file)
    # To generate new callibration data

    if red_cal_save_loc is None or blue_cal_save_loc is None:
        # Read calibration values from file
        circle_cal = np.loadtxt('circle_cal.csv', delimiter=',')

        red_circle_annotator = CircleAnnotator(
            x0= circle_cal[0,0], y0 = circle_cal[0,1],
            SinvUT = np.array([[circle_cal[0,2], circle_cal[0,3]],
                                [circle_cal[0,4],  circle_cal[0,5] ]]),
            diagnostic_file_name=red_diagnostic_save_loc,
            diagnostic_name="red")
        blue_circle_annotator = CircleAnnotator(
            x0= circle_cal[1,0], y0 = circle_cal[1,1],
            SinvUT = np.array([[circle_cal[1,2], circle_cal[1,3]],
                                [circle_cal[1,4],  circle_cal[1,5] ]]),
            diagnostic_file_name=blue_diagnostic_save_loc,
            diagnostic_name="blue")
    else:
        red_circle_annotator = CircleAnnotator(
            calibration_file_name=red_cal_save_loc,
            diagnostic_file_name=red_diagnostic_save_loc,
            diagnostic_name="red")
        blue_circle_annotator = CircleAnnotator(
            calibration_file_name=blue_cal_save_loc,
            diagnostic_file_name=blue_diagnostic_save_loc,
            diagnostic_name="blue")

    

    mask = inner_mask = outer_mask = None
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret ==True:
            frame=frame[:320, 25:-1]
            
            if mask is None:
                cv2.imwrite(pre_mask_save_loc, frame)
                inner_mask = cv2.resize(cv2.imread(inner_mask_loc), frame.shape[1::-1])
                outer_mask = cv2.resize(cv2.imread(outer_mask_loc), frame.shape[1::-1])
                # mask = cv2.resize(cv2.imread('testbed_mask.png'), frame.shape[1::-1])
                mask = cv2.bitwise_or(inner_mask, outer_mask)
            frame = cv2.bitwise_and(frame, mask)
            grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray0 = isolate_white_fiducials(grayframe, inner_mask[:,:,0])
            gray2 = isolate_white_fiducials(grayframe, outer_mask[:,:,0])

            # hist = cv2.calcHist([frame[:,:, 2]], [0], None, [256], [0, 256])
            # plt.plot( hist)
            # cv2.imshow('gray', gray2)
            # plt.show()
            # exit()
            blue_circle_annotator.annotate_circles(frame, gray0, (255,0,0), inner_mask[:,:,0], display=display)
            red_circle_annotator.annotate_circles(frame, gray2, (0,0,255), outer_mask[:,:,0], display=display)


            if display:
                cv2.imshow('Frame', frame)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
            # time.sleep(0.1)
        else:
            break


    red_circle_annotator.save_cal()
    blue_circle_annotator.save_cal()
    red_circle_annotator.save_diagnostics()
    blue_circle_annotator.save_diagnostics()

    cap.release()
    if display:
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
