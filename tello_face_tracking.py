from pyimagesearch.objcenter import ObjCenter
import cv2
from pyimagesearch.pid import PID
from djitellopy import Tello
import signal
import sys
import imutils
import time
from datetime import datetime
from multiprocessing import Manager, Process, Pipe, Event

global tello
tello = None
video_writer = None

# function to handle keyboard interrupt
def signal_handler(sig, frame):
    print("Signal Handler")
    try:
        tello.streamoff()
        #tello.land()
    except:
        pass

    try:
        video_writer.release()
    except:
        pass

    sys.exit()




if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    run_pid = True
    track_face = True  # True - cause the Tello to start to track/follow a face
    save_video = True
    fly = False
    
    start_time = time.time()
    frameCount = 0
    fps = 0

    exit_event = Event()  

    max_speed_limit=40
    max_speed_threshold = max_speed_limit

    tello = Tello() 
    tello.connect()
    tello.streamon()


    if fly:
        tello.takeoff()
        tello.move_up(20)

    face_center = ObjCenter("./haarcascade_frontalface_default.xml")
    pan_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
    tilt_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
    pan_pid.initialize()
    tilt_pid.initialize()

    while not exit_event.is_set():
        loop_time = time.time()
       
        frame_read = tello.get_frame_read()
        frame = frame_read.frame

        H, W, _ = frame.shape
        print(H,W)
    
        frame = imutils.resize(frame, width=400)
        H, W, _ = frame.shape
        print(W,H)

        gframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if save_video == True and video_writer == None:
            video_file = f"video_{datetime.now().strftime('%d-%m-%Y_%I-%M-%S_%p')}.mp4"
            video_writer = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'mp4v'), 5, (W, H))


        # calculate the center of the frame as this is (ideally) where
        # we will we wish to keep the object
        centerX = W // 2
        centerY = H // 2

        # draw a circle in the center of the frame
        cv2.circle(frame, center=(centerX, centerY), radius=5, color=(0, 0, 255), thickness=-1)

        # find the object's location
        frame_center = (centerX, centerY)
        objectLoc = face_center.update(frame, frameCenter=None)
        # print(centerX, centerY, objectLoc)

        ((objX, objY), rect, d) = objectLoc
        if d > 25 or d == -1:
            # then either we got a false face, or we have no faces.
            # the d - distance - value is used to keep the jitter down of false positive faces detected where there
            #                   were none.
            # if it is a false positive, or we cannot determine a distance, just stay put
            # print(int(pan_update), int(tilt_update))
            if track_face and fly:
                tello.send_rc_control(0, 0, 0, 0)
            ##continue  # ignore the sample as it is too far from the previous sample

        if rect is not None:
            (x, y, w, h) = rect
            cv2.rectangle(frame, (x, y), (x + w, y + h),
                        (0, 255, 0), 2)

            # draw a circle in the center of the face
            cv2.circle(frame, center=(objX, objY), radius=5, color=(255, 0, 0), thickness=-1)

            # Draw line from frameCenter to face center
            cv2.arrowedLine(frame, frame_center, (objX, objY), color=(0, 255, 0), thickness=2)

            if run_pid:
                # calculate the pan and tilt errors and run through pid controllers
                pan_error = centerX - objX
                pan_update = pan_pid.update(pan_error, sleep=0)

                tilt_error = centerY - objY
                tilt_update = tilt_pid.update(tilt_error, sleep=0)

                # print(pan_error, int(pan_update), tilt_error, int(tilt_update))
                cv2.putText(frame, f"X Error: {pan_error} PID: {pan_update:.2f}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 255, 0), 2, cv2.LINE_AA)

                cv2.putText(frame, f"Y Error: {tilt_error} PID: {tilt_update:.2f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (0, 0, 255), 2, cv2.LINE_AA)

               
                if pan_update > max_speed_threshold:
                    pan_update = max_speed_threshold
                elif pan_update < -max_speed_threshold:
                    pan_update = -max_speed_threshold

                # NOTE: if face is to the right of the drone, the distance will be negative, but
                # the drone has to have positive power so I am flipping the sign
                pan_update = pan_update * -1

                if tilt_update > max_speed_threshold:
                    tilt_update = max_speed_threshold
                elif tilt_update < -max_speed_threshold:
                    tilt_update = -max_speed_threshold

                print(int(pan_update), int(tilt_update))
                if track_face and fly:
                    # left/right: -100/100
                    tello.send_rc_control(int(pan_update // 3), 0, int(tilt_update // 2), 0)

        
        fps = 1 / (time.time() - loop_time)
        cv2.putText(frame, f"FPS: {fps:.2f}", (20, 110), cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (200, 200, 200), 2, cv2.LINE_AA)

        # display the frame to the screen

        cv2.imshow("Gray - Drone Face Tracking", gframe)
        cv2.imshow("Color - Drone Face Tracking", frame)      
        
        if frameCount == 0 :
            cv2.moveWindow("Gray - Drone Face Tracking", 300,200)
            cv2.moveWindow("Color - Drone Face Tracking", 800,200)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit_event.set()

        if save_video == True:
            video_writer.write(frame)

        frameCount = frameCount + 1
        


    try:
        tello.streamoff()
        #tello.land()
    except:
        pass

    try:
        video_writer.release()
    except:
        pass

    print("FrameCount:", frameCount)
    print("FPS:", frameCount / (time.time() - start_time) )
    print("Complete...")


