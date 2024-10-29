import cv2

# cv2.VideoCapture() allows you to read a video file or capture video from camera.
vid_capture = cv2.VideoCapture('video_12-06-2020_08-19-53_PM.mp4')
#Get the frame rate
fps = vid_capture.get(5)
print('Frames per second : ', fps,'FPS')
#get total number of frames
frame_count = vid_capture.get(7)
print('Frame count : ', frame_count)

# Read each frame and display
while True:
    ret, frame = vid_capture.read()

    # Check if the frame is successfully read
    if ret:
        cv2.imshow('',frame)
    else:
        # Break the loop if no more frames are available
        break

cv2.waitKey(0)
# Release the video capture object
vid_capture.release()
# Close all windows
cv2.destroyAllWindows()