# This code is used to read the image from the camera and send data to arduino
import cv2
import numpy as np

def read_camera():
    template_1 = cv.imread('turn_right.png', cv.IMREAD_GRAYSCALE)
    template_2 = cv.imread('turn_left.png', cv.IMREAD_GRAYSCALE)
    template_3 = cv.imread('stop.png', cv.IMREAD_GRAYSCALE)
    template_4 = cv.imread('turn_around.png', cv.IMREAD_GRAYSCALE)
    threshold = 0.4

    scale_percent = 20 # percent of original size
    width = int(template_1.shape[1] * scale_percent / 100)
    height = int(template_1.shape[0] * scale_percent / 100)
    dim = (width, height)

    # resize image
    template_1 = cv.resize(template_1, dim, interpolation = cv.INTER_AREA)
    template_2 = cv.resize(template_2, dim, interpolation = cv.INTER_AREA)
    template_3 = cv.resize(template_3, dim, interpolation = cv.INTER_AREA)
    w, h = template_1.shape[::-1]

    # All the 6 methods for comparison in a list
    method = str("cv.TM_CCOEFF_NORMED")
    cameraCapture = cv.VideoCapture(1) 
    # Read and process frames in loop
    success, frame = cameraCapture.read()
    frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    img2 = frame.copy()
    img = img2.copy()
    met = eval(method)
    # print(frame)
    # Apply template Matching
    res_1 = cv.matchTemplate(img,template_1,met,)
    res_2 = cv.matchTemplate(img,template_2,met,)
    res_3 = cv.matchTemplate(img,template_3,met,)
    # print(res)
    # Specify a threshold 
    # Store the coordinates of matched area in a numpy array 
    loc_1 = np.where(res_1 >= threshold)
    loc_2 = np.where(res_2 >= threshold)
    loc_3 = np.where(res_3 >= threshold) 
    
    # print (len(loc_1[0]))
    if len(loc_1[0]) != 0:
        for pt in zip(*loc_1[::-1]): 
            cv.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0, 255, 255), 2) 
        arduino.write(b'r')
        print("TURN RIGHT")
    elif len(loc_2[0]) != 0:
        for pt in zip(*loc_2[::-1]): 
            cv.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0, 255, 255), 2) 
        arduino.write(b'l')
        print("TURN LEFT")
    elif len(loc_3[0]) != 0:
        for pt in zip(*loc_3[::-1]): 
            cv.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0, 255, 255), 2) 
        arduino.write(b's')
        print("STOP!")

while True:
    line = arduino.readline()
    # print(line)
    if line == b'c\r\n':
        print('got signal')
        read_camera()

    if cv.waitKey(1) == ord('q'):
        break
cameraCapture.release()
cv.destroyAllWindows()
