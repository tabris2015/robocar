import cv2
import numpy as np
boundaries = [
	([17, 15, 100], [50, 56, 200]),     # red
	([86, 31, 4], [220, 88, 50]),       # blue
	([25, 146, 190], [62, 174, 250]),   # yellow
	([103, 86, 65], [145, 133, 128])    # gray
    ]

lowerBound=np.array([33,80,40])
upperBound=np.array([102,255,255])

def show_webcam(mirror=False):
    cam = cv2.VideoCapture(0)
    
    while True:
        ret_val, img = cam.read()
        img = cv2.resize(img, (320,240))
        imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(imgHSV,lowerBound,upperBound)
        
        kernelOpen=np.ones((6,6))
        kernelClose=np.ones((20,20))
        maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

        maskFinal = maskClose

        im2, conts, hierarchy = cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        
        if not conts: 
            continue
        cv2.drawContours(img,conts,-1,(255,0,0),3)
        c = max(conts, key = cv2.contourArea)
        


        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    
        # draw the contour and center of the shape on the image
        cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
        cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)

        area_txt = str(cv2.contourArea(c))
        deviation = str(int(cX - 160))
        cv2.putText(img, area_txt, (cX - 20, cY - 20),
		        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(img, deviation, (cX - 20, cY + 20),
		        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    

        x,y,w,h = cv2.boundingRect(c)
        # draw the book contour (in green)
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        # loop over the boundaries
        # for (lower, upper) in boundaries:
        #     # create NumPy arrays from the boundaries
        #     lower = np.array(lower, dtype = "uint8")
        #     upper = np.array(upper, dtype = "uint8")
        
        #     # find the colors within the specified boundaries and apply
        #     # the mask
        #     mask = cv2.inRange(img, lower, upper)
        #     output = cv2.bitwise_and(img, img, mask = mask)
 
        # cv2.imshow("images", np.hstack([img, mask]))
        cv2.imshow('my webcam', img)
        cv2.imshow('mask', maskFinal)
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    cv2.destroyAllWindows()


def main():
    show_webcam(mirror=True)


if __name__ == '__main__':
    main()