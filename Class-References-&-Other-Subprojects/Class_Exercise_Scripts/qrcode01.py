import cv2
import os

command = 'sudo modprobe bcm2835-v412'
os.system(command)

# Open video capture
cap = cv2.VideoCapture(0)

# Define detector
detector = cv2.QRCodeDetector()

while True:
    
    check, img = cap.read()
    
    data, bbox, _= detector.detectAndDecode(img)
    
    if (bbox is not None):
        for i in range(len(bbox)):
            cv2.line(img, tuple(bbox[i][0]),tuple(bbox[(i+1)%len(bbox)][0]), color=(0,0,255),thickness=4)
            cv2.putText(img, data, (int(bbox[0][0][0]), int(bbox[0][0][0])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),2)
            
    if data:
        print("Data: ", data)
        
    # Show result to screen
    cv2. imshow("QR Code detector", img)
    
    # Break out of loop by pressing the q key
    if(cv2.waitKey(1) == ord("q")):
        break
    
cap.release()
cv2.destroyAllWindows()