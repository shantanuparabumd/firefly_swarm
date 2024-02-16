import cv2
import numpy as np

# Define a function to update the mask based on the current slider positions
def update_mask(lower_hue, lower_saturation, lower_value, upper_hue, upper_saturation, upper_value,lower_area,upper_area,height):
    # Define the lower and upper bounds of the color range based on the current slider positions
    lower_range = np.array([lower_hue, lower_saturation, lower_value])
    upper_range = np.array([upper_hue, upper_saturation, upper_value])

    # Create a mask for the color range using cv2.inRange()
    mask = cv2.inRange(hsv_image, lower_range, upper_range)
    # mask = np.zeros((height, width), dtype=np.uint8)
    mask[0:height, :] = 0

    # Apply the mask to the original image using cv2.bitwise_and()
    result = cv2.bitwise_and(image, image, mask=mask)

    kernel = np.ones((3,3),np.uint8)
    erosion = cv2.erode(mask, kernel, iterations = 1)
    dilation = cv2.dilate(erosion, kernel, iterations = 1)
    # Apply gaussian blur to he mask
    mask_blur = cv2.GaussianBlur(dilation, (3, 3), 0)
    # Find the contours in the binary image
    contours, hierarchy = cv2.findContours(mask_blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
    status=0
    x,y,w,h=0,0,0,0
    if len(contours)==0:
        status = 0
    else:
        for max_contour in contours:
            area = cv2.contourArea(max_contour)
            if area>lower_area and area<upper_area:
                status = 1
                x,y,w,h = cv2.boundingRect(max_contour)
                tolerance=0
                mask = np.zeros(mask_blur.shape, np.uint8)
                mask[y:y+h+tolerance, x:x+w+tolerance] = mask_blur[y:y+h+tolerance, x:x+w+tolerance]    
                cv2.circle(result, (int(x+(w/2)),int(y+(h/2))), 2, (0,0,0), -1)
                cv2.circle(result, (int(x+(w/2)),int(y+(h/2))), 20, (0,255,255), 1)
                cv2.rectangle(result,(int(x),int(y)),(int(x+w),int(y+h)),(0,0,0),1)
                cv2.putText(result,f"Area {area}",(int(x-5),int(y-5)),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (25, 255, 0), 2, cv2.LINE_AA)
                
        
    cv2.line(img, (320, 220), (320, 260), (0, 0, 0), 2)
    cv2.line(img, (300, 240), (340, 240), (0, 0, 0), 2)
    
    # Display the result
    cv2.imshow("Result", result)


# Load the image
img = cv2.imread("red.png")
scale_percent = 60 # percent of original size
width = int(320)
height = int(280)
dim = (width, height)
# resize image
# image = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
image = img
image=cv2.flip(image,0)
image=cv2.flip(image,1)
# Convert the image to the HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Create a window to display the result
cv2.namedWindow("Track")

# Create sliders for the lower and upper bounds of the color range using cv2.createTrackbar()
cv2.createTrackbar("Lower Hue", "Track", 0, 179, lambda x: None)
cv2.createTrackbar("Lower Saturation", "Track", 0, 255, lambda x: None)
cv2.createTrackbar("Lower Value", "Track", 0, 255, lambda x: None)
cv2.createTrackbar("Upper Hue", "Track", 0, 179, lambda x: None)
cv2.createTrackbar("Upper Saturation", "Track", 0, 255, lambda x: None)
cv2.createTrackbar("Upper Value", "Track", 0, 255, lambda x: None)
cv2.createTrackbar("Upper Area", "Track", 0, 50000, lambda x: None)
cv2.createTrackbar("Lower Area", "Track", 0, 1000, lambda x: None)
cv2.createTrackbar("Height", "Track", 0, 480, lambda x: None)

# Set the initial slider positions
cv2.setTrackbarPos("Lower Hue", "Track", 0)
cv2.setTrackbarPos("Lower Saturation", "Track", 0)
cv2.setTrackbarPos("Lower Value", "Track", 0)
cv2.setTrackbarPos("Upper Hue", "Track", 179)
cv2.setTrackbarPos("Upper Saturation", "Track", 255)
cv2.setTrackbarPos("Upper Value", "Track", 255)

# Display the original image
cv2.imshow("Result", image)

while True:
    # Wait for a key press
    key = cv2.waitKey(1) & 0xFF

    # If the 'q' key is pressed, quit the program
    if key == ord("q"):
        break
    if key == ord("r"):
        print("Color Red")
        print("Lower Hue: ",lower_hue)
        print("Lower Saturation: ",lower_saturation)
        print("Lower Value: ",lower_value)

        print("Upper Hue: ",upper_hue)
        print("Upper Saturation: ",upper_saturation)
        print("Upper Value: ",upper_value)
    if key == ord("g"):
        print("Color Green")
        print("Lower Hue: ",lower_hue)
        print("Lower Saturation: ",lower_saturation)
        print("Lower Value: ",lower_value)

        print("Upper Hue: ",upper_hue)
        print("Upper Saturation: ",upper_saturation)
        print("Upper Value: ",upper_value)

    if key == ord("b"):
        print("Color Blue")
        print("Lower Hue: ",lower_hue)
        print("Lower Saturation: ",lower_saturation)
        print("Lower Value: ",lower_value)

        print("Upper Hue: ",upper_hue)
        print("Upper Saturation: ",upper_saturation)
        print("Upper Value: ",upper_value)

    # Get the current slider positions
    lower_hue = cv2.getTrackbarPos("Lower Hue", "Track")
    lower_saturation = cv2.getTrackbarPos("Lower Saturation", "Track")
    lower_value = cv2.getTrackbarPos("Lower Value", "Track")
    upper_hue = cv2.getTrackbarPos("Upper Hue", "Track")
    upper_saturation = cv2.getTrackbarPos("Upper Saturation", "Track")
    upper_value = cv2.getTrackbarPos("Upper Value", "Track")
    upper_saturation = cv2.getTrackbarPos("Upper Saturation", "Track")
    lower_area = cv2.getTrackbarPos("Lower Area", "Track")
    upper_area = cv2.getTrackbarPos("Upper Area", "Track")
    height = cv2.getTrackbarPos("Height", "Track")

    # Update the mask based on the current slider positions
    update_mask(lower_hue, lower_saturation, lower_value, upper_hue, upper_saturation, upper_value,lower_area,upper_area,height)


cv2.destroyAllWindows()
