from tkinter import *
import tkinter.messagebox
import numpy as np
import cv2

root = Tk()
root.title('NHOM_03')
root.geometry('960x593')
image = PhotoImage(file="bg.png")
label = Label(image=image)
label.pack()

def laneDetection(): 

    def region_of_interest(canny):  #Tạo ra vùng ảnh ta sẽ áp dụng các xử lý / biến đổi trên đó và bỏ qua các vùng còn lại.

        height = canny.shape[0]   # lấy kích thước của hình canny
        width = canny.shape[1]
        # tạo mảng mask cùng kt và kiểu dữ liệu với ảnh canny , tất cả gái trị thiết lập là 0
        mask = np.zeros_like(canny)  
        #Tạo một matrix với kích thước bằng đúng kích thước khu vực mình quan tâm để xử lí 
        triangle = np.array(
            [
                [
                    (200, height),
                    (800, 350),
                    (1200, height),
                ]
            ],
            np.int32,
        )
        cv2.fillPoly(mask, triangle, 255) 
        # bitwise and với ảnh gốc để tạo ra vùng một matrix chứa các điểm ảnh trong vùng ROI
        masked_image = cv2.bitwise_and(canny, mask)
        print(12)
        return masked_image


    def houghLine(cropped_canny):     # trả về mảng chứa các đường thẳng có trong ảnh 
        return cv2.HoughLinesP ( cropped_canny, 2, np.pi / 180, 100, np.array([]), minLineLength=10, maxLineGap=2 )
            

    def addWeighted(frame, line_image): #Trộn 2 ảnh lại với nhau để ra ảnh hoàn chỉnh ( ảnh gốc và ảnh line)
        return cv2.addWeighted(frame, 0.8, line_image, 1, 0)


    def canny(img):  # Dùng thuật toán Canny Edge Detection để phát hiện các góc cạnh trong bức ảnh .

        if img is None:
            cap.release()
            cv2.destroyAllWindows()
            exit()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # chuyển ảnh RGB sang Gray 
        # Áp dụng bộ lọc Gauss để giảm nhiễu ảnh với mặt nạ kích thước 5x5
        blur = cv2.GaussianBlur(gray, ksize=(5,5), sigmaX=0, sigmaY=0) 

        # Dùng thuật toán canny để xác định cạnh trong ảnh với 2 ngưỡng tự chọn
        canny = cv2.Canny(blur, 50, 150)     # các ngưỡng so sánh với cường độ pixel 
        return canny

            
    def display_lines(img, lines):   # Vẽ ra các đường thẳng đã được detect vào hình 
     
        line_image = np.zeros_like(img)
        if lines is not None:   
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 5)  # hàm để vẽ đường thẳng 
        return line_image


    def make_points(image, line): #Taọ ra 2 điểm dựa vào phương trình đường thẳng
        # slope là hệ số góc , intercept là hệ số giao
        slope, intercept = line  # tính hệ số góc và hệ số giao của đường thẳng 
        y1 = int(image.shape[0])
        y2 = int(y1 * 3.5 / 5)
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return [[x1, y1, x2, y2]]

    def average_slope_intercept(image, lines): 
      #Tạo ra một phương trình đường thẳng trung bình dựa vào các lines mà đã được detect ở phía trên

        left_fit = []
        right_fit = []
        if lines is None:
            return None
        for line in lines:
            for x1, y1, x2, y2 in line:
                fit = np.polyfit((x1, x2), (y1, y2), 1)   # tính pt đường thẳng 
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    left_fit.append((slope, intercept))
                else:
                    right_fit.append((slope, intercept))
        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)
        left_line = make_points(image, left_fit_average)
        right_line = make_points(image, right_fit_average)
        averaged_lines = (left_line, right_line)
        return averaged_lines


    if __name__ == "__main__":
        cap = cv2.VideoCapture("vd_normal.mp4")  # open video file
        while cap.isOpened():
            _, frame = cap.read()  
            canny_image = canny(frame)              
            cropped_canny = region_of_interest(canny_image)  

            lines = houghLine(cropped_canny)                            # trả về mảng chứa các đường thẳng có trong ảnh 
            averaged_lines = average_slope_intercept(frame, lines)      #Tạo ra một phương trình đường thẳng trung bình
            line_image = display_lines(frame, averaged_lines)           # Vẽ ra các đường thẳng đã được detect vào hình 
            combo_image = addWeighted(frame, line_image)                #Trộn 2 ảnh lại với nhau để ra ảnh hoàn chỉnh 
            cv2.imshow("canny_image",canny_image)
            cv2.imshow("cropped_canny",cropped_canny)
            cv2.imshow("LANE_DETECTION_NHOM_03 ", combo_image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cap.release()
        cv2.destroyWindow()   

B = Button(root, text = "START", command = laneDetection,width= 10, height = 2)
B.place(x = 390,y = 540)
root.mainloop()  
cv2.destroyAllWindows()
