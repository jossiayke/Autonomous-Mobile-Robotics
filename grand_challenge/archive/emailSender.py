import cv2
from datetime import datetime
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

# For visual perception tasks
from Perception import cntr_bbox

img = cntr_bbox.picam_frame()
num = 4
sit = 'pics'
block_now = 'red'
counter = 35
img_folder = '../test'
cv2.imwrite(img_folder+f'/{block_now}_{counter}.png',img)
# img_r = 'challenge_pics/new_pic.png'
# # Email information
# smtpUser = 'ykebede2@terpmail.umd.edu'
# smtpPass = 'QwE@$d1219'
# 
# # Destination email information
# toAdd =['ENPM809TS19@gmail.com','yosephcollege@gmail.com']
# fromAdd = smtpUser
# cc = 'jsuriya@umd.edu'
# 
# # img1 = cv2.imread('greenGripped.png')
# img_name = 'challenge_pics/greenGripped.png'
# 
# def sendEmail(img_name):
# 
#     # Define time stamp & record an image
#     pic_time = datetime.now().strftime('%Y%m%d%H%M%S')
# 
#     subject = 'Gripper Retrieved ' + pic_time
# 
#     msg = MIMEMultipart()
#     msg['Subject'] = subject
#     msg['From'] = fromAdd
#     msg['To'] = ",".join(toAdd) #toAdd
#     msg['Cc'] = cc
#     msg.preamble = 'Gripper Retrieved ' + pic_time
# 
#     # Email text
#     body = MIMEText("Gripper Retrieved at " + pic_time)
#     msg.attach(body)
# 
#     # Attach image
#     fp = open(img_name, 'rb')
#     img = MIMEImage (fp.read())
#     fp.close()
#     msg.attach(img)
# 
#     # Send email
#     s = smtplib.SMTP('smtp.gmail.com', 587)
# 
#     s.ehlo()
#     s.starttls()
#     s.ehlo()
# 
#     s.login(smtpUser, smtpPass)
#     s.sendmail(fromAdd, toAdd, msg.as_string())
#     s.quit()
# 
#     print("Email delivered")
# 
# sendEmail(img_r)