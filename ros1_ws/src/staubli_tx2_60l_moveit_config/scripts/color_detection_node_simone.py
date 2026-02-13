#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import time
import cv2
from staubli_tx2_60l_moveit_config.msg import BBoxes
from datetime import datetime
import time

pub = rospy.Publisher('/bboxes', BBoxes, queue_size=10)
# trackbar callback fucntion does nothing but required for trackbar
def callback(x):
    pass

def calculate_area(x1, y1, x2, y2):
    return abs((x2 - x1) * (y2 - y1))

def check_intersections(rect1, rect2):
    #print('sono dentro check intersections')
    x1_rect1, y1_rect1, x2_rect1, y2_rect1 = rect1
    x1_rect2, y1_rect2, x2_rect2, y2_rect2 = rect2

    if x1_rect2 > x2_rect1 or x2_rect2 <= x1_rect1:
        return False  # Non c'e intersezione lungo l'asse x

    if y1_rect2 > y2_rect1 or y2_rect2 <= y1_rect1:
        return False  # Non c'e intersezione lungo l'asse y

    #print('check return TRUE gli oggetti si intersecano')	
    return True

def check_area_bboxes(rect1,rect2):
    #print('sono dentro check area bboxes--')
    #print('rect1=',rect1,'rect2=',rect2)
    x1_rect1, y1_rect1, x2_rect1, y2_rect1 = rect1
    x1_rect2, y1_rect2, x2_rect2, y2_rect2 = rect2
    margine=25
    
    diff_x1 = x1_rect2 - x1_rect1
    diff_y1 = y1_rect2 - y2_rect1
    diff_x2 = x2_rect1 - x2_rect2
    diff_y2 = y1_rect1 - y2_rect2
    #print('DIFF=',diff_x1,diff_x2,diff_y1,diff_y2)
    
    # Verifica se le differenze sono entro il margine
    adiacenza_x = -margine <= diff_x1 <= margine or -margine <= diff_x2 <= margine
    adiacenza_y = -margine <= diff_y1 <= margine or -margine <= diff_y2 <= margine

    #print('ADICENZA=',adiacenza_x,adiacenza_y)
    
    if adiacenza_x and adiacenza_y: #solo se sono entrambi i rettangoli entro il margine su entrambi gli assi mi vai a creare un nuovo rettangolo
        # Creazione di un nuovo rettangolo
        nuovo_x1 = min(x1_rect1, x1_rect2)
        nuovo_y1 = min(y1_rect1, y1_rect2)
        nuovo_x2 = max(x2_rect1, x2_rect2)
        nuovo_y2 = max(y2_rect1, y2_rect2)
        nuovo_rect= [(nuovo_x1, nuovo_y1), (nuovo_x2, nuovo_y2)]
        #print('NEW RECT=',nuovo_rect)
        return nuovo_rect
    
    #print('NOn sono ADICENTI')
    return False

def pop_element(rect1, rect2,i):
    area_rect1 = calculate_area(*rect1)
    area_rect2 = calculate_area(*rect2)
    if area_rect1 >= area_rect2:
        return i+1
    else:
        return i

def create_and_publish_message(list):
    msg_to_send = BBoxes()
    for b in list:
        msg_to_send.bboxes.extend(b[0] + b[1]) #estendo la lista msg_to_send.bboxes con gli elementi della tupla risultante da b[0] + b[1] e dunque conterra (x,y,x',y')
    pub.publish(msg_to_send)
    #print('HO INVIATO IL FRAME')

def talker():
    
    rospy.init_node('color_detection_node', anonymous=True)

    cap = cv2.VideoCapture(0)
    cv2.namedWindow('Calibration')

    # management values
    calibration = 1
    detection = 0

    # default values
    x1d = 191
    y1d = 157
    x2d = 567
    y2d = 191
    x3d = 550
    y3d = 419
    x4d = 163
    y4d = 367	

    x1 = x1d
    y1 = y1d
    x2 = x2d
    y2 = y2d
    x3 = x3d
    y3 = y3d
    x4 = x4d
    y4 = y4d

    ilowH = 90
    ihighH = 179
    ilowS = 125
    ihighS = 255
    ilowV = 60
    ihighV = 146

    #flag and treshold
    treshold = 3
    send_message=0
    detected = 0
    curr_detected = 0
    first_frame = 1
    padding = 10 #aggiungere padding nella funzione di controllo delle intersezioni in caso di rilevamento di piu di 1 blocco in una stessa area cosi da fare un check di intersezioni in una area maggiore
    pr = []	

    # create crop trackbars
    cv2.createTrackbar('x1', 'Calibration', x1, 639, callback)
    cv2.createTrackbar('y1', 'Calibration', y1, 479, callback)

    cv2.createTrackbar('x2', 'Calibration', x2, 639, callback)
    cv2.createTrackbar('y2', 'Calibration', y2, 479, callback)

    cv2.createTrackbar('x3', 'Calibration', x3, 639, callback)
    cv2.createTrackbar('y3', 'Calibration', y3, 479, callback)

    cv2.createTrackbar('x4', 'Calibration', x4, 639, callback)
    cv2.createTrackbar('y4', 'Calibration', y4, 479, callback)

    # create hsv trackbars for color change
    cv2.createTrackbar('lowH', 'Calibration', ilowH, 179, callback)
    cv2.createTrackbar('highH', 'Calibration', ihighH, 179, callback)

    cv2.createTrackbar('lowS', 'Calibration', ilowS, 255, callback)
    cv2.createTrackbar('highS', 'Calibration', ihighS, 255, callback)

    cv2.createTrackbar('lowV', 'Calibration', ilowV, 255, callback)
    cv2.createTrackbar('highV', 'Calibration', ihighV, 255, callback)

    while calibration:

        # grab the frame
        ret, frame = cap.read()

        # get trackbar positions
        x1 = cv2.getTrackbarPos('x1', 'Calibration')
        y1 = cv2.getTrackbarPos('y1', 'Calibration')
        x2 = cv2.getTrackbarPos('x2', 'Calibration')
        y2 = cv2.getTrackbarPos('y2', 'Calibration')
        x3 = cv2.getTrackbarPos('x3', 'Calibration')
        y3 = cv2.getTrackbarPos('y3', 'Calibration')
        x4 = cv2.getTrackbarPos('x4', 'Calibration')
        y4 = cv2.getTrackbarPos('y4', 'Calibration')
        ilowH = cv2.getTrackbarPos('lowH', 'Calibration')
        ihighH = cv2.getTrackbarPos('highH', 'Calibration')
        ilowS = cv2.getTrackbarPos('lowS', 'Calibration')
        ihighS = cv2.getTrackbarPos('highS', 'Calibration')
        ilowV = cv2.getTrackbarPos('lowV', 'Calibration')
        ihighV = cv2.getTrackbarPos('highV', 'Calibration')

        # compute in hsv color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([ilowH, ilowS, ilowV])
        higher_hsv = np.array([ihighH, ihighS, ihighV])
        mask = cv2.inRange(hsv_frame, lower_hsv, higher_hsv)
        frame = cv2.bitwise_and(frame, frame, mask=mask)

        # draw polygon
        crop_border = np.array([[[x1, y1], [x2, y2], [x3, y3], [x4, y4]]], np.int32)
        frame_poly = frame.copy()
        cv2.polylines(frame_poly, [crop_border], True, (0,0,255), thickness=1)

        # warp frame
        pts1 = np.float32([[x1,y1],[x2,y2],[x4,y4],[x3,y3]])
        pts2 = np.float32([[0,0],[853,0],[0,480],[853,480]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        mask_warp = cv2.warpPerspective(mask, matrix, (853, 480))

        # show thresholded image & mask
        mask_warp_3ch = cv2.merge((mask_warp,mask_warp,mask_warp))
        result = np.concatenate((frame_poly, mask_warp_3ch), axis=1)
        cv2.imshow('Calibration', result)

        # key input manager
        k = cv2.waitKey(1)
        if k == ord('q'):    # q key to stop
            print('quit')
            break
        elif k == ord('c'):  # c key to confirm calibration
            print('confirmed calibration')
            cv2.destroyWindow('Calibration')
            calibration = 0
            detection = 1
        elif k == ord('r'):  # r key to reset parameters
            print('resetted parameters')
            x1 = cv2.setTrackbarPos('x1', 'Calibration', x1d)
            y1 = cv2.setTrackbarPos('y1', 'Calibration', y1d)
            x2 = cv2.setTrackbarPos('x2', 'Calibration', x2d)
            y2 = cv2.setTrackbarPos('y2', 'Calibration', y2d)
            x3 = cv2.setTrackbarPos('x3', 'Calibration', x3d)
            y3 = cv2.setTrackbarPos('y3', 'Calibration', y3d)
            x4 = cv2.setTrackbarPos('x4', 'Calibration', x4d)
            y4 = cv2.setTrackbarPos('y4', 'Calibration', y4d)    
            ilowH = cv2.setTrackbarPos('lowH', 'Calibration', 0)
            ihighH = cv2.setTrackbarPos('highH', 'Calibration', 255)
            ilowS = cv2.setTrackbarPos('lowS', 'Calibration', 0)
            ihighS = cv2.setTrackbarPos('highS', 'Calibration', 255)
            ilowV = cv2.setTrackbarPos('lowV', 'Calibration', 0)
            ihighV = cv2.setTrackbarPos('highV', 'Calibration', 255)
        elif k != 255:
            print('c: confirm calibration\nr: reset parameters\nq: quit') # else print keys info

    while detection:

        # grab the frame
        ret, frame = cap.read()

        # warp frame
        pts1 = np.float32([[x1,y1],[x2,y2],[x4,y4],[x3,y3]])
        pts2 = np.float32([[0,0],[1000,0],[0,560],[1000,560]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        frame_warp = cv2.warpPerspective(frame, matrix, (1000, 560))

        # compute in hsv color space
        hsv_frame_warp = cv2.cvtColor(frame_warp, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([ilowH, ilowS, ilowV])
        higher_hsv = np.array([ihighH, ihighS, ihighV])
        mask_warp = cv2.inRange(hsv_frame_warp, lower_hsv, higher_hsv)

        # compute and draw the bounding boxes
        contours, hierarchy = cv2.findContours(mask_warp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        list_bboxes = []
        for index, c in enumerate(contours):
            area = cv2.contourArea(c)
            if (hierarchy[0,index,3] == -1 and area > 1000 and len(list_bboxes) < 10):
                cv2.drawContours(frame_warp, contours, index, (0,255,0), 1)
                x,y,w,h = cv2.boundingRect(c)  #x and y are the coordinates of the top left corner while w is the width and h is the height of the rectangle
                lt_corner = (x, y)
                rb_corner = (x + w, y + h)
                list_bboxes.append([lt_corner, rb_corner])
                cv2.rectangle(frame_warp, lt_corner, rb_corner, (255,0,0), 2)

        cv2.imshow('Detection', frame_warp)
        #current_time = time.time()
        #current_time_millis2 = int(round(time.time() * 1000))

        # key input manager
        k = cv2.waitKey(1)
        if k == ord('q'):    # q key to stop
            print('quit')
            break
        elif k == ord('p'):  # p key to publish bboxes
            print('bboxes published: ' + str(len(list_bboxes)))
            create_and_publish_message(list_bboxes)
        elif k != 255:
            print('p: publish bboxes\nq: quit') # else print keys info
        					
        curr_detected = len(list_bboxes) #aggiorno la variabile curr_detected
        print('curr_detected1=',curr_detected)
        print('list_bboxes=',list_bboxes)
        
        #controllo se per un oggetto ho rilevato 2 bounding box che si intersecano o sono molto vicine
        if(len(list_bboxes) > 1):  #solo se ho rilevato piu di un elemento
            pop_elements = []  #inizializzo la lista di elementi da rimuovere
            add_rect = False
            #verifico intersezioni tra ogni elemento della lista
            for i in range(len(list_bboxes) - 1):
                for j in range(i+1,len(list_bboxes)):
                    #print('i=',i,'j=',j)
                    rect1 = [list_bboxes[i][0][0],list_bboxes[i][0][1],list_bboxes[i][1][0],list_bboxes[i][1][1]]
                    rect2 = [list_bboxes[j][0][0],list_bboxes[j][0][1],list_bboxes[j][1][0],list_bboxes[j][1][1]]
                    if (check_intersections(rect1,rect2)): #controllo se si intersecano
                        pop_elements.append(pop_element(rect1,rect2,i)) #Ho un elemento da rimuovere
                    elif (check_area_bboxes(rect1,rect2) != False): #se ho rilevato 2 aree adiacenti
                        pop_elements.append(i)
                        pop_elements.append(j) #rimuovo entrambi i rettangoli dalla lista
                        add_rect = True
                        new_rect = check_area_bboxes(rect1,rect2)
            if(len(pop_elements) > 0):
                #print('pop elements=',pop_elements)	  
                for i in reversed(pop_elements):
                    #print('///////////////////////////////////////////////////////////////////////////////////////////object has been popped from list',list_bboxes[i])
                    list_bboxes.pop(i) #rimuovo l'elemento dalla lista
                if(add_rect == True):
                    list_bboxes.append(new_rect) #alla fine aggiungo l'elemento alla lista
                    add_rect = False
                    
        #valutare anche l'inserimento di una funzione che se rileva due blocchi molto vicini li unisce insieme perche dalle prove a grandi velocita rileva sempre meta blocco superiore e inferiore come due 		#blocchi diversi pero in questo caso non sarebbe piu possibile posizionare due oggetti vicini

        #se sono nel primo frame
        if first_frame==1:
                detected = len(list_bboxes) #inizializzo la variabile detected con il numero di oggetti rilevati nel primo frame
                curr_detected = detected	
                if detected > 0:            #se ho rilevato degli oggetti allora invio il ros message contenente le coordinate dei vertici letti
                    pr.extend(list_bboxes)
                    create_and_publish_message(list_bboxes)
                    print('FIRST bboxes published: ' + str(len(list_bboxes)))		  
                    first_frame=0
                    continue #go to the next iteration
            
        
        curr_detected = len(list_bboxes) #aggiorno la variabile curr_detected
        #print('curr_detected2=',curr_detected)
        if(curr_detected == 5):
            print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@STOP@@@')
            print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@STOP@@@')
            print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@STOP@@@')
            print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@STOP@@@')
            print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@STOP@@@')
            break
        print('list_bboxes=',list_bboxes)
        print('DETECTED=',detected,'curr_detected=',curr_detected)
        if detected == 0 and curr_detected == 0:  #nel caso in cui entrambi sono a zero significa che sto continuando a non rilevare nulla quindi non invio nulla e vado alla prossima iterazione
            continue
        elif detected > 0 and curr_detected == 0:
            msg_to_send = BBoxes()	
            pub.publish(msg_to_send)
            #print('-----------------------------------------------------------Ho appena inviato un frame dopo non aver letto nulla)
            detected=curr_detected
            continue #go to the next iteration
        elif curr_detected > 0 and detected == 0:
            pr.extend(list_bboxes)
            create_and_publish_message(list_bboxes)
            #print('-----------------------------------------------------------Ho appena inviato un frame dopo aver letto di nuovo qualcosa)	
            #pub.publish(msg_to_send)
            detected=curr_detected
            #decommentare questa parte per fare prova senza rimozione di oggetti o per una versiona piu stabile da risolvere il flckering perche vengono mandati troppi ros message in questo caso   
        elif curr_detected != detected:	#questo mi serve perche se ho 1 oggetto e poi 2 capita che questo non viene rilevato finche non spostato perche senza questo controllo il ros messagge viene mandato
            pr.extend(list_bboxes)
            create_and_publish_message(list_bboxes)
            print('-----------------------------------------------------------Ho appena inviato un frame dopo aver letto un numero di oggetti diverso da quello precedente')	
            #pub.publish(msg_to_send)
            detected=curr_detected
            continue


            #se invece i due valori non sono uguali allora c'e stato un cambiamento nel numero di elementi letti
        #check if the new values are different more than a certain treshold from the previous ones
        for b,p in zip(list_bboxes,pr):
            diff = []
            for k in range(2):		  	
                    diff.append(tuple(map(lambda i, j: abs(i - j), b[k], p[k])))		    
                    
            if (diff[0][0] > treshold and diff[1][0] > treshold) or (diff[0][1] > treshold and diff[1][1] > treshold): #se una delle differenze sui valori tl e br hanno superato la soglia
                send_message=1
                                
        #se i valori sono cambiati oltre la certa soglia invio il messaggio e aggiorno la lista con i valori correnti
        if send_message==1:
            create_and_publish_message(list_bboxes)
            print('---------------------------------LOGGETTO E STATO SPOSTATO')
            print('bboxes published in loop: ' + str(len(list_bboxes)))
            #pub.publish(msg_to_send) #send the data
            send_message=0
            pr = []
            pr.extend(list_bboxes) #update the previous values with the new ones for the next iteration
        #print('detected=========',detected,curr_detected)
        detected = curr_detected #aggiorno il valore degli oggetti precedentemente rilevati
        
    cv2.destroyAllWindows()
    cap.release()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
