import cv2, os, traceback, numpy as np, time, pyzbar.pyzbar, stack, math, requests, socket, sys, struct, pickle, threading, datetime
from multiprocessing.pool import ThreadPool
import json

class Controller():

    def __init__(self):
        self.auto_pilot = True
        self.in_flight = False
        self.right = False
        self.left = False
        self.up = None
        self.is_translating = False
        self.ignore_depth = False
        self.roi = (250, 450)
        self.current = None
        self.shelf_counter = 0
        self.completed = False
        self.item_dict = {}
        self.call_stack = stack.Stack()
        self.pool = ThreadPool(processes=1)
        self.current_movement = None
        self.angle = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.number_frames = 0
        self.fps = 30
        self.out = cv2.VideoWriter('appsrc ! videoconvert ! x264enc noise-reduction=10000 speed-preset=ultrafast tune=zerolatency ! rtph264pay config-interval=1 pt=96 !tcpserversink host=192.168.1.2 port=5000 sync=false', 0, self.fps, (960,
                                                                                                                                                                                                                                            720))
        config = None
        with open('config.json') as (f):
            config = json.load(f)
        HOST = config['controller']
        PORT = 8888
        CMD_HOST = config['warehouse']
        CMD_PORT = 8889
        try:
            self.sock.bind((HOST, PORT))
        except socket.error as msg:
            print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
            sys.exit()

        self.sock.listen(10)
        while 1:
            conn, addr = self.sock.accept()
            print 'Connected with ' + addr[0] + ':' + str(addr[1])
            self.cmd_sock.connect((CMD_HOST, CMD_PORT))
            break

        self.main(conn)
        return

    def current_milli_time(self):
        return int(round(time.time() * 1000))

    def track_qr_code(self, img):
        qr_codes = pyzbar.decode(img)
        area = None
        if qr_codes != []:
            for qr in qr_codes:
                points = qr.polygon
                if qr.type == 'QRCODE':
                    area = qr.rect[2] * qr.rect[-1]
                font = cv2.FONT_HERSHEY_SIMPLEX
                with open('shelf.json') as (f):
                    shelf = json.load(f)
                    if qr.data not in shelf['shelf1'] and qr.data not in shelf['shelf2']:
                        cv2.putText(img, qr.data, (points[0][1] - 50, points[0][1] - 50), font, 0.7, (0,
                                                                                                      255,
                                                                                                      0), 2, cv2.LINE_AA)
                    if len(points) > 4:
                        hull = cv2.convexHull(np.array([ point for point in points ], dtype=np.float32))
                        hull = list(map(tuple, np.squeeze(hull)))
                    else:
                        hull = points
                    n = len(hull)
                    for j in range(0, n):
                        cv2.line(img, hull[j], hull[(j + 1) % n], (0, 0, 255), 3)

                    if qr.data == shelf['shelf1'][-1] and self.up == None and points[0][0] <= 600 and self.right:
                        self.right = False
                        self.up = True
                        self.ignore_depth = True
                        self.shelf_counter += 1
                    else:
                        if qr.data == shelf['shelf2'][0] and self.up == None and points[0][0] >= 600 and not self.right and self.left:
                            self.left = False
                            self.up = True
                            self.ignore_depth = True
                            self.shelf_counter += 1
                        else:
                            if qr.data == 'com' and self.up == None and points[0][0] <= 400:
                                self.right = False
                                self.left = False
                                self.up = None
                                self.completed = True

        return (
         img, area, qr_codes)

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))

    def adjust_angle(self, drone, hsv, image):
        green_lower = (55, 55, 100)
        green_upper = (90, 207, 200)
        mask = cv2.inRange(hsv, green_lower, green_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
        if lines is not None:
            for line in lines:
                for rho, theta in line:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * -b)
                    y1 = int(y0 + 1000 * a)
                    x2 = int(x0 - 1000 * -b)
                    y2 = int(y0 - 1000 * a)
                    cv2.line(image, (x1, y1), (x2, y2), (127, 0, 255), 2)

        return image

    def scan_item(self, qr0, items_arr):
        if items_arr == []:
            r = requests.post('http://localhost:3000/addItem', data={'bin': qr0.data, 'items': ''})
        else:
            r = requests.post('http://localhost:3000/addItem', data={'bin': qr0.data, 'items': items_arr})

    def print_items(self):
        print self.item_dict

    def thread_processing(self, conn, image):
        distance = 0
        area = None
        angle = None
        qr_codes = []
        with open('shelf.json') as (f):
            shelf = json.load(f)
            if not self.in_flight:
                self.current = self.current_milli_time()
                self.in_flight = True
                self.right = True
                self.shelf_counter += 1
                self.cmd_sock.send('takeoff')
            if self.up == None:
                image, area, qr_codes = self.track_qr_code(image)
            if qr_codes != []:
                qr0 = qr_codes[-1]
                qr1 = qr_codes[len(qr_codes) - 2]
                bins = []
                items = []
                for i in range(0, len(qr_codes)):
                    if qr_codes[i].data not in shelf['shelf1'] and qr_codes[i].data not in shelf['shelf2']:
                        items.append(qr_codes[i])
                    else:
                        bins.append(qr_codes[i])

                if bins != []:
                    for b in bins:
                        if items != []:
                            items_arr = []
                            for item in items:
                                pts0 = b.polygon[0]
                                pts1 = item.polygon[0]
                                if pts0 != None and pts1 != None:
                                    angle = math.atan2(pts1[1] - pts0[1], pts1[0] - pts0[0])
                                    angle = math.degrees(angle)
                                    dist = self.euclidean_distance(pts0[0], pts0[1], pts1[0], pts1[1])
                                    if self.angle == None:
                                        self.angle = angle
                                    if angle <= -60 and angle >= -135 and dist <= 250:
                                        cv2.line(image, pts0, pts1, (255, 0, 0), 2)
                                        if self.angle != None:
                                            cv2.putText(image, ('Angle: {}').format(self.angle), (200,
                                                                                                  700), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,
                                                                                                                                      127,
                                                                                                                                      255), 2)
                                        items_arr.append(item.data)

                            if b.data not in self.item_dict or self.item_dict[b.data] < len(items_arr):
                                self.item_dict[b.data] = len(items_arr)
                                self.scan_item(b, items_arr)
                        elif b.data not in self.item_dict:
                            self.scan_item(b, [])
                            self.item_dict[b.data] = 0

                if (qr0.data in shelf['shelf1'] or qr0.data in shelf['shelf2']) and (qr1.data in shelf['shelf1'] or qr1.data in shelf['shelf2']):
                    pts0 = qr0.polygon[0]
                    pts1 = qr1.polygon[0]
                    if pts0 != None and pts1 != None:
                        cv2.line(image, pts0, pts1, (255, 0, 0), 2)
                        distance = self.euclidean_distance(pts0[0], pts0[1], pts1[0], pts1[1])
                        cv2.putText(image, ('Distance: {}').format(distance), (pts0[0] + 20, pts0[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,
                                                                                                                                            0,
                                                                                                                                            0), 2)
            if self.current_milli_time() - self.current >= 3000 and self.auto_pilot:
                if self.completed or self.shelf_counter > 2:
                    self.cmd_sock.send('land')
                    self.in_flight = False
                    return
                if area is not None and area > 10000 and not self.is_translating and not self.ignore_depth:
                    print 'backward'
                    self.cmd_sock.send('backward')
                    self.current = self.current_milli_time()
                    self.is_translating = True
                    self.current_movement = 'Moving Backward'
                else:
                    if area is not None and area < 7000 and not self.is_translating and not self.ignore_depth:
                        print 'forward'
                        self.cmd_sock.send('forward')
                        self.current = self.current_milli_time()
                        self.is_translating = True
                        self.current_movement = 'Moving Forward'
                if distance != None and self.is_translating and self.right and self.up == None:
                    print 'right'
                    self.cmd_sock.send('tellopy_right')
                    self.call_stack.push('right')
                    self.current = self.current_milli_time()
                    self.is_translating = False
                    self.current_movement = 'Moving Right'
                elif not self.right and self.up == None and self.left:
                    print 'left'
                    self.cmd_sock.send('tellopy_left')
                    self.call_stack.push('left')
                    self.current = self.current_milli_time()
                    self.is_translating = False
                    self.current_movement = 'Moving Left'
                elif not self.right and not self.left and self.up:
                    print 'up'
                    self.cmd_sock.send('up')
                    self.is_translating = False
                    self.up = None
                    if self.call_stack.peek() == 'right':
                        self.left = True
                    else:
                        if self.call_stack.peek() == 'left':
                            self.right = True
                    self.ignore_depth = False
                    self.call_stack.push('up')
                    self.current = self.current_milli_time()
                    self.current_movement = 'Moving Forward'
        return image

    def main(self, conn):
        data = ''
        payload_size = struct.calcsize('>L')
        print ('payload_size: {}').format(payload_size)
        while True:
            while len(data) < payload_size:
                data += conn.recv(4096)

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack('>L', packed_msg_size)[0]
            while len(data) < msg_size:
                data += conn.recv(4096)

            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = pickle.loads(frame_data)
            image = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            image = self.thread_processing(conn, image)
            if self.current_movement != None:
                cv2.putText(image, self.current_movement, (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,
                                                                                                     0,
                                                                                                     255), 2)
            if self.auto_pilot == False:
                cv2.putText(image, 'Switched to manual mode.', (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,
                                                                                                         0,
                                                                                                         255), 2)
            cv2.imshow('Video', image)
            key = cv2.waitKey(1)
            if key == 27:
                break
            if key == 32:
                self.cmd_sock.send('hover')
                self.auto_pilot = not self.auto_pilot
            if self.auto_pilot == False:
                if key == 65 or key == 97:
                    self.cmd_sock.send('left')
                elif key == 87 or key == 119:
                    self.cmd_sock.send('forward')
                elif key == 68 or key == 100:
                    self.cmd_sock.send('right')
                elif key == 83 or key == 115:
                    self.cmd_sock.send('backward')
                elif key == 81 or key == 113:
                    self.cmd_sock.send('counter-clockwise')
                elif key == 69 or key == 101:
                    self.cmd_sock.send('clockwise')
                elif key == 67 or key == 99:
                    self.cmd_sock.send('up')
                elif key == 86 or key == 118:
                    self.cmd_sock.send('down')

        if self.in_flight:
            self.cmd_sock.send('land')
        self.sock.close()
        self.cmd_sock.close()
        self.out.release()
        return


if __name__ == '__main__':
    controller = Controller()