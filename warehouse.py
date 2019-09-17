import tello, cv2, os, av, sys, traceback, numpy as np, time, pyzbar.pyzbar, stack, math, requests, socket, struct, pickle, threading, datetime, json, argparse
auto_pilot = True
in_flight = False
right = False
left = False
up = None
is_translating = False
ignore_depth = False
roi = (250, 450)
current = None
current_milli_time = lambda : int(round(time.time() * 1000))

def hover(drone):
    drone.tellopy_left(0)
    drone.tellopy_right(0)
    drone.tellopy_forward(0)
    drone.tellopy_backward(0)


def main(drone, sock, conn):
    global auto_pilot
    encode_param = [
     int(cv2.IMWRITE_JPEG_QUALITY), 90]
    container = av.open(drone.get_video_stream())
    frame_skip = 300
    for frame in container.decode(video=0):
        if 0 < frame_skip:
            frame_skip = frame_skip - 1
            continue
        start_time = time.time()
        image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
        result, f = cv2.imencode('.jpg', image, encode_param)
        data = pickle.dumps(f, 0)
        size = len(data)
        sock.sendall(struct.pack('>L', size) + data)
        if not auto_pilot:
            cv2.putText(image, 'Switched to manual mode', (200, 700), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,
                                                                                                    127,
                                                                                                    255), 2)
        frame_skip = int((time.time() - start_time) / frame.time_base)


def _cmd_thread(cmd_sock, conn, drone, speed):
    global in_flight
    while True:
        cmd = conn.recv(2048)
        if auto_pilot:
            if 'takeoff' == cmd:
                drone.takeoff()
                in_flight = True
            elif 'up' == cmd:
                hover(drone)
                drone.up(80)
            elif 'tellopy_left' == cmd:
                hover(drone)
                drone.tellopy_left(speed)
            elif 'left' == cmd:
                hover(drone)
                drone.left(20)
            elif 'tellopy_right' == cmd:
                hover(drone)
                drone.tellopy_right(speed)
            elif 'right' == cmd:
                hover(drone)
                drone.right(20)
            elif 'backward' == cmd:
                hover(drone)
                drone.backward(20)
            elif 'forward' == cmd:
                hover(drone)
                drone.forward(20)
            elif 'clockwise' == cmd:
                hover(drone)
                drone.clockwise(20)
            elif 'counter-clockwise' == cmd:
                hover(drone)
                drone.counter_clockwise(20)
            elif 'hover' == cmd:
                hover(drone)
            elif 'land' == cmd:
                drone.land()
                in_flight = False
                break


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('-s', '--speed', type=int, help='Specify the drone speed (less than 100).', default=5)
    args = vars(ap.parse_args())
    drone = tello.Tello()
    config = None
    with open('config.json') as (f):
        config = json.load(f)
    speed = args['speed']
    if speed == None:
        speed = 5
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((config['controller'], 8888))
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    HOST = config['warehouse']
    PORT = 8889
    conn = None
    t = None
    try:
        cmd_sock.bind((HOST, PORT))
    except socket.error as msg:
        print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()

    cmd_sock.listen(1)
    while 1:
        conn, addr = cmd_sock.accept()
        print 'Connected with ' + addr[0] + ':' + str(addr[1])
        break

    try:
        drone.connect()
        drone.wait_for_connection(60.0)
        if conn != None:
            t = threading.Thread(name='cmd_thread', target=_cmd_thread, args=(cmd_sock, conn, drone, speed))
            t.setDaemon(True)
            t.start()
        main(drone, sock, conn)
    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print ex
    finally:
        if in_flight:
            drone.land()
        drone.quit()
        sock.close()
        cmd_sock.close()