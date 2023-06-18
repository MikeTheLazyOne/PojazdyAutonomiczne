
#!/usr/bin/env python

import sys
import rospy
import time
import math
import termios
import tty
 
import sensor_msgs.msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from statistics import median


 

publisher = rospy.Publisher('/revised_scan', LaserScan, queue_size=10)
scann = LaserScan()


def Average(data):
    if len(data) == 0:
        return 0
    else:
        return sum(data) / len(data)


def callback(data):

    # sprawdzamy odległość do najbliższej prędkości z przodu pojazdu
    # jeśli jest ona mniejsza od 1.5m, przechodzimy do funkci move2, pozwalającej na omijanie przeszkód
    # w przeciwnym wypadku przechodzimy do funkcji move1 służącej do utrzymywania stałej odległości i równoległości od ściany  
    if (distance(data,0) + distance(data,1))/2 < 1.5:
        move2(data)
    else:
        move1(data)


 #funkcji służąca do utrzymywania stałej odległości i równoległości od ściany  
def move1(data):

    #sprawdzamy w jakiej odległości od lewej sciany jest nasz pojazd(prostopadle) 
    distance_left = (distance(data,9) +distance(data,10))/2
    # sprawdzamy odległości od ściany pod względem 20 stopni wzlędem normalnej
    distance_left_forward =  (distance(data,7) +distance(data,8))/2
    distance_left_back = (distance(data,11) +distance(data,12))/2

    # odległość jaką nasz pojazd powinnien utrzymywać od ściany
    reference_distance = 0.65

    #uchyb wzlędem odległości od ściany
    e_dist_left = (distance_left-reference_distance)
    # uchyb względem równoległości -> jeśli odległość pod kątem 250 stopni jest równa odległości pod kątem 290
    # oznacza to że nasz pojazd jedzie równolegle do ściany i uchyb jest zerowy 
    e_dist_parallel = (distance_left_forward-distance_left_back)



    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist = Twist()

    # regulatory PI używane do utrzymanie raównoległości oraz odległości od ściany
    Pi1.Calc(e_dist_left)
    Pi2.Calc(e_dist_parallel)


    twist.linear.x = 1
    twist.angular.z = (2*Pi2.get_y() +Pi1.get_y())/3

    pub.publish(twist)

#funkcja służąca do omijania przeszkód
def move2(data):

    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist = Twist()

    # odległości od przeszkód pod katąmi (30-60) stopni oraz (300-330) stopni
    # pozwala to stwierdzić z której strony jest więcej miejsca i skręcic w tamtą stronę
    distance_left_avg = (distance(data,4) + distance(data,5) + distance(data,6))/3
    distance_right_avg = (distance(data,33) + distance(data,32) + distance(data,31) )/3

 
    # jeśli odległość z przodu jest zbyt mała (poniżej 25 cm), pojazd wycofa się
    if (distance(data,0) + distance(data,1))/2 < 0.25:
        if distance_left_avg > distance_right_avg:
            twist.linear.x = -0.5
            twist.angular.z = -0.1

        else:
            twist.linear.x = -0.5
            twist.angular.z = 0.1

 
    # w przeciwnym wypadku pojazd pojedzie do przodu skręcając w stronę z której jest więcej miejsca
    else:
        if distance_left_avg > distance_right_avg:
            twist.linear.x = 2
            twist.angular.z = 1 
        else:
            twist.linear.x = 2
            twist.angular.z = -1

    pub.publish(twist)

 

class PI():
    def __init__(self,Kp,Ki):
        self.x = 0
        self.x_prev = 0
        self.y_sum = 0
        self.y_int = 0
        self.y = 0
        self.y_prev = 0
        self.Kp = Kp
        self.Ki = Ki
        self.time_prev = time.time()
        self.Ts = 0

 

    def Calc(self, x):
        self.x = x
        # sprawdzamy czas który upłynął od ostatniego pomiaru
        self.Ts = time.time() - self.time_prev
        self.time_prev = time.time()

        # implementacja członu proporcjonalnego
        self.y_sum = self.x * self.Kp - self.x_prev * self.Kp
        #implementacja członu całkującego oraz anto-windupa
        # jeśli w poprzedniej chwili czasowej wyjście regulatora było większe niż 1, wyłączamy człon całkujący
        if (self.y_prev > 1 or self.y_prev<-1):
            self.y_int = 0
        else:
            self.y_int = self.Ts * self.x * self.Ki / 2 + self.Ts * self.x_prev * self.Ki / 2 + self.y_prev
        # wyjściem regulatora jest suma członu całkującego i proporcjonalnego
        self.y = self.y_sum + self.y_int
        self.y_prev = self.y

 
        if self.y > 1:
            self.y = 1
        if self.y < -1:
            self.y = -1

        self.x_prev = x

    #funkcja dostępowa regulatora PI
    def get_y(self):
        return self.y

 
# funkcja służaca do pomiaru odległości pod zadanym kątem
def distance(data, angle_ref):
    n = len(data.ranges)
    angle = len(data.ranges) / math.pi

    distance = []
    if angle_ref == 0:
        for i in range(0, n - 1):
            if (i > 35 * n / 36) and (i < n):
                a = data.ranges[i]
                b = data.ranges[i + 1]
                new = (a * b * math.sin(angle)) / (math.sqrt(a * a + b * b - 2 * a * b * math.cos(angle)))
                if not math.isnan(new):
                    distance.append(new)
    else:
        for i in range(0, n - 1):
            if (i > (angle_ref - 1) * n / 36) and (i < angle_ref * n / 36):
                a = data.ranges[i]
                b = data.ranges[i + 1]
                new = (a * b * math.sin(angle)) / (math.sqrt(a * a + b * b - 2 * a * b * math.cos(angle)))
                if not math.isnan(new):
                    distance.append(new)

    # liczenie średniej odległości od przeszków dla zadanego kąta
    # już nie używana
    dist_avg = Average(distance)


    if len(distance) ==0:
        return 0
    # wartością zwracaną przez funkcję jest mediana odległości dla zadanego kąta
    dist_median = median(distance)

    return dist_median

 

def listener():
    rospy.init_node('revised_scan', anonymous=True)
    subscriber = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

 
# inicjalizacja regulatorów 
Pi1 = PI(1.3, 0.1)
Pi2 = PI(0.9 ,0.1)


def main(args):
    listener()

 
if __name__ == '__main__':
    main(sys.argv)