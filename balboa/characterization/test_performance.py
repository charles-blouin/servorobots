from a_star import AStar
a_star = AStar()
import time



delay = 0.01
time_a = time.time()

for i in range(0,10):
    time_a = time.time()
    a_star.leds(1, 0, 0)
    time.sleep(delay)
    a_star.leds(0, 1, 0)
    time.sleep(delay)
    a_star.leds(0, 0, 1)
    time.sleep(delay)
    print(time_a - time.time())
    time_a = time.time()