import datetime

import mysql.connector as mariadb
import sched
import time
import random
import datetime

s = sched.scheduler(time.time, time.sleep)
conn_params = {
    "user": "chris",
    "password": "1630",
    "host": "localhost",
    "database": "kRobot"
}
connection = mariadb.connect(**conn_params)
cursor = connection.cursor()
fmt = "%Y-%m-%d %H:%M:%S"

def do_something(sc):
    temp = random.randint(0, 255)
    now = datetime.datetime.now()
    now_str = datetime.datetime.strftime(now, fmt)
    statement = "INSERT INTO commander (itwoC, Targ, Data, Active, IsRead, AliveAt) VALUES (%s, %s, %s, %s, %s, %s)"
    data = (temp, temp, temp, True, False, now_str)
    cursor.execute(statement, data)
    connection.commit()
    print("Doing stuff...")
    # do your stuff
    sc.enter(1, 1, do_something, (sc,))  # self call by first param in seconds


# put your code here


# line below clean up
s.enter(1, 1, do_something, (s,))
s.run()

cursor.close()
connection.close()
