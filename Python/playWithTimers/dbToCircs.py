import mysql.connector as mariadb
import sched
import time
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
    print("scanning")
    connection.commit()

    statement = "SELECT ref, itwoC, Targ, Data, IsRead  FROM commander WHERE Active=1 LIMIT 1"
    cursor.execute(statement)
    records = cursor.fetchall()
    print(records)
    #print(len(records))
    if len(records) > 0:
        # there is an active command in cursor (top one!)
        for ref, itwoC, Targ, Data, IsRead in records:
            print(f"ref: {ref} itwoc: {itwoC}, Targ:{Targ}, Data:{Data}, IsRead:{IsRead}")

            # values below cmd_ need to be passed via i2C rountines
            cmd_Ref = ref
            cmd_itwoC = itwoC
            cmd_Targ = Targ
            cmd_Data = Data
            cmd_IsRead = IsRead

            # lets assume the i2c stuff is done now ...
            # deactivate this active command
            now = datetime.datetime.now()
            now_str = datetime.datetime.strftime(now, fmt)
            statement = f"UPDATE commander SET Active=False, DontAT='{now_str}' WHERE Ref={cmd_Ref}"
            cursor.execute(statement)
            connection.commit()
            print (f"Ref:{cmd_Ref} is reset")
    else:
        print("Nowt to do")



    sc.enter(1, 1, do_something, (sc,))  # self call by first param in seconds


# put your code here


# line below clean up
s.enter(1, 1, do_something, (s,))
s.run()

cursor.close()
connection.close()
