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
    #print("scanning")
    connection.commit()

    statement = "SELECT ref, itwoC, Targ, Data, IsRead, AliveAt, DontAT  FROM commander WHERE Active=0 LIMIT 1"
    cursor.execute(statement)
    records = cursor.fetchall()
    #print(records)
    #print(len(records))
    if len(records) > 0:
        # there is an active command in cursor (top one!)
        for ref, itwoC, Targ, Data, IsRead, AliveAt, DontAT in records:
            print(f"ref: {ref} itwoc: {itwoC}, Targ:{Targ}, Data:{Data}, IsRead:{IsRead}, AliveAt:{AliveAt}, DontAT:{DontAT}")

            # values below cmd_ need to be archived
            cmd_Ref = ref
            cmd_itwoC = itwoC
            cmd_Targ = Targ
            cmd_Data = Data
            cmd_IsRead = IsRead
            cmd_AliveAt = AliveAt
            cmd_DoneAt = DontAT  #typo here to fix in all areas CK

        print("archiving")
        now = datetime.datetime.now()
        now_str = datetime.datetime.strftime(now, fmt)
        statement = "INSERT INTO DoneIts (ItwoC, Targ, Data, IsRead, AliveAt, DoneAt, ArchAt) VALUES (%s, %s, %s, %s, %s, %s, %s)"
        data = (cmd_itwoC, cmd_Targ, cmd_Data, cmd_IsRead, cmd_AliveAt, cmd_DoneAt, now_str)
        cursor.execute(statement, data)
        connection.commit()

        statement = f"DELETE FROM commander WHERE Ref={cmd_Ref}"
        cursor.execute(statement)
        connection.commit()
    else:
        print("No archive action")








    sc.enter(2, 1, do_something, (sc,))  # self call by first param in seconds









# line below clean up
s.enter(1, 1, do_something, (s,))
s.run()

cursor.close()
connection.close()
