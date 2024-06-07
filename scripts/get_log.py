import sys
import os

import time

def get_data_from_serial(serial = "/dev/ttyUSB0", baud=""):
    from serial import Serial
    input("Press Enter to continue...")
    if(baud == ""):
        ser = Serial(serial)   
    else:     
        ser = Serial(serial, baud)
    log = ""
    
    path = f"output/{time.asctime()}.txt"
    with open(path, "+w") as f:
        try:
            while True:
                data = str(ser.read_all().decode("utf-8"))
                f.write(data)
                print(data,end="")
                # if "END" in data:
                #     break
        except KeyboardInterrupt:
            
            print("KeyboardInterrupt")
            
        finally:
            f.close()
            ser.close()
            clean(path)

"""
state:
    INIT,  //initializing system
    SUSPENDED,  //standby while drone is flying landing gear is fixed in default position
    ACTIVE,     //landing gear is moving dynamically
    DESCENDING, //landing gear is calculating the terrain and adjust its height
    TOUCHDOWN, //landing gear is touching the ground, moving dyanmically using IMU
    STANDBY, //landing gear is in standby position after touchdown
"""
def clean(path):
    log_name = [
        "LOG_IMU",
        "LOG_SERVO",
        "LOG_PID",
        "LOG_LIDAR",
        "LOG_FK",
        "LOG_ADC",
        "LOG_RAW_ADC",
        "LOG_STATE",
    ]
    clean_data = ["", "", "", "", "", "", "",""]
    with open(path, "r") as f:
        while True:
            log = f.readline()
            if log == "":
                break

            print(log)

            for i, name in enumerate(log_name):
                if name in log:
                    if log.find("T,") == -1:
                        clean_data[i] += log[log.index("(") + 1 : log.index(")")]
                        clean_data[i] += ","

                    # print(
                    #     f"name {name} pos:{log.index(name)},len {len(name) + 2}, {len(name) + 2+log.index(name)} "
                    # )
                    clean_data[i] += log[log.index(name) + len(name) + 2 : -5]
                    clean_data[i] += "\n"

            # if "END" in log:
            #     break

        f.close()

        for i, name in enumerate(log_name):
            if clean_data[i] != "":
                write = open(path[:-4] + f"_{name}.csv", "w")
                write.write(clean_data[i])
                write.close()

if __name__ == "__main__":

    get_data_from_serial(baud=115200)
    # clean("output/Wed Jun  5 17:14:55 2024.txt")
    # clean("scripts/output/Fri Jun  7 11:56:16 2024.txt")
    # data = ""
    # name = "LOG_STATE"
    # log = "[0;32mI (6918) LOG_STATE: T,state[0m\n"
    # # data += log[log.index("(") + 1 : log.index(")")]
    # # data += ","
    # data += log[log.index(name) + len(name) + 2 : -5]
    # data += "\n"
    # print(data)
