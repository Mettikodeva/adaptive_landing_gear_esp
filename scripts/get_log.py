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
                if "END" in data:
                    break
        except KeyboardInterrupt:
            
            print("KeyboardInterrupt")
            
        finally:
            f.close()
            ser.close()
            clean(path)


def clean(path):
    log_name = ["LOG_IMU", "LOG_SERVO", "LOG_PID", "LOG_LIDAR", "LOG_FK", "LOG_ADC"]
    clean_data = ["", "", "", "", "", ""]
    with open(path, "r") as f:
        while True:
            log = f.readline()
            if log == "":
                break
                
            print(log)

            for i, name in enumerate(log_name):
                if name in log:
                    if "T" not in log:
                        clean_data[i] += log[log.index("(") + 1 : log.index(")")]
                        clean_data[i] += ","

                    clean_data[i] += log[log.index(name) + len(name) + 2 : -5]
                    clean_data[i] += "\n"

            if "END" in log:
                break

        f.close()

        for i, name in enumerate(log_name):
            if clean_data[i] != "":
                write = open(path[:-4] + f"_{name}.csv", "w")
                write.write(clean_data[i])
                write.close()

if __name__ == "__main__":
    
    # get_data_from_serial(baud=115200)
    clean("output/outputWed Jun  5 01:34:55 2024.txt")
  