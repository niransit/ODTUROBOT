from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import cv2
import numpy as np

"""# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

sitl = None
"""
"""# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()"""

# Connect to the Vehicle
#vehicle = connect(connection_string, wait_ready=True)
vehicle = connect('/dev/ttyACM0', wait_ready = True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def camcam():
    cap = cv2.VideoCapture(0)

    font = cv2.FONT_HERSHEY_COMPLEX

    counterTimer = 0
    counter_O_H = 0
    counter_O_S = 0
    counter_STM = 0
    counter_RBT = 0
    counter_IL = 0
    while True:
        _, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        lower_red = np.array([160, 55, 55])
        upper_red = np.array([200, 255, 255])

        lower_green = np.array([60, 40, 40])
        upper_green = np.array([100, 255, 255])

        lower_blue = np.array([95, 90, 30])
        upper_blue = np.array([130, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        mask2 = cv2.inRange(hsv, lower_green, upper_green)
        mask3 = cv2.inRange(hsv, lower_blue, upper_blue)
        mask4 = cv2.inRange(gray, 140, 180)

        kernel = np.ones((5, 5), np.uint8)

        mask1 = cv2.erode(mask1, kernel)
        mask2 = cv2.erode(mask2, kernel)
        mask3 = cv2.erode(mask3, kernel)
        mask4 = cv2.erode(mask4, kernel)
        mask1 = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, kernel)
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, kernel)
        mask3 = cv2.morphologyEx(mask3, cv2.MORPH_CLOSE, kernel)

        # Contours detection
        if int(cv2.__version__[0]) > 3:
            # Opencv 4.x.x
            contours, _ = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours1, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours2, _ = cv2.findContours(mask3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours3, _ = cv2.findContours(mask4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        else:
            # Opencv 3.x.x
            _, contours, _ = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            _, contours1, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            _, contours2, _ = cv2.findContours(mask3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            _, contours3, _ = cv2.findContours(mask4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            if area > 400:
                cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

                if 5 <= len(approx) <= 6:
                    cv2.putText(mask1, "YARIM SEKIZGEN", (x, y), font, 1, (255, 0, 0))
                    counter_O_S += 1
                if 7 < len(approx) <= 10:
                    cv2.putText(mask1, "HILAL", (x, y), font, 1, (255, 0, 0))
                    counter_O_H += 1

        for cnt in contours1:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            if area > 400:
                cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

                if len(approx) >= 9:
                    cv2.putText(mask2, "BURDA BISEY VAR", (x, y), font, 1, (255, 0, 0))
                    counter_RBT += 1

        for cnt in contours2:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            if area > 400:
                cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

                if 9 < len(approx) <= 12:
                    cv2.putText(mask3, "M YI GORDUM", (x, y), font, 1, (255, 0, 0))
                    counter_STM += 1
                if len(approx) == 4:
                    cv2.putText(mask3, "4KOSELI", (x, y), font, 1, (255, 0, 0))
                    counter_STM += 1
                if len(approx) == 3:
                    cv2.putText(mask3, "Triangle", (x, y), font, 1, (255, 0, 0))
                    counter_STM += 1

        for cnt in contours3:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            if area > 400:
                cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

                if len(approx) >= 10:
                    cv2.putText(mask4, "Circle", (x, y), font, 1, (255, 0, 0))
                    counter_IL += 1
        counterTimer += 1

        cv2.imshow("Frame", frame)
        cv2.imshow("RedMask", mask1)
        cv2.imshow("GreenMask", mask2)
        cv2.imshow("BlueMask", mask3)
        cv2.imshow("WhiteMask", mask4)

        if counterTimer == 140:
            print("STM SAYACI=>" + str(counter_STM))
            print("ODTU SAYACI=>" + str(counter_O_S + counter_O_H))
            print("ROBOT SAYACI=>" + str(counter_RBT))
            print("INIS LOGOSU=>" + str(counter_IL))

            if (counter_STM > 95):
                cap.release()
                cv2.destroyAllWindows()
                return 1
            elif (counter_O_H + counter_O_S >= 147):
                cap.release()
                cv2.destroyAllWindows()
                return 2
            elif (counter_RBT > 100):
                cap.release()
                cv2.destroyAllWindows()
                return 3
            elif (counter_IL > 185):
                cap.release()
                cv2.destroyAllWindows()
                return 4
            else:
                cap.release()
                cv2.destroyAllWindows()
                return camcam()

        key = cv2.waitKey(1)
        if key == 27:
            cap.release()
            cv2.destroyAllWindows()
            break

# centre--39.8720560, 32.7321880
# solalt--39.872026, 32.732092
# solust--39.872104, 32.732081
# sagust--39.872109, 32.732278
# sagalt--39.872029, 32.732278
search_Array = [[0, 0], [0, 0], [0, 0], [0, 0]]

arm_and_takeoff(4)
vehicle.airspeed = 1
currentLocation = vehicle.location.global_relative_frame
print("SU AN BURDAYIM-->" + str(currentLocation))

p0 = LocationGlobalRelative(currentLocation.lat + 0.00002891423203, currentLocation.lon - 0.0000357251842, 4)
p1 = LocationGlobalRelative(currentLocation.lat + 0.00002891423203, currentLocation.lon + 0.0000357251842, 4)
p2 = LocationGlobalRelative(currentLocation.lat - 0.00002891423203, currentLocation.lon + 0.0000357251842, 4)
p3 = LocationGlobalRelative(currentLocation.lat - 0.00002891423203, currentLocation.lon - 0.0000357251842, 4)

def var_mi(searching):
    for i in range(4):
        if (search_Array[i][0] == searching):
            return i
    return 8


def search(searching, location, index):
    if (searching == 5):
        vehicle.simple_goto(currentLocation)
        time.sleep(10)
        print("Setting LAND mode...")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(3)
        print("BITTI GALIBAM GOREV")
        exit(0)
    if (search_Array[index][1] == 1):
        index = (index + 1) % 4
        if (index == 0):
            search(searching, p0, 0)

        elif (index == 1):
            search(searching, p1, 1)

        elif (index == 2):
            search(searching, p2, 2)

        elif (index == 3):
            search(searching, p3, 3)

    control = var_mi(searching)
    if (control != 8):
        if (control == 0):
            vehicle.simple_goto(p0)
            time.sleep(10)
            print(str(searching) + " LOGOSUNA INIS YAPIYORUM")
            print("Setting LAND mode...")
            vehicle.mode = VehicleMode("LAND")
            print("LANDLIYORUM")
            time.sleep(2)
            while (vehicle.armed):
                print("HALA ARMLIYIM")
                time.sleep(1)
            time.sleep(2)
            arm_and_takeoff(4)
            search_Array[control][1] = 1
            search(searching + 1, location, index)
        elif (control == 1):
            vehicle.simple_goto(p1)
            time.sleep(10)
            print(str(searching) + " LOGOSUNA INIS YAPIYORUM")
            print("Setting LAND mode...")
            vehicle.mode = VehicleMode("LAND")
            print("LANDLIYORUM")
            time.sleep(2)
            while (vehicle.armed):
                print("HALA ARMLIYIM")
                time.sleep(1)
            time.sleep(2)
            arm_and_takeoff(4)
            search_Array[control][1] = 1
            search(searching + 1, location, index)
        elif (control == 2):
            vehicle.simple_goto(p2)
            time.sleep(10)
            print(str(searching) + " LOGOSUNA INIS YAPIYORUM")
            print("Setting LAND mode...")
            vehicle.mode = VehicleMode("LAND")
            print("LANDLIYORUM")
            time.sleep(2)
            while (vehicle.armed):
                print("HALA ARMLIYIM")
                time.sleep(1)
            time.sleep(2)
            arm_and_takeoff(4)
            search_Array[control][1] = 1
            search(searching + 1, location, index)
        elif (control == 3):
            vehicle.simple_goto(p3)
            time.sleep(10)
            print(str(searching) + " LOGOSUNA INIS YAPIYORUM")
            print("Setting LAND mode...")
            vehicle.mode = VehicleMode("LAND")
            print("LANDLIYORUM")
            time.sleep(2)
            while(vehicle.armed):
                print("HALA ARMLIYIM")
                time.sleep(1)
            time.sleep(2)
            arm_and_takeoff(4)
            search_Array[control][1] = 1
            search(searching + 1, location, index)
    vehicle.simple_goto(location)
    time.sleep(9)
    catch = camcam()
    # 1 Logosu->STM
    # 2 Logosu->ODTU
    # 3 Logosu->ROBOT
    # 4 Logosu->INIS

    if (catch == searching):
        print(str(searching) + " LOGOSUNA INIS YAPIYORUM")
        print("Setting LAND mode...")
        vehicle.mode = VehicleMode("LAND")
        print("LANDLIYORUM")
        time.sleep(2)
        while (vehicle.armed):
            print("HALA ARMLIYIM")
            time.sleep(1)
        time.sleep(2)
        arm_and_takeoff(4)
        search_Array[index][0] = searching
        search_Array[index][1] = 1
        search(searching + 1, location, index)
    else:
        print(str(catch)+" Logosunu Kaydettim")
        search_Array[index][0] = catch
        index = (index + 1) % 4
        if (index == 0):
            search(searching, p0, 0)
        elif (index == 1):
            search(searching, p1, 1)
        elif (index == 2):
            search(searching, p2, 2)
        elif (index == 3):
            search(searching, p3, 3)


search(1, p0, 0)
