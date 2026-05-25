import sys
import os
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
        
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


from st3215.scservo_sdk import *


def main():
    print("Start programu!") 
    SCS_ID                      = 2                 
    BAUDRATE                    = 1000000           
    DEVICENAME                  = '/dev/ttyACM0'    
    SCS_MINIMUM_POSITION_VALUE  = 0           
    SCS_MAXIMUM_POSITION_VALUE  = 1000
    SCS_MOVING_SPEED            = 4000       
    SCS_MOVING_ACC              = 5   
    scs_target_speed = [4000, 0]     
    index = 0 # Upewnij się, że index też jest zdefiniowany przed pętlą
    
    TIMEOUT_SECONDS             = 3.0 # Maksymalny czas w sekundach, jaki dajemy serwu na wykonanie ruchu

    index = 0
    scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]         
    portHandler = PortHandler(DEVICENAME)
    packetHandler = sms_sts(portHandler)
        
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        getch()
        quit()

    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        getch()
        quit()

    for i in range(3):
        ID_C = i+1
        # 1. Odblokuj pamięć EPROM
        packetHandler.unLockEprom(ID_C)

        # 2. Ustaw tryb pozycyjny (zapisz 0 do rejestru SMS_STS_MODE)
        packetHandler.write1ByteTxRx(ID_C, SMS_STS_MODE, 1)

        # 3. Zablokuj ponownie pamięć EPROM
        packetHandler.LockEprom(ID_C)
############################################################################################

    while 1:
        print("\nPress any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            break
            
        for i in range(3):
            servo_id = i + 1
            print(f"--- Uruchamiam serwo ID: {servo_id} ---")
            
            # Zmiana: Użycie funkcji WriteSpec do nadania prędkości zamiast WritePosEx
            # Zakładam, że masz zdefiniowaną tablicę scs_target_speed[index] (np. [1000, -1000])
            scs_comm_result, scs_error = packetHandler.WriteSpec(servo_id, scs_target_speed[index], SCS_MOVING_ACC)
            
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % packetHandler.getRxPacketError(scs_error))

            # Rejestrujemy czas startu ruchu
            start_time = time.time()

            # Pętla monitorująca dla konkretnego serwa
            while 1:
                is_stopped = False 
                
                # Zabezpieczenie: Timeout (jeśli mija za dużo czasu, uciekamy z pętli)
                if (time.time() - start_time) > TIMEOUT_SECONDS:
                    print(f"[OSTRZEŻENIE] Timeout! Serwo ID {servo_id} utknęło lub kręci się w kółko. Przerywam oczekiwanie.")
                    break # Wychodzimy z pętli while, idziemy do następnego serwa

                scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(servo_id)
                
                if scs_comm_result == COMM_SUCCESS:
                    # Wyświetla pozycję i prędkość (zmieniono logowanie celu z Goal na GoalSpd)
                    print("[ID:%03d] GoalSpd:%d PresPos:%d PresSpd:%d" % (servo_id, scs_target_speed[index], scs_present_position, scs_present_speed))
                
                # Odczyt statusu ruchu
                moving, scs_comm_result, scs_error = packetHandler.ReadMoving(servo_id)
                
                if scs_comm_result == COMM_SUCCESS:
                    if moving == 0:
                        is_stopped = True 
                
                if is_stopped:
                    print(f"Serwo ID: {servo_id} zatrzymało się.")
                    break 
                    
                time.sleep(0.01)

##################################################################################################

        if index == 0:
            index = 1
        else:
            index = 0

    portHandler.closePort()

if __name__ == "__main__":
    main()