# chapter 28 in python

# sudo apt-get install python3-pip
# python3 -m pip install pyserial
# sudo apt-get install python3-matplotlib

import serial
# import signal
# from statistics import mean
import matplotlib.pyplot as plt

from genref import genRef

# # signal handler
# def handler(signum, frame):
#     print("\nGracefully Exiting via Ctrl-c (closing serial port)")
#     ser.close()
#     exit(0)
 
# signal.signal(signal.SIGINT, handler)

ser = serial.Serial('/dev/tty.usbserial-1140',230400)
print('Opening port: ')
print(ser.name)

mode_map = {
    0: "IDLE",
    1: "PWM",
    2: "ITEST",
    3: "HOLD",
    4: "TRACK"
}


 
# def read_plot_matrix():
#     n_str = ser.read_until(b'\n');  # get the number of data points to receive
#     n_int = int(n_str) # turn it into an int
#     print('Data lengeth = ' + str(n_int))
#     ref = []
#     data = []
#     data_received = 0
#     while data_received < n_int:
#         dat_str = ser.read_until(b'\n');  # get the data as a string, ints seperated by spaces
#         dat_int = list(map(int,dat_str.split())) # now the data is a list
#         ref.append(dat_int[0])
#         data.append(dat_int[1])
#         data_received = data_received + 1
#     meanzip = zip(ref,data)
#     meanlist = []
#     for i,j in meanzip:
#         meanlist.append(abs(i-j))
#     score = mean(meanlist)
#     t = range(len(ref)) # index array
#     plt.plot(t,ref,'r*-',t,data,'b*-')
#     plt.title('Score = ' + str(score))
#     plt.ylabel('value')
#     plt.xlabel('index')
#     plt.show()

try:
    has_quit = False
    # menu loop
    while not has_quit:
        print('PIC32 MOTOR DRIVER INTERFACE')
        # display the menu options this list will grow
        
        # print('\tb: Read current sensor [mA]')
        # print('\tc: Read Encoder [counts] \td: Read Encoder [deg]')
        # print('\te: Reset Encoder \tf: Set PWM [-100 to 100]')
        # print('\tg: Set Current Gains \th: Get Current gains')
        # # print('\ti: Set Position gains \tj: Get Position Gains')
        # print('\tk: Test Current Control \tl: Go To Angle [deg]')
        # # print('\tm: Load Step Trajectory \tn: Load Cubic Trajectory')
        # # print('\to: Execute Trajectory \tp: Unpower the motor')
        # print('\tq: Quit Client \tr: Get Mode')
        
        menu_width = 35  # Set a width for the menu items

        print(f"{'b: Read current sensor [mA]':<{menu_width}}{'c: Read Encoder [counts]':<{menu_width}}")
        print(f"{'d: Read Encoder [deg]':<{menu_width}}{'e: Reset Encoder':<{menu_width}}")
        print(f"{'f: Set PWM [-100 to 100]':<{menu_width}}{'g: Set Current Gains':<{menu_width}}")
        print(f"{'h: Get Current gains':<{menu_width}}{'i: Set Position gains':<{menu_width}}")
        print(f"{'j: Get Position Gains':<{menu_width}}{'k: Test Current Control':<{menu_width}}")
        print(f"{'l: Go To Angle [deg]':<{menu_width}}{'m: Load Step Trajectory':<{menu_width}}")
        print(f"{'n: Load Cubic Trajectory':<{menu_width}}{'o: Execute Trajectory':<{menu_width}}")
        print(f"{'p: Unpower the motor':<{menu_width}}{'q: Quit Client':<{menu_width}}")
        print(f"{'r: Get Mode':<{menu_width}}")

        
        # read the user's choice
        selection = input('\nENTER COMMAND: ')
        selection_endline = selection+'\n'
        
        # send the command to the PIC32
        ser.write(selection_endline.encode()) # .encode() turns the string into a char array
        
        
        # EXAMPLE
        # if (selection == 'd'):
        #     # example operation
        #     n_str = input('Enter number: ') # get the number to send
        #     n_int = int(n_str) # turn it into an int
        #     print('number = ' + str(n_int)) # print it to the screen to double check

        #     ser.write((str(n_int)+'\n').encode()) # send the number
        #     n_str = ser.read_until(b'\n')  # get the incremented number back
        #     n_int = int(n_str) # turn it into an int
        #     print('Got back: ' + str(n_int) + '\n') # print it to the screen
        
        # take the appropriate action
        if selection == "b":
            # gets cur_current
            cur_current = ser.read_until(b'\n').decode()
            print(cur_current)
            cur_current_float = float(cur_current)
            print("Current [mA]: ", cur_current_float)
        elif selection == "c":
            # read encoder counts
            enc_cnts_str = ser.read_until(b'\n')
            enc_cnts_int = int(enc_cnts_str)
            print("Encoder count: ", enc_cnts_int)
        elif selection == "d":
            # read encoder degree
            enc_cnts_str = ser.read_until(b'\n')
            enc_cnts_int = float(enc_cnts_str)
            print("Encoder count: ", enc_cnts_int)
        elif selection == "e":
            # reset encoder
            print("Encoder count reset to 0")
        elif selection == "f":
            # drive PWM @ DutyCycle + direc
            pwm_str = input('Enter PWM [-100, 100]: ') # get the number to send
            pwm_int = int(pwm_str)
            ser.write((str(pwm_int)+'\n').encode()) # send the PWM
            print("Should see motor spinning in direc at PWM!")
        elif selection == "g":
            # set current gains (kp, ki)
            kp = input('Enter current kp [float]: ')
            ser.write((kp+'\n').encode()) # send the kp
            ki = input('Enter current ki [float]: ')
            ser.write((ki+'\n').encode()) # send the ki
            print(f"Selected kp: {kp}, ki: {ki}")
        elif selection == "h":
            # get current gains
            cur_gains_kpki = ser.read_until(b'\n').decode().split(';')
            kp, ki = float(cur_gains_kpki[0]), float(cur_gains_kpki[1])
            print(f"Current gains kp: {kp}, ki: {ki}")
            
        elif selection == "i":
            # set position gains (kp, ki)
            kp = input('Enter position kp [float]: ')
            ser.write((kp+'\n').encode()) # send the kp
            ki = input('Enter position ki [float]: ')
            ser.write((ki+'\n').encode()) # send the ki
            kd = input('Enter position kd [float]: ')
            ser.write((kd+'\n').encode()) # send the kd
            print(f"Selected kp: {kp}, ki: {ki}, kd: {kd}")
        elif selection == "j":
            # get position gains
            cur_gains_kpki = ser.read_until(b'\n').decode().split(';')
            kp, ki, kd = float(cur_gains_kpki[0]), float(cur_gains_kpki[1]), float(cur_gains_kpki[2])
            print(f"Current kp: {kp}, ki: {ki}, kd: {kd}")
            
        elif selection == "k":
            # run ITEST and plot
            # read_plot_matrix()
            # TODO: make reading this a function
            print("Running ITEST!")
            # read the data back as int in reverse order
            read_samples = 100
            measured = []
            ref = []
            while read_samples > 1:
                data_read = ser.read_until(b'\n',50)
                print(data_read)
                data_text = str(data_read,'utf-8')
                data = list(map(int,data_text.split()))

                if(len(data)==3):
                    read_samples = data[0]
                    measured.append(data[1])
                    ref.append(data[2])

            # plot it
            t = range(len(measured)) # time array
            plt.plot(t, measured,'r*-', label="Measured Current [mA]")    # this is in range 1024. Need to map it to PR scale.
            plt.plot(t, ref,'b*-',label="Reference Current [mA]")
            plt.ylabel('value')
            plt.xlabel('sample')
            plt.legend()
            plt.show()
        elif selection == "l":
            # go to angle (deg)
            print("HOLDing!")
            pass
        elif selection == "m":
            # load step trajectory
            ref = genRef('step')
            #print(len(ref))
            t = range(len(ref))
            plt.plot(t,ref,'r*-')
            plt.ylabel('ange in degrees')
            plt.xlabel('index')
            plt.show()
            
            # send: number of data points, each data point
            ser.write((str(len(ref))+'\n').encode())
            for i in ref:
                ser.write((str(i)+'\n').encode())
        elif selection == "n":
            # load cubic trajectory
            ref = genRef('cubic')
            t = range(len(ref))
            plt.plot(t,ref,'r*-')
            plt.ylabel('ange in degrees')
            plt.xlabel('index')
            plt.show()
            
            # send: number of data points, each data point
            ser.write((str(len(ref))+'\n').encode())
            for i in ref:
                ser.write((str(i)+'\n').encode())
        elif selection == "o":
            # execute trajectory
            print("Execute trajectory")
            pass
        elif selection == "p":
            # unpower motor (go to IDLE)
            print("Motor Unpowered to IDLE")
        elif selection == 'q':
            # quit
            print('Exiting client')
            has_quit = True
        elif selection == "r":
            # read what mode
            mode_str = ser.read_until(b'\n')
            mode = int(mode_str)
            print("Mode: ", mode_map[mode])
            pass
        
        else:
            print('Invalid Selection ' + selection_endline)
except Exception as e:
    print(e)
    
finally:
    ser.close()
    print("\nClosing gracefully")