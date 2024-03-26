#!/usr/bin/env python3

import io
import json
import queue
import time
from multiprocessing import Process, Manager, Queue, Value
import threading
from typing import Optional, List

import sensors

import picamera
import requests

from communication.android import AndroidLink, AndroidMessage
from communication.stm32 import STMLink
from consts import SYMBOL_MAP
from logger import prepare_logger
from settings import API_IP, API_PORT, OUTDOOR_BIG_TURN, API_IMAGE_IP, API_IMAGE_PORT

arrow_dir = "none"
first_arrow_dir = "none"
second_arrow_dir = "none"

class PiAction:

    """
    Represents an action that the Pi is responsible for:
    - Changing the robot's mode (manual/path)
    - Requesting a path from the API
    - Snapping an image and requesting the image-rec result from the API
    """

    def __init__(self, cat, value):
        self._cat = cat
        self._value = value

    @property
    def cat(self):
        return self._cat

    @property
    def value(self):
        return self._value


class RaspberryPi:
    def __init__(self):
        # prepare logger
        self.logger = prepare_logger()

        # communication links
        self.android_link = AndroidLink()
        self.stm_link = STMLink()

        # for sharing information between child processes
        manager = Manager()

        # 0: manual, 1: path (default: 1)
        self.robot_mode = manager.Value('i', 0)

        # events
        self.android_dropped = manager.Event()  # set when the android link drops
        self.unpause = manager.Event()  # commands will be retrieved from commands queue when this event is set

        # movement lock, commands will only be sent to STM32 if this is released
        self.movement_lock = manager.Lock()

        # queues
        self.android_queue = manager.Queue()
        self.rpi_action_queue = manager.Queue()
        self.command_queue = manager.Queue()
        self.path_queue = manager.Queue()

        # define processes
        self.proc_recv_android = None
        self.proc_recv_stm32 = None
        self.proc_android_sender = None
        self.proc_command_follower = None
        self.proc_rpi_action = None

        # define threads
        self.thread_set_us_flag = None
        self.thread_set_irl_flag = None
        self.thread_set_irr_flag = None

        # define shared data
        self.ultrasonic_is_clear = Value('i', 1)  # variable to check if obstacle is near using US, shared between processes
        self.ultrasonic_is_clear_samula = Value('i', 1)
        self.ir_left_is_clear = Value('i', 1) # variable to check if obstacle is near using left IR, shared between processes
        self.ir_right_is_clear = Value('i', 1) # variable to check if obstacle is near using right IR, shared between processes
        self.total_xdist = Value('i', 0)
        self.total_ytime = Value('f', 0)

        self.compensate_ydist = Value('f', 0)
        self.between_obstacles_ydist = Value('i', 0)

    def start(self):
        try:
            # establish bluetooth connection with Android
            self.android_link.connect()
            self.android_queue.put(AndroidMessage('info', 'You are connected to the RPi!'))

            # establish connection with STM32
            self.stm_link.connect()

            # check api status
            self.check_api()

            # define processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_recv_stm32 = Process(target=self.recv_stm)
            self.proc_android_sender = Process(target=self.android_sender)
            self.proc_command_follower = Process(target=self.command_follower)
            self.proc_rpi_action = Process(target=self.rpi_action)

            # start processes
            self.proc_recv_android.start()
            self.proc_recv_stm32.start()
            self.proc_android_sender.start()
            self.proc_command_follower.start()
            self.proc_rpi_action.start()

            # define threads
            self.thread_set_us_flag = threading.Thread(target=self.set_us_flag)
            self.thread_set_irl_flag = threading.Thread(target=self.set_irl_flag)
            self.thread_set_irr_flag = threading.Thread(target=self.set_irr_flag)

            # start threads
            self.thread_set_us_flag.start()
            self.thread_set_irl_flag.start()
            self.thread_set_irr_flag.start()

            self.logger.info("Child Processes started")
            self.android_queue.put(AndroidMessage('info', 'Robot is ready!'))
            self.android_queue.put(AndroidMessage('mode', 'path' if self.robot_mode.value == 1 else 'manual'))

            # buzz STM32 (2 times)
            self.stm_link.send("ZZ02")

            # reconnect handler to watch over android connection
            self.reconnect_android()

        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        self.android_link.disconnect()
        self.stm_link.disconnect()
        self.logger.info("Program exited!")

    def reconnect_android(self):
        self.logger.info("Reconnection handler is watching...")

        while True:
            # wait for android connection to drop
            self.android_dropped.wait()

            self.logger.error("Android link is down!")

            # buzz STM32 (3 times)
            self.stm_link.send("ZZ03")

            # kill child processes
            self.logger.debug("Killing android child processes")
            self.proc_android_sender.kill()
            self.proc_recv_android.kill()

            # wait for the child processes to finish
            self.proc_android_sender.join()
            self.proc_recv_android.join()
            assert self.proc_android_sender.is_alive() is False
            assert self.proc_recv_android.is_alive() is False
            self.logger.debug("Android child processes killed")

            # clean up old sockets
            self.android_link.disconnect()

            # reconnect
            self.android_link.connect()

            # recreate android processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_android_sender = Process(target=self.android_sender)

            # start processes
            self.proc_recv_android.start()
            self.proc_android_sender.start()

            self.logger.info("Android child processes restarted")
            self.android_queue.put(AndroidMessage("info", "You are reconnected!"))
            self.android_queue.put(AndroidMessage('mode', 'path' if self.robot_mode.value == 1 else 'manual'))

            # buzz STM32 (2 times)
            self.stm_link.send("ZZ02")

            self.android_dropped.clear()

    def recv_android(self) -> None:
        while True:
            msg_str: Optional[str] = None
            try:
                msg_str = self.android_link.recv()
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Event set: Android connection dropped")

            # if an error occurred in recv()
            if msg_str is None:
                continue

            message: dict = json.loads(msg_str)

            self.logger.debug(f"message: {message}")

            # change mode command
            if message['cat'] == "mode":
                self.rpi_action_queue.put(PiAction(**message))
                self.logger.debug(f"Change mode PiAction added to queue: {message}")

            # manual movement commands
            elif message['cat'] == "manual":
                if self.robot_mode.value == 0:  # robot must be in manual mode
                    if message['value'] == "FC01":
                        print("\nTask2: Starting Fastest Car")
                        self.fastest_car()
                    else:
                        self.command_queue.put(message['value'])
                        self.logger.debug(f"Manual Movement added to command queue: {message['value']}")
                else:
                    self.android_queue.put(AndroidMessage("error", "Manual movement not allowed in Path mode."))
                    self.logger.warning("Manual movement not allowed in Path mode.")

            # set obstacles
            elif message['cat'] == "obstacles":
                if self.robot_mode.value == 1:  # robot must be in path mode
                    self.rpi_action_queue.put(PiAction(**message))
                    self.logger.debug(f"Set obstacles PiAction added to queue: {message}")
                else:
                    self.android_queue.put(AndroidMessage("error", "Robot must be in Path mode to set obstacles."))
                    self.logger.warning("Robot must be in Path mode to set obstacles.")

            # control commands
            elif message['cat'] == "control":
                if message['value'] == "start":
                    # robot must be in path mode
                    if self.robot_mode.value == 1:
                        # check api
                        if not self.check_api():
                            self.logger.error("API is down! Start command aborted.")
                            self.android_queue.put(AndroidMessage('error', "API is down, start command aborted."))

                            # buzz STM32 (4 times)
                            self.stm_link.send("ZZ04")

                        # commencing path following
                        if not self.command_queue.empty():
                            self.unpause.set()
                            self.logger.info("Start command received, starting robot on path!")
                            self.android_queue.put(AndroidMessage('info', 'Starting robot on path!'))
                            self.android_queue.put(AndroidMessage('status', 'running'))
                        else:
                            self.logger.warning("The command queue is empty, please set obstacles.")
                            self.android_queue.put(
                                AndroidMessage("error", "Command queue is empty, did you set obstacles?"))
                    else:
                        self.android_queue.put(
                            AndroidMessage("error", "Robot must be in Path mode to start robot on path."))
                        self.logger.warning("Robot must be in Path mode to start robot on path.")

            # navigate around obstacle
            elif message['cat'] == "single-obstacle":
                if self.robot_mode.value == 1:  # robot must be in path mode
                    self.rpi_action_queue.put(PiAction(**message))
                    self.logger.debug(f"Single-obstacle PiAction added to queue: {message}")
                else:
                    self.android_queue.put(
                        AndroidMessage("error", "Robot must be in Path mode to set single obstacle."))
                    self.logger.warning("Robot must be in Path mode to set single obstacle.")

    def recv_stm(self) -> None:
        """
        Receive acknowledgement messages from STM32, and release the movement lock
        """
        while True:
            message: str = self.stm_link.recv()

            # acknowledgement from STM32
            if message.startswith("ACK"):
                # release movement lock
                try:
                    self.movement_lock.release()
                    self.logger.debug("ACK from STM32 received, movement lock released.")

                    # if in path mode, get new location and notify android
                    if self.robot_mode.value == 1:
                        temp = self.path_queue.get_nowait()
                        location = {
                            "x": temp['x'],
                            "y": temp['y'],
                            "d": temp['d'],
                        }
                        self.android_queue.put(AndroidMessage('location', location))
                    else:
                        if message == "ACK|X":
                            self.logger.debug("Fastest car ACK received from STM32!")
                            self.android_queue.put(AndroidMessage("info", "Robot has completed fastest car!"))
                            self.android_queue.put(AndroidMessage("status", "finished"))
                except Exception:
                    self.logger.warning("Tried to release a released lock!")
            else:
                self.logger.warning(f"Ignored unknown message from STM: {message}")

    def android_sender(self) -> None:
        """
        Responsible for retrieving messages from the outgoing message queue and sending them over the Android Link
        """
        while True:
            # retrieve from queue
            try:
                message: AndroidMessage = self.android_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            # send it over the android link
            try:
                self.android_link.send(message)
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Event set: Android dropped")

    def command_follower(self) -> None:
        while True:
            # retrieve next movement command
            command: str = self.command_queue.get()

            # wait for unpause event to be true
            # self.unpause.wait()

            # acquire lock first (needed for both moving, and snapping pictures)
            self.movement_lock.acquire()

            # STM32 commands
            stm32_prefixes = ("FS", "BS", "FW", "BW", "FL", "FR", "BL", "BR", "TL", "TR","IR", "A", "C", "DT", "STOP", "ZZ")
            if command.startswith(stm32_prefixes):
                #number =int(command[2:])
                self.stm_link.send(command)
                # print("Command sent to STM: ", command)

            elif command.startswith("WN"):
                # self.stm_link.send(command)
                self.android_queue.put(AndroidMessage('status', 'running'))
                self.android_queue.put(AndroidMessage("info", "Starting robot on fastest car!"))
                
            # snap command (path mode)
            elif command.startswith("SNAP"):
                obstacle_id = command.replace("SNAP", "")
                self.rpi_action_queue.put(PiAction(cat="snap", value=obstacle_id))

            # snap command (manual mode)
            elif command.startswith("MANSNAP"):
                obstacle_id = "99"
                self.rpi_action_queue.put(PiAction(cat="snap", value=obstacle_id))

            # no-op (a workaround to let the robot stop after a non-bullseye face has been found)
            elif command.startswith("NOOP"):
                # self.stm_link.send("FW00")
                self.movement_lock.release()

            # end of path
            elif command == "FIN":
                self.stm_link.send("ZZ01")
                # clear the unpause event (no new command will be retrieved from queue)
                self.unpause.clear()
                self.movement_lock.release()
                self.logger.info("Commands queue finished.")
                self.android_queue.put(AndroidMessage("info", "Commands queue finished."))
                self.android_queue.put(AndroidMessage("status", "finished"))
                self.rpi_action_queue.put(PiAction(cat="stitch", value=""))
            else:
                raise Exception(f"Unknown command: {command}")
    
    def set_us_flag(self) -> None:
        obstacle_count_short = 0
        obstacle_count_long = 0

        while True:
            dist = sensors.distance()
            self.compensate_ydist.value = dist # MAY HAVE TO DELETE THIS!
            # print("\nTask2: Ultrasonic Distance = ", dist)

            # if (dist <= 50.0):
            #     obstacle_count_long += 1

            #     if (dist <= 28.0):
            #         obstacle_count_short += 1
            #     else:
            #         obstacle_count_short = 0
            #         self.ultrasonic_is_clear.value = 1
            #     if (obstacle_count_short == 2):
            #         self.ultrasonic_is_clear.value = 0
            #     if (obstacle_count_long == 2):
            #         self.ultrasonic_is_clear_samula.value = 0
            # else:
            #     obstacle_count_long = 0
            #     self.ultrasonic_is_clear_samula.value = 1

            # time.sleep(0.1)


            if (dist <= 28.0): # Change threshold accordingly
                obstacle_count_short += 1
            else:
                obstacle_count_short = 0
                self.ultrasonic_is_clear.value = 1

            if obstacle_count_short == 2:
                self.ultrasonic_is_clear.value = 0

            time.sleep(0.1)

    def move_until_obstacle_us(self): # Not Threaded
        # Testing Code
        # while True:
        #     print("\nself.ultrasonic_is_clear.value: ", self.ultrasonic_is_clear.value)
        #     time.sleep(0.5)

        if (self.ultrasonic_is_clear.value == 1):
            print("\nTask2: Moving forward from US, putting FW-- in command queue")
            self.clear_queues()
            self.command_queue.put("FW--")
            start_time = time.time()
        else:
            start_time = time.time()

        while True:
            if (self.ultrasonic_is_clear.value == 0):
                end_time = time.time()
                print("\nTask2: Stop from US, putting STOP in command queue")
                self.clear_queues()
                self.command_queue.put("STOP")
                # self.command_queue.put("BW20")
                # should save distance travelled to total yDist
                break

            # time.sleep(0.5) # May need to change this value for accuracy

        elapsed_time = end_time - start_time
        self.total_ytime.value += elapsed_time
        print("!IMPORTANT: self.total_ytime.value = ", self.total_ytime.value)

    def set_irl_flag(self) -> None:

       while True:
            # wall = IRtest.readL()
            wall = sensors.readL()
            #print("\nTask2: IR Left  = ", wall)

            if (wall): # Change threshold accordingly
                # print("\nTask2: if (wall) == 1, ir left is clear")
                # obstacle_count += 1
                self.ir_left_is_clear.value = 1
            else:
                # print("\nTask2: else, ir left wall detected")
                # obstacle_count = 0
                self.ir_left_is_clear.value = 0

            # if obstacle_count == 3:
            #     self.ultrasonic_is_clear.value = 0

            time.sleep(0.1)

    def old_move_past_obstacle_irl(self):
        print("\nmove_past_obstacle_irl start")
        if (self.ir_left_is_clear.value == 0): # If not clear, move indefinitely
            print("\nTask2: Moving forward from IR, putting FW-- in command queue")
            self.clear_queues()
            self.command_queue.put("FW--")

        while True:
            if (self.ir_left_is_clear.value == 1):
                print("\nTask2: Stop from IR Left, putting STOP in command queue")
                self.clear_queues()
                self.command_queue.put("STOP")
                # self.command_queue.put("FW10")
                # self.command_queue.put("BW20")
                # should save distance travelled to total yDist
                break

            time.sleep(0.5) # May need to change this value for accuracy

    def move_past_obstacle_irl(self):
        if (self.ir_left_is_clear.value == 0): # If not clear, move FW10
            print("\nTask2: Moving forward from IR, putting FW10 in command queue")
            self.clear_queues() # CHECK IF NEED DELETE THIS!!
            self.command_queue.put("FW10")
            time.sleep(2) # Just Added
            self.total_xdist.value += 10

        while True:
            if (self.ir_left_is_clear.value == 0): # If not clear, move FW10
                print("\nTask2: Moving forward from IR FROM WHILE TRUE, putting FW10 in command queue")
                
                # self.clear_queues() # Just commented this 9:45pm
                time.sleep(2) # Just added this
                self.command_queue.put("FW10")
                self.total_xdist.value += 10
            else: # If clear obstacle, stop moving and break
                break
            
            print("move_past_obstacle_irl(): Sleeping for 3 seconds")
            time.sleep(3) # should correspond to how long each fw10 takes, ADJUST!!

        self.command_queue.put("FW10") # CHECK THIS! ADDED TO MAKE SURE CLEAR OBSTACLE 2
        self.total_xdist.value += 10 # CHECK THIS! ADDED TO MAKE SURE CLEAR OBSTACLE 2
        
    def move_past_obstacle_irl_long(self):
        if (self.ir_left_is_clear.value == 0): # If not clear, move FW10
            print("\nTask2: Moving forward from IR, putting FW10 in command queue")
            self.clear_queues() # CHECK IF NEED DELETE THIS!!
            self.command_queue.put("FW10")
            self.total_xdist.value += 10
            time.sleep(2) # Just Added

        while True:
            if (self.ir_left_is_clear.value == 0): # If not clear, move FW10
                print("\nTask2: Moving forward from IR FROM WHILE TRUE, putting FW10 in command queue")
                
                # self.clear_queues() # Just commented this 9:45pm
                time.sleep(2) # Just added this
                self.command_queue.put("FW10")
                self.total_xdist.value += 10
            else: # If clear obstacle, stop moving and break
                break
            
            print("move_past_obstacle_irl(): Sleeping for 3 seconds")
            time.sleep(3) # should correspond to how long each fw10 takes, ADJUST!!

        # self.command_queue.put("FW10") # CHECK THIS! ADDED TO MAKE SURE CLEAR OBSTACLE 2
        # self.total_xdist.value += 10 # CHECK THIS! ADDED TO MAKE SURE CLEAR OBSTACLE 2

    def set_irr_flag(self) -> None:
        
        while True:
        # wall = IRtest.readL()
            wall = sensors.readR()
            #print("\nTask2: IR Left  = ", wall)

            if (wall): # Change threshold accordingly
                # print("\nTask2: if (wall) == 1, ir right is clear")
                # obstacle_count += 1
                self.ir_right_is_clear.value = 1
            else:
                # print("\nTask2: else, ir right wall detected")
                # obstacle_count = 0
                self.ir_right_is_clear.value = 0

            # if obstacle_count == 3:
            #     self.ultrasonic_is_clear.value = 0

            time.sleep(0.1)
    
    def move_past_obstacle_irr(self):
        if (self.ir_right_is_clear.value == 0): # If not clear, move FW10
            print("\nTask2: Moving forward from IR, putting FW10 in command queue")
            self.clear_queues() # CHECK IF NEED DELETE THIS!!
            self.command_queue.put("FW10")
            time.sleep(2) # Just Added
            self.total_xdist.value += 10

        while True:
            if (self.ir_right_is_clear.value == 0): # If not clear, move FW10
                print("\nTask2: Moving forward from IR, putting FW10 in command queue")
                
                # self.clear_queues() # Just commented this 9:45pm
                time.sleep(2) # Just added this
                self.command_queue.put("FW10")
                self.total_xdist.value += 10
            else: # If clear obstacle, stop moving and break
                break
            
            print("move_past_obstacle_irr(): sleeping for 3 seconds")
            time.sleep(3) # should correspond to how long each fw10 takes, ADJUST!!

        self.command_queue.put("FW10") # CHECK THIS! ADDED TO MAKE SURE CLEAR OBSTACLE 2
        self.total_xdist.value += 10 # CHECK THIS! ADDED TO MAKE SURE CLEAR OBSTACLE 2

    def move_past_obstacle_irr_long(self):
        if (self.ir_right_is_clear.value == 0): # If not clear, move FW10
            print("\nTask2: Moving forward from IR, putting FW10 in command queue")
            self.clear_queues() # CHECK IF NEED DELETE THIS!!
            self.command_queue.put("FW10")
            self.total_xdist.value += 10
            time.sleep(2) # Just Added

        while True:
            if (self.ir_right_is_clear.value == 0): # If not clear, move FW10
                print("\nTask2: Moving forward from IR, putting FW10 in command queue")
                
                # self.clear_queues() # Just commented this 9:45pm
                time.sleep(2) # Just added this
                self.command_queue.put("FW10")
                self.total_xdist.value += 10
            else: # If clear obstacle, stop moving and break
                break
            
            print("move_past_obstacle_irr(): sleeping for 3 seconds")
            time.sleep(3) # should correspond to how long each fw10 takes, ADJUST!!

        # self.command_queue.put("FW10") # CHECK THIS! ADDED TO MAKE SURE CLEAR OBSTACLE 2
        # self.total_xdist.value += 10 # CHECK THIS! ADDED TO MAKE SURE CLEAR OBSTACLE 2   

    def clear_first_obstacle(self, first_arrow_dir: str):
        if first_arrow_dir == "38": # Right Arrow
            hardcoded_path: List[str] = [
                "FR30", "FL30", "FL30", "FR30"
            ]
            self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

        elif first_arrow_dir == "39": # Left Arrow
            hardcoded_path: List[str] = [
                "FL30", "FR30", "FR30", "FL30"
            ]
            self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

        else:
            hardcoded_path: List[str] = [
                "FR30", "FL30", "FL30", "FR30"
            ]
            self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

    def clear_second_obstacle_p1(self, second_arrow_dir: str):
        print("clear_first_obstacle(): sleeping for 5 seconds")
        time.sleep(5)
        print("\nTask2: clear_second_obstacle_p1")

        if second_arrow_dir == "38": # Right Arrow
            print("\n\n\ncompensate_ydist:", self.compensate_ydist.value)
            if (self.compensate_ydist.value < 15.0):
                dist_to_move = int(27 - int(self.compensate_ydist.value))
                if dist_to_move >= 0:
                    # self.command_queue.put(f"BW{dist_to_move:02d}")
                    self.command_queue.put("BW05")
                    time.sleep(2)
            
            print("\nTask2: clear_second_obstacle_p1 -> Hardcoded FR30 to command_queue")
            hardcoded_path: List[str] = ["FR30"]
            self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)
            print("clear_second_obstacle_p1() -> first right turn: sleeping for 10 seconds")
            time.sleep(10)

            print("\nTask2: clear_second_obstacle_p1 -> move_past_obstacle_irl")
            # self.old_move_past_obstacle_irl()
            self.move_past_obstacle_irl()
            print("\nIMPORTANT! total xdist = ", self.total_xdist.value)

            print("clear_second_obstacle_p1() -> after move_past_obstacle_irl : sleeping for 5 seconds")
            time.sleep(5)

            # Along width of Obstacle 2
            print("\nTask2: clear_second_obstacle_p1 -> Hardcoded FL30 to command_queue")
            hardcoded_path: List[str] = ["FL30", "FW10"]
            # self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

            # print("\nSee if this moves forward before turning left to xaxis travel")
            # hardcoded_path: List[str] = ["FW20"]
            # # self.clear_queues()
            # for c in hardcoded_path:
            #     self.command_queue.put(c)

        elif second_arrow_dir == "39": # Left Arrow
            print("\n\n\ncompensate_ydist:", self.compensate_ydist.value)
            if (self.compensate_ydist.value < 15.0):
                dist_to_move = int(27 - int(self.compensate_ydist.value))
                if dist_to_move >= 0:
                    # self.command_queue.put(f"BW{dist_to_move:02d}")
                    self.command_queue.put("BW05")
                    time.sleep(2)
            
            print("\nTask2: clear_second_obstacle_p1 -> Hardcoded FL30 to command_queue")
            hardcoded_path: List[str] = ["FL30"]
            self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

            print("clear_second_obstacle_p1() -> first left turn: sleeping for 10 seconds")
            time.sleep(10)

            print("\nTask2: clear_second_obstacle_p1 -> move_past_obstacle_irr")
            # self.old_move_past_obstacle_irr()
            self.move_past_obstacle_irr()
            print("\nIMPORTANT! total xdist = ", self.total_xdist.value)
            print("clear_second_obstacle_p1() -> after move_past_obstacle_irr : sleeping for 5 seconds")
            time.sleep(5)

            # Along width of Obstacle 2
            print("\nTask2: clear_second_obstacle_p1 -> Hardcoded FR30 to command_queue")
            hardcoded_path: List[str] = ["FR30", "FW10"]
            # self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

            # print("\nSee if this moves forward before turning left to xaxis travel")
            # hardcoded_path: List[str] = ["FW20"]
            # # self.clear_queues()
            # for c in hardcoded_path:
            #     self.command_queue.put(c)

        else:
            print("\n\n\ncompensate_ydist:", self.compensate_ydist.value)
            if (self.compensate_ydist.value < 15.0):
                dist_to_move = int(27 - int(self.compensate_ydist.value))
                if dist_to_move >= 0:
                    # self.command_queue.put(f"BW{dist_to_move:02d}")
                    self.command_queue.put("BW05")
                    time.sleep(2)
            
            print("\nTask2: clear_second_obstacle_p1 -> Hardcoded FR30 to command_queue")
            hardcoded_path: List[str] = ["FR30"]
            self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

            print("clear_second_obstacle_p1() -> first right turn: sleeping for 10 seconds")
            time.sleep(10)

            print("\nTask2: clear_second_obstacle_p1 -> move_past_obstacle_irl")
            # self.old_move_past_obstacle_irl()
            self.move_past_obstacle_irl()
            print("clear_second_obstacle_p1() -> after move_past_obstacle_irl : sleeping for 5 seconds")
            time.sleep(5)

            # Along width of Obstacle 2
            print("\nTask2: clear_second_obstacle_p1 -> Hardcoded FL30 to command_queue")
            hardcoded_path: List[str] = ["FL30", "FW10"]
            # self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

            # print("\nSee if this moves forward before turning left to xaxis travel")
            # hardcoded_path: List[str] = ["FW20"]
            # # self.clear_queues()
            # for c in hardcoded_path:
            #     self.command_queue.put(c)

    def clear_second_obstacle_p2(self, second_arrow_dir: str):
        print("clear_second_obstacle_p2(): sleeping for 5 seconds")
        time.sleep(5)
        print("\nTask2: clear_second_obstacle_p2")

        if second_arrow_dir == "38": # Right Arrow
            # Along X Axis
            print("\nTask2: clear_second_obstacle_p2 -> Hardcoded FL30 to command_queue, turn to xaxis ")
            hardcoded_path: List[str] = ["FL30"]
            # self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

            doubled_xdist = ((self.total_xdist.value) * 2) + 30
            if (self.total_xdist.value <= 20):
                doubled_xdist += 20
            print("\n!IMPORTANT: MOVING FORWARD BY ", doubled_xdist)

            if (doubled_xdist <= 99):
                print("\nFW", doubled_xdist)
                self.command_queue.put(f"FW{doubled_xdist:02d}")
            else: # If exceeds 99
                print("\nFW99")
                self.command_queue.put("FW99")
                remaining_xdist = doubled_xdist - 99

                if (remaining_xdist < 99):
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")
                else: # Stil longer than 99
                    print("\nFW99")
                    self.command_queue.put("FW99")
                    remaining_xdist = remaining_xdist - 99
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")
            print("clear_second_obstacle_p2() -> after clearing x-axis: sleeping for 5 seconds")
            time.sleep(5)
                
            # print("\n!IMPORTANT: MOVING FORWARD BY ", self.total_xdist.value)

            # if (self.total_xdist.value <= 99):
            #     print("\nFW", self.total_xdist.value)
            #     self.command_queue.put(f"FW{self.total_xdist.value:02d}")
            # else: # If exceeds 99
            #     print("\nFW99")
            #     self.command_queue.put("FW99")
            #     remaining_xdist = self.total_xdist.value - 99

            #     if (remaining_xdist < 99):
            #         print("\nFW", remaining_xdist)
            #         self.command_queue.put(f"FW{remaining_xdist:02d}")
            #     else: # Stil longer than 99
            #         print("\nFW99")
            #         self.command_queue.put("FW99")
            #         remaining_xdist = remaining_xdist - 99
            #         print("\nFW", remaining_xdist)
            #         self.command_queue.put(f"FW{remaining_xdist:02d}")
            # print("clear_second_obstacle_p2() -> after clearing x-axis: sleeping for 5 seconds")
            # time.sleep(5)
            
            self.move_past_obstacle_irl_long()

        elif second_arrow_dir == "39": # Left Arrow
            # Along X Axis
            print("\nTask2: clear_second_obstacle_p2 -> Hardcoded FR30 to command_queue, turn to xaxis ")
            hardcoded_path: List[str] = ["FR30"]
            # self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

            doubled_xdist = ((self.total_xdist.value) * 2) + 30
            if (self.total_xdist.value <= 20):
                doubled_xdist += 20
            print("\n!IMPORTANT: MOVING FORWARD BY ", doubled_xdist)

            if (doubled_xdist <= 99):
                print("\nFW", doubled_xdist)
                self.command_queue.put(f"FW{doubled_xdist:02d}")
            else: # If exceeds 99
                print("\nFW99")
                self.command_queue.put("FW99")
                remaining_xdist = doubled_xdist - 99

                if (remaining_xdist < 99):
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")
                else: # Stil longer than 99
                    print("\nFW99")
                    self.command_queue.put("FW99")
                    remaining_xdist = remaining_xdist - 99
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")
            print("clear_second_obstacle_p2() -> after clearing x-axis: sleeping for 5 seconds")
            time.sleep(5)
            
            # print("\n!IMPORTANT: MOVING FORWARD BY ", self.total_xdist.value)

            # if (self.total_xdist.value <= 99):
            #     print("\nFW", self.total_xdist.value)
            #     self.command_queue.put(f"FW{self.total_xdist.value:02d}")
            # else: # If exceeds 99
            #     print("\nFW99")
            #     self.command_queue.put("FW99")
            #     remaining_xdist = self.total_xdist.value - 99

            #     if (remaining_xdist < 99):
            #         print("\nFW", remaining_xdist)
            #         self.command_queue.put(f"FW{remaining_xdist:02d}")
            #     else: # Stil longer than 99
            #         print("\nFW99")
            #         self.command_queue.put("FW99")
            #         remaining_xdist = remaining_xdist - 99
            #         print("\nFW", remaining_xdist)
            #         self.command_queue.put(f"FW{remaining_xdist:02d}")
            # print("clear_second_obstacle_p2() -> after clearing x-axis: sleeping for 5 seconds")
            # time.sleep(5)

            self.move_past_obstacle_irr_long()

        else:
            # Along X Axis
            print("\nTask2: clear_second_obstacle_p2 -> Hardcoded FL30 to command_queue, turn to xaxis ")
            hardcoded_path: List[str] = ["FL30"]
            # self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

            doubled_xdist = ((self.total_xdist.value) * 2) + 30
            if (self.total_xdist.value <= 20):
                doubled_xdist += 20
            print("\n!IMPORTANT: MOVING FORWARD BY ", doubled_xdist)

            if (doubled_xdist <= 99):
                print("\nFW", doubled_xdist)
                self.command_queue.put(f"FW{doubled_xdist:02d}")
            else: # If exceeds 99
                print("\nFW99")
                self.command_queue.put("FW99")
                remaining_xdist = doubled_xdist - 99

                if (remaining_xdist < 99):
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")
                else: # Stil longer than 99
                    print("\nFW99")
                    self.command_queue.put("FW99")
                    remaining_xdist = remaining_xdist - 99
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")
            print("clear_second_obstacle_p2() -> after clearing x-axis: sleeping for 5 seconds")
            time.sleep(5)
                
            # print("\n!IMPORTANT: MOVING FORWARD BY ", self.total_xdist.value)

            # if (self.total_xdist.value <= 99):
            #     print("\nFW", self.total_xdist.value)
            #     self.command_queue.put(f"FW{self.total_xdist.value:02d}")
            # else: # If exceeds 99
            #     print("\nFW99")
            #     self.command_queue.put("FW99")
            #     remaining_xdist = self.total_xdist.value - 99

            #     if (remaining_xdist < 99):
            #         print("\nFW", remaining_xdist)
            #         self.command_queue.put(f"FW{remaining_xdist:02d}")
            #     else: # Stil longer than 99
            #         print("\nFW99")
            #         self.command_queue.put("FW99")
            #         remaining_xdist = remaining_xdist - 99
            #         print("\nFW", remaining_xdist)
            #         self.command_queue.put(f"FW{remaining_xdist:02d}")
            # print("clear_second_obstacle_p2() -> after clearing x-axis: sleeping for 5 seconds")
            # time.sleep(5)
            
            self.move_past_obstacle_irl_long()
    
    def clear_second_obstacle_p3(self, second_arrow_dir: str):
        print("clear_second_obstacle_p3() -> before start: sleeping for 5 seconds")
        time.sleep(5)
        print("\nTask2: clear_second_obstacle_p3")

        if second_arrow_dir == "38": # Right Arrow
            # To Face Back Home
            print("\nTask2: clear_second_obstacle_p3 -> Hardcoded FL30 to command_queue")
            hardcoded_path: List[str] = ["FL30"]
            # self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

        elif second_arrow_dir == "39": # Left Arrow
            # To Face Back Home
            print("\nTask2: clear_second_obstacle_p3 -> Hardcoded FR30 to command_queue")
            hardcoded_path: List[str] = ["FR30"]
            # self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

        else:
            # To Face Back Home
            print("\nTask2: clear_second_obstacle_p3 -> Hardcoded FL30 to command_queue")
            hardcoded_path: List[str] = ["FL30"]
            # self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

    def return_home(self):
        print("\nTask2: return_home() pang kang lo")

        # self.command_queue.put("FW10")
        # time.sleep(1)
        # self.total_ytime.value = self.total_ytime.value - 0.3
        # print("!IMPORTANT: total_ytime.value MOVING FORWARD FOR ", self.total_ytime.value)

        # if (self.total_xdist.value <= 20):
        #     self.total_xdist.value = self.total_xdist.value + 10

        # self.command_queue.put("FW--")
        # print("return home() -> going straight: sleeping for ", self.total_ytime.value," seconds")
        # time.sleep(self.total_ytime.value)
        # self.command_queue.put("STOP")

        # Final Turns to Go Home
        time.sleep(3)
        local_ydist = 0
        local_ydist = self.between_obstacles_ydist.value
        print("!YOYO: PUTTING FW", local_ydist)
        # self.command_queue.put(f"FW{local_ydist:02d}")
        
        if (local_ydist <= 99):
            print("\nFW", local_ydist)
            self.command_queue.put(f"FW{local_ydist:02d}")
        else: # If exceeds 99
            print("\nFW99")
            self.command_queue.put("FW99")
            remaining_xdist = local_ydist - 99

            if (remaining_xdist < 99):
                print("\nFW", remaining_xdist)
                self.command_queue.put(f"FW{remaining_xdist:02d}")
            else: # Stil longer than 99
                print("\nFW99")
                self.command_queue.put("FW99")
                remaining_xdist = remaining_xdist - 99
                print("\nFW", remaining_xdist)
                self.command_queue.put(f"FW{remaining_xdist:02d}")

        time.sleep(5)

        if second_arrow_dir == "38": # Right Arrow:
            hardcoded_path: List[str] = ["FL30"]
            self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

            time.sleep(5)
            print("ultrasonic_is_clear_samula.value is ", self.ultrasonic_is_clear_samula.value)
            # check for obstacle using US
            if (self.ultrasonic_is_clear_samula.value == 0):
                print("\nreturn home() -> samula code: sleeping for ", 3," seconds")
                time.sleep(3)

                # if there is an obstacle, BL30
                hardcoded_path: List[str] = ["BL30", "FW30", "FL30"]
                for c in hardcoded_path:
                    self.command_queue.put(c)
                print("return home() -> samula code: sleeping for ", 3," seconds")
                time.sleep(10)
            else:
                print("\n#CHECK! SAMULA DID NOT HAPPEN")

            single_xdist = int((self.total_xdist.value * 2) / 4)
            single_xdist += 20
            print("\n!IMPORTANT: MOVING FORWARD BY ", single_xdist)
            if (single_xdist <= 99):
                print("\nFW", single_xdist)
                self.command_queue.put(f"FW{single_xdist:02d}")
            else: # If exceeds 99
                print("\nFW99")
                self.command_queue.put("FW99")
                remaining_xdist = single_xdist - 99

                if (remaining_xdist < 99):
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")
                else: # Stil longer than 99
                    print("\nFW99")
                    self.command_queue.put("FW99")
                    remaining_xdist = remaining_xdist - 99
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")


            time.sleep(3)
            self.command_queue.put("FR30")
            print("return home() -> after facing carpark : sleeping for 10 seconds")
            time.sleep(10)

            # print("\nTask2: calling self.move_until_obstacle_us()")
            # self.move_until_obstacle_us()
            
        elif second_arrow_dir == "39": # Left Arrow
            hardcoded_path: List[str] = ["FR30"]
            self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)
            
            time.sleep(5)
            print("ultrasonic_is_clear_samula.value is ", self.ultrasonic_is_clear_samula.value)
            # check for obstacle using US
            if (self.ultrasonic_is_clear_samula.value == 0):
                print("\nreturn home() -> samula code: sleeping for ", 3," seconds")
                time.sleep(3)

                # if there is an obstacle, BL30
                hardcoded_path: List[str] = ["BR30", "FW30", "FR30"]
                for c in hardcoded_path:
                    self.command_queue.put(c)
                print("return home() -> samula code: sleeping for ", 3," seconds")
                time.sleep(10)
            else:
                print("\n#CHECK! SAMULA DID NOT HAPPEN")

            single_xdist = int((self.total_xdist.value * 2) / 4)
            single_xdist += 20
            print("\n!IMPORTANT: MOVING FORWARD BY ", single_xdist)
            if (single_xdist <= 99):
                print("\nFW", single_xdist)
                self.command_queue.put(f"FW{single_xdist:02d}")
            else: # If exceeds 99
                print("\nFW99")
                self.command_queue.put("FW99")
                remaining_xdist = single_xdist - 99

                if (remaining_xdist < 99):
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")
                else: # Stil longer than 99
                    print("\nFW99")
                    self.command_queue.put("FW99")
                    remaining_xdist = remaining_xdist - 99
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")

            time.sleep(3)
            self.command_queue.put("FL30")
            print("return home() -> after facing carpark : sleeping for 10 seconds")
            time.sleep(10)

        else:
            hardcoded_path: List[str] = ["FL30"]
            self.clear_queues()
            for c in hardcoded_path:
                self.command_queue.put(c)

            time.sleep(5)
            print("ultrasonic_is_clear_samula.value is ", self.ultrasonic_is_clear_samula.value)
            # check for obstacle using US
            if (self.ultrasonic_is_clear_samula.value == 0):
                print("\nreturn home() -> samula code: sleeping for ", 3," seconds")
                time.sleep(3)

                # if there is an obstacle, BL30
                hardcoded_path: List[str] = ["BL30", "FW30", "FL30"]
                for c in hardcoded_path:
                    self.command_queue.put(c)
                print("return home() -> samula code: sleeping for ", 3," seconds")
                time.sleep(10)
            else:
                print("\n#CHECK! SAMULA DID NOT HAPPEN")

            single_xdist = int((self.total_xdist.value * 2) / 4)
            single_xdist += 20
            print("\n!IMPORTANT: MOVING FORWARD BY ", single_xdist)
            if (single_xdist <= 99):
                print("\nFW", single_xdist)
                self.command_queue.put(f"FW{single_xdist:02d}")
            else: # If exceeds 99
                print("\nFW99")
                self.command_queue.put("FW99")
                remaining_xdist = single_xdist - 99

                if (remaining_xdist < 99):
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")
                else: # Stil longer than 99
                    print("\nFW99")
                    self.command_queue.put("FW99")
                    remaining_xdist = remaining_xdist - 99
                    print("\nFW", remaining_xdist)
                    self.command_queue.put(f"FW{remaining_xdist:02d}")


            time.sleep(3)
            self.command_queue.put("FR30")
            print("return home() -> after facing carpark : sleeping for 10 seconds")
            time.sleep(10)

            # print("\nTask2: calling self.move_until_obstacle_us()")
            # self.move_until_obstacle_us()

    def return_home_2(self):
        print("\nTask2: return_home_2() pang kang lo")

        if (self.ultrasonic_is_clear.value == 1):
            print("\nTask2: FINAL Moving forward from US, putting FW-- in command queue")
            self.command_queue.put("FW--")

        while True:
            if (self.ultrasonic_is_clear.value == 0):
                print("\nTask2: FINAL Stop from US, putting STOP in command queue")
                self.clear_queues()
                self.command_queue.put("STOP")
                break


    def detect_arrow_image(self):
        obstacle_id = "_1_obstacle"
        arrow_direction = self.snap_and_rec_2(obstacle_id)
        return arrow_direction

    def snap_and_rec_2(self, obstacle_id: str) -> None:
        # capture an image
        stream = io.BytesIO()
        with picamera.PiCamera() as camera:
            camera.start_preview()
            time.sleep(1)
            camera.capture(stream, format='jpeg')

        self.logger.info("Image captured. Calling image-rec api...")
        # release lock so that bot can continue moving
        # self.movement_lock.release()

        # call image-rec API endpoint
        self.logger.debug("Requesting from image API")
        url = f"http://{API_IMAGE_IP}:{API_IMAGE_PORT}/image"
        filename = f"{int(time.time())}_{obstacle_id}.jpg"
        image_data = stream.getvalue()
        response = requests.post(url, files={"file": (filename, image_data)})

        if response.status_code != 200:
            self.logger.error("Something went wrong when requesting path from image-rec API. Please try again.")
            self.android_queue.put(AndroidMessage(
                "error", "Something went wrong when requesting path from image-rec API. Please try again."))
            return

        results = json.loads(response.content)
        # self.android_queue.put(AndroidMessage("image-rec", results))
        # return results

        # response_data = response.json()
        image_id = results['image_id']
        global arrow_dir
        arrow_dir = image_id
        return arrow_dir

    def fastest_car(self):
        global first_arrow_dir
        global second_arrow_dir

        xdist = 0 # left/right
        ydist = 0 # forward/backward

        # Obstacle 1 code
        print("\nTask2: calling self.move_until_obstacle_us()")
        self.move_until_obstacle_us()

        print("fastest_car() : sleeping for 1 seconds")
        time.sleep(1)

        print("\nTask2: calling self.detect_arrow_image()")
        first_arrow_dir = self.detect_arrow_image()
        print("\nTask2: first_arrow_dir = ", first_arrow_dir)

        print("\nTask2: calling clear_first_obstacle()")
        self.clear_first_obstacle(first_arrow_dir)

        print("fastest_car() -> after clear_first_obstacle() : sleeping for 10 seconds")
        time.sleep(10) # CHECK THIS!!!

        # OBSTACLE 2 CODE ONWARDS
        # self.between_obstacles_ydist.value = self.compensate_ydist.value

        time.sleep(3)
        # special_ydist = 0
        max_list = []
        for i in range(5):
            if int(self.compensate_ydist.value) < 150:
                max_list.append(int(self.compensate_ydist.value))
            time.sleep(0.5)
        if max(max_list) is not None:
            self.between_obstacles_ydist.value = max(max_list)
        else:
            self.between_obstacles_ydist.value = 100
        self.between_obstacles_ydist.value += 70

        print("\nTask2: calling self.move_until_obstacle_us()")
        self.move_until_obstacle_us()
        
        print("\nTask2: calling self.detect_arrow_image()")
        second_arrow_dir = self.detect_arrow_image()
        print("\nTask2: second_arrow_dir = ", second_arrow_dir)

        print("\nTask2: calling clear_second_obstacle_p1()")
        self.clear_second_obstacle_p1(second_arrow_dir)

        print("\nTask2: calling clear_second_obstacle_p2()")
        self.clear_second_obstacle_p2(second_arrow_dir)

        print("\nTask2: calling clear_second_obstacle_p3()")
        self.clear_second_obstacle_p3(second_arrow_dir)

        print("fastest_car() -> after clear_second_obstacle_p3() : sleeping for 5 seconds")
        time.sleep(5)

        print("\nTask2: calling return_home()")
        self.return_home()

        time.sleep(4)
        self.return_home_2()

        time.sleep(1)
        self.stm_link.send("ZZ02")
        

    
    def wait_for_acknowledgment(self) -> bool:
        """
        Waits for acknowledgment (ACK) from STM32 after sending a command.
        Returns True if acknowledgment received within a timeout, otherwise False.
        """
        timeout = 10  # Adjust the timeout value as needed
        start_time = time.time()
        
        while time.time() - start_time < timeout:
                return True
        
        return False

    def rpi_action(self):
        while True:
            action: PiAction = self.rpi_action_queue.get()
            self.logger.debug(f"PiAction retrieved from queue: {action.cat} {action.value}")

            if action.cat == "mode":
                self.change_mode(action.value)
            elif action.cat == "single-obstacle":
                self.add_navigate_path()
            elif action.cat == "stitch":
                self.request_stitch()

    def change_mode(self, new_mode):
        # if robot already in correct mode
        if new_mode == "manual" and self.robot_mode.value == 0:
            self.android_queue.put(AndroidMessage('error', 'Robot already in Manual mode.'))
            self.logger.warning("Robot already in Manual mode.")
        elif new_mode == "path" and self.robot_mode.value == 1:
            self.android_queue.put(AndroidMessage('error', 'Robot already in Path mode.'))
            self.logger.warning("Robot already in Path mode.")
        else:
            # change robot mode
            self.robot_mode.value = 0 if new_mode == 'manual' else 1

            # clear command, path queues
            self.clear_queues()

            # set unpause event, so that robot can freely move
            if new_mode == "manual":
                self.unpause.set()
            else:
                self.unpause.clear()

            # release movement lock, if it was previously acquired
            try:
                self.movement_lock.release()
            except Exception:
                self.logger.warning("Tried to release a released lock!")

            # notify android
            self.android_queue.put(AndroidMessage('info', f'Robot is now in {new_mode.title()} mode.'))
            self.logger.info(f"Robot is now in {new_mode.title()} mode.")

            # buzz stm32 (1 time)
            self.stm_link.send("ZZ01")

    def clear_queues(self):
        while not self.command_queue.empty():
            self.command_queue.get()
        while not self.path_queue.empty():
            self.path_queue.get()

    def check_api(self) -> bool:
        url = f"http://{API_IP}:{API_PORT}/status"
        try:
            response = requests.get(url, timeout=1)
            if response.status_code == 200:
                self.logger.debug("API is up!")
                return True
        except ConnectionError:
            self.logger.warning("API Connection Error")
            return False
        except requests.Timeout:
            self.logger.warning("API Timeout")
            return False
        except Exception as e:
            self.logger.warning(f"API Exception: {e}")
            return False

    def request_stitch(self):
        url = f"http://{API_IMAGE_IP}:{API_IMAGE_PORT}/stitch"
        response = requests.get(url)

        # error encountered at the server, return early
        if response.status_code != 200:
            # notify android
            self.android_queue.put(AndroidMessage("error", "Something went wrong when requesting stitch from the API."))
            self.logger.error("Something went wrong when requesting stitch from the API.")
            return

        self.logger.info("Images stitched!")
        self.android_queue.put(AndroidMessage("info", "Images stitched!"))

    def add_navigate_path(self):
        # our hardcoded path
        hardcoded_path: List[str] = [
            "DT20", "SNAPS", "NOOP",
            "FR00", "FL00", "FW30", "BR00", "FW10", "SNAPE", "NOOP",
            "FR00", "FL00", "FW30", "BR00", "FW10", "SNAPN", "NOOP",
            "FR00", "FL00", "FW30", "BR00", "FW10", "SNAPW", "NOOP",
            "FIN"
        ]

        # put commands and paths into queues
        self.clear_queues()
        for c in hardcoded_path:
            self.command_queue.put(c)
            self.path_queue.put({
                "d": 0,
                "s": -1,
                "x": 1,
                "y": 1
            })

        self.logger.info("Navigate-around-obstacle path loaded. Robot is ready to move.")
        self.android_queue.put(AndroidMessage("info", "Navigate-around-obstacle path loaded. Robot is ready to move."))

    @staticmethod
    def outdoorsify(original):
        # for turns, only replace regular 3-1 turns (TL00), with outdoor-calibrated 3-1 turns (TL20)
        # large turns (TL30) do not need to be changed, as they are already calibrated for outdoors
        if original in ["FL00", "FR00", "BL00", "BR00"]:
            return original[:2] + "20"
        elif original.startswith("FW"):
            return original.replace("FW", "FS")
        elif original.startswith("BW"):
            return original.replace("BW", "BS")
        else:
            return original


if __name__ == "__main__":
    rpi = RaspberryPi()
    rpi.start()
