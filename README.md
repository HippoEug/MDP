MDP Code is built on [Group 28's](https://github.com/CZ3004-Group-28) code.

- Only 1 image recognition model file (best.pt) in /image_rec/Weights/ is used for Task 1 and Task 2.
- File /image_rec/main.py and /image_rec/model.py is used for the image recognition inference server on your own machine.
- File /image_rec/interface.py is used to test the inference server.

Task 2:
- task2.py is used for Task2 only, to be run on RPI
- But be warned though, this code is freaking slow, we were the slowest successful group
- Movement Command Calls sent to the STM should be blocked with Acks before the next instruction in the future..
- Ultrasonic sensor is placed on the front of the car
- 2 IR sensors are placed on both the left side and right side of the car
- Task2 is really RNG.. good luck.
