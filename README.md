# ECEN-361-STM32-Lab-07 Queues and Semaphores
## BYU-I  Department of Electrical and Computer Engineering
<!-- div style="text-align: right">Initially Written:  Fall-2023   LRW</div> -->

## Overview / Purpose
The purpose of this lab is to introduce the student to Queues and Semaphores as interprocess communication tools in an RTOS.   This will be done using FreeRTOS.   The Queues, Semaphore and FReeRTOS are automatically configured within this IDE via the STM32MX tool (as brought up by the .ioc file). 

This lab demonstrates a common structure with a queue between producers and consumers.  This code implements two producers and a single consumer:

**Producers:**
1. TTY Keyboard input (typing) accepting and inputing ASCII characters (converted to uppercase when put in the queue)
2.  Periodic process outputting random lower-case ASCII characters
3.  Periodic process outputting random digits

**Consumer:**
1. TTY Screen output processing the queue and putting them into lines on the screen:
    1.)  Uppers:   **THIS HAS BEEN TYPED**
    2.)  Lowers:   **sdkflkjwoueiytn**
    3.)  Numbers:  **8479201847562**


All of the lab will start from a pre-built STM32 'Project.'  If you're reading this, you've probably already accepted and cloned the project into your STM32CubeIDE workspace.  If you need help do that, see the instructions [HERE](./Documentation/Working_with_Labs_from_Github_Classroom_Repository.pdf).

Open and follow the instructions found in the WORD document in your cloned repo: 

>./Documentation/Assignment.md

Modify the document with answers when finished, then commit your repository to github. The commit will upload this document as well and serves as the lab submission.

The final part of the assignment is to paste the URL of this repo into the iLearn Assignment.  

<!----------------------------------->
