/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TCAPTURE 21
#define PRIORITY_TBATTERY 19
#define PRIORITY_TRELOADWD 24
#define PRIORITY_TREINIT 29
#define PRIORITY_TMONITORCOMROBOT 24

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
 */
void Tasks::Init() {
    int status;
    int err;
    th_capture_mode = TH_MODE_NO_ARENA;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: mutex_monitor" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: mutex_robot" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: mutex_robotStarted" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: mutex_move" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: mutex_camera" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: sem_barrier" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_reinit, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: sem_reinit" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: sem_openComRobot" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: sem_serverOk" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: sem_startRobot" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_reloadWDRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: sem_reloadWDRobot" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openCam, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: sem_openCam" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_search_arena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: sem_search_arena" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: th_server" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: th_sendToMon" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: th_receiveFromMon" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: th_openComRobot" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: th_startRobot" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_reloadWdRobot, "th_reloadWDRobot", 0, PRIORITY_TRELOADWD, 0)) {
        cerr << "Error task create: th_reloadWDRobot" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: th_move" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: th_battery" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_camera, "th_camera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: th_camera" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_capture, "th_capture", 0, PRIORITY_TCAPTURE, 0)) {
        cerr << "Error task create: th_capture" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_comRobotMonitor, "th_comRobotMonitor", 0, PRIORITY_TMONITORCOMROBOT, 0)) {
        cerr << "Error task create: th_comRobotMonitor" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", MSG_QUEUE_SIZE, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: q_messageToMon" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Starts tasks
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;
    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: th_server" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: th_sendToMon" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: th_receiveFromMon" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: th_openComRobot " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: th_startRobot" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_reloadWdRobot, (void(*)(void*)) & Tasks::ReloadWdTask, this)) {
        cerr << "Error task start: th_reloadWdRobot" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: th_move" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::GetBatteryTask, this)) {
        cerr << "Error task start: th_battery" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_camera, (void(*)(void*)) & Tasks::OpenCamTask, this)) {
        cerr << "Error task start: th_camera" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_capture, (void(*)(void*)) & Tasks::CaptureImageTask, this)) {
        cerr << "Error task start: th_capture" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_comRobotMonitor, (void(*)(void*)) & Tasks::ComRobotMonitorTask, this)) {
        cerr << "Error task start: th_comRobotMonitor" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Stops tasks
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
    camera.Close();
}

/**
 * @brief Task synchronization
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Reset the robot state
 */
void Tasks::ResetRobot() {

    int status;

    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
    if (robotStarted) {
        //cout << "reset" << endl;
        Message * tmp;
        int tmpCpt = 0;
        do {
            tmp = robot.Write(robot.Reset());
            tmpCpt++;
        } while (tmp->GetID() != MESSAGE_ANSWER_ACK && tmpCpt < 3);
    }
    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    robotStarted = 0;
    rt_mutex_release(&mutex_robotStarted);
    status = robot.Close();
    rt_mutex_release(&mutex_robot);

    move = MESSAGE_ROBOT_STOP;
    watchdog = false;
    compteur = 0;
    cout << "Close robot status = [ " << status << " ]" << endl;
    rt_sem_v(&sem_openComRobot);
}

/**
 * @brief Restarting the supervisor in case of connection lost
 */
void Tasks::ReinitTask(void *arg) {
    ResetRobot();
    //Reinitialize the global variables.
    startCapture = false;
    findRobot = false;
    th_capture_mode = TH_MODE_NO_ARENA;
    compteur = 0;
    robotAnswer = -1;

    //Close all connections
    this->Stop();

    //Restart tasks

    //Delete tasks
    int err = 0;
    cout << "Deleting tasks..." << endl << flush;
    if (err = rt_task_delete(&th_sendToMon)) {
        cerr << "Error task deletion: th_sendToMon" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_delete(&th_openComRobot)) {
        cerr << "Error task deletion: th_openComRobot " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_delete(&th_startRobot)) {
        cerr << "Error task deletion: th_startRobot" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_delete(&th_reloadWdRobot)) {
        cerr << "Error task deletion: th_reloadWdRobot" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_delete(&th_move)) {
        cerr << "Error task deletion: th_move" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_delete(&th_battery)) {
        cerr << "Error task deletion: th_battery" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_delete(&th_camera)) {
        cerr << "Error task deletion: th_camera" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_delete(&th_capture)) {
        cerr << "Error task deletion: th_capture" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_delete(&th_comRobotMonitor)) {
        cerr << "Error task deletion: th_comRobotMonitor " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks successfully deleted" << endl << flush;

    //Delete queues
    if ((err = rt_queue_delete(&q_messageToMon)) < 0) {
        cerr << "Error queue deletion: q_messageToMon" << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues successfully deleted" << endl;

    //Initialize mutexes, semaphores and tasks
    this->Init();
    //Start tasks
    this->Run();
    //Task synchronization
    this->Join();
    cout << "Supervisor reinitialised with success." << endl;
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        msg = ReadInQueue(&q_messageToMon);
        if (PERIODIC_DEBUG)
            cout << "SENDING To Monitor -> " << msg->ToString() << endl;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    bool run = true;
    while (run) {
        msgRcv = monitor.Read();
        if (DEBUG)
            cout << "Message received <= " << msgRcv->ToString() << " id = " << msgRcv->GetID() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            cout << endl << "Connection with Monitor Lost!" << endl;
            //Start the Reinit task
            int err;
            if (err = rt_task_delete(&th_reinit)) {
                cerr << endl << endl << "Error Task Reinit deletion: " << strerror(-err) << endl << endl << flush;
            }
            if (err = rt_task_create(&th_reinit, "th_reinit", 0, PRIORITY_TREINIT, 0)) {
                cerr << "Error Task Reinit creation: " << strerror(-err) << endl << flush;
                exit(EXIT_FAILURE);
            }
            if (err = rt_task_start(&th_reinit, (void(*)(void*)) & Tasks::ReinitTask, this)) {
                cerr << "Error Task Reinit start: " << strerror(-err) << endl << flush;
                exit(EXIT_FAILURE);
            }
            //Stop condition for ReceiveFromMon
            run = false;
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_CLOSE)) {
            cout << endl << endl << "Closing communication with robot-kun" << endl << endl;
            robot.Close();
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            watchdog = false;
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            watchdog = true;
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_RESET)) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robotAnswer = robot.Write(robot.Reset())->GetID();
            rt_mutex_release(&mutex_robot);
            watchdog = false;
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 2;
            rt_mutex_release(&mutex_robotStarted);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            th_capture_mode = TH_MODE_NO_ARENA;
            rt_sem_v(&sem_openCam);
        } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            startCapture = false;
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            camera.Close();
            rt_mutex_release(&mutex_camera);
            WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_ACK));
        } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            int status;
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            status = camera.IsOpen();
            rt_mutex_release(&mutex_camera);
            if (status) {
                th_capture_mode = TH_MODE_SEARCH_ARENA;
            }
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            int status;
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            status = camera.IsOpen();
            rt_mutex_release(&mutex_camera);
            if (status) {
                th_capture_mode = TH_MODE_WITH_ARENA;
                rt_sem_v(&sem_search_arena);
            }
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            int status;
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            status = camera.IsOpen();
            rt_mutex_release(&mutex_camera);
            if (status) {
                th_capture_mode = TH_MODE_NO_ARENA;
                rt_sem_v(&sem_search_arena);
            }
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
            findRobot = true;
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
            findRobot = false;
        }
        delete(msgRcv); // Must be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_ROBOT_ERROR);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/

    while (1) {

        rt_sem_p(&sem_startRobot, TM_INFINITE);
        Message * toSend = (watchdog) ? robot.StartWithWD() : robot.StartWithoutWD();
        Message * msgSend;
        //rt_sem_p(&sem_startRobot, TM_INFINITE);

        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(toSend);
        rt_mutex_release(&mutex_robot);

        robotAnswer = msgSend->GetID();

        if (DEBUG) {
            std::string str = (watchdog) ? "Start with wd answer: " : "Start without wd answer: ";
            cout << str << msgSend->ToString() << endl << flush;
        }

        if (msgSend->GetID() == MESSAGE_ANSWER_ROBOT_TIMEOUT)
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        else if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            if (DEBUG) {
                std::string str = (watchdog) ? "Watchdog ON!!!!!!!!!!!! " : "Watchdog OFF!!!!!!!!!!!!";
                cout << str << msgSend->ToString() << endl << flush;
            }
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            rt_sem_v(&sem_reloadWDRobot);
        }
        WriteInQueue(&q_messageToMon, msgSend);
    }
}

/**
 * @brief Thread resetting the robot watchdog timeout timer every 1s.
 */
void Tasks::ReloadWdTask(void* arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    /**************************************************************************************/
    /* The task Reload watch dog starts here                                                    */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
    Message * response;
    while (1) {
        if (watchdog) {
            rt_task_wait_period(NULL);
            if (robotStarted == 1) {
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                response = robot.Write(robot.ReloadWD());
                rt_mutex_release(&mutex_robot);
                robotAnswer = response->GetID();
                if (DEBUG)
                    cout << endl << "Watchdog reset answer : " << response->ToString() << endl;
            }
        } else {
            rt_sem_p(&sem_reloadWDRobot, TM_INFINITE);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    //int rs;
    int cpMove;
    Message * answer;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        if (robotStarted == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            answer = robot.Write(new Message((MessageID) cpMove));
            rt_mutex_release(&mutex_robot);
            if (PERIODIC_DEBUG)
                cout << "Move answer " << answer->ToString() << endl;
            robotAnswer = answer->GetID();
        }
    }
}

/**
 * @brief Thread asking for battery level.
 */
void Tasks::GetBatteryTask(void * arg) {
    Message * batteryLevel;
    int rs;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    while (1) {
        rt_task_wait_period(NULL);
        if (robotStarted == 1) {

            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            if (robotStarted)
                batteryLevel = robot.Write(robot.GetBattery());
            rt_mutex_release(&mutex_robot);
            if (PERIODIC_DEBUG)
                cout << "Battery Answer " << batteryLevel->ToString() << endl;
            robotAnswer = batteryLevel->GetID();

            //ComRobotMonitor will send failure messages later.
            if (!(batteryLevel->GetID() == MESSAGE_ANSWER_ROBOT_TIMEOUT
                    || batteryLevel->GetID() == MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND
                    || batteryLevel->GetID() == MESSAGE_ANSWER_ROBOT_ERROR
                    || batteryLevel->GetID() == MESSAGE_ANSWER_COM_ERROR))
                WriteInQueue(&q_messageToMon, batteryLevel);
        }

    }
}

/**
 * @brief Thread opening communication with the camera.
 */
void Tasks::OpenCamTask(void *arg) {
    bool status;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);


    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openCam, TM_INFINITE);
        cout << "Open camera (";
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        status = camera.Open();
        rt_mutex_release(&mutex_camera);
        cout << status << ")" << endl << flush;

        Message * msgSend;
        if (!status) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            WriteInQueue(&q_messageToMon, msgSend);
        } else
            startCapture = true;
    }
}

/**
 * @brief Thread starting capture with the camera.
 */
void Tasks::CaptureImageTask(void * arg) {
    MessageImg* msg;
    bool status;
    Arena arena;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        status = camera.IsOpen();
        rt_mutex_release(&mutex_camera);
        if (status && startCapture) {
            if (th_capture_mode == TH_MODE_NO_ARENA) { //Mode with normal video
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                msg = new MessageImg(MESSAGE_CAM_IMAGE, camera.Grab().Copy());
                rt_mutex_release(&mutex_camera);
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msg);
                rt_mutex_release(&mutex_monitor);
            } else if (th_capture_mode == TH_MODE_WITH_ARENA) { //Mode with the arena drawn over the video 
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                Img * img = camera.Grab().Copy();
                rt_mutex_release(&mutex_camera);
                if (findRobot) { //If we need to find the position of the robot
                    std::list<Position> positions = img->SearchRobot(arena);
                    if (positions.size() > 0) {
                        img->DrawRobot(positions.front());
                        WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION, positions.front()));
                    } else { //If the robot was not found, send (-1,-1)
                        Position p;
                        p.center = cv::Point2f(-1, -1);
                        WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION, p));
                    }
                }

                img->DrawArena(arena); //Draw the arena on top of the image
                msg = new MessageImg(MESSAGE_CAM_IMAGE, img);
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msg);
                rt_mutex_release(&mutex_monitor);
            } else if (th_capture_mode == TH_MODE_SEARCH_ARENA) { //Mode that sends one image and tries to draw the arena on top
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                Img * img = camera.Grab().Copy();
                rt_mutex_release(&mutex_camera);
                arena = img->SearchArena();

                cout << arena.ToString() << endl;
                if (!arena.IsEmpty()) { //If the arena was found
                    if (DEBUG)
                        cout << "Arena found" << endl;
                    img->DrawArena(arena);
                    msg = new MessageImg(MESSAGE_CAM_IMAGE, img);
                    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                    monitor.Write(msg);
                    rt_mutex_release(&mutex_monitor);
                    rt_sem_p(&sem_search_arena, TM_INFINITE);
                } else { //If the arena is was not found we go back to normal video mode
                    if (DEBUG)
                        cout << "Arena not found" << endl;
                    th_capture_mode = TH_MODE_NO_ARENA;
                    WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK));
                }
            }
        }

    }
}

/**
 * @brief Thread monitoring com with robot.
 */
void Tasks::ComRobotMonitorTask(void* arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    /**************************************************************************************/
    /* The task Reload watch dog starts here                                                    */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        rt_task_wait_period(NULL);
        if (robotAnswer == MESSAGE_ANSWER_ROBOT_TIMEOUT
                || robotAnswer == MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND
                || robotAnswer == MESSAGE_ANSWER_ROBOT_ERROR
                || robotAnswer == MESSAGE_ANSWER_COM_ERROR) { //If the message monitor finds an error message we update our counter
            compteur++;
            if (DEBUG)
                cout << "Counter activated! " << compteur << "<--" << robotAnswer << endl;
        } else if (robotAnswer != -1) { // We ignore tha -1 message. It is used to restart the global variable
            if (DEBUG && compteur != 0)
                cout << robotAnswer << " Reinitializing counter " << endl;
            compteur = 0;
        }
        if (compteur >= 3) { //We reached the failure threshold
            cout << endl << "Connection lost. Reseting robot... " << endl;
            WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_ROBOT_TIMEOUT));
            ResetRobot();
        }
        robotAnswer = -1;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

