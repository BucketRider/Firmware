/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include"follow_dg.hpp"

namespace follow_dg_module
{
FollowDgModule	*instance;
}

//work_s	FollowDgModule::_work = {};

FollowDgModule::FollowDgModule():
       _task_should_exit(false),
      _main_task(-1)
{
        //_follow_dg_sub = orb_subscribe(ORB_ID(follow_dg));
        //_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

        memset(_follow_trajectory, 0, sizeof(_follow_trajectory));
        _last_trajpoint = _follow_trajectory;

        memset(_vel_buffer, 0, sizeof(_vel_buffer));
        _vel_new = _vel_buffer;
}

FollowDgModule::~FollowDgModule()
{
        if (_main_task != -1) {

            /* task wakes up every 100ms or so at the longest */
            _task_should_exit = true;

            /* wait for a second for the task to quit at our request */
            unsigned i = 0;

            do {
                    /* wait 20ms */
                    px4_usleep(20000);

                    /* if we have given up, kill it */
                    if (++i > 50) {
                            px4_task_delete(_main_task);
                            break;
                    }
            } while (_main_task != -1);
        }

        //orb_unsubscribe(_follow_dg_sub);
//        orb_unsubscribe(_vehicle_status_sub);
//        orb_unadvertise(_follow_target_pub);
        //orb_unadvertise(_command_pub);

        follow_dg_module::instance = nullptr;
}

int
FollowDgModule::start()
{

        /* start the task */
        _main_task = px4_task_spawn_cmd("follow_dg",
                                        SCHED_DEFAULT,
                                        SCHED_PRIORITY_DEFAULT + 15,
                                        3000,
                                        (px4_main_t)&FollowDgModule::task_main_trampoline,
                                        nullptr);

        if (_main_task < 0) {
                warn("task start failed");
                return -errno;
        }

        return OK;

}

void
FollowDgModule::stop()
{
        if (follow_dg_module::instance != nullptr) {
                delete (follow_dg_module::instance);
        }
}

bool
FollowDgModule::uart_init()
{
    //char const *uart_name = "/dev/ttyS3";
    char const *uart_name = "/dev/ttyS1";
    _serial_fd = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY);
     if (_serial_fd < 0) {
             printf("failed to open port: %s\n", uart_name);
             return false;
     }
     printf("Open the %s\n",uart_name);

     struct termios uart_config;

     int termios_state;

     int speed = B115200;

     tcgetattr(_serial_fd, &uart_config); // 获取终端参数

     /* clear ONLCR flag (which appends a CR for every LF) */
     uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

     /* 无偶校验，一个停止位 */
     uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
     // CSTOPB 使用两个停止位，PARENB 表示偶校验, CRTSCTS 使用流控

     cfmakeraw(&uart_config);

      /* 设置波特率 */
     if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
              printf("ERR: %d (cfsetispeed)\n", termios_state);
             return false;
     }

     if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
               printf("ERR: %d (cfsetospeed)\n", termios_state);
             return false;
     }
     // 设置与终端相关的参数，TCSANOW 立即改变参数
     if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
             printf("ERR: %d (tcsetattr)\n", termios_state);
             return false;
     }
     return true;
}

//void
//FollowDgModule::send_command(int mode)
//{
//    static struct vehicle_command_s _command= {};

//    _command.command = 176;
//    _command.param1 = 189;
//    _command.param2 = 4;
//    _command.param3 = mode;
//    _command.param4 = 0;
//    _command.param5 = 0;
//    _command.param6 = 0;
//    _command.param7 = 0;
//    _command.target_system = 1;
//    _command.target_component =1;
//    _command.source_system =255;
//    _command.source_component =0;
//    _command.confirmation =0;
//    _command.from_external =1;
//    _command.timestamp = hrt_absolute_time();

//    int inst;
//    orb_publish_auto(ORB_ID(vehicle_command), &_command_pub, &_command, &inst, ORB_PRIO_DEFAULT);
//}

bool FollowDgModule::uart_receive(){

    uint8_t buffer[5 * MSG_LEN] ={};
    int nread = 0;
    int read_finish = 0;
    int remain =0;
    int error_count = 0;
    bool pass_check = true;
    uint8_t len = 0;

    px4_pollfd_struct_t fds[1];
    fds[0].fd = _serial_fd;
    fds[0].events = POLLIN;

    while(1){

       if (error_count > 20) {
           return false;
        }

          //usleep(20000);
          if (poll(&fds[0], 1, 20) > 0)
          {
          //printf("error_count is %d\n", error_count);
            nread= read(_serial_fd, &buffer[remain], sizeof(buffer) - (size_t)remain);
            printf("nread is %d\n", nread);

//          for (int i=0;i< (int)sizeof(buffer) - remain; i++){
//              nread= read(_serial_fd, &buffer[remain + i], 1);
//              printf("nread is %d\n", nread);
//              printf("buffer %d is %d\n", i, buffer[remain + i]);
//          }

            if (nread <= 0) {
                nread =0;
                error_count++;
            }
            {
                for ( read_finish = 0; read_finish < (nread + remain); ) {
                    if (buffer[read_finish] == '$'){
                        if ((nread + remain - read_finish) < MSG_LEN) {
                            error_count++;
                            break;
                        }
                        else
                        {
                            char head[3] = {'G','P','S'};
                            pass_check = true;

                            //check head '$GPS' if not read_finish++
                            for (int i = 0; i < 3; ++i) {
                                 if ((buffer[read_finish +1+ i]) != (uint8_t)head[i]){
                                     pass_check =false;
                                     break;
                                 }
                            }
                            if (!pass_check){
                                read_finish++;
                                continue;
                            }

                            // check data available and sum check, if not read_finish + len
                            len = buffer[read_finish + 4];

                            if (buffer[read_finish + 5] != 'A'){
                                pass_check =false;
                                printf("available char is not A\n");
                            }
                            if (pass_check){
                                uint8_t sum = 0;
                                for (int i=0; i < len -1; i++){
                                    sum += buffer[read_finish + i];
                                }
                                if (sum != buffer[len -1]) {
                                    pass_check =false;
                                    printf("sum check is errror\n");
                                }
                            }
                            if (!pass_check){
                                read_finish += len;
                                continue;
                            }

                            follow_trajectory new_trajpoint;
                            new_trajpoint.lon = (double)*(uint32_t*)((uint32_t)buffer+ read_finish + 6) * 1e-7;
                            new_trajpoint.lon = (buffer[read_finish + 10] == 'E') ? new_trajpoint.lon : -new_trajpoint.lon;
                            new_trajpoint.lat = (double)*(uint32_t*)((uint32_t)buffer+ read_finish + 11) * 1e-7;
                            new_trajpoint.lon = (buffer[read_finish + 15] == 'N') ? new_trajpoint.lon : -new_trajpoint.lon;
                            new_trajpoint.vel = (float)*(uint32_t*)((uint32_t)buffer+ read_finish + 16) * 0.001f *0.5144f;
                            new_trajpoint.alt = (float)*(uint32_t*)((uint32_t)buffer+ read_finish + 20) *0.1f;
                            printf("lon is %.6f, lat is %.6f, alt is %.1f\n",new_trajpoint.lon, new_trajpoint.lat, (double)new_trajpoint.alt);

                            read_finish += len;

                            float dist_xy;
                            float dist_z;
                            follow_trajectory* prev_trajpoint = (_last_trajpoint == _follow_trajectory) ? _last_trajpoint : (_last_trajpoint-1);

//                            get_distance_to_point_global_wgs84(new_trajpoint.lon, new_trajpoint.lat, 0.0,
//                                                               prev_trajpoint->lon, prev_trajpoint->lat, 0.0,
//                                                                &dist_xy, &dist_z);

                            struct map_projection_reference_s target_ref;
                            map_projection_init(&target_ref, prev_trajpoint->lat, prev_trajpoint->lon);
                            map_projection_project(&target_ref, new_trajpoint.lat, new_trajpoint.lon, &dist_xy, &dist_z);
                            dist_xy = sqrtf(dist_xy * dist_xy + dist_z * dist_z);

                            //printf("Distance is %.3f\n", (double)fabsf(dist_xy));

                            if (fabsf(dist_xy) > 100.0f && _last_trajpoint != _follow_trajectory) {
                                error_count ++;
                                continue;
                            }

//                            if ((float)new_trajpoint.lon < -180.0f || (float)new_trajpoint.lon > 180.0f) continue;
//                            if ((float)new_trajpoint.lat < -90.0f || (float)new_trajpoint.lat > 90.0f) continue;

//                            if ((float)new_trajpoint.lon < 116.27f || (float)new_trajpoint.lon > 116.29f) continue;
//                            if ((float)new_trajpoint.lat < 40.05f || (float)new_trajpoint.lat > 40.06f) continue;

                            if (_vel_new != &_vel_buffer[VEL_BUFF -1]){
                                *_vel_new = new_trajpoint.vel;
                                _vel_new++;
                            } else {
                                 *_vel_new = new_trajpoint.vel;
                                _vel_new = _vel_buffer;
                            }
                            //_vel_num++;
                            //if (_vel_num > VEL_BUFF) _vel_num = VEL_BUFF;

                            _velocity = 0.0f;
                            for (int i = 0; i < VEL_BUFF /*_vel_num*/; i++){
                                _velocity += _vel_buffer[i];
                            }
                             //_velocity = _velocity/_vel_num;
                            _velocity = _velocity/VEL_BUFF;

                            if (fabsf(dist_xy) > MIN_DISTANCE){
                                new_trajpoint.vel = _velocity;
                                memcpy(_last_trajpoint, &new_trajpoint, sizeof(follow_trajectory));
                                _last_trajpoint ++;
                                return true;
                            }
//                            else {
//                                error_count++;
//                                break;
//                            }
                        }
                    }
                else {
                     read_finish++;
                }
            }
                error_count ++;

            remain = nread + remain - read_finish;
            uint8_t buffer_move[5 * MSG_LEN] = {};
            memcpy(buffer_move, &buffer[read_finish], (size_t)remain);
            memcpy(buffer, buffer_move, sizeof(buffer_move));
          }
       }
          else error_count++;
    }
    return false;

}

void
FollowDgModule::task_main()
{
     _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    //static struct follow_dg_s follow_dg_msg = {};
    static struct vehicle_status_s vehicle_status_msg = {};
    //orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &vehicle_status_msg);

    px4_pollfd_struct_t fds[1];
    //fds[0].fd = _follow_dg_sub;
    fds[0].fd = _vehicle_status_sub;
    fds[0].events = POLLIN;

    if (!uart_init()) return;

    while(!_task_should_exit){

//        if (poll(&fds[0], 1, 100) > 0){
//            orb_copy(ORB_ID(follow_dg), _follow_dg_sub, &follow_dg_msg);
//            if (follow_dg_msg.enable != (vehicle_status_msg.nav_state == 19)){
//                if (follow_dg_msg.enable){
//                    send_command(8);
//                    tcflush(_serial_fd, TCIOFLUSH);
//                    printf("follow enable \n");
//                }
//                else {
//                    send_command(3);
//                    printf("follow disable \n");
//                }
//                //_follow_enable =  follow_dg_msg.enable;
//            }
//        }

        if (poll(&fds[0], 1, 50) > 0){
                orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &vehicle_status_msg);
                printf("vehicle_status is %d\n", vehicle_status_msg.nav_state);
        }
         else printf("vehicle_status not update\n");

        if (vehicle_status_msg.nav_state == 19){
            tcflush(_serial_fd, TCIOFLUSH);
            bool getnew = uart_receive();
            printf("uart getnew is %d\n", getnew);
            printf("_follow_trajectory lenth is %d\n", (_last_trajpoint - _follow_trajectory) );
            if ((_last_trajpoint - _follow_trajectory) > LEAST_POINT -1){

                struct follow_target_s follow_msg;
                follow_msg.timestamp = hrt_absolute_time();
                follow_msg.lat = _follow_trajectory[0].lat;
                follow_msg.lon = _follow_trajectory[0].lon;
                follow_msg.alt = _follow_trajectory[0].alt;

                float angel = 0;
                angel = get_bearing_to_next_waypoint(_follow_trajectory[0].lat, _follow_trajectory[0].lon,
                                                                      _follow_trajectory[1].lat, _follow_trajectory[1].lon);
//                follow_msg.vx = getnew? (float)((double)_follow_trajectory[0].vel * cos(angel)) : 0;
//                follow_msg.vy = getnew? (float)((double)_follow_trajectory[0].vel * sin(angel)) : 0;
                follow_msg.vx = (float)((double)_velocity * cos(angel));
                follow_msg.vy = (float)((double)_velocity * sin(angel));
                follow_msg.vz = 0;

                if(getnew){
                //int orb_pub =
                orb_publish_auto(ORB_ID(follow_target), &_follow_target_pub, &follow_msg, &_instance, ORB_PRIO_DEFAULT);
                //printf("orb_publish is %d\n",orb_pub);
                }

                if (getnew && ((_last_trajpoint - _follow_trajectory) > LEAST_POINT))
                {
                    follow_trajectory follow_trajectory_move[MAX_POINT];
                    memcpy(follow_trajectory_move, &_follow_trajectory[1], sizeof(follow_trajectory) * (MAX_POINT-1));
                    memcpy(_follow_trajectory, follow_trajectory_move, sizeof(follow_trajectory_move));
                    _last_trajpoint--;
                }
            }
        }
    }

    _main_task = -1;
    close(_serial_fd);
    orb_unsubscribe(_vehicle_status_sub);
    orb_unadvertise(_follow_target_pub);
}

int
FollowDgModule::task_main_trampoline(int argc, char *argv[])
{
        follow_dg_module::instance->task_main();
        return 0;
}

extern "C" __EXPORT int follow_dg_main(int argc, char *argv[]);

int follow_dg_main(int argc, char *argv[])
{
    if (argc < 2) {
            warnx("usage: follow_dg {start|stop|status}");
            return 1;
    }

    if (!strcmp(argv[1], "start")) {

            if (follow_dg_module::instance != nullptr) {
                    warnx("already running");
                    return 1;
            }

            follow_dg_module::instance = new FollowDgModule();

            if (follow_dg_module::instance == nullptr) {
                    warnx("alloc failed");
                    return 1;
            }

            if (OK != follow_dg_module::instance->start()) {
                    delete follow_dg_module::instance;
                    follow_dg_module::instance = nullptr;
                    warnx("start failed");
                    return 1;
            }

            return 0;
    }

    if (!strcmp(argv[1], "stop")) {
            if (follow_dg_module::instance == nullptr) {
                    warnx("not running");
                    return 1;
            }

            delete follow_dg_module::instance;
            follow_dg_module::instance = nullptr;
            return 0;
    }

    if (!strcmp(argv[1], "status")) {
            if (follow_dg_module::instance) {
                    warnx("running");
                    return 0;

            } else {
                    warnx("not running");
                    return 1;
            }
    }

    warnx("unrecognized command");
    return 1;
}
