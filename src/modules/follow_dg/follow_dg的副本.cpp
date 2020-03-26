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

#include <drivers/drv_hrt.h>
//#include <matrix/math.hpp>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <termios.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
//#include <px4_workqueue.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/follow_dg.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

//#define SCHEDULE_INTERVAL	200000	/**< The schedule interval in usec (20 Hz) */
#define MIN_DISTANCE (float)0.0
#define MAX_DISTANCE (float)200.0
#define MSG_LEN 25
#define LEAST_POINT 1
#define MAX_POINT LEAST_POINT+2
#define VEL_BUFF 5
//using matrix::Dcmf;
//using matrix::Quatf;
//using matrix::Vector2f;
//using matrix::Vector3f;

typedef struct
{
    //char head[4];
    uint8_t len;
    double lat;
    double lon;
    float vel;
    float alt;
    //uint8_t sum_check;
}follow_trajectory;

class FollowDgModule : public ModuleBase<FollowDgModule>, public ModuleParams
{
public:

        FollowDgModule();

        ~FollowDgModule();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

        /** @see ModuleBase */
        static FollowDgModule *instantiate(int argc, char *argv[]);

         //run the main loop
        //void cycle();

        /** @see ModuleBase::run() */
        void run() override;

	int print_status() override;

private:

        orb_advert_t 	_follow_target_pub{nullptr};
        //orb_advert_t       _command_pub{nullptr};

        //int _follow_dg_sub{-1};
        int _vehicle_status_sub{-1};

        follow_trajectory *_last_trajpoint;
        follow_trajectory _follow_trajectory[MAX_POINT];

        float _vel_buffer[VEL_BUFF];
        float *_vel_new;
//        perf_counter_t _perf_elapsed{};
//        perf_counter_t _perf_interval{};

        int	_instance{-1};
//        bool  _follow_enable{false};
        int    _serial_fd{-1};

//        int    _vel_num{0};

        float _velocity{0.0f};

//	DEFINE_PARAMETERS(

//	)

//        static void	cycle_trampoline(void *arg);

//        int                  start();

//        void		update_params();

//	bool 		subscribe_topics();

        bool          uart_init();

        bool          uart_receive();

        //void          send_command(int mode); /**< mode = 8 ->follow target; = 3 -> loiter */
};

//work_s	FollowDgModule::_work = {};

FollowDgModule::FollowDgModule():
	ModuleParams(nullptr)
{
        //_follow_dg_sub = orb_subscribe(ORB_ID(follow_dg));
        _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	// initialise parameters
        //update_params();

        memset(_follow_trajectory, 0, sizeof(_follow_trajectory));
        _last_trajpoint = _follow_trajectory;

        memset(_vel_buffer, 0, sizeof(_vel_buffer));
        _vel_new = _vel_buffer;

//        _perf_elapsed = perf_alloc_once(PC_ELAPSED, "wind_estimator elapsed");
//        _perf_interval = perf_alloc_once(PC_INTERVAL, "wind_estimator interval");
}

FollowDgModule::~FollowDgModule()
{
        //orb_unsubscribe(_follow_dg_sub);
        orb_unsubscribe(_vehicle_status_sub);
        orb_unadvertise(_follow_target_pub);
        //orb_unadvertise(_command_pub);

//        perf_free(_perf_elapsed);
//        perf_free(_perf_interval);
}

FollowDgModule *FollowDgModule::instantiate(int argc, char *argv[])
{
        FollowDgModule *instance = new FollowDgModule();

        return instance;
}

int
FollowDgModule::task_spawn(int argc, char *argv[])
{
        /* schedule a cycle to start things */
//        work_queue(LPWORK, &_work, (worker_t)&FollowDgModule::cycle_trampoline, nullptr, 0);

//        // wait until task is up & running
//        if (wait_until_running() < 0) {
//                _task_id = -1;

//        } else {
//                _task_id = task_id_is_work_queue;
//                return PX4_OK;
//        }

//        return PX4_ERROR;

     _task_id = px4_task_spawn_cmd("follow_dg",
                                    SCHED_DEFAULT,
                                    SCHED_PRIORITY_DEFAULT,
                                    3000,
                                    (px4_main_t)&run_trampoline,
                                    (char *const *)argv);

        if (_task_id < 0) {
                _task_id = -1;
                return -errno;
        }

        return 0;

}

//void
//FollowDgModule::cycle_trampoline(void *arg)
//{
//        FollowDgModule *dev = reinterpret_cast<FollowDgModule *>(arg);

//        // check if the trampoline is called for the first time
//        if (!dev) {
//                dev = new FollowDgModule();

//                if (!dev) {
//                        PX4_ERR("alloc failed");
//                        return;
//                }

//                _object.store(dev);
//        }

//        if (dev) {
//                dev->cycle();
//        }
//}

bool
FollowDgModule::uart_init()
{
    char const *uart_name = "/dev/ttyS3";
    //char const *uart_name = "/dev/ttyS1";
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
            //printf("nread is %d\n", nread);

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
                            //printf("lon is %.6f, lat is %.6f, alt is %.1f\n",new_trajpoint.lon, new_trajpoint.lat, (double)new_trajpoint.alt);

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
FollowDgModule::run()
{
//	perf_count(_perf_interval);
//	perf_begin(_perf_elapsed);

    //static struct follow_dg_s follow_dg_msg = {};
    static struct vehicle_status_s vehicle_status_msg = {};
    //orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &vehicle_status_msg);

    px4_pollfd_struct_t fds[1];
    //fds[0].fd = _follow_dg_sub;
    fds[0].fd = _vehicle_status_sub;
    fds[0].events = POLLIN;

    if (!uart_init()) return;

    while(!should_exit()){

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
                //printf("vehicle_status is %d\n", vehicle_status_msg);
        }
//         else printf("vehicle_status not update\n");

        if (vehicle_status_msg.nav_state == 19){
            tcflush(_serial_fd, TCIOFLUSH);
            bool getnew = uart_receive();
            //printf("uart getnew is %d\n", getnew);
            //printf("_follow_trajectory lenth is %d\n", (_last_trajpoint - _follow_trajectory) );
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
}

//void FollowDgModule::update_params()
//{
//	updateParams();

//}

int FollowDgModule::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
                int ret = FollowDgModule::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int FollowDgModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

        PRINT_MODULE_USAGE_NAME("follow_dg", "DG modules");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FollowDgModule::print_status()
{
//	perf_print_counter(_perf_elapsed);
//	perf_print_counter(_perf_interval);

	if (_instance > -1) {
                uORB::Subscription<follow_target_s> est{ORB_ID(follow_target), (unsigned)_instance};
		est.update();

		print_message(est.get());
	} else {
		PX4_INFO("Running, but never published");
	}

	return 0;
}

extern "C" __EXPORT int follow_dg_main(int argc, char *argv[]);

int
follow_dg_main(int argc, char *argv[])
{
        return FollowDgModule::main(argc, argv);
}
