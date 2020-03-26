/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
//#include <perf/perf_counter.h>
//#include <px4_module.h>
//#include <px4_module_params.h>
#include <px4_tasks.h>
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

class FollowDgModule
{
public:

        FollowDgModule();

        ~FollowDgModule();

        /**
         * Start the task.
         *
         * @return		OK on success.
         */
        int            start();

        /**
         * Stop the task.
         */
        void		stop();

private:

        orb_advert_t 	_follow_target_pub{nullptr};
        //orb_advert_t       _command_pub{nullptr};

        //int _follow_dg_sub{-1};
        int _vehicle_status_sub{-1};

        follow_trajectory *_last_trajpoint;
        follow_trajectory _follow_trajectory[MAX_POINT];

        float _vel_buffer[VEL_BUFF];
        float *_vel_new;

        bool		_task_should_exit;		/**< if true, task should exit */
        int			_main_task;				/**< handle for task */

        int	_instance{-1};
//        bool  _follow_enable{false};
        int    _serial_fd{-1};

//        int    _vel_num{0};

        float _velocity{0.0f};

        bool          uart_init();

        bool          uart_receive();

        void		task_main();

        /**
         * Shim for calling task_main from task_create.
         */
        static int	task_main_trampoline(int argc, char *argv[]);
};
