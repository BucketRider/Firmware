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

#include "mavlink/mavlink_mission.h"

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

#include <uORB/topics/dg_mission.h>

class MissionDgModule : public ModuleBase<MissionDgModule>, public ModuleParams
{
public:
        MissionDgModule();

        ~MissionDgModule();

        /** @see ModuleBase */
        static int task_spawn(int argc, char *argv[]);

        /** @see ModuleBase */
        static int custom_command(int argc, char *argv[]);

        /** @see ModuleBase */
        static int print_usage(const char *reason = nullptr);

        /** @see ModuleBase */
        static MissionDgModule *instantiate(int argc, char *argv[]);

         //run the main loop
        //void cycle();

        /** @see ModuleBase::run() */
        void run() override;

private:
        MavlinkMissionManager _dg_mission_manager;

        int                     _mission_dg_sub{-1};

        bool                  _mission_enable{false};

        //dg_mission_s      _mission{};
};


MissionDgModule::MissionDgModule():
        ModuleParams(nullptr),
      _dg_mission_manager(nullptr)
{

    _mission_dg_sub = orb_subscribe(ORB_ID(dg_mission));

}

MissionDgModule::~MissionDgModule()
{
    orb_unsubscribe(_mission_dg_sub);
}

MissionDgModule *MissionDgModule::instantiate(int argc, char *argv[])
{
        MissionDgModule *instance = new MissionDgModule();

        return instance;
}

int
MissionDgModule::task_spawn(int argc, char *argv[])
{
     _task_id = px4_task_spawn_cmd("mission_dg",
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

void
MissionDgModule::run()
{

    px4_pollfd_struct_t fds[1];
    fds[0].fd = _mission_dg_sub;
    fds[0].events = POLLIN;

    while(!should_exit()){

        if (poll(&fds[0], 1, 500) > 0){
            orb_copy(ORB_ID(dg_mission), _mission_dg_sub, &_dg_mission_manager.dg_mission);
            mavlink_message_t msg_empty ={};
            _dg_mission_manager.dg_mission_enable =true;
            if (_dg_mission_manager.dg_mission.msg_type == 0){
                _dg_mission_manager.handle_mission_item(&msg_empty);
            }
            else if (_dg_mission_manager.dg_mission.msg_type == 1){
                _dg_mission_manager.handle_mission_clear_all(&msg_empty);
                _dg_mission_manager.handle_mission_count(&msg_empty);
            }
            else if (_dg_mission_manager.dg_mission.msg_type == 2){
                _dg_mission_manager.handle_mission_set_current(&msg_empty);
            }
//            else if (dg_manager.dg_mission.msg_type == 4){
//                dg_manager.add_transfer_count();
//                dg_manager.handle_mission_item(&msg_empty);
//            }
            _dg_mission_manager.dg_mission_enable =false;
        }
    }
}

//void FollowDgModule::update_params()
//{
//	updateParams();

//}

int MissionDgModule::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
                int ret = MissionDgModule::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int MissionDgModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

        PRINT_MODULE_USAGE_NAME("mission_dg", "DG modules");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mission_dg_main(int argc, char *argv[]);

int
mission_dg_main(int argc, char *argv[])
{
        return MissionDgModule::main(argc, argv);
}
