#include <string.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

#include "param_server.h"
#include "uORB/uORBManager.hpp"

// uORB topics needed to keep parameter server and client in sync
#include <uORB/topics/parameter_client_set_request.h>
#include <uORB/topics/parameter_client_set_response.h>
#include <uORB/topics/parameter_server_set_used.h>
#include <uORB/topics/parameter_server_set_value.h>

static orb_advert_t param_set_req_topic = nullptr;
static int          param_set_rsp_topic = PX4_ERROR;
static px4_task_t   sync_thread_tid;
static const char  *sync_thread_name = "server_sync_thread";

static int param_sync_thread(int argc, char *argv[]) {

    // Need to wait until the uORB and muORB are ready
    // Check for uORB initialization with get_instance
    while (uORB::Manager::get_instance() == nullptr) { usleep(100); }

    // Check for muORB initialization with get_uorb_communicator
    while (uORB::Manager::get_instance()->get_uorb_communicator() == nullptr) { usleep(100); }

    int parameter_server_set_used_h = orb_subscribe(ORB_ID(parameter_server_set_used));
    int parameter_server_set_value_h = orb_subscribe(ORB_ID(parameter_server_set_value));

    px4_pollfd_struct_t fds[2] = { { .fd = parameter_server_set_used_h, .events = POLLIN },
                                   { .fd = parameter_server_set_value_h, .events = POLLIN } };
    while (true) {
    	px4_poll(fds, 1, 1000);
    	if (fds[0].revents & POLLIN) {
            struct parameter_server_set_used_s msg;
    		orb_copy(ORB_ID(parameter_server_set_used), parameter_server_set_used_h, &msg);
    		PX4_INFO("Got parameter_server_set_used for %s", msg.parameter_name);
            (void) param_find(msg.parameter_name);
    	} else if (fds[1].revents & POLLIN) {
            struct parameter_server_set_value_s msg;
    		orb_copy(ORB_ID(parameter_server_set_value), parameter_server_set_value_h, &msg);
    		PX4_INFO("Got parameter_server_set_value for %s", msg.parameter_name);
            param_t param = param_find(msg.parameter_name);
            switch (param_type(param)) {
    		case PARAM_TYPE_INT32:
                param_set(param, (const void *) &msg.int_value);
    			break;

    		case PARAM_TYPE_FLOAT:
                param_set(param, (const void *) &msg.float_value);
    			break;

    		default:
    			PX4_ERR("Parameter must be either int or float");
                break;
            }
    	}
    }

    return 0;
}

void
param_server_init() {
    sync_thread_tid = px4_task_spawn_cmd(sync_thread_name,
				                         SCHED_DEFAULT,
				                         SCHED_PRIORITY_PARAMS,
				                         (1024 * 4),
				                         param_sync_thread,
				                         NULL);
}

void param_server_set(param_t param, const void *val) {
    PX4_INFO("Param changed in server");
    bool send_request = true;
    struct parameter_client_set_request_s req;
	req.timestamp = hrt_absolute_time();
	strncpy(req.parameter_name, param_name(param), 16);
    req.parameter_name[16] = 0;
	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		req.int_value = *(int32_t *)val;
		break;

	case PARAM_TYPE_FLOAT:
		req.float_value = *(float *)val;
		break;

	default:
		PX4_ERR("Parameter must be either int or float");
        send_request = false;
        break;
	}

	if (param_set_rsp_topic == PX4_ERROR) {
        PX4_INFO("Subscribing to parameter_client_set_response");
		param_set_rsp_topic = orb_subscribe(ORB_ID(parameter_client_set_response));
    	if (param_set_rsp_topic == PX4_ERROR) {
            PX4_INFO("Subscription to parameter_client_set_response failed");
    	} else {
            PX4_INFO("Subscription to parameter_client_set_response succeeded");
        }
	}

    if (send_request) {
        PX4_INFO("Sending param set request to client for %s", req.parameter_name);

    	if (param_set_req_topic == nullptr) {
    		param_set_req_topic = orb_advertise(ORB_ID(parameter_client_set_request), nullptr);
    	}

		orb_publish(ORB_ID(parameter_client_set_request), param_set_req_topic, &req);

        // Wait for response
        PX4_INFO("Waiting for parameter_client_set_response");
        usleep(100);
        bool updated = false;
        int count = 100;
        while (--count) {
            (void) orb_check(param_set_rsp_topic, &updated);
            if (updated) {
                PX4_INFO("Got parameter_client_set_response for %s", req.parameter_name);
                struct parameter_client_set_response_s rsp;
                orb_copy(ORB_ID(parameter_client_set_response), param_set_rsp_topic, &rsp);
                break;
        	}
            usleep(100);
        }
        if ( ! count) {
            PX4_ERR("Timeout waiting for parameter_client_set_response");
        }
    }
}
