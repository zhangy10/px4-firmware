
#include <string.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

#include "param_client.h"
#include "uORB/uORB.h"

// uORB topics needed to keep parameter server and client in sync
#include <uORB/topics/parameter_client_reset_request.h>
#include <uORB/topics/parameter_client_reset_response.h>
#include <uORB/topics/parameter_client_set_request.h>
#include <uORB/topics/parameter_client_set_response.h>
#include <uORB/topics/parameter_server_set_used.h>
#include <uORB/topics/parameter_server_set_value.h>

static px4_task_t   sync_thread_tid;
static const char  *sync_thread_name = "client_sync_thread";
static orb_advert_t parameter_server_set_used_h = nullptr;
static orb_advert_t parameter_server_set_value_h = nullptr;

static int param_sync_thread(int argc, char *argv[]) {

    // This thread gets started by the client side during PX4 initialization.
    // We cannot send out the subscribe request immediately because the server
    // side will not be ready to receive it on the muorb yet and it will get dropped.
    // So, sleep a little bit to give server side a chance to finish initialization
    // of the muorb. But don't wait too long otherwise a set request from the server
    // side could be missed.
    usleep(500);

    int param_set_req_fd   = orb_subscribe(ORB_ID(parameter_client_set_request));
    int param_reset_req_fd = orb_subscribe(ORB_ID(parameter_client_reset_request));

    orb_advert_t param_set_rsp_fd   = nullptr;
    orb_advert_t param_reset_rsp_fd = nullptr;

	struct parameter_client_set_request_s    s_req;
	struct parameter_client_set_response_s   s_rsp;
	struct parameter_client_reset_request_s  r_req;
	struct parameter_client_reset_response_s r_rsp;

    bool updated = false;
    while (true) {
        usleep(100);
        (void) orb_check(param_set_req_fd, &updated);
        if (updated) {
            orb_copy(ORB_ID(parameter_client_set_request), param_set_req_fd, &s_req);
    		PX4_INFO("Got parameter_client_set_request for %s", s_req.parameter_name);
            // This will find the parameter and also set its used flag
            param_t param = param_find(s_req.parameter_name);
            switch (param_type(param)) {
    		case PARAM_TYPE_INT32:
                param_set_no_notification(param, (const void *) &s_req.int_value);
    			break;

    		case PARAM_TYPE_FLOAT:
                param_set_no_notification(param, (const void *) &s_req.float_value);
    			break;

    		default:
    			PX4_ERR("Parameter must be either int or float");
                break;
            }

            s_rsp.timestamp = hrt_absolute_time();
            if (param_set_rsp_fd == nullptr) {
                param_set_rsp_fd = orb_advertise(ORB_ID(parameter_client_set_response), &s_rsp);
            } else {
                orb_publish(ORB_ID(parameter_client_set_response), param_set_rsp_fd, &s_rsp);
            }
    	}
        (void) orb_check(param_reset_req_fd, &updated);
        if (updated) {
            orb_copy(ORB_ID(parameter_client_reset_request), param_reset_req_fd, &r_req);
    		PX4_INFO("Got parameter_client_reset_request");
            if (r_req.reset_all) {
                param_reset_all();
            } else {
                param_t param = param_find_no_notification(r_req.parameter_name);
                param_reset_no_notification(param);
            }

            r_rsp.timestamp = hrt_absolute_time();
            if (param_reset_rsp_fd == nullptr) {
                param_reset_rsp_fd = orb_advertise(ORB_ID(parameter_client_reset_response), &r_rsp);
            } else {
                orb_publish(ORB_ID(parameter_client_reset_response), param_reset_rsp_fd, &r_rsp);
            }
    	}
    }

    return 0;
}

void
param_client_init()
{
    sync_thread_tid = px4_task_spawn_cmd(sync_thread_name,
				                         SCHED_DEFAULT,
				                         SCHED_PRIORITY_PARAMS,
				                         (1024 * 4),
				                         param_sync_thread,
				                         NULL);
}

void param_client_set(param_t param, const void *val) {

    PX4_INFO("Param changed in client");

    // If this is the parameter client, make sure that the server is updated
    bool send_event = false;
    struct parameter_server_set_value_s event = {};
	event.timestamp = hrt_absolute_time();
	strncpy(event.parameter_name, param_name(param), 16);
    event.parameter_name[16] = 0;
	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		event.int_value = *(int32_t *)val;
		break;

	case PARAM_TYPE_FLOAT:
		event.float_value = *(float *)val;
		break;

	default:
		PX4_ERR("Parameter must be either int or float");
        send_event = false;
        break;
	}

    if (send_event) {
    	/*
    	 * If we don't have a handle to our topic, create one now; otherwise
    	 * just publish.
    	 */
    	if (parameter_server_set_value_h == nullptr) {
    		parameter_server_set_value_h = orb_advertise(ORB_ID(parameter_server_set_value), &event);

    	} else {
    		orb_publish(ORB_ID(parameter_server_set_value), parameter_server_set_value_h, &event);
    	}
    }
}

void
param_client_set_used(param_t param) {
    // Notify the parameter server that this parameter has been marked as used
    struct parameter_server_set_used_s event = {};
	event.timestamp = hrt_absolute_time();
	strncpy(event.parameter_name, param_name(param), 16);
    event.parameter_name[16] = 0;
	if (parameter_server_set_used_h == nullptr) {
		parameter_server_set_used_h = orb_advertise(ORB_ID(parameter_server_set_used), &event);

	} else {
		orb_publish(ORB_ID(parameter_server_set_used), parameter_server_set_used_h, &event);
	}
}
