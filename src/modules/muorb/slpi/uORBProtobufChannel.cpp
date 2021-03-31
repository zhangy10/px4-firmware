/****************************************************************************
 *
 * Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
#include "uORBManager.hpp"
#include "uORBProtobufChannel.hpp"
#include <px4_platform_common/log.h>
#include <algorithm>
#include <string.h>
#include <drivers/drv_hrt.h>
#include "px4muorb.hpp"

fc_func_ptrs muorb_func_ptrs;

// static initialization.
uORB::ProtobufChannel uORB::ProtobufChannel::_Instance;
uORBCommunicator::IChannelRxHandler *uORB::ProtobufChannel::_RxHandler;
std::map<std::string, int> uORB::ProtobufChannel::_AppsSubscriberCache;

//==============================================================================
//==============================================================================
int16_t uORB::ProtobufChannel::topic_advertised(const char *messageName)
{
	PX4_INFO("Advertising %s on remote side", messageName);
	if (muorb_func_ptrs.advertise_func_ptr) {
        return muorb_func_ptrs.advertise_func_ptr(messageName);
    }

    PX4_ERR("advertise_func_ptr is null in %s", __FUNCTION__);
    return -1;
}

//==============================================================================
//==============================================================================
int16_t uORB::ProtobufChannel::add_subscription(const char *messageName, int32_t msgRateInHz)
{
	PX4_INFO("Subscribing to %s on remote side", messageName);
	if (muorb_func_ptrs.subscribe_func_ptr) {
        return muorb_func_ptrs.subscribe_func_ptr(messageName);
    }

    PX4_ERR("subscribe_func_ptr is null in %s", __FUNCTION__);
    return -1;
}

//==============================================================================
//==============================================================================
int16_t uORB::ProtobufChannel::remove_subscription(const char *messageName)
{
	PX4_INFO("Unsubscribing from %s on remote side", messageName);
	if (muorb_func_ptrs.unsubscribe_func_ptr) {
        return muorb_func_ptrs.unsubscribe_func_ptr(messageName);
    }

    PX4_ERR("unsubscribe_func_ptr is null in %s", __FUNCTION__);
    return -1;
}

//==============================================================================
//==============================================================================
int16_t uORB::ProtobufChannel::register_handler(uORBCommunicator::IChannelRxHandler *handler)
{
	_RxHandler = handler;
	return 0;
}

//==============================================================================
//==============================================================================

int16_t uORB::ProtobufChannel::send_message(const char *messageName, int32_t length, uint8_t *data)
{
    if (muorb_func_ptrs.topic_data_func_ptr) {
        std::string temp(messageName);
        if (_AppsSubscriberCache[temp]) {
    	   return muorb_func_ptrs.topic_data_func_ptr(messageName, data, length);
        }
        // If there are no remote subscribers then we do not need to send the
        // message over. That is still a success.
        return 0;
    }

    PX4_ERR("topic_data_func_ptr is null in %s", __FUNCTION__);
    return -1;
}

__BEGIN_DECLS
extern int dspal_main(int argc, char *argv[]);
__END_DECLS

int px4muorb_orb_initialize(fc_func_ptrs *func_ptrs)
{
	// HAP_power_request(100, 100, 1000);
	// shmem_info_p = NULL;

	// The uORB Manager needs to be initialized first up, otherwise the instance is nullptr.
	uORB::Manager::initialize();
	// Register the protobuf muorb with uORBManager.
	uORB::Manager::get_instance()->set_uorb_communicator(
		uORB::ProtobufChannel::GetInstance());

	// Now continue with the usual dspal startup.
	const char *argv[3] = { "dspal", "start" };
	int argc = 2;
	int rc;

    // Make sure that argv has a NULL pointer in the end.
    argv[argc] = NULL;

	rc = dspal_main(argc, (char **) argv);

    // Save off the function pointers needed to get access to
    // the SLPI protobuf functions.
    muorb_func_ptrs = *func_ptrs;

	return rc;
}

int px4muorb_topic_advertised(const char *topic_name)
{
	PX4_INFO("px4muorb_topic_advertised [%s] on remote side...", topic_name);

	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();
    if (channel) {
    	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();
    	if (rxHandler) {
    		return rxHandler->process_remote_topic(topic_name);
    	} else {
            PX4_ERR("Null rx handler in %s", __FUNCTION__);
    	}
    } else {
        PX4_ERR("Null channel pointer in %s", __FUNCTION__);
    }

	return -1;
}

int px4muorb_add_subscriber(const char *topic_name)
{
	PX4_INFO("px4muorb_add_subscriber [%s] on remote side...", topic_name);

	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();
    if (channel) {
    	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();
    	if (rxHandler) {
            channel->AddRemoteSubscriber(topic_name);
    		return rxHandler->process_add_subscription(topic_name);
    	} else {
            PX4_ERR("Null rx handler in %s", __FUNCTION__);
    	}
    } else {
        PX4_ERR("Null channel pointer in %s", __FUNCTION__);
    }

	return -1;
}

int px4muorb_remove_subscriber(const char *topic_name)
{
	PX4_INFO("px4muorb_remove_subscriber [%s] on remote side...", topic_name);

	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();
    if (channel) {
    	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();
    	if (rxHandler) {
            channel->RemoveRemoteSubscriber(topic_name);
    		return rxHandler->process_remove_subscription(topic_name);
    	} else {
            PX4_ERR("Null rx handler in %s", __FUNCTION__);
    	}
    } else {
        PX4_ERR("Null channel pointer in %s", __FUNCTION__);
    }

	return -1;
}

int px4muorb_send_topic_data(const char *topic_name, const uint8_t *data,
			                 int data_len_in_bytes)
{
	PX4_INFO("px4muorb_send_topic_data [%s] on remote side...", topic_name);

	uORB::ProtobufChannel *channel = uORB::ProtobufChannel::GetInstance();
    if (channel) {
    	uORBCommunicator::IChannelRxHandler *rxHandler = channel->GetRxHandler();
    	if (rxHandler) {
    		return rxHandler->process_received_message(topic_name,
                                                       data_len_in_bytes,
    				                                   (uint8_t *) data);
    	} else {
            PX4_ERR("Null rx handler in %s", __FUNCTION__);
    	}
    } else {
        PX4_ERR("Null channel pointer in %s", __FUNCTION__);
    }

	return -1;
}
