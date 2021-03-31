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

#include "uORBAppsProtobufChannel.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include <cstdio>
#include <pthread.h>
#include <string.h>

#include "fc_sensor.h"

// TODO: Get rid of all the shmem stuff!
unsigned char *adsp_changed_index = nullptr;

uORB::AppsProtobufChannel *uORB::AppsProtobufChannel::_InstancePtr = nullptr;
uORBCommunicator::IChannelRxHandler *uORB::AppsProtobufChannel::_RxHandler = nullptr;
std::map<std::string, int> uORB::AppsProtobufChannel::_SlpiSubscriberCache;

const uint32_t uORB::AppsProtobufChannel::_TOPIC_DATA_BUFFER_LENGTH;

char uORB::AppsProtobufChannel::_topic_name_buffer[_TOPIC_DATA_BUFFER_LENGTH];
uint8_t uORB::AppsProtobufChannel::_topic_data_buffer[_TOPIC_DATA_BUFFER_LENGTH];

pthread_mutex_t uORB::AppsProtobufChannel::_mutex = PTHREAD_MUTEX_INITIALIZER;

void uORB::AppsProtobufChannel::ReceiveCallback(const char *topic,
                                                const uint8_t *data,
                                                uint32_t length_in_bytes) {
    PX4_INFO("Got received data callback for topic %s", topic);

    if (length_in_bytes < _TOPIC_DATA_BUFFER_LENGTH) {
        if (_RxHandler) {
            pthread_mutex_lock(&_mutex);
            strncpy(_topic_name_buffer, topic, _TOPIC_DATA_BUFFER_LENGTH);
            memcpy(_topic_data_buffer, data, length_in_bytes);
            _RxHandler->process_received_message(_topic_name_buffer, length_in_bytes, _topic_data_buffer);
            pthread_mutex_unlock(&_mutex);
        } else {
            PX4_ERR("uORB pointer is null in %s", __FUNCTION__);
        }
    } else {
        PX4_ERR("ReceiveCallback topic data too long %d", length_in_bytes);
    }
}

void uORB::AppsProtobufChannel::AdvertiseCallback(const char *topic) {
    PX4_INFO("Got advertisement callback for topic %s", topic);

    if (_RxHandler) {
        pthread_mutex_lock(&_mutex);
        strncpy(_topic_name_buffer, topic, _TOPIC_DATA_BUFFER_LENGTH);
        _RxHandler->process_remote_topic(_topic_name_buffer);
        pthread_mutex_unlock(&_mutex);
    } else {
        PX4_ERR("uORB pointer is null in %s", __FUNCTION__);
    }
}

void uORB::AppsProtobufChannel::SubscribeCallback(const char *topic) {
    PX4_INFO("Got subscription callback for topic %s", topic);

    pthread_mutex_lock(&_mutex);
    _SlpiSubscriberCache[topic]++;
    pthread_mutex_unlock(&_mutex);

    if (_RxHandler) {
        pthread_mutex_lock(&_mutex);
        strncpy(_topic_name_buffer, topic, _TOPIC_DATA_BUFFER_LENGTH);
        _RxHandler->process_add_subscription(_topic_name_buffer);
        pthread_mutex_unlock(&_mutex);
    } else {
        // This can happen on startup if the remote entity is up and
        // running before this side has completed initialization. It is
        // okay because we have noted the event in the subscriber cache.
        PX4_WARN("uORB pointer is null in %s", __FUNCTION__);
    }
}

void uORB::AppsProtobufChannel::UnsubscribeCallback(const char *topic) {
    PX4_INFO("Got remove subscription callback for topic %s", topic);

    if (_RxHandler) {
        pthread_mutex_lock(&_mutex);
        if (_SlpiSubscriberCache[topic]) _SlpiSubscriberCache[topic]--;
        strncpy(_topic_name_buffer, topic, _TOPIC_DATA_BUFFER_LENGTH);
        _RxHandler->process_remove_subscription(_topic_name_buffer);
        pthread_mutex_unlock(&_mutex);
    } else {
        PX4_ERR("uORB pointer is null in %s", __FUNCTION__);
    }
}

bool uORB::AppsProtobufChannel::Initialize(bool enable_debug) {
    if (_Initialized == false) {
        fc_callbacks cb = {&ReceiveCallback, &AdvertiseCallback,
                           &SubscribeCallback, &UnsubscribeCallback};
        if (fc_sensor_initialize(enable_debug, &cb) != 0) {
        	PX4_ERR("Error calling the muorb protobuf initalize method..");
        } else {
            _Initialized = true;
        }
    }
    return _Initialized;
}

int16_t uORB::AppsProtobufChannel::topic_advertised(const char *messageName)
{
    if (_Initialized) return fc_sensor_advertise(messageName);
    else return -1;
}

int16_t uORB::AppsProtobufChannel::add_subscription(const char *messageName, int msgRateInHz)
{
    (void)(msgRateInHz);
    if (_Initialized) return fc_sensor_subscribe(messageName);
    else return -1;
}

int16_t uORB::AppsProtobufChannel::remove_subscription(const char *messageName)
{
    if (_Initialized) return fc_sensor_unsubscribe(messageName);
    else return -1;
}

int16_t uORB::AppsProtobufChannel::register_handler(uORBCommunicator::IChannelRxHandler *handler)
{
	_RxHandler = handler;
	return 0;
}

int16_t uORB::AppsProtobufChannel::send_message(const char *messageName, int length, uint8_t *data)
{
    bool enable_debug = false;
    if (strcmp(messageName, "qshell_req") == 0) enable_debug = true;
    if (strcmp(messageName, "qshell_retval") == 0) enable_debug = true;

    if (_Initialized) {
        if (_SlpiSubscriberCache[messageName]) {
            if (enable_debug) PX4_INFO("Sending data in %s", __FUNCTION__);
            return fc_sensor_send_data(messageName, data, length);
        } else {
            // There are no remote subscribers so no need to actually send
            // the data. If a subscription comes in later, the data will
            // be re-sent to them at that time.
            if (enable_debug) PX4_INFO("No subscribers (yet) in %s for topic %s", __FUNCTION__, messageName);
            return 0;
        }
    }

    if (enable_debug) PX4_ERR("AppsProtobufChannel not yet initialized in %s", __FUNCTION__);
    return -1;
}
