/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/**
 * @file Imu.hpp
 *
 * Defines basic functionality of Cyphal Imu publisher
 *
 * @author
 */

#pragma once

// UDRAL Specification Messages
#include <uORB/topics/vehicle_attitude.h>
#include <uavcan/si/unit/angle/Quaternion_1_0.h>
#include "../Publisher.hpp"

class UavcanImuPublisher : public UavcanPublisher
{
public:
	UavcanImuPublisher(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanPublisher(handle, pmgr, "udral.", "imu", instance)
	{

	};

	~UavcanImuPublisher() override = default;

	// Update the uORB Subscription and broadcast a UAVCAN message
	virtual void update() override
	{
		// if (_imu_sub.updated() && _port_id != CANARD_PORT_ID_UNSET) {
			vehicle_attitude_s imu {};
			_imu_sub.update(&imu);
			size_t payload_size = uavcan_si_unit_angle_Quaternion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

			uavcan_si_unit_angle_Quaternion_1_0 qua {};
			qua.wxyz[0] = imu.q[0];
			qua.wxyz[1] = imu.q[1];
			qua.wxyz[2] = imu.q[2];
			qua.wxyz[3] = imu.q[3];

			uint8_t qua_payload_buffer[uavcan_si_unit_angle_Quaternion_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

			const CanardTransferMetadata transfer_metadata = {
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = _port_id, // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = _transfer_id,
			};

			int32_t result = uavcan_si_unit_angle_Quaternion_1_0_serialize_(&qua, qua_payload_buffer,
					 &payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							       &transfer_metadata,
							       payload_size,
							       &qua_payload_buffer);
			}
		// }
	};

private:

	/// TODO: Allow >1 instance
	uORB::Subscription _imu_sub{ORB_ID(vehicle_attitude)};
	CanardTransferID _transfer_id;
};
