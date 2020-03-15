/**
 * @file px4_pmen_app.cpp
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_vector_thrust_setpoint.h>


extern "C" __EXPORT int px4_pmen_app_main(int argc, char *argv[]);

int px4_pmen_app_main(int argc, char *argv[])
{
	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	int actuator_outputs_sub_fd = orb_subscribe(ORB_ID(actuator_outputs));
	int vector_thrust_sp_sub_fd = orb_subscribe(ORB_ID(vehicle_vector_thrust_setpoint));

	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);
	orb_set_interval(actuator_outputs_sub_fd, 200);
	orb_set_interval(vector_thrust_sp_sub_fd, 200);

	/* advertise attitude topic */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* advertise vehicle_vector_thrust_setpoint topic */
	struct vehicle_vector_thrust_setpoint_s vt_sp;
	memset(&vt_sp, 0, sizeof(vt_sp));
	orb_advert_t vt_sp_pub = orb_advertise(ORB_ID(vehicle_vector_thrust_setpoint), &vt_sp);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[3] = {
		{ .fd = actuator_outputs_sub_fd,   .events = POLLIN },
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = vector_thrust_sp_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 3, 5000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct actuator_outputs_s _actuator_outputs {};
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(actuator_outputs), actuator_outputs_sub_fd, &_actuator_outputs);
				PX4_INFO("Outputs: %8.4f | %8.4f | %8.4f | %8.4f\n",
					 (double)_actuator_outputs.output[0],
					 (double)_actuator_outputs.output[1],
					 (double)_actuator_outputs.output[2],
					 (double)_actuator_outputs.output[3]
					);
			}

			if (fds[2].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct vehicle_vector_thrust_setpoint_s  _vec_thr_sp_read;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_vector_thrust_setpoint), vector_thrust_sp_sub_fd, &_vec_thr_sp_read);
				PX4_INFO("VECTHRUST: %8.4f | %8.4f\n",
					 (double)_vec_thr_sp_read.thrust_n,
					 (double)_vec_thr_sp_read.thrust_e
					);
			}

			if (fds[1].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				PX4_INFO("Accelerometer: %8.4f | %8.4f | %8.4f\n\t\t\t      Gyro: %8.4f | %8.4f | %8.4f\n",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2],
					 (double)raw.gyro_rad[0],
					 (double)raw.gyro_rad[1],
					 (double)raw.gyro_rad[2]
					);

				att.q[0] = raw.accelerometer_m_s2[0];
				att.q[1] = raw.accelerometer_m_s2[1];
				att.q[2] = raw.accelerometer_m_s2[2];

				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);

				vt_sp.thrust_n = 0.6;
				vt_sp.thrust_e = 0.77;

				orb_publish(ORB_ID(vehicle_vector_thrust_setpoint), vt_sp_pub, &vt_sp);

			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}



	PX4_INFO("exiting");

	return 0;
};

