#include "ros3.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
	GROUND_N = 16,
	BALL_LAT = 8,
	BALL_LON = 12,
	GROUND_POINTS = GROUND_N * GROUND_N,
	BALL_POINTS = BALL_LAT * BALL_LON,
	POINT_COUNT = GROUND_POINTS + BALL_POINTS,
	TF_EDGE_COUNT = 2
};

static void build_topic(char *out, size_t out_size, const char *name, const char *suffix) {
	snprintf(out, out_size, "/sim/%s/%s", name, suffix);
}

static const char *parse_name(int argc, char **argv) {
	for(int i = 1; i < argc; ++i) {
		if(argv[i] && argv[i][0] && strcmp(argv[i], "--paused") != 0) {
			return argv[i];
		}
	}
	return "falling_ball";
}

static int parse_start_paused(int argc, char **argv) {
	for(int i = 1; i < argc; ++i) {
		if(argv[i] && strcmp(argv[i], "--paused") == 0) {
			return 1;
		}
	}
	return 0;
}

static void fill_ground_points(f32 *dst) {
	int idx = 0;
	for(int gy = 0; gy < GROUND_N; ++gy) {
		for(int gx = 0; gx < GROUND_N; ++gx) {
			f32 x = ((f32) gx / (f32) (GROUND_N - 1) - 0.5f) * 1.8f;
			f32 y = ((f32) gy / (f32) (GROUND_N - 1) - 0.5f) * 1.8f;
			dst[idx++] = x;
			dst[idx++] = y;
			dst[idx++] = 0.0f;
		}
	}
}

static void fill_ball_offsets(f32 *dst) {
	int idx = 0;
	const f32 radius = 0.12f;
	for(int lat = 0; lat < BALL_LAT; ++lat) {
		f32 v = ((f32) lat + 0.5f) / (f32) BALL_LAT;
		f32 phi = (v - 0.5f) * 3.1415926535f;
		f32 cp = cosf(phi);
		f32 sp = sinf(phi);
		for(int lon = 0; lon < BALL_LON; ++lon) {
			f32 u = (f32) lon / (f32) BALL_LON;
			f32 theta = u * 6.283185307f;
			f32 ct = cosf(theta);
			f32 st = sinf(theta);
			dst[idx++] = radius * cp * ct;
			dst[idx++] = radius * cp * st;
			dst[idx++] = radius * sp;
		}
	}
}

static void fill_points(f32 *points, const f32 *ball_offsets, f32 ball_z) {
	int base = GROUND_POINTS * 3;
	fill_ground_points(points);
	for(int i = 0; i < BALL_POINTS; ++i) {
		points[base + i * 3 + 0] = ball_offsets[i * 3 + 0];
		points[base + i * 3 + 1] = ball_offsets[i * 3 + 1];
		points[base + i * 3 + 2] = ball_offsets[i * 3 + 2] + ball_z;
	}
}

static void fill_tf(f32 *tf, f32 ball_z) {
	tf[0] = -1.0f;
	tf[1] = 0.0f;
	tf[2] = 0.0f;
	tf[3] = 0.0f;
	tf[4] = 0.0f;

	tf[5] = 0.0f;
	tf[6] = 1.0f;
	tf[7] = 0.0f;
	tf[8] = 0.0f;
	tf[9] = ball_z;
}

static void fill_status(f32 *status, f32 time_s, f32 ball_z, f32 vel_z, int running) {
	status[0] = time_s;
	status[1] = ball_z;
	status[2] = vel_z;
	status[3] = running ? 1.0f : 0.0f;
}

static void apply_command(mirage_msg *msg, int *running, int *step_once, f32 *ball_z, f32 *vel_z) {
	char fn_name[32];
	i64 fn_len = 0;
	i64 fn_argc = 0;

	if(!msg || !running || !step_once || !ball_z || !vel_z) {
		return;
	}
	msg->cursor = 0;
	if(mirage_read_start(msg) < 0) {
		return;
	}
	if(mirage_read_fn(msg, fn_name, &fn_len, (i64) sizeof(fn_name) - 1, &fn_argc) < 0) {
		return;
	}
	fn_name[fn_len] = '\0';
	if(strcmp(fn_name, "play") == 0) {
		*running = 1;
		return;
	}
	if(strcmp(fn_name, "pause") == 0) {
		*running = 0;
		return;
	}
	if(strcmp(fn_name, "reset") == 0) {
		*ball_z = 1.4f;
		*vel_z = 0.0f;
		*running = 1;
		*step_once = 0;
		return;
	}
	if(strcmp(fn_name, "step") == 0) {
		*step_once = 1;
	}
}

int main(int argc, char **argv) {
	const char *sim_name = parse_name(argc, argv);
	int running = parse_start_paused(argc, argv) ? 0 : 1;
	int step_once = 0;
	const f32 dt = 1.0f / 60.0f;
	const f32 gravity = -9.81f;
	const f32 bounce = 0.65f;
	const f32 radius = 0.12f;
	f32 time_s = 0.0f;
	f32 ball_z = 1.4f;
	f32 vel_z = 0.0f;
	char points_topic[256];
	char tf_topic[256];
	char status_topic[256];
	char cmd_topic[256];
	f32 points[POINT_COUNT * 3];
	f32 ball_offsets[BALL_POINTS * 3];
	f32 tf[TF_EDGE_COUNT * 5];
	f32 status[4];
	i64 point_dims[2] = {POINT_COUNT, 3};
	i64 tf_dims[2] = {TF_EDGE_COUNT, 5};
	i64 status_dims[2] = {1, 4};
	rose_node *node;
	rose_publisher *points_pub;
	rose_publisher *tf_pub;
	rose_publisher *status_pub;
	rose_subscriber *cmd_sub = NULL;
	mirage_msg *points_msg;
	mirage_msg *tf_msg;
	mirage_msg *status_msg;
	mirage_msg *cmd_msg;

	build_topic(points_topic, sizeof(points_topic), sim_name, "points");
	build_topic(tf_topic, sizeof(tf_topic), sim_name, "tf");
	build_topic(status_topic, sizeof(status_topic), sim_name, "status");
	build_topic(cmd_topic, sizeof(cmd_topic), sim_name, "cmd");

	node = rose_init((char) argc, argv, "roseviz_falling_ball", NULL, NULL);
	if(!node) {
		fprintf(stderr, "rose_init failed: %s\n", rose_last_error());
		return 1;
	}

	points_pub = rose_create_pub(node, points_topic, -1, 32768, 64);
	tf_pub = rose_create_pub(node, tf_topic, -1, 4096, 64);
	status_pub = rose_create_pub(node, status_topic, -1, 4096, 64);
	if(!points_pub || !tf_pub || !status_pub) {
		fprintf(stderr, "rose topic setup failed: %s\n", rose_last_error());
		rose_shutdown(node);
		return 1;
	}

	points_msg = mirage_create(32768, NULL);
	tf_msg = mirage_create(4096, NULL);
	status_msg = mirage_create(4096, NULL);
	cmd_msg = mirage_create(256, NULL);
	if(!points_msg || !tf_msg || !status_msg || !cmd_msg) {
		fprintf(stderr, "mirage_create failed\n");
		mirage_destroy(&points_msg, NULL);
		mirage_destroy(&tf_msg, NULL);
		mirage_destroy(&status_msg, NULL);
		mirage_destroy(&cmd_msg, NULL);
		rose_shutdown(node);
		return 1;
	}

	fill_ball_offsets(ball_offsets);

	while(rose_ok(node)) {
		if(!cmd_sub) {
			cmd_sub = rose_create_sub(node, cmd_topic, -1, 1, NULL);
		}
		if(cmd_sub) {
			i64 read_rc = rose_readlast(cmd_sub, cmd_msg);
			if(read_rc >= 0) {
				apply_command(cmd_msg, &running, &step_once, &ball_z, &vel_z);
			}
		}

		if(running || step_once) {
			vel_z += gravity * dt;
			ball_z += vel_z * dt;
			if(ball_z < radius) {
				ball_z = radius;
				vel_z = -vel_z * bounce;
				if(fabsf(vel_z) < 0.05f) {
					vel_z = 0.0f;
				}
			}
			time_s += dt;
			step_once = 0;
		}

		fill_points(points, ball_offsets, ball_z);
		fill_tf(tf, ball_z);
		fill_status(status, time_s, ball_z, vel_z, running);

		if(mirage_write_start(points_msg) < 0
		   || mirage_write_numericarray(points_msg, MIRAGE_ARRAY_F32, 2, point_dims, points) < 0
		   || mirage_write_start(tf_msg) < 0
		   || mirage_write_numericarray(tf_msg, MIRAGE_ARRAY_F32, 2, tf_dims, tf) < 0
		   || mirage_write_start(status_msg) < 0
		   || mirage_write_numericarray(status_msg, MIRAGE_ARRAY_F32, 2, status_dims, status) < 0) {
			fprintf(stderr, "mirage encode failed\n");
			break;
		}

		if(rose_publish(points_pub, points_msg) < 0
		   || rose_publish(tf_pub, tf_msg) < 0
		   || rose_publish(status_pub, status_msg) < 0) {
			fprintf(stderr, "rose_publish failed: %s\n", rose_last_error());
			break;
		}

		rose_sleep(dt);
	}

	mirage_destroy(&points_msg, NULL);
	mirage_destroy(&tf_msg, NULL);
	mirage_destroy(&status_msg, NULL);
	mirage_destroy(&cmd_msg, NULL);
	rose_shutdown(node);
	return 0;
}
