#include "ros3.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#ifdef __linux__
#include <sys/prctl.h>
#endif

enum {
	ROSEVIZ_TOPIC_SLOT_COUNT = 32,
	ROSEVIZ_TOPIC_SLOT_BYTES = 4 * 1024 * 1024,
	ROSEVIZ_TOPIC_NAME_BYTES = 256,
	ROSEVIZ_TOPIC_CATALOG_MAX = 256,
	ROSEVIZ_DISCOVERY_PERIOD_NS = 3000000000ULL,
	ROSEVIZ_CONNECT_RETRY_NS = 250000000ULL
};

typedef struct {
	char name[ROSEVIZ_TOPIC_NAME_BYTES];
	rose_subscriber *sub;
	uint64_t seq;
	uint64_t last_retry_ns;
	uint64_t last_ok_ns;
	int payload_size;
	int last_error;
	int active;
} roseviz_topic_slot;

static rose_node *g_node = NULL;
static quicksand_connection *g_dump_conn = NULL;
static uint64_t g_last_discovery_ns = 0;
static int g_catalog_count = 0;
static char g_catalog[ROSEVIZ_TOPIC_CATALOG_MAX][ROSEVIZ_TOPIC_NAME_BYTES];
static rose_publisher *g_cmd_pub = NULL;
static char g_cmd_pub_topic[ROSEVIZ_TOPIC_NAME_BYTES];
static roseviz_topic_slot g_slots[ROSEVIZ_TOPIC_SLOT_COUNT];
static uint8_t g_slot_store[ROSEVIZ_TOPIC_SLOT_COUNT][ROSEVIZ_TOPIC_SLOT_BYTES];

typedef struct {
	mirage_array_type type;
	int rank;
	int64_t dims[4];
	const uint8_t *data;
	int64_t data_bytes;
} roseviz_numericarray_view;

static int roseviz_copy_cstr(char *dst, int dst_size, const char *src) {
	int i = 0;
	if(!dst || dst_size <= 0) {
		return 0;
	}
	if(!src) {
		dst[0] = '\0';
		return 0;
	}
	while(src[i] && i < dst_size - 1) {
		dst[i] = src[i];
		i++;
	}
	dst[i] = '\0';
	return i;
}

static void roseviz_topic_to_display(char *dst, int dst_size, const char *src) {
	int i = 0;
	if(!dst || dst_size <= 0) {
		return;
	}
	if(!src) {
		dst[0] = '\0';
		return;
	}
	while(src[i] && i < dst_size - 1) {
		dst[i] = (src[i] == '%') ? '/' : src[i];
		i++;
	}
	dst[i] = '\0';
}

static void roseviz_build_query_topic(char *dst, int dst_size, const char *suffix) {
	char topic[256];
	snprintf(topic, sizeof(topic), "rosecore_%s", suffix);
	rose_topic_to_shm(dst, (size_t) dst_size, topic);
}

static void roseviz_skip_to_latest(quicksand_connection *conn) {
	uint8_t buf[4096];
	i64 sz = (i64) sizeof(buf);
	if(conn) {
		quicksand_read_latest(conn, buf, &sz);
	}
}

static void roseviz_send_dump_request(quicksand_connection *conn) {
	mirage_msg *msg = mirage_create(256, NULL);
	if(!conn || !msg) {
		mirage_destroy(&msg, NULL);
		return;
	}

	if(mirage_write_start(msg) >= 0
	   && mirage_write_fn(msg, ROSE_MSG_DUMP_ALL, (i64) strlen(ROSE_MSG_DUMP_ALL), 0) >= 0) {
		quicksand_write(conn, msg->data, msg->cursor);
	}

	mirage_destroy(&msg, NULL);
}

static int roseviz_parse_node_info(mirage_msg *msg,
				   char *pubs, int pubs_size,
				   char *subs, int subs_size) {
	char fn_name[64];
	i64 fn_len = 0;
	i64 fn_argc = 0;
	i64 str_len = 0;
	char scratch[256];
	i64 ignore_i64 = 0;

	msg->cursor = 0;
	if(mirage_read_start(msg) < 0) {
		return -1;
	}
	if(mirage_read_fn(msg, fn_name, &fn_len, (i64) sizeof(fn_name) - 1, &fn_argc) < 0) {
		return -1;
	}
	fn_name[fn_len] = '\0';
	if(strcmp(fn_name, ROSE_MSG_DONE) == 0) {
		return 0;
	}
	if(strcmp(fn_name, ROSE_MSG_NODE_INFO) != 0) {
		return -1;
	}
	if(mirage_read_string(msg, scratch, &str_len, (i64) sizeof(scratch) - 1) < 0) {
		return -1;
	}
	if(mirage_read_i64(msg, &ignore_i64) < 0) {
		return -1;
	}
	if(mirage_read_i64(msg, &ignore_i64) < 0) {
		return -1;
	}
	if(mirage_read_string(msg, pubs, &str_len, (i64) pubs_size - 1) < 0) {
		return -1;
	}
	pubs[str_len] = '\0';
	if(mirage_read_string(msg, subs, &str_len, (i64) subs_size - 1) < 0) {
		return -1;
	}
	subs[str_len] = '\0';
	return 1;
}

static void roseviz_catalog_add(char catalog[][ROSEVIZ_TOPIC_NAME_BYTES], int *count, const char *topic) {
	char display[ROSEVIZ_TOPIC_NAME_BYTES];
	int i;

	if(!count || !topic || !topic[0]) {
		return;
	}
	roseviz_topic_to_display(display, (int) sizeof(display), topic);
	for(i = 0; i < *count; ++i) {
		if(strcmp(catalog[i], display) == 0) {
			return;
		}
	}
	if(*count >= ROSEVIZ_TOPIC_CATALOG_MAX) {
		return;
	}
	roseviz_copy_cstr(catalog[*count], (int) sizeof(catalog[*count]), display);
	*count += 1;
}

static void roseviz_catalog_add_csv(char catalog[][ROSEVIZ_TOPIC_NAME_BYTES], int *count, const char *csv) {
	char buf[ROSEVIZ_TOPIC_NAME_BYTES];
	int i = 0;

	if(!count || !csv) {
		return;
	}
	while(*csv) {
		if(*csv == ',') {
			csv++;
			continue;
		}
		i = 0;
		while(*csv && *csv != ',' && i < (int) sizeof(buf) - 1) {
			buf[i++] = *csv++;
		}
		buf[i] = '\0';
		if(i > 0) {
			roseviz_catalog_add(catalog, count, buf);
		}
	}
}

static int roseviz_catalog_cmp(const void *lhs, const void *rhs) {
	return strcmp((const char *) lhs, (const char *) rhs);
}

static void roseviz_try_discover_topics(void) {
	char topic[512];
	char pubs[16384];
	char subs[16384];
	char next_catalog[ROSEVIZ_TOPIC_CATALOG_MAX][ROSEVIZ_TOPIC_NAME_BYTES];
	int next_count = 0;
	int waited_ms = 0;
	const int timeout_ms = 250;
	const int poll_ms = 10;
	int got_any = 0;
	mirage_msg *msg = NULL;

	if(!g_node) {
		return;
	}
	if(!g_dump_conn) {
		roseviz_build_query_topic(topic, (int) sizeof(topic), "dump");
		if(quicksand_connect(&g_dump_conn, topic, -1, ROSE_QUERY_MSG_SIZE,
				     ROSE_DUMP_RING_SIZE, NULL) != 0) {
			g_dump_conn = NULL;
			return;
		}
	}

	msg = mirage_create(ROSE_QUERY_MSG_SIZE, NULL);
	if(!msg) {
		return;
	}

	roseviz_skip_to_latest(g_dump_conn);
	roseviz_send_dump_request(g_dump_conn);

	while(waited_ms < timeout_ms) {
		i64 sz = msg->max_length;
		int parse_rc;
		i64 read_rc = quicksand_read(g_dump_conn, msg->data, &sz);
		if(read_rc >= 0) {
			msg->length = sz;
			parse_rc = roseviz_parse_node_info(msg, pubs, (int) sizeof(pubs), subs, (int) sizeof(subs));
			if(parse_rc == 0) {
				got_any = 1;
				break;
			}
			if(parse_rc == 1) {
				got_any = 1;
				roseviz_catalog_add_csv(next_catalog, &next_count, pubs);
				roseviz_catalog_add_csv(next_catalog, &next_count, subs);
			}
			waited_ms = 0;
		} else {
			quicksand_sleep((u64) poll_ms * 1000000ULL);
			waited_ms += poll_ms;
		}
	}

	if(got_any) {
		qsort(next_catalog, (size_t) next_count, sizeof(next_catalog[0]), roseviz_catalog_cmp);
		memcpy(g_catalog, next_catalog, (size_t) next_count * sizeof(next_catalog[0]));
		g_catalog_count = next_count;
	}

	mirage_destroy(&msg, NULL);
}

static int roseviz_find_slot(const char *topic) {
	int i;
	if(!topic || !topic[0]) {
		return -1;
	}
	for(i = 0; i < ROSEVIZ_TOPIC_SLOT_COUNT; ++i) {
		if(g_slots[i].active && strcmp(g_slots[i].name, topic) == 0) {
			return i;
		}
	}
	return -1;
}

static int roseviz_try_read_top_numericarray(int slot, roseviz_numericarray_view *view) {
	mirage_msg msg;
	mirage_array_type type;
	mirage_element_type elem_type;
	i64 rank = 0;
	i64 dims[4] = {0, 0, 0, 0};
	i64 rc;
	i64 data_bytes = 1;
	i64 start;
	int i;

	if(!view || slot < 0 || slot >= ROSEVIZ_TOPIC_SLOT_COUNT) {
		return -1;
	}
	if(!g_slots[slot].active || g_slots[slot].payload_size <= 0) {
		return -1;
	}

	mirage_init(&msg, g_slot_store[slot], g_slots[slot].payload_size);
	msg.length = g_slots[slot].payload_size;
	if(mirage_read_start(&msg) < 0) {
		return -1;
	}
	if(mirage_peek_element_type(&msg, &elem_type) < 0 || elem_type != MIRAGE_NUMERICARRAY) {
		return -1;
	}

	start = msg.cursor;
	rc = mirage_read_numericarray(&msg, &type, &rank, dims, 4, NULL, MIRAGE_SKIP_DATA);
	if(rc < 0 || rank < 0 || rank > 4) {
		return -1;
	}
	for(i = 0; i < rank; ++i) {
		if(dims[i] <= 0) {
			return -1;
		}
		data_bytes *= dims[i];
	}
	data_bytes *= (i64) mirage_array_type_size[type];
	if(data_bytes <= 0 || data_bytes > g_slots[slot].payload_size) {
		return -1;
	}

	view->type = type;
	view->rank = (int) rank;
	for(i = 0; i < 4; ++i) {
		view->dims[i] = dims[i];
	}
	view->data_bytes = data_bytes;
	view->data = msg.data + start + rc - data_bytes;
	return 0;
}

int roseviz_bridge_init(void) {
	static char program_name[] = "roseviz";
	static char *argv[] = {program_name, NULL};

	if(g_node) {
		return 1;
	}

	memset(g_slots, 0, sizeof(g_slots));
	g_catalog_count = 0;
	g_last_discovery_ns = 0;
	g_dump_conn = NULL;
	g_cmd_pub = NULL;
	g_cmd_pub_topic[0] = '\0';

	g_node = rose_init(1, argv, program_name, NULL, NULL);
	return g_node != NULL;
}

void roseviz_bridge_shutdown(void) {
	if(g_dump_conn) {
		quicksand_disconnect(&g_dump_conn, NULL);
		g_dump_conn = NULL;
	}
	if(g_node) {
		rose_shutdown(g_node);
		g_node = NULL;
	}
	memset(g_slots, 0, sizeof(g_slots));
	g_catalog_count = 0;
	g_last_discovery_ns = 0;
	g_cmd_pub = NULL;
	g_cmd_pub_topic[0] = '\0';
}

void roseviz_bridge_poll(void) {
	uint64_t now;
	int i;

	if(!g_node) {
		return;
	}
	if(!rose_ok(g_node)) {
		return;
	}

	now = rose_now();
	if(g_catalog_count == 0 || now - g_last_discovery_ns >= ROSEVIZ_DISCOVERY_PERIOD_NS) {
		roseviz_try_discover_topics();
		g_last_discovery_ns = now;
	}

	for(i = 0; i < ROSEVIZ_TOPIC_SLOT_COUNT; ++i) {
		mirage_msg msg;
		i64 read_rc;
		roseviz_topic_slot *slot = &g_slots[i];

		if(!slot->active) {
			continue;
		}
		if(!slot->sub) {
			if(now - slot->last_retry_ns < ROSEVIZ_CONNECT_RETRY_NS) {
				continue;
			}
			slot->last_retry_ns = now;
			slot->sub = rose_create_sub(g_node, slot->name, -1, 1, NULL);
			if(!slot->sub) {
				slot->last_error = (int) rose_last_error_code();
			}
			continue;
		}

		mirage_init(&msg, g_slot_store[i], ROSEVIZ_TOPIC_SLOT_BYTES);
		msg.length = ROSEVIZ_TOPIC_SLOT_BYTES;
		read_rc = rose_readlast(slot->sub, &msg);
		if(read_rc >= 0) {
			slot->payload_size = (int) msg.length;
			slot->seq += 1;
			slot->last_ok_ns = now;
			slot->last_error = 0;
		} else if(read_rc < 0 && read_rc != ROSE_NO_DATA) {
			slot->last_error = (int) read_rc;
		}
	}
}

int roseviz_bridge_track_topic(uint8_t *topic_bytes) {
	int i;
	const char *topic = (const char *) topic_bytes;
	int existing = roseviz_find_slot(topic);
	if(existing >= 0) {
		return existing;
	}
	if(!topic || !topic[0]) {
		return -1;
	}
	for(i = 0; i < ROSEVIZ_TOPIC_SLOT_COUNT; ++i) {
		if(!g_slots[i].active) {
			memset(&g_slots[i], 0, sizeof(g_slots[i]));
			roseviz_copy_cstr(g_slots[i].name, (int) sizeof(g_slots[i].name), topic);
			g_slots[i].active = 1;
			g_slots[i].last_error = ROSE_NO_DATA;
			return i;
		}
	}
	return -1;
}

int roseviz_bridge_topic_count(void) {
	return g_catalog_count;
}

int roseviz_bridge_topic_name(int index, uint8_t *out, int out_size) {
	if(index < 0 || index >= g_catalog_count || !out || out_size <= 0) {
		return 0;
	}
	return roseviz_copy_cstr((char *) out, out_size, g_catalog[index]);
}

int roseviz_bridge_slot_payload_size(int slot) {
	if(slot < 0 || slot >= ROSEVIZ_TOPIC_SLOT_COUNT || !g_slots[slot].active) {
		return -1;
	}
	return g_slots[slot].payload_size;
}

int roseviz_bridge_slot_status(int slot) {
	if(slot < 0 || slot >= ROSEVIZ_TOPIC_SLOT_COUNT || !g_slots[slot].active) {
		return -1;
	}
	return g_slots[slot].last_error;
}

uint64_t roseviz_bridge_slot_seq(int slot) {
	if(slot < 0 || slot >= ROSEVIZ_TOPIC_SLOT_COUNT || !g_slots[slot].active) {
		return 0;
	}
	return g_slots[slot].seq;
}

int roseviz_bridge_decode_slot_image_rgb(int slot, uint8_t *out_rgb, int max_bytes,
					 int *out_w, int *out_h, int *out_channels) {
	roseviz_numericarray_view view;
	int64_t h = 0;
	int64_t w = 0;
	int64_t c = 1;
	int64_t pixels;
	int64_t i;
	const uint8_t *src;

	if(roseviz_try_read_top_numericarray(slot, &view) != 0 || view.type != MIRAGE_ARRAY_U8) {
		return -1;
	}
	if(view.rank == 2) {
		h = view.dims[0];
		w = view.dims[1];
		c = 1;
	} else if(view.rank == 3) {
		h = view.dims[0];
		w = view.dims[1];
		c = view.dims[2];
	} else {
		return -1;
	}
	if(h <= 0 || w <= 0 || c <= 0) {
		return -1;
	}
	pixels = h * w;
	if(pixels <= 0 || pixels > (max_bytes / 3)) {
		return -1;
	}

	src = view.data;
	if(c == 3) {
		memcpy(out_rgb, src, (size_t) pixels * 3u);
	} else if(c == 1) {
		for(i = 0; i < pixels; ++i) {
			uint8_t v = src[i];
			out_rgb[i * 3 + 0] = v;
			out_rgb[i * 3 + 1] = v;
			out_rgb[i * 3 + 2] = v;
		}
	} else {
		for(i = 0; i < pixels; ++i) {
			int64_t base = i * c;
			out_rgb[i * 3 + 0] = src[base + 0];
			out_rgb[i * 3 + 1] = (c > 1) ? src[base + 1] : src[base + 0];
			out_rgb[i * 3 + 2] = (c > 2) ? src[base + 2] : 0u;
		}
	}

	if(out_w) {
		*out_w = (int) w;
	}
	if(out_h) {
		*out_h = (int) h;
	}
	if(out_channels) {
		*out_channels = 3;
	}
	return (int) pixels;
}

int roseviz_bridge_decode_slot_points_xyz(int slot, float *out_xyz, int max_points) {
	roseviz_numericarray_view view;
	int64_t count;
	int64_t cols;
	int i;

	if(max_points <= 0 || roseviz_try_read_top_numericarray(slot, &view) != 0 || view.rank != 2) {
		return -1;
	}
	count = view.dims[0];
	cols = view.dims[1];
	if(count <= 0 || cols < 3) {
		return -1;
	}
	if(count > max_points) {
		count = max_points;
	}

	if(view.type == MIRAGE_ARRAY_F32) {
		const float *src = (const float *) view.data;
		if(cols == 3) {
			memcpy(out_xyz, src, (size_t) count * 3u * sizeof(float));
		} else {
			for(i = 0; i < count; ++i) {
				out_xyz[i * 3 + 0] = src[i * cols + 0];
				out_xyz[i * 3 + 1] = src[i * cols + 1];
				out_xyz[i * 3 + 2] = src[i * cols + 2];
			}
		}
		return (int) count;
	}
	if(view.type == MIRAGE_ARRAY_F64) {
		const double *src = (const double *) view.data;
		for(i = 0; i < count; ++i) {
			out_xyz[i * 3 + 0] = (float) src[i * cols + 0];
			out_xyz[i * 3 + 1] = (float) src[i * cols + 1];
			out_xyz[i * 3 + 2] = (float) src[i * cols + 2];
		}
		return (int) count;
	}
	return -1;
}

int roseviz_bridge_decode_slot_transforms(int slot, float *out_tf, int max_edges) {
	roseviz_numericarray_view view;
	int64_t count;
	int64_t cols;
	int i;

	if(max_edges <= 0 || roseviz_try_read_top_numericarray(slot, &view) != 0 || view.rank != 2) {
		return -1;
	}
	count = view.dims[0];
	cols = view.dims[1];
	if(count <= 0 || cols < 5) {
		return -1;
	}
	if(count > max_edges) {
		count = max_edges;
	}

	if(view.type == MIRAGE_ARRAY_F32) {
		const float *src = (const float *) view.data;
		for(i = 0; i < count; ++i) {
			out_tf[i * 5 + 0] = src[i * cols + 0];
			out_tf[i * 5 + 1] = src[i * cols + 1];
			out_tf[i * 5 + 2] = src[i * cols + 2];
			out_tf[i * 5 + 3] = src[i * cols + 3];
			out_tf[i * 5 + 4] = src[i * cols + 4];
		}
		return (int) count;
	}
	if(view.type == MIRAGE_ARRAY_F64) {
		const double *src = (const double *) view.data;
		for(i = 0; i < count; ++i) {
			out_tf[i * 5 + 0] = (float) src[i * cols + 0];
			out_tf[i * 5 + 1] = (float) src[i * cols + 1];
			out_tf[i * 5 + 2] = (float) src[i * cols + 2];
			out_tf[i * 5 + 3] = (float) src[i * cols + 3];
			out_tf[i * 5 + 4] = (float) src[i * cols + 4];
		}
		return (int) count;
	}
	return -1;
}

int roseviz_bridge_publish_command(uint8_t *topic_bytes, uint8_t *cmd_bytes) {
	const char *topic = (const char *) topic_bytes;
	const char *cmd = (const char *) cmd_bytes;
	mirage_msg *msg = NULL;

	if(!g_node || !topic || !topic[0] || !cmd || !cmd[0]) {
		return -1;
	}

	if(!g_cmd_pub || strcmp(g_cmd_pub_topic, topic) != 0) {
		g_cmd_pub = rose_create_pub(g_node, (char *) topic, -1, 256, 64);
		if(!g_cmd_pub) {
			return -1;
		}
		roseviz_copy_cstr(g_cmd_pub_topic, (int) sizeof(g_cmd_pub_topic), topic);
	}

	msg = mirage_create(256, NULL);
	if(!msg) {
		return -1;
	}
	if(mirage_write_start(msg) < 0
	   || mirage_write_fn(msg, cmd, (i64) strlen(cmd), 0) < 0
	   || rose_publish(g_cmd_pub, msg) < 0) {
		mirage_destroy(&msg, NULL);
		return -1;
	}

	mirage_destroy(&msg, NULL);
	return 1;
}

int roseviz_spawn_process(uint8_t *path_bytes, uint8_t *arg1_bytes, uint8_t *arg2_bytes, uint8_t *log_path_bytes) {
	const char *path = (const char *) path_bytes;
	const char *arg1 = (const char *) arg1_bytes;
	const char *arg2 = (const char *) arg2_bytes;
	const char *log_path = (const char *) log_path_bytes;
	pid_t pid;

	if(!path || !path[0]) {
		return -1;
	}

	pid = fork();
	if(pid < 0) {
		return -1;
	}
	if(pid == 0) {
		char *argv[4];
		int argc = 0;

#ifdef __linux__
		prctl(PR_SET_PDEATHSIG, SIGTERM);
#endif
		if(log_path && log_path[0]) {
			int fd = open(log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
			if(fd >= 0) {
				dup2(fd, STDOUT_FILENO);
				dup2(fd, STDERR_FILENO);
				if(fd != STDOUT_FILENO && fd != STDERR_FILENO) {
					close(fd);
				}
			}
		}

		argv[argc++] = (char *) path;
		if(arg1 && arg1[0]) {
			argv[argc++] = (char *) arg1;
		}
		if(arg2 && arg2[0]) {
			argv[argc++] = (char *) arg2;
		}
		argv[argc] = NULL;
		execvp(path, argv);
		_exit(127);
	}

	return (int) pid;
}

void roseviz_stop_process(int pid) {
	int status = 0;
	int waited_ms = 0;
	if(pid <= 0) {
		return;
	}
	kill((pid_t) pid, SIGTERM);
	while(waited_ms < 500) {
		pid_t rc = waitpid((pid_t) pid, &status, WNOHANG);
		if(rc == (pid_t) pid) {
			return;
		}
		usleep(10000u);
		waited_ms += 10;
	}
	kill((pid_t) pid, SIGKILL);
	waitpid((pid_t) pid, &status, 0);
}
