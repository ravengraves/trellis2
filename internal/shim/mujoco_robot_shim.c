/*
 * mujoco_robot_shim.c — MuJoCo + ROS3 shim for the robot_sim target.
 *
 * Physics publishes immutable snapshots for render-thread reads. robot_sim now
 * steps MuJoCo from the main loop, while the optional background-thread entry
 * points remain available for compatibility.
 */

#include <mujoco/mujoco.h>
#include "ros3.h"

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CTRL_TOPIC_CAP 256
#define CTRL_NAME_CAP 128
#define SNAPSHOT_COUNT 2
#define CONTROL_TIMEOUT_S 0.5
#define CONTROL_MSG_BYTES 1024
#define CONTROL_SUB_RETRY_NS 250000000ULL
#define RESET_SUB_RETRY_NS 250000000ULL

typedef struct {
    float *geom_pos;
    float *geom_mat;
    float *body_pos;
    float *body_mat;
    float *site_pos;
    double time;
} mj_snapshot;

static mjModel *g_model = NULL;
static mjData *g_data = NULL;

static mj_snapshot g_snapshots[SNAPSHOT_COUNT];
static int g_front_snapshot = 0;
static __thread int g_snapshot_depth = 0;
static __thread int g_snapshot_index = 0;

static pthread_t g_physics_thread;
static int g_physics_thread_running = 0;
static int g_physics_stop = 0;

static char g_control_topic[CTRL_TOPIC_CAP] = "/control";
static char g_reset_topic[CTRL_TOPIC_CAP] = "/simulator/reset";
static char g_left_actuator_name[CTRL_NAME_CAP] = "";
static char g_right_actuator_name[CTRL_NAME_CAP] = "";
static int g_left_actuator = -1;
static int g_right_actuator = -1;
static float g_max_cmd = 0.3f;
static float g_last_left = 0.0f;
static float g_last_right = 0.0f;
static u64 g_last_control_ns = 0;
static u64 g_last_control_debug_ns = 0;
static int g_debug_control = -1;

static pthread_mutex_t g_control_lock = PTHREAD_MUTEX_INITIALIZER;
static rose_node *g_ros_node = NULL;
static rose_subscriber *g_control_sub = NULL;
static rose_subscriber *g_reset_sub = NULL;
static rose_publisher *g_rgb_pub = NULL;
static rose_publisher *g_depth_pub = NULL;
static rose_publisher *g_imu_pub = NULL;
static rose_publisher *g_vio_pub = NULL;
static mirage_msg *g_rgb_msg = NULL;
static mirage_msg *g_depth_msg = NULL;
static mirage_msg *g_imu_msg = NULL;
static mirage_msg *g_vio_msg = NULL;
static u64 g_control_sub_last_retry_ns = 0;
static u64 g_reset_sub_last_retry_ns = 0;
static int g_control_sub_failures = 0;
static int g_reset_sub_failures = 0;
static int g_rgb_publish_failures = 0;
static int g_depth_publish_failures = 0;
static int g_imu_publish_failures = 0;
static int g_vio_publish_failures = 0;
static int g_frame_body = -1;
static int g_imu_gyro_sensor = -1;
static int g_imu_accel_sensor = -1;
static int g_imu_gyro_adr = -1;
static int g_imu_accel_adr = -1;
static double g_imu_period_s = 0.001;
static double g_vio_period_s = 1.0 / 30.0;
static double g_next_imu_pub_time = 0.0;
static double g_next_vio_pub_time = 0.0;
static rose_publisher *g_pose_pub = NULL;
static mirage_msg *g_pose_msg = NULL;
static double g_pose_period_s = 1.0 / 30.0;
static double g_next_pose_pub_time = 0.0;
static int g_pose_publish_failures = 0;
static rose_publisher *g_vel_pub = NULL;
static mirage_msg *g_vel_msg = NULL;
static double g_vel_period_s = 1.0 / 30.0;
static double g_next_vel_pub_time = 0.0;
static int g_vel_publish_failures = 0;
static rose_publisher *g_accel_pub = NULL;
static mirage_msg *g_accel_msg = NULL;
static double g_accel_period_s = 1.0 / 30.0;
static double g_next_accel_pub_time = 0.0;
static int g_accel_publish_failures = 0;
static int g_reset_requested = 0;
static u64 g_reset_count = 0;

typedef struct {
    int has_pose;
    int has_z;
    int has_yaw;
    double x;
    double y;
    double z;
    double yaw;
} ResetRequest;

static pthread_mutex_t g_reset_lock = PTHREAD_MUTEX_INITIALIZER;
static ResetRequest g_pending_reset = {0};

static int debug_control_enabled(void) {
    const char *value;
    if(g_debug_control >= 0) {
        return g_debug_control;
    }
    value = getenv("ROBOT_SIM_DEBUG_CONTROL");
    g_debug_control = value && value[0] && strcmp(value, "0") != 0;
    return g_debug_control;
}

void mj_shim_stop_threaded(void);
void mj_shim_ros_motion_publish_due(void);
static int ensure_control_subscriber(u64 now_ns, int force);
static int ensure_reset_subscriber(u64 now_ns, int force);

static void snapshot_free(mj_snapshot *snap) {
    if(!snap) {
        return;
    }
    free(snap->geom_pos);
    free(snap->geom_mat);
    free(snap->body_pos);
    free(snap->body_mat);
    free(snap->site_pos);
    snap->geom_pos = NULL;
    snap->geom_mat = NULL;
    snap->body_pos = NULL;
    snap->body_mat = NULL;
    snap->site_pos = NULL;
    snap->time = 0.0;
}

static int snapshot_alloc(mj_snapshot *snap, int ngeom, int nbody, int nsite) {
    size_t geom_pos_count = (size_t) ngeom * 3u;
    size_t geom_mat_count = (size_t) ngeom * 9u;
    size_t body_pos_count = (size_t) nbody * 3u;
    size_t body_mat_count = (size_t) nbody * 9u;
    size_t site_pos_count = (size_t) nsite * 3u;

    if(!snap) {
        return -1;
    }

    memset(snap, 0, sizeof(*snap));
    snap->geom_pos = (float *) calloc(geom_pos_count ? geom_pos_count : 1u, sizeof(float));
    snap->geom_mat = (float *) calloc(geom_mat_count ? geom_mat_count : 1u, sizeof(float));
    snap->body_pos = (float *) calloc(body_pos_count ? body_pos_count : 1u, sizeof(float));
    snap->body_mat = (float *) calloc(body_mat_count ? body_mat_count : 1u, sizeof(float));
    snap->site_pos = (float *) calloc(site_pos_count ? site_pos_count : 1u, sizeof(float));
    if((geom_pos_count && !snap->geom_pos)
       || (geom_mat_count && !snap->geom_mat)
       || (body_pos_count && !snap->body_pos)
       || (body_mat_count && !snap->body_mat)
       || (site_pos_count && !snap->site_pos)) {
        snapshot_free(snap);
        return -1;
    }
    return 0;
}

static void shutdown_ros_io(void) {
    g_control_sub = NULL;
    g_reset_sub = NULL;
    g_rgb_pub = NULL;
    g_depth_pub = NULL;
    g_imu_pub = NULL;
    g_vio_pub = NULL;
    g_control_sub_last_retry_ns = 0;
    g_reset_sub_last_retry_ns = 0;
    g_control_sub_failures = 0;
    g_reset_sub_failures = 0;
    if(g_rgb_msg) {
        mirage_destroy(&g_rgb_msg, NULL);
        g_rgb_msg = NULL;
    }
    if(g_depth_msg) {
        mirage_destroy(&g_depth_msg, NULL);
        g_depth_msg = NULL;
    }
    if(g_imu_msg) {
        mirage_destroy(&g_imu_msg, NULL);
        g_imu_msg = NULL;
    }
    if(g_vio_msg) {
        mirage_destroy(&g_vio_msg, NULL);
        g_vio_msg = NULL;
    }
    if(g_ros_node) {
        rose_shutdown(g_ros_node);
        g_ros_node = NULL;
    }
}

static void shutdown_motion_io(void) {
    g_imu_pub = NULL;
    g_vio_pub = NULL;
    if(g_imu_msg) {
        mirage_destroy(&g_imu_msg, NULL);
        g_imu_msg = NULL;
    }
    if(g_vio_msg) {
        mirage_destroy(&g_vio_msg, NULL);
        g_vio_msg = NULL;
    }
    g_pose_pub = NULL;
    if(g_pose_msg) {
        mirage_destroy(&g_pose_msg, NULL);
        g_pose_msg = NULL;
    }
    g_imu_publish_failures = 0;
    g_vio_publish_failures = 0;
    g_pose_publish_failures = 0;
    g_vel_pub = NULL;
    if(g_vel_msg) {
        mirage_destroy(&g_vel_msg, NULL);
        g_vel_msg = NULL;
    }
    g_vel_publish_failures = 0;
    g_accel_pub = NULL;
    if(g_accel_msg) {
        mirage_destroy(&g_accel_msg, NULL);
        g_accel_msg = NULL;
    }
    g_accel_publish_failures = 0;
}

static void copy_arg_string(char *dst, size_t dst_size, const unsigned char *src, const char *fallback) {
    if(!dst || dst_size == 0u) {
        return;
    }
    if(src && src[0]) {
        snprintf(dst, dst_size, "%s", (const char *) src);
        return;
    }
    if(fallback) {
        snprintf(dst, dst_size, "%s", fallback);
    } else {
        dst[0] = '\0';
    }
}

static void resolve_actuator_ids(void) {
    if(!g_model) {
        g_left_actuator = -1;
        g_right_actuator = -1;
        return;
    }
    g_left_actuator = g_left_actuator_name[0]
        ? mj_name2id(g_model, mjOBJ_ACTUATOR, g_left_actuator_name)
        : -1;
    g_right_actuator = g_right_actuator_name[0]
        ? mj_name2id(g_model, mjOBJ_ACTUATOR, g_right_actuator_name)
        : -1;
}

static int ensure_ros_node(unsigned char *node_name) {
    char node_name_buf[CTRL_NAME_CAP];

    if(g_ros_node) {
        return 0;
    }

    copy_arg_string(node_name_buf, sizeof(node_name_buf), node_name, "simulator");
    g_ros_node = rose_init(0, NULL, node_name_buf, NULL, NULL);
    if(!g_ros_node) {
        return -1;
    }
    (void) ensure_control_subscriber(rose_now(), 1);
    (void) ensure_reset_subscriber(rose_now(), 1);
    return 0;
}

static void resolve_motion_sources(void) {
    g_frame_body = -1;
    g_imu_gyro_sensor = -1;
    g_imu_accel_sensor = -1;
    g_imu_gyro_adr = -1;
    g_imu_accel_adr = -1;

    if(!g_model) {
        return;
    }

    g_frame_body = mj_name2id(g_model, mjOBJ_BODY, "frame");
    g_imu_gyro_sensor = mj_name2id(g_model, mjOBJ_SENSOR, "imu_gyro");
    if(g_imu_gyro_sensor >= 0
       && g_model->sensor_dim[g_imu_gyro_sensor] == 3
       && g_model->sensor_adr[g_imu_gyro_sensor] >= 0) {
        g_imu_gyro_adr = g_model->sensor_adr[g_imu_gyro_sensor];
    } else {
        g_imu_gyro_sensor = -1;
    }

    g_imu_accel_sensor = mj_name2id(g_model, mjOBJ_SENSOR, "imu_accel");
    if(g_imu_accel_sensor >= 0
       && g_model->sensor_dim[g_imu_accel_sensor] == 3
       && g_model->sensor_adr[g_imu_accel_sensor] >= 0) {
        g_imu_accel_adr = g_model->sensor_adr[g_imu_accel_sensor];
    } else {
        g_imu_accel_sensor = -1;
    }
}

static void reset_motion_schedule(void) {
    double now = g_data ? g_data->time : 0.0;
    g_next_imu_pub_time = now + g_imu_period_s;
    g_next_vio_pub_time = now + g_vio_period_s;
    g_next_pose_pub_time = now + g_pose_period_s;
    g_next_vel_pub_time = now + g_vel_period_s;
    g_next_accel_pub_time = now + g_accel_period_s;
}

static const mj_snapshot *active_snapshot(void) {
    int idx = 0;
    if(!g_model) {
        return NULL;
    }
    if(g_snapshot_depth > 0) {
        idx = g_snapshot_index;
    } else {
        idx = __atomic_load_n(&g_front_snapshot, __ATOMIC_ACQUIRE);
    }
    if(idx < 0 || idx >= SNAPSHOT_COUNT) {
        idx = 0;
    }
    return &g_snapshots[idx];
}

static void capture_snapshot(int slot) {
    mj_snapshot *snap;
    int i;

    if(!g_model || !g_data || slot < 0 || slot >= SNAPSHOT_COUNT) {
        return;
    }

    snap = &g_snapshots[slot];
    for(i = 0; i < g_model->ngeom * 3; ++i) {
        snap->geom_pos[i] = (float) g_data->geom_xpos[i];
    }
    for(i = 0; i < g_model->ngeom * 9; ++i) {
        snap->geom_mat[i] = (float) g_data->geom_xmat[i];
    }
    for(i = 0; i < g_model->nbody * 3; ++i) {
        snap->body_pos[i] = (float) g_data->xpos[i];
    }
    for(i = 0; i < g_model->nbody * 9; ++i) {
        snap->body_mat[i] = (float) g_data->xmat[i];
    }
    for(i = 0; i < g_model->nsite * 3; ++i) {
        snap->site_pos[i] = (float) g_data->site_xpos[i];
    }
    snap->time = g_data->time;
    __atomic_store_n(&g_front_snapshot, slot, __ATOMIC_RELEASE);
}

static mjtNum clamp_actuator_command(int actuator_id, mjtNum cmd) {
    if(!g_model || actuator_id < 0 || actuator_id >= g_model->nu) {
        return 0.0;
    }
    if(g_model->actuator_ctrllimited[actuator_id]) {
        const mjtNum *range = g_model->actuator_ctrlrange + actuator_id * 2;
        cmd = mju_clip(cmd, range[0], range[1]);
    }
    return cmd;
}

static void apply_controls(float left, float right) {
    if(!g_model || !g_data || g_model->nu <= 0) {
        return;
    }
    if(g_left_actuator >= 0 && g_left_actuator < g_model->nu) {
        g_data->ctrl[g_left_actuator] = clamp_actuator_command(
            g_left_actuator,
            (mjtNum) left * (mjtNum) g_max_cmd);
    }
    if(g_right_actuator >= 0 && g_right_actuator < g_model->nu) {
        g_data->ctrl[g_right_actuator] = clamp_actuator_command(
            g_right_actuator,
            (mjtNum) right * (mjtNum) g_max_cmd);
    }
}

static int apply_reset_pose(const ResetRequest *request) {
    int root_joint = -1;
    int qadr = -1;
    double half_yaw;

    if(!request || !request->has_pose || !g_model || !g_data) {
        return 0;
    }

    root_joint = mj_name2id(g_model, mjOBJ_JOINT, "root");
    if(root_joint < 0 || root_joint >= g_model->njnt) {
        return -1;
    }
    qadr = g_model->jnt_qposadr[root_joint];
    if(qadr < 0 || qadr + 6 >= g_model->nq) {
        return -1;
    }

    g_data->qpos[qadr + 0] = (mjtNum) request->x;
    g_data->qpos[qadr + 1] = (mjtNum) request->y;
    if(request->has_z) {
        g_data->qpos[qadr + 2] = (mjtNum) request->z;
    }
    if(request->has_yaw) {
        half_yaw = 0.5 * request->yaw;
        g_data->qpos[qadr + 3] = (mjtNum) cos(half_yaw);
        g_data->qpos[qadr + 4] = 0.0;
        g_data->qpos[qadr + 5] = 0.0;
        g_data->qpos[qadr + 6] = (mjtNum) sin(half_yaw);
    }
    return 0;
}

static void reset_simulation_now(const ResetRequest *request) {
    int back_slot;
    int root_joint = -1;
    int qadr = -1;

    if(!g_model || !g_data) {
        return;
    }

    mj_resetData(g_model, g_data);
    (void) apply_reset_pose(request);
    mj_forward(g_model, g_data);

    pthread_mutex_lock(&g_control_lock);
    g_last_left = 0.0f;
    g_last_right = 0.0f;
    g_last_control_ns = 0;
    pthread_mutex_unlock(&g_control_lock);

    resolve_motion_sources();
    reset_motion_schedule();
    capture_snapshot(0);
    back_slot = 1 - __atomic_load_n(&g_front_snapshot, __ATOMIC_RELAXED);
    capture_snapshot(back_slot);
    g_reset_count += 1u;

    if(debug_control_enabled()) {
        root_joint = mj_name2id(g_model, mjOBJ_JOINT, "root");
        qadr = root_joint >= 0 ? g_model->jnt_qposadr[root_joint] : -1;
        if(qadr >= 0 && qadr + 6 < g_model->nq) {
            fprintf(stderr,
                    "robot_sim debug reset count=%llu pose=%d qpos=[%.4f %.4f %.4f %.4f %.4f %.4f %.4f] left_act=%d right_act=%d\n",
                    (unsigned long long)g_reset_count,
                    request ? request->has_pose : 0,
                    (double)g_data->qpos[qadr + 0],
                    (double)g_data->qpos[qadr + 1],
                    (double)g_data->qpos[qadr + 2],
                    (double)g_data->qpos[qadr + 3],
                    (double)g_data->qpos[qadr + 4],
                    (double)g_data->qpos[qadr + 5],
                    (double)g_data->qpos[qadr + 6],
                    g_left_actuator,
                    g_right_actuator);
        }
    }
}

int mj_shim_reset_now(void) {
    if(!g_model || !g_data) {
        return -1;
    }
    reset_simulation_now(NULL);
    return 0;
}

int mj_shim_consume_reset_request(void) {
    ResetRequest request;

    if(!__atomic_exchange_n(&g_reset_requested, 0, __ATOMIC_ACQ_REL)) {
        return 0;
    }
    if(!g_model || !g_data) {
        return -1;
    }
    pthread_mutex_lock(&g_reset_lock);
    request = g_pending_reset;
    memset(&g_pending_reset, 0, sizeof(g_pending_reset));
    pthread_mutex_unlock(&g_reset_lock);

    reset_simulation_now(&request);
    return 1;
}

u64 mj_shim_reset_count(void) {
    return g_reset_count;
}

static void step_simulation_once(void) {
    float left = 0.0f;
    float right = 0.0f;
    u64 control_ns = 0;
    u64 now_ns = 0;
    int back_slot;
    int root_joint = -1;
    int qadr = -1;
    int dadr = -1;

    if(!g_model || !g_data) {
        return;
    }

    pthread_mutex_lock(&g_control_lock);
    left = g_last_left;
    right = g_last_right;
    control_ns = g_last_control_ns;
    pthread_mutex_unlock(&g_control_lock);

    if(control_ns == 0 || rose_elapsed_seconds(control_ns) > CONTROL_TIMEOUT_S) {
        left = 0.0f;
        right = 0.0f;
    }

    apply_controls(left, right);
    mj_step(g_model, g_data);
    mj_shim_ros_motion_publish_due();

    if(debug_control_enabled()) {
        now_ns = rose_now();
        if(g_last_control_debug_ns == 0 ||
           now_ns - g_last_control_debug_ns > 500000000ULL) {
            g_last_control_debug_ns = now_ns;
            root_joint = mj_name2id(g_model, mjOBJ_JOINT, "root");
            qadr = root_joint >= 0 ? g_model->jnt_qposadr[root_joint] : -1;
            dadr = root_joint >= 0 ? g_model->jnt_dofadr[root_joint] : -1;
            fprintf(stderr,
                    "robot_sim debug step t=%.3f left=%.3f right=%.3f age=%.3f ctrl=[%.4f %.4f] qpos=[%.4f %.4f %.4f] qvel=[%.4f %.4f %.4f]\n",
                    g_data->time,
                    (double)left,
                    (double)right,
                    control_ns ? rose_elapsed_seconds(control_ns) : -1.0,
                    g_left_actuator >= 0 ? (double)g_data->ctrl[g_left_actuator] : 0.0,
                    g_right_actuator >= 0 ? (double)g_data->ctrl[g_right_actuator] : 0.0,
                    qadr >= 0 && qadr + 2 < g_model->nq ? (double)g_data->qpos[qadr + 0] : 0.0,
                    qadr >= 0 && qadr + 2 < g_model->nq ? (double)g_data->qpos[qadr + 1] : 0.0,
                    qadr >= 0 && qadr + 2 < g_model->nq ? (double)g_data->qpos[qadr + 2] : 0.0,
                    dadr >= 0 && dadr + 2 < g_model->nv ? (double)g_data->qvel[dadr + 0] : 0.0,
                    dadr >= 0 && dadr + 2 < g_model->nv ? (double)g_data->qvel[dadr + 1] : 0.0,
                    dadr >= 0 && dadr + 2 < g_model->nv ? (double)g_data->qvel[dadr + 2] : 0.0);
        }
    }

    back_slot = 1 - __atomic_load_n(&g_front_snapshot, __ATOMIC_RELAXED);
    capture_snapshot(back_slot);
}

static float clamp_signed_unit_f64(double value) {
    if(value < -1.0) {
        return -1.0f;
    }
    if(value > 1.0) {
        return 1.0f;
    }
    return (float) value;
}

static int decode_control_message(mirage_msg *msg, float *out_left, float *out_right) {
    char fn_name[32];
    i64 fn_len = 0;
    i64 argc = 0;
    double left = 0.0;
    double right = 0.0;

    if(!out_left || !out_right) {
        return 0;
    }
    if(!msg) {
        return 0;
    }
    if(mirage_read_start(msg) < 0) {
        return 0;
    }
    if(mirage_read_fn(msg, fn_name, &fn_len, (i64) sizeof(fn_name) - 1, &argc) < 0) {
        return 0;
    }
    if(argc < 2) {
        return 0;
    }
    if(mirage_read_f64(msg, &left) < 0
       || mirage_read_f64(msg, &right) < 0) {
        return 0;
    }
    *out_left = clamp_signed_unit_f64(left);
    *out_right = clamp_signed_unit_f64(right);
    return 1;
}

static void control_callback(mirage_msg *msg) {
    float left = 0.0f;
    float right = 0.0f;
    if(!decode_control_message(msg, &left, &right)) {
        if(debug_control_enabled()) {
            fprintf(stderr, "robot_sim debug control decode failed\n");
        }
        return;
    }
    pthread_mutex_lock(&g_control_lock);
    g_last_left = left;
    g_last_right = right;
    g_last_control_ns = rose_now();
    pthread_mutex_unlock(&g_control_lock);
    if(debug_control_enabled()) {
        fprintf(stderr, "robot_sim debug control left=%.3f right=%.3f\n",
                (double)left,
                (double)right);
    }
}

static int decode_reset_message(mirage_msg *msg, ResetRequest *out) {
    char fn_name[32];
    char command[32];
    i64 fn_len = 0;
    i64 command_len = 0;
    i64 argc = 0;
    i64 numeric_argc = 0;
    double values[4] = {0.0, 0.0, 0.0, 0.0};
    int value_count = 0;

    if(!out) {
        return 0;
    }
    memset(out, 0, sizeof(*out));
    if(!msg || mirage_read_start(msg) < 0) {
        return 0;
    }
    if(mirage_read_fn(msg, fn_name, &fn_len, (i64) sizeof(fn_name) - 1, &argc) < 0) {
        return 0;
    }
    fn_name[fn_len >= 0 && fn_len < (i64) sizeof(fn_name) ? fn_len : (i64) sizeof(fn_name) - 1] = '\0';
    if(argc <= 0) {
        return 1;
    }

    numeric_argc = argc;
    if(mirage_read_string(msg, command, &command_len, (i64) sizeof(command) - 1) >= 0) {
        command[command_len >= 0 && command_len < (i64) sizeof(command) ? command_len : (i64) sizeof(command) - 1] = '\0';
        numeric_argc = argc - 1;
    } else {
        if(mirage_read_start(msg) < 0 ||
           mirage_read_fn(msg, fn_name, &fn_len, (i64) sizeof(fn_name) - 1, &argc) < 0) {
            return 0;
        }
        numeric_argc = argc;
    }

    while(value_count < 4 && value_count < numeric_argc) {
        if(mirage_read_f64(msg, &values[value_count]) < 0) {
            break;
        }
        value_count++;
    }

    if(value_count >= 2) {
        out->has_pose = 1;
        out->x = values[0];
        out->y = values[1];
    }
    if(value_count == 3) {
        out->has_yaw = 1;
        out->yaw = values[2];
    } else if(value_count >= 4) {
        out->has_z = 1;
        out->z = values[2];
        out->has_yaw = 1;
        out->yaw = values[3];
    }
    return 1;
}

static void reset_callback(mirage_msg *msg) {
    ResetRequest request;

    if(!decode_reset_message(msg, &request)) {
        memset(&request, 0, sizeof(request));
    }
    pthread_mutex_lock(&g_reset_lock);
    g_pending_reset = request;
    pthread_mutex_unlock(&g_reset_lock);
    __atomic_store_n(&g_reset_requested, 1, __ATOMIC_RELEASE);
}

static int ensure_reset_subscriber(u64 now_ns, int force) {
    i64 err_code;

    if(!g_ros_node || g_reset_sub) {
        return g_reset_sub ? 0 : -1;
    }
    if(!force && g_reset_sub_last_retry_ns != 0
       && now_ns - g_reset_sub_last_retry_ns < RESET_SUB_RETRY_NS) {
        return -1;
    }

    g_reset_sub_last_retry_ns = now_ns;
    g_reset_sub = rose_create_sub(g_ros_node, g_reset_topic, -1, 1, reset_callback);
    if(g_reset_sub) {
        if(g_reset_sub_failures > 0) {
            fprintf(stderr, "robot_sim: reset subscriber connected to %s\n", g_reset_topic);
        }
        g_reset_sub_failures = 0;
        return 0;
    }

    if(g_reset_sub_failures == 0) {
        err_code = rose_last_error_code();
        if(err_code == -2) {
            fprintf(stderr,
                    "robot_sim: reset subscriber pending for %s; publisher not available yet, retrying\n",
                    g_reset_topic);
        } else {
            fprintf(stderr,
                    "robot_sim: reset subscriber pending for %s; create failed with code %lld, retrying\n",
                    g_reset_topic,
                    (long long) err_code);
        }
    }
    g_reset_sub_failures += 1;
    return -1;
}

static int ensure_control_subscriber(u64 now_ns, int force) {
    i64 err_code;

    if(!g_ros_node || g_control_sub) {
        return g_control_sub ? 0 : -1;
    }
    if(!force && g_control_sub_last_retry_ns != 0
       && now_ns - g_control_sub_last_retry_ns < CONTROL_SUB_RETRY_NS) {
        return -1;
    }

    g_control_sub_last_retry_ns = now_ns;
    g_control_sub = rose_create_sub(g_ros_node, g_control_topic, -1, 1, control_callback);
    if(g_control_sub) {
        if(g_control_sub_failures > 0) {
            fprintf(stderr, "robot_sim: control subscriber connected to %s\n", g_control_topic);
        }
        g_control_sub_failures = 0;
        return 0;
    }

    if(g_control_sub_failures == 0) {
        err_code = rose_last_error_code();
        if(err_code == -2) {
            fprintf(stderr,
                    "robot_sim: control subscriber pending for %s; publisher not available yet, retrying\n",
                    g_control_topic);
        } else {
            fprintf(stderr,
                    "robot_sim: control subscriber pending for %s; create failed with code %lld, retrying\n",
                    g_control_topic,
                    (long long) err_code);
        }
    }
    g_control_sub_failures += 1;
    return -1;
}

static void *physics_thread_main(void *unused) {
    double dt = 0.01;
    (void) unused;

    if(g_model) {
        dt = g_model->opt.timestep;
    }
    if(dt <= 0.0) {
        dt = 0.01;
    }

    while(!__atomic_load_n(&g_physics_stop, __ATOMIC_ACQUIRE)) {
        step_simulation_once();
        rose_sleep(dt);
    }
    return NULL;
}

static void free_model_data(void) {
    int i;

    for(i = 0; i < SNAPSHOT_COUNT; ++i) {
        snapshot_free(&g_snapshots[i]);
    }
    if(g_data) {
        mj_deleteData(g_data);
        g_data = NULL;
    }
    if(g_model) {
        mj_deleteModel(g_model);
        g_model = NULL;
    }
    g_front_snapshot = 0;
    g_snapshot_depth = 0;
    g_snapshot_index = 0;
    g_left_actuator = -1;
    g_right_actuator = -1;
    g_frame_body = -1;
    g_imu_gyro_sensor = -1;
    g_imu_accel_sensor = -1;
    g_imu_gyro_adr = -1;
    g_imu_accel_adr = -1;
    g_last_left = 0.0f;
    g_last_right = 0.0f;
    g_last_control_ns = 0;
    g_reset_requested = 0;
    memset(&g_pending_reset, 0, sizeof(g_pending_reset));
}

int mj_shim_load(unsigned char *path) {
    char err[512] = {0};
    int i;

    mj_shim_stop_threaded();
    shutdown_ros_io();
    free_model_data();

    g_model = mj_loadXML((const char *) path, NULL, err, sizeof(err));
    if(!g_model) {
        fprintf(stderr, "mj_shim_load: %s\n", err);
        return -1;
    }
    g_data = mj_makeData(g_model);
    if(!g_data) {
        mj_deleteModel(g_model);
        g_model = NULL;
        return -1;
    }
    for(i = 0; i < SNAPSHOT_COUNT; ++i) {
        if(snapshot_alloc(&g_snapshots[i], g_model->ngeom, g_model->nbody, g_model->nsite) != 0) {
            free_model_data();
            return -1;
        }
    }
    mj_forward(g_model, g_data);
    resolve_actuator_ids();
    resolve_motion_sources();
    reset_motion_schedule();
    capture_snapshot(0);
    return 0;
}

void mj_shim_snapshot_begin(void) {
    if(g_snapshot_depth == 0) {
        g_snapshot_index = __atomic_load_n(&g_front_snapshot, __ATOMIC_ACQUIRE);
    }
    g_snapshot_depth += 1;
}

void mj_shim_snapshot_end(void) {
    if(g_snapshot_depth > 0) {
        g_snapshot_depth -= 1;
    }
}

void mj_shim_step(void) {
    if(!g_model || !g_data) {
        return;
    }
    if(g_physics_thread_running) {
        return;
    }
    step_simulation_once();
}

int mj_shim_ngeom(void) {
    return g_model ? g_model->ngeom : 0;
}

int mj_shim_geom_type(int i) {
    if(!g_model || i < 0 || i >= g_model->ngeom) {
        return -1;
    }
    return g_model->geom_type[i];
}

void mj_shim_geom_size(int i, float *out) {
    const mjtNum *s;

    if(!g_model || !out || i < 0 || i >= g_model->ngeom) {
        return;
    }
    s = g_model->geom_size + i * 3;
    out[0] = (float) s[0];
    out[1] = (float) s[1];
    out[2] = (float) s[2];
}

void mj_shim_geom_pos(int i, float *out) {
    const mj_snapshot *snap = active_snapshot();
    const float *src;

    if(!snap || !out || i < 0 || i >= g_model->ngeom) {
        return;
    }
    src = snap->geom_pos + i * 3;
    out[0] = src[0];
    out[1] = src[1];
    out[2] = src[2];
}

void mj_shim_geom_mat(int i, float *out) {
    const mj_snapshot *snap = active_snapshot();
    int j;

    if(!snap || !out || i < 0 || i >= g_model->ngeom) {
        return;
    }
    for(j = 0; j < 9; ++j) {
        out[j] = snap->geom_mat[i * 9 + j];
    }
}

void mj_shim_geom_rgba(int i, float *out) {
    int matid;
    const float *rgba;

    if(!g_model || !out || i < 0 || i >= g_model->ngeom) {
        return;
    }
    matid = g_model->geom_matid[i];
    if(matid >= 0 && matid < g_model->nmat) {
        rgba = g_model->mat_rgba + matid * 4;
    } else {
        rgba = g_model->geom_rgba + i * 4;
    }
    out[0] = rgba[0];
    out[1] = rgba[1];
    out[2] = rgba[2];
    out[3] = rgba[3];
}

double mj_shim_timestep(void) {
    return g_model ? g_model->opt.timestep : 0.002;
}

int mj_shim_set_timestep(double timestep) {
    if(!g_model || timestep <= 0.0) {
        return -1;
    }
    g_model->opt.timestep = (mjtNum) timestep;
    return 0;
}

float mj_shim_stat_extent(void) {
    return g_model ? (float) g_model->stat.extent : 5.0f;
}

double mj_shim_time(void) {
    const mj_snapshot *snap = active_snapshot();
    if(snap) {
        return snap->time;
    }
    return g_data ? g_data->time : 0.0;
}

int mj_shim_body_id(unsigned char *name) {
    if(!g_model || !name) {
        return -1;
    }
    return mj_name2id(g_model, mjOBJ_BODY, (const char *) name);
}

int mj_shim_geom_id(unsigned char *name) {
    if(!g_model || !name) {
        return -1;
    }
    return mj_name2id(g_model, mjOBJ_GEOM, (const char *) name);
}

int mj_shim_nbody(void) {
    return g_model ? g_model->nbody : 0;
}

int mj_shim_nsite(void) {
    return g_model ? g_model->nsite : 0;
}

int mj_shim_ntendon(void) {
    return g_model ? g_model->ntendon : 0;
}

void mj_shim_body_pos(int i, float *out) {
    const mj_snapshot *snap = active_snapshot();
    const float *src;

    if(!snap || !out || i < 0 || i >= g_model->nbody) {
        return;
    }
    src = snap->body_pos + i * 3;
    out[0] = src[0];
    out[1] = src[1];
    out[2] = src[2];
}

void mj_shim_body_mat(int i, float *out) {
    const mj_snapshot *snap = active_snapshot();
    int j;

    if(!snap || !out || i < 0 || i >= g_model->nbody) {
        return;
    }
    for(j = 0; j < 9; ++j) {
        out[j] = snap->body_mat[i * 9 + j];
    }
}

void mj_shim_site_pos(int i, float *out) {
    const mj_snapshot *snap = active_snapshot();
    const float *src;

    if(!snap || !out || !g_model || i < 0 || i >= g_model->nsite) {
        return;
    }
    src = snap->site_pos + i * 3;
    out[0] = src[0];
    out[1] = src[1];
    out[2] = src[2];
}

int mj_shim_tendon_site_pair(int i, int *out) {
    int adr;

    if(!g_model || !out || i < 0 || i >= g_model->ntendon) {
        return 0;
    }
    if(g_model->tendon_num[i] != 2) {
        return 0;
    }
    adr = g_model->tendon_adr[i];
    if(adr < 0 || adr + 1 >= g_model->nwrap) {
        return 0;
    }
    if(g_model->wrap_type[adr] != mjWRAP_SITE
       || g_model->wrap_type[adr + 1] != mjWRAP_SITE) {
        return 0;
    }
    out[0] = g_model->wrap_objid[adr];
    out[1] = g_model->wrap_objid[adr + 1];
    return 1;
}

float mj_shim_tendon_width(int i) {
    if(!g_model || i < 0 || i >= g_model->ntendon) {
        return 0.0f;
    }
    return (float) g_model->tendon_width[i];
}

void mj_shim_tendon_rgba(int i, float *out) {
    const float *rgba;

    if(!out) {
        return;
    }
    out[0] = 0.75f;
    out[1] = 0.82f;
    out[2] = 0.92f;
    out[3] = 1.0f;
    if(!g_model || i < 0 || i >= g_model->ntendon) {
        return;
    }
    rgba = g_model->tendon_rgba + i * 4;
    out[0] = rgba[0];
    out[1] = rgba[1];
    out[2] = rgba[2];
    out[3] = rgba[3];
}

int mj_shim_start_threaded(void) {
    if(!g_model || !g_data) {
        return -1;
    }
    if(g_physics_thread_running) {
        return 0;
    }
    __atomic_store_n(&g_physics_stop, 0, __ATOMIC_RELEASE);
    if(pthread_create(&g_physics_thread, NULL, physics_thread_main, NULL) != 0) {
        return -1;
    }
    g_physics_thread_running = 1;
    return 0;
}

void mj_shim_stop_threaded(void) {
    if(!g_physics_thread_running) {
        return;
    }
    __atomic_store_n(&g_physics_stop, 1, __ATOMIC_RELEASE);
    pthread_join(g_physics_thread, NULL);
    g_physics_thread_running = 0;
}

int mj_shim_threaded_active(void) {
    return g_physics_thread_running;
}

int mj_shim_configure_tank_controls(
    unsigned char *topic,
    unsigned char *left_actuator,
    unsigned char *right_actuator,
    float max_cmd) {
    copy_arg_string(g_control_topic, sizeof(g_control_topic), topic, "/control");
    copy_arg_string(g_left_actuator_name, sizeof(g_left_actuator_name), left_actuator, "");
    copy_arg_string(g_right_actuator_name, sizeof(g_right_actuator_name), right_actuator, "");
    g_max_cmd = max_cmd > 0.0f ? max_cmd : 0.3f;
    resolve_actuator_ids();
    if(!g_model) {
        return 0;
    }
    return (g_left_actuator >= 0 && g_right_actuator >= 0) ? 0 : -1;
}

int mj_shim_motion_source_status(void) {
    int status = 0;
    if(g_frame_body >= 0) {
        status |= 1;
    }
    if(g_imu_gyro_adr >= 0) {
        status |= 2;
    }
    if(g_imu_accel_adr >= 0) {
        status |= 4;
    }
    return status;
}

int mj_shim_motion_sample(double *imu_out, double *vio_out) {
    mjtNum vel[6];
    mjtNum acc[6];
    int i;

    if(!g_model || !g_data) {
        return -1;
    }
    if(g_frame_body < 0 || g_frame_body >= g_model->nbody) {
        resolve_motion_sources();
    }
    if(g_frame_body < 0 || g_frame_body >= g_model->nbody) {
        return -1;
    }

    if(imu_out) {
        imu_out[0] = (double) g_data->time;
        if(g_imu_gyro_adr >= 0 && g_imu_gyro_adr + 2 < g_model->nsensordata) {
            for(i = 0; i < 3; ++i) {
                imu_out[1 + i] = (double) g_data->sensordata[g_imu_gyro_adr + i];
            }
        } else {
            mj_objectVelocity(g_model, g_data, mjOBJ_XBODY, g_frame_body, vel, 1);
            for(i = 0; i < 3; ++i) {
                imu_out[1 + i] = (double) vel[i];
            }
        }

        if(g_imu_accel_adr >= 0 && g_imu_accel_adr + 2 < g_model->nsensordata) {
            for(i = 0; i < 3; ++i) {
                imu_out[4 + i] = (double) g_data->sensordata[g_imu_accel_adr + i];
            }
        } else {
            mj_objectAcceleration(g_model, g_data, mjOBJ_XBODY, g_frame_body, acc, 1);
            for(i = 0; i < 3; ++i) {
                imu_out[4 + i] = (double) acc[3 + i];
            }
        }
    }

    if(vio_out) {
        mj_objectVelocity(g_model, g_data, mjOBJ_XBODY, g_frame_body, vel, 0);
        vio_out[0] = (double) g_data->time;
        for(i = 0; i < 3; ++i) {
            vio_out[1 + i] = (double) g_data->xpos[g_frame_body * 3 + i];
            vio_out[4 + i] = (double) vel[3 + i];
        }
    }

    return 0;
}

static int publish_f64_array(
    rose_publisher *pub,
    mirage_msg *msg,
    const double *data,
    i64 count,
    const char *label,
    int *failures) {
    i64 dims[1];
    int64_t rc;

    if(!pub || !msg || !data || count <= 0) {
        return -1;
    }

    dims[0] = count;
    rc = mirage_write_start(msg);
    if(rc < 0) {
        return -1;
    }
    rc = mirage_write_numericarray(msg, MIRAGE_ARRAY_F64, 1, dims, (void *) data);
    if(rc < 0) {
        return -1;
    }
    if(rose_publish(pub, msg) < 0) {
        if(failures && *failures < 8) {
            fprintf(stderr, "robot_sim: %s publish failed: %s\n", label, rose_last_error());
        }
        if(failures) {
            *failures += 1;
        }
        return -1;
    }
    if(failures) {
        *failures = 0;
    }
    return 0;
}

void mj_shim_ros_motion_publish_due(void) {
    double now;
    double imu[7];
    double vio[7];

    if(!g_model || !g_data) {
        return;
    }
    now = (double) g_data->time;

    if(g_imu_pub && g_imu_msg && g_imu_period_s > 0.0
       && now + 1e-12 >= g_next_imu_pub_time) {
        if(mj_shim_motion_sample(imu, NULL) == 0) {
            (void) publish_f64_array(
                g_imu_pub, g_imu_msg, imu, 7, "IMU", &g_imu_publish_failures);
        }
        do {
            g_next_imu_pub_time += g_imu_period_s;
        } while(now + 1e-12 >= g_next_imu_pub_time);
    }

    if(g_vio_pub && g_vio_msg && g_vio_period_s > 0.0
       && now + 1e-12 >= g_next_vio_pub_time) {
        if(mj_shim_motion_sample(NULL, vio) == 0) {
            (void) publish_f64_array(
                g_vio_pub, g_vio_msg, vio, 7, "VIO", &g_vio_publish_failures);
        }
        do {
            g_next_vio_pub_time += g_vio_period_s;
        } while(now + 1e-12 >= g_next_vio_pub_time);
    }

    if(g_pose_pub && g_pose_msg) {
        if(g_frame_body >= 0 && g_frame_body < g_model->nbody) {
            double pose[16];
            int bi = g_frame_body;
            pose[0] = (double) g_data->xmat[bi * 9 + 0];
            pose[1] = (double) g_data->xmat[bi * 9 + 1];
            pose[2] = (double) g_data->xmat[bi * 9 + 2];
            pose[3] = 0.0;
            pose[4] = (double) g_data->xmat[bi * 9 + 3];
            pose[5] = (double) g_data->xmat[bi * 9 + 4];
            pose[6] = (double) g_data->xmat[bi * 9 + 5];
            pose[7] = 0.0;
            pose[8] = (double) g_data->xmat[bi * 9 + 6];
            pose[9] = (double) g_data->xmat[bi * 9 + 7];
            pose[10] = (double) g_data->xmat[bi * 9 + 8];
            pose[11] = 0.0;
            pose[12] = (double) g_data->xpos[bi * 3 + 0];
            pose[13] = (double) g_data->xpos[bi * 3 + 1];
            pose[14] = (double) g_data->xpos[bi * 3 + 2];
            pose[15] = 1.0;
            (void) publish_f64_array(
                g_pose_pub, g_pose_msg, pose, 16, "pose", &g_pose_publish_failures);
        }
    }

    if(g_vel_pub && g_vel_msg) {
        if(g_frame_body >= 0 && g_frame_body < g_model->nbody) {
            mjtNum vel[6];
            double vel_arr[7];
            int i;
            mj_objectVelocity(g_model, g_data, mjOBJ_XBODY, g_frame_body, vel, 0);
            vel_arr[0] = (double) g_data->time;
            for(i = 0; i < 3; ++i) {
                vel_arr[1 + i] = (double) vel[i];
                vel_arr[4 + i] = (double) vel[3 + i];
            }
            (void) publish_f64_array(
                g_vel_pub, g_vel_msg, vel_arr, 7, "vel", &g_vel_publish_failures);
        }
    }

    if(g_accel_pub && g_accel_msg) {
        if(g_frame_body >= 0 && g_frame_body < g_model->nbody) {
            mjtNum acc[6];
            double accel_arr[4];
            int i;
            mj_objectAcceleration(g_model, g_data, mjOBJ_XBODY, g_frame_body, acc, 1);
            accel_arr[0] = (double) g_data->time;
            for(i = 0; i < 3; ++i) {
                accel_arr[1 + i] = (double) acc[3 + i];
            }
            (void) publish_f64_array(
                g_accel_pub, g_accel_msg, accel_arr, 4, "accel", &g_accel_publish_failures);
        }
    }
}

int mj_shim_ros_motion_init(
    unsigned char *node_name,
    unsigned char *imu_topic,
    unsigned char *vio_topic,
    int imu_message_bytes,
    int vio_message_bytes,
    int imu_hz,
    int vio_hz) {
    char imu_topic_buf[CTRL_TOPIC_CAP];
    char vio_topic_buf[CTRL_TOPIC_CAP];
    int imu_bytes = imu_message_bytes > 0 ? imu_message_bytes : 1024;
    int vio_bytes = vio_message_bytes > 0 ? vio_message_bytes : 1024;
    int imu_rate_hz = imu_hz > 0 ? imu_hz : 1000;
    int vio_rate_hz = vio_hz > 0 ? vio_hz : 30;

    shutdown_motion_io();
    if(ensure_ros_node(node_name) != 0) {
        return -1;
    }

    copy_arg_string(imu_topic_buf, sizeof(imu_topic_buf), imu_topic, "/simulator/imu");
    copy_arg_string(vio_topic_buf, sizeof(vio_topic_buf), vio_topic, "/simulator/vio");
    g_imu_period_s = 1.0 / (double) imu_rate_hz;
    g_vio_period_s = 1.0 / (double) vio_rate_hz;
    resolve_motion_sources();
    reset_motion_schedule();

    g_imu_pub = rose_create_pub(g_ros_node, imu_topic_buf, -1, (u64) imu_bytes, (u64) imu_rate_hz);
    g_vio_pub = rose_create_pub(g_ros_node, vio_topic_buf, -1, (u64) vio_bytes, (u64) vio_rate_hz);
    g_pose_pub = rose_create_pub(g_ros_node, "/simulator/pose", -1, 1024, (u64) vio_rate_hz);
    g_vel_pub = rose_create_pub(g_ros_node, "/simulator/vel", -1, 1024, (u64) vio_rate_hz);
    g_accel_pub = rose_create_pub(g_ros_node, "/simulator/accel", -1, 1024, (u64) vio_rate_hz);
    g_imu_msg = mirage_create(imu_bytes, NULL);
    g_vio_msg = mirage_create(vio_bytes, NULL);
    g_pose_msg = mirage_create(1024, NULL);
    g_vel_msg = mirage_create(1024, NULL);
    g_accel_msg = mirage_create(1024, NULL);
    if(!g_imu_pub || !g_vio_pub || !g_pose_pub || !g_vel_pub || !g_accel_pub
       || !g_imu_msg || !g_vio_msg || !g_pose_msg || !g_vel_msg || !g_accel_msg) {
        shutdown_motion_io();
        return -1;
    }
    return 0;
}

int mj_shim_ros_render_init(
    unsigned char *node_name,
    unsigned char *rgb_topic,
    unsigned char *depth_topic,
    int rgb_message_bytes,
    int depth_message_bytes,
    int hz) {
    char node_name_buf[CTRL_NAME_CAP];
    char rgb_topic_buf[CTRL_TOPIC_CAP];
    char depth_topic_buf[CTRL_TOPIC_CAP];
    int rgb_bytes = rgb_message_bytes > 0 ? rgb_message_bytes : 65536;
    int depth_bytes = depth_message_bytes > 0 ? depth_message_bytes : 65536;
    int rate_hz = hz > 0 ? hz : 30;

    shutdown_ros_io();
    copy_arg_string(node_name_buf, sizeof(node_name_buf), node_name, "simulator");
    copy_arg_string(rgb_topic_buf, sizeof(rgb_topic_buf), rgb_topic, "/simulator/rgb");
    copy_arg_string(depth_topic_buf, sizeof(depth_topic_buf), depth_topic, "/simulator/depth");

    g_ros_node = rose_init(0, NULL, node_name_buf, NULL, NULL);
    if(!g_ros_node) {
        shutdown_ros_io();
        return -1;
    }

    (void) ensure_control_subscriber(rose_now(), 1);
    (void) ensure_reset_subscriber(rose_now(), 1);
    g_rgb_pub = rose_create_pub(g_ros_node, rgb_topic_buf, -1, (u64) rgb_bytes, (u64) rate_hz);
    g_depth_pub = rose_create_pub(g_ros_node, depth_topic_buf, -1, (u64) depth_bytes, (u64) rate_hz);
    g_rgb_msg = mirage_create(rgb_bytes, NULL);
    g_depth_msg = mirage_create(depth_bytes, NULL);

    pthread_mutex_lock(&g_control_lock);
    g_last_left = 0.0f;
    g_last_right = 0.0f;
    g_last_control_ns = 0;
    pthread_mutex_unlock(&g_control_lock);
    g_rgb_publish_failures = 0;
    g_depth_publish_failures = 0;

    if(!g_rgb_pub || !g_depth_pub || !g_rgb_msg || !g_depth_msg) {
        shutdown_ros_io();
        return -1;
    }
    return 0;
}

void mj_shim_ros_spin_once(void) {
    if(g_ros_node) {
        if(!g_control_sub) {
            (void) ensure_control_subscriber(rose_now(), 0);
        }
        if(!g_reset_sub) {
            (void) ensure_reset_subscriber(rose_now(), 0);
        }
        rose_spin_once(g_ros_node);
    }
}

int mj_shim_ros_ok(void) {
    if(!g_ros_node) {
        return 0;
    }
    return rose_ok(g_ros_node) ? 1 : 0;
}

u64 mj_shim_ros_rgb_index(void) {
    if(!g_rgb_pub || !g_rgb_pub->pub || !g_rgb_pub->pub->buffer) {
        return 0;
    }
    return (u64) g_rgb_pub->pub->buffer->index;
}

u64 mj_shim_ros_imu_index(void) {
    if(!g_imu_pub || !g_imu_pub->pub || !g_imu_pub->pub->buffer) {
        return 0;
    }
    return (u64) g_imu_pub->pub->buffer->index;
}

u64 mj_shim_ros_vio_index(void) {
    if(!g_vio_pub || !g_vio_pub->pub || !g_vio_pub->pub->buffer) {
        return 0;
    }
    return (u64) g_vio_pub->pub->buffer->index;
}

int mj_shim_ros_render_publish_rgb(unsigned char *rgb, int width, int height, int channels) {
    i64 dims2[2];
    i64 dims3[3];
    int64_t rc;

    if(!g_rgb_pub || !g_rgb_msg || !rgb || width <= 0 || height <= 0) {
        return -1;
    }

    dims2[0] = height;
    dims2[1] = width;
    dims3[0] = height;
    dims3[1] = width;
    dims3[2] = channels > 0 ? channels : 3;

    rc = mirage_write_start(g_rgb_msg);
    if(rc < 0) {
        return -1;
    }
    if(channels <= 1) {
        rc = mirage_write_numericarray(g_rgb_msg, MIRAGE_ARRAY_U8, 2, dims2, rgb);
    } else {
        rc = mirage_write_numericarray(g_rgb_msg, MIRAGE_ARRAY_U8, 3, dims3, rgb);
    }
    if(rc < 0) {
        return -1;
    }
    if(rose_publish(g_rgb_pub, g_rgb_msg) < 0) {
        if(g_rgb_publish_failures < 8) {
            fprintf(stderr, "robot_sim: RGB publish failed: %s\n", rose_last_error());
        }
        g_rgb_publish_failures += 1;
        return -1;
    }
    g_rgb_publish_failures = 0;
    return 0;
}

int mj_shim_ros_render_publish_depth(float *depth, int width, int height) {
    i64 dims[2];
    int64_t rc;

    if(!g_depth_pub || !g_depth_msg || !depth || width <= 0 || height <= 0) {
        return -1;
    }

    dims[0] = height;
    dims[1] = width;
    rc = mirage_write_start(g_depth_msg);
    if(rc < 0) {
        return -1;
    }
    rc = mirage_write_numericarray(g_depth_msg, MIRAGE_ARRAY_F32, 2, dims, depth);
    if(rc < 0) {
        return -1;
    }
    if(rose_publish(g_depth_pub, g_depth_msg) < 0) {
        if(g_depth_publish_failures < 8) {
            fprintf(stderr, "robot_sim: depth publish failed: %s\n", rose_last_error());
        }
        g_depth_publish_failures += 1;
        return -1;
    }
    g_depth_publish_failures = 0;
    return 0;
}

void mj_shim_free(void) {
    mj_shim_stop_threaded();
    shutdown_ros_io();
    free_model_data();
}
