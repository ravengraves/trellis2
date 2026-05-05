/*
 * mujoco_shim.c — C shim wrapping MuJoCo API for Pika FFI.
 *
 * Holds mjModel* and mjData* as static globals (same pattern as pollyn_shim.c).
 * Pika calls these flat functions via @symbol declarations.
 */

#include <stdio.h>
#include <string.h>
#include <mujoco/mujoco.h>

static mjModel *g_model = NULL;
static mjData  *g_data  = NULL;

/* Load a MuJoCo XML model, create data, compute initial state.
 * Returns 0 on success, -1 on failure. */
int mj_shim_load(unsigned char *path) {
    char err[512] = {0};
    g_model = mj_loadXML((const char *)path, NULL, err, sizeof(err));
    if (!g_model) {
        fprintf(stderr, "mj_shim_load: %s\n", err);
        return -1;
    }
    g_data = mj_makeData(g_model);
    if (!g_data) {
        mj_deleteModel(g_model);
        g_model = NULL;
        return -1;
    }
    mj_forward(g_model, g_data);
    return 0;
}

/* Compatibility hook for snapshot-based readers. */
void mj_shim_snapshot_begin(void) {
}

void mj_shim_snapshot_end(void) {
}

/* Step the simulation forward by one timestep. */
void mj_shim_step(void) {
    if (g_model && g_data) {
        mj_step(g_model, g_data);
    }
}

/* Return the number of geoms in the model. */
int mj_shim_ngeom(void) {
    return g_model ? g_model->ngeom : 0;
}

/* Return the geom type for geom i. */
int mj_shim_geom_type(int i) {
    if (!g_model || i < 0 || i >= g_model->ngeom) return -1;
    return g_model->geom_type[i];
}

/* Copy geom_size[i*3..i*3+3] into out (3 floats). */
void mj_shim_geom_size(int i, float *out) {
    if (!g_model || i < 0 || i >= g_model->ngeom) return;
    const mjtNum *s = g_model->geom_size + i * 3;
    out[0] = (float)s[0];
    out[1] = (float)s[1];
    out[2] = (float)s[2];
}

/* Copy data->geom_xpos[i*3..i*3+3] into out (3 floats). */
void mj_shim_geom_pos(int i, float *out) {
    if (!g_data || !g_model || i < 0 || i >= g_model->ngeom) return;
    const mjtNum *p = g_data->geom_xpos + i * 3;
    out[0] = (float)p[0];
    out[1] = (float)p[1];
    out[2] = (float)p[2];
}

/* Copy data->geom_xmat[i*9..i*9+9] into out (9 floats, row-major 3x3). */
void mj_shim_geom_mat(int i, float *out) {
    if (!g_data || !g_model || i < 0 || i >= g_model->ngeom) return;
    const mjtNum *m = g_data->geom_xmat + i * 9;
    for (int j = 0; j < 9; j++) {
        out[j] = (float)m[j];
    }
}

/* Resolve RGBA for geom: use material if assigned, else geom_rgba. */
void mj_shim_geom_rgba(int i, float *out) {
    if (!g_model || i < 0 || i >= g_model->ngeom) return;
    int matid = g_model->geom_matid[i];
    if (matid >= 0 && matid < g_model->nmat) {
        const float *rgba = g_model->mat_rgba + matid * 4;
        out[0] = rgba[0];
        out[1] = rgba[1];
        out[2] = rgba[2];
        out[3] = rgba[3];
    } else {
        const float *rgba = g_model->geom_rgba + i * 4;
        out[0] = rgba[0];
        out[1] = rgba[1];
        out[2] = rgba[2];
        out[3] = rgba[3];
    }
}

/* Return model timestep. */
double mj_shim_timestep(void) {
    return g_model ? g_model->opt.timestep : 0.002;
}

/* Return model stat extent (for plane scaling). */
float mj_shim_stat_extent(void) {
    return g_model ? (float)g_model->stat.extent : 5.0f;
}

/* Return current simulation time. */
double mj_shim_time(void) {
    return g_data ? g_data->time : 0.0;
}

/* Resolve a body id from its MuJoCo name. */
int mj_shim_body_id(unsigned char *name) {
    if (!g_model || !name) return -1;
    return mj_name2id(g_model, mjOBJ_BODY, (const char *)name);
}

/* Return the number of bodies. */
int mj_shim_nbody(void) {
    return g_model ? g_model->nbody : 0;
}

/* Return the number of sites. */
int mj_shim_nsite(void) {
    return g_model ? g_model->nsite : 0;
}

/* Return the number of tendons. */
int mj_shim_ntendon(void) {
    return g_model ? g_model->ntendon : 0;
}

/* Copy body world position into out. */
void mj_shim_body_pos(int i, float *out) {
    if (!g_data || !g_model || !out || i < 0 || i >= g_model->nbody) return;
    const mjtNum *p = g_data->xpos + i * 3;
    out[0] = (float)p[0];
    out[1] = (float)p[1];
    out[2] = (float)p[2];
}

/* Copy body world orientation into out (row-major 3x3). */
void mj_shim_body_mat(int i, float *out) {
    if (!g_data || !g_model || !out || i < 0 || i >= g_model->nbody) return;
    const mjtNum *m = g_data->xmat + i * 9;
    for (int j = 0; j < 9; j++) {
        out[j] = (float)m[j];
    }
}

/* Copy site world position into out. */
void mj_shim_site_pos(int i, float *out) {
    if (!g_data || !g_model || !out || i < 0 || i >= g_model->nsite) return;
    const mjtNum *p = g_data->site_xpos + i * 3;
    out[0] = (float)p[0];
    out[1] = (float)p[1];
    out[2] = (float)p[2];
}

/* Resolve a simple two-site tendon into site ids. */
int mj_shim_tendon_site_pair(int i, int *out) {
    if (!g_model || !out || i < 0 || i >= g_model->ntendon) return 0;
    if (g_model->tendon_num[i] != 2) return 0;
    int adr = g_model->tendon_adr[i];
    if (adr < 0 || adr + 1 >= g_model->nwrap) return 0;
    if (g_model->wrap_type[adr] != mjWRAP_SITE ||
        g_model->wrap_type[adr + 1] != mjWRAP_SITE) {
        return 0;
    }
    out[0] = g_model->wrap_objid[adr];
    out[1] = g_model->wrap_objid[adr + 1];
    return 1;
}

/* Return tendon render width. */
float mj_shim_tendon_width(int i) {
    if (!g_model || i < 0 || i >= g_model->ntendon) return 0.0f;
    return (float)g_model->tendon_width[i];
}

/* Resolve tendon RGBA, with a neutral fallback. */
void mj_shim_tendon_rgba(int i, float *out) {
    if (!out) return;
    out[0] = 0.75f;
    out[1] = 0.82f;
    out[2] = 0.92f;
    out[3] = 1.0f;
    if (!g_model || i < 0 || i >= g_model->ntendon) return;
    const float *rgba = g_model->tendon_rgba + i * 4;
    out[0] = rgba[0];
    out[1] = rgba[1];
    out[2] = rgba[2];
    out[3] = rgba[3];
}

/* Free model and data. */
void mj_shim_free(void) {
    if (g_data) {
        mj_deleteData(g_data);
        g_data = NULL;
    }
    if (g_model) {
        mj_deleteModel(g_model);
        g_model = NULL;
    }
}
