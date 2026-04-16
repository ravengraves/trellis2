/*
 * pollyn_shim.c — C shim for Pollyn Engine
 *
 * Handles GLFW window management, callback-to-buffer bridging,
 * OpenGL ES 2.0 calls, and terminal raw mode.
 *
 * Pika has no function pointer support, so GLFW callbacks write
 * events into a ring buffer that Pika polls each frame.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>

/* Prefer Pika stdlib-generated timing symbols when they are linked in.
 * Weak linkage lets shim-only targets keep building without clock.pka users. */
extern int64_t clock_TimeNS(void) __attribute__((weak));

/* ================================================================
 * MONOTONIC CLOCK (works without GL)
 * ================================================================ */

double shim_clock_ms(void) {
    if (clock_TimeNS) {
        return (double)clock_TimeNS() / 1000000.0;
    }
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0;
}

#include <GLFW/glfw3.h>
#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include "shaders.h"

#define SHIM_MESH_FLOAT_STRIDE 12
#define SHIM_MESH_BYTE_STRIDE 48
#define SHIM_MESH_POS_OFFSET 0
#define SHIM_MESH_NORM_OFFSET 12
#define SHIM_MESH_UV_OFFSET 24
#define SHIM_MESH_TANGENT_OFFSET 32

/* ================================================================
 * EVENT RING BUFFER
 * GLFW callbacks write here; Pika reads via shim_event_poll().
 * ================================================================ */

#define EVT_RING_CAP 256

/* Event types — must match Pika enum */
enum {
    EVT_NONE        = 0,
    EVT_KEY_PRESS   = 1,
    EVT_KEY_RELEASE = 2,
    EVT_CHAR        = 3,
    EVT_MOUSE_PRESS = 4,
    EVT_MOUSE_RELEASE = 5,
    EVT_CURSOR_MOVE = 6,
    EVT_SCROLL      = 7,
    EVT_RESIZE      = 8,
    EVT_CLOSE       = 9,
};

typedef struct {
    int type;
    int key;        /* key code or button or codepoint */
    int mods;       /* modifier bits */
    float x, y;     /* cursor pos or scroll delta or size */
} ShimEvent;

static ShimEvent g_ring[EVT_RING_CAP];
static int       g_ring_head = 0;   /* next write position */
static int       g_ring_tail = 0;   /* next read position */

static void ring_push(ShimEvent e) {
    int next = (g_ring_head + 1) % EVT_RING_CAP;
    if (next == g_ring_tail) return;  /* full — drop event */
    g_ring[g_ring_head] = e;
    g_ring_head = next;
}

/* Pika-callable: poll one event. Returns type (0 = none). */
int shim_event_poll(int *out_key, int *out_mods, float *out_x, float *out_y) {
    if (g_ring_tail == g_ring_head) {
        *out_key = 0; *out_mods = 0; *out_x = 0; *out_y = 0;
        return EVT_NONE;
    }
    ShimEvent e = g_ring[g_ring_tail];
    g_ring_tail = (g_ring_tail + 1) % EVT_RING_CAP;
    *out_key  = e.key;
    *out_mods = e.mods;
    *out_x    = e.x;
    *out_y    = e.y;
    return e.type;
}

/* Pika-callable: flush all events */
void shim_event_clear(void) {
    g_ring_tail = g_ring_head;
}

/* ================================================================
 * TERMINAL RAW MODE (works without GL)
 * ================================================================ */

#ifdef __linux__
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>

static struct termios g_orig_termios;
static int g_raw_mode = 0;

void shim_term_raw_enter(void) {
    if (g_raw_mode) return;
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    struct termios raw = g_orig_termios;
    raw.c_lflag &= (unsigned)~(ECHO | ICANON | ISIG);
    raw.c_iflag &= (unsigned)~(IXON | ICRNL);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    /* Hide cursor, switch to alt screen, enable SGR mouse */
    write(STDOUT_FILENO,
        "\x1b[?25l\x1b[?1049h"
        "\x1b[?1000h\x1b[?1002h\x1b[?1006h", 39);
    g_raw_mode = 1;
}

void shim_term_raw_exit(void) {
    if (!g_raw_mode) return;
    /* Disable SGR mouse, restore screen, show cursor */
    write(STDOUT_FILENO,
        "\x1b[?1006l\x1b[?1002l\x1b[?1000l"
        "\x1b[?1049l\x1b[?25h", 39);
    tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
    g_raw_mode = 0;
}

int shim_term_width(void) {
    struct winsize ws;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0) return ws.ws_col;
    return 80;
}

int shim_term_height(void) {
    struct winsize ws;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0) return ws.ws_row;
    return 24;
}

/* Helper: parse decimal int from buffer, advance index */
static int parse_int(const unsigned char *buf, int n, int *pos) {
    int val = 0;
    while (*pos < n && buf[*pos] >= '0' && buf[*pos] <= '9') {
        val = val * 10 + (buf[*pos] - '0');
        (*pos)++;
    }
    return val;
}

/* Read terminal input bytes, push as events.
 * Handles ESC sequences, SGR mouse, and SSH ESC timing. */
void shim_term_poll_input(void) {
    unsigned char buf[256];
    int n = (int)read(STDIN_FILENO, buf, sizeof(buf));
    if (n <= 0) return;

    int i = 0;
    while (i < n) {
        ShimEvent e = {0};

        if (buf[i] == 27) { /* ESC byte */
            /* If ESC is at end of buffer, wait briefly for more (SSH fix) */
            if (i + 1 >= n) {
                struct pollfd pfd;
                pfd.fd = STDIN_FILENO;
                pfd.events = POLLIN;
                pfd.revents = 0;
                if (poll(&pfd, 1, 25) > 0) {
                    int extra = (int)read(STDIN_FILENO,
                        buf + n, sizeof(buf) - (size_t)n);
                    if (extra > 0) n += extra;
                }
            }

            if (i + 1 < n && buf[i+1] == '[') {
                /* CSI sequence: ESC [ ... */
                if (i + 2 < n && buf[i+2] == '<') {
                    /* SGR mouse: ESC [ < btn ; col ; row M/m */
                    int p = i + 3;
                    int btn = parse_int(buf, n, &p);
                    if (p < n && buf[p] == ';') p++;
                    int col = parse_int(buf, n, &p);
                    if (p < n && buf[p] == ';') p++;
                    int row = parse_int(buf, n, &p);
                    int release = 0;
                    if (p < n) {
                        release = (buf[p] == 'm');
                        p++;
                    }
                    i = p;

                    int base_btn = btn & 3;
                    int is_motion = (btn & 32) != 0;
                    int is_scroll = (btn & 64) != 0;

                    /* Emit cursor position for all mouse events */
                    e.type = EVT_CURSOR_MOVE;
                    e.x = (float)col;
                    e.y = (float)row;
                    ring_push(e);

                    if (is_scroll) {
                        ShimEvent se = {0};
                        se.type = EVT_SCROLL;
                        /* btn 64 = scroll up (+1), btn 65 = scroll down (-1) */
                        se.y = (base_btn == 0) ? 1.0f : -1.0f;
                        ring_push(se);
                    } else if (!is_motion) {
                        /* Button press or release */
                        /* SGR: 0=left, 1=middle, 2=right */
                        /* Pika: MOUSE_LEFT=0, MOUSE_RIGHT=1, MOUSE_MIDDLE=2 */
                        int mapped = base_btn;
                        if (base_btn == 1) mapped = 2; /* middle */
                        else if (base_btn == 2) mapped = 1; /* right */
                        ShimEvent be = {0};
                        be.type = release ? EVT_MOUSE_RELEASE : EVT_MOUSE_PRESS;
                        be.key = mapped;
                        ring_push(be);
                    }
                    /* Drag (is_motion): cursor move already emitted */
                    continue;
                }

                /* Arrow keys: ESC [ A/B/C/D */
                if (i + 2 < n) {
                    int key = 0;
                    switch (buf[i+2]) {
                        case 'A': key = 265; break; /* UP    */
                        case 'B': key = 264; break; /* DOWN  */
                        case 'C': key = 262; break; /* RIGHT */
                        case 'D': key = 263; break; /* LEFT  */
                    }
                    if (key != 0) {
                        e.type = EVT_KEY_PRESS;
                        e.key = key;
                        ring_push(e);
                        i += 3;
                        continue;
                    }
                }

                /* Unknown CSI — skip params and final byte */
                i += 2;
                while (i < n && buf[i] >= 0x20 && buf[i] <= 0x3F) i++;
                if (i < n) i++; /* final byte */
                continue;
            }

            /* Bare ESC (no '[' follows) */
            e.type = EVT_KEY_PRESS;
            e.key = 256; /* ESCAPE */
            ring_push(e);
            i++;
            continue;
        }

        /* Regular bytes */
        if (buf[i] == 3) {
            e.type = EVT_KEY_PRESS;
            e.key  = 1000; /* Ctrl+C */
        } else if (buf[i] == 13 || buf[i] == 10) {
            e.type = EVT_KEY_PRESS;
            e.key  = 257; /* ENTER */
        } else if (buf[i] == 32) {
            e.type = EVT_KEY_PRESS;
            e.key  = 32;  /* SPACE */
        } else if (buf[i] >= 'a' && buf[i] <= 'z') {
            e.type = EVT_KEY_PRESS;
            e.key  = (int)buf[i] - 32; /* to uppercase keycode */
        } else if (buf[i] >= 'A' && buf[i] <= 'Z') {
            e.type = EVT_KEY_PRESS;
            e.key  = (int)buf[i];
            e.mods = 1; /* Shift */
        } else {
            e.type = EVT_CHAR;
            e.key  = (int)buf[i];
        }
        ring_push(e);
        i++;
    }
}

/* Write raw bytes to stdout (for terminal rendering) */
void shim_term_write(unsigned char *data, int len) {
    write(STDOUT_FILENO, data, (size_t)len);
}

void shim_term_flush(void) {
    /* stdout is already unbuffered when writing to fd directly */
}

#else
/* Stubs for non-Linux */
void shim_term_raw_enter(void) {}
void shim_term_raw_exit(void) {}
int  shim_term_width(void)  { return 80; }
int  shim_term_height(void) { return 24; }
void shim_term_poll_input(void) {}
void shim_term_write(unsigned char *data, int len) { (void)data; (void)len; }
void shim_term_flush(void) {}
#endif

/* ================================================================
 * GLFW WINDOW + GL
 * ================================================================ */

static GLFWwindow *g_window = NULL;
static int g_win_w = 0, g_win_h = 0;

/* EGL state for offscreen rendering.
 * API split:
 * - display acquisition: open/close the EGLDisplay
 * - offscreen context creation: choose config + pbuffer + context + MakeCurrent
 * - windowed context creation: still handled by GLFW in shim_window_create()
 * Fallback order: pbuffer first. Surfaceless can be added later as an optimization. */
typedef struct {
    EGLDisplay display;
    EGLContext context;
    EGLSurface surface;
    EGLConfig  config;
    int initialized;
    int refcount;
} ShimEglState;

static ShimEglState g_egl = {
    EGL_NO_DISPLAY,
    EGL_NO_CONTEXT,
    EGL_NO_SURFACE,
    (EGLConfig)0,
    0,
    0,
};

static void shim_egl_reset(ShimEglState *egl) {
    egl->display = EGL_NO_DISPLAY;
    egl->context = EGL_NO_CONTEXT;
    egl->surface = EGL_NO_SURFACE;
    egl->config = (EGLConfig)0;
    egl->initialized = 0;
    egl->refcount = 0;
}

static EGLDisplay shim_egl_get_default_display(void) {
#ifdef __linux__
    if (!getenv("EGL_PLATFORM")) {
        setenv("EGL_PLATFORM", "surfaceless", 0);
    }
#endif

    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (display != EGL_NO_DISPLAY) {
        return display;
    }

#ifdef EGL_MESA_platform_surfaceless
    PFNEGLGETPLATFORMDISPLAYEXTPROC get_platform_display =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    if (get_platform_display) {
        display = get_platform_display(EGL_PLATFORM_SURFACELESS_MESA,
                                       EGL_DEFAULT_DISPLAY, NULL);
        if (display != EGL_NO_DISPLAY) {
            return display;
        }
    }
#endif

    return EGL_NO_DISPLAY;
}

static int shim_egl_display_open(ShimEglState *egl) {
    if (egl->display != EGL_NO_DISPLAY) {
        return 0;
    }

    egl->display = shim_egl_get_default_display();
    if (egl->display == EGL_NO_DISPLAY) {
        return -1;
    }

    EGLint major = 0;
    EGLint minor = 0;
    if (!eglInitialize(egl->display, &major, &minor)) {
        egl->display = EGL_NO_DISPLAY;
        return -1;
    }
    if (!eglBindAPI(EGL_OPENGL_ES_API)) {
        eglTerminate(egl->display);
        egl->display = EGL_NO_DISPLAY;
        return -1;
    }

    egl->initialized = 1;
    return 0;
}

static void shim_egl_display_close(ShimEglState *egl) {
    if (egl->initialized && egl->display != EGL_NO_DISPLAY) {
        eglTerminate(egl->display);
    }
    shim_egl_reset(egl);
}

static int shim_egl_choose_config(ShimEglState *egl, EGLConfig *out_cfg) {
    EGLint cfg_attrs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 16,
        EGL_NONE
    };
    EGLConfig cfg = (EGLConfig)0;
    EGLint num_cfg = 0;
    if (!eglChooseConfig(egl->display, cfg_attrs, &cfg, 1, &num_cfg) || num_cfg == 0) {
        return -1;
    }
    *out_cfg = cfg;
    return 0;
}

static int shim_egl_create_context(ShimEglState *egl, EGLConfig cfg) {
    EGLint ctx_attrs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    egl->context = eglCreateContext(egl->display, cfg, EGL_NO_CONTEXT, ctx_attrs);
    return (egl->context == EGL_NO_CONTEXT) ? -1 : 0;
}

static int shim_egl_create_pbuffer_surface(ShimEglState *egl, EGLConfig cfg,
                                           int width, int height) {
    EGLint pbuffer_attrs[] = {
        EGL_WIDTH, width,
        EGL_HEIGHT, height,
        EGL_NONE
    };
    egl->surface = eglCreatePbufferSurface(egl->display, cfg, pbuffer_attrs);
    return (egl->surface == EGL_NO_SURFACE) ? -1 : 0;
}

static void shim_egl_release_current(ShimEglState *egl) {
    if (egl->display != EGL_NO_DISPLAY) {
        eglMakeCurrent(egl->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }
}

static void shim_egl_context_close(ShimEglState *egl) {
    shim_egl_release_current(egl);
    if (egl->surface != EGL_NO_SURFACE) {
        eglDestroySurface(egl->display, egl->surface);
        egl->surface = EGL_NO_SURFACE;
    }
    if (egl->context != EGL_NO_CONTEXT) {
        eglDestroyContext(egl->display, egl->context);
        egl->context = EGL_NO_CONTEXT;
    }
    egl->config = (EGLConfig)0;
}

static int shim_egl_make_current(ShimEglState *egl) {
    if (!eglMakeCurrent(egl->display, egl->surface, egl->surface, egl->context)) {
        return -1;
    }
    return 0;
}

static int shim_egl_acquire_offscreen(ShimEglState *egl, int width, int height) {
    if (egl->context != EGL_NO_CONTEXT) {
        egl->refcount++;
        return 0;
    }

    if (width < 1) width = 1;
    if (height < 1) height = 1;

    if (shim_egl_display_open(egl) < 0) {
        return -1;
    }
    if (shim_egl_choose_config(egl, &egl->config) < 0) {
        shim_egl_display_close(egl);
        return -1;
    }
    if (shim_egl_create_pbuffer_surface(egl, egl->config, width, height) < 0) {
        shim_egl_display_close(egl);
        return -1;
    }
    if (shim_egl_create_context(egl, egl->config) < 0) {
        shim_egl_context_close(egl);
        shim_egl_display_close(egl);
        return -1;
    }
    if (shim_egl_make_current(egl) < 0) {
        shim_egl_context_close(egl);
        shim_egl_display_close(egl);
        return -1;
    }

    egl->refcount = 1;
    return 0;
}

static void shim_egl_release_offscreen(ShimEglState *egl) {
    if (egl->context == EGL_NO_CONTEXT) {
        return;
    }

    egl->refcount--;
    if (egl->refcount > 0) {
        return;
    }

    shim_egl_context_close(egl);
    shim_egl_display_close(egl);
}

/* --- GLFW Callbacks (write to ring buffer) --- */

static void cb_key(GLFWwindow *w, int key, int scancode, int action, int mods) {
    (void)w; (void)scancode;
    ShimEvent e = {0};
    e.type = (action == GLFW_RELEASE) ? EVT_KEY_RELEASE : EVT_KEY_PRESS;
    e.key  = key;
    e.mods = mods;
    ring_push(e);
}

static void cb_char(GLFWwindow *w, unsigned int codepoint) {
    (void)w;
    ShimEvent e = {0};
    e.type = EVT_CHAR;
    e.key  = (int)codepoint;
    ring_push(e);
}

static void cb_mouse_button(GLFWwindow *w, int button, int action, int mods) {
    (void)w; (void)mods;
    ShimEvent e = {0};
    e.type = (action == GLFW_PRESS) ? EVT_MOUSE_PRESS : EVT_MOUSE_RELEASE;
    e.key  = button;
    double mx, my;
    glfwGetCursorPos(w, &mx, &my);
    e.x = (float)mx;
    e.y = (float)my;
    ring_push(e);
}

static void cb_cursor_pos(GLFWwindow *w, double x, double y) {
    (void)w;
    ShimEvent e = {0};
    e.type = EVT_CURSOR_MOVE;
    e.x = (float)x;
    e.y = (float)y;
    ring_push(e);
}

static void cb_scroll(GLFWwindow *w, double dx, double dy) {
    (void)w;
    ShimEvent e = {0};
    e.type = EVT_SCROLL;
    e.x = (float)dx;
    e.y = (float)dy;
    ring_push(e);
}

static void cb_framebuffer_size(GLFWwindow *w, int width, int height) {
    (void)w;
    ShimEvent e = {0};
    e.type = EVT_RESIZE;
    e.x = (float)width;
    e.y = (float)height;
    g_win_w = width;
    g_win_h = height;
    ring_push(e);
}

static void cb_close(GLFWwindow *w) {
    (void)w;
    ShimEvent e = {0};
    e.type = EVT_CLOSE;
    ring_push(e);
}

/* --- Window management --- */

int shim_window_create(int width, int height, unsigned char *title) {
    if (!glfwInit()) return -1;

    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    g_window = glfwCreateWindow(width, height, (const char *)title, NULL, NULL);
    if (!g_window) { glfwTerminate(); return -1; }

    glfwMakeContextCurrent(g_window);
    glfwSwapInterval(1);

    /* Register all callbacks */
    glfwSetKeyCallback(g_window, cb_key);
    glfwSetCharCallback(g_window, cb_char);
    glfwSetMouseButtonCallback(g_window, cb_mouse_button);
    glfwSetCursorPosCallback(g_window, cb_cursor_pos);
    glfwSetScrollCallback(g_window, cb_scroll);
    glfwSetFramebufferSizeCallback(g_window, cb_framebuffer_size);
    glfwSetWindowCloseCallback(g_window, cb_close);

    glfwGetFramebufferSize(g_window, &g_win_w, &g_win_h);
    return 0;
}

void shim_window_destroy(void) {
    if (g_window) {
        glfwDestroyWindow(g_window);
        g_window = NULL;
        glfwTerminate();
    }
    shim_egl_release_offscreen(&g_egl);
}

int shim_window_should_close(void) {
    return g_window ? glfwWindowShouldClose(g_window) : 1;
}

void shim_window_poll(void) {
    if (g_window) glfwPollEvents();
}

void shim_window_swap(void) {
    if (g_window) glfwSwapBuffers(g_window);
}

int shim_window_width(void) { return g_win_w; }
int shim_window_height(void) { return g_win_h; }

double shim_time(void) {
    if (clock_TimeNS) {
        return (double)clock_TimeNS() / 1000000000.0;
    }
    return glfwGetTime();
}

/* ================================================================
 * GL WRAPPER FUNCTIONS
 * ================================================================ */

void shim_gl_viewport(int x, int y, int w, int h) {
    glViewport(x, y, w, h);
}

void shim_gl_clear(float r, float g, float b, float a) {
    glClearColor(r, g, b, a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void shim_gl_enable_depth(void) {
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
}

void shim_gl_disable_depth(void) {
    glDisable(GL_DEPTH_TEST);
}

void shim_gl_enable_blend(void) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void shim_gl_disable_blend(void) {
    glDisable(GL_BLEND);
}

void shim_gl_line_width(float w) {
    glLineWidth(w);
}

/* --- Shader compilation --- */

unsigned int shim_gl_compile_shader(unsigned char *vert_src, unsigned char *frag_src) {
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    const char *vs_str = (const char *)vert_src;
    glShaderSource(vs, 1, &vs_str, NULL);
    glCompileShader(vs);
    GLint ok = 0;
    glGetShaderiv(vs, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(vs, 512, NULL, log);
        fprintf(stderr, "vertex shader error: %s\n", log);
        glDeleteShader(vs);
        return 0;
    }

    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    const char *fs_str = (const char *)frag_src;
    glShaderSource(fs, 1, &fs_str, NULL);
    glCompileShader(fs);
    glGetShaderiv(fs, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(fs, 512, NULL, log);
        fprintf(stderr, "fragment shader error: %s\n", log);
        glDeleteShader(vs);
        glDeleteShader(fs);
        return 0;
    }

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(prog, 512, NULL, log);
        fprintf(stderr, "shader link error: %s\n", log);
        glDeleteProgram(prog);
        glDeleteShader(vs);
        glDeleteShader(fs);
        return 0;
    }

    glDeleteShader(vs);
    glDeleteShader(fs);
    return (unsigned int)prog;
}

void shim_gl_delete_shader(unsigned int prog) {
    glDeleteProgram(prog);
}

void shim_gl_use_shader(unsigned int prog) {
    glUseProgram(prog);
}

int shim_gl_get_uniform(unsigned int prog, unsigned char *name) {
    return glGetUniformLocation(prog, (const char *)name);
}

int shim_gl_get_attrib(unsigned int prog, unsigned char *name) {
    return glGetAttribLocation(prog, (const char *)name);
}

void shim_gl_uniform_mat4(int loc, float *m) {
    glUniformMatrix4fv(loc, 1, GL_FALSE, m);
}

void shim_gl_uniform_4f(int loc, float x, float y, float z, float w) {
    glUniform4f(loc, x, y, z, w);
}

void shim_gl_uniform_3f(int loc, float x, float y, float z) {
    glUniform3f(loc, x, y, z);
}

void shim_gl_uniform_1f(int loc, float v) {
    glUniform1f(loc, v);
}

void shim_gl_uniform_1i(int loc, int v) {
    glUniform1i(loc, v);
}

static void shim_gl_upload_mvp(GLint loc, const float *mvp) {
    if (loc < 0) {
        return;
    }
    if (g_egl.context != EGL_NO_CONTEXT) {
        /* Normalize offscreen EGL rendering to match the windowed path.
         * On the current headless EGL+pbuffer stack, clip-space X is reflected
         * relative to GLFW window rendering unless we flip the MVP row here. */
        float flipped[16];
        memcpy(flipped, mvp, sizeof(flipped));
        flipped[0] = -flipped[0];
        flipped[4] = -flipped[4];
        flipped[8] = -flipped[8];
        flipped[12] = -flipped[12];
        glUniformMatrix4fv(loc, 1, GL_FALSE, flipped);
        return;
    }
    glUniformMatrix4fv(loc, 1, GL_FALSE, mvp);
}

/* --- Buffer management --- */

unsigned int shim_gl_create_vbo(float *data, int count) {
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(count * (int)sizeof(float)),
                 data, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return vbo;
}

unsigned int shim_gl_create_ibo(unsigned short *data, int count) {
    GLuint ibo;
    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 (GLsizeiptr)(count * (int)sizeof(unsigned short)),
                 data, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    return ibo;
}

static int cmp_wire_edge_key(const void *a, const void *b) {
    uint32_t ka = *(const uint32_t *)a;
    uint32_t kb = *(const uint32_t *)b;
    if (ka < kb) return -1;
    if (ka > kb) return 1;
    return 0;
}

unsigned int shim_gl_create_wire_ibo(unsigned short *data, int count,
                                     int *out_wire_count) {
    if (out_wire_count) *out_wire_count = 0;
    if (!data || count < 3) return 0;

    int tri_count = count / 3;
    int edge_count = tri_count * 3;
    uint32_t *edge_keys = (uint32_t *)malloc((size_t)edge_count * sizeof(uint32_t));
    if (!edge_keys) return 0;

    int key_count = 0;
    for (int tri = 0; tri < tri_count; tri++) {
        int base = tri * 3;
        unsigned short ia = data[base + 0];
        unsigned short ib = data[base + 1];
        unsigned short ic = data[base + 2];
        unsigned short e0a = ia < ib ? ia : ib;
        unsigned short e0b = ia < ib ? ib : ia;
        unsigned short e1a = ib < ic ? ib : ic;
        unsigned short e1b = ib < ic ? ic : ib;
        unsigned short e2a = ic < ia ? ic : ia;
        unsigned short e2b = ic < ia ? ia : ic;
        edge_keys[key_count++] = ((uint32_t)e0a << 16) | (uint32_t)e0b;
        edge_keys[key_count++] = ((uint32_t)e1a << 16) | (uint32_t)e1b;
        edge_keys[key_count++] = ((uint32_t)e2a << 16) | (uint32_t)e2b;
    }

    qsort(edge_keys, (size_t)key_count, sizeof(uint32_t), cmp_wire_edge_key);

    int unique_edges = 0;
    for (int i = 0; i < key_count; i++) {
        if (i == 0 || edge_keys[i] != edge_keys[i - 1]) {
            unique_edges++;
        }
    }
    if (unique_edges <= 0) {
        free(edge_keys);
        return 0;
    }

    unsigned short *wire_indices = (unsigned short *)malloc(
        (size_t)(unique_edges * 2) * sizeof(unsigned short));
    if (!wire_indices) {
        free(edge_keys);
        return 0;
    }

    int wi = 0;
    for (int i = 0; i < key_count; i++) {
        if (i > 0 && edge_keys[i] == edge_keys[i - 1]) continue;
        wire_indices[wi++] = (unsigned short)(edge_keys[i] >> 16);
        wire_indices[wi++] = (unsigned short)(edge_keys[i] & 0xffffu);
    }
    free(edge_keys);

    GLuint ibo;
    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 (GLsizeiptr)(wi * (int)sizeof(unsigned short)),
                 wire_indices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    free(wire_indices);

    if (out_wire_count) *out_wire_count = wi;
    return ibo;
}

void shim_gl_delete_buffer(unsigned int buf) {
    glDeleteBuffers(1, &buf);
}

/* --- Drawing --- */

void shim_gl_draw_triangles(unsigned int vbo, unsigned int ibo,
                            int attr_pos, int attr_norm,
                            int index_count) {
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    /* Position: 3 floats at offset 0, stride 48 (12 floats = pos+normal+uv+tangent) */
    if (attr_pos >= 0) {
        glEnableVertexAttribArray((GLuint)attr_pos);
        glVertexAttribPointer((GLuint)attr_pos, 3, GL_FLOAT, GL_FALSE,
                              SHIM_MESH_BYTE_STRIDE, (void *)SHIM_MESH_POS_OFFSET);
    }
    /* Normal: 3 floats at offset 12 */
    if (attr_norm >= 0) {
        glEnableVertexAttribArray((GLuint)attr_norm);
        glVertexAttribPointer((GLuint)attr_norm, 3, GL_FLOAT, GL_FALSE,
                              SHIM_MESH_BYTE_STRIDE, (void *)(intptr_t)SHIM_MESH_NORM_OFFSET);
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glDrawElements(GL_TRIANGLES, index_count, GL_UNSIGNED_SHORT, NULL);

    if (attr_pos >= 0)  glDisableVertexAttribArray((GLuint)attr_pos);
    if (attr_norm >= 0) glDisableVertexAttribArray((GLuint)attr_norm);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void shim_gl_draw_lines(unsigned int vbo, int attr_pos, int attr_color,
                        int vertex_count) {
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    /* Position: 3 floats, stride 28 (7 floats = pos+color) */
    if (attr_pos >= 0) {
        glEnableVertexAttribArray((GLuint)attr_pos);
        glVertexAttribPointer((GLuint)attr_pos, 3, GL_FLOAT, GL_FALSE,
                              28, (void *)0);
    }
    /* Color: 4 floats at offset 12 */
    if (attr_color >= 0) {
        glEnableVertexAttribArray((GLuint)attr_color);
        glVertexAttribPointer((GLuint)attr_color, 4, GL_FLOAT, GL_FALSE,
                              28, (void *)(intptr_t)12);
    }
    glDrawArrays(GL_LINES, 0, vertex_count);

    if (attr_pos >= 0)   glDisableVertexAttribArray((GLuint)attr_pos);
    if (attr_color >= 0) glDisableVertexAttribArray((GLuint)attr_color);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void shim_gl_draw_points(unsigned int vbo, int attr_pos, int attr_color,
                         int attr_size, int vertex_count) {
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    /* Position: 3 floats, stride 32 (8 floats = pos+color+size) */
    if (attr_pos >= 0) {
        glEnableVertexAttribArray((GLuint)attr_pos);
        glVertexAttribPointer((GLuint)attr_pos, 3, GL_FLOAT, GL_FALSE,
                              32, (void *)0);
    }
    if (attr_color >= 0) {
        glEnableVertexAttribArray((GLuint)attr_color);
        glVertexAttribPointer((GLuint)attr_color, 4, GL_FLOAT, GL_FALSE,
                              32, (void *)(intptr_t)12);
    }
    if (attr_size >= 0) {
        glEnableVertexAttribArray((GLuint)attr_size);
        glVertexAttribPointer((GLuint)attr_size, 1, GL_FLOAT, GL_FALSE,
                              32, (void *)(intptr_t)28);
    }
    glDrawArrays(GL_POINTS, 0, vertex_count);

    if (attr_pos >= 0)   glDisableVertexAttribArray((GLuint)attr_pos);
    if (attr_color >= 0) glDisableVertexAttribArray((GLuint)attr_color);
    if (attr_size >= 0)  glDisableVertexAttribArray((GLuint)attr_size);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

/* --- 2D overlay drawing (no depth, simple quad VBO) --- */

void shim_gl_draw_overlay_tris(unsigned int vbo, int attr_pos, int vertex_count) {
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    if (attr_pos >= 0) {
        glEnableVertexAttribArray((GLuint)attr_pos);
        glVertexAttribPointer((GLuint)attr_pos, 2, GL_FLOAT, GL_FALSE,
                              8, (void *)0);
    }
    glDrawArrays(GL_TRIANGLES, 0, vertex_count);
    if (attr_pos >= 0) glDisableVertexAttribArray((GLuint)attr_pos);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

/* --- Framebuffer for terminal readback --- */

static GLuint g_fbo = 0, g_fbo_color = 0, g_fbo_depth = 0;
static int    g_fbo_w = 0, g_fbo_h = 0;

int shim_gl_fbo_create(int w, int h) {
    g_fbo_w = w; g_fbo_h = h;
    glGenFramebuffers(1, &g_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, g_fbo);

    /* GLES2 requires RGBA for glReadPixels */
    glGenTextures(1, &g_fbo_color);
    glBindTexture(GL_TEXTURE_2D, g_fbo_color);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, g_fbo_color, 0);

    glGenRenderbuffers(1, &g_fbo_depth);
    glBindRenderbuffer(GL_RENDERBUFFER, g_fbo_depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, w, h);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, g_fbo_depth);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return (status == GL_FRAMEBUFFER_COMPLETE) ? 0 : -1;
}

void shim_gl_fbo_bind(void) {
    glBindFramebuffer(GL_FRAMEBUFFER, g_fbo);
    glViewport(0, 0, g_fbo_w, g_fbo_h);
}

void shim_gl_fbo_unbind(void) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void shim_gl_fbo_read_pixels(unsigned char *out_rgb, int w, int h) {
    glBindFramebuffer(GL_FRAMEBUFFER, g_fbo);
    /* GLES2 only supports GL_RGBA readback; convert to RGB */
    unsigned char *tmp = (unsigned char *)malloc((size_t)(w * 4));
    for (int row = 0; row < h; row++) {
        glReadPixels(0, row, w, 1, GL_RGBA, GL_UNSIGNED_BYTE, tmp);
        for (int x = 0; x < w; x++) {
            out_rgb[(row * w + x) * 3 + 0] = tmp[x * 4 + 0];
            out_rgb[(row * w + x) * 3 + 1] = tmp[x * 4 + 1];
            out_rgb[(row * w + x) * 3 + 2] = tmp[x * 4 + 2];
        }
    }
    free(tmp);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void shim_gl_fbo_destroy(void) {
    if (g_fbo_color) { glDeleteTextures(1, &g_fbo_color); g_fbo_color = 0; }
    if (g_fbo_depth) { glDeleteRenderbuffers(1, &g_fbo_depth); g_fbo_depth = 0; }
    if (g_fbo)       { glDeleteFramebuffers(1, &g_fbo); g_fbo = 0; }
}

/* --- Offscreen GL context via EGL pbuffer (portable EGL baseline) --- */

int shim_gl_init_offscreen(void) {
    return shim_egl_acquire_offscreen(&g_egl, 64, 64);
}

/* --- Flipped readback (GL bottom-up → top-down) --- */

void shim_gl_fbo_read_pixels_flipped(unsigned char *out_rgb, int w, int h) {
    glBindFramebuffer(GL_FRAMEBUFFER, g_fbo);
    /* GLES2 only supports GL_RGBA readback; read RGBA and convert to RGB, flipping Y */
    unsigned char *tmp = (unsigned char *)malloc((size_t)(w * 4));
    for (int row = 0; row < h; row++) {
        int gl_row = h - 1 - row;
        glReadPixels(0, gl_row, w, 1, GL_RGBA, GL_UNSIGNED_BYTE, tmp);
        for (int x = 0; x < w; x++) {
            out_rgb[(row * w + x) * 3 + 0] = tmp[x * 4 + 0];
            out_rgb[(row * w + x) * 3 + 1] = tmp[x * 4 + 1];
            out_rgb[(row * w + x) * 3 + 2] = tmp[x * 4 + 2];
        }
    }
    free(tmp);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

/* ================================================================
 * SHADER PROGRAMS — Flat, PBR, Cell, Wire, Line, Text
 * ================================================================ */

static GLuint g_flat_prog = 0, g_pbr_prog = 0, g_cell_prog = 0, g_wire_prog = 0;
static GLuint g_line_prog = 0, g_depth_prog = 0, g_constant_prog = 0;
static GLuint g_text_prog = 0;

/* Flat shader cached locations */
static GLint g_flat_a_pos, g_flat_a_norm, g_flat_a_uv;
static GLint g_flat_u_mvp, g_flat_u_model;
static GLint g_flat_u_light_dir, g_flat_u_light_color, g_flat_u_base_color;
static GLint g_flat_u_use_texture, g_flat_u_albedo_tex;

/* PBR shader cached locations */
static GLint g_pbr_a_pos, g_pbr_a_norm, g_pbr_a_uv, g_pbr_a_tangent;
static GLint g_pbr_u_mvp, g_pbr_u_model;
static GLint g_pbr_u_light_count;
static GLint g_pbr_u_light_pos_type, g_pbr_u_light_dir_data, g_pbr_u_light_color;
static GLint g_pbr_u_eye_pos;
static GLint g_pbr_u_albedo, g_pbr_u_metallic, g_pbr_u_roughness;
static GLint g_pbr_u_env_sky, g_pbr_u_env_ground, g_pbr_u_env_horizon;
static GLint g_pbr_u_env_specular, g_pbr_u_env_intensity;
static GLint g_pbr_u_env_diffuse_mix, g_pbr_u_env_specular_mix;
static GLint g_pbr_u_alpha;
static GLint g_pbr_u_use_texture, g_pbr_u_albedo_tex;
static GLint g_pbr_u_use_orm_texture, g_pbr_u_orm_tex;
static GLint g_pbr_u_use_normal_texture, g_pbr_u_normal_tex;
static GLint g_pbr_u_use_env_global_texture, g_pbr_u_env_global_tex;
static GLint g_pbr_u_use_env_global_blur_texture, g_pbr_u_env_global_blur_tex;
static GLint g_pbr_u_use_env_local_texture, g_pbr_u_env_local_tex;
static GLint g_pbr_u_use_env_local_blur_texture, g_pbr_u_env_local_blur_tex;

/* Cell shader cached locations */
static GLint g_cell_a_pos, g_cell_a_norm, g_cell_a_uv;
static GLint g_cell_u_mvp, g_cell_u_model;
static GLint g_cell_u_light_dir, g_cell_u_light_color, g_cell_u_base_color;
static GLint g_cell_u_use_texture, g_cell_u_albedo_tex;

/* Wire shader cached locations */
static GLint g_wire_a_pos;
static GLint g_wire_u_mvp, g_wire_u_color;

/* Line shader cached locations */
static GLint g_line_a_pos, g_line_a_color;
static GLint g_line_u_mvp;

/* Depth shader cached locations */
static GLint g_depth_a_pos, g_depth_a_uv;
static GLint g_depth_u_mvp, g_depth_u_znear, g_depth_u_zfar;
static GLint g_depth_u_use_texture, g_depth_u_albedo_tex;

/* Constant color shader cached locations */
static GLint g_constant_a_pos, g_constant_a_uv;
static GLint g_constant_u_mvp, g_constant_u_color;
static GLint g_constant_u_use_texture, g_constant_u_albedo_tex;

/* Text shader cached locations + resources */
static GLint g_text_a_pos, g_text_a_uv;
static GLint g_text_u_screen, g_text_u_color, g_text_u_font;
static GLuint g_text_vbo = 0;
static GLuint g_text_font_tex = 0;
static int g_text_font_ready = 0;
static int g_text_font_first = 32;
static int g_text_font_last = 126;
static int g_text_glyph_w = 8;
static int g_text_glyph_h = 8;
static int g_text_tex_w = 0;
static int g_text_tex_h = 0;

static GLuint compile_prog(const char *vs_src, const char *fs_src) {
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vs_src, NULL);
    glCompileShader(vs);
    GLint ok = 0;
    glGetShaderiv(vs, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(vs, 512, NULL, log);
        fprintf(stderr, "vertex shader error: %s\n", log);
        glDeleteShader(vs);
        return 0;
    }
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fs_src, NULL);
    glCompileShader(fs);
    glGetShaderiv(fs, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(fs, 512, NULL, log);
        fprintf(stderr, "fragment shader error: %s\n", log);
        glDeleteShader(vs); glDeleteShader(fs);
        return 0;
    }
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(prog, 512, NULL, log);
        fprintf(stderr, "shader link error: %s\n", log);
        glDeleteProgram(prog); glDeleteShader(vs); glDeleteShader(fs);
        return 0;
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

int shim_gl_init_shaders(unsigned char *flat_vs, unsigned char *flat_fs,
                         unsigned char *pbr_vs, unsigned char *pbr_fs,
                         unsigned char *cell_vs, unsigned char *cell_fs,
                         unsigned char *wire_vs, unsigned char *wire_fs,
                         unsigned char *line_vs, unsigned char *line_fs,
                         unsigned char *depth_vs, unsigned char *depth_fs,
                         unsigned char *constant_vs, unsigned char *constant_fs,
                         unsigned char *text_vs, unsigned char *text_fs) {
    if (!flat_vs || !flat_fs || !pbr_vs || !pbr_fs ||
        !cell_vs || !cell_fs ||
        !wire_vs || !wire_fs ||
        !line_vs || !line_fs || !depth_vs || !depth_fs ||
        !constant_vs || !constant_fs ||
        !text_vs || !text_fs) {
        return -1;
    }

    g_flat_prog = compile_prog((const char *)flat_vs, (const char *)flat_fs);
    if (!g_flat_prog) return -1;
    g_flat_a_pos      = glGetAttribLocation(g_flat_prog, "a_pos");
    g_flat_a_norm     = glGetAttribLocation(g_flat_prog, "a_norm");
    g_flat_a_uv       = glGetAttribLocation(g_flat_prog, "a_uv");
    g_flat_u_mvp      = glGetUniformLocation(g_flat_prog, "u_mvp");
    g_flat_u_model    = glGetUniformLocation(g_flat_prog, "u_model");
    g_flat_u_light_dir   = glGetUniformLocation(g_flat_prog, "u_light_dir");
    g_flat_u_light_color = glGetUniformLocation(g_flat_prog, "u_light_color");
    g_flat_u_base_color  = glGetUniformLocation(g_flat_prog, "u_base_color");
    g_flat_u_use_texture = glGetUniformLocation(g_flat_prog, "u_use_texture");
    g_flat_u_albedo_tex  = glGetUniformLocation(g_flat_prog, "u_albedo_tex");

    g_pbr_prog = compile_prog((const char *)pbr_vs, (const char *)pbr_fs);
    if (!g_pbr_prog) return -1;
    g_pbr_a_pos       = glGetAttribLocation(g_pbr_prog, "a_pos");
    g_pbr_a_norm      = glGetAttribLocation(g_pbr_prog, "a_norm");
    g_pbr_a_uv        = glGetAttribLocation(g_pbr_prog, "a_uv");
    g_pbr_a_tangent   = glGetAttribLocation(g_pbr_prog, "a_tangent");
    g_pbr_u_mvp       = glGetUniformLocation(g_pbr_prog, "u_mvp");
    g_pbr_u_model     = glGetUniformLocation(g_pbr_prog, "u_model");
    g_pbr_u_light_count  = glGetUniformLocation(g_pbr_prog, "u_light_count");
    g_pbr_u_light_pos_type = glGetUniformLocation(g_pbr_prog, "u_light_pos_type[0]");
    g_pbr_u_light_dir_data = glGetUniformLocation(g_pbr_prog, "u_light_dir_data[0]");
    g_pbr_u_light_color    = glGetUniformLocation(g_pbr_prog, "u_light_color[0]");
    g_pbr_u_eye_pos      = glGetUniformLocation(g_pbr_prog, "u_eye_pos");
    g_pbr_u_albedo       = glGetUniformLocation(g_pbr_prog, "u_albedo");
    g_pbr_u_metallic     = glGetUniformLocation(g_pbr_prog, "u_metallic");
    g_pbr_u_roughness    = glGetUniformLocation(g_pbr_prog, "u_roughness");
    g_pbr_u_env_sky      = glGetUniformLocation(g_pbr_prog, "u_env_sky");
    g_pbr_u_env_ground   = glGetUniformLocation(g_pbr_prog, "u_env_ground");
    g_pbr_u_env_horizon  = glGetUniformLocation(g_pbr_prog, "u_env_horizon");
    g_pbr_u_env_specular = glGetUniformLocation(g_pbr_prog, "u_env_specular");
    g_pbr_u_env_intensity= glGetUniformLocation(g_pbr_prog, "u_env_intensity");
    g_pbr_u_env_diffuse_mix = glGetUniformLocation(g_pbr_prog, "u_env_diffuse_mix");
    g_pbr_u_env_specular_mix = glGetUniformLocation(g_pbr_prog, "u_env_specular_mix");
    g_pbr_u_alpha        = glGetUniformLocation(g_pbr_prog, "u_alpha");
    g_pbr_u_use_texture  = glGetUniformLocation(g_pbr_prog, "u_use_texture");
    g_pbr_u_albedo_tex   = glGetUniformLocation(g_pbr_prog, "u_albedo_tex");
    g_pbr_u_use_orm_texture = glGetUniformLocation(g_pbr_prog, "u_use_orm_texture");
    g_pbr_u_orm_tex      = glGetUniformLocation(g_pbr_prog, "u_orm_tex");
    g_pbr_u_use_normal_texture = glGetUniformLocation(g_pbr_prog, "u_use_normal_texture");
    g_pbr_u_normal_tex   = glGetUniformLocation(g_pbr_prog, "u_normal_tex");
    g_pbr_u_use_env_global_texture =
        glGetUniformLocation(g_pbr_prog, "u_use_env_global_texture");
    g_pbr_u_env_global_tex =
        glGetUniformLocation(g_pbr_prog, "u_env_global_tex");
    g_pbr_u_use_env_global_blur_texture =
        glGetUniformLocation(g_pbr_prog, "u_use_env_global_blur_texture");
    g_pbr_u_env_global_blur_tex =
        glGetUniformLocation(g_pbr_prog, "u_env_global_blur_tex");
    g_pbr_u_use_env_local_texture =
        glGetUniformLocation(g_pbr_prog, "u_use_env_local_texture");
    g_pbr_u_env_local_tex =
        glGetUniformLocation(g_pbr_prog, "u_env_local_tex");
    g_pbr_u_use_env_local_blur_texture =
        glGetUniformLocation(g_pbr_prog, "u_use_env_local_blur_texture");
    g_pbr_u_env_local_blur_tex =
        glGetUniformLocation(g_pbr_prog, "u_env_local_blur_tex");

    g_cell_prog = compile_prog((const char *)cell_vs, (const char *)cell_fs);
    if (!g_cell_prog) return -1;
    g_cell_a_pos      = glGetAttribLocation(g_cell_prog, "a_pos");
    g_cell_a_norm     = glGetAttribLocation(g_cell_prog, "a_norm");
    g_cell_a_uv       = glGetAttribLocation(g_cell_prog, "a_uv");
    g_cell_u_mvp      = glGetUniformLocation(g_cell_prog, "u_mvp");
    g_cell_u_model    = glGetUniformLocation(g_cell_prog, "u_model");
    g_cell_u_light_dir   = glGetUniformLocation(g_cell_prog, "u_light_dir");
    g_cell_u_light_color = glGetUniformLocation(g_cell_prog, "u_light_color");
    g_cell_u_base_color  = glGetUniformLocation(g_cell_prog, "u_base_color");
    g_cell_u_use_texture = glGetUniformLocation(g_cell_prog, "u_use_texture");
    g_cell_u_albedo_tex  = glGetUniformLocation(g_cell_prog, "u_albedo_tex");

    g_wire_prog = compile_prog((const char *)wire_vs, (const char *)wire_fs);
    if (!g_wire_prog) return -1;
    g_wire_a_pos = glGetAttribLocation(g_wire_prog, "a_pos");
    g_wire_u_mvp = glGetUniformLocation(g_wire_prog, "u_mvp");
    g_wire_u_color = glGetUniformLocation(g_wire_prog, "u_color");

    g_line_prog = compile_prog((const char *)line_vs, (const char *)line_fs);
    if (!g_line_prog) return -1;
    g_line_a_pos   = glGetAttribLocation(g_line_prog, "a_pos");
    g_line_a_color = glGetAttribLocation(g_line_prog, "a_color");
    g_line_u_mvp   = glGetUniformLocation(g_line_prog, "u_mvp");

    g_depth_prog = compile_prog((const char *)depth_vs, (const char *)depth_fs);
    if (!g_depth_prog) return -1;
    g_depth_a_pos  = glGetAttribLocation(g_depth_prog, "a_pos");
    g_depth_a_uv   = glGetAttribLocation(g_depth_prog, "a_uv");
    g_depth_u_mvp  = glGetUniformLocation(g_depth_prog, "u_mvp");
    g_depth_u_znear = glGetUniformLocation(g_depth_prog, "u_znear");
    g_depth_u_zfar  = glGetUniformLocation(g_depth_prog, "u_zfar");
    g_depth_u_use_texture = glGetUniformLocation(g_depth_prog, "u_use_texture");
    g_depth_u_albedo_tex  = glGetUniformLocation(g_depth_prog, "u_albedo_tex");

    g_constant_prog = compile_prog((const char *)constant_vs, (const char *)constant_fs);
    if (!g_constant_prog) return -1;
    g_constant_a_pos = glGetAttribLocation(g_constant_prog, "a_pos");
    g_constant_a_uv = glGetAttribLocation(g_constant_prog, "a_uv");
    g_constant_u_mvp = glGetUniformLocation(g_constant_prog, "u_mvp");
    g_constant_u_color = glGetUniformLocation(g_constant_prog, "u_color");
    g_constant_u_use_texture = glGetUniformLocation(g_constant_prog, "u_use_texture");
    g_constant_u_albedo_tex = glGetUniformLocation(g_constant_prog, "u_albedo_tex");

    g_text_prog = compile_prog((const char *)text_vs, (const char *)text_fs);
    if (!g_text_prog) return -1;
    g_text_a_pos   = glGetAttribLocation(g_text_prog, "a_pos");
    g_text_a_uv    = glGetAttribLocation(g_text_prog, "a_uv");
    g_text_u_screen= glGetUniformLocation(g_text_prog, "u_screen");
    g_text_u_color = glGetUniformLocation(g_text_prog, "u_color");
    g_text_u_font  = glGetUniformLocation(g_text_prog, "u_font");
    if (!g_text_vbo) {
        glGenBuffers(1, &g_text_vbo);
    }

    return 0;
}

int shim_gl_init_embedded_shaders(void) {
    return shim_gl_init_shaders(
        (unsigned char *)FLAT_VS, (unsigned char *)FLAT_FS,
        (unsigned char *)PBR_VS, (unsigned char *)PBR_FS,
        (unsigned char *)CELL_VS, (unsigned char *)CELL_FS,
        (unsigned char *)WIRE_VS, (unsigned char *)WIRE_FS,
        (unsigned char *)LINE_VS, (unsigned char *)LINE_FS,
        (unsigned char *)DEPTH_VS, (unsigned char *)DEPTH_FS,
        (unsigned char *)CONSTANT_VS, (unsigned char *)CONSTANT_FS,
        (unsigned char *)TEXT_VS, (unsigned char *)TEXT_FS);
}

int shim_gl_text_set_font(unsigned char *font_rows, int row_count,
                          int glyph_w, int glyph_h,
                          int first_cp, int last_cp) {
    if (g_text_font_ready) return 0;
    if (!font_rows || row_count <= 0 || glyph_w <= 0 || glyph_h <= 0) return -1;
    if (first_cp < 0 || last_cp < first_cp) return -1;

    int glyph_count = last_cp - first_cp + 1;
    int expected_rows = glyph_count * glyph_h;
    if (row_count < expected_rows) return -1;

    int chars_per_row = 16;
    int rows = (glyph_count + chars_per_row - 1) / chars_per_row;
    int tex_w = chars_per_row * glyph_w;
    int tex_h = rows * glyph_h;

    unsigned char *pix = (unsigned char *)calloc((size_t)tex_w * (size_t)tex_h, 1);
    if (!pix) return -1;

    for (int gi = 0; gi < glyph_count; gi++) {
        int gx = (gi % chars_per_row) * glyph_w;
        int gy = (gi / chars_per_row) * glyph_h;
        int row_base = gi * glyph_h;
        for (int y = 0; y < glyph_h; y++) {
            unsigned char bits = font_rows[row_base + y];
            for (int x = 0; x < glyph_w && x < 8; x++) {
                if (bits & (0x80u >> x)) {
                    pix[(gy + y) * tex_w + (gx + x)] = 255;
                }
            }
        }
    }

    if (!g_text_font_tex) {
        glGenTextures(1, &g_text_font_tex);
    }
    glBindTexture(GL_TEXTURE_2D, g_text_font_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
                 tex_w, tex_h, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, pix);
    free(pix);

    g_text_font_first = first_cp;
    g_text_font_last = last_cp;
    g_text_glyph_w = glyph_w;
    g_text_glyph_h = glyph_h;
    g_text_tex_w = tex_w;
    g_text_tex_h = tex_h;
    g_text_font_ready = 1;
    return 0;
}

/* ================================================================
 * HIGH-LEVEL DRAW FUNCTIONS
 * ================================================================ */

void shim_gl_begin_flat(float *mvp, float *model,
                        float ldx, float ldy, float ldz,
                        float lr, float lg, float lb,
                        float cr, float cg, float cb, float ca) {
    glUseProgram(g_flat_prog);
    shim_gl_upload_mvp(g_flat_u_mvp, mvp);
    glUniformMatrix4fv(g_flat_u_model, 1, GL_FALSE, model);
    glUniform3f(g_flat_u_light_dir, ldx, ldy, ldz);
    glUniform3f(g_flat_u_light_color, lr, lg, lb);
    glUniform4f(g_flat_u_base_color, cr, cg, cb, ca);
}

void shim_gl_set_pbr_lights(float *light_data, int light_count) {
    GLfloat pos_type[16] = {0};
    GLfloat dir_data[16] = {0};
    GLfloat light_color[16] = {0};
    GLint cur_prog = 0;
    int count = light_count;

    glGetIntegerv(GL_CURRENT_PROGRAM, &cur_prog);
    if ((GLuint)cur_prog != g_pbr_prog) {
        return;
    }
    if (!light_data) {
        count = 0;
    }
    if (count < 0) count = 0;
    if (count > 4) count = 4;

    for (int i = 0; i < count; ++i) {
        int src = i * 12;
        int dst = i * 4;
        pos_type[dst + 0] = light_data[src + 0];
        pos_type[dst + 1] = light_data[src + 1];
        pos_type[dst + 2] = light_data[src + 2];
        pos_type[dst + 3] = light_data[src + 3];
        dir_data[dst + 0] = light_data[src + 4];
        dir_data[dst + 1] = light_data[src + 5];
        dir_data[dst + 2] = light_data[src + 6];
        dir_data[dst + 3] = light_data[src + 7];
        light_color[dst + 0] = light_data[src + 8];
        light_color[dst + 1] = light_data[src + 9];
        light_color[dst + 2] = light_data[src + 10];
        light_color[dst + 3] = light_data[src + 11];
    }

    if (g_pbr_u_light_count >= 0) {
        glUniform1i(g_pbr_u_light_count, count);
    }
    if (g_pbr_u_light_pos_type >= 0) {
        glUniform4fv(g_pbr_u_light_pos_type, 4, pos_type);
    }
    if (g_pbr_u_light_dir_data >= 0) {
        glUniform4fv(g_pbr_u_light_dir_data, 4, dir_data);
    }
    if (g_pbr_u_light_color >= 0) {
        glUniform4fv(g_pbr_u_light_color, 4, light_color);
    }
}

void shim_gl_begin_pbr(float *mvp, float *model,
                       float ldx, float ldy, float ldz,
                       float lr, float lg, float lb,
                       float ex, float ey, float ez,
                       float ar, float ag, float ab,
                       float metallic, float roughness,
                       float *env, float alpha) {
    (void)ldx;
    (void)ldy;
    (void)ldz;
    (void)lr;
    (void)lg;
    (void)lb;
    glUseProgram(g_pbr_prog);
    shim_gl_upload_mvp(g_pbr_u_mvp, mvp);
    glUniformMatrix4fv(g_pbr_u_model, 1, GL_FALSE, model);
    glUniform3f(g_pbr_u_eye_pos, ex, ey, ez);
    glUniform3f(g_pbr_u_albedo, ar, ag, ab);
    glUniform1f(g_pbr_u_metallic, metallic);
    glUniform1f(g_pbr_u_roughness, roughness);
    /* env: sky3 ground3 horizon3 spec3 intensity diffuse_mix specular_mix */
    glUniform3f(g_pbr_u_env_sky,      env[0], env[1], env[2]);
    glUniform3f(g_pbr_u_env_ground,   env[3], env[4], env[5]);
    glUniform3f(g_pbr_u_env_horizon,  env[6], env[7], env[8]);
    glUniform3f(g_pbr_u_env_specular, env[9], env[10], env[11]);
    glUniform1f(g_pbr_u_env_intensity, env[12]);
    glUniform1f(g_pbr_u_env_diffuse_mix, env[13]);
    glUniform1f(g_pbr_u_env_specular_mix, env[14]);
    glUniform1f(g_pbr_u_alpha, alpha);
    if (g_pbr_u_light_count >= 0) {
        glUniform1i(g_pbr_u_light_count, 0);
    }
}

void shim_gl_begin_cell(float *mvp, float *model,
                        float ldx, float ldy, float ldz,
                        float lr, float lg, float lb,
                        float cr, float cg, float cb, float ca) {
    glUseProgram(g_cell_prog);
    shim_gl_upload_mvp(g_cell_u_mvp, mvp);
    glUniformMatrix4fv(g_cell_u_model, 1, GL_FALSE, model);
    glUniform3f(g_cell_u_light_dir, ldx, ldy, ldz);
    glUniform3f(g_cell_u_light_color, lr, lg, lb);
    glUniform4f(g_cell_u_base_color, cr, cg, cb, ca);
}

void shim_gl_begin_wire(float *mvp, float cr, float cg, float cb, float ca) {
    glUseProgram(g_wire_prog);
    shim_gl_upload_mvp(g_wire_u_mvp, mvp);
    glUniform4f(g_wire_u_color, cr, cg, cb, ca);
}

void shim_gl_begin_constant(float *mvp,
                            float cr, float cg, float cb, float ca) {
    glUseProgram(g_constant_prog);
    shim_gl_upload_mvp(g_constant_u_mvp, mvp);
    glUniform4f(g_constant_u_color, cr, cg, cb, ca);
}

void shim_gl_set_pbr_ibl_textures(unsigned int global_tex,
                                  unsigned int global_blur_tex,
                                  unsigned int local_tex,
                                  unsigned int local_blur_tex) {
    GLint cur_prog = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &cur_prog);
    if ((GLuint)cur_prog != g_pbr_prog) {
        return;
    }

    glActiveTexture(GL_TEXTURE3);
    if (global_tex != 0) {
        glBindTexture(GL_TEXTURE_2D, (GLuint)global_tex);
        if (g_pbr_u_env_global_tex >= 0) glUniform1i(g_pbr_u_env_global_tex, 3);
        if (g_pbr_u_use_env_global_texture >= 0) {
            glUniform1i(g_pbr_u_use_env_global_texture, 1);
        }
    } else {
        glBindTexture(GL_TEXTURE_2D, 0);
        if (g_pbr_u_use_env_global_texture >= 0) {
            glUniform1i(g_pbr_u_use_env_global_texture, 0);
        }
    }

    glActiveTexture(GL_TEXTURE4);
    if (global_blur_tex != 0) {
        glBindTexture(GL_TEXTURE_2D, (GLuint)global_blur_tex);
        if (g_pbr_u_env_global_blur_tex >= 0) {
            glUniform1i(g_pbr_u_env_global_blur_tex, 4);
        }
        if (g_pbr_u_use_env_global_blur_texture >= 0) {
            glUniform1i(g_pbr_u_use_env_global_blur_texture, 1);
        }
    } else {
        glBindTexture(GL_TEXTURE_2D, 0);
        if (g_pbr_u_use_env_global_blur_texture >= 0) {
            glUniform1i(g_pbr_u_use_env_global_blur_texture, 0);
        }
    }

    glActiveTexture(GL_TEXTURE5);
    if (local_tex != 0) {
        glBindTexture(GL_TEXTURE_2D, (GLuint)local_tex);
        if (g_pbr_u_env_local_tex >= 0) glUniform1i(g_pbr_u_env_local_tex, 5);
        if (g_pbr_u_use_env_local_texture >= 0) {
            glUniform1i(g_pbr_u_use_env_local_texture, 1);
        }
    } else {
        glBindTexture(GL_TEXTURE_2D, 0);
        if (g_pbr_u_use_env_local_texture >= 0) {
            glUniform1i(g_pbr_u_use_env_local_texture, 0);
        }
    }

    glActiveTexture(GL_TEXTURE6);
    if (local_blur_tex != 0) {
        glBindTexture(GL_TEXTURE_2D, (GLuint)local_blur_tex);
        if (g_pbr_u_env_local_blur_tex >= 0) {
            glUniform1i(g_pbr_u_env_local_blur_tex, 6);
        }
        if (g_pbr_u_use_env_local_blur_texture >= 0) {
            glUniform1i(g_pbr_u_use_env_local_blur_texture, 1);
        }
    } else {
        glBindTexture(GL_TEXTURE_2D, 0);
        if (g_pbr_u_use_env_local_blur_texture >= 0) {
            glUniform1i(g_pbr_u_use_env_local_blur_texture, 0);
        }
    }

    glActiveTexture(GL_TEXTURE0);
}

void shim_gl_set_material_textures(unsigned int albedo_tex, unsigned int orm_tex,
                                   unsigned int normal_tex) {
    GLint cur_prog = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &cur_prog);
    GLint u_use_texture = -1, u_albedo_tex = -1;
    GLint u_use_orm_texture = -1, u_orm_tex = -1;
    GLint u_use_normal_texture = -1, u_normal_tex = -1;
    int is_pbr = 0;
    if ((GLuint)cur_prog == g_pbr_prog) {
        u_use_texture = g_pbr_u_use_texture;
        u_albedo_tex = g_pbr_u_albedo_tex;
        u_use_orm_texture = g_pbr_u_use_orm_texture;
        u_orm_tex = g_pbr_u_orm_tex;
        u_use_normal_texture = g_pbr_u_use_normal_texture;
        u_normal_tex = g_pbr_u_normal_tex;
        is_pbr = 1;
    } else if ((GLuint)cur_prog == g_cell_prog) {
        u_use_texture = g_cell_u_use_texture;
        u_albedo_tex = g_cell_u_albedo_tex;
    } else if ((GLuint)cur_prog == g_flat_prog) {
        u_use_texture = g_flat_u_use_texture;
        u_albedo_tex = g_flat_u_albedo_tex;
    } else if ((GLuint)cur_prog == g_depth_prog) {
        u_use_texture = g_depth_u_use_texture;
        u_albedo_tex = g_depth_u_albedo_tex;
    } else if ((GLuint)cur_prog == g_constant_prog) {
        u_use_texture = g_constant_u_use_texture;
        u_albedo_tex = g_constant_u_albedo_tex;
    } else {
        return;
    }
    glActiveTexture(GL_TEXTURE0);
    if (albedo_tex != 0) {
        glBindTexture(GL_TEXTURE_2D, (GLuint)albedo_tex);
        if (u_albedo_tex >= 0) glUniform1i(u_albedo_tex, 0);
        if (u_use_texture >= 0) glUniform1i(u_use_texture, 1);
    } else {
        glBindTexture(GL_TEXTURE_2D, 0);
        if (u_use_texture >= 0) glUniform1i(u_use_texture, 0);
    }

    if (is_pbr) {
        glActiveTexture(GL_TEXTURE1);
        if (orm_tex != 0) {
            glBindTexture(GL_TEXTURE_2D, (GLuint)orm_tex);
            if (u_orm_tex >= 0) glUniform1i(u_orm_tex, 1);
            if (u_use_orm_texture >= 0) glUniform1i(u_use_orm_texture, 1);
        } else {
            glBindTexture(GL_TEXTURE_2D, 0);
            if (u_use_orm_texture >= 0) glUniform1i(u_use_orm_texture, 0);
        }
        glActiveTexture(GL_TEXTURE2);
        if (normal_tex != 0) {
            glBindTexture(GL_TEXTURE_2D, (GLuint)normal_tex);
            if (u_normal_tex >= 0) glUniform1i(u_normal_tex, 2);
            if (u_use_normal_texture >= 0) glUniform1i(u_use_normal_texture, 1);
        } else {
            glBindTexture(GL_TEXTURE_2D, 0);
            if (u_use_normal_texture >= 0) glUniform1i(u_use_normal_texture, 0);
        }
        glActiveTexture(GL_TEXTURE0);
    }
}

void shim_gl_set_albedo_texture(unsigned int tex) {
    shim_gl_set_material_textures(tex, 0, 0);
}

void shim_gl_begin_line(float *mvp) {
    glUseProgram(g_line_prog);
    shim_gl_upload_mvp(g_line_u_mvp, mvp);
}

void shim_gl_draw_mesh_handles(unsigned int vbo, unsigned int ibo,
                               int index_count) {
    GLint cur_prog = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &cur_prog);
    GLint a_pos, a_norm, a_uv, a_tangent;
    if ((GLuint)cur_prog == g_depth_prog) {
        a_pos = g_depth_a_pos;
        a_norm = -1;
        a_uv = g_depth_a_uv;
        a_tangent = -1;
    } else if ((GLuint)cur_prog == g_constant_prog) {
        a_pos = g_constant_a_pos;
        a_norm = -1;
        a_uv = g_constant_a_uv;
        a_tangent = -1;
    } else if ((GLuint)cur_prog == g_pbr_prog) {
        a_pos = g_pbr_a_pos;
        a_norm = g_pbr_a_norm;
        a_uv = g_pbr_a_uv;
        a_tangent = g_pbr_a_tangent;
    } else if ((GLuint)cur_prog == g_cell_prog) {
        a_pos = g_cell_a_pos;
        a_norm = g_cell_a_norm;
        a_uv = g_cell_a_uv;
        a_tangent = -1;
    } else {
        a_pos = g_flat_a_pos;
        a_norm = g_flat_a_norm;
        a_uv = g_flat_a_uv;
        a_tangent = -1;
    }

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

    if (a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)a_pos);
        glVertexAttribPointer((GLuint)a_pos, 3, GL_FLOAT, GL_FALSE,
                              SHIM_MESH_BYTE_STRIDE, (void *)SHIM_MESH_POS_OFFSET);
    }
    if (a_norm >= 0) {
        glEnableVertexAttribArray((GLuint)a_norm);
        glVertexAttribPointer((GLuint)a_norm, 3, GL_FLOAT, GL_FALSE,
                              SHIM_MESH_BYTE_STRIDE, (void *)(intptr_t)SHIM_MESH_NORM_OFFSET);
    }
    if (a_uv >= 0) {
        glEnableVertexAttribArray((GLuint)a_uv);
        glVertexAttribPointer((GLuint)a_uv, 2, GL_FLOAT, GL_FALSE,
                              SHIM_MESH_BYTE_STRIDE, (void *)(intptr_t)SHIM_MESH_UV_OFFSET);
    }
    if (a_tangent >= 0) {
        glEnableVertexAttribArray((GLuint)a_tangent);
        glVertexAttribPointer((GLuint)a_tangent, 4, GL_FLOAT, GL_FALSE,
                              SHIM_MESH_BYTE_STRIDE, (void *)(intptr_t)SHIM_MESH_TANGENT_OFFSET);
    }

    glDrawElements(GL_TRIANGLES, index_count, GL_UNSIGNED_SHORT, NULL);

    if (a_pos >= 0)  glDisableVertexAttribArray((GLuint)a_pos);
    if (a_norm >= 0) glDisableVertexAttribArray((GLuint)a_norm);
    if (a_uv >= 0)   glDisableVertexAttribArray((GLuint)a_uv);
    if (a_tangent >= 0) glDisableVertexAttribArray((GLuint)a_tangent);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void shim_gl_draw_mesh_wire_handles(unsigned int vbo, unsigned int wire_ibo,
                                    int wire_index_count) {
    if (!g_wire_prog || !vbo || !wire_ibo || wire_index_count <= 0) return;

    GLboolean had_cull = glIsEnabled(GL_CULL_FACE);
    GLint old_depth_func = GL_LESS;
    glGetIntegerv(GL_DEPTH_FUNC, &old_depth_func);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, wire_ibo);

    if (g_wire_a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)g_wire_a_pos);
        glVertexAttribPointer((GLuint)g_wire_a_pos, 3, GL_FLOAT, GL_FALSE,
                              SHIM_MESH_BYTE_STRIDE, (void *)SHIM_MESH_POS_OFFSET);
    }

    if (had_cull) glDisable(GL_CULL_FACE);
    glDepthFunc(GL_LEQUAL);
    glLineWidth(1.0f);
    glDrawElements(GL_LINES, wire_index_count, GL_UNSIGNED_SHORT, NULL);
    glDepthFunc((GLenum)old_depth_func);
    if (had_cull) glEnable(GL_CULL_FACE);

    if (g_wire_a_pos >= 0) glDisableVertexAttribArray((GLuint)g_wire_a_pos);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

/* primitive: 1=lines, 2=points, 3=triangles */
void shim_gl_draw_color_handles(unsigned int vbo, int vertex_count,
                                int primitive, float width) {
    GLenum mode = GL_LINES;
    if (primitive == 2) mode = GL_POINTS;
    else if (primitive == 3) mode = GL_TRIANGLES;

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    if (g_line_a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)g_line_a_pos);
        glVertexAttribPointer((GLuint)g_line_a_pos, 3, GL_FLOAT, GL_FALSE, 28,
                              (void *)0);
    }
    if (g_line_a_color >= 0) {
        glEnableVertexAttribArray((GLuint)g_line_a_color);
        glVertexAttribPointer((GLuint)g_line_a_color, 4, GL_FLOAT, GL_FALSE, 28,
                              (void *)(intptr_t)12);
    }

    if (mode == GL_LINES) {
        if (width < 1.0f) width = 1.0f;
        glLineWidth(width);
    }

    glDrawArrays(mode, 0, vertex_count);

    if (mode == GL_LINES) glLineWidth(1.0f);
    if (g_line_a_pos >= 0)   glDisableVertexAttribArray((GLuint)g_line_a_pos);
    if (g_line_a_color >= 0) glDisableVertexAttribArray((GLuint)g_line_a_color);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void shim_gl_draw_mesh(float *verts, int vert_floats,
                       unsigned short *indices, int index_count) {
    /* Determine which program is active to get correct attrib locs */
    GLint cur_prog = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &cur_prog);
    GLint a_pos, a_norm, a_uv, a_tangent;
    if ((GLuint)cur_prog == g_depth_prog) {
        a_pos = g_depth_a_pos;
        a_norm = -1; /* depth shader has no normal attribute */
        a_uv = g_depth_a_uv;
        a_tangent = -1;
    } else if ((GLuint)cur_prog == g_constant_prog) {
        a_pos = g_constant_a_pos;
        a_norm = -1; /* constant color shader only needs positions + UVs for alpha mask */
        a_uv = g_constant_a_uv;
        a_tangent = -1;
    } else if ((GLuint)cur_prog == g_pbr_prog) {
        a_pos = g_pbr_a_pos;
        a_norm = g_pbr_a_norm;
        a_uv = g_pbr_a_uv;
        a_tangent = g_pbr_a_tangent;
    } else {
        a_pos = g_flat_a_pos;
        a_norm = g_flat_a_norm;
        a_uv = g_flat_a_uv;
        a_tangent = -1;
    }

    GLuint vbo, ibo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(vert_floats * (int)sizeof(float)),
                 verts, GL_STREAM_DRAW);

    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 (GLsizeiptr)(index_count * (int)sizeof(unsigned short)),
                 indices, GL_STREAM_DRAW);

    if (a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)a_pos);
        glVertexAttribPointer((GLuint)a_pos, 3, GL_FLOAT, GL_FALSE,
                              SHIM_MESH_BYTE_STRIDE, (void *)SHIM_MESH_POS_OFFSET);
    }
    if (a_norm >= 0) {
        glEnableVertexAttribArray((GLuint)a_norm);
        glVertexAttribPointer((GLuint)a_norm, 3, GL_FLOAT, GL_FALSE,
                              SHIM_MESH_BYTE_STRIDE, (void *)(intptr_t)SHIM_MESH_NORM_OFFSET);
    }
    if (a_uv >= 0) {
        glEnableVertexAttribArray((GLuint)a_uv);
        glVertexAttribPointer((GLuint)a_uv, 2, GL_FLOAT, GL_FALSE,
                              SHIM_MESH_BYTE_STRIDE, (void *)(intptr_t)SHIM_MESH_UV_OFFSET);
    }
    if (a_tangent >= 0) {
        glEnableVertexAttribArray((GLuint)a_tangent);
        glVertexAttribPointer((GLuint)a_tangent, 4, GL_FLOAT, GL_FALSE,
                              SHIM_MESH_BYTE_STRIDE, (void *)(intptr_t)SHIM_MESH_TANGENT_OFFSET);
    }

    glDrawElements(GL_TRIANGLES, index_count, GL_UNSIGNED_SHORT, NULL);

    if (a_pos >= 0)  glDisableVertexAttribArray((GLuint)a_pos);
    if (a_norm >= 0) glDisableVertexAttribArray((GLuint)a_norm);
    if (a_uv >= 0)   glDisableVertexAttribArray((GLuint)a_uv);
    if (a_tangent >= 0) glDisableVertexAttribArray((GLuint)a_tangent);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &ibo);
}

void shim_gl_draw_lines_batch(float *verts, int vert_count, float *mvp) {
    glUseProgram(g_line_prog);
    shim_gl_upload_mvp(g_line_u_mvp, mvp);

    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(vert_count * 7 * (int)sizeof(float)),
                 verts, GL_STREAM_DRAW);

    if (g_line_a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)g_line_a_pos);
        glVertexAttribPointer((GLuint)g_line_a_pos, 3, GL_FLOAT, GL_FALSE, 28, (void *)0);
    }
    if (g_line_a_color >= 0) {
        glEnableVertexAttribArray((GLuint)g_line_a_color);
        glVertexAttribPointer((GLuint)g_line_a_color, 4, GL_FLOAT, GL_FALSE, 28, (void *)(intptr_t)12);
    }

    glDrawArrays(GL_LINES, 0, vert_count);

    if (g_line_a_pos >= 0)   glDisableVertexAttribArray((GLuint)g_line_a_pos);
    if (g_line_a_color >= 0) glDisableVertexAttribArray((GLuint)g_line_a_color);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDeleteBuffers(1, &vbo);
}

void shim_gl_draw_line_segment(float *verts, float *mvp, float width) {
    if (!verts) return;

    glUseProgram(g_line_prog);
    shim_gl_upload_mvp(g_line_u_mvp, mvp);

    if (width < 1.0f) width = 1.0f;
    glLineWidth(width);

    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(14 * (int)sizeof(float)),
                 verts, GL_STREAM_DRAW);

    if (g_line_a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)g_line_a_pos);
        glVertexAttribPointer((GLuint)g_line_a_pos, 3, GL_FLOAT, GL_FALSE, 28, (void *)0);
    }
    if (g_line_a_color >= 0) {
        glEnableVertexAttribArray((GLuint)g_line_a_color);
        glVertexAttribPointer((GLuint)g_line_a_color, 4, GL_FLOAT, GL_FALSE, 28,
                              (void *)(intptr_t)12);
    }

    glDrawArrays(GL_LINES, 0, 2);

    if (g_line_a_pos >= 0)   glDisableVertexAttribArray((GLuint)g_line_a_pos);
    if (g_line_a_color >= 0) glDisableVertexAttribArray((GLuint)g_line_a_color);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDeleteBuffers(1, &vbo);
    glLineWidth(1.0f);
}

void shim_gl_draw_text_bytes(unsigned char *data, int len,
                             float start_x, float start_y, float origin_x,
                             int w, int h, float glyph_w, float glyph_h,
                             float r, float g, float b) {
    if (!data || len <= 0 || w <= 0 || h <= 0) return;
    if (!g_text_font_ready || !g_text_prog || !g_text_font_tex || !g_text_vbo) return;

    GLboolean had_depth = glIsEnabled(GL_DEPTH_TEST);
    GLboolean had_blend = glIsEnabled(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(g_text_prog);
    glUniform2f(g_text_u_screen, (float)w, (float)h);
    glUniform3f(g_text_u_color, r, g, b);
    glUniform1i(g_text_u_font, 0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, g_text_font_tex);
    glBindBuffer(GL_ARRAY_BUFFER, g_text_vbo);

    if (g_text_a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)g_text_a_pos);
        glVertexAttribPointer((GLuint)g_text_a_pos, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float), (void *)0);
    }
    if (g_text_a_uv >= 0) {
        glEnableVertexAttribArray((GLuint)g_text_a_uv);
        glVertexAttribPointer((GLuint)g_text_a_uv, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float), (void *)(intptr_t)(2 * sizeof(float)));
    }

    float x = start_x;
    float y = start_y;
    float row_origin = origin_x;
    if (x < 0.0f) x = 0.0f;
    if (y < 0.0f) y = 0.0f;
    if (row_origin < 0.0f) row_origin = 0.0f;

    float gw = glyph_w;
    float gh = glyph_h;
    if (gw < 1.0f) gw = 1.0f;
    if (gh < 1.0f) gh = 1.0f;
    float atlas_gw = (float)g_text_glyph_w;
    float atlas_gh = (float)g_text_glyph_h;
    float tex_w = (float)g_text_tex_w;
    float tex_h = (float)g_text_tex_h;
    const int chars_per_row = 16;

    for (int i = 0; i < len; i++) {
        unsigned char ch = data[i];

        if (ch == '\n') {
            x = row_origin;
            y += gh;
            if (y + gh > (float)h) break;
            continue;
        }

        if (x + gw > (float)w) {
            x = row_origin;
            y += gh;
            if (y + gh > (float)h) break;
        }

        int cp = (int)ch;
        if (cp < g_text_font_first || cp > g_text_font_last) {
            cp = '?';
            if (cp < g_text_font_first || cp > g_text_font_last) {
                cp = g_text_font_first;
            }
        }

        int glyph_idx = cp - g_text_font_first;
        int tx = glyph_idx % chars_per_row;
        int ty = glyph_idx / chars_per_row;
        float u0 = ((float)tx * atlas_gw) / tex_w;
        float v0 = ((float)ty * atlas_gh) / tex_h;
        float u1 = (((float)tx + 1.0f) * atlas_gw) / tex_w;
        float v1 = (((float)ty + 1.0f) * atlas_gh) / tex_h;

        float x0 = x;
        float y0 = y;
        float x1 = x + gw;
        float y1 = y + gh;

        float verts[24] = {
            x0, y0, u0, v0,
            x1, y0, u1, v0,
            x1, y1, u1, v1,
            x0, y0, u0, v0,
            x1, y1, u1, v1,
            x0, y1, u0, v1,
        };
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)sizeof(verts), verts, GL_STREAM_DRAW);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        x += gw;
    }

    if (g_text_a_pos >= 0) glDisableVertexAttribArray((GLuint)g_text_a_pos);
    if (g_text_a_uv >= 0) glDisableVertexAttribArray((GLuint)g_text_a_uv);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    if (!had_depth) glDisable(GL_DEPTH_TEST);
    else glEnable(GL_DEPTH_TEST);
    if (!had_blend) glDisable(GL_BLEND);
}

/* ================================================================
 * DEPTH RENDERING
 * ================================================================ */

void shim_gl_begin_depth(float *mvp, float znear, float zfar) {
    glUseProgram(g_depth_prog);
    shim_gl_upload_mvp(g_depth_u_mvp, mvp);
    glUniform1f(g_depth_u_znear, znear);
    glUniform1f(g_depth_u_zfar, zfar);
}

void shim_gl_fbo_read_depth_encoded(float *out_depth, int w, int h,
                                     float znear, float zfar) {
    glBindFramebuffer(GL_FRAMEBUFFER, g_fbo);
    unsigned char *tmp = (unsigned char *)malloc((size_t)(w * 4));
    float range = zfar - znear;
    for (int row = 0; row < h; row++) {
        int gl_row = h - 1 - row; /* flip Y: GL bottom-up -> top-down */
        glReadPixels(0, gl_row, w, 1, GL_RGBA, GL_UNSIGNED_BYTE, tmp);
        for (int x = 0; x < w; x++) {
            float r = (float)tmp[x * 4 + 0] / 255.0f;
            float g = (float)tmp[x * 4 + 1] / 255.0f;
            float b = (float)tmp[x * 4 + 2] / 255.0f;
            float a = (float)tmp[x * 4 + 3] / 255.0f;
            float depth01 = r + g / 255.0f + b / 65025.0f + a / 16581375.0f;
            out_depth[row * w + x] = znear + depth01 * range;
        }
    }
    free(tmp);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

/* ================================================================
 * TEXTURE MANAGEMENT + BLIT (for 2D camera feed visualization)
 * ================================================================ */

static GLuint g_blit_prog = 0;
static GLint g_blit_a_pos, g_blit_a_uv;
static GLint g_blit_u_screen, g_blit_u_tex;
static GLuint g_blit_vbo = 0;

static void shim_gl_set_texture_params(void) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

static unsigned char *shim_rgb_to_rgba(const unsigned char *data, int w, int h) {
    size_t pixel_count = (size_t)w * (size_t)h;
    unsigned char *rgba = (unsigned char *)malloc(pixel_count * 4);
    if (!rgba) return NULL;
    for (size_t i = 0; i < pixel_count; i++) {
        rgba[i * 4 + 0] = data[i * 3 + 0];
        rgba[i * 4 + 1] = data[i * 3 + 1];
        rgba[i * 4 + 2] = data[i * 3 + 2];
        rgba[i * 4 + 3] = 255;
    }
    return rgba;
}

static void shim_blur_equirect_rgb(const unsigned char *src, unsigned char *dst,
                                   int w, int h) {
    int rx = w / 16;
    int ry = h / 16;
    if (rx < 1) rx = 1;
    if (ry < 1) ry = 1;
    if (rx > 24) rx = 24;
    if (ry > 12) ry = 12;

    unsigned char *tmp = (unsigned char *)malloc((size_t)w * (size_t)h * 3);
    if (!tmp) {
        memcpy(dst, src, (size_t)w * (size_t)h * 3);
        return;
    }

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            for (int c = 0; c < 3; c++) {
                int sum = 0;
                int count = 0;
                for (int dx = -rx; dx <= rx; dx++) {
                    int xx = x + dx;
                    while (xx < 0) xx += w;
                    while (xx >= w) xx -= w;
                    sum += src[(y * w + xx) * 3 + c];
                    count++;
                }
                tmp[(y * w + x) * 3 + c] = (unsigned char)(sum / count);
            }
        }
    }

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            for (int c = 0; c < 3; c++) {
                int sum = 0;
                int count = 0;
                for (int dy = -ry; dy <= ry; dy++) {
                    int yy = y + dy;
                    if (yy < 0) yy = 0;
                    if (yy >= h) yy = h - 1;
                    sum += tmp[(yy * w + x) * 3 + c];
                    count++;
                }
                dst[(y * w + x) * 3 + c] = (unsigned char)(sum / count);
            }
        }
    }

    free(tmp);
}

unsigned int shim_gl_create_texture_rgb(unsigned char *data, int w, int h) {
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    shim_gl_set_texture_params();
    /* GLES2: internal format must match format for glTexImage2D.
     * Use RGBA and convert from RGB to avoid driver issues. */
    unsigned char *rgba = shim_rgb_to_rgba(data, w, h);
    if (rgba) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, rgba);
        free(rgba);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    return (unsigned int)tex;
}

unsigned int shim_gl_create_texture_rgb_blurred(unsigned char *data, int w, int h) {
    size_t rgb_bytes = (size_t)w * (size_t)h * 3;
    unsigned char *blurred = (unsigned char *)malloc(rgb_bytes);
    if (!blurred) {
        return shim_gl_create_texture_rgb(data, w, h);
    }
    shim_blur_equirect_rgb(data, blurred, w, h);
    unsigned int tex = shim_gl_create_texture_rgb(blurred, w, h);
    free(blurred);
    return tex;
}

unsigned int shim_gl_create_texture_rgba(unsigned char *data, int w, int h) {
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    shim_gl_set_texture_params();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, data);
    glBindTexture(GL_TEXTURE_2D, 0);
    return (unsigned int)tex;
}

void shim_gl_update_texture_rgb(unsigned int tex, unsigned char *data, int w, int h) {
    glBindTexture(GL_TEXTURE_2D, tex);
    unsigned char *rgba = shim_rgb_to_rgba(data, w, h);
    if (rgba) {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h,
                        GL_RGBA, GL_UNSIGNED_BYTE, rgba);
        free(rgba);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
}

void shim_gl_update_texture_rgb_blurred(unsigned int tex, unsigned char *data, int w, int h) {
    size_t rgb_bytes = (size_t)w * (size_t)h * 3;
    unsigned char *blurred = (unsigned char *)malloc(rgb_bytes);
    if (!blurred) {
        shim_gl_update_texture_rgb(tex, data, w, h);
        return;
    }
    shim_blur_equirect_rgb(data, blurred, w, h);
    shim_gl_update_texture_rgb(tex, blurred, w, h);
    free(blurred);
}

void shim_gl_update_texture_rgba(unsigned int tex, unsigned char *data, int w, int h) {
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h,
                    GL_RGBA, GL_UNSIGNED_BYTE, data);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void shim_gl_delete_texture(unsigned int tex) {
    GLuint t = (GLuint)tex;
    glDeleteTextures(1, &t);
}

/* Initialize blit shader + quad VBO on first use.
 * Blit shader source is passed from Pika (loaded from file). */
int shim_gl_init_blit_shader(unsigned char *blit_vs, unsigned char *blit_fs) {
    if (g_blit_prog) return 0;
    g_blit_prog = compile_prog((const char *)blit_vs, (const char *)blit_fs);
    if (!g_blit_prog) return -1;
    g_blit_a_pos   = glGetAttribLocation(g_blit_prog, "a_pos");
    g_blit_a_uv    = glGetAttribLocation(g_blit_prog, "a_uv");
    g_blit_u_screen = glGetUniformLocation(g_blit_prog, "u_screen");
    g_blit_u_tex   = glGetUniformLocation(g_blit_prog, "u_tex");
    if (!g_blit_vbo) glGenBuffers(1, &g_blit_vbo);
    return 0;
}

int shim_gl_init_embedded_blit_shader(void) {
    return shim_gl_init_blit_shader(
        (unsigned char *)BLIT_VS, (unsigned char *)BLIT_FS);
}

/* Draw a textured quad at screen position (x,y) with size (w,h) in pixels.
 * screen_w, screen_h = viewport dimensions for NDC conversion. */
void shim_gl_draw_textured_quad(unsigned int tex,
                                float x, float y, float w, float h,
                                int screen_w, int screen_h) {
    if (!g_blit_prog) return;

    GLboolean had_depth = glIsEnabled(GL_DEPTH_TEST);
    GLboolean had_blend = glIsEnabled(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(g_blit_prog);
    glUniform2f(g_blit_u_screen, (float)screen_w, (float)screen_h);
    glUniform1i(g_blit_u_tex, 0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);

    /* Build quad: 2 triangles, interleaved pos(x,y) + uv(u,v) */
    float x0 = x, y0 = y, x1 = x + w, y1 = y + h;
    float verts[24] = {
        x0, y0, 0.0f, 0.0f,
        x1, y0, 1.0f, 0.0f,
        x1, y1, 1.0f, 1.0f,
        x0, y0, 0.0f, 0.0f,
        x1, y1, 1.0f, 1.0f,
        x0, y1, 0.0f, 1.0f,
    };

    glBindBuffer(GL_ARRAY_BUFFER, g_blit_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)sizeof(verts), verts, GL_STREAM_DRAW);

    if (g_blit_a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)g_blit_a_pos);
        glVertexAttribPointer((GLuint)g_blit_a_pos, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float), (void *)0);
    }
    if (g_blit_a_uv >= 0) {
        glEnableVertexAttribArray((GLuint)g_blit_a_uv);
        glVertexAttribPointer((GLuint)g_blit_a_uv, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float),
                              (void *)(intptr_t)(2 * sizeof(float)));
    }

    glDrawArrays(GL_TRIANGLES, 0, 6);

    if (g_blit_a_pos >= 0) glDisableVertexAttribArray((GLuint)g_blit_a_pos);
    if (g_blit_a_uv >= 0)  glDisableVertexAttribArray((GLuint)g_blit_a_uv);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    if (had_depth) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);
    if (!had_blend) glDisable(GL_BLEND);
}

/* ================================================================
 * PPM file writer (for render verification)
 * ================================================================ */

int shim_write_ppm(unsigned char *filename, unsigned char *pixels, int w, int h) {
    FILE *f = fopen((const char *)filename, "wb");
    if (!f) return -1;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    fwrite(pixels, 1, (size_t)(w * h * 3), f);
    fclose(f);
    return 0;
}

/* ================================================================
 * Binary STL loader with vertex welding and smooth normals
 * Reads binary STL, merges duplicate vertices, accumulates normals.
 * Writes interleaved [px,py,pz,nx,ny,nz,u,v,tx,ty,tz,tw] per vertex.
 * UVs are set to (0,0) since STL has no UV data, and tangents default to +X.
 * Optional normalization maps mesh into [-0.5, 0.5] centered at origin.
 * ================================================================ */

static int shim_load_stl_impl(unsigned char *filename, float *verts, uint16_t *indices,
                              int *vert_count, int *index_count,
                              int max_verts, int max_indices,
                              int normalize_to_unit) {
    FILE *f = fopen((const char *)filename, "rb");
    if (!f) return -1;

    /* Skip 80-byte header */
    fseek(f, 80, SEEK_SET);

    /* Read triangle count */
    uint32_t tri_count;
    if (fread(&tri_count, 4, 1, f) != 1) { fclose(f); return -1; }

    int vc = 0, ic = 0;
    float weld_eps = 0.0001f;

    int tri_budget = max_indices / 3;
    if (max_verts / 3 < tri_budget) tri_budget = max_verts / 3;
    if (tri_budget < 1) tri_budget = 1;

    /*
     * Huge STLs exceed our fixed buffers. In that case sample triangles
     * uniformly and skip welding so we keep a coarse full-volume shape
     * instead of loading only an early file slice.
     */
    int decimate = (tri_count > (uint32_t)(tri_budget * 8));
    uint32_t tri_iter = decimate ? (uint32_t)tri_budget : tri_count;

    for (uint32_t t = 0; t < tri_iter; t++) {
        float face_normal[3];
        float positions[9]; /* 3 vertices * 3 components */
        uint16_t attr;

        if (decimate) {
            uint32_t src_t = (uint32_t)(((uint64_t)t * (uint64_t)tri_count) / (uint64_t)tri_iter);
            long off = 84L + (long)src_t * 50L;
            if (fseek(f, off, SEEK_SET) != 0) break;
        }

        if (fread(face_normal, sizeof(float), 3, f) != 3) break;
        if (fread(positions, sizeof(float), 9, f) != 9) break;
        if (fread(&attr, 2, 1, f) != 1) break;

        if (decimate) {
            for (int v = 0; v < 3; v++) {
                if (vc >= max_verts || ic >= max_indices) break;
                int b = vc * SHIM_MESH_FLOAT_STRIDE;
                verts[b + 0] = positions[v * 3 + 0];
                verts[b + 1] = positions[v * 3 + 1];
                verts[b + 2] = positions[v * 3 + 2];
                verts[b + 3] = face_normal[0];
                verts[b + 4] = face_normal[1];
                verts[b + 5] = face_normal[2];
                verts[b + 6] = 0.0f;
                verts[b + 7] = 0.0f;
                verts[b + 8] = 1.0f;
                verts[b + 9] = 0.0f;
                verts[b + 10] = 0.0f;
                verts[b + 11] = 1.0f;
                indices[ic++] = (uint16_t)vc;
                vc++;
            }
            continue;
        }

        /* Process each vertex of the triangle */
        for (int v = 0; v < 3; v++) {
            float px = positions[v * 3 + 0];
            float py = positions[v * 3 + 1];
            float pz = positions[v * 3 + 2];

            /* Search for existing vertex with same position */
            int found = -1;
            for (int e = 0; e < vc; e++) {
                int base = e * SHIM_MESH_FLOAT_STRIDE;
                float dx = verts[base + 0] - px;
                float dy = verts[base + 1] - py;
                float dz = verts[base + 2] - pz;
                if (dx * dx + dy * dy + dz * dz < weld_eps * weld_eps) {
                    found = e;
                    break;
                }
            }

            if (found >= 0) {
                /* Accumulate face normal to existing vertex */
                int base = found * SHIM_MESH_FLOAT_STRIDE;
                verts[base + 3] += face_normal[0];
                verts[base + 4] += face_normal[1];
                verts[base + 5] += face_normal[2];
                if (ic < max_indices) {
                    indices[ic++] = (uint16_t)found;
                }
            } else if (vc < max_verts) {
                /* New vertex */
                int base = vc * SHIM_MESH_FLOAT_STRIDE;
                verts[base + 0] = px;
                verts[base + 1] = py;
                verts[base + 2] = pz;
                verts[base + 3] = face_normal[0];
                verts[base + 4] = face_normal[1];
                verts[base + 5] = face_normal[2];
                verts[base + 6] = 0.0f;
                verts[base + 7] = 0.0f;
                verts[base + 8] = 1.0f;
                verts[base + 9] = 0.0f;
                verts[base + 10] = 0.0f;
                verts[base + 11] = 1.0f;
                if (ic < max_indices) {
                    indices[ic++] = (uint16_t)vc;
                }
                vc++;
            }
        }
    }
    fclose(f);

    /* Compute bounding box */
    float min_x = 1e30f, min_y = 1e30f, min_z = 1e30f;
    float max_x = -1e30f, max_y = -1e30f, max_z = -1e30f;
    for (int i = 0; i < vc; i++) {
        int base = i * SHIM_MESH_FLOAT_STRIDE;
        if (verts[base] < min_x) min_x = verts[base];
        if (verts[base + 1] < min_y) min_y = verts[base + 1];
        if (verts[base + 2] < min_z) min_z = verts[base + 2];
        if (verts[base] > max_x) max_x = verts[base];
        if (verts[base + 1] > max_y) max_y = verts[base + 1];
        if (verts[base + 2] > max_z) max_z = verts[base + 2];
    }

    if (normalize_to_unit) {
        /* Center and normalize to fit in [-0.5, 0.5] */
        float cx = (min_x + max_x) * 0.5f;
        float cy = (min_y + max_y) * 0.5f;
        float cz = (min_z + max_z) * 0.5f;
        float sx = max_x - min_x;
        float sy = max_y - min_y;
        float sz = max_z - min_z;
        float scale = sx;
        if (sy > scale) scale = sy;
        if (sz > scale) scale = sz;
        if (scale < 0.0001f) scale = 1.0f;
        float inv_scale = 1.0f / scale;

        for (int i = 0; i < vc; i++) {
            int base = i * SHIM_MESH_FLOAT_STRIDE;
            verts[base + 0] = (verts[base + 0] - cx) * inv_scale;
            verts[base + 1] = (verts[base + 1] - cy) * inv_scale;
            verts[base + 2] = (verts[base + 2] - cz) * inv_scale;
        }
    }

    /* Normalize all vertex normals */
    for (int i = 0; i < vc; i++) {
        int base = i * SHIM_MESH_FLOAT_STRIDE;
        float nx = verts[base + 3];
        float ny = verts[base + 4];
        float nz = verts[base + 5];
        float len = sqrtf(nx * nx + ny * ny + nz * nz);
        if (len > 0.0001f) {
            float inv_len = 1.0f / len;
            verts[base + 3] = nx * inv_len;
            verts[base + 4] = ny * inv_len;
            verts[base + 5] = nz * inv_len;
        }
    }

    *vert_count = vc;
    *index_count = ic;
    return 0;
}

int shim_load_stl(unsigned char *filename, float *verts, uint16_t *indices,
                  int *vert_count, int *index_count,
                  int max_verts, int max_indices) {
    return shim_load_stl_impl(filename, verts, indices,
                              vert_count, index_count,
                              max_verts, max_indices, 1);
}

int shim_load_stl_raw(unsigned char *filename, float *verts, uint16_t *indices,
                      int *vert_count, int *index_count,
                      int max_verts, int max_indices) {
    return shim_load_stl_impl(filename, verts, indices,
                              vert_count, index_count,
                              max_verts, max_indices, 0);
}
