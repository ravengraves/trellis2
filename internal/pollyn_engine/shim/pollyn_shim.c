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

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#endif
#define STBI_ONLY_PNG
#define STBI_NO_HDR
#define STBI_NO_LINEAR
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

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
    glfwSwapInterval(0);
    glDisable(GL_DITHER);

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
    /* Window and offscreen targets use the same camera handedness.
     * CPU readback handles the vertical image-origin difference separately. */
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

void shim_gl_fbo_destroy(void);

int shim_gl_fbo_create(int w, int h) {
    if (w <= 0 || h <= 0) return -1;
    if (g_fbo && g_fbo_w == w && g_fbo_h == h) return 0;
    /* Camera exports and terminal rendering share this target. */
    shim_gl_fbo_destroy();
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
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        shim_gl_fbo_destroy();
        return -1;
    }
    return 0;
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
    g_fbo_w = g_fbo_h = 0;
}

/* --- Offscreen GL context via EGL pbuffer (portable EGL baseline) --- */

int shim_gl_init_offscreen(void) {
    int rc = shim_egl_acquire_offscreen(&g_egl, 64, 64);
    if (rc == 0) {
        glDisable(GL_DITHER);
    }
    return rc;
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
static GLint g_depth_u_mvp, g_depth_u_model, g_depth_u_znear, g_depth_u_zfar;
static GLint g_depth_u_eye_pos;
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
    g_depth_u_model = glGetUniformLocation(g_depth_prog, "u_model");
    g_depth_u_znear = glGetUniformLocation(g_depth_prog, "u_znear");
    g_depth_u_zfar  = glGetUniformLocation(g_depth_prog, "u_zfar");
    g_depth_u_eye_pos = glGetUniformLocation(g_depth_prog, "u_eye_pos");
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
    GLboolean had_cull = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
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

void shim_gl_begin_depth(float *mvp, float *model,
                         float ex, float ey, float ez,
                         float znear, float zfar) {
    glUseProgram(g_depth_prog);
    shim_gl_upload_mvp(g_depth_u_mvp, mvp);
    glUniformMatrix4fv(g_depth_u_model, 1, GL_FALSE, model);
    glUniform3f(g_depth_u_eye_pos, ex, ey, ez);
    glUniform1f(g_depth_u_znear, znear);
    glUniform1f(g_depth_u_zfar, zfar);
}

void shim_gl_fbo_read_depth_encoded(float *out_depth, int w, int h,
                                     float znear, float zfar) {
    glBindFramebuffer(GL_FRAMEBUFFER, g_fbo);
    unsigned char *tmp = (unsigned char *)malloc((size_t)(w * 4));
    float range = zfar - znear;
    if (range <= 0.0f) {
        range = 1.0f;
    }
    for (int row = 0; row < h; row++) {
        int gl_row = h - 1 - row; /* flip Y: GL bottom-up -> top-down */
        glReadPixels(0, gl_row, w, 1, GL_RGBA, GL_UNSIGNED_BYTE, tmp);
        for (int x = 0; x < w; x++) {
            float r = (float)tmp[x * 4 + 0] / 255.0f;
            float g = (float)tmp[x * 4 + 1] / 255.0f;
            float b = (float)tmp[x * 4 + 2] / 255.0f;
            float a = (float)tmp[x * 4 + 3] / 255.0f;
            float depth01 = r + g / 255.0f + b / 65025.0f + a / 16581375.0f;
            if (depth01 < 0.0f) {
                depth01 = 0.0f;
            } else if (depth01 > 1.0f) {
                depth01 = 1.0f;
            }
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
static GLint g_blit_u_screen, g_blit_u_tex, g_blit_u_tiled;
static GLuint g_blit_vbo = 0;

static GLuint g_dust_prog = 0;
static GLint g_dust_a_pos, g_dust_a_uv;
static GLint g_dust_u_screen, g_dust_u_color_tex, g_dust_u_depth_tex;
static GLint g_dust_u_texel, g_dust_u_dust_color, g_dust_u_znear, g_dust_u_zfar;
static GLint g_dust_u_start, g_dust_u_end, g_dust_u_falloff_distance, g_dust_u_density;
static GLint g_dust_u_noise_strength, g_dust_u_noise_scale;
static GLint g_dust_u_blur_px, g_dust_u_time;
static GLuint g_dust_vbo = 0;

static GLuint g_radiation_prog = 0;
static GLint g_radiation_a_pos, g_radiation_a_uv;
static GLint g_radiation_u_screen, g_radiation_u_tex;
static GLint g_radiation_u_density, g_radiation_u_min_value;
static GLint g_radiation_u_max_value, g_radiation_u_seed;
static GLint g_radiation_u_overlay;
static GLuint g_radiation_vbo = 0;

static GLuint g_laser_glare_prog = 0;
static GLint g_laser_glare_a_pos, g_laser_glare_a_uv;
static GLint g_laser_glare_u_screen, g_laser_glare_u_tex;
static GLint g_laser_glare_u_center, g_laser_glare_u_radius;
static GLint g_laser_glare_u_decay, g_laser_glare_u_hot_radius;
static GLint g_laser_glare_u_hot_strength, g_laser_glare_u_glare_strength;
static GLint g_laser_glare_u_streak_strength;
static GLuint g_laser_glare_vbo = 0;

static void shim_gl_set_texture_params(void) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void shim_gl_set_texture_repeat(unsigned int tex) {
    if (!tex) return;
    glBindTexture(GL_TEXTURE_2D, (GLuint)tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glBindTexture(GL_TEXTURE_2D, 0);
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

static void shim_pack_depth01(float depth01, unsigned char *out) {
    const float max_depth01 = 1.0f - (1.0f / 16777216.0f);
    if (depth01 < 0.0f) depth01 = 0.0f;
    if (depth01 > max_depth01) depth01 = max_depth01;

    float e0 = depth01 - floorf(depth01);
    float e1 = depth01 * 255.0f;
    float e2 = depth01 * 65025.0f;
    float e3 = depth01 * 16581375.0f;
    e1 = e1 - floorf(e1);
    e2 = e2 - floorf(e2);
    e3 = e3 - floorf(e3);
    e0 -= e1 * (1.0f / 255.0f);
    e1 -= e2 * (1.0f / 255.0f);
    e2 -= e3 * (1.0f / 255.0f);
    if (e0 < 0.0f) e0 = 0.0f;
    if (e1 < 0.0f) e1 = 0.0f;
    if (e2 < 0.0f) e2 = 0.0f;
    if (e3 < 0.0f) e3 = 0.0f;
    if (e0 > 1.0f) e0 = 1.0f;
    if (e1 > 1.0f) e1 = 1.0f;
    if (e2 > 1.0f) e2 = 1.0f;
    if (e3 > 1.0f) e3 = 1.0f;
    out[0] = (unsigned char)(e0 * 255.0f + 0.5f);
    out[1] = (unsigned char)(e1 * 255.0f + 0.5f);
    out[2] = (unsigned char)(e2 * 255.0f + 0.5f);
    out[3] = (unsigned char)(e3 * 255.0f + 0.5f);
}

static unsigned char *shim_depth_f32_to_rgba(const float *depth,
                                             int w, int h,
                                             float znear, float zfar) {
    size_t pixel_count = (size_t)w * (size_t)h;
    unsigned char *rgba = (unsigned char *)malloc(pixel_count * 4);
    if (!rgba) return NULL;
    float range = zfar - znear;
    if (range <= 0.0f) range = 1.0f;
    for (size_t i = 0; i < pixel_count; i++) {
        float depth01 = (depth[i] - znear) / range;
        shim_pack_depth01(depth01, rgba + i * 4);
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

unsigned int shim_gl_create_texture_from_png(unsigned char *filename,
                                             int desired_channels,
                                             int *out_w, int *out_h) {
    if (out_w) *out_w = 0;
    if (out_h) *out_h = 0;
    if (!filename) return 0;

    int channels = desired_channels;
    if (channels != 3 && channels != 4) {
        channels = 4;
    }

    int w = 0, h = 0, src_channels = 0;
    /* Mesh UVs use OpenGL's bottom-left texture convention. */
    stbi_set_flip_vertically_on_load(1);
    unsigned char *pixels = stbi_load((const char *)filename,
                                      &w, &h, &src_channels, channels);
    if (!pixels || w <= 0 || h <= 0) {
        if (pixels) stbi_image_free(pixels);
        return 0;
    }

    unsigned int tex = 0;
    if (channels == 3) {
        tex = shim_gl_create_texture_rgb(pixels, w, h);
    } else {
        tex = shim_gl_create_texture_rgba(pixels, w, h);
    }
    stbi_image_free(pixels);

    if (tex != 0) {
        if (out_w) *out_w = w;
        if (out_h) *out_h = h;
    }
    return tex;
}

unsigned int shim_gl_create_depth_texture_f32(float *data, int w, int h,
                                               float znear, float zfar) {
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    shim_gl_set_texture_params();
    unsigned char *rgba = shim_depth_f32_to_rgba(data, w, h, znear, zfar);
    if (rgba) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, rgba);
        free(rgba);
    }
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

void shim_gl_update_depth_texture_f32(unsigned int tex, float *data, int w, int h,
                                      float znear, float zfar) {
    glBindTexture(GL_TEXTURE_2D, tex);
    unsigned char *rgba = shim_depth_f32_to_rgba(data, w, h, znear, zfar);
    if (rgba) {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h,
                        GL_RGBA, GL_UNSIGNED_BYTE, rgba);
        free(rgba);
    }
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
    g_blit_u_tiled = glGetUniformLocation(g_blit_prog, "u_tiled");
    if (!g_blit_vbo) glGenBuffers(1, &g_blit_vbo);
    return 0;
}

int shim_gl_init_embedded_blit_shader(void) {
    return shim_gl_init_blit_shader(
        (unsigned char *)BLIT_VS, (unsigned char *)BLIT_FS);
}

/* Draw a textured quad at screen position (x,y) with size (w,h) in pixels.
 * screen_w, screen_h = viewport dimensions for NDC conversion. */
static void shim_gl_draw_textured_quad_uv(unsigned int tex,
                                          float x, float y, float w, float h,
                                          int screen_w, int screen_h,
                                          float tiles_x, float tiles_y,
                                          int tiled) {
    if (!g_blit_prog) return;
    if (tiles_x <= 0.0f) tiles_x = 1.0f;
    if (tiles_y <= 0.0f) tiles_y = 1.0f;

    GLboolean had_depth = glIsEnabled(GL_DEPTH_TEST);
    GLboolean had_blend = glIsEnabled(GL_BLEND);
    GLboolean had_cull = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(g_blit_prog);
    glUniform2f(g_blit_u_screen, (float)screen_w, (float)screen_h);
    glUniform1i(g_blit_u_tex, 0);
    if (g_blit_u_tiled >= 0) {
        glUniform1i(g_blit_u_tiled, tiled ? 1 : 0);
    }

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);

    /* Build quad: 2 triangles, interleaved pos(x,y) + uv(u,v) */
    float x0 = x, y0 = y, x1 = x + w, y1 = y + h;
    float u0 = 0.0f, v0 = 0.0f, u1 = tiles_x, v1 = tiles_y;
    float verts[24] = {
        x0, y0, u0, v0,
        x1, y0, u1, v0,
        x1, y1, u1, v1,
        x0, y0, u0, v0,
        x1, y1, u1, v1,
        x0, y1, u0, v1,
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
    if (had_cull) glEnable(GL_CULL_FACE);
    else glDisable(GL_CULL_FACE);
    if (!had_blend) glDisable(GL_BLEND);
}

void shim_gl_draw_textured_quad(unsigned int tex,
                                float x, float y, float w, float h,
                                int screen_w, int screen_h) {
    shim_gl_draw_textured_quad_uv(tex, x, y, w, h, screen_w, screen_h,
                                  1.0f, 1.0f, 0);
}

void shim_gl_draw_textured_quad_tiled(unsigned int tex,
                                      float x, float y, float w, float h,
                                      int screen_w, int screen_h,
                                      float tiles_x, float tiles_y) {
    shim_gl_draw_textured_quad_uv(tex, x, y, w, h, screen_w, screen_h,
                                  tiles_x, tiles_y, 1);
}

int shim_gl_init_dust_shader(unsigned char *dust_vs, unsigned char *dust_fs) {
    if (g_dust_prog) return 0;
    g_dust_prog = compile_prog((const char *)dust_vs, (const char *)dust_fs);
    if (!g_dust_prog) return -1;
    g_dust_a_pos = glGetAttribLocation(g_dust_prog, "a_pos");
    g_dust_a_uv = glGetAttribLocation(g_dust_prog, "a_uv");
    g_dust_u_screen = glGetUniformLocation(g_dust_prog, "u_screen");
    g_dust_u_color_tex = glGetUniformLocation(g_dust_prog, "u_color_tex");
    g_dust_u_depth_tex = glGetUniformLocation(g_dust_prog, "u_depth_tex");
    g_dust_u_texel = glGetUniformLocation(g_dust_prog, "u_texel");
    g_dust_u_dust_color = glGetUniformLocation(g_dust_prog, "u_dust_color");
    g_dust_u_znear = glGetUniformLocation(g_dust_prog, "u_znear");
    g_dust_u_zfar = glGetUniformLocation(g_dust_prog, "u_zfar");
    g_dust_u_start = glGetUniformLocation(g_dust_prog, "u_start");
    g_dust_u_end = glGetUniformLocation(g_dust_prog, "u_end");
    g_dust_u_falloff_distance = glGetUniformLocation(g_dust_prog, "u_falloff_distance");
    g_dust_u_density = glGetUniformLocation(g_dust_prog, "u_density");
    g_dust_u_noise_strength = glGetUniformLocation(g_dust_prog, "u_noise_strength");
    g_dust_u_noise_scale = glGetUniformLocation(g_dust_prog, "u_noise_scale");
    g_dust_u_blur_px = glGetUniformLocation(g_dust_prog, "u_blur_px");
    g_dust_u_time = glGetUniformLocation(g_dust_prog, "u_time");
    if (!g_dust_vbo) glGenBuffers(1, &g_dust_vbo);
    return 0;
}

int shim_gl_init_embedded_dust_shader(void) {
    return shim_gl_init_dust_shader(
        (unsigned char *)DUST_VS, (unsigned char *)DUST_FS);
}

void shim_gl_draw_dust_quad(unsigned int color_tex, unsigned int depth_tex,
                            float x, float y, float w, float h,
                            int screen_w, int screen_h,
                            float dust_r, float dust_g, float dust_b,
                            float znear, float zfar,
                            float start, float end, float falloff_distance, float density,
                            float noise_strength, float noise_scale,
                            float blur_px, float time_sec) {
    if (!g_dust_prog) return;
    if (screen_w <= 0 || screen_h <= 0) return;

    GLboolean had_depth = glIsEnabled(GL_DEPTH_TEST);
    GLboolean had_blend = glIsEnabled(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    glUseProgram(g_dust_prog);
    glUniform2f(g_dust_u_screen, (float)screen_w, (float)screen_h);
    glUniform1i(g_dust_u_color_tex, 0);
    glUniform1i(g_dust_u_depth_tex, 1);
    glUniform2f(g_dust_u_texel, 1.0f / (float)screen_w, 1.0f / (float)screen_h);
    glUniform3f(g_dust_u_dust_color, dust_r, dust_g, dust_b);
    glUniform1f(g_dust_u_znear, znear);
    glUniform1f(g_dust_u_zfar, zfar);
    glUniform1f(g_dust_u_start, start);
    glUniform1f(g_dust_u_end, end);
    glUniform1f(g_dust_u_falloff_distance, falloff_distance);
    glUniform1f(g_dust_u_density, density);
    glUniform1f(g_dust_u_noise_strength, noise_strength);
    glUniform1f(g_dust_u_noise_scale, noise_scale);
    glUniform1f(g_dust_u_blur_px, blur_px);
    glUniform1f(g_dust_u_time, time_sec);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, color_tex);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, depth_tex);

    float x0 = x, y0 = y, x1 = x + w, y1 = y + h;
    float verts[24] = {
        x0, y0, 0.0f, 0.0f,
        x1, y0, 1.0f, 0.0f,
        x1, y1, 1.0f, 1.0f,
        x0, y0, 0.0f, 0.0f,
        x1, y1, 1.0f, 1.0f,
        x0, y1, 0.0f, 1.0f,
    };

    glBindBuffer(GL_ARRAY_BUFFER, g_dust_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)sizeof(verts), verts, GL_STREAM_DRAW);
    if (g_dust_a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)g_dust_a_pos);
        glVertexAttribPointer((GLuint)g_dust_a_pos, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float), (void *)0);
    }
    if (g_dust_a_uv >= 0) {
        glEnableVertexAttribArray((GLuint)g_dust_a_uv);
        glVertexAttribPointer((GLuint)g_dust_a_uv, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float),
                              (void *)(intptr_t)(2 * sizeof(float)));
    }

    glDrawArrays(GL_TRIANGLES, 0, 6);

    if (g_dust_a_pos >= 0) glDisableVertexAttribArray((GLuint)g_dust_a_pos);
    if (g_dust_a_uv >= 0) glDisableVertexAttribArray((GLuint)g_dust_a_uv);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);

    if (had_depth) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);
    if (had_blend) glEnable(GL_BLEND);
    else glDisable(GL_BLEND);
}

int shim_gl_init_radiation_shader(unsigned char *radiation_vs,
                                  unsigned char *radiation_fs) {
    if (g_radiation_prog) return 0;
    g_radiation_prog = compile_prog((const char *)radiation_vs,
                                    (const char *)radiation_fs);
    if (!g_radiation_prog) return -1;
    g_radiation_a_pos = glGetAttribLocation(g_radiation_prog, "a_pos");
    g_radiation_a_uv = glGetAttribLocation(g_radiation_prog, "a_uv");
    g_radiation_u_screen = glGetUniformLocation(g_radiation_prog, "u_screen");
    g_radiation_u_tex = glGetUniformLocation(g_radiation_prog, "u_tex");
    g_radiation_u_density = glGetUniformLocation(g_radiation_prog, "u_density");
    g_radiation_u_min_value = glGetUniformLocation(g_radiation_prog, "u_min_value");
    g_radiation_u_max_value = glGetUniformLocation(g_radiation_prog, "u_max_value");
    g_radiation_u_seed = glGetUniformLocation(g_radiation_prog, "u_seed");
    g_radiation_u_overlay = glGetUniformLocation(g_radiation_prog, "u_overlay");
    if (!g_radiation_vbo) glGenBuffers(1, &g_radiation_vbo);
    return 0;
}

int shim_gl_init_embedded_radiation_shader(void) {
    return shim_gl_init_radiation_shader(
        (unsigned char *)RADIATION_VS, (unsigned char *)RADIATION_FS);
}

void shim_gl_draw_radiation_quad(unsigned int tex,
                                 float x, float y, float w, float h,
                                 int screen_w, int screen_h,
                                 float density,
                                 float min_value,
                                 float max_value,
                                 float seed) {
    if (!g_radiation_prog) return;
    if (screen_w <= 0 || screen_h <= 0) return;

    GLboolean had_depth = glIsEnabled(GL_DEPTH_TEST);
    GLboolean had_blend = glIsEnabled(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    glUseProgram(g_radiation_prog);
    glUniform2f(g_radiation_u_screen, (float)screen_w, (float)screen_h);
    glUniform1i(g_radiation_u_tex, 0);
    glUniform1f(g_radiation_u_density, density);
    glUniform1f(g_radiation_u_min_value, min_value);
    glUniform1f(g_radiation_u_max_value, max_value);
    glUniform1f(g_radiation_u_seed, seed);
    if (g_radiation_u_overlay >= 0) glUniform1i(g_radiation_u_overlay, 0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);

    float x0 = x, y0 = y, x1 = x + w, y1 = y + h;
    float verts[24] = {
        x0, y0, 0.0f, 0.0f,
        x1, y0, 1.0f, 0.0f,
        x1, y1, 1.0f, 1.0f,
        x0, y0, 0.0f, 0.0f,
        x1, y1, 1.0f, 1.0f,
        x0, y1, 0.0f, 1.0f,
    };

    glBindBuffer(GL_ARRAY_BUFFER, g_radiation_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)sizeof(verts), verts, GL_STREAM_DRAW);
    if (g_radiation_a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)g_radiation_a_pos);
        glVertexAttribPointer((GLuint)g_radiation_a_pos, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float), (void *)0);
    }
    if (g_radiation_a_uv >= 0) {
        glEnableVertexAttribArray((GLuint)g_radiation_a_uv);
        glVertexAttribPointer((GLuint)g_radiation_a_uv, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float),
                              (void *)(intptr_t)(2 * sizeof(float)));
    }

    glDrawArrays(GL_TRIANGLES, 0, 6);

    if (g_radiation_a_pos >= 0) glDisableVertexAttribArray((GLuint)g_radiation_a_pos);
    if (g_radiation_a_uv >= 0) glDisableVertexAttribArray((GLuint)g_radiation_a_uv);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    if (had_depth) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);
    if (had_blend) glEnable(GL_BLEND);
    else glDisable(GL_BLEND);
}

void shim_gl_draw_radiation_overlay_quad(float x, float y, float w, float h,
                                         int screen_w, int screen_h,
                                         float density,
                                         float min_value,
                                         float max_value,
                                         float seed) {
    if (!g_radiation_prog) return;
    if (screen_w <= 0 || screen_h <= 0) return;

    GLboolean had_depth = glIsEnabled(GL_DEPTH_TEST);
    GLboolean had_blend = glIsEnabled(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    glUseProgram(g_radiation_prog);
    glUniform2f(g_radiation_u_screen, (float)screen_w, (float)screen_h);
    glUniform1i(g_radiation_u_tex, 0);
    glUniform1f(g_radiation_u_density, density);
    glUniform1f(g_radiation_u_min_value, min_value);
    glUniform1f(g_radiation_u_max_value, max_value);
    glUniform1f(g_radiation_u_seed, seed);
    if (g_radiation_u_overlay >= 0) glUniform1i(g_radiation_u_overlay, 1);

    float x0 = x, y0 = y, x1 = x + w, y1 = y + h;
    float verts[24] = {
        x0, y0, 0.0f, 0.0f,
        x1, y0, 1.0f, 0.0f,
        x1, y1, 1.0f, 1.0f,
        x0, y0, 0.0f, 0.0f,
        x1, y1, 1.0f, 1.0f,
        x0, y1, 0.0f, 1.0f,
    };

    glBindBuffer(GL_ARRAY_BUFFER, g_radiation_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)sizeof(verts), verts, GL_STREAM_DRAW);
    if (g_radiation_a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)g_radiation_a_pos);
        glVertexAttribPointer((GLuint)g_radiation_a_pos, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float), (void *)0);
    }
    if (g_radiation_a_uv >= 0) {
        glEnableVertexAttribArray((GLuint)g_radiation_a_uv);
        glVertexAttribPointer((GLuint)g_radiation_a_uv, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float),
                              (void *)(intptr_t)(2 * sizeof(float)));
    }

    glDrawArrays(GL_TRIANGLES, 0, 6);

    if (g_radiation_a_pos >= 0) glDisableVertexAttribArray((GLuint)g_radiation_a_pos);
    if (g_radiation_a_uv >= 0) glDisableVertexAttribArray((GLuint)g_radiation_a_uv);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    if (g_radiation_u_overlay >= 0) glUniform1i(g_radiation_u_overlay, 0);

    if (had_depth) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (had_blend) glEnable(GL_BLEND);
    else glDisable(GL_BLEND);
}

int shim_gl_init_laser_glare_shader(unsigned char *laser_glare_vs,
                                    unsigned char *laser_glare_fs) {
    if (g_laser_glare_prog) return 0;
    g_laser_glare_prog = compile_prog((const char *)laser_glare_vs,
                                      (const char *)laser_glare_fs);
    if (!g_laser_glare_prog) return -1;
    g_laser_glare_a_pos = glGetAttribLocation(g_laser_glare_prog, "a_pos");
    g_laser_glare_a_uv = glGetAttribLocation(g_laser_glare_prog, "a_uv");
    g_laser_glare_u_screen = glGetUniformLocation(g_laser_glare_prog, "u_screen");
    g_laser_glare_u_tex = glGetUniformLocation(g_laser_glare_prog, "u_tex");
    g_laser_glare_u_center = glGetUniformLocation(g_laser_glare_prog, "u_center");
    g_laser_glare_u_radius = glGetUniformLocation(g_laser_glare_prog, "u_radius");
    g_laser_glare_u_decay = glGetUniformLocation(g_laser_glare_prog, "u_decay");
    g_laser_glare_u_hot_radius = glGetUniformLocation(g_laser_glare_prog, "u_hot_radius");
    g_laser_glare_u_hot_strength = glGetUniformLocation(g_laser_glare_prog, "u_hot_strength");
    g_laser_glare_u_glare_strength = glGetUniformLocation(g_laser_glare_prog, "u_glare_strength");
    g_laser_glare_u_streak_strength = glGetUniformLocation(g_laser_glare_prog, "u_streak_strength");
    if (!g_laser_glare_vbo) glGenBuffers(1, &g_laser_glare_vbo);
    return 0;
}

int shim_gl_init_embedded_laser_glare_shader(void) {
    return shim_gl_init_laser_glare_shader(
        (unsigned char *)LASER_GLARE_VS, (unsigned char *)LASER_GLARE_FS);
}

void shim_gl_draw_laser_glare_quad(unsigned int tex,
                                   float x, float y, float w, float h,
                                   int screen_w, int screen_h,
                                   float center_x, float center_y,
                                   float radius, float decay,
                                   float hot_radius, float hot_strength,
                                   float glare_strength,
                                   float streak_strength) {
    if (!g_laser_glare_prog) return;
    if (screen_w <= 0 || screen_h <= 0) return;

    GLboolean had_depth = glIsEnabled(GL_DEPTH_TEST);
    GLboolean had_blend = glIsEnabled(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    glUseProgram(g_laser_glare_prog);
    glUniform2f(g_laser_glare_u_screen, (float)screen_w, (float)screen_h);
    glUniform1i(g_laser_glare_u_tex, 0);
    glUniform2f(g_laser_glare_u_center, center_x, center_y);
    glUniform1f(g_laser_glare_u_radius, radius);
    glUniform1f(g_laser_glare_u_decay, decay);
    glUniform1f(g_laser_glare_u_hot_radius, hot_radius);
    glUniform1f(g_laser_glare_u_hot_strength, hot_strength);
    glUniform1f(g_laser_glare_u_glare_strength, glare_strength);
    glUniform1f(g_laser_glare_u_streak_strength, streak_strength);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);

    float x0 = x, y0 = y, x1 = x + w, y1 = y + h;
    float verts[24] = {
        x0, y0, 0.0f, 0.0f,
        x1, y0, 1.0f, 0.0f,
        x1, y1, 1.0f, 1.0f,
        x0, y0, 0.0f, 0.0f,
        x1, y1, 1.0f, 1.0f,
        x0, y1, 0.0f, 1.0f,
    };

    glBindBuffer(GL_ARRAY_BUFFER, g_laser_glare_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)sizeof(verts), verts, GL_STREAM_DRAW);
    if (g_laser_glare_a_pos >= 0) {
        glEnableVertexAttribArray((GLuint)g_laser_glare_a_pos);
        glVertexAttribPointer((GLuint)g_laser_glare_a_pos, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float), (void *)0);
    }
    if (g_laser_glare_a_uv >= 0) {
        glEnableVertexAttribArray((GLuint)g_laser_glare_a_uv);
        glVertexAttribPointer((GLuint)g_laser_glare_a_uv, 2, GL_FLOAT, GL_FALSE,
                              4 * (int)sizeof(float),
                              (void *)(intptr_t)(2 * sizeof(float)));
    }

    glDrawArrays(GL_TRIANGLES, 0, 6);

    if (g_laser_glare_a_pos >= 0) glDisableVertexAttribArray((GLuint)g_laser_glare_a_pos);
    if (g_laser_glare_a_uv >= 0) glDisableVertexAttribArray((GLuint)g_laser_glare_a_uv);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    if (had_depth) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);
    if (had_blend) glEnable(GL_BLEND);
    else glDisable(GL_BLEND);
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
 * Minimal OBJ loader for Blender-style static textured meshes.
 * Supports v/vt/vn/f records and triangulates polygons as a fan.
 * ================================================================ */

typedef struct {
    float x, y, z;
} ShimVec3;

typedef struct {
    float u, v;
} ShimVec2;

static ShimVec3 shim_vec3_sub(ShimVec3 a, ShimVec3 b) {
    ShimVec3 r = {a.x - b.x, a.y - b.y, a.z - b.z};
    return r;
}

static ShimVec3 shim_vec3_add(ShimVec3 a, ShimVec3 b) {
    ShimVec3 r = {a.x + b.x, a.y + b.y, a.z + b.z};
    return r;
}

static ShimVec3 shim_vec3_scale(ShimVec3 v, float s) {
    ShimVec3 r = {v.x * s, v.y * s, v.z * s};
    return r;
}

static float shim_vec3_dot(ShimVec3 a, ShimVec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static float shim_vec3_len(ShimVec3 v) {
    return sqrtf(shim_vec3_dot(v, v));
}

static ShimVec3 shim_vec3_cross(ShimVec3 a, ShimVec3 b) {
    ShimVec3 r = {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
    return r;
}

static ShimVec3 shim_vec3_normalize(ShimVec3 v) {
    float len = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len <= 0.0001f) {
        ShimVec3 up = {0.0f, 0.0f, 1.0f};
        return up;
    }
    float inv = 1.0f / len;
    ShimVec3 r = {v.x * inv, v.y * inv, v.z * inv};
    return r;
}

static int shim_obj_push_vec3(ShimVec3 **items, int *count, int *cap, ShimVec3 v) {
    if (*count >= *cap) {
        int next_cap = *cap > 0 ? *cap * 2 : 256;
        ShimVec3 *next = (ShimVec3 *)realloc(*items, (size_t)next_cap * sizeof(ShimVec3));
        if (!next) return 0;
        *items = next;
        *cap = next_cap;
    }
    (*items)[*count] = v;
    *count += 1;
    return 1;
}

static int shim_obj_push_vec2(ShimVec2 **items, int *count, int *cap, ShimVec2 v) {
    if (*count >= *cap) {
        int next_cap = *cap > 0 ? *cap * 2 : 256;
        ShimVec2 *next = (ShimVec2 *)realloc(*items, (size_t)next_cap * sizeof(ShimVec2));
        if (!next) return 0;
        *items = next;
        *cap = next_cap;
    }
    (*items)[*count] = v;
    *count += 1;
    return 1;
}

static int shim_obj_resolve_index(int idx, int count) {
    if (idx > 0) return idx - 1;
    if (idx < 0) return count + idx;
    return -1;
}

static int shim_obj_parse_face_token(const char *tok, int *vi, int *ti, int *ni) {
    char *end = NULL;
    *vi = 0;
    *ti = 0;
    *ni = 0;
    long v = strtol(tok, &end, 10);
    if (end == tok) return 0;
    *vi = (int)v;
    if (*end == '/') {
        const char *p = end + 1;
        if (*p != '/') {
            long t = strtol(p, &end, 10);
            *ti = (int)t;
        } else {
            end = (char *)p;
        }
        if (*end == '/') {
            p = end + 1;
            long n = strtol(p, &end, 10);
            *ni = (int)n;
        }
    }
    return 1;
}

static int shim_obj_emit_corner(float *verts, uint16_t *indices,
                                int *vc, int *ic, int max_verts, int max_indices,
                                ShimVec3 p, ShimVec3 n, ShimVec2 uv) {
    if (*vc >= max_verts || *ic >= max_indices) return 0;
    int base = *vc * SHIM_MESH_FLOAT_STRIDE;
    verts[base + 0] = p.x;
    verts[base + 1] = p.y;
    verts[base + 2] = p.z;
    verts[base + 3] = n.x;
    verts[base + 4] = n.y;
    verts[base + 5] = n.z;
    verts[base + 6] = uv.u;
    verts[base + 7] = uv.v;
    verts[base + 8] = 1.0f;
    verts[base + 9] = 0.0f;
    verts[base + 10] = 0.0f;
    verts[base + 11] = 1.0f;
    indices[*ic] = (uint16_t)*vc;
    *vc += 1;
    *ic += 1;
    return 1;
}

int shim_load_obj(unsigned char *filename, float *verts, uint16_t *indices,
                  int *vert_count, int *index_count,
                  int max_verts, int max_indices) {
    *vert_count = 0;
    *index_count = 0;
    FILE *f = fopen((const char *)filename, "r");
    if (!f) return -1;

    int total_tris = 0;
    char count_line[1024];
    while (fgets(count_line, sizeof(count_line), f)) {
        if (count_line[0] == 'f' && count_line[1] == ' ') {
            int corner_count = 0;
            char *tok = strtok(count_line + 2, " \t\r\n");
            while (tok && corner_count < 32) {
                corner_count += 1;
                tok = strtok(NULL, " \t\r\n");
            }
            if (corner_count >= 3) total_tris += corner_count - 2;
        }
    }

    int tri_budget = max_indices / 3;
    if (max_verts / 3 < tri_budget) tri_budget = max_verts / 3;
    if (tri_budget < 1) tri_budget = 1;
    int sample_step = 1;
    if (total_tris > tri_budget) {
        sample_step = (total_tris + tri_budget - 1) / tri_budget;
    }
    rewind(f);

    ShimVec3 *positions = NULL;
    ShimVec2 *uvs = NULL;
    ShimVec3 *normals = NULL;
    int pos_count = 0, uv_count = 0, norm_count = 0;
    int pos_cap = 0, uv_cap = 0, norm_cap = 0;
    int vc = 0, ic = 0;
    int tri_seen = 0;
    int ok = 1;
    char line[1024];

    while (ok && fgets(line, sizeof(line), f)) {
        if (line[0] == 'v' && line[1] == ' ') {
            ShimVec3 p = {0.0f, 0.0f, 0.0f};
            if (sscanf(line + 2, "%f %f %f", &p.x, &p.y, &p.z) == 3) {
                ok = shim_obj_push_vec3(&positions, &pos_count, &pos_cap, p);
            }
        } else if (line[0] == 'v' && line[1] == 't' && line[2] == ' ') {
            ShimVec2 uv = {0.0f, 0.0f};
            if (sscanf(line + 3, "%f %f", &uv.u, &uv.v) >= 2) {
                ok = shim_obj_push_vec2(&uvs, &uv_count, &uv_cap, uv);
            }
        } else if (line[0] == 'v' && line[1] == 'n' && line[2] == ' ') {
            ShimVec3 n = {0.0f, 0.0f, 1.0f};
            if (sscanf(line + 3, "%f %f %f", &n.x, &n.y, &n.z) == 3) {
                ok = shim_obj_push_vec3(&normals, &norm_count, &norm_cap,
                                        shim_vec3_normalize(n));
            }
        } else if (line[0] == 'f' && line[1] == ' ') {
            int vis[32], tis[32], nis[32];
            int corner_count = 0;
            char *tok = strtok(line + 2, " \t\r\n");
            while (tok && corner_count < 32) {
                if (shim_obj_parse_face_token(tok, &vis[corner_count],
                                              &tis[corner_count],
                                              &nis[corner_count])) {
                    corner_count += 1;
                }
                tok = strtok(NULL, " \t\r\n");
            }
            if (corner_count < 3) continue;

            for (int tri = 1; tri + 1 < corner_count; tri++) {
                int emit_tri = ((tri_seen % sample_step) == 0);
                tri_seen += 1;
                if (!emit_tri) continue;
                if (vc + 3 > max_verts || ic + 3 > max_indices) continue;

                int cidx[3] = {0, tri, tri + 1};
                ShimVec3 p[3];
                ShimVec2 uv[3];
                ShimVec3 n[3];
                int has_normal[3] = {0, 0, 0};
                int valid = 1;
                for (int k = 0; k < 3; k++) {
                    int ci = cidx[k];
                    int pi = shim_obj_resolve_index(vis[ci], pos_count);
                    int ui = shim_obj_resolve_index(tis[ci], uv_count);
                    int ni = shim_obj_resolve_index(nis[ci], norm_count);
                    if (pi < 0 || pi >= pos_count) {
                        valid = 0;
                        break;
                    }
                    p[k] = positions[pi];
                    uv[k].u = 0.0f;
                    uv[k].v = 0.0f;
                    if (ui >= 0 && ui < uv_count) {
                        uv[k] = uvs[ui];
                    }
                    n[k].x = 0.0f;
                    n[k].y = 0.0f;
                    n[k].z = 1.0f;
                    if (ni >= 0 && ni < norm_count) {
                        n[k] = normals[ni];
                        has_normal[k] = 1;
                    }
                }
                if (!valid) continue;

                ShimVec3 face_n = shim_vec3_normalize(
                    shim_vec3_cross(shim_vec3_sub(p[1], p[0]),
                                    shim_vec3_sub(p[2], p[0])));
                for (int k = 0; k < 3; k++) {
                    if (!has_normal[k]) n[k] = face_n;
                    ok = shim_obj_emit_corner(verts, indices, &vc, &ic,
                                              max_verts, max_indices,
                                              p[k], n[k], uv[k]);
                    if (!ok) break;
                }
            }
        }
    }

    fclose(f);
    free(positions);
    free(uvs);
    free(normals);
    *vert_count = vc;
    *index_count = ic;
    return ok ? 0 : -1;
}

static void shim_obj_generate_tangents(float *verts, uint16_t *indices,
                                       int vert_count, int index_count) {
    if (!verts || !indices || vert_count <= 0 || index_count < 3) return;

    ShimVec3 *tan1 = (ShimVec3 *)calloc((size_t)vert_count, sizeof(ShimVec3));
    ShimVec3 *tan2 = (ShimVec3 *)calloc((size_t)vert_count, sizeof(ShimVec3));
    if (!tan1 || !tan2) {
        free(tan1);
        free(tan2);
        return;
    }

    for (int ti = 0; ti + 2 < index_count; ti += 3) {
        int i0 = indices[ti + 0];
        int i1 = indices[ti + 1];
        int i2 = indices[ti + 2];
        if (i0 < 0 || i0 >= vert_count ||
            i1 < 0 || i1 >= vert_count ||
            i2 < 0 || i2 >= vert_count) {
            continue;
        }

        float *v0 = verts + i0 * SHIM_MESH_FLOAT_STRIDE;
        float *v1 = verts + i1 * SHIM_MESH_FLOAT_STRIDE;
        float *v2 = verts + i2 * SHIM_MESH_FLOAT_STRIDE;
        float x1 = v1[0] - v0[0], x2 = v2[0] - v0[0];
        float y1 = v1[1] - v0[1], y2 = v2[1] - v0[1];
        float z1 = v1[2] - v0[2], z2 = v2[2] - v0[2];
        float s1 = v1[6] - v0[6], s2 = v2[6] - v0[6];
        float t1 = v1[7] - v0[7], t2 = v2[7] - v0[7];
        float denom = s1 * t2 - s2 * t1;
        if (denom > -0.000001f && denom < 0.000001f) continue;

        float r = 1.0f / denom;
        ShimVec3 sdir = {
            (t2 * x1 - t1 * x2) * r,
            (t2 * y1 - t1 * y2) * r,
            (t2 * z1 - t1 * z2) * r
        };
        ShimVec3 tdir = {
            (s1 * x2 - s2 * x1) * r,
            (s1 * y2 - s2 * y1) * r,
            (s1 * z2 - s2 * z1) * r
        };

        tan1[i0] = shim_vec3_add(tan1[i0], sdir);
        tan1[i1] = shim_vec3_add(tan1[i1], sdir);
        tan1[i2] = shim_vec3_add(tan1[i2], sdir);
        tan2[i0] = shim_vec3_add(tan2[i0], tdir);
        tan2[i1] = shim_vec3_add(tan2[i1], tdir);
        tan2[i2] = shim_vec3_add(tan2[i2], tdir);
    }

    for (int i = 0; i < vert_count; i++) {
        float *v = verts + i * SHIM_MESH_FLOAT_STRIDE;
        ShimVec3 n = {v[3], v[4], v[5]};
        ShimVec3 t = tan1[i];
        float ndott = shim_vec3_dot(n, t);
        t = shim_vec3_sub(t, shim_vec3_scale(n, ndott));
        if (shim_vec3_len(t) < 0.0001f) {
            ShimVec3 ref = {0.0f, 0.0f, 1.0f};
            if (n.z > 0.9f || n.z < -0.9f) {
                ref.x = 0.0f; ref.y = 1.0f; ref.z = 0.0f;
            }
            t = shim_vec3_cross(ref, n);
            if (shim_vec3_len(t) < 0.0001f) {
                ShimVec3 xref = {1.0f, 0.0f, 0.0f};
                t = shim_vec3_cross(xref, n);
            }
        }
        t = shim_vec3_normalize(t);
        float handedness = (shim_vec3_dot(shim_vec3_cross(n, t), tan2[i]) < 0.0f)
            ? -1.0f : 1.0f;
        v[8] = t.x;
        v[9] = t.y;
        v[10] = t.z;
        v[11] = handedness;
    }

    free(tan1);
    free(tan2);
}

int shim_gl_load_obj_upload(unsigned char *filename, int target_triangles,
                            unsigned int *out_vbo, unsigned int *out_ibo,
                            unsigned int *out_wire_ibo,
                            int *out_index_count, int *out_wire_index_count) {
    if (out_vbo) *out_vbo = 0;
    if (out_ibo) *out_ibo = 0;
    if (out_wire_ibo) *out_wire_ibo = 0;
    if (out_index_count) *out_index_count = 0;
    if (out_wire_index_count) *out_wire_index_count = 0;
    if (!filename || !out_vbo || !out_ibo || !out_wire_ibo ||
        !out_index_count || !out_wire_index_count) {
        return -1;
    }

    if (target_triangles <= 0) target_triangles = 20000;
    if (target_triangles > 21845) target_triangles = 21845;
    int max_indices = target_triangles * 3;
    int max_verts = max_indices;
    float *verts = (float *)calloc((size_t)max_verts * SHIM_MESH_FLOAT_STRIDE,
                                   sizeof(float));
    uint16_t *indices = (uint16_t *)calloc((size_t)max_indices,
                                           sizeof(uint16_t));
    if (!verts || !indices) {
        free(verts);
        free(indices);
        return -1;
    }

    int vert_count = 0;
    int index_count = 0;
    int rc = shim_load_obj(filename, verts, indices,
                           &vert_count, &index_count, max_verts, max_indices);
    if (rc != 0 || vert_count <= 0 || index_count <= 0) {
        free(verts);
        free(indices);
        return -1;
    }

    shim_obj_generate_tangents(verts, indices, vert_count, index_count);

    unsigned int vbo = shim_gl_create_vbo(verts, vert_count * SHIM_MESH_FLOAT_STRIDE);
    unsigned int ibo = shim_gl_create_ibo(indices, index_count);
    int wire_index_count = 0;
    unsigned int wire_ibo = shim_gl_create_wire_ibo(indices, index_count,
                                                    &wire_index_count);
    free(verts);
    free(indices);

    if (!vbo || !ibo) {
        if (vbo) shim_gl_delete_buffer(vbo);
        if (ibo) shim_gl_delete_buffer(ibo);
        if (wire_ibo) shim_gl_delete_buffer(wire_ibo);
        return -1;
    }

    *out_vbo = vbo;
    *out_ibo = ibo;
    *out_wire_ibo = wire_ibo;
    *out_index_count = index_count;
    *out_wire_index_count = wire_index_count;
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
