# ROS3 Pika bindings

See `ros3/ALLOCATIONS.md` in the Rose repository for the current
ROS3/Mirage/Quicksand allocation boundaries and deferred arena-allocator work.

Native Pika functions for ROS3, implemented in Pika and linked directly to the
ROS3 C library. No C wrapper library or simulator dependency is required.

## Public API and symbol names

Use `rose::` names with Pika strings:

```pika
let node = rose::create_node("my_node", "");
let pub = rose::create_pub(node, "/cmd_vel", 256u64, 30u64);
let sub = rose::create_sub(node, "/cmd_vel", true);
rose::info("ready");
// Publish/read Mirage messages, then release handles.
rose::shutdown(node);
```

The node prefix is explicit (empty means no namespace). Subscriber `latest_only`
is an explicit boolean; native subscriptions use polling. Publisher message
size and ring capacity are explicit. Native constructors use standard allocation.
There are no default arguments in the ROS3 declarations or native adapters.

Pika resolves `rose::create_pub` to the Pika name `rose_create_pub`. Its
`@symbol("pk_rose_create_pub")` gives the implementation a distinct linker name:

```pika
export @symbol("rose_create_pub") fn c_rose_create_pub(n: RoseNode,
    name: *u8, name_length: i64, message_length: u64, message_hz: u64) RosePublisher;

export @symbol("pk_rose_create_pub") fn rose_create_pub(n: RoseNode, name: str,
    message_length: u64, message_hz: u64) RosePublisher = {
    let bytes = strings::toutf8(name);
    return c_rose_create_pub(n, bytes.data, bytes.len: i64,
        message_length, message_hz);
};
```

The native API passes exact UTF-8 byte lengths, with no fixed buffers or
NUL-termination assumption. Parameters and logging use the same pattern.
Functions whose signatures already fit Pika, such as `rose::publish`,
`rose::shutdown`, and the clock functions, map directly to their C symbols.
`@symbol` changes linkage, not argument conversion; adapter bodies perform that
conversion explicitly.

## Raw interop and ownership

The `c_rose_*` APIs expose explicit pointer/length arguments. Function-pointer
parameters are omitted: initialization and subscription adapters always pass
null allocation hooks and callbacks. The full ABI declarations are internal
`ffi_*` functions, not exported APIs. Use `c_rose_init` for C argc/argv
initialization, and `c_rose_create_node` for remaps. `rose::last_error` and
`rose::topic_to_shm` currently retain the C pointer/buffer API. `rose::last_error`
returns borrowed storage; inspect `rose::last_error_code` after failures.

Raw string arguments need exact lengths, or NUL termination when passing -1.
Remap buffers transfer to ROS3 on successful construction and must use the
system malloc/free pair used by these bindings. Do not pass borrowed Pika slices as transferred remaps.
On failure they remain caller-owned. The lengths array contains two entries
per remap pair.

Custom allocator hooks and subscription callbacks are not currently exposed.
Subscriptions use explicit reads. Handles are opaque: destroy messages with
`mirage_destroy` and nodes with `rose::shutdown`; do not use them afterward.

## Files and integration

- `ros3.pka`: native API, raw ROS3 declarations, and a time section for clocks
  and sleep functions.
- `mirage.pka`: complete standard `mirage.h` API, independent of ROS3 types.

## Mirage API

All functions in the standard `mirage/include/mirage.h` are covered. The
embedded Mirage ABI is separate and is not interchangeable with these bindings.

| API | Pika interface |
| --- | --- |
| `create`, `destroy` | Standard allocation; internal calls always pass null allocator/deallocator hooks |
| `init` | `*MirageBuffer` and borrowed `[]u8` storage |
| `write_start`, `read_start`, `reset`, `remaining` | Direct C mappings |
| `write_i8/i16/i32/i64/f64`, matching reads | Typed scalar values and output pointers |
| `write_fn`, `write_string/sym/bigint/bigreal` | Pika `str`, converted to exact UTF-8 spans |
| `write_bin` | Byte slice |
| `read_fn`, `read_string/sym/bigint/bigreal/bin` | Output byte slice and required length pointer; `read_fn` also takes an argument-count pointer |
| `write_numericarray`, `read_numericarray` | Explicit type, rank, dimensions, payload pointers and read capacities |
| `write_association`, `read_association`, `write_assoc_rule`, `read_assoc_rule` | Direct C mappings |
| `peek_element_type` | Element-type output pointer; does not advance the cursor |

For example, after starting a message:

```pika
mirage::write_string(msg, "hello");
// After read_start on the encoded message:
let buffer: [64]u8 = [0...];
let length: i64 = 0;
let result = mirage::read_string(msg, buffer[0z..64z], &length);
// Check result >= 0 before using buffer[0z..length: size].
```

Read adapters do not allocate, append NULs, or validate UTF-8. They retain C's
byte-count/error return convention; on failure the cursor and outputs remain
unchanged. To skip copying, use the `c_mirage_read_*` declaration with
`MIRAGE_SKIP_COPY`. Numeric-array dimensions and payloads can be skipped
independently using the C API's -1 capacities. Payload storage must have the
size and alignment required by its element type.

`MirageBuffer` mirrors the public message struct's field order and types.
Allocated `MirageMsg` handles can be cast to `*MirageBuffer` for buffer access.
Keep caller-provided storage alive while using an initialized message; never
pass a caller-owned message to `destroy`. These bindings' `destroy` function
is only for messages allocated with the standard allocator, not custom C heaps.

All element/array enum values, error codes, and length/size constants are
included. Element enum values are ordinals, not wire tags. The public encoding
tables are mirrored as immutable `MIRAGE_ELEMENT_TYPE_ENCODING`,
`MIRAGE_ARRAY_TYPE_ENCODING`, and `MIRAGE_ARRAY_TYPE_SIZE` constants using the
values in Mirage's `src/mirage.c`; they are not imported C globals.

## Compiling

Compile these source files with the application and link ROS3, Mirage, and
Quicksand. The current build concatenates application sources; `rose::` syntax
works with the exported `rose_*` names without an installed module. These files
can be moved together into a separate package or ROS3's `lang/pika` directory.

With the installed Pika driver, compile from any working directory:

```sh
pikacc \
  /path/to/internal/bindings/ros3/ros3.pka \
  /path/to/internal/bindings/ros3/mirage.pka \
  /path/to/app.pka -R --platform libc -o /path/to/app -- \
  -L/usr/local/lib -Wl,-rpath,/usr/local/lib \
  -lros3 -lmirage -lquicksand -lm -lpthread -ldl
```

Link separately compiled C objects; do not concatenate `ros3.h` or `mirage.h`
into generated Pika C, whose opaque declarations use different C types.

Trellis2's joystick uses the native API.
