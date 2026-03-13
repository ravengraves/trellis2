# Roseviz UI Status

`roseviz` is currently a minimal keyboard-driven viewer shell for:

- image tiles
- point cloud tiles
- a global transform overlay/topic
- one OS window with tiled subviews and bottom tab workspaces

It is usable for development and testing, but the UX is still primitive.

# Current Behavior

At startup, each tab opens with two tiles:

- top tile: image
- bottom tile: point cloud

Default demo bindings are applied when the ROS3 bridge is available:

- image: `/demo/camera/front`
- points: `/demo/points/lidar`
- tf: `/demo/tf/basic`

The bottom bar contains tabs `0` through `9` plus `all`.
The selected tab is brighter.
The selected tile border is brighter.
The left sidebar shows the selected tile's:

- tile type
- current topic binding
- deserialize string
- error state
- transform topic
- simulation state
- known topic list

Only the active tab/workspace is updated and rendered.

# Current Controls

- `0`..`9`: switch to tab `0`..`9`
- `a`: switch to `all`
- `tab`: switch focus between tile area and sidebar
- `up/down/left/right`: move tile focus
- `h`: split focused tile horizontally
- `v`: split focused tile vertically
- `x`: close focused tile
- `ctrl+p`: open palette
- `ctrl+d`: edit deserialize string for selected tile
- `space`: in sidebar focus, toggle selected topic on selected tile
- `p`: pause/play spawned sim
- `r`: reset spawned sim
- `s`: single-step spawned sim

For point-cloud tiles, orbit controls are active when that tile is focused.

# Current Flow: Add A New Tile

Today there is no explicit "new empty tile" action.

Current flow:

1. Focus the tile area.
2. Move focus to the tile you want to split.
3. Press `h` or `v`.

What happens now:

- the focused leaf is split
- the new tile is auto-created
- the new tile kind is the opposite of the focused tile
- image splits create a new point-cloud tile
- point-cloud splits create a new image tile
- the new tile is automatically bound to the default demo topic for that kind
- focus moves to the new tile

This is functional, but not good UX for real work because the user is not asked:

- what tile type they want
- whether the tile should start unbound
- which topic should be attached

# Current Flow: Add A New Subscriber / Bind A Topic

A tile effectively becomes a subscriber when a topic is bound to it.

There are two current flows.

## Sidebar Flow

1. Press `tab` until the sidebar is focused.
2. Use `up/down` to choose a discovered topic.
3. Press `space`.

Behavior:

- if the selected tile is already bound to that topic, it is unbound
- otherwise the topic is bound to the selected tile

## Palette Flow

1. Focus the tile you want.
2. Press `ctrl+p`.
3. Press `enter` on `add_topic`.
4. Type a fuzzy query.
5. Use `up/down` to choose a topic.
6. Press `enter`.

Behavior:

- the selected topic is bound to the selected tile
- palette closes after binding

# Current Deserialize Flow

1. Focus the tile you want.
2. Press `ctrl+d`.
3. Edit the deserialize string.
4. Press `enter` or `escape`.

Current implementation note:

- the UI stores and shows the deserialize string
- decode still follows the minimal rule: first numericarray wins
- bad decode/bind states are shown in red and should not crash the app

# Current Limitations

- adding a tile is implicit via split, not explicit
- new tiles are not created empty
- tile type is inferred from split source instead of chosen by user
- binding is one-topic-per-tile
- transforms are global, not per tile/workspace
- deserialize editing exists, but the parser behavior is still minimal
- image and point cloud are the only real display types
- no dedicated mouse-first interaction model yet
- no right-click/context actions
- no persistent layouts/config yet

# Planned UX

The next UI pass should keep the implementation simple but make the workflow explicit.

## Planned Tile Creation Flow

Recommended command-palette flow:

1. `ctrl+p`
2. choose `new_tile`
3. choose split direction: `horizontal` or `vertical`
4. choose tile type:
   - `image`
   - `point_cloud`
5. optionally choose `bind_topic_now` or `leave_unbound`

Recommended behavior:

- new tiles start empty unless the user explicitly chooses a topic
- tile type is chosen directly
- focus moves to the new tile

## Planned Topic Binding Flow

Recommended palette actions:

- `bind_topic`
- `unbind_topic`
- `set_transform_topic`
- `edit_deserialize`

Recommended bind flow:

1. focus tile
2. `ctrl+p`
3. choose `bind_topic`
4. type fuzzy query
5. confirm topic

Sidebar should remain as the fast toggle path for known topics, but the palette should be the primary explicit action flow.

## Planned Sidebar Improvements

- show tile id and tile type
- show whether topic is bound, stale, decode-failing, or active
- show a small per-topic type guess: `image`, `points`, `unknown`
- show transform topic separately from visual topic list
- show current tab/workspace name

## Planned Visual Feedback

- clearer active-tile border
- clearer active-sidebar row
- explicit "unbound" tile overlay
- explicit "decode error" overlay with deserialize string
- explicit "hidden tabs are idle" note in debug/status mode

## Planned Mouse UX

Minimal mouse improvements worth adding:

- click tile to focus it
- click tab to switch
- click sidebar row to select it
- double-click sidebar row to bind/unbind

# Short Usage Summary

For current usage:

1. launch `./build/roseviz`
2. use `h` or `v` to create another tile
3. use arrows to focus a tile
4. use `tab` to move to sidebar
5. use `space` on a topic row to bind/unbind
6. or use `ctrl+p` to fuzzy-bind a topic

That is the current working flow.
