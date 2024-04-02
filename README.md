# Zig Multiplayer Arena Shooter

Features:

* Hand-written renderer, using OpenGL.
* Skeletal Animations.

* AABB-to-Triangle Mesh physics.

* Netcode: Rollback and server-authoritative, with limited subtick.

# Running

To run a client and server at the same time:

```
zig build run -- --listen
```

To connect to a running server:

```
zig build run -- --addr <address>:<port>
```

To start a server:

```
zig build run -- --dedicated --addr <address>:<port>
```

Tested with Zig version 0.12.0-dev.1849+bb0f7d55e.
