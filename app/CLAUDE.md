# `app` — environment gotchas

Loaded only when working under `app/`.

## `--headless` sidesteps every gotcha below

Everything on this page is about getting a GUI window to open under WSL. A
scripted session doesn't need one: `./target/debug/app --headless` skips viz
and runs the control loop on the main thread. The notes below matter only when
the change is about the visualization itself, or when you want to watch the
plots.

## Viz renderer: use `glow`, not the default `wgpu`

eframe's default `wgpu` renderer **fails at startup in this WSL setup**
(`WinitEventLoop(ExitFailure(1))`) — there's no `/dev/dri` render node and no
Vulkan ICD, only WSL's `/dev/dxg` GPU passthrough. `glow` (OpenGL) via Mesa,
through `/dev/dxg`, works. Don't switch the dependency back to `wgpu`.

eframe's default `accesskit` feature is disabled: it needs a D-Bus session
daemon, which is absent here.

## Running a GUI app on a fresh WSL setup

WSLg provides the compositor, **not** the client-side libraries. Install:

```
apt-get install libwayland-client0 libwayland-egl1 libwayland-cursor0 \
                libxkbcommon0 libegl1 libgl1
```

## Window sizing

`ViewportBuilder::with_inner_size([1000.0, 900.0])` in `main()` — eframe's
unset default is too short for the current plot count. The `CentralPanel`'s
content is wrapped in `egui::ScrollArea::vertical()` so growing content stays
reachable regardless of window size or axis count.
