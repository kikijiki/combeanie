# Re-record the docs demo video

Target: `website/static/media/restocking-showcase.mp4` (under ~25MB), plus poster/loop siblings.
The wrapper enters the pinned baseline shell, including FFmpeg for encoding; build the ROS
workspace first with `just build`.

## Showcase recording: `restocking-showcase.*`

Runs `baseline.launch.py` on the `just demo` scene (`bundle_scenario:=true`, `showcase_materials.yaml`,
the dressed `restocking_showcase.sdf`) with the same ground-truth evidence pins. One 1920x1080
camera records every frame at 6 Hz of simulated time while `showcase_director.py` cuts it from the
robot's state: wide during surveys and idle time, a tray close-up from the +X side while the tool
goes in and takes a product, a mid shot while it is carried, and a close shot of the shelf mouth
while it is set down. Each cut is logged with the frame count, and `showcase_edit.py` keeps every
frame of the close-ups, thins the wide and mid shots, encodes 1280x720 at 24 fps, writes the poster
(the middle of the first placement) and a seam-matched loop of the wide shot (the closest pair of
frames four to eight seconds apart; a forwards-then-backwards loop when none is close). It writes
`restocking-showcase.mp4`, `restocking-showcase-loop.webp` and `restocking-showcase-poster.png` next
in `website/static/media`.

```bash
export DISPLAY=:0 WAYLAND_DISPLAY=wayland-1 QT_QPA_PLATFORM=xcb
bash website/scripts/record_showcase.bash
```

Optional bounds (defaults shown): `REC_TARGET_TRANSFERS=3`, `REC_MAX_CYCLES=12`,
`REC_MAX_SECONDS=1500`, `REC_RATE=6` and `REC_POSTER_FRAME=<n>` to choose the poster by hand. The
camera sensor writes every frame as a 1920x1080 PNG (about 0.5 MB) under `$REC_DIR/raw_frames`, named
`showcase_cam_recording::link::camera_<n>.png`; the recorder bridges the topic only to make Gazebo
render the camera. Keep a fresh `REC_DIR` on a disk with room (a run is about 1.7k frames), and
delete it afterwards. The run takes about seven minutes at the current real-time factor.

`showcase_views.py list` names the views; `showcase_views.py sdf <view>` prints a camera model for
any of them, which is how screenshots are taken from a running scene:

```bash
gz service -s /world/restocking/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 8000 --req "sdf_filename: \"cam.sdf\", name: \"showcase_cam_tray\""
```

## GUI fallback

To watch the same scene in the GUI and screen-record it:

```bash
export DISPLAY=:0 WAYLAND_DISPLAY=wayland-1 QT_QPA_PLATFORM=xcb
just demo 3  # up to three progress cycles / transfers
nix-shell -p wf-recorder --run \
  'wf-recorder -o HDMI-A-2 -f /tmp/restocking-raw.mp4 -c libx264 -p crf=23 -r 20 -D -y'
# Stop with Ctrl-C, then compress:
ffmpeg -y -i /tmp/restocking-raw.mp4 -t 45 -vf scale=1280:-2 \
  -c:v libx264 -crf 23 -preset medium -an -movflags +faststart \
  website/static/media/restocking-showcase.mp4
```

Notes:

- The recorder uses the shared domain/partition lease and reaps only its own tagged orphans.
- The recorder acquires the shared simulator slot with a one-slot limit before launching.
  Keep other simulator campaigns serialized under the same lease convention.
- Force `QT_QPA_PLATFORM=xcb` for Gazebo under niri + xwayland-satellite.
- Do not commit uncompressed multi-minute 4K captures.
- Do not run live ffmpeg or rosbag on the camera topic during the transfer; dump frames, encode after.

## Regenerate the website GLBs

Build first, then run the generator through the installed ROS workspace so Xacro resolves the
UR10e meshes:

```bash
just build
scripts/with_workspace.bash python website/scripts/generate_glb_models.py
```

The generator reads surveyed workcell geometry, the product collision catalog, and
`dense_restock_products.yaml`. It writes `robot.glb`, `workcell.glb`, and `restocking_cell.glb`;
the latter two include all partial-front and dense-back scenario cylinders at their spawn poses.
Keep `website/static/models/LICENSE-UR10e.md` with the exported UR10e assets.
