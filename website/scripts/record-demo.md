# Re-record the docs demo video

Target: `website/static/media/restocking-demo.mp4` (under ~25MB), plus poster/loop siblings.
The wrapper enters the pinned baseline shell, including FFmpeg for encoding; build the ROS
workspace first with `just build`. Existing footage remains historical until an accepted capture
is explicitly reviewed and committed.

## Preferred: autonomous dense campaign + side camera

Runs `baseline.launch.py` with `autonomous_campaign:=true`, `campaign_restart_acknowledged:=true`, up to 12 campaign cycles, and
the same dense product/lane-policy pair and ground-truth evidence pins as `just demo gt:=true`.
A fresh capture directory prevents persisted lane policy from overriding the fixture. It spawns a temporary low-rate Gazebo side camera during startup, starts recording when the autonomous front sweep begins, and stops after three
successful evidence-selected transfers. Sampled frames play back at 4x speed, so the clip shows
the survey and several transfers, not the full 13-transfer target. Each progress cycle transfers at most one product; extra cycles
allow bounded survey/recovery work but do not guarantee a successful capture.

```bash
export DISPLAY=:0 WAYLAND_DISPLAY=wayland-1 QT_QPA_PLATFORM=xcb
bash website/scripts/record_docs_demo.bash
```

Optional recording bounds (defaults shown):

```bash
REC_TARGET_TRANSFERS=3 REC_MAX_CYCLES=12 REC_MAX_SECONDS=720 \
  bash website/scripts/record_docs_demo.bash
```

Camera pose / rate: `website/scripts/docs_side_camera.sdf`. Keep the 8 Hz camera and four-frame
sampling interval low: the coordinator needs timely wrist depth, and raw RGB frames grow quickly.
The script writes the MP4, poster, and loop only if it captured the campaign survey, at least two
successful transfers, at least 90 sampled frames, and an encode below 25 MB. The final encode
retains the full accepted capture, including late transfers; it is not trimmed to 90 seconds.
At the nominal 4x compression, a 720-second recording is about 180 seconds of video. A full
capture that exceeds 25 MB is refused, preserving the existing media.

## Air-motion fallback

`website/scripts/docs_demo_motion.py` plus `simulation.launch.py` gives a quick side-view motion
check. Use the restock path above for Introduction media.

## GUI fallback

To watch the same campaign in the GUI and screen-record it:

```bash
export DISPLAY=:0 WAYLAND_DISPLAY=wayland-1 QT_QPA_PLATFORM=xcb
just demo 3 gt:=true  # up to three progress cycles / transfers
nix-shell -p wf-recorder --run \
  'wf-recorder -o HDMI-A-2 -f /tmp/restocking-raw.mp4 -c libx264 -p crf=23 -r 20 -D -y'
# Stop with Ctrl-C, then compress:
ffmpeg -y -i /tmp/restocking-raw.mp4 -t 45 -vf scale=1280:-2 \
  -c:v libx264 -crf 23 -preset medium -an -movflags +faststart \
  website/static/media/restocking-demo.mp4
```

Poster / loop are written by the restock recorder; for a manual encode:

```bash
ffmpeg -y -ss 8 -i website/static/media/restocking-demo.mp4 -frames:v 1 \
  website/static/media/restocking-demo-poster.png
ffmpeg -y -ss 4 -t 5 -i website/static/media/restocking-demo.mp4 \
  -vf "fps=8,scale=640:-2" -loop 0 -an \
  website/static/media/restocking-demo-loop.webp
```

Notes:

- The recorder uses the shared domain/partition lease and reaps only its own tagged orphans.
- The recorder acquires the shared simulator slot with a one-slot limit before launching.
  Keep other simulator campaigns serialized under the same lease convention.
- Captures use a fresh `/tmp/combeanie-docs-rec.*` directory. `REC_DIR` may name a new directory, never an existing capture.
- Check the launch arguments without ROS or a campaign:
  `bash website/scripts/record_docs_demo_inner_restock.bash --print-launch`.
- `REC_TARGET_TRANSFERS` must be at least two and no greater than `REC_MAX_CYCLES`.
- The deadline bounds recording after the survey starts; startup has separate bounded waits.
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
