# Binary instance mask for exactly one detection, encoded as run lengths in
# row-major order over that detection's bounding box. The first run is
# background and runs alternate background/foreground. Run lengths sum to
# width * height, and width/height must equal the extent of the bounding box
# that carries this mask, so a mask can never be applied to a different box.
#
# This message has no header. A mask is only delivered inside the ObjectDetection it segments,
# which is delivered inside the single PerceptionFrame that owns the stamp and frame for the
# whole acquisition.

uint16 width
uint16 height
uint32[] rle_counts
float32 score
