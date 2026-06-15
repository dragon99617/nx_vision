# Debug Studio

Run:

```bash
./scripts/run_debug_studio.sh
```

Panels:

- `main`: raw image with target, center, PnP, angle, and serial overlay.
- `binary`: thresholded black-frame mask.
- `edges`: Canny output.
- `combined`: morphology/edge combined mask.
- `contours`: candidate contours and selected rectangle.
- `perspective`: rectified target view.
- `pose`: projected PnP axes.
- `serial`: actual packet sent to the gimbal.

Keys:

- `q` or `Esc`: quit.
- `s`: save snapshot.

