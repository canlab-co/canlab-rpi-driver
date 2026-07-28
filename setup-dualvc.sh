#!/bin/bash
# setup-dualvc.sh - configure the rp1-cfe media pipeline for the canlab
# dual virtual-channel camera.
#
# Usage:
#   ./setup-dualvc.sh <mode>
#   ./setup-dualvc.sh          (prompts interactively if no mode given)
#
# Modes (VC0 is always 1920x1080):
#   vga    VC0 + VC1 640x480    (default canlab-downstream overlay)
#   qvga   VC0 + VC1 320x240    (dtoverlay=canlab-downstream,qvga)
#   single VC0 only (video0)    (VC1 left disabled; works with either overlay)
set -e

MODE="$1"

if [ -z "$MODE" ]; then
    echo "Select mode:"
    echo "  1) vga     (VC0 + VC1 640x480)"
    echo "  2) qvga    (VC0 + VC1 320x240)"
    echo "  3) single  (VC0 only)"
    read -rp "Enter 1/2/3 or vga/qvga/single: " CHOICE
    case "$CHOICE" in
        1|vga)    MODE="vga"    ;;
        2|qvga)   MODE="qvga"   ;;
        3|single) MODE="single" ;;
        *) echo "Invalid selection: $CHOICE"; exit 1 ;;
    esac
fi

case "$MODE" in
    vga)    VC1_W=640;  VC1_H=480  ;;
    qvga)   VC1_W=320;  VC1_H=240  ;;
    single) ;;  # VC1 dims not used; VC1 stays disabled
    *)
        echo "Unknown mode: $MODE (expected vga | qvga | single)"
        exit 1
        ;;
esac

if [ "$MODE" = "single" ]; then
    echo "mode: single (VC0 1920x1080 only)"
else
    echo "mode: $MODE (VC0 1920x1080, VC1 ${VC1_W}x${VC1_H})"
fi

MDEV=""
for m in /dev/media*; do
    if media-ctl -d "$m" -p 2>/dev/null | grep -q "model *rp1-cfe"; then
        MDEV=$m
        break
    fi
done

if [ -z "$MDEV" ]; then
    echo "rp1-cfe media device not found"
    exit 1
fi
echo "rp1-cfe: $MDEV"

CAMSD=$(media-ctl -d "$MDEV" -p | grep -oE "canlab [0-9]+-001a" | head -1)
if [ -z "$CAMSD" ]; then
    echo "canlab subdev not found"
    exit 1
fi
echo "sensor: $CAMSD"

# csi2 pad4 -> /dev/video0 (VC0), always enabled
media-ctl -d "$MDEV" -l "'csi2':4 -> 'rp1-cfe-csi2_ch0':0 [1]"

media-ctl -d "$MDEV" -V "'$CAMSD':0 [fmt:UYVY8_1X16/1920x1080 field:none]"
media-ctl -d "$MDEV" -V "'csi2':0 [fmt:UYVY8_1X16/1920x1080 field:none]"
media-ctl -d "$MDEV" -V "'csi2':4 [fmt:UYVY8_1X16/1920x1080 field:none]"

# pad1 (embedded/filler) is an IMMUTABLE sink link like pad2 (VC1) below -
# always mirror it to the sensor's own default (VC0 res) even though it's
# unused, or link_validate fails at STREAMON regardless of mode.
media-ctl -d "$MDEV" -V "'csi2':1 [fmt:UYVY8_1X16/1920x1080 field:none]"

v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=1920,height=1080,pixelformat=UYVY

if [ "$MODE" = "single" ]; then
    # csi2 pad6 -> /dev/video2 (VC1) stays disabled: VC1 is never captured.
    media-ctl -d "$MDEV" -l "'csi2':6 -> 'rp1-cfe-csi2_ch2':0 [0]"

    # pad2 (VC1) is, like pad1, an IMMUTABLE sink link from the sensor and
    # is included in pipeline validation regardless of whether it's
    # enabled downstream. Disabling the ch2 fan-out link alone is NOT
    # enough - csi2:2 must still be mirrored to whatever VC1 resolution
    # the sensor itself is currently reporting (depends on the vga/qvga
    # overlay param actually in use), or STREAMON fails with -EPIPE.
    SENSOR_VC1_FMT=$(media-ctl -d "$MDEV" -p \
        | sed -n "/entity [0-9]*: $CAMSD /,\$p" \
        | awk '/pad2: SOURCE/{getline; print; exit}' \
        | grep -oE "UYVY8_1X16/[0-9]+x[0-9]+")

    if [ -z "$SENSOR_VC1_FMT" ]; then
        echo "could not read sensor's VC1 (pad2) default format"
        exit 1
    fi
    echo "mirroring csi2:2 to sensor default: $SENSOR_VC1_FMT"
    media-ctl -d "$MDEV" -V "'csi2':2 [fmt:${SENSOR_VC1_FMT} field:none]"
else
    # csi2 pad6 -> /dev/video2 (VC1)
    media-ctl -d "$MDEV" -l "'csi2':6 -> 'rp1-cfe-csi2_ch2':0 [1]"

    media-ctl -d "$MDEV" -V "'$CAMSD':2 [fmt:UYVY8_1X16/${VC1_W}x${VC1_H} field:none]"
    media-ctl -d "$MDEV" -V "'csi2':2 [fmt:UYVY8_1X16/${VC1_W}x${VC1_H} field:none]"
    media-ctl -d "$MDEV" -V "'csi2':6 [fmt:UYVY8_1X16/${VC1_W}x${VC1_H} field:none]"

    v4l2-ctl -d /dev/video2 \
      --set-fmt-video=width=${VC1_W},height=${VC1_H},pixelformat=UYVY
fi

echo "done"
