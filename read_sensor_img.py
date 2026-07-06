# ethernet : 10.0.2.123:5004
# wifi : 10.206.50.47:5004
import re
import sys
import time

import cv2

# --- Stream configuration -------------------------------------------------
# The sensor is configured to send the camera stream by UNICAST directly to this
# machine's wired IP (Camera Configuration -> Stream -> Method: Unicast,
# Destination: <this-host>:5004). We therefore just listen on the UDP port; there
# is no multicast group to join.
PORT = 5004

# This GStreamer pipeline listens for the unicast RTP/H.264 stream on PORT.
# With the OpenCV backend the practical symptom of "no data" is simply that
# cap.read() blocks / returns nothing -- so we guard with a wall-clock timeout below.
gst_pipeline = (
    f"udpsrc port={PORT} "
    'caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, '
    'encoding-name=(string)H264, payload=(int)96" '
    # rtph264depay strips RTP -> h264parse frames it -> avdec_h264 decodes.
    # Requires: gstreamer1.0-plugins-good (depay), -plugins-bad (h264parse),
    # -libav (avdec_h264).
    "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert "
    "! appsink drop=true max-buffers=1"
)

# How long to wait for the very first frame before giving up (seconds).
FIRST_FRAME_TIMEOUT = 10.0


# --- Diagnostic 1: is OpenCV even built with GStreamer? -------------------
def check_gstreamer_support():
    info = cv2.getBuildInformation()
    match = re.search(r"GStreamer:\s*(.*)", info)
    status = match.group(1).strip() if match else "unknown"
    print(f"[diag] OpenCV version : {cv2.__version__}")
    print(f"[diag] GStreamer      : {status}")
    if not match or not status.upper().startswith("YES"):
        print("[diag] ERROR: This OpenCV build has NO GStreamer support.")
        print("       The GStreamer pipeline string will be treated as a file path and fail.")
        print("       Fix: install an OpenCV built with GStreamer (e.g. apt's python3-opencv,")
        print("       or build from source with -D WITH_GSTREAMER=ON).")
        return False
    return True


def main():
    if not check_gstreamer_support():
        sys.exit(1)

    print(f"[diag] Listening for unicast RTP/H.264 on UDP port {PORT} ...")
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Error: Could not open the GStreamer pipeline.")
        print("[diag] The pipeline failed to construct/start. Likely causes:")
        print("       - a GStreamer plugin is missing. Check the H.264 decode chain:")
        print("           gst-inspect-1.0 rtph264depay h264parse avdec_h264")
        print("         Install any missing pieces with:")
        print("           sudo apt install gstreamer1.0-libav gstreamer1.0-plugins-bad")
        print("       - the caps string is malformed")
        print("       This is NOT usually a 'no packets' problem -- that shows up as a")
        print("       timeout below, not here.")
        sys.exit(1)

    print("Successfully connected to the Vision Navigator camera stream!")
    print(f"[diag] Waiting up to {FIRST_FRAME_TIMEOUT:.0f}s for the first frame ...")

    got_first_frame = False
    start = time.monotonic()
    frame_count = 0

    while True:
        ret, frame = cap.read()

        if not ret:
            # Before the first frame this almost always means "no packets are
            # arriving", not a decode error.
            if not got_first_frame:
                waited = time.monotonic() - start
                if waited >= FIRST_FRAME_TIMEOUT:
                    print(f"[diag] TIMEOUT: no frame after {waited:.1f}s.")
                    print("       No H.264 stream is reaching this machine. Check:")
                    print(f"       - Is the sensor's Stream set to Unicast with Destination <this-host>:{PORT}?")
                    print("         (This machine's wired IP is DHCP-assigned and can change on reboot.)")
                    print("       - Is Fusion running on the sensor? (Streaming is active while it runs.)")
                    print("       - Are you on the same subnet / correct NIC as the sensor?")
                    print("       - Firewall blocking UDP? Test the raw stream with:")
                    print(f"           gst-launch-1.0 -v udpsrc port={PORT} "
                          '! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 '
                          "! rtph264depay ! decodebin ! autovideosink")
                    break
                # Keep polling until the timeout elapses.
                continue
            else:
                print("Failed to grab frame")
                break

        if not got_first_frame:
            got_first_frame = True
            h, w = frame.shape[:2]
            print(f"[diag] First frame received after {time.monotonic() - start:.2f}s "
                  f"({w}x{h}). Streaming -- press 'q' in the window to quit.")

        frame_count += 1

        # Display the resulting frame
        cv2.imshow("Xsens Vision Navigator Stream", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    print(f"[diag] Total frames displayed: {frame_count}")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
