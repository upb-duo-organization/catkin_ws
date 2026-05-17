#!/usr/bin/env python3

import rospy
import numpy as np
import csv
import threading

from audio_common_msgs.msg import AudioData
from yamnet_coral.msg import AudioClassification
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters import common

SAMPLE_RATE = 16000

PATCH_SAMPLES = 10240      # ~0.64s latency
HOP_SAMPLES = PATCH_SAMPLES // 2

WINDOW_SECONDS = 0.025
HOP_SECONDS = 0.010

MEL_BANDS = 64
FFT_LENGTH = int(WINDOW_SECONDS * SAMPLE_RATE)

HPF_CUTOFF = 80.0
PRE_EMPHASIS = 0.97
NOISE_GATE_RMS = 0.002

CONF_THRESHOLD = 0.5


buffer = np.zeros(PATCH_SAMPLES * 4, dtype=np.float32)
buf_idx = 0
buf_lock = threading.Lock()

interp = None
labels = None
input_details = None
output_details = None
pub = None

mel_fb = None


def load_labels(path):
    out = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            out.append(row["display_name"])
    return out


def high_pass_filter(x, cutoff, sr):
    rc = 1.0 / (2 * np.pi * cutoff)
    dt = 1.0 / sr
    alpha = rc / (rc + dt)

    y = np.empty_like(x)
    y[0] = x[0]

    for i in range(1, len(x)):
        y[i] = alpha * (y[i-1] + x[i] - x[i-1])

    return y


def pre_emphasis(x, coeff=0.97):
    return np.concatenate(([x[0]], x[1:] - coeff * x[:-1]))


def noise_gate(x):
    rms = np.sqrt(np.mean(x * x))
    return rms >= NOISE_GATE_RMS, rms


def apply_filters(x):
    x = high_pass_filter(x, HPF_CUTOFF, SAMPLE_RATE)
    x = pre_emphasis(x, PRE_EMPHASIS)

    peak = np.max(np.abs(x))
    if peak > 0:
        x = x / peak

    return x


def stft_magnitude(signal):
    win_length = FFT_LENGTH
    hop_length = int(HOP_SECONDS * SAMPLE_RATE)
    window = np.hanning(win_length).astype(np.float32)

    frames = []
    for start in range(0, len(signal) - win_length + 1, hop_length):
        frame = signal[start:start + win_length] * window
        spec = np.abs(np.fft.rfft(frame))
        frames.append(spec)

    return np.array(frames, dtype=np.float32)


def build_mel_filterbank(sr, n_fft, n_mels=64, fmin=125.0, fmax=7500.0):
    mel_low = 2595 * np.log10(1 + fmin / 700)
    mel_high = 2595 * np.log10(1 + fmax / 700)

    mel_points = np.linspace(mel_low, mel_high, n_mels + 2)
    hz = 700 * (10 ** (mel_points / 2595) - 1)

    bins = np.floor((n_fft + 1) * hz / sr).astype(int)

    fb = np.zeros((n_fft // 2 + 1, n_mels), dtype=np.float32)

    for m in range(1, n_mels + 1):
        l, c, r = bins[m-1], bins[m], bins[m+1]

        if c > l:
            fb[l:c, m-1] = (np.arange(l, c) - l) / (c - l)

        if r > c:
            fb[c:r, m-1] = (r - np.arange(c, r)) / (r - c)

    return fb


def to_mel(spec):
    return np.dot(spec, mel_fb)


def audio_to_features(audio):
    spec = stft_magnitude(audio)
    mel = to_mel(spec)
    log_mel = np.log(mel + 1e-3)

    if log_mel.shape[0] < 96:
        pad = 96 - log_mel.shape[0]
        log_mel = np.pad(
            log_mel,
            ((0, pad), (0, 0)),
            mode="constant"
        )
    else:
        log_mel = log_mel[:96]

    return log_mel.astype(np.float32)


def load_model():
    global interp, labels, input_details, output_details, mel_fb

    model_path = rospy.get_param("~model")
    label_path = rospy.get_param("~labels")

    interp = make_interpreter(model_path)
    interp.allocate_tensors()

    input_details = interp.get_input_details()
    output_details = interp.get_output_details()

    labels = load_labels(label_path)
    mel_fb = build_mel_filterbank(SAMPLE_RATE, FFT_LENGTH)

    rospy.loginfo("Model loaded")


def classify(chunk):
    global pub

    active, rms = noise_gate(chunk)
    if not active:
        return

    chunk = apply_filters(chunk)
    features = audio_to_features(chunk)
    features = np.expand_dims(features, 0)

    # quantization
    if input_details[0]["dtype"] == np.int8:
        scale, zp = input_details[0]["quantization"]
        features = (features / scale + zp).astype(np.int8)

    common.set_input(interp, features)
    interp.invoke()

    scores = common.output_tensor(interp, 0).flatten()

    if output_details[0]["dtype"] == np.int8:
        scale, zp = output_details[0]["quantization"]
        scores = (scores.astype(np.float32) - zp) * scale

    idx = int(np.argmax(scores))
    conf = float(scores[idx])

    if conf < CONF_THRESHOLD:
        return

    msg = AudioClassification()
    msg.label = labels[idx]
    msg.confidence = conf
    msg.rms = float(rms)
    msg.stamp = rospy.Time.now()

    pub.publish(msg)


def audio_cb(msg):
    global buffer, buf_idx

    pcm = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

    with buf_lock:
        for sample in pcm:
            buffer[buf_idx] = sample
            buf_idx = (buf_idx + 1) % len(buffer)

        # only run inference if enough data
        if buf_idx >= PATCH_SAMPLES:
            if buf_idx + PATCH_SAMPLES <= len(buffer):
                chunk = buffer[buf_idx-PATCH_SAMPLES:buf_idx].copy()
            else:
                chunk = np.concatenate(
                    (buffer[buf_idx-PATCH_SAMPLES:], buffer[:buf_idx])
                ).copy()

            classify(chunk)


if __name__ == "__main__":
    rospy.init_node("coral_yamnet")

    load_model()

    pub = rospy.Publisher(
        "/audio/classification",
        AudioClassification,
        queue_size=10
    )

    rospy.Subscriber("/audio", AudioData, audio_cb, queue_size=1)

    rospy.loginfo("YAMNet node running (stable version)")
    rospy.spin()
