#!/usr/bin/env python3

import rospy
import numpy as np
import csv
import time

from audio_common_msgs.msg import AudioData
from yamnet_coral.msg import AudioClassification
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters import common

# ─────────────────────────────────────────────────────────────
# Audio / model constants
# ─────────────────────────────────────────────────────────────
SAMPLE_RATE = 16000
PATCH_SAMPLES = 15360  # 0.96s @ 16kHz

PATCH_FRAMES = 96
MEL_BANDS = 64

HOP_SECONDS = 0.010
WINDOW_SECONDS = 0.025

HPF_CUTOFF = 80.0
PRE_EMPHASIS = 0.97
NOISE_GATE_RMS = 0.002

# ─────────────────────────────────────────────────────────────
# Globals
# ─────────────────────────────────────────────────────────────
buffer = np.array([], dtype=np.float32)
interp = None
labels = None
input_details = None
output_details = None
pub = None


# ─────────────────────────────────────────────────────────────
# Labels
# ─────────────────────────────────────────────────────────────
def load_labels(path):
    out = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            out.append(row["display_name"])
    return out


# ─────────────────────────────────────────────────────────────
# Filters
# ─────────────────────────────────────────────────────────────
def high_pass_filter(audio, cutoff_hz, sr):
    rc = 1.0 / (2.0 * np.pi * cutoff_hz)
    dt = 1.0 / sr
    alpha = rc / (rc + dt)

    y = np.zeros_like(audio)
    y[0] = audio[0]

    for i in range(1, len(audio)):
        y[i] = alpha * (y[i-1] + audio[i] - audio[i-1])

    return y


def pre_emphasis(audio, coeff=0.97):
    return np.append(audio[0], audio[1:] - coeff * audio[:-1])


def noise_gate(audio):
    rms = np.sqrt(np.mean(audio ** 2))
    return rms >= NOISE_GATE_RMS, rms


def apply_filters(audio):
    audio = high_pass_filter(audio, HPF_CUTOFF, SAMPLE_RATE)
    audio = pre_emphasis(audio, PRE_EMPHASIS)

    peak = np.max(np.abs(audio))
    if peak > 0:
        audio /= peak

    return audio


# ─────────────────────────────────────────────────────────────
# Spectrogram
# ─────────────────────────────────────────────────────────────
def stft_magnitude(signal, fft_length, hop_length, win_length):
    frames = []
    window = np.hanning(win_length).astype(np.float32)

    for start in range(0, len(signal) - win_length + 1, hop_length):
        frame = signal[start:start + win_length] * window
        spec = np.abs(np.fft.rfft(frame, n=fft_length))
        frames.append(spec)

    return np.array(frames)


def linear_to_mel(spec, sr, n_mels=64, fmin=125.0, fmax=7500.0):
    num_freq = spec.shape[1]

    mel_low = 2595 * np.log10(1 + fmin / 700)
    mel_high = 2595 * np.log10(1 + fmax / 700)

    mel_points = np.linspace(mel_low, mel_high, n_mels + 2)
    hz_points = 700 * (10 ** (mel_points / 2595) - 1)

    bins = np.floor((2 * num_freq) * hz_points / sr).astype(int)

    fb = np.zeros((num_freq, n_mels), dtype=np.float32)

    for m in range(1, n_mels + 1):
        l, c, r = bins[m-1], bins[m], bins[m+1]

        for k in range(l, c):
            if c != l:
                fb[k, m-1] = (k - l) / (c - l)

        for k in range(c, r):
            if r != c:
                fb[k, m-1] = (r - k) / (r - c)

    return np.dot(spec, fb)


def audio_to_spectrogram(audio):
    fft_length = int(WINDOW_SECONDS * SAMPLE_RATE)
    hop_length = int(HOP_SECONDS * SAMPLE_RATE)

    spec = stft_magnitude(audio, fft_length, hop_length, fft_length)
    mel = linear_to_mel(spec, SAMPLE_RATE, MEL_BANDS)
    log_mel = np.log(mel + 0.001)

    if log_mel.shape[0] < PATCH_FRAMES:
        pad = PATCH_FRAMES - log_mel.shape[0]
        log_mel = np.pad(log_mel, ((0, pad), (0, 0)), mode='constant')
    else:
        log_mel = log_mel[:PATCH_FRAMES]

    return log_mel.astype(np.float32)


# ─────────────────────────────────────────────────────────────
# Model
# ─────────────────────────────────────────────────────────────
def load_model():
    global interp, labels, input_details, output_details

    model_path = rospy.get_param("~model")
    label_path = rospy.get_param("~labels")

    interp = make_interpreter(model_path)
    interp.allocate_tensors()

    input_details = interp.get_input_details()
    output_details = interp.get_output_details()

    labels = load_labels(label_path)

    rospy.loginfo("Model loaded")


def classify(chunk):
    global pub

    active, rms = noise_gate(chunk)
    if not active:
        return

    processed = apply_filters(chunk)
    spec = audio_to_spectrogram(processed)

    data = np.expand_dims(spec, axis=0)

    # quantization
    if input_details[0]["dtype"] == np.int8:
        scale, zp = input_details[0]["quantization"]
        data = (data / scale + zp).astype(np.int8)

    common.set_input(interp, data)
    interp.invoke()

    scores = common.output_tensor(interp, 0).flatten()

    if output_details[0]["dtype"] == np.int8:
        scale, zp = output_details[0]["quantization"]
        scores = (scores.astype(np.float32) - zp) * scale

    idx = int(np.argmax(scores))

    msg = AudioClassification()
    msg.label = labels[idx]
    msg.confidence = float(scores[idx])
    msg.rms = float(rms)
    msg.stamp = rospy.Time.now()

    pub.publish(msg)


# ─────────────────────────────────────────────────────────────
# Audio callback (ROS)
# ─────────────────────────────────────────────────────────────
def audio_cb(msg):
    global buffer

    pcm = np.frombuffer(bytes(msg.data), dtype=np.int16)
    pcm = pcm.astype(np.float32) / 32768.0

    buffer = np.concatenate([buffer, pcm])

    while len(buffer) >= PATCH_SAMPLES:
        chunk = buffer[:PATCH_SAMPLES]

        # 50% overlap
        buffer = buffer[PATCH_SAMPLES // 2:]

        classify(chunk)


# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    rospy.init_node("coral_yamnet")

    load_model()

    pub = rospy.Publisher(
        "/audio/classification",
        AudioClassification,
        queue_size=10
    )

    rospy.Subscriber(
        "/audio",
        AudioData,
        audio_cb,
        queue_size=1
    )

    rospy.loginfo("YAMNet ROS node started")
    rospy.spin()
