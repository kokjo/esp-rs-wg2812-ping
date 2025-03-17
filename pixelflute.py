#!/usr/bin/env python3
from argparse import ArgumentParser
from socket import socket, AF_INET, SOCK_DGRAM
from struct import pack
from time import sleep
from itertools import cycle

def chunks(n, l): return [l[i:i+n] for i in range(0, len(l), n)]

def send_pixels(sock, pixels):
    for chunk in chunks(150, list(enumerate(pixels))):
        packet = pack("<BB", 0, 1)
        packet += b"".join(pack("<HHBBB", i, 0, *pixel) for i, pixel in chunk)
        try:
            sock.send(packet)
        except OSError as e:
            print(f"Failed to send pixel packet: {e}")

def fader(colors, brightness, speed=500):
    prev_color = (0x00, 0x00, 0x00)
    for next_color in cycle(colors):
        for i in range(speed):
            t = i / speed
            yield (
                int(brightness*((1-t)*prev_color[0] + t*next_color[0])),
                int(brightness*((1-t)*prev_color[1] + t*next_color[1])),
                int(brightness*((1-t)*prev_color[2] + t*next_color[2])),
            )
        prev_color = next_color

def parse_args():
    p = ArgumentParser()
    p.add_argument("--num-pixels", default=300, type=int)
    p.add_argument("--brightness", default=0.2, type=float)
    p.add_argument("--sleep-time", default=0.01, type=float)
    p.add_argument("--fade-speed", default=500, type=int)
    p.add_argument("host")
    p.add_argument("port", type=int)
    return p.parse_args()

def main():
    args = parse_args()

    sock = socket(AF_INET, SOCK_DGRAM)
    sock.connect((args.host, args.port))

    colors = [
        (0xff, 0x00, 0x00),
        (0xff, 0xff, 0x00),
        (0x00, 0xff, 0x00),
        (0x00, 0xff, 0xff),
        (0x00, 0x00, 0xff),
        (0xff, 0x00, 0xff),
    ]
    pixels = [(0x00, 0x00, 0x00)] * args.num_pixels
    for pixel in fader(colors, args.brightness, args.fade_speed):
        pixels = pixels[1:] + [pixel]
        send_pixels(sock, pixels)
        sleep(args.sleep_time)

if __name__ == "__main__": main()
