from argparse import ArgumentParser
from socket import socket, AF_INET, SOCK_DGRAM
from struct import pack
from time import sleep

def chunks(n, l): return [l[i:i+n] for i in range(0, len(l), n)]

def parse_args():
    p = ArgumentParser()
    p.add_argument("--num-pixels", default=300, type=int)
    p.add_argument("--max-brightness", default=50, type=int)
    p.add_argument("host")
    p.add_argument("port", type=int)
    return p.parse_args()

def main():
    args = parse_args()

    pixels = [
        (int(args.max_brightness * (1 - s)), 0, int(args.max_brightness * s))
        for i, s in ((i, i / (args.num_pixels)) for i in range(args.num_pixels))
    ]

    sock = socket(AF_INET, SOCK_DGRAM)
    sock.connect((args.host, args.port))

    print(pixels)

    while True:
        for chunk in chunks(100, list(enumerate(pixels))):
            packet = pack("<BB", 0, 1)
            packet += b"".join(pack("<HHBBB", i, 0, *pixel) for i, pixel in chunk)
            sock.send(packet)
        pixels = pixels[1:] + pixels[:1]
        sleep(0.01)

if __name__ == "__main__": main()
