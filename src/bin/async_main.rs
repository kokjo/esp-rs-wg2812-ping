#![no_std]
#![no_main]

use alloc::collections::BTreeMap;
use embassy_executor::Spawner;
use embassy_net::udp::UdpSocket;
use embassy_net::Stack;
use embassy_net::{udp::PacketMetadata, IpListenEndpoint, Runner, StackResources};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::dma::{self, DmaChannelFor};
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::gpio::{Input, Pull};
use esp_hal::peripheral::Peripheral;
use esp_hal::rng::Rng;
use esp_hal::spi::{self};
use esp_hal::time::RateExtU32;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{dma_buffers, Async};
use esp_wifi::wifi::{Configuration, WifiEvent};
use esp_wifi::{
    init,
    wifi::{self, ClientConfiguration, WifiController, WifiDevice, WifiStaDevice},
    EspWifiController,
};
use log::info;
use ws2812_ping::mdns::mdns_task;
use ws2812_ping::mk_static;
use ws2812_ping::net::{start_net, wait_for_dhcp, wait_for_link};
use ws2812_ping::wifi::start_wifi_sta;

extern crate alloc;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const HOSTNAME: &str = env!("HOSTNAME");

type RGB = [u8; 3];

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Event {
    Ping,
    Click,
}

const NUM_EVENTS: usize = 10;

type EventChannel = channel::Channel<NoopRawMutex, Event, NUM_EVENTS>;
type EventSender = channel::Sender<'static, NoopRawMutex, Event, NUM_EVENTS>;
type EventReceiver = channel::Receiver<'static, NoopRawMutex, Event, NUM_EVENTS>;

const NUM_PIXELS: usize = 600;

struct WS2812Controller<'d> {
    spidma: spi::master::SpiDmaBus<'d, Async>,
}

impl<'d> WS2812Controller<'d> {
    pub fn new(
        spi: impl Peripheral<P = impl spi::master::PeripheralInstance> + 'd,
        pin: impl Peripheral<P = impl PeripheralOutput> + 'd,
        channel: impl Peripheral<P = impl DmaChannelFor<spi::AnySpi>> + 'd,
    ) -> Self {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            dma_buffers!(3 * 4 * NUM_PIXELS);
        let dma_rx_buf = dma::DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = dma::DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        Self {
            spidma: spi::master::Spi::new(
                spi,
                spi::master::Config::default().with_frequency(3200u32.kHz()),
            )
            .unwrap()
            .with_mosi(pin)
            .with_dma(channel)
            .with_buffers(dma_rx_buf, dma_tx_buf)
            .into_async(),
        }
    }

    pub async fn send_pixels(
        &mut self,
        pixels: impl Iterator<Item = RGB>,
    ) -> Result<(), spi::Error> {
        let mut pulses = [0u8; 3 * 4 * NUM_PIXELS];
        let mut bitoff = 0;
        for pixel in pixels {
            let pixel = ((pixel[1] as u32) << 16) | ((pixel[0] as u32) << 8) | (pixel[2] as u32);
            for j in 0..24 {
                let bitpulse = if (pixel >> (23 - j)) & 1 == 1 {
                    0b0111
                } else {
                    0b0001
                };
                for k in 0..4 {
                    let bit = (bitpulse >> k) & 1;
                    pulses[bitoff / u8::BITS as usize] |= bit << (7 - (bitoff % u8::BITS as usize));
                    bitoff += 1;
                }
            }
        }
        self.spidma.write_async(&pulses).await
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_160MHz);
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);
    info!("Embassy initialized!");

    let events = mk_static!(EventChannel, EventChannel::new());

    let rng = Rng::new(peripherals.RNG);

    let ws2812_ctrl = mk_static!(
        WS2812Controller<'_>,
        WS2812Controller::new(peripherals.SPI2, peripherals.GPIO4, peripherals.DMA_CH0)
    );

    let pixels = mk_static!(
        Mutex<NoopRawMutex, [RGB; NUM_PIXELS]>,
        Mutex::new([RGB::default(); NUM_PIXELS])
    );

    spawner.must_spawn(ws2812_task(events.receiver(), ws2812_ctrl, pixels));

    // Start button task
    let btn = mk_static!(Input, Input::new(peripherals.GPIO9, Pull::None));
    spawner.must_spawn(btn_task(btn, events.sender()));

    // start wifi stack
    let wifi_sta = start_wifi_sta(
        spawner,
        peripherals.TIMG0,
        rng,
        peripherals.RADIO_CLK,
        peripherals.WIFI,
        SSID,
        PASSWORD,
    );

    // start network stack
    let stack = start_net(spawner, rng, wifi_sta);

    wait_for_link(stack).await;
    wait_for_dhcp(stack).await;

    spawner.must_spawn(pixelping(stack, events.sender()));
    spawner.must_spawn(pixelflute(stack, pixels));
    spawner.must_spawn(mdns_task(stack, HOSTNAME));
}

#[embassy_executor::task]
async fn pixelping(stack: Stack<'static>, events: EventSender) {
    let mut rx_data = [0u8; 4096];
    let mut rx_meta = [PacketMetadata::EMPTY; 2];

    let mut tx_data = [0u8; 4096];
    let mut tx_meta = [PacketMetadata::EMPTY; 2];

    let mut sock = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_data,
        &mut tx_meta,
        &mut tx_data,
    );

    sock.bind(IpListenEndpoint {
        addr: None,
        port: 1337,
    })
    .unwrap();

    let mut packet = [0u8; 1024];

    loop {
        let (size, meta) = sock.recv_from(&mut packet).await.unwrap();
        info!("Got UDP ping");
        sock.send_to(&packet[..size], meta.endpoint).await.unwrap();
        events.send(Event::Ping).await;
    }
}

#[embassy_executor::task]
async fn ws2812_task(
    events: EventReceiver,
    ws2812_ctrl: &'static mut WS2812Controller<'static>,
    pixels: &'static Mutex<NoopRawMutex, [RGB; NUM_PIXELS]>,
) {
    let mut lights: BTreeMap<i32, [u8; 3]> = BTreeMap::new();

    let mut next_color = 0;

    let colors = [
        [0xff, 0x00, 0x00],
        [0x00, 0xff, 0x00],
        [0x00, 0x00, 0xff],
        [0xff, 0xff, 0x00],
        [0x00, 0xff, 0xff],
        [0xff, 0x00, 0xff],
        [0xff, 0xff, 0xff],
        [0xff, 0x20, 0x20],
        [0x20, 0xff, 0x20],
        [0x20, 0x20, 0xff],
        [0xff, 0xff, 0xff],
    ];

    loop {
        let pixels_copy = *pixels.lock().await;

        let data = (0..NUM_PIXELS as i32)
            .map(|i| lights.get(&i).cloned().unwrap_or(pixels_copy[i as usize]));

        ws2812_ctrl.send_pixels(data).await.unwrap();

        lights = lights
            .into_iter()
            .filter_map(|(i, rgb)| (i < NUM_PIXELS as i32).then_some((i + 1, rgb)))
            .collect();

        while let Ok(event) = events.try_receive() {
            match event {
                Event::Ping => {
                    let color = colors[next_color % colors.len()];
                    next_color += 1;
                    lights.insert(0, color);
                }
                Event::Click => {
                    pixels
                        .lock()
                        .await
                        .iter_mut()
                        .for_each(|pixel| *pixel = [0, 0, 0]);
                }
            }
        }

        Timer::after(Duration::from_millis(5)).await;
    }
}

#[embassy_executor::task]
async fn btn_task(btn: &'static mut Input<'static>, events: EventSender) {
    loop {
        btn.wait_for_falling_edge().await;
        info!("Button clicked!");
        events.send(Event::Click).await;
    }
}

#[embassy_executor::task]
async fn pixelflute(
    stack: Stack<'static>,
    pixels: &'static Mutex<NoopRawMutex, [RGB; NUM_PIXELS]>,
) {
    let mut rx_data = [0u8; 4096];
    let mut rx_meta = [PacketMetadata::EMPTY; 2];

    let mut tx_data = [0u8; 4096];
    let mut tx_meta = [PacketMetadata::EMPTY; 2];

    let mut sock = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_data,
        &mut tx_meta,
        &mut tx_data,
    );

    sock.bind(IpListenEndpoint {
        addr: None,
        port: 4242,
    })
    .unwrap();

    let mut packet = [0; 1500];

    loop {
        let (size, meta) = sock.recv_from(&mut packet).await.unwrap();
        if size < 2 {
            info!("Pixelflute packet too short");
            continue;
        }

        let format = packet[0];
        let version = packet[1];

        info!(
            "Got pixelflute packet, from: {} size: {}, version: {}, format: {}",
            meta.endpoint, size, version, format
        );

        if format != 0 && version != 1 {
            info!("This Pixel flute only supports version 1 with format 0(xyrgb 16:16:8:8:8)");
            continue;
        }

        for pixel in packet[2..size].chunks_exact(7) {
            let x = u16::from_le_bytes(pixel[0..2].try_into().unwrap()) as usize;
            let y = u16::from_le_bytes(pixel[2..4].try_into().unwrap()) as usize;
            let rgb = [pixel[4], pixel[5], pixel[6]];
            if let Some(pixel) = pixels.lock().await.get_mut(x + y * 10) {
                *pixel = rgb;
            }
        }
    }
}
