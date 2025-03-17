#![no_std]
#![no_main]

use core::net::{Ipv4Addr, Ipv6Addr};

use alloc::collections::BTreeMap;
use edge_mdns::buf::VecBufAccess;
use edge_mdns::domain::base::Ttl;
use edge_mdns::host::Host;
use edge_mdns::io::{Mdns, DEFAULT_SOCKET};
use edge_mdns::HostAnswersMdnsHandler;
use edge_nal::{UdpBind as _, UdpSplit as _};
use edge_nal_embassy::{Udp, UdpBuffers};
use embassy_executor::Spawner;
use embassy_net::udp::UdpSocket;
use embassy_net::Stack;
use embassy_net::{udp::PacketMetadata, IpListenEndpoint, Runner, StackResources};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
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

extern crate alloc;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

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

    let mut rng = Rng::new(peripherals.RNG);

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

    // Setup and spawn wifi stack
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng, peripherals.RADIO_CLK).unwrap()
    );

    let (wifi_sta, wifi_ctrl) =
        wifi::new_with_mode(esp_wifi_ctrl, peripherals.WIFI, WifiStaDevice).unwrap();
    spawner.must_spawn(wifi_ctrl_task(wifi_ctrl));

    // Setup and spawn Network stack
    let (stack, runner) = embassy_net::new(
        wifi_sta,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<5>, StackResources::<5>::new()),
        (rng.random() as u64) << 32 | rng.random() as u64,
    );
    spawner.must_spawn(net_task(runner));

    info!("Waiting for link...");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after_millis(500).await;
    }

    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after_millis(500).await;
    }

    spawner.must_spawn(pixelping(stack, events.sender()));
    spawner.must_spawn(pixelflute(stack, pixels));
    spawner.must_spawn(mdns_task(stack, "pixelflute"));
}

#[embassy_executor::task]
async fn mdns_task(stack: Stack<'static>, hostname: &'static str) {
        let mdns_udp_buff = UdpBuffers::<1, 1024, 1024, 1>::new();
    let mdns_udp = Udp::new(stack, &mdns_udp_buff);
    let mut mdns_udp_sock = mdns_udp.bind(DEFAULT_SOCKET).await.unwrap();
    let (mdns_rx, mdns_tx) = mdns_udp_sock.split();
    let (recv_buf, send_buf) = (
        VecBufAccess::<NoopRawMutex, 1500>::new(),
        VecBufAccess::<NoopRawMutex, 1500>::new(),
    );

    let signal = Signal::<NoopRawMutex, ()>::new();

    let mdns = Mdns::<NoopRawMutex, _, _, _, _>::new(
        Some(Ipv4Addr::UNSPECIFIED),
        None,
        mdns_rx,
        mdns_tx,
        recv_buf,
        send_buf,
        |buf| buf.iter_mut().for_each(|x| *x = 0),
        &signal
    );

    loop {
        mdns.run(HostAnswersMdnsHandler::new(&Host {
            hostname: hostname,
            ipv4: stack.config_v4().unwrap().address.address(),
            ipv6: Ipv6Addr::UNSPECIFIED,
            ttl: Ttl::from_secs(60)
        })).await.unwrap();
    }

}

#[embassy_executor::task]
async fn wifi_ctrl_task(mut wifi_ctrl: WifiController<'static>) {
    wifi_ctrl
        .set_configuration(&Configuration::Client(ClientConfiguration {
            ssid: SSID.try_into().unwrap(),
            password: PASSWORD.try_into().unwrap(),
            ..Default::default()
        }))
        .unwrap();

    wifi_ctrl.start_async().await.unwrap();

    loop {
        match wifi_ctrl.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => info!("Failed to connect to wifi: {e:?}"),
        }
        if wifi_ctrl.is_connected().unwrap() {
            info!("Waiting for wifi to disconnect");
            wifi_ctrl.wait_for_event(WifiEvent::StaDisconnected).await;
        }
        Timer::after_secs(5).await
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static, WifiStaDevice>>) {
    runner.run().await
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
