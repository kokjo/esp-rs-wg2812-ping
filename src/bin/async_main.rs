#![no_std]
#![no_main]

use alloc::collections::BTreeMap;
use embassy_executor::Spawner;
use embassy_net::{udp::PacketMetadata, IpListenEndpoint, Runner, StackResources};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel};
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

#[derive(PartialEq, Eq, PartialOrd, Ord)]
enum Event {
    Ping,
    Click,
}

const NUM_PIXELS: usize = 300;

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
            dma_buffers!(3 * 3 * NUM_PIXELS);
        let dma_rx_buf = dma::DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = dma::DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        Self {
            spidma: spi::master::Spi::new(
                spi,
                spi::master::Config::default().with_frequency(2500u32.kHz()),
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
        pixels: impl Iterator<Item = [u8; 3]>,
    ) -> Result<(), spi::Error> {
        let mut pulses = [0u8; 3 * 3 * NUM_PIXELS];
        let mut bitoff = 0;
        for pixel in pixels {
            let pixel = ((pixel[1] as u32) << 16) | ((pixel[0] as u32) << 8) | (pixel[2] as u32);
            for j in 0..24 {
                let bitpulse = if (pixel >> j) & 1 == 1 { 0b110 } else { 0b100 };
                for k in 0..3 {
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

    let events = mk_static!(
        channel::Channel::<CriticalSectionRawMutex, Event, 10>,
        channel::Channel::<CriticalSectionRawMutex, Event, 10>::new()
    );

    let mut rng = Rng::new(peripherals.RNG);

    let ws2812_ctrl = mk_static!(
        WS2812Controller<'_>,
        WS2812Controller::new(peripherals.SPI2, peripherals.GPIO4, peripherals.DMA_CH0)
    );

    spawner.must_spawn(ws2812_task(events.receiver(), ws2812_ctrl, rng.clone()));

    // Start button task
    let btn = mk_static!(Input, Input::new(peripherals.GPIO9, Pull::None));
    spawner.must_spawn(btn_task(btn, events.sender()));

    // Setup and spawn wifi stack
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );

    let (wifi_sta, wifi_ctrl) =
        wifi::new_with_mode(esp_wifi_ctrl, peripherals.WIFI, WifiStaDevice).unwrap();
    spawner.must_spawn(wifi_ctrl_task(wifi_ctrl));

    // Setup and spawn Network stack
    let (stack, runner) = embassy_net::new(
        wifi_sta,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<3>, StackResources::<3>::new()),
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

    let mut rx_data = [0u8; 4096];
    let mut rx_meta = [PacketMetadata::EMPTY; 2];

    let mut tx_data = [0u8; 4096];
    let mut tx_meta = [PacketMetadata::EMPTY; 2];

    let mut sock = embassy_net::udp::UdpSocket::new(
        stack.clone(),
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
async fn ws2812_task(
    events: channel::Receiver<'static, CriticalSectionRawMutex, Event, 10>,
    ws2812_ctrl: &'static mut WS2812Controller<'static>,
    mut rng: Rng,
) {
    let mut lights: BTreeMap<i32, [u8; 3]> = BTreeMap::new();

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

    let mut dir = 1;

    loop {
        let pixels = (0..NUM_PIXELS as i32).map(|i| lights.get(&i).cloned().unwrap_or_default());

        ws2812_ctrl.send_pixels(pixels).await.unwrap();

        lights = lights
            .into_iter()
            .filter_map(|(i, rgb)| (0 <= i && i <= NUM_PIXELS as i32).then_some((i + dir, rgb)))
            .collect();

        let event = events.try_receive();

        if event == Ok(Event::Ping) || lights.is_empty() {
            let color = colors[(rng.random() as usize) % colors.len()];
            lights.insert((dir == 1).then_some(0).unwrap_or(NUM_PIXELS as i32), color);
        } else if event == Ok(Event::Click) {
            dir = -dir;
        }

        Timer::after(Duration::from_millis(1)).await;
    }
}

#[embassy_executor::task]
async fn btn_task(
    btn: &'static mut Input<'static>,
    events: channel::Sender<'static, CriticalSectionRawMutex, Event, 10>,
) {
    loop {
        btn.wait_for_falling_edge().await;
        info!("Button clicked!");
        events.send(Event::Click).await;
    }
}
