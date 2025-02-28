#![no_std]
#![no_main]

use alloc::{collections::BTreeMap, vec::Vec};
use embassy_executor::Spawner;
use embassy_net::{udp::PacketMetadata, IpListenEndpoint, Runner, StackResources};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{riscv, Async};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::time::RateExtU32;
use esp_hal::rng::Rng;
use esp_hal::rmt::{self, PulseCode, Rmt, TxChannelAsync, TxChannelConfig, TxChannelCreatorAsync};
use esp_hal::interrupt::{software::SoftwareInterruptControl, Priority};
use esp_hal::gpio::{Input, Pull};
use esp_hal::clock::CpuClock;
use esp_hal_embassy::InterruptExecutor;
use esp_wifi::wifi::{Configuration, WifiEvent};
use log::info;
use esp_wifi::{
    init,
    wifi::{self, ClientConfiguration, WifiController, WifiDevice, WifiStaDevice},
    EspWifiController,
};

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

    // setup and spawn ws2812 task

    let rmt = Rmt::new(peripherals.RMT, 80.MHz()).unwrap().into_async();
    let channel = rmt.channel0.configure(peripherals.GPIO4, TxChannelConfig {
        clk_divider: 1,
        idle_output_level: false,
        idle_output: true,
        .. Default::default()
    }).unwrap();

    esp_hal::interrupt::enable(
        esp_hal::peripherals::Interrupt::RMT,
        Priority::Priority14
    ).unwrap();

    // let sw_inters = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    // let high_prio_executor = mk_static!(
    //     InterruptExecutor::<0>, InterruptExecutor::<0>::new(
    //     sw_inters.software_interrupt0
    // ));
    // let high_prio_spawner = high_prio_executor.start(Priority::Priority15);
    // high_prio_spawner.spawn(ws2812_task(events.receiver(), channel, rng.clone())).ok();
    spawner.spawn(ws2812_task(events.receiver(), channel, rng.clone())).ok();

    // Start button task
    let btn = mk_static!(Input, Input::new(peripherals.GPIO9, Pull::None));
    spawner.spawn(btn_task(btn, events.sender())).ok();

    // Setup and spawn wifi stack
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );

    let (wifi_sta, wifi_ctrl) = wifi::new_with_mode(
        esp_wifi_ctrl,
        peripherals.WIFI,
        WifiStaDevice
    ).unwrap();
    spawner.spawn(wifi_ctrl_task(wifi_ctrl)).ok();

    // Setup and spawn Network stack
    let (stack, runner) = embassy_net::new(
        wifi_sta,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        (rng.random() as u64) << 32 | rng.random() as u64
    );
    spawner.spawn(net_task(runner)).ok();

    // esp_hal::interrupt::enable(
    //     esp_hal::peripherals::Interrupt::WIFI_BB,
    //     Priority::Priority10
    // );

    esp_hal::interrupt::enable(
        esp_hal::peripherals::Interrupt::WIFI_MAC,
        Priority::Priority10
    );

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

    sock.bind(IpListenEndpoint { addr: None, port: 1337 }).unwrap();

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
    wifi_ctrl.set_configuration(
        &Configuration::Client(ClientConfiguration{
            ssid: SSID.try_into().unwrap(),
            password: PASSWORD.try_into().unwrap(),
            .. Default::default()
        })
    ).unwrap();

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
async fn ws2812_task(events: channel::Receiver<'static, CriticalSectionRawMutex, Event, 10>, mut channel: rmt::Channel<Async, 0>, mut rng: Rng) {
    const NUM_LEDS: i32 = 300;

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

    const NUM_SAMPLES: usize = 16;
    let mut mins = [0u64; NUM_SAMPLES];
    let mut avgs = [0u64; NUM_SAMPLES];
    let mut maxs = [0u64; NUM_SAMPLES];

    for i in 0.. {
        let pixels = (0..NUM_LEDS).map(|i| lights.get(&i).cloned().unwrap_or_default());

        let (min, avg, max) = send_pixels(&mut channel, pixels).await.unwrap();

        mins[i % mins.len()] = min;
        avgs[i % mins.len()] = avg;
        maxs[i % maxs.len()] = max;
        if i % NUM_SAMPLES == 0 {
            info!("num_samples = {} min_min = {} avg_min = {} max_min = {} min_avg = {} avg_avg = {} max_avg = {} min_max = {} avg_max = {} max_max = {}",
                NUM_SAMPLES,
                mins.iter().min().unwrap(),
                mins.iter().sum::<u64>() / (mins.len() as u64),
                mins.iter().max().unwrap(),

                avgs.iter().min().unwrap(),
                avgs.iter().sum::<u64>() / (avgs.len() as u64),
                avgs.iter().max().unwrap(),

                maxs.iter().min().unwrap(),
                maxs.iter().sum::<u64>() / (maxs.len() as u64),
                maxs.iter().max().unwrap()
            );
        }

        lights = lights.into_iter().filter_map(|(i, rgb)| (0 <= i && i <= NUM_LEDS).then_some((i + dir, rgb))).collect();

        let event = events.try_receive();        

        if event == Ok(Event::Ping) || lights.is_empty() {
            let color = colors[(rng.random() as usize) % colors.len()];
            lights.insert((dir == 1).then_some(0).unwrap_or(NUM_LEDS), color);
        } else if event == Ok(Event::Click) {
            dir = -dir;
        }

//        info!("{:?}", lights);
        
        Timer::after(Duration::from_millis(10)).await;
    }
}

fn make_pixel_pulses(pixel: [u8; 3]) -> [u32; 25] {
    let mut pulses = [0u32; 25];
    let pixel = ((pixel[1] as u32) << 16) | ((pixel[0] as u32) << 8) | (pixel[2] as u32);
    for i in 0..24 {
        if (pixel >> (23-i)) & 1 == 0 {
            pulses[i] = PulseCode::new(true, 32, false, 68);
        } else {
            pulses[i] = PulseCode::new(true, 68, false, 32);
        }
    }
    pulses
}

async fn send_pixels(channel: &mut impl TxChannelAsync, pixels: impl Iterator<Item = [u8; 3]>) -> Result<(u64, u64, u64), esp_hal::rmt::Error> {
    let mut times = Vec::with_capacity(500);
    for pixel in pixels {
        times.push(embassy_time::Instant::now().as_ticks());
        channel.transmit(&make_pixel_pulses(pixel)).await.unwrap();
    }

    let (mut sum, mut cnt, mut min, mut max) = (0, 0, u64::MAX, u64::MIN);
    for time in times.windows(2).map(|x| x[1] - x[0]) {
        sum += time;
        cnt += 1;
        min = min.min(time);
        max = max.max(time);
    }
    let avg = sum / cnt;

    Ok((min, avg, max))
}

#[embassy_executor::task]
async fn btn_task(btn: &'static mut Input<'static>, events: channel::Sender<'static, CriticalSectionRawMutex, Event, 10>) {
    loop {
        btn.wait_for_falling_edge().await;
        info!("Button clicked!");
        events.send(Event::Click).await;
    }
}