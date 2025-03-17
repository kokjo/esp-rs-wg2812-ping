use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::Timer;
use esp_hal::rng::Rng;
use esp_wifi::wifi::{WifiDevice, WifiStaDevice};
use log::info;

pub fn start_net(
    spawner: Spawner,
    mut rng: Rng,
    wifi_sta: WifiDevice<'static, WifiStaDevice>,
) -> Stack<'static> {
    // Setup and spawn Network stack
    let (stack, runner) = embassy_net::new(
        wifi_sta,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<5>, StackResources::<5>::new()),
        (rng.random() as u64) << 32 | rng.random() as u64,
    );
    spawner.must_spawn(net_task(runner));

    stack
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static, WifiStaDevice>>) {
    runner.run().await
}


pub async fn wait_for_link(stack: Stack<'static>) {
    info!("Waiting for link...");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after_millis(500).await;
    }
}

pub async fn wait_for_dhcp(stack: Stack<'static>) {
    info!("Waiting to DHCP...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after_millis(500).await;
    }
}