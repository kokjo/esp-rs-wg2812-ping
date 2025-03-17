use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_hal::{
    peripheral::Peripheral,
    peripherals,
    timer::timg::{TimerGroup, TimerGroupInstance},
};
use esp_wifi::{
    init,
    wifi::{
        self, ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent,
        WifiStaDevice,
    },
    EspWifiController, EspWifiRngSource,
};
use log::info;

pub fn start_wifi_sta(
    spawner: Spawner,
    // timg: impl Peripheral<P = peripherals::TIMG0> +'static,
    timg: impl TimerGroupInstance,
    rng: impl EspWifiRngSource,
    radio_clk: impl Peripheral<P = peripherals::RADIO_CLK> + 'static,
    wifi: impl Peripheral<P = peripherals::WIFI> + 'static,
    ssid: &'static str,
    password: &'static str,
) -> WifiDevice<'static, WifiStaDevice> {
    let timg0 = TimerGroup::new(timg);

    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng, radio_clk).unwrap()
    );

    let (wifi_sta, wifi_ctrl) = wifi::new_with_mode(esp_wifi_ctrl, wifi, WifiStaDevice).unwrap();
    spawner.must_spawn(wifi_ctrl_task(wifi_ctrl, ssid, password));

    wifi_sta
}
#[embassy_executor::task]
async fn wifi_ctrl_task(
    mut wifi_ctrl: WifiController<'static>,
    ssid: &'static str,
    password: &'static str,
) {
    wifi_ctrl
        .set_configuration(&Configuration::Client(ClientConfiguration {
            ssid: ssid.try_into().unwrap(),
            password: password.try_into().unwrap(),
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
