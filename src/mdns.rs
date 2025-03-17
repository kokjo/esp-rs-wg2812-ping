use core::net::{Ipv4Addr, Ipv6Addr};

use edge_mdns::{
    buf::VecBufAccess,
    domain::base::Ttl,
    host::Host,
    io::{Mdns, DEFAULT_SOCKET},
    HostAnswersMdnsHandler,
};
use edge_nal::{UdpBind as _, UdpSplit as _};
use edge_nal_embassy::{Udp, UdpBuffers};
use embassy_net::Stack;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};

#[embassy_executor::task]
pub async fn mdns_task(stack: Stack<'static>, hostname: &'static str) {
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
        &signal,
    );

    loop {
        mdns.run(HostAnswersMdnsHandler::new(&Host {
            hostname: hostname,
            ipv4: stack.config_v4().unwrap().address.address(),
            ipv6: Ipv6Addr::UNSPECIFIED,
            ttl: Ttl::from_secs(60),
        }))
        .await
        .unwrap();
    }
}
